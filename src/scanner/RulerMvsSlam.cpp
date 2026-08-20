#include "JMEngine/RulerMvsSlam.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <chrono>
#include <iostream>

#if defined(JMENGINE_HAS_RULERMVS)
#include <DBoW3/DBoW3.h>
#include <opencv2/opencv.hpp>
#include <rulermvs.hpp>
#include <rulermvs/MarkerExtractor.hpp>
#include <rulermvs/RGBD_MarkerFusion.hpp>
#include <rulermvs/image.hpp>
#include <rulermvs/oneshot.hpp>
#include <rulermvs/rgbdslam.h>
#endif

namespace JMEngine {

#if defined(JMENGINE_HAS_RULERMVS)
namespace {

std::uint32_t packBgr(const cv::Vec3b& bgr) {
    // Keep the pre-refactor RulerMVS contract: OpenCV camera/color data is BGR.
    // Point::rgba is consumed as R,G,B,A bytes by the renderer.
    return std::uint32_t(bgr[2]) |
           (std::uint32_t(bgr[1]) << 8u) |
           (std::uint32_t(bgr[0]) << 16u) |
           0xff000000u;
}

float matrixValue(const cv::Mat& matrix, int row, int column) {
    return matrix.type() == CV_32F ? matrix.at<float>(row, column)
                                   : float(matrix.at<double>(row, column));
}

Pose poseFromCv(const cv::Mat& matrix) {
    Pose pose;
    if (matrix.rows < 3 || matrix.cols < 4)
        return pose;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 4; ++column) {
            pose.matrix[std::size_t(column) * 4u + std::size_t(row)] =
                matrixValue(matrix, row, column);
        }
    }
    return pose;
}

cv::Mat rulerPoseToCv(const rulermvs::Pose& pose) {
    double values[12]{};
    pose.toMatrix(values);
    cv::Mat matrix = cv::Mat::eye(4, 4, CV_64F);
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 4; ++column)
            matrix.at<double>(row, column) = values[row * 4 + column];
    return matrix;
}

class RulerMvsBackend final : public ISlam {
  public:
    ~RulerMvsBackend() override {
        reset();
    }

    bool initialize(const ScanConfig& config, std::string* error) override {
        reset();
        config_ = config;
        if (config.calibrationPath.empty() || config.vocabularyPath.empty()) {
            setError(error, "calibrationPath/vocabularyPath is empty");
            return false;
        }

        oneshot_ = rulermvs::IOneShot::create();
        if (!oneshot_ || oneshot_->init(config.calibrationPath)) {
            setError(error, "RulerMVS OneShot initialization failed");
            return false;
        }

        // Restore the exact pre-refactor pose basis. RGBDFusion reports its pose in
        // the calibration/device basis; the live color-camera view uses colorRT as
        // the base transform. Omitting this step only happened after scanner ownership
        // moved into JMEngine and makes real-camera stacking differ from dataset mode.
        rulermvs::IOneShot::DevicePara devicePara;
        if (rulermvs::IOneShot::loadDeviceFile(config.calibrationPath, devicePara)) {
            baseRt_ = rulerPoseToCv(devicePara.colorRT);
            baseRtInv_ = baseRt_.inv();
        } else {
            baseRt_ = cv::Mat::eye(4, 4, CV_64F);
            baseRtInv_ = baseRt_.clone();
        }
        accepting_.store(true, std::memory_order_release);
        inflight_.store(0, std::memory_order_release);
        {
            std::lock_guard<std::mutex> inputLock(inputMutex_);
            pendingFrame_.reset();
        }

        // These types match the supplied RulerMVS SDK and OpenCV interfaces.
        oneshot_->getColorCamera(camera_);
        rgbSize_ = cv::Size(camera_.width, camera_.height);
        depthSize_ = rgbSize_ / kDepthScale;

        const double scaleX = double(depthSize_.width) / double(rgbSize_.width);
        const double scaleY = double(depthSize_.height) / double(rgbSize_.height);
        depthK_ = cv::Mat::eye(3, 3, CV_64F);
        depthK_.at<double>(0, 0) = camera_.fx * scaleX;
        depthK_.at<double>(0, 2) = camera_.cx * scaleX;
        depthK_.at<double>(1, 1) = camera_.fy * scaleY;
        depthK_.at<double>(1, 2) = camera_.cy * scaleY;

        rulermvs::createUndistorRectifyMap(
            camera_, {}, camera_.nodistor().noskew() / kRectifyScale, mapX_, mapY_);

        vocabulary_.load(config.vocabularyPath);
        database_ = std::make_unique<DBoW3::Database>(vocabulary_, false, 0);
        std::vector<double> maxDistances{kMaxFeatureDistance};
        std::vector<int> maxIterations{kFusionMaxIterations};
        fusion_ = std::make_unique<rgbdslam::RGBDFusion>(
            depthK_, vocabulary_, *database_, depthSize_.width, depthSize_.height,
            maxDistances.data(), maxIterations.data(), int(maxDistances.size()),
                 8, 16, true, true);

        //cv::setNumThreads(1);

        auto& parameters = fusion_->para();
        parameters.is_use_dbow =
            config.registrationMode == ScanRegistrationMode::Texture;
        parameters.colorTheta = 0.0001;
        parameters.minOverlap = 0.3;
        parameters.minMatchNum = 5;
        parameters.maxFeatureNum = 500;
        parameters.minEdgeRatio = 0.9;
        parameters.maxFeatureDist = maxDistances.front();
        parameters.localMode = 2;
        parameters.localMaxIter = 5;
        parameters.globalMode = 2;
        parameters.maxAngle = 0.5236;

        if (config.registrationMode == ScanRegistrationMode::Marker &&
            !initializeMarkerFusion(error)) {
            return false;
        }

        fusion_->setTraceCallBack(
            [this](const rgbdslam::IRGBDResult& result) { handleResult(result); });
        fusion_->start();
        return true;
    }

    bool process(const CameraFrame& frame) override {
        if (!fusion_ || !frame.valid() || !frame.code)
            return false;
        if (frame.width != rgbSize_.width || frame.height != rgbSize_.height)
            return false;

        // Reintroduce the old RGBDFusion admission control INSIDE the backend.
        // JMScanner's outer FrameQueue cannot provide this back-pressure because
        // addFrameInCallBack() returns immediately while decode continues asynchronously.
        // When full, retain only the newest camera pair just like ScanFlowController did.
        {
            std::lock_guard<std::mutex> lock(inputMutex_);
            if (!accepting_.load(std::memory_order_acquire))
                return false;
            const int maxInflight = std::max(1, config_.maxInflightFrames);
            if (inflight_.load(std::memory_order_acquire) >= maxInflight) {
                pendingFrame_ = frame;
                return true;
            }
            inflight_.fetch_add(1, std::memory_order_acq_rel);
        }
        submitFrame(frame);
        return true;
    }

    void submitFrame(const CameraFrame& frame) {
        // CameraFrame already owns its pixel memory. Capturing the shared owners keeps
        // that memory alive for RulerMVS without the second full-resolution rgb/code clone
        // introduced by the refactor.
        const auto rgbOwner = frame.rgb;
        const auto codeOwner = frame.code;
        const int width = frame.width;
        const int height = frame.height;
        const int codeWidth = frame.codeWidth > 0 ? frame.codeWidth : width;
        const int codeHeight = frame.codeHeight > 0 ? frame.codeHeight : height;
        const int frameId = frame.frameId;

        fusion_->addFrameInCallBack(
            [this, rgbOwner, codeOwner, width, height, codeWidth, codeHeight, frameId](
                int& userId, int64_t& timestamp, cv::Mat& depth, cv::Mat& color,
                cv::Mat& mask, cv::Mat& gray) {
                (void)mask;
                userId = frameId;
                timestamp = fusion_->getCurrentTime();
                cv::Mat rgb(height, width, CV_8UC3, rgbOwner->data());
                cv::Mat code(codeHeight, codeWidth, CV_8UC1, codeOwner->data());

                rulermvs::IOneShot::DecodePara decode;
                decode.sigma = 2.0f;
                decode.darkness = 1.0f;
                decode.smoothX = 5;
                decode.smoothY = 5;
                decode.lineThreshold = 0.75f;
                decode.linkInterval = 20;
                decode.minGroup = 200;
                decode.bFuzzyDecode = true;

                const auto perfDecodeStart = std::chrono::steady_clock::now();
                rulermvs::Image8u codeImage;
                rulermvs::convertTo(code, codeImage);
                rulermvs::Imagef depthImage;
                rulermvs::SimpleTriMesh mesh;
                oneshot_->decode(codeImage, mesh, decode);
                rulermvs::rasterDepth(
                    mesh, camera_.nodistor().noskew() / kDepthScale, depthImage);
                depth = depthImage.to<cv::Mat>().clone();

                cv::Mat mapX = mapX_.to<cv::Mat>();
                cv::Mat mapY = mapY_.to<cv::Mat>();
                cv::remap(rgb, color, mapX, mapY, cv::INTER_LINEAR);
                cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
                cv::resize(color, color, depthSize_);

                saveTextureFrame(frameId, color);
                processMarkerFrame(frameId, color, depth);
                const double decodeMs = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - perfDecodeStart).count();
                if (decodeMs > 80.0) {
                    std::cout << "[SCAN STALL][DECODE] frame=" << frameId
                              << " total=" << decodeMs << "ms code=" << codeWidth
                              << "x" << codeHeight << " rgb=" << width << "x" << height
                              << std::endl;
                }
                releaseInputSlot();
            });
    }

    void releaseInputSlot() {
        CameraFrame next;
        bool hasNext = false;
        {
            std::lock_guard<std::mutex> lock(inputMutex_);
            if (inflight_.load(std::memory_order_relaxed) > 0)
                inflight_.fetch_sub(1, std::memory_order_acq_rel);
            if (accepting_.load(std::memory_order_acquire) && pendingFrame_) {
                const int maxInflight = std::max(1, config_.maxInflightFrames);
                if (inflight_.load(std::memory_order_acquire) < maxInflight) {
                    next = std::move(*pendingFrame_);
                    pendingFrame_.reset();
                    inflight_.fetch_add(1, std::memory_order_acq_rel);
                    hasNext = true;
                }
            }
        }
        if (hasNext && fusion_)
            submitFrame(next);
    }

    Pose pose() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return pose_;
    }

    std::shared_ptr<PointCloud> cloud() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return cloud_;
    }

    std::shared_ptr<PointCloud> reconstruct(
        const std::function<void(int)>& progress, std::string* error) override {
        if (markerFusion_)
            return reconstructMarkers(progress);
        if (!fusion_) {
            setError(error, "no scan data available for reconstruction");
            return {};
        }

        fusion_->stop();
        fusion_->optimizePointMap(
            config_.offlineVoxel, config_.offlineIterations,
            [&progress](int value, bool& stop) {
                stop = false;
                if (progress)
                    progress(value);
            });

        std::vector<cv::Point3f> points;
        std::vector<cv::Point3f> normals;
        std::vector<cv::Vec3b> colors;
        fusion_->fusePoints(points, normals, colors);
        auto result = makePointCloud(points, normals, colors);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cloud_ = result;
        }
        return result;
    }

    void reset() override {
        accepting_.store(false, std::memory_order_release);
        {
            std::lock_guard<std::mutex> inputLock(inputMutex_);
            pendingFrame_.reset();
        }
        if (fusion_)
            fusion_->stop();
        if (markerFusion_) {
            markerFusion_->waiting();
            markerFusion_->stop();
            markerFusion_->clear();
        }
        markerFusion_.reset();
        fusion_.reset();
        database_.reset();
        oneshot_ = nullptr;
        {
            std::lock_guard<std::mutex> lock(textureMutex_);
            textureImages_.clear();
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cloud_ = std::make_shared<PointCloud>();
            pose_ = Pose{};
        }
        inflight_.store(0, std::memory_order_release);
    }

    void setUpdateCallback(UpdateCallback callback) override {
        std::lock_guard<std::mutex> lock(mutex_);
        updateCallback_ = std::move(callback);
    }

    void setMarkerCallback(MarkerCallback callback) override {
        std::lock_guard<std::mutex> lock(mutex_);
        markerCallback_ = std::move(callback);
    }

    std::vector<TextureKeyframe> takeTextureKeyframes() override {
        std::unordered_map<int, cv::Mat> poses;
        if (fusion_) {
            fusion_->getResults([&poses](const rgbdslam::IRGBDResult& result) {
                if (result.getFlag() == 0)
                    poses[result.getFrameID()] = result.getRT().inv();
            });
        }

        std::vector<TextureKeyframe> frames;
        std::lock_guard<std::mutex> lock(textureMutex_);
        frames.reserve(textureImages_.size());
        for (auto& item : textureImages_) {
            const auto poseIt = poses.find(item.first);
            if (poseIt == poses.end())
                continue;
            TextureKeyframe frame;
            frame.frameId = item.first;
            frame.width = item.second.cols;
            frame.height = item.second.rows;
            frame.fx = float(depthK_.at<double>(0, 0));
            frame.fy = float(depthK_.at<double>(1, 1));
            frame.cx = float(depthK_.at<double>(0, 2));
            frame.cy = float(depthK_.at<double>(1, 2));
            frame.worldToCamera = poseFromCv(poseIt->second);
            frame.rgb = std::make_shared<std::vector<std::uint8_t>>(
                item.second.data,
                item.second.data + item.second.total() * item.second.elemSize());
            frames.push_back(std::move(frame));
        }
        textureImages_.clear();
        return frames;
    }

  private:
    // Keep these values identical to rulermvsPlugin/rulermvsWrap.cpp.
    static constexpr int kDepthScale = 16;
    static constexpr int kRectifyScale = 4;
    static constexpr double kMaxFeatureDistance = 3.0;
    static constexpr int kFusionMaxIterations = 3;
    static constexpr int kFusionThreadPoolSize = 8;
    static constexpr int kFusionGroupSize = 15;
    static constexpr bool kFusionUpsampling = true;

    static void setError(std::string* error, const std::string& message) {
        if (error)
            *error = message;
    }

    bool initializeMarkerFusion(std::string* error) {
        rulermvs::IRGBD_MarkerFusionPara parameters;
        parameters.use_marker_track = true;
        parameters.input_rectified = true;
        parameters.input_rectifiefcamera = true;
        parameters.width = depthSize_.width;
        parameters.height = depthSize_.height;
        parameters.min_match_num = 4;
        parameters.min_mappoint_num = 4;
        parameters.match_adjacentframe_nums = 2;
        parameters.consecutive = true;

        const auto markerCamera = rulermvs::resizeCamera(
            camera_.nodistor().noskew(),
            rulermvs::Size(depthSize_.width, depthSize_.height));
        markerFusion_ = rulermvs::IRGBD_MarkerFusion::create(
            markerCamera, parameters, false, 2);
        if (!markerFusion_) {
            setError(error, "RulerMVS MarkerFusion initialization failed");
            return false;
        }
        return true;
    }

    void saveTextureFrame(int frameId, const cv::Mat& color) {
        if (config_.registrationMode != ScanRegistrationMode::Texture ||
            frameId % std::max(1, config_.textureKeyframeStride) != 0) {
            return;
        }
        std::lock_guard<std::mutex> lock(textureMutex_);
        const std::size_t hardLimit =
            std::size_t(std::max(1, config_.textureMaxKeyframes)) * 2u;
        if (textureImages_.size() < hardLimit || textureImages_.count(frameId) != 0u)
            textureImages_[frameId] = color.clone();
    }

    void processMarkerFrame(int frameId, const cv::Mat& color,
                            const cv::Mat& depth) {
        if (!markerFusion_)
            return;
        std::lock_guard<std::mutex> lock(markerFusionMutex_);
        rulermvs::RGBImage markerRgb;
        rulermvs::Imagef markerDepth;
        rulermvs::convertTo(color, markerRgb);
        rulermvs::convertTo(depth, markerDepth);
        markerFusion_->proc(
            rulermvs::RGBDImage(markerRgb, markerDepth),
            [this, frameId](const rulermvs::IRGBD_MarkerFusionResult& result) {
                handleMarkerResult(frameId, result);
            });
        markerFusion_->waiting();
    }

    void handleResult(const rgbdslam::IRGBDResult& result) {
        using Clock = std::chrono::steady_clock;
        static auto lastResult = Clock::now();
        const auto resultNow = Clock::now();
        const double resultGapMs = std::chrono::duration<double, std::milli>(resultNow - lastResult).count();
        lastResult = resultNow;
        const auto resultStart = resultNow;
        if (result.getFlag() != 0)
            return;

        std::vector<cv::Point3f> points;
        std::vector<cv::Point3f> normals;
        std::vector<cv::Vec3b> colors;
        const auto toCloudStart = Clock::now();
        result.toCloud(points, normals, colors);
        const double toCloudMs = std::chrono::duration<double, std::milli>(Clock::now() - toCloudStart).count();
        cv::Mat transform = result.getRT();
        if (!baseRtInv_.empty())
            transform = baseRtInv_ * transform;

        // Keep the pre-refactor live-render contract: publish LOCAL points plus the
        // frame pose.  The Qt renderer appends each local frame once and applies pose
        // per draw; historical points are never CPU-transformed/re-uploaded when poses
        // change.  A separate bounded WORLD aggregate is maintained only for liveCloud().
        PointCloud::Container localPreview;
        PointCloud::Container worldPreview;
        // Match the pre-JMScanner live-preview budget. The old pipeline spread the
        // total preview budget across the configured scan instead of uploading up to
        // previewPointsPerFrame on every SLAM result. With the defaults this is about
        // 250 points/frame instead of 12000 points/frame.
        const int budgetPerFrame = std::max(
            250, config_.previewPointLimit / std::max(1, config_.maxFrames));
        const std::size_t target = std::size_t(std::max(
            1, std::min(config_.previewPointsPerFrame, budgetPerFrame)));
        const std::size_t stride = std::max<std::size_t>(1, points.size() / target);
        localPreview.reserve(std::min(points.size(), target));
        worldPreview.reserve(std::min(points.size(), target));
        for (std::size_t index = 0; index < points.size(); index += stride) {
            const auto& source = points[index];
            Point local;
            local.position = {source.x, source.y, source.z};
            if (index < normals.size())
                local.normal = {normals[index].x, normals[index].y, normals[index].z};
            if (index < colors.size())
                local.rgba = packBgr(colors[index]);
            localPreview.push_back(local);

            Point world = local;
            world.position = {
                matrixValue(transform, 0, 0) * source.x + matrixValue(transform, 0, 1) * source.y +
                    matrixValue(transform, 0, 2) * source.z + matrixValue(transform, 0, 3),
                matrixValue(transform, 1, 0) * source.x + matrixValue(transform, 1, 1) * source.y +
                    matrixValue(transform, 1, 2) * source.z + matrixValue(transform, 1, 3),
                matrixValue(transform, 2, 0) * source.x + matrixValue(transform, 2, 1) * source.y +
                    matrixValue(transform, 2, 2) * source.z + matrixValue(transform, 2, 3)};
            worldPreview.push_back(world);
        }

        auto frameChunk = std::make_shared<PointCloud>(std::move(localPreview));
        UpdateCallback callback;
        Pose currentPose;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pose_ = poseFromCv(transform);
            auto& destination = cloud_->points();
            // Never erase from the beginning of a large vector in the realtime callback.
            // That O(N) compaction caused periodic stalls once the preview reached its cap.
            // The per-frame budget above naturally fills the configured total budget; after
            // that, liveCloud() simply remains capped while the renderer still receives the
            // newest local frame chunks through updateCallback_.
            const std::size_t limit =
                std::size_t(std::max(1, config_.previewPointLimit));
            if (destination.size() < limit) {
                const std::size_t remaining = limit - destination.size();
                const std::size_t appendCount = std::min(remaining, worldPreview.size());
                destination.insert(destination.end(), worldPreview.begin(),
                                   worldPreview.begin() + static_cast<std::ptrdiff_t>(appendCount));
            }
            currentPose = pose_;
            callback = updateCallback_;
        }
        const auto callbackStart = Clock::now();
        if (callback)
            callback(result.getFrameID(), currentPose, std::move(frameChunk));
        const double callbackMs = std::chrono::duration<double, std::milli>(Clock::now() - callbackStart).count();
        const double totalMs = std::chrono::duration<double, std::milli>(Clock::now() - resultStart).count();
        if (resultGapMs > 180.0 || toCloudMs > 20.0 || callbackMs > 10.0 || totalMs > 30.0) {
            std::cout << "[SCAN STALL][SLAM RESULT] frame=" << result.getFrameID()
                      << " gap=" << resultGapMs << "ms toCloud=" << toCloudMs
                      << "ms callback=" << callbackMs << "ms total=" << totalMs
                      << "ms rawPts=" << points.size() << std::endl;
        }
    }

    void handleMarkerResult(
        int frameId, const rulermvs::IRGBD_MarkerFusionResult& result) {
        ScanMarkerFrame frame;
        frame.frameId = frameId;
        const auto points = result.getMarker3Dpoints();
        frame.markers.reserve(points.size());
        for (std::size_t index = 0; index < points.size(); ++index) {
            ScanMarker marker;
            marker.localId = int(index);
            marker.hasDepth = true;
            marker.point3d = {float(points[index].x), float(points[index].y),
                              float(points[index].z)};
            frame.markers.push_back(marker);
        }
        MarkerCallback callback;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            callback = markerCallback_;
        }
        if (callback)
            callback(frame);
    }

    std::shared_ptr<PointCloud> reconstructMarkers(
        const std::function<void(int)>& progress) {
        markerFusion_->waiting();
        markerFusion_->optimize_point_clouds([&progress](int value) {
            if (progress)
                progress(value);
        });
        const auto fused = markerFusion_->fuseRGBMap();
        PointCloud::Container output;
        output.reserve(fused.points.size());
        for (std::size_t index = 0; index < fused.points.size(); ++index) {
            Point point;
            point.position = {fused.points[index].x, fused.points[index].y,
                              fused.points[index].z};
            if (index < fused.normals.size()) {
                point.normal = {fused.normals[index].x, fused.normals[index].y,
                                fused.normals[index].z};
            }
            if (index < fused.pixels.size()) {
                const auto& pixel = fused.pixels[index];
                point.rgba = std::uint32_t(pixel.r) |
                             (std::uint32_t(pixel.g) << 8u) |
                             (std::uint32_t(pixel.b) << 16u) | 0xff000000u;
            }
            output.push_back(point);
        }
        auto result = std::make_shared<PointCloud>(std::move(output));
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cloud_ = result;
        }
        return result;
    }

    static std::shared_ptr<PointCloud> makePointCloud(
        const std::vector<cv::Point3f>& points,
        const std::vector<cv::Point3f>& normals,
        const std::vector<cv::Vec3b>& colors) {
        PointCloud::Container output;
        output.reserve(points.size());
        for (std::size_t index = 0; index < points.size(); ++index) {
            Point point;
            point.position = {points[index].x, points[index].y, points[index].z};
            if (index < normals.size())
                point.normal = {normals[index].x, normals[index].y, normals[index].z};
            if (index < colors.size())
                point.rgba = packBgr(colors[index]);
            output.push_back(point);
        }
        return std::make_shared<PointCloud>(std::move(output));
    }

    ScanConfig config_;
    rulermvs::CameraSkewPB camera_;
    rulermvs::Imagef mapX_;
    rulermvs::Imagef mapY_;
    cv::Size rgbSize_;
    cv::Size depthSize_;
    cv::Mat depthK_;
    cv::Mat baseRt_;
    cv::Mat baseRtInv_;
    std::mutex inputMutex_;
    std::atomic<int> inflight_{0};
    std::atomic<bool> accepting_{false};
    std::optional<CameraFrame> pendingFrame_;
    rulermvs::IOneShot::Ptr oneshot_{nullptr};
    DBoW3::Vocabulary vocabulary_;
    std::unique_ptr<DBoW3::Database> database_;
    std::unique_ptr<rgbdslam::RGBDFusion> fusion_;
    rulermvs::IRGBD_MarkerFusion::Ptr markerFusion_;
    mutable std::mutex mutex_;
    std::mutex markerFusionMutex_;
    std::mutex textureMutex_;
    Pose pose_;
    std::shared_ptr<PointCloud> cloud_{std::make_shared<PointCloud>()};
    UpdateCallback updateCallback_;
    MarkerCallback markerCallback_;
    std::unordered_map<int, cv::Mat> textureImages_;
};

} // namespace

std::unique_ptr<ISlam> createRulerMvsSlam() {
    return std::make_unique<RulerMvsBackend>();
}

bool rulerMvsAvailable() noexcept {
    return true;
}

#else
namespace {

class MissingRulerMvsBackend final : public ISlam {
  public:
    bool initialize(const ScanConfig&, std::string* error) override {
        if (error)
            *error = "JMEngine was built without RulerMVS for this ABI";
        return false;
    }

    bool process(const CameraFrame&) override {
        return false;
    }

    Pose pose() const override {
        return {};
    }

    std::shared_ptr<PointCloud> cloud() override {
        return {};
    }
};

} // namespace

std::unique_ptr<ISlam> createRulerMvsSlam() {
    return std::make_unique<MissingRulerMvsBackend>();
}

bool rulerMvsAvailable() noexcept {
    return false;
}

#endif

} // namespace JMEngine
