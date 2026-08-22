#include "JMEngine/RulerMvsSlam.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#if defined(JMENGINE_HAS_RULERMVS)
#include <DBoW3/DBoW3.h>
#include <opencv2/opencv.hpp>
#include <rulermvs.hpp>
#include <rulermvs/MarkerExtractor.hpp>
#include <rulermvs/image.hpp>
#include <rulermvs/oneshot.hpp>
#include <rulermvs/rgbdslam.h>
#endif

namespace JMEngine {

#if defined(JMENGINE_HAS_RULERMVS)
namespace {

std::uint32_t packBgr(const cv::Vec3b& bgr) {
    return std::uint32_t(bgr[2]) |
           (std::uint32_t(bgr[1]) << 8u) |
           (std::uint32_t(bgr[0]) << 16u) |
           0xff000000u;
}

float matrixValue(const cv::Mat& matrix, int row, int column) {
    return matrix.type() == CV_32F ? matrix.at<float>(row, column)
                                   : float(matrix.at<double>(row, column));
}

Vec3f transformPoint(const cv::Mat& rt, const cv::Point3f& p) {
    if (rt.empty() || rt.rows < 3 || rt.cols < 4)
        return {p.x, p.y, p.z};
    return {
        matrixValue(rt, 0, 0) * p.x + matrixValue(rt, 0, 1) * p.y + matrixValue(rt, 0, 2) * p.z + matrixValue(rt, 0, 3),
        matrixValue(rt, 1, 0) * p.x + matrixValue(rt, 1, 1) * p.y + matrixValue(rt, 1, 2) * p.z + matrixValue(rt, 1, 3),
        matrixValue(rt, 2, 0) * p.x + matrixValue(rt, 2, 1) * p.y + matrixValue(rt, 2, 2) * p.z + matrixValue(rt, 2, 3)
    };
}

Vec3f transformNormal(const cv::Mat& rt, const cv::Point3f& n) {
    if (rt.empty() || rt.rows < 3 || rt.cols < 3)
        return {n.x, n.y, n.z};
    Vec3f out{
        matrixValue(rt, 0, 0) * n.x + matrixValue(rt, 0, 1) * n.y + matrixValue(rt, 0, 2) * n.z,
        matrixValue(rt, 1, 0) * n.x + matrixValue(rt, 1, 1) * n.y + matrixValue(rt, 1, 2) * n.z,
        matrixValue(rt, 2, 0) * n.x + matrixValue(rt, 2, 1) * n.y + matrixValue(rt, 2, 2) * n.z
    };
    const float length = std::sqrt(out.x * out.x + out.y * out.y + out.z * out.z);
    if (length > 1.0e-8f) {
        out.x /= length;
        out.y /= length;
        out.z /= length;
    }
    return out;
}

Pose poseFromCv(const cv::Mat& rt) {
    Pose pose;
    if (rt.empty() || rt.rows < 3 || rt.cols < 4)
        return pose;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 4; ++column) {
            pose.matrix[std::size_t(column) * 4u + std::size_t(row)] =
                matrixValue(rt, row, column);
        }
    }
    return pose;
}

cv::Mat rulerPoseToCv(const rulermvs::Pose& pose) {
    double values[12]{};
    pose.toMatrix(values);
    cv::Mat rt = cv::Mat::eye(4, 4, CV_64F);
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 4; ++column)
            rt.at<double>(row, column) = values[row * 4 + column];
    return rt;
}

bool poseChanged(const Pose& a, const Pose& b, float epsilon = 1.0e-5f) {
    for (std::size_t i = 0; i < a.matrix.size(); ++i) {
        if (std::fabs(a.matrix[i] - b.matrix[i]) > epsilon)
            return true;
    }
    return false;
}

class RulerMvsBackend final : public ISlam {
  public:
    ~RulerMvsBackend() override { reset(); }

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

        rulermvs::IOneShot::DevicePara devicePara;
        if (rulermvs::IOneShot::loadDeviceFile(config.calibrationPath, devicePara)) {
            baseRt_ = rulerPoseToCv(devicePara.colorRT);
            baseRtInv_ = baseRt_.inv();
        } else {
            baseRt_ = cv::Mat::eye(4, 4, CV_64F);
            baseRtInv_ = baseRt_.clone();
        }

        oneshot_->getColorCamera(camera_);
        rgbSize_ = {camera_.width, camera_.height};
        depthSize_ = rgbSize_ / kDepthScale;

        const double sx = double(depthSize_.width) / double(rgbSize_.width);
        const double sy = double(depthSize_.height) / double(rgbSize_.height);
        depthK_ = cv::Mat::eye(3, 3, CV_64F);
        depthK_.at<double>(0, 0) = camera_.fx * sx;
        depthK_.at<double>(0, 2) = camera_.cx * sx;
        depthK_.at<double>(1, 1) = camera_.fy * sy;
        depthK_.at<double>(1, 2) = camera_.cy * sy;

        // Exact reference flow: SLAM RGB rectify map is generated at /4.
        rulermvs::createUndistorRectifyMap(
            camera_, {}, camera_.nodistor().noskew() / kRectifyScale, mapX_, mapY_);

        // Final texture mapping keeps its own full-resolution undistort map.
        textureK_ = cv::Mat::eye(3, 3, CV_64F);
        textureK_.at<double>(0, 0) = camera_.fx;
        textureK_.at<double>(0, 2) = camera_.cx;
        textureK_.at<double>(1, 1) = camera_.fy;
        textureK_.at<double>(1, 2) = camera_.cy;
        if (config_.registrationMode == ScanRegistrationMode::Texture &&
            config_.keepTextureInMemory) {
            rulermvs::createUndistorRectifyMap(
                camera_, {}, camera_.nodistor().noskew(), textureMapX_, textureMapY_);
        }

        vocabulary_.load(config.vocabularyPath);
        database_ = std::make_unique<DBoW3::Database>(vocabulary_, false, 0);

        std::vector<double> maxDists{3.0};
        std::vector<int> maxIters{5};
        fusion_ = std::make_unique<rgbdslam::RGBDFusion>(
            depthK_, vocabulary_, *database_, depthSize_.width, depthSize_.height,
            maxDists.data(), maxIters.data(), int(maxDists.size()),
            8, 8, true, true);

        auto& p = fusion_->para();
        p.is_use_dbow = config_.registrationMode == ScanRegistrationMode::Texture;
        p.colorTheta = 0.0001;
        p.minOverlap = 0.3;
        p.minMatchNum = 5;
        p.maxFeatureNum = 500;
        p.minEdgeRatio = 0.9;
        p.maxFeatureDist = maxDists[0];
        p.localMode = 2;
        p.localMaxIter = 5;
        p.globalMode = 2;
        p.maxAngle = 0.5236;

        markerConfigs_ = rulermvs::CicrleConfigs();
        markerConfigs_.scale_ = 1;
        markerConfigs_.multi_thread_ = false;
        markerConfigs_.inputscaled_ = false;
        markerConfigs_.has_distorted = true;

        accepting_.store(true, std::memory_order_release);
        inflight_.store(0, std::memory_order_release);
        submitted_.store(0, std::memory_order_release);
        completed_.store(0, std::memory_order_release);
        converted_.store(0, std::memory_order_release);
        pendingReplaced_.store(0, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lock(inputMutex_);
            pendingFrames_.clear();
        }
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            lastValidFramePose_.release();
            lastValidPoseFrameId_ = -1;
            lastStateFrameId_ = -1;
            pose_ = Pose{};
            cloud_ = std::make_shared<PointCloud>();
            lastPublishedPoseByFrame_.clear();
        }
        {
            std::lock_guard<std::mutex> lock(textureMutex_);
            textureImages_.clear();
        }

        startResultWorkers();
        fusion_->setTraceCallBack(
            [this](const rgbdslam::IRGBDResult& result) { handleFusionResult(result); });
        fusion_->start();
        return true;
    }

    bool process(const CameraFrame& frame) override {
        if (!fusion_ || !frame.valid() || !frame.code)
            return false;
        if (frame.width != rgbSize_.width || frame.height != rgbSize_.height)
            return false;

        std::lock_guard<std::mutex> lock(inputMutex_);
        if (!accepting_.load(std::memory_order_acquire))
            return false;

        const int maxInflight = std::max(1, config_.maxInflightFrames);
        if (inflight_.load(std::memory_order_acquire) >= maxInflight) {
            // Preserve a short sequence of adjacent frames instead of replacing
            // one pending frame with the newest image. Fast turntable motion
            // needs small inter-frame pose changes for robust registration.
            const std::size_t maxPending =
                std::size_t(std::clamp(maxInflight, 3, 8));
            if (pendingFrames_.size() >= maxPending) {
                pendingReplaced_.fetch_add(1, std::memory_order_relaxed);
                std::cout << "[SCAN STALL][SLAM INPUT] frame=" << frame.frameId
                          << " pending=" << pendingFrames_.size()
                          << " inflight=" << inflight_.load(std::memory_order_relaxed)
                          << " dropNewest=1" << std::endl;
                return true;
            }
            pendingFrames_.push_back(frame);
            return true;
        }

        inflight_.fetch_add(1, std::memory_order_acq_rel);
        submitFrame(frame);
        return true;
    }

    Pose pose() const override {
        std::lock_guard<std::mutex> lock(stateMutex_);
        return pose_;
    }

    std::shared_ptr<PointCloud> cloud() override {
        std::lock_guard<std::mutex> lock(stateMutex_);
        return cloud_;
    }

    std::shared_ptr<PointCloud> reconstruct(
        const std::function<void(int)>& progress, std::string* error) override {
        if (!fusion_) {
            setError(error, "no scan data available for reconstruction");
            return {};
        }

        accepting_.store(false, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lock(inputMutex_);
            pendingFrames_.clear();
        }
        stopResultWorkers();
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

        PointCloud::Container out;
        out.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            Point point;
            point.position = {points[i].x, points[i].y, points[i].z};
            if (i < normals.size())
                point.normal = {normals[i].x, normals[i].y, normals[i].z};
            if (i < colors.size())
                point.rgba = packBgr(colors[i]);
            out.push_back(point);
        }

        auto result = std::make_shared<PointCloud>(std::move(out));
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            cloud_ = result;
        }
        return result;
    }

    void reset() override {
        accepting_.store(false, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lock(inputMutex_);
            pendingFrames_.clear();
        }

        stopResultWorkers();
        if (fusion_)
            fusion_->stop();
        fusion_.reset();
        database_.reset();
        oneshot_ = nullptr;

        inflight_.store(0, std::memory_order_release);
        submitted_.store(0, std::memory_order_release);
        completed_.store(0, std::memory_order_release);
        converted_.store(0, std::memory_order_release);
        pendingReplaced_.store(0, std::memory_order_release);

        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            pose_ = Pose{};
            cloud_ = std::make_shared<PointCloud>();
            lastValidFramePose_.release();
            lastValidPoseFrameId_ = -1;
            lastStateFrameId_ = -1;
            lastPublishedPoseByFrame_.clear();
        }
        {
            std::lock_guard<std::mutex> lock(textureMutex_);
            textureImages_.clear();
        }
    }

    void setUpdateCallback(UpdateCallback callback) override {
        std::lock_guard<std::mutex> lock(stateMutex_);
        updateCallback_ = std::move(callback);
    }

    void setMarkerCallback(MarkerCallback callback) override {
        std::lock_guard<std::mutex> lock(stateMutex_);
        markerCallback_ = std::move(callback);
    }

    void setPoseUpdateCallback(PoseUpdateCallback callback) override {
        std::lock_guard<std::mutex> lock(stateMutex_);
        poseUpdateCallback_ = std::move(callback);
    }

    std::vector<TextureKeyframe> takeTextureKeyframes() override {
        if (!config_.keepTextureInMemory)
            return {};

        std::unordered_map<int, cv::Mat> finalWorldFromCamera;
        if (fusion_) {
            std::lock_guard<std::mutex> resultsLock(fusionResultsMutex_);
            fusion_->getResults([this, &finalWorldFromCamera](const rgbdslam::IRGBDResult& result) {
                if (result.getFlag() != 0)
                    return;
                cv::Mat pose = result.getRT();
                if (!baseRtInv_.empty())
                    pose = baseRtInv_ * pose;
                finalWorldFromCamera[result.getFrameID()] = pose.clone();
            });
        }

        std::vector<std::pair<int, cv::Mat>> savedImages;
        {
            std::lock_guard<std::mutex> lock(textureMutex_);
            savedImages.reserve(textureImages_.size());
            for (const auto& item : textureImages_)
                savedImages.push_back(item);
            textureImages_.clear();
        }
        std::sort(savedImages.begin(), savedImages.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        const std::size_t maxFrames =
            std::size_t(std::max(1, config_.textureMaxKeyframes));
        const std::size_t step = savedImages.size() > maxFrames
            ? std::size_t(std::ceil(double(savedImages.size()) / double(maxFrames)))
            : 1u;

        std::vector<TextureKeyframe> frames;
        frames.reserve(std::min(maxFrames, savedImages.size()));
        for (std::size_t i = 0; i < savedImages.size(); i += step) {
            const int frameId = savedImages[i].first;
            const auto poseIt = finalWorldFromCamera.find(frameId);
            if (poseIt == finalWorldFromCamera.end())
                continue;

            TextureKeyframe frame;
            frame.frameId = frameId;
            frame.width = savedImages[i].second.cols;
            frame.height = savedImages[i].second.rows;
            frame.fx = float(textureK_.at<double>(0, 0));
            frame.fy = float(textureK_.at<double>(1, 1));
            frame.cx = float(textureK_.at<double>(0, 2));
            frame.cy = float(textureK_.at<double>(1, 2));
            frame.worldToCamera = poseFromCv(poseIt->second.inv());
            frame.rgb = std::make_shared<std::vector<std::uint8_t>>(
                savedImages[i].second.data,
                savedImages[i].second.data + savedImages[i].second.total() * savedImages[i].second.elemSize());
            frames.push_back(std::move(frame));
            if (frames.size() >= maxFrames)
                break;
        }
        return frames;
    }

  private:
    static constexpr int kDepthScale = 16;
    static constexpr int kRectifyScale = 4;
    static constexpr int kLivePoseRefreshInterval = 30;
    static constexpr std::size_t kMaxRawResultQueue = 4;

    static void setError(std::string* error, const std::string& message) {
        if (error)
            *error = message;
    }

    void submitFrame(const CameraFrame& frame) {
        if (!fusion_ || !accepting_.load(std::memory_order_acquire)) {
            releaseInputSlot();
            return;
        }

        const auto rgbOwner = frame.rgb;
        const auto codeOwner = frame.code;
        const int width = frame.width;
        const int height = frame.height;
        const int codeWidth = frame.codeWidth > 0 ? frame.codeWidth : width;
        const int codeHeight = frame.codeHeight > 0 ? frame.codeHeight : height;
        const int frameIndex = frame.frameId;

        rulermvs::IOneShot::DecodePara decode;
        decode.sigma = 2.0f;
        decode.darkness = 1.0f;
        decode.smoothX = 5;
        decode.smoothY = 5;
        decode.lineThreshold = 0.75f;
        decode.linkInterval = 20;
        decode.minGroup = 200;
        decode.bFuzzyDecode = true;

        submitted_.fetch_add(1, std::memory_order_relaxed);
        fusion_->addFrameInCallBack(
            [this, rgbOwner, codeOwner, width, height, codeWidth, codeHeight,
             frameIndex, decode](int& userID, int64_t& nTime, cv::Mat& depth,
                                 cv::Mat& color, cv::Mat& mask, cv::Mat& gray) mutable {
                (void)mask;
                userID = frameIndex;
                nTime = fusion_->getCurrentTime();

                cv::Mat rgb(height, width, CV_8UC3, rgbOwner->data());
                cv::Mat code(codeHeight, codeWidth, CV_8UC1, codeOwner->data());

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

                if (config_.registrationMode == ScanRegistrationMode::Marker)
                    detectAndPublishMarkers(frameIndex, gray, depth, nTime);

                // Exact reference sequence: remap at /4, gray from /4 image,
                // then resize RGB to the /16 depth size before RGBDFusion consumes it.
                cv::resize(color, color, depthSize_);

                if (config_.registrationMode == ScanRegistrationMode::Texture &&
                    config_.keepTextureInMemory) {
                    const int textureStride = std::max(1, config_.textureKeyframeStride);
                    if ((frameIndex % textureStride) == 0) {
                        cv::Mat textureColor;
                        cv::Mat texMapX = textureMapX_.to<cv::Mat>();
                        cv::Mat texMapY = textureMapY_.to<cv::Mat>();
                        cv::remap(rgb, textureColor, texMapX, texMapY, cv::INTER_LINEAR);
                        std::lock_guard<std::mutex> lock(textureMutex_);
                        const std::size_t hardLimit =
                            std::size_t(std::max(1, config_.textureMaxKeyframes)) * 2u;
                        if (textureImages_.size() < hardLimit ||
                            textureImages_.count(frameIndex) != 0u) {
                            textureImages_[frameIndex] = std::move(textureColor);
                        }
                    }
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

            if (accepting_.load(std::memory_order_acquire) && !pendingFrames_.empty()) {
                const int maxInflight = std::max(1, config_.maxInflightFrames);
                if (inflight_.load(std::memory_order_acquire) < maxInflight) {
                    next = std::move(pendingFrames_.front());
                    pendingFrames_.pop_front();
                    inflight_.fetch_add(1, std::memory_order_acq_rel);
                    hasNext = true;
                }
            }
        }
        if (hasNext)
            submitFrame(next);
    }

    void detectAndPublishMarkers(int frameId, const cv::Mat& grayFull,
                                 const cv::Mat& depthSmall, int64_t timestampUs) {
        if (grayFull.empty())
            return;

        ScanMarkerFrame frame;
        frame.frameId = frameId;
        frame.timestampUs = std::uint64_t(std::max<int64_t>(0, timestampUs));

        std::vector<rulermvs::Corner> corners;
        std::vector<cv::RotatedRect> ellipses;
        bool detected = false;
        {
            std::lock_guard<std::mutex> lock(markerExtractorMutex_);
            cv::Mat markerImage;
            if (grayFull.type() == CV_8UC1 && grayFull.isContinuous())
                markerImage = grayFull.clone();
            else if (grayFull.channels() == 1)
                grayFull.convertTo(markerImage, CV_8U);
            else
                cv::cvtColor(grayFull, markerImage, cv::COLOR_BGR2GRAY);

            detected = markerExtractor_.CircleExtractSimple(
                markerImage, corners, 0, markerConfigs_);
            ellipses = markerExtractor_.ellipses_;
        }

        if (detected) {
            frame.markers.reserve(corners.size());
            const double sx = depthSmall.empty() ? 0.0
                : double(depthSmall.cols) / double(grayFull.cols);
            const double sy = depthSmall.empty() ? 0.0
                : double(depthSmall.rows) / double(grayFull.rows);

            for (std::size_t i = 0; i < corners.size(); ++i) {
                ScanMarker marker;
                marker.localId = int(i);
                marker.imageX = float(corners[i].x_);
                marker.imageY = float(corners[i].y_);

                if (!depthSmall.empty() && depthSmall.type() == CV_32F && sx > 0.0 && sy > 0.0) {
                    const int u = std::clamp(int(std::lround(corners[i].x_ * sx)), 0, depthSmall.cols - 1);
                    const int v = std::clamp(int(std::lround(corners[i].y_ * sy)), 0, depthSmall.rows - 1);
                    std::vector<float> samples;
                    samples.reserve(9);
                    for (int yy = std::max(0, v - 1); yy <= std::min(depthSmall.rows - 1, v + 1); ++yy) {
                        for (int xx = std::max(0, u - 1); xx <= std::min(depthSmall.cols - 1, u + 1); ++xx) {
                            const float d = depthSmall.at<float>(yy, xx);
                            if (std::isfinite(d) && d > 0.0f)
                                samples.push_back(d);
                        }
                    }
                    if (!samples.empty()) {
                        auto mid = samples.begin() + samples.size() / 2;
                        std::nth_element(samples.begin(), mid, samples.end());
                        const float z = *mid;
                        if (z > 0.0f) {
                            const double fx = depthK_.at<double>(0, 0);
                            const double fy = depthK_.at<double>(1, 1);
                            const double cx = depthK_.at<double>(0, 2);
                            const double cy = depthK_.at<double>(1, 2);
                            marker.hasDepth = true;
                            marker.point3d = {
                                float((double(u) - cx) * double(z) / fx),
                                float((double(v) - cy) * double(z) / fy),
                                z};
                        }
                    }
                }
                frame.markers.push_back(marker);
            }
        }

        MarkerCallback callback;
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            callback = markerCallback_;
        }
        if (callback)
            callback(frame);
    }

    struct ResultSignal {
        int frameId{-1};
    };

    struct RawFusionResult {
        int frameId{-1};
        bool trackingOk{false};
        cv::Mat measuredPose;
        std::vector<cv::Point3f> points;
        std::vector<cv::Point3f> normals;
        std::vector<cv::Vec3b> colors;
    };

    void startResultWorkers() {
        stopResultWorkers();
        {
            std::lock_guard<std::mutex> lock(resultSignalMutex_);
            pendingResultSignals_.clear();
            acceptedResultFrameIds_.clear();
        }
        {
            std::lock_guard<std::mutex> lock(resultConvertMutex_);
            pendingRawResults_.clear();
        }
        resultWorkersRunning_.store(true, std::memory_order_release);
        resultConsumerThread_ = std::thread([this] { resultConsumerLoop(); });
        resultConvertThread_ = std::thread([this] { resultConvertLoop(); });
    }

    void stopResultWorkers() {
        resultWorkersRunning_.store(false, std::memory_order_release);
        resultSignalCv_.notify_all();
        resultConvertCv_.notify_all();
        if (resultConsumerThread_.joinable())
            resultConsumerThread_.join();
        if (resultConvertThread_.joinable())
            resultConvertThread_.join();
        {
            std::lock_guard<std::mutex> lock(resultSignalMutex_);
            pendingResultSignals_.clear();
            acceptedResultFrameIds_.clear();
        }
        {
            std::lock_guard<std::mutex> lock(resultConvertMutex_);
            pendingRawResults_.clear();
        }
    }

    // This callback runs on RGBDFusion's worker thread(s). Keep it deliberately tiny.
    // IRGBDResult is received by const reference and its lifetime is not documented as extending
    // past the callback, so never queue the reference/pointer itself. Queue only frame metadata.
    // The consumer snapshots BOTH RT and cloud from the same getResults() result so the pose can
    // never be an older callback-time RT paired with a newer/optimized point cloud.
    void handleFusionResult(const rgbdslam::IRGBDResult& result) {
        completed_.fetch_add(1, std::memory_order_relaxed);
        if (!resultWorkersRunning_.load(std::memory_order_acquire))
            return;

        ResultSignal signal;
        signal.frameId = result.getFrameID();

        {
            std::lock_guard<std::mutex> lock(resultSignalMutex_);
            auto pending = pendingResultSignals_.find(signal.frameId);
            if (pending != pendingResultSignals_.end()) {
                // Same frame callback while it is still pending: one wake-up is enough.
                pending->second = std::move(signal);
            } else if (!acceptedResultFrameIds_.insert(signal.frameId).second) {
                // The frame is already being consumed or has been published. Do not append a
                // duplicate persistent live frame if RGBDFusion reports the same id twice.
                return;
            } else {
                pendingResultSignals_.emplace(signal.frameId, std::move(signal));
            }
        }
        resultSignalCv_.notify_one();
    }

    void resultConsumerLoop() {
        using namespace std::chrono_literals;
        while (true) {
            std::map<int, ResultSignal> wanted;

            // Bound the full-resolution raw cloud queue. If conversion temporarily falls
            // behind, only tiny ResultSignal metadata accumulates; large point buffers do not.
            std::size_t availableSlots = 0;
            {
                std::unique_lock<std::mutex> rawLock(resultConvertMutex_);
                resultConvertCv_.wait(rawLock, [this] {
                    return !resultWorkersRunning_.load(std::memory_order_acquire) ||
                           pendingRawResults_.size() < kMaxRawResultQueue;
                });
                if (!resultWorkersRunning_.load(std::memory_order_acquire))
                    break;
                availableSlots = kMaxRawResultQueue - pendingRawResults_.size();
            }

            {
                std::unique_lock<std::mutex> lock(resultSignalMutex_);
                resultSignalCv_.wait(lock, [this] {
                    return !resultWorkersRunning_.load(std::memory_order_acquire) ||
                           !pendingResultSignals_.empty();
                });
                if (!resultWorkersRunning_.load(std::memory_order_acquire))
                    break;

                // pendingResultSignals_ is ordered by frame id. Consume only as many frames as
                // the conversion queue can accept, keeping callback-side memory bounded.
                const std::size_t takeCount =
                    std::min(availableSlots, pendingResultSignals_.size());
                auto it = pendingResultSignals_.begin();
                for (std::size_t i = 0; i < takeCount && it != pendingResultSignals_.end(); ++i) {
                    auto current = it++;
                    wanted.emplace(current->first, std::move(current->second));
                    pendingResultSignals_.erase(current);
                }
            }

            if (!fusion_ || wanted.empty())
                continue;

            std::vector<RawFusionResult> ready;
            ready.reserve(wanted.size());
            {
                // getResults()/toCloud() are deliberately off RGBDFusion callback threads.
                // IRGBDResult itself cannot safely cross the callback boundary, so this worker
                // snapshots owning vectors; the next worker performs the expensive coordinate
                // conversion / PointCloud construction.
                std::lock_guard<std::mutex> resultsLock(fusionResultsMutex_);
                fusion_->getResults([&wanted, &ready](const rgbdslam::IRGBDResult& result) {
                    auto it = wanted.find(result.getFrameID());
                    if (it == wanted.end())
                        return;

                    RawFusionResult raw;
                    raw.frameId = result.getFrameID();
                    raw.trackingOk = result.getFlag() == 0;
                    // IMPORTANT: RT and point cloud must come from this same getResults()
                    // snapshot. Callback-time RT may already have been changed by live
                    // optimization by the time getResults() exposes the frame.
                    raw.measuredPose = result.getRT().clone();
                    result.toCloud(raw.points, raw.normals, raw.colors);
                    ready.push_back(std::move(raw));
                    wanted.erase(it);
                });
            }

            // Some SDK builds invoke trace slightly before the frame becomes visible from
            // getResults(). Requeue the tiny metadata rather than spinning or dropping it.
            if (!wanted.empty() && resultWorkersRunning_.load(std::memory_order_acquire)) {
                std::this_thread::sleep_for(2ms);
                {
                    std::lock_guard<std::mutex> lock(resultSignalMutex_);
                    for (auto& item : wanted)
                        pendingResultSignals_[item.first] = std::move(item.second);
                }
                resultSignalCv_.notify_one();
            }

            if (ready.empty())
                continue;

            std::sort(ready.begin(), ready.end(), [](const RawFusionResult& a,
                                                     const RawFusionResult& b) {
                return a.frameId < b.frameId;
            });
            {
                std::lock_guard<std::mutex> lock(resultConvertMutex_);
                for (auto& raw : ready)
                    pendingRawResults_.push_back(std::move(raw));
            }
            resultConvertCv_.notify_all();
        }
    }

    void resultConvertLoop() {
        while (true) {
            RawFusionResult raw;
            {
                std::unique_lock<std::mutex> lock(resultConvertMutex_);
                resultConvertCv_.wait(lock, [this] {
                    return !resultWorkersRunning_.load(std::memory_order_acquire) ||
                           !pendingRawResults_.empty();
                });
                if (!resultWorkersRunning_.load(std::memory_order_acquire) &&
                    pendingRawResults_.empty())
                    break;
                raw = std::move(pendingRawResults_.front());
                pendingRawResults_.pop_front();
            }
            resultConvertCv_.notify_all();
            convertAndPublishFusionResult(std::move(raw));
        }
    }

    void convertAndPublishFusionResult(RawFusionResult raw) {
        cv::Mat measuredPose = std::move(raw.measuredPose);
        if (!baseRtInv_.empty())
            measuredPose = baseRtInv_ * measuredPose;

        cv::Mat framePose = measuredPose;
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            if (raw.trackingOk) {
                // A late callback must not replace the recovery pose with an older frame.
                if (raw.frameId >= lastValidPoseFrameId_) {
                    lastValidFramePose_ = measuredPose.clone();
                    lastValidPoseFrameId_ = raw.frameId;
                }
            } else if (!lastValidFramePose_.empty()) {
                framePose = lastValidFramePose_;
            }
        }

        const int budgetPerFrame = std::max(
            250, config_.previewPointLimit / std::max(1, config_.maxFrames));
        const int limit = std::max(
            1, std::min(config_.previewPointsPerFrame, budgetPerFrame));
        const std::size_t stride = std::max<std::size_t>(
            1, raw.points.size() / std::size_t(limit));

        PointCloud::Container localPoints;
        PointCloud::Container worldPoints;
        localPoints.reserve(std::min<std::size_t>(raw.points.size(), std::size_t(limit)));
        worldPoints.reserve(localPoints.capacity());

        for (std::size_t i = 0; i < raw.points.size(); i += stride) {
            Point local;
            local.position = {raw.points[i].x, raw.points[i].y, raw.points[i].z};
            if (i < raw.normals.size())
                local.normal = {raw.normals[i].x, raw.normals[i].y, raw.normals[i].z};
            if (i < raw.colors.size())
                local.rgba = packBgr(raw.colors[i]);
            localPoints.push_back(local);

            Point world = local;
            world.position = transformPoint(framePose, raw.points[i]);
            if (i < raw.normals.size())
                world.normal = transformNormal(framePose, raw.normals[i]);
            worldPoints.push_back(world);

            if (int(localPoints.size()) >= limit)
                break;
        }

        std::shared_ptr<PointCloud> localCloud;
        if (raw.trackingOk && !localPoints.empty())
            localCloud = std::make_shared<PointCloud>(std::move(localPoints));
        auto worldCloud = worldPoints.empty()
            ? std::shared_ptr<PointCloud>{}
            : std::make_shared<PointCloud>(std::move(worldPoints));

        const Pose measured = poseFromCv(measuredPose);
        const Pose displayed = poseFromCv(framePose);

        UpdateCallback update;
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            // pose()/cloud() represent the newest visual state only. Late frames are still
            // published so the accumulated live cloud can fill holes, but they cannot roll
            // the scanner's current state backwards.
            if (raw.frameId >= lastStateFrameId_) {
                lastStateFrameId_ = raw.frameId;
                pose_ = displayed;
                if (raw.trackingOk && localCloud)
                    cloud_ = localCloud;
            }
            if (raw.trackingOk && localCloud)
                lastPublishedPoseByFrame_[raw.frameId] = measured;
            update = updateCallback_;
        }

        if (update && worldCloud) {
            update(raw.frameId, displayed, std::move(localCloud),
                   std::move(worldCloud), raw.trackingOk);
        }

        const int convertedNow = converted_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (config_.registrationMode == ScanRegistrationMode::Texture &&
            config_.maxFrames > 0 && convertedNow % kLivePoseRefreshInterval == 0) {
            refreshLiveOptimizedPreview();
        }
    }

    void refreshLiveOptimizedPreview() {
        if (!fusion_)
            return;

        PoseUpdateCallback callback;
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            callback = poseUpdateCallback_;
        }
        if (!callback)
            return;

        std::vector<FramePoseUpdate> updates;
        {
            std::lock_guard<std::mutex> resultsLock(fusionResultsMutex_);
            fusion_->getResults([this, &updates](const rgbdslam::IRGBDResult& result) {
                if (result.getFlag() != 0)
                    return;

                cv::Mat pose = result.getRT();
                if (!baseRtInv_.empty())
                    pose = baseRtInv_ * pose;
                const Pose next = poseFromCv(pose);

                std::lock_guard<std::mutex> lock(stateMutex_);
                auto it = lastPublishedPoseByFrame_.find(result.getFrameID());
                if (it == lastPublishedPoseByFrame_.end() || !poseChanged(it->second, next))
                    return;
                it->second = next;

                FramePoseUpdate framePose;
                framePose.frameId = result.getFrameID();
                framePose.pose = next;
                updates.push_back(std::move(framePose));
            });
        }

        if (!updates.empty())
            callback(std::move(updates));
    }

    ScanConfig config_;
    rulermvs::CircleMarkerExtractor markerExtractor_;
    rulermvs::CicrleConfigs markerConfigs_;
    std::mutex markerExtractorMutex_;

    rulermvs::CameraSkewPB camera_;
    rulermvs::Imagef mapX_;
    rulermvs::Imagef mapY_;
    rulermvs::Imagef textureMapX_;
    rulermvs::Imagef textureMapY_;
    cv::Size rgbSize_;
    cv::Size depthSize_;
    cv::Mat depthK_;
    cv::Mat textureK_;
    cv::Mat baseRt_;
    cv::Mat baseRtInv_;
    cv::Mat lastValidFramePose_;
    int lastValidPoseFrameId_{-1};
    int lastStateFrameId_{-1};

    rulermvs::IOneShot::Ptr oneshot_{nullptr};
    DBoW3::Vocabulary vocabulary_;
    std::unique_ptr<DBoW3::Database> database_;
    std::unique_ptr<rgbdslam::RGBDFusion> fusion_;

    std::atomic<bool> resultWorkersRunning_{false};
    std::thread resultConsumerThread_;
    std::thread resultConvertThread_;
    std::mutex resultSignalMutex_;
    std::condition_variable resultSignalCv_;
    std::map<int, ResultSignal> pendingResultSignals_;
    std::unordered_set<int> acceptedResultFrameIds_;
    std::mutex resultConvertMutex_;
    std::condition_variable resultConvertCv_;
    std::deque<RawFusionResult> pendingRawResults_;
    std::mutex fusionResultsMutex_;

    std::mutex inputMutex_;
    std::deque<CameraFrame> pendingFrames_;
    std::atomic<int> inflight_{0};
    std::atomic<int> submitted_{0};
    std::atomic<int> completed_{0};
    std::atomic<int> converted_{0};
    std::atomic<unsigned long long> pendingReplaced_{0};
    std::atomic<bool> accepting_{false};

    mutable std::mutex stateMutex_;
    Pose pose_;
    std::shared_ptr<PointCloud> cloud_{std::make_shared<PointCloud>()};
    UpdateCallback updateCallback_;
    MarkerCallback markerCallback_;
    PoseUpdateCallback poseUpdateCallback_;
    std::unordered_map<int, Pose> lastPublishedPoseByFrame_;

    std::mutex textureMutex_;
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

    bool process(const CameraFrame&) override { return false; }
    Pose pose() const override { return {}; }
    std::shared_ptr<PointCloud> cloud() override { return {}; }
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
