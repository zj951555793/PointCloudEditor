#include "JMEngine/RulerMvsSlam.h"
#include "JMEngine/ScanProject.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <iostream>
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

float poseTranslationDelta(const Pose& a, const Pose& b) {
    const float dx = a.matrix[12] - b.matrix[12];
    const float dy = a.matrix[13] - b.matrix[13];
    const float dz = a.matrix[14] - b.matrix[14];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float poseRotationDeltaRadians(const Pose& a, const Pose& b) {
    // Pose matrices are column-major. Compute trace(Ra^T * Rb) without constructing
    // temporary matrices; this is invariant to translation and gives a physical angle.
    float trace = 0.0f;
    for (int c = 0; c < 3; ++c) {
        for (int r = 0; r < 3; ++r) {
            const std::size_t i = std::size_t(c) * 4u + std::size_t(r);
            trace += a.matrix[i] * b.matrix[i];
        }
    }
    const float cosine = std::clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f);
    return std::acos(cosine);
}

bool poseChangedMeaningfully(const Pose& a, const Pose& b,
                             float translationEpsilon,
                             float rotationEpsilonRadians) {
    return poseTranslationDelta(a, b) > translationEpsilon ||
           poseRotationDeltaRadians(a, b) > rotationEpsilonRadians;
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
        std::cout << "[SLAM CALIB] color=" << rgbSize_.width << "x" << rgbSize_.height
                  << " depth=" << depthSize_.width << "x" << depthSize_.height
                  << " losslessDatasetReplay=" << (config_.losslessDatasetReplay ? 1 : 0)
                  << std::endl;

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
            false) {
            rulermvs::createUndistorRectifyMap(
                camera_, {}, camera_.nodistor().noskew(), textureMapX_, textureMapY_);
        }

        if (config_.saveScanProject && !config_.scanProjectPath.empty()) {
            project_ = std::make_unique<ScanProject>();
            project_->open(config_.scanProjectPath);
        }

        vocabulary_.load(config.vocabularyPath);
        database_ = std::make_unique<DBoW3::Database>(vocabulary_, false, 0);

        std::vector<double> maxDists{3.0};
        std::vector<int> maxIters{5};
        fusion_ = std::make_unique<rgbdslam::RGBDFusion>(
            depthK_, vocabulary_, *database_, depthSize_.width, depthSize_.height,
            maxDists.data(), maxIters.data(), int(maxDists.size()),
            8, 8, true, true, true, false);

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
        inputWaitInterrupted_.store(false, std::memory_order_release);
        inflight_.store(0, std::memory_order_release);
        submitted_.store(0, std::memory_order_release);
        completed_.store(0, std::memory_order_release);
        converted_.store(0, std::memory_order_release);
        pendingReplaced_.store(0, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lock(inputMutex_);
            pendingFrames_.clear();
            lastLostModeSubmitTime_.reset();
        }
        trackingLost_.store(false, std::memory_order_release);
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
        {
            std::lock_guard<std::mutex> lock(decodedFrameCloudMutex_);
            decodedFrameClouds_.clear();
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
        if (frame.width != rgbSize_.width || frame.height != rgbSize_.height) {
            const int previous = inputSizeMismatchReports_.fetch_add(1, std::memory_order_relaxed);
            if (previous < 5) {
                std::cerr << "[SLAM INPUT] RGB size mismatch frame=" << frame.frameId
                          << " actual=" << frame.width << "x" << frame.height
                          << " calibration=" << rgbSize_.width << "x" << rgbSize_.height
                          << " code=" << frame.codeWidth << "x" << frame.codeHeight
                          << std::endl;
            }
            return false;
        }

        std::unique_lock<std::mutex> lock(inputMutex_);
        if (!accepting_.load(std::memory_order_acquire))
            return false;

        const int configuredMaxInflight = std::max(1, config_.maxInflightFrames);
        const int maxInflight = config_.losslessDatasetReplay
            ? std::min(2, configuredMaxInflight)
            : configuredMaxInflight;
        if (config_.losslessDatasetReplay) {
            // Keep virtual replay lossless, but do not flood the vendor SDK with
            // six simultaneous OneShot callbacks. IOneShot owns internal scratch
            // state and is not treated as a re-entrant decoder below. Two slots
            // keep decode + RGBDFusion pipelined while preserving frame order.
            using namespace std::chrono_literals;
            while (accepting_.load(std::memory_order_acquire) &&
                   !inputWaitInterrupted_.load(std::memory_order_acquire) &&
                   inflight_.load(std::memory_order_acquire) >= maxInflight) {
                if (inputSlotCondition_.wait_for(lock, 2s) == std::cv_status::timeout) {
                    std::cout << "[VIRTUAL BACKPRESSURE] frame=" << frame.frameId
                              << " inflight=" << inflight_.load(std::memory_order_relaxed)
                              << " submitted=" << submitted_.load(std::memory_order_relaxed)
                              << " completed=" << completed_.load(std::memory_order_relaxed)
                              << " converted=" << converted_.load(std::memory_order_relaxed)
                              << std::endl;
                }
            }
            if (!accepting_.load(std::memory_order_acquire) ||
                inputWaitInterrupted_.load(std::memory_order_acquire))
                return false;

            inflight_.fetch_add(1, std::memory_order_acq_rel);
            lock.unlock();
            submitFrame(frame);
            return true;
        }

        if (shouldThrottleLostTrackingInputLocked()) {
            pendingReplaced_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

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
        lock.unlock();
        submitFrame(frame);
        return true;
    }

    void setLosslessInputReplay(bool enabled) override {
        // JMScanner calls this before the source starts producing frames. A new
        // source also clears a previous stop/restart wake-up token.
        {
            std::lock_guard<std::mutex> lock(inputMutex_);
            config_.losslessDatasetReplay = enabled;
            inputWaitInterrupted_.store(false, std::memory_order_release);
        }
        inputSlotCondition_.notify_all();
    }

    void interruptInputWait() override {
        inputWaitInterrupted_.store(true, std::memory_order_release);
        inputSlotCondition_.notify_all();
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
        inputSlotCondition_.notify_all();
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

        saveOptimizedProjectPoses();

        if (project_) {
            PointCloud projectCloud;
            if (project_->rebuildProjectCloud(projectCloud)) {
                auto result = std::make_shared<PointCloud>(std::move(projectCloud.points()));
                {
                    std::lock_guard<std::mutex> lock(stateMutex_);
                    cloud_ = result;
                }
                if (progress)
                    progress(100);
                return result;
            }
        }

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
        inputSlotCondition_.notify_all();
        {
            std::lock_guard<std::mutex> lock(inputMutex_);
            pendingFrames_.clear();
            lastLostModeSubmitTime_.reset();
        }
        trackingLost_.store(false, std::memory_order_release);

        stopResultWorkers();
        if (fusion_)
            fusion_->stop();
        fusion_.reset();
        database_.reset();
        oneshot_ = nullptr;
        if (project_)
            project_->close();
        project_.reset();

        inflight_.store(0, std::memory_order_release);
        submitted_.store(0, std::memory_order_release);
        completed_.store(0, std::memory_order_release);
        converted_.store(0, std::memory_order_release);
        pendingReplaced_.store(0, std::memory_order_release);
        inputSizeMismatchReports_.store(0, std::memory_order_release);
        decodeEmptyReports_.store(0, std::memory_order_release);

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
        {
            std::lock_guard<std::mutex> lock(decodedFrameCloudMutex_);
            decodedFrameClouds_.clear();
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
    static constexpr int kLivePoseRefreshInterval = 120;
    // Never rewrite the live frontier with loop/local-optimization poses. The newest
    // frames are also the green current-frame/reference shown by the UI; moving them
    // while acquisition is still advancing creates a visible one-frame jump. Once
    // newer frames arrive these protected frames become history and are optimized by
    // the next refresh.
    static constexpr int kLivePoseGuardFrames = 3;
    static constexpr std::size_t kMaxRawResultQueue = 4;

    static void setError(std::string* error, const std::string& message) {
        if (error)
            *error = message;
    }

    std::shared_ptr<PointCloud> buildDecodedFrameCloud(
        const rulermvs::SimpleTriMesh& mesh, const cv::Mat& rectifiedColor,
        std::size_t pointLimit) const {
        PointCloud::Container points;
        if (mesh.points.empty())
            return std::make_shared<PointCloud>(std::move(points));

        const auto colorCamera = camera_.nodistor().noskew() / kRectifyScale;
        const bool canSampleColor = !rectifiedColor.empty() && rectifiedColor.type() == CV_8UC3;
        const std::size_t stride = pointLimit > 0 && mesh.points.size() > pointLimit
            ? std::max<std::size_t>(1u, (mesh.points.size() + pointLimit - 1u) / pointLimit)
            : 1u;
        const std::size_t reserveCount = pointLimit > 0
            ? std::min(pointLimit, (mesh.points.size() + stride - 1u) / stride)
            : mesh.points.size();
        points.reserve(reserveCount);

        for (std::size_t i = 0; i < mesh.points.size(); i += stride) {
            const auto& src = mesh.points[i];
            Point point;
            point.position = {src.x, src.y, src.z};
            if (i < mesh.normals.size()) {
                const auto& normal = mesh.normals[i];
                point.normal = {normal.x, normal.y, normal.z};
            }
            if (canSampleColor && src.z > 1.0e-6f) {
                const double u = colorCamera.fx * src.x / src.z + colorCamera.cx;
                const double v = colorCamera.fy * src.y / src.z + colorCamera.cy;
                const int x = int(std::lround(u));
                const int y = int(std::lround(v));
                if (x >= 0 && y >= 0 && x < rectifiedColor.cols && y < rectifiedColor.rows)
                    point.rgba = packBgr(rectifiedColor.at<cv::Vec3b>(y, x));
            }
            points.push_back(point);
            if (pointLimit > 0 && points.size() >= pointLimit)
                break;
        }
        return std::make_shared<PointCloud>(std::move(points));
    }

    void cacheDecodedFrameCloud(int frameId, const rulermvs::SimpleTriMesh& mesh, const cv::Mat& rectifiedColor) {
        if (!project_ || mesh.points.empty())
            return;

        const auto colorCamera = camera_.nodistor().noskew() / kRectifyScale;
        const bool canSampleColor = !rectifiedColor.empty() && rectifiedColor.type() == CV_8UC3;

        PointCloud::Container points;
        points.reserve(mesh.points.size());
        for (std::size_t i = 0; i < mesh.points.size(); ++i) {
            const auto& src = mesh.points[i];
            Point point;
            point.position = {src.x, src.y, src.z};
            if (i < mesh.normals.size()) {
                const auto& normal = mesh.normals[i];
                point.normal = {normal.x, normal.y, normal.z};
            }
            if (canSampleColor && src.z > 1.0e-6f) {
                const double u = colorCamera.fx * src.x / src.z + colorCamera.cx;
                const double v = colorCamera.fy * src.y / src.z + colorCamera.cy;
                const int x = int(std::lround(u));
                const int y = int(std::lround(v));
                if (x >= 0 && y >= 0 && x < rectifiedColor.cols && y < rectifiedColor.rows)
                    point.rgba = packBgr(rectifiedColor.at<cv::Vec3b>(y, x));
            }
            points.push_back(point);
        }

        auto cloud = std::make_shared<PointCloud>(std::move(points));
        std::lock_guard<std::mutex> lock(decodedFrameCloudMutex_);
        // Do not evict the oldest decoded frame here. RGBDFusion may publish a frame
        // result several input frames later; evicting by a small fixed window can
        // specifically remove frame_0 before its pose arrives and silently fall back
        // to a non-OneShot project frame. Each result consumes its matching entry in
        // takeDecodedFrameCloud(), and reset() clears any unmatched tail frames.
        decodedFrameClouds_[frameId] = std::move(cloud);
    }

    std::shared_ptr<PointCloud> takeDecodedFrameCloud(int frameId) {
        std::lock_guard<std::mutex> lock(decodedFrameCloudMutex_);
        auto it = decodedFrameClouds_.find(frameId);
        if (it == decodedFrameClouds_.end())
            return {};
        auto cloud = std::move(it->second);
        decodedFrameClouds_.erase(it);
        return cloud;
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
                {
                    // The same IOneShot instance is shared by all RGBDFusion
                    // input callbacks. Treat it as non-reentrant: concurrent
                    // decode() calls can stall on some JMC1S recordings even
                    // though JMC1L happens to tolerate them.
                    std::lock_guard<std::mutex> decodeLock(oneShotDecodeMutex_);
                    oneshot_->decode(codeImage, mesh, decode);
                }
                if (frameIndex < 3) {
                    std::cout << "[ONESHOT DECODE] frame=" << frameIndex
                              << " code=" << codeWidth << "x" << codeHeight
                              << " points=" << mesh.points.size() << std::endl;
                } else if (mesh.points.empty()) {
                    const int report = decodeEmptyReports_.fetch_add(1, std::memory_order_relaxed);
                    if (report < 8) {
                        std::cerr << "[ONESHOT DECODE] empty mesh frame=" << frameIndex
                                  << " code=" << codeWidth << "x" << codeHeight
                                  << " (virtual input is not rotated)" << std::endl;
                    }
                }

                // rasterDepth() leaves the output untouched when decode() returns an
                // empty mesh.  That is exactly what happens when the structured-light
                // camera is lost or its code frame is invalid.  Passing the still-empty
                // image to RGBDFusion produces "Wrong size of input depth".  Allocate
                // the contract-sized depth image first; an empty mesh then becomes a
                // zero-depth frame, which RGBDFusion can correctly classify as tracking
                // lost without rejecting the input dimensions.
                depthImage.create(depthSize_.width, depthSize_.height);
                depthImage.memsetZero();
                rulermvs::rasterDepth(
                    mesh, camera_.nodistor().noskew() / kDepthScale, depthImage);
                depth = depthImage.to<cv::Mat>().clone();

                cv::Mat mapX = mapX_.to<cv::Mat>();
                cv::Mat mapY = mapY_.to<cv::Mat>();
                cv::remap(rgb, color, mapX, mapY, cv::INTER_LINEAR);

                // Scan-project frame_x/cloud.ply must be the point cloud produced by
                // IOneShot::decode() for that exact camera frame.  Keep it in the
                // OneShot/color-camera local coordinate system here; RGBDFusion only
                // supplies the frame pose later when its result becomes available.
                cacheDecodedFrameCloud(frameIndex, mesh, color);

                cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);

                if (config_.registrationMode == ScanRegistrationMode::Marker)
                    detectAndPublishMarkers(frameIndex, gray, depth, nTime);

                // Exact reference sequence: remap at /4, gray from /4 image,
                // then resize RGB to the /16 depth size before RGBDFusion consumes it.
                cv::resize(color, color, depthSize_);

                if (config_.registrationMode == ScanRegistrationMode::Texture &&
                    false) {
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

            if (trackingLost_.load(std::memory_order_acquire) &&
                shouldThrottleLostTrackingInputLocked()) {
                pendingFrames_.clear();
                hasNext = false;
            } else if (accepting_.load(std::memory_order_acquire) && !pendingFrames_.empty()) {
                const int maxInflight = std::max(1, config_.maxInflightFrames);
                if (inflight_.load(std::memory_order_acquire) < maxInflight) {
                    next = std::move(pendingFrames_.front());
                    pendingFrames_.pop_front();
                    inflight_.fetch_add(1, std::memory_order_acq_rel);
                    hasNext = true;
                }
            }
        }
        inputSlotCondition_.notify_all();
        if (hasNext)
            submitFrame(next);
    }

    bool shouldThrottleLostTrackingInputLocked() {
        if (!trackingLost_.load(std::memory_order_acquire))
            return false;

        constexpr auto kLostModeMinSubmitGap = std::chrono::milliseconds(200);
        const auto now = std::chrono::steady_clock::now();
        if (!lastLostModeSubmitTime_ ||
            now - *lastLostModeSubmitTime_ >= kLostModeMinSubmitGap) {
            lastLostModeSubmitTime_ = now;
            return false;
        }
        return true;
    }

    void updateLostTrackingInputLimit(bool trackingOk) {
        if (trackingOk) {
            if (trackingLost_.exchange(false, std::memory_order_acq_rel)) {
                std::lock_guard<std::mutex> lock(inputMutex_);
                lastLostModeSubmitTime_.reset();
            }
            return;
        }

        if (!trackingLost_.exchange(true, std::memory_order_acq_rel)) {
            std::lock_guard<std::mutex> lock(inputMutex_);
            pendingFrames_.clear();
            lastLostModeSubmitTime_.reset();
        }
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
                static_cast<const cv::Mat&>(markerImage), corners, 0, markerConfigs_);
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
        bool trackingOk{false};
        cv::Mat trackingPose;
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
        {
            std::lock_guard<std::mutex> lock(poseRefreshMutex_);
            poseRefreshRequested_ = false;
        }
        resultWorkersRunning_.store(true, std::memory_order_release);
        liveResultSnapshotActive_.store(false, std::memory_order_release);
        resultConsumerThread_ = std::thread([this] { resultConsumerLoop(); });
        resultConvertThread_ = std::thread([this] { resultConvertLoop(); });
        poseRefreshThread_ = std::thread([this] { poseRefreshLoop(); });
    }

    void stopResultWorkers() {
        resultWorkersRunning_.store(false, std::memory_order_release);
        resultSignalCv_.notify_all();
        resultConvertCv_.notify_all();
        poseRefreshCv_.notify_all();
        if (resultConsumerThread_.joinable())
            resultConsumerThread_.join();
        if (resultConvertThread_.joinable())
            resultConvertThread_.join();
        if (poseRefreshThread_.joinable())
            poseRefreshThread_.join();
        liveResultSnapshotActive_.store(false, std::memory_order_release);
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
    // IRGBDResult itself must not cross the callback boundary, but the tracking RT is small and
    // MUST be cloned here. getResults() is allowed to expose a later online-optimized RT for the
    // same frame; using that RT as the live/current-frame pose makes the green frame and chase
    // camera oscillate continuously while optimization is active. The consumer later snapshots
    // only the owning cloud data from getResults(); live display always uses this callback-time
    // tracking pose. Historical frames can still receive optimized RT through poseUpdateCallback_.
    void handleFusionResult(const rgbdslam::IRGBDResult& result) {
        completed_.fetch_add(1, std::memory_order_relaxed);
        if (!resultWorkersRunning_.load(std::memory_order_acquire))
            return;

        ResultSignal signal;
        signal.frameId = result.getFrameID();
        signal.trackingOk = result.getFlag() == 0;
        signal.trackingPose = result.getRT().clone();

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

            const auto liveSnapshotStart = std::chrono::steady_clock::now();
            liveResultSnapshotActive_.store(true, std::memory_order_release);
            {
                // Live/result consumption is intentionally independent from online pose refresh.
                // This pass only resolves the requested IRGBDResult clouds. Historical optimized
                // poses are collected by poseRefreshLoop() on a low-priority background path.
                std::lock_guard<std::mutex> resultsLock(fusionResultsMutex_);
                fusion_->getResults([this, &wanted, &ready](const rgbdslam::IRGBDResult& result) {
                    auto it = wanted.find(result.getFrameID());
                    if (it == wanted.end())
                        return;

                    RawFusionResult raw;
                    raw.frameId = result.getFrameID();
                    // Current/live pose is always the trace-callback tracking pose. getResults()
                    // supplies only the owning IRGBDResult cloud for preview/history geometry.
                    raw.trackingOk = it->second.trackingOk;
                    raw.measuredPose = it->second.trackingPose.clone();
                    result.toCloud(raw.points, raw.normals, raw.colors);
                    if (raw.frameId == 0) {
                        std::cout << "[LIVE PREVIEW] frame_0 source=IRGBDResult::toCloud points="
                                  << raw.points.size() << std::endl;
                    }
                    ready.push_back(std::move(raw));
                    wanted.erase(it);
                });
            }
            liveResultSnapshotActive_.store(false, std::memory_order_release);
            poseRefreshCv_.notify_one();

            const double liveSnapshotMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - liveSnapshotStart).count();
            if (liveSnapshotMs > 20.0) {
                std::size_t pendingCount = 0;
                {
                    std::lock_guard<std::mutex> lock(resultSignalMutex_);
                    pendingCount = pendingResultSignals_.size();
                }
                std::cout << "[SLAM LIVE RESULT] getResults+toCloud=" << liveSnapshotMs
                          << "ms batch=" << ready.size()
                          << " pending=" << pendingCount << std::endl;
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

    void requestLivePoseRefresh() {
        if (!config_.liveOptimizationEnabled ||
            config_.registrationMode != ScanRegistrationMode::Texture)
            return;
        {
            std::lock_guard<std::mutex> lock(poseRefreshMutex_);
            // Multiple refresh requests collapse into one. Historical pose refresh is best effort;
            // the live result path always has priority.
            poseRefreshRequested_ = true;
        }
        poseRefreshCv_.notify_one();
    }

    bool liveResultWorkPending() {
        if (liveResultSnapshotActive_.load(std::memory_order_acquire))
            return true;
        {
            std::lock_guard<std::mutex> lock(resultSignalMutex_);
            if (!pendingResultSignals_.empty())
                return true;
        }
        return false;
    }

    void poseRefreshLoop() {
        using namespace std::chrono_literals;

        while (resultWorkersRunning_.load(std::memory_order_acquire)) {
            {
                std::unique_lock<std::mutex> lock(poseRefreshMutex_);
                poseRefreshCv_.wait_for(lock, 25ms, [this] {
                    return !resultWorkersRunning_.load(std::memory_order_acquire) ||
                           poseRefreshRequested_;
                });
                if (!resultWorkersRunning_.load(std::memory_order_acquire))
                    break;
                if (!poseRefreshRequested_)
                    continue;
            }

            // Real-time cloud consumption wins. Do not even attempt a fusion snapshot while
            // current/history geometry is waiting to be resolved or converted.
            if (liveResultWorkPending()) {
                std::this_thread::sleep_for(2ms);
                continue;
            }

            int optimizeThroughFrame = -1;
            {
                std::lock_guard<std::mutex> lock(stateMutex_);
                if (!poseUpdateCallback_) {
                    std::lock_guard<std::mutex> refreshLock(poseRefreshMutex_);
                    poseRefreshRequested_ = false;
                    continue;
                }
                optimizeThroughFrame = lastStateFrameId_ - kLivePoseGuardFrames;
            }
            if (optimizeThroughFrame < 0) {
                std::lock_guard<std::mutex> lock(poseRefreshMutex_);
                poseRefreshRequested_ = false;
                continue;
            }

            // Never wait behind the live consumer. If it owns the fusion snapshot mutex, simply
            // defer this optimization refresh to the next idle gap.
            std::unique_lock<std::mutex> resultsLock(fusionResultsMutex_, std::try_to_lock);
            if (!resultsLock.owns_lock()) {
                std::this_thread::sleep_for(2ms);
                continue;
            }

            // A live request may have arrived between the first idle check and try_lock(). Give
            // the mutex back immediately in that case.
            if (liveResultWorkPending()) {
                resultsLock.unlock();
                std::this_thread::sleep_for(1ms);
                continue;
            }

            // Consume the refresh token before entering the potentially long SDK snapshot. If a
            // newer refresh request arrives while getResults() is running it sets the flag again
            // and is therefore preserved for the next idle gap instead of being cleared here.
            {
                std::lock_guard<std::mutex> lock(poseRefreshMutex_);
                poseRefreshRequested_ = false;
            }

            std::vector<FramePoseUpdate> poseCandidates;
            poseCandidates.reserve(std::size_t(optimizeThroughFrame + 1));
            const auto snapshotStart = std::chrono::steady_clock::now();
            fusion_->getResults([this, &poseCandidates, optimizeThroughFrame]
                                (const rgbdslam::IRGBDResult& result) {
                if (result.getFlag() != 0 || result.getFrameID() > optimizeThroughFrame)
                    return;

                cv::Mat optimizedPose = result.getRT();
                if (!baseRtInv_.empty())
                    optimizedPose = baseRtInv_ * optimizedPose;
                poseCandidates.push_back(
                    {int(result.getFrameID()), poseFromCv(optimizedPose)});
            });
            resultsLock.unlock();

            const double snapshotMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - snapshotStart).count();

            publishLiveOptimizedPoseCandidates(std::move(poseCandidates), snapshotMs);
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
        updateLostTrackingInputLimit(raw.trackingOk);

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

        // previewPointsPerFrame is the actual per-frame preview target. The Qt
        // consumer enforces previewPointLimit on the accumulated history and
        // compacts old samples when necessary, so dividing the total budget by
        // maxFrames here would silently reduce 3000 points/frame to ~1000 for a
        // 20k-frame scan. Keep the per-frame sampling independent from history.
        const int limit = std::max(1, config_.previewPointsPerFrame);
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

        // A scan project stores the original per-frame OneShot cloud plus the pose
        // estimated by RGBDFusion.  Do not replace frame_0 (or any later frame) with
        // result.toCloud(): that cloud is a SLAM-side representation and is not the
        // original OneShot measurement the project is expected to preserve.
        if (project_) {
            auto frameCloud = takeDecodedFrameCloud(raw.frameId);
            if (raw.trackingOk) {
                if (frameCloud && !frameCloud->empty()) {
                    if (!project_->saveFrame(raw.frameId, measured, *frameCloud)) {
                        std::cerr << "[SCAN PROJECT] failed to save OneShot frame="
                                  << raw.frameId << std::endl;
                    } else if (raw.frameId == 0) {
                        std::cout << "[SCAN PROJECT] frame_0 source=OneShot points="
                                  << frameCloud->size() << std::endl;
                    }
                } else {
                    // Never fall back to RGBDFusion::toCloud() here. A missing
                    // OneShot frame is better reported/skipped than silently writing
                    // a project frame with the wrong data source.
                    std::cerr << "[SCAN PROJECT] missing OneShot cloud for frame="
                              << raw.frameId << "; frame not saved" << std::endl;
                }
            }
        }

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
        // Historical optimized-pose refresh stays asynchronous. Live geometry itself is always
        // sourced from IRGBDResult::toCloud(); OneShot remains project-save-only.
        const int poseRefreshInterval = convertedNow < 3000
            ? kLivePoseRefreshInterval
            : (convertedNow < 8000 ? kLivePoseRefreshInterval * 2
                                   : kLivePoseRefreshInterval * 3);
        if (config_.liveOptimizationEnabled &&
            config_.registrationMode == ScanRegistrationMode::Texture &&
            config_.maxFrames > 0 && convertedNow % poseRefreshInterval == 0) {
            requestLivePoseRefresh();
        }
    }

    void publishLiveOptimizedPoseCandidates(std::vector<FramePoseUpdate> candidates,
                                               double backgroundSnapshotMs) {
        if (candidates.empty())
            return;

        PoseUpdateCallback callback;
        constexpr float kPi = 3.14159265358979323846f;
        const float translationEpsilon = std::max(
            1.0e-6f, float(std::fabs(config_.offlineVoxel)) * 0.01f);
        constexpr float rotationEpsilon = 0.03f * kPi / 180.0f;

        std::vector<FramePoseUpdate> updates;
        updates.reserve(candidates.size() / 8u + 8u);
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            callback = poseUpdateCallback_;
            for (const auto& candidate : candidates) {
                auto it = lastPublishedPoseByFrame_.find(candidate.frameId);
                if (it == lastPublishedPoseByFrame_.end() ||
                    !poseChangedMeaningfully(it->second, candidate.pose,
                                              translationEpsilon, rotationEpsilon))
                    continue;
                it->second = candidate.pose;
                updates.push_back(candidate);
            }
        }

        const std::size_t updateCount = updates.size();
        if (callback && !updates.empty())
            callback(std::move(updates));

        std::cout << "[SLAM LIVE OPT] background getResults snapshot=" << backgroundSnapshotMs
                  << "ms updates=" << updateCount << std::endl;
    }

    void saveOptimizedProjectPoses() {
        if (!fusion_ || !project_)
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
                const Pose optimizedPose = poseFromCv(pose);
                project_->savePose(result.getFrameID(), optimizedPose);

                FramePoseUpdate update;
                update.frameId = result.getFrameID();
                update.pose = optimizedPose;
                updates.push_back(std::move(update));
            });
        }

        if (!updates.empty()) {
            PoseUpdateCallback callback;
            {
                std::lock_guard<std::mutex> lock(stateMutex_);
                for (const auto& update : updates)
                    lastPublishedPoseByFrame_[update.frameId] = update.pose;
                callback = poseUpdateCallback_;
            }
            if (callback)
                callback(std::move(updates));
        }
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
    std::mutex oneShotDecodeMutex_;
    DBoW3::Vocabulary vocabulary_;
    std::unique_ptr<DBoW3::Database> database_;
    std::unique_ptr<rgbdslam::RGBDFusion> fusion_;
    std::unique_ptr<ScanProject> project_;

    std::atomic<bool> resultWorkersRunning_{false};
    std::thread resultConsumerThread_;
    std::thread resultConvertThread_;
    std::thread poseRefreshThread_;
    std::mutex poseRefreshMutex_;
    std::condition_variable poseRefreshCv_;
    bool poseRefreshRequested_{false};
    std::atomic<bool> liveResultSnapshotActive_{false};
    std::mutex resultSignalMutex_;
    std::condition_variable resultSignalCv_;
    std::map<int, ResultSignal> pendingResultSignals_;
    std::unordered_set<int> acceptedResultFrameIds_;
    std::mutex resultConvertMutex_;
    std::condition_variable resultConvertCv_;
    std::deque<RawFusionResult> pendingRawResults_;
    std::mutex fusionResultsMutex_;

    std::mutex inputMutex_;
    std::condition_variable inputSlotCondition_;
    std::deque<CameraFrame> pendingFrames_;
    std::optional<std::chrono::steady_clock::time_point> lastLostModeSubmitTime_;
    std::atomic<int> inflight_{0};
    std::atomic<int> submitted_{0};
    std::atomic<int> completed_{0};
    std::atomic<int> converted_{0};
    std::atomic<unsigned long long> pendingReplaced_{0};
    std::atomic<int> inputSizeMismatchReports_{0};
    std::atomic<int> decodeEmptyReports_{0};
    std::atomic<bool> inputWaitInterrupted_{false};
    std::atomic<bool> accepting_{false};
    std::atomic<bool> trackingLost_{false};

    mutable std::mutex stateMutex_;
    Pose pose_;
    std::shared_ptr<PointCloud> cloud_{std::make_shared<PointCloud>()};
    UpdateCallback updateCallback_;
    MarkerCallback markerCallback_;
    PoseUpdateCallback poseUpdateCallback_;
    std::unordered_map<int, Pose> lastPublishedPoseByFrame_;

    std::mutex textureMutex_;
    std::unordered_map<int, cv::Mat> textureImages_;
    std::mutex decodedFrameCloudMutex_;
    // Full OneShot clouds are retained only for scan-project persistence.
    std::map<int, std::shared_ptr<PointCloud>> decodedFrameClouds_;
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
