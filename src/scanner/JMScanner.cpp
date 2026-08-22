#include "JMEngine/JMScanner.h"

#include "JMEngine/FrameQueue.h"
#include "JMEngine/RulerMvsSlam.h"

#include <algorithm>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>
#include <utility>

namespace JMEngine {

class JMScanner::Impl {
  public:
    explicit Impl(std::unique_ptr<ISlam> scanBackend)
        : backend(std::move(scanBackend)), queue(1) {}

    ~Impl() {
        stopWorker();
    }

    void setState(ScanState newState) {
        StateCallback callback;
        {
            std::lock_guard<std::mutex> lock(mutex);
            state = newState;
            statistics.state = newState;
            callback = stateCallback;
        }
        if (callback)
            callback(newState);
    }

    void stopWorker() {
        if (source)
            source->stop();
        running.store(false);
        queue.close();
        if (worker.joinable())
            worker.join();
        queue.clear();
    }

    void publish(int frameId, const Pose& framePose,
                 std::shared_ptr<PointCloud> frameCloud,
                 std::shared_ptr<PointCloud> statusCloud, bool trackingOk) {
        const auto aggregate = backend ? backend->cloud() : frameCloud;
        FrameCallback callback;
        {
            std::lock_guard<std::mutex> lock(mutex);
            liveCloud = aggregate;
            pose = framePose;
            statistics.livePoints = aggregate ? aggregate->size() : 0;
            callback = frameCallback;
        }
        if (callback)
            callback(frameId, framePose, std::move(frameCloud),
                     std::move(statusCloud), trackingOk);
    }

    void publishMarkers(const ScanMarkerFrame& frame) {
        MarkerCallback callback;
        {
            std::lock_guard<std::mutex> lock(mutex);
            callback = markerCallback;
        }
        if (callback)
            callback(frame);
    }

    void publishPoseUpdates(std::vector<FramePoseUpdate> updates) {
        if (updates.empty())
            return;
        PoseUpdateCallback callback;
        {
            std::lock_guard<std::mutex> lock(mutex);
            callback = poseUpdateCallback;
        }
        if (callback)
            callback(std::move(updates));
    }

    void run() {
        using Clock = std::chrono::steady_clock;
        CameraFrame frame;
        auto lastPop = Clock::now();
        while (running.load() && queue.pop(frame)) {
            if (!running.load())
                break;
            const auto popNow = Clock::now();
            const double popGapMs = std::chrono::duration<double, std::milli>(popNow - lastPop).count();
            lastPop = popNow;
            const auto t0 = Clock::now();
            const bool accepted = backend->process(frame);
            const double processMs = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            if (popGapMs > 180.0 || processMs > 20.0) {
                std::cout << "[SCAN STALL][QUEUE] frame=" << frame.frameId
                          << " popGap=" << popGapMs << "ms process=" << processMs
                          << "ms accepted=" << (accepted ? 1 : 0) << std::endl;
            }
            std::lock_guard<std::mutex> lock(mutex);
            if (accepted)
                ++statistics.processedFrames;
            else
                ++statistics.rejectedFrames;
        }
    }

    mutable std::mutex mutex;
    std::unique_ptr<ISlam> backend;
    FrameQueue queue;
    std::thread worker;
    std::atomic<bool> running{false};
    ScanConfig config;
    ScanState state{ScanState::Idle};
    ScanStatistics statistics;
    Pose pose;
    std::shared_ptr<PointCloud> liveCloud;
    std::shared_ptr<PointCloud> resultCloud;
    std::string error;
    StateCallback stateCallback;
    FrameCallback frameCallback;
    MessageCallback messageCallback;
    ProgressCallback progressCallback;
    MarkerCallback markerCallback;
    PoseUpdateCallback poseUpdateCallback;
    std::function<void(std::shared_ptr<std::vector<std::uint8_t>>, int, int)>
        previewCallback;
    std::unique_ptr<ICameraSource> source;
};

JMScanner::JMScanner()
    : JMScanner(createRulerMvsSlam()) {}

JMScanner::JMScanner(std::unique_ptr<ISlam> backend)
    : impl_(std::make_unique<Impl>(
          backend ? std::move(backend) : createRulerMvsSlam())) {}

JMScanner::~JMScanner() = default;

bool JMScanner::initialize(const ScanConfig& config) {
    if (state() == ScanState::Scanning || state() == ScanState::Reconstructing)
        return false;

    impl_->setState(ScanState::Initializing);
    impl_->backend->setUpdateCallback(
        [implementation = impl_.get()](int frameId, const Pose& pose,
                                       std::shared_ptr<PointCloud> cloud,
                                       std::shared_ptr<PointCloud> statusCloud,
                                       bool trackingOk) {
            implementation->publish(frameId, pose, std::move(cloud),
                                    std::move(statusCloud), trackingOk);
        });
    impl_->backend->setMarkerCallback(
        [implementation = impl_.get()](const ScanMarkerFrame& frame) {
            implementation->publishMarkers(frame);
        });
    impl_->backend->setPoseUpdateCallback(
        [implementation = impl_.get()](std::vector<FramePoseUpdate> updates) {
            implementation->publishPoseUpdates(std::move(updates));
        });

    std::string error;
    if (!impl_->backend || !impl_->backend->initialize(config, &error)) {
        {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->error = error.empty() ? "RulerMVS initialization failed" : error;
        }
        impl_->setState(ScanState::Error);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->config = config;
        impl_->statistics = {};
        impl_->error.clear();
        impl_->liveCloud.reset();
        impl_->resultCloud.reset();
    }
    // Preserve a short run of consecutive camera pairs before the SLAM backend.
    // Fast turntable motion needs small pose deltas between adjacent frames; a
    // one-slot latest-only mailbox can otherwise turn 100,101,102... into
    // 100,104,108... when the backend is briefly busy. Keep the queue bounded
    // so latency and memory cannot grow without limit.
    impl_->queue.setCapacity(std::size_t(std::clamp(config.maxInflightFrames, 3, 8)));
    impl_->queue.reopen();
    impl_->setState(ScanState::Idle);
    return true;
}

bool JMScanner::start() {
    const auto currentState = state();
    if (currentState != ScanState::Idle &&
        currentState != ScanState::ReadyForReconstruction) {
        return false;
    }

    impl_->stopWorker();
    impl_->queue.reopen();
    impl_->running.store(true);
    impl_->worker = std::thread([implementation = impl_.get()] {
        implementation->run();
    });
    impl_->setState(ScanState::Scanning);
    return true;
}

bool JMScanner::startCameras(const DualCameraConfig& config) {
    if (state() != ScanState::Scanning && !start())
        return false;

    impl_->source = createPlatformCameraSource();
    impl_->source->setPreviewCallback(impl_->previewCallback);
    std::string error;
    const bool started = impl_->source->start(
        config,
        [this](CameraFrame frame) { submit(std::move(frame)); },
        [this](const std::string& message) {
            MessageCallback callback;
            {
                std::lock_guard<std::mutex> lock(impl_->mutex);
                impl_->error = message;
                callback = impl_->messageCallback;
            }
            if (callback)
                callback(message);
        },
        &error);

    if (!started) {
        impl_->stopWorker();
        {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->error = error;
        }
        impl_->setState(ScanState::Error);
    }
    return started;
}

bool JMScanner::startDataset(const std::string& directory) {
    if (state() != ScanState::Scanning && !start())
        return false;

    impl_->source = createDatasetCameraSource(directory);
    impl_->source->setPreviewCallback(impl_->previewCallback);
    std::string error;
    const bool started = impl_->source->start(
        {},
        [this](CameraFrame frame) { submit(std::move(frame)); },
        [this](const std::string& message) {
            MessageCallback callback;
            {
                std::lock_guard<std::mutex> lock(impl_->mutex);
                impl_->error = message;
                callback = impl_->messageCallback;
            }
            if (callback)
                callback(message);
        },
        &error);

    if (!started) {
        impl_->stopWorker();
        {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->error = error;
        }
        impl_->setState(ScanState::Error);
    }
    return started;
}

void JMScanner::setCameraExposure(int role, double value) {
    if (impl_->source)
        impl_->source->setExposure(role, value);
}

void JMScanner::setCameraBacklight(int role, double value) {
    if (impl_->source)
        impl_->source->setBacklight(role, value);
}

void JMScanner::setCameraPreviewCallback(
    std::function<void(std::shared_ptr<std::vector<std::uint8_t>>, int, int)>
        callback) {
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->previewCallback = callback;
    }
    if (impl_->source)
        impl_->source->setPreviewCallback(std::move(callback));
}

bool JMScanner::submit(CameraFrame frame) {
    if (state() != ScanState::Scanning || !frame.valid()) {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        ++impl_->statistics.rejectedFrames;
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (impl_->statistics.submittedFrames >=
            std::uint64_t(std::max(1, impl_->config.maxFrames))) {
            ++impl_->statistics.rejectedFrames;
            return false;
        }
        ++impl_->statistics.submittedFrames;
    }

    static auto lastSubmit = std::chrono::steady_clock::now();
    const auto now = std::chrono::steady_clock::now();
    const double submitGapMs = std::chrono::duration<double, std::milli>(now - lastSubmit).count();
    lastSubmit = now;
    const int submitFrameId = frame.frameId;
    // Do not evict an older queued frame: preserving adjacency is more important
    // for fast rigid turntable motion than always displaying the newest frame.
    const bool queued = impl_->queue.pushSequential(std::move(frame));
    if (submitGapMs > 180.0 || !queued) {
        std::cout << "[SCAN STALL][SOURCE] frame=" << submitFrameId
                  << " gap=" << submitGapMs << "ms queueDropNewest=" << (!queued ? 1 : 0)
                  << std::endl;
    }
    if (!queued) {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        ++impl_->statistics.replacedFrames;
    }
    return true;
}

void JMScanner::stop() {
    if (state() != ScanState::Scanning)
        return;
    impl_->setState(ScanState::Stopping);
    impl_->stopWorker();
    impl_->setState(ScanState::ReadyForReconstruction);
}

bool JMScanner::reconstruct() {
    if (state() != ScanState::ReadyForReconstruction)
        return false;
    impl_->setState(ScanState::Reconstructing);

    std::string error;
    auto cloud = impl_->backend->reconstruct(
        [this](int value) {
            ProgressCallback callback;
            {
                std::lock_guard<std::mutex> lock(impl_->mutex);
                callback = impl_->progressCallback;
            }
            if (callback)
                callback(value);
        },
        &error);

    if (!cloud) {
        {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->error = error.empty() ? "offline reconstruction failed" : error;
        }
        impl_->setState(ScanState::Error);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->resultCloud = cloud;
        impl_->liveCloud = cloud;
        impl_->statistics.livePoints = cloud->size();
    }
    impl_->setState(ScanState::ReadyForReconstruction);
    return true;
}

void JMScanner::reset() {
    impl_->stopWorker();
    if (impl_->backend)
        impl_->backend->reset();
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->statistics = {};
        impl_->liveCloud.reset();
        impl_->resultCloud.reset();
        impl_->error.clear();
    }
    impl_->setState(ScanState::Idle);
}

ScanState JMScanner::state() const noexcept {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->state;
}

Pose JMScanner::pose() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->pose;
}

std::shared_ptr<PointCloud> JMScanner::liveCloud() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->liveCloud;
}

std::shared_ptr<PointCloud> JMScanner::resultCloud() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->resultCloud;
}

ScanStatistics JMScanner::statistics() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->statistics;
}

std::string JMScanner::lastError() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->error;
}

std::vector<TextureKeyframe> JMScanner::takeTextureKeyframes() {
    return impl_->backend ? impl_->backend->takeTextureKeyframes()
                          : std::vector<TextureKeyframe>{};
}

void JMScanner::setStateCallback(StateCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stateCallback = std::move(callback);
}

void JMScanner::setFrameCallback(FrameCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->frameCallback = std::move(callback);
}

void JMScanner::setMessageCallback(MessageCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->messageCallback = std::move(callback);
}

void JMScanner::setProgressCallback(ProgressCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->progressCallback = std::move(callback);
}

void JMScanner::setMarkerCallback(MarkerCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->markerCallback = std::move(callback);
}

void JMScanner::setPoseUpdateCallback(PoseUpdateCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->poseUpdateCallback = std::move(callback);
}

} // namespace JMEngine
