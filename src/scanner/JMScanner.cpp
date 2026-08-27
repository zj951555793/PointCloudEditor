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

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace JMEngine {

#if defined(_WIN32)
namespace {
class ScannerMutex {
  public:
    ScannerMutex() { InitializeCriticalSection(&section_); }
    ~ScannerMutex() { DeleteCriticalSection(&section_); }

    ScannerMutex(const ScannerMutex&) = delete;
    ScannerMutex& operator=(const ScannerMutex&) = delete;

    void lock() { EnterCriticalSection(&section_); }
    void unlock() { LeaveCriticalSection(&section_); }

  private:
    CRITICAL_SECTION section_;
};
} // namespace
#else
using ScannerMutex = std::mutex;
#endif

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
            std::lock_guard<ScannerMutex> lock(mutex);
            state = newState;
            statistics.state = newState;
            callback = stateCallback;
        }
        if (callback)
            callback(newState);
    }

    void stopWorker() {
        // Close the scanner queue before joining a lossless DatasetCamera.  Its
        // producer may be blocked in pushBlocking() waiting for capacity; closing
        // the queue wakes it immediately and prevents stop() deadlock.
        running.store(false);
        queue.close();
        if (backend)
            backend->interruptInputWait();
        if (source)
            source->stop();
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
            std::lock_guard<ScannerMutex> lock(mutex);
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
            std::lock_guard<ScannerMutex> lock(mutex);
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
            std::lock_guard<ScannerMutex> lock(mutex);
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
            MessageCallback rejectCallback;
            bool reportDatasetReject = false;
            {
                std::lock_guard<ScannerMutex> lock(mutex);
                if (accepted) {
                    ++statistics.processedFrames;
                } else {
                    ++statistics.rejectedFrames;
                    if (config.losslessDatasetReplay &&
                        !datasetRejectReported.exchange(true)) {
                        reportDatasetReject = true;
                        rejectCallback = messageCallback;
                    }
                }
            }
            if (reportDatasetReject && rejectCallback) {
                rejectCallback(
                    "Virtual frame rejected by SLAM. Check calib.txt RGB size and "
                    "[SLAM INPUT]/[ONESHOT DECODE] diagnostics.");
            }
        }
    }

    bool ensureBackend(std::string* backendError) {
        if (backend)
            return true;
        backend = createRulerMvsSlam();
        if (backend)
            return true;
        if (backendError)
            *backendError = "RulerMVS backend creation failed";
        return false;
    }

    mutable ScannerMutex mutex;
    std::unique_ptr<ISlam> backend;
    FrameQueue queue;
    std::thread worker;
    std::atomic<bool> running{false};
    std::atomic<bool> datasetRejectReported{false};
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
    : impl_(std::make_unique<Impl>(nullptr)) {}

JMScanner::JMScanner(std::unique_ptr<ISlam> backend)
    : impl_(std::make_unique<Impl>(std::move(backend))) {}

JMScanner::~JMScanner() = default;

bool JMScanner::initialize(const ScanConfig& config) {
    if (state() == ScanState::Scanning || state() == ScanState::Reconstructing)
        return false;

    impl_->setState(ScanState::Initializing);
    std::string error;
    if (!impl_->ensureBackend(&error)) {
        {
            std::lock_guard<ScannerMutex> lock(impl_->mutex);
            impl_->error = error.empty() ? "RulerMVS backend creation failed" : error;
        }
        impl_->setState(ScanState::Error);
        return false;
    }

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

    if (!impl_->backend || !impl_->backend->initialize(config, &error)) {
        {
            std::lock_guard<ScannerMutex> lock(impl_->mutex);
            impl_->error = error.empty() ? "RulerMVS initialization failed" : error;
        }
        impl_->setState(ScanState::Error);
        return false;
    }

    {
        std::lock_guard<ScannerMutex> lock(impl_->mutex);
        impl_->config = config;
        impl_->statistics = {};
        impl_->datasetRejectReported.store(false);
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

    std::string error;
    if (!impl_->ensureBackend(&error)) {
        {
            std::lock_guard<ScannerMutex> lock(impl_->mutex);
            impl_->error = error.empty() ? "RulerMVS backend creation failed" : error;
        }
        impl_->setState(ScanState::Error);
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

    impl_->config.losslessDatasetReplay = false;
    if (impl_->backend)
        impl_->backend->setLosslessInputReplay(false);
    impl_->source = createPlatformCameraSource();
    impl_->source->setPreviewCallback(impl_->previewCallback);
    std::string error;
    const bool started = impl_->source->start(
        config,
        [this](CameraFrame frame) { submit(std::move(frame)); },
        [this](const std::string& message) {
            MessageCallback callback;
            {
                std::lock_guard<ScannerMutex> lock(impl_->mutex);
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
            std::lock_guard<ScannerMutex> lock(impl_->mutex);
            impl_->error = error;
        }
        impl_->setState(ScanState::Error);
    }
    return started;
}

bool JMScanner::startDataset(const std::string& directory) {
    if (state() != ScanState::Scanning && !start())
        return false;

    // Enforce lossless replay at the scanner API boundary as well as in the Qt
    // config so non-Qt callers cannot accidentally use the live-camera drop
    // policy for a dataset.
    impl_->config.losslessDatasetReplay = true;
    if (impl_->backend)
        impl_->backend->setLosslessInputReplay(true);
    impl_->source = createDatasetCameraSource(directory);
    impl_->source->setPreviewCallback(impl_->previewCallback);
    std::string error;
    const bool started = impl_->source->start(
        {},
        [this](CameraFrame frame) { submit(std::move(frame)); },
        [this](const std::string& message) {
            MessageCallback callback;
            {
                std::lock_guard<ScannerMutex> lock(impl_->mutex);
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
            std::lock_guard<ScannerMutex> lock(impl_->mutex);
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
        std::lock_guard<ScannerMutex> lock(impl_->mutex);
        impl_->previewCallback = callback;
    }
    if (impl_->source)
        impl_->source->setPreviewCallback(std::move(callback));
}

bool JMScanner::submit(CameraFrame frame) {
    if (state() != ScanState::Scanning || !frame.valid()) {
        std::lock_guard<ScannerMutex> lock(impl_->mutex);
        ++impl_->statistics.rejectedFrames;
        return false;
    }

    {
        std::lock_guard<ScannerMutex> lock(impl_->mutex);
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
    // Physical cameras keep bounded real-time latency. Virtual datasets are
    // different: every recorded frame is valuable and must remain consecutive,
    // so apply producer back-pressure instead of dropping the newest frame.
    const bool losslessReplay = impl_->config.losslessDatasetReplay;
    const bool queued = losslessReplay
        ? impl_->queue.pushBlocking(std::move(frame))
        : impl_->queue.pushSequential(std::move(frame));
    if (submitGapMs > 180.0 || !queued) {
        std::cout << "[SCAN STALL][SOURCE] frame=" << submitFrameId
                  << " gap=" << submitGapMs
                  << "ms queueDropNewest=" << (!queued ? 1 : 0)
                  << " losslessReplay=" << (losslessReplay ? 1 : 0)
                  << std::endl;
    }
    if (!queued) {
        std::lock_guard<ScannerMutex> lock(impl_->mutex);
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
                std::lock_guard<ScannerMutex> lock(impl_->mutex);
                callback = impl_->progressCallback;
            }
            if (callback)
                callback(value);
        },
        &error);

    if (!cloud) {
        {
            std::lock_guard<ScannerMutex> lock(impl_->mutex);
            impl_->error = error.empty() ? "offline reconstruction failed" : error;
        }
        impl_->setState(ScanState::Error);
        return false;
    }

    {
        std::lock_guard<ScannerMutex> lock(impl_->mutex);
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
        std::lock_guard<ScannerMutex> lock(impl_->mutex);
        impl_->statistics = {};
        impl_->liveCloud.reset();
        impl_->resultCloud.reset();
        impl_->error.clear();
    }
    impl_->setState(ScanState::Idle);
}

ScanState JMScanner::state() const noexcept {
    std::lock_guard<ScannerMutex> lock(impl_->mutex);
    return impl_->state;
}

Pose JMScanner::pose() const {
    std::lock_guard<ScannerMutex> lock(impl_->mutex);
    return impl_->pose;
}

std::shared_ptr<PointCloud> JMScanner::liveCloud() const {
    std::lock_guard<ScannerMutex> lock(impl_->mutex);
    return impl_->liveCloud;
}

std::shared_ptr<PointCloud> JMScanner::resultCloud() const {
    std::lock_guard<ScannerMutex> lock(impl_->mutex);
    return impl_->resultCloud;
}

ScanStatistics JMScanner::statistics() const {
    std::lock_guard<ScannerMutex> lock(impl_->mutex);
    return impl_->statistics;
}

std::string JMScanner::lastError() const {
    std::lock_guard<ScannerMutex> lock(impl_->mutex);
    return impl_->error;
}

std::vector<TextureKeyframe> JMScanner::takeTextureKeyframes() {
    return impl_->backend ? impl_->backend->takeTextureKeyframes()
                          : std::vector<TextureKeyframe>{};
}

void JMScanner::setStateCallback(StateCallback callback) {
    std::lock_guard<ScannerMutex> lock(impl_->mutex);
    impl_->stateCallback = std::move(callback);
}

void JMScanner::setFrameCallback(FrameCallback callback) {
    std::lock_guard<ScannerMutex> lock(impl_->mutex);
    impl_->frameCallback = std::move(callback);
}

void JMScanner::setMessageCallback(MessageCallback callback) {
    std::lock_guard<ScannerMutex> lock(impl_->mutex);
    impl_->messageCallback = std::move(callback);
}

void JMScanner::setProgressCallback(ProgressCallback callback) {
    std::lock_guard<ScannerMutex> lock(impl_->mutex);
    impl_->progressCallback = std::move(callback);
}

void JMScanner::setMarkerCallback(MarkerCallback callback) {
    std::lock_guard<ScannerMutex> lock(impl_->mutex);
    impl_->markerCallback = std::move(callback);
}

void JMScanner::setPoseUpdateCallback(PoseUpdateCallback callback) {
    std::lock_guard<ScannerMutex> lock(impl_->mutex);
    impl_->poseUpdateCallback = std::move(callback);
}

} // namespace JMEngine
