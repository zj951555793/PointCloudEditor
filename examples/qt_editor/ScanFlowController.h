#pragma once

#include <QObject>
#include <QThread>
#include <QString>
#include <QImage>
#include <atomic>
#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>
#include <deque>
#include <unordered_map>

#include <JMEngine/PointCloud.h>
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
#include <JMEngine/texture/TextureMapper.h>
#endif
#include "CameraDeviceManager.h"

enum class ScanSourceMode {
    Virtual = 0,
    Camera = 1
};

enum class ScanCameraRole {
    CameraA = 0,
    CameraB = 1
};

// Registration strategy selected from the scan operator UI.
// Keep this independent from ScanSourceMode: source mode decides where frames come from,
// registration mode decides how consecutive scans are aligned.
enum class ScanRegistrationMode {
    Geometry = 0,
    Texture = 1,
    Marker = 2
};


struct ScanMarkerPoint {
    float x{0.0f};
    float y{0.0f};
    float width{0.0f};
    float height{0.0f};
    float angleDeg{0.0f};
    int localId{-1};
    bool hasDepth{false};
    std::array<float, 3> point3d{0.0f, 0.0f, 0.0f};
};

struct ScanMarkerFrame {
    int frameId{-1};
    int imageWidth{0};
    int imageHeight{0};
    qint64 timestampUs{0};
    std::vector<ScanMarkerPoint> markers;
};

struct LiveFramePoseUpdate {
    int frameId{-1};
    std::array<float, 16> pose{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
};

struct ScanPoseState {
    std::array<float, 3> position{0.0f, 0.0f, 0.0f};
    std::array<float, 3> right{1.0f, 0.0f, 0.0f};
    std::array<float, 3> up{0.0f, 1.0f, 0.0f};
    std::array<float, 3> forward{0.0f, 0.0f, 1.0f};
    bool trackingOk{false};
    int frameId{-1};
};

struct ScanConfig {
    ScanSourceMode sourceMode{ScanSourceMode::Virtual};
    ScanRegistrationMode registrationMode{ScanRegistrationMode::Texture};

    // Common algorithm configuration.
    QString calibrationPath; // Production camera mode should explicitly set this.
    QString vocabularyPath;
    int maxFrames{2000};
    int maxInflightFrames{6};
    int previewPointsPerFrame{12000};
    int previewPointLimit{500000};
    double offlineVoxel{3.0};
    int offlineIterations{30};
    bool liveOptimizationEnabled{true};
    int textureKeyframeStride{5};
    int textureMaxKeyframes{120};
    int textureImageScaleDivisor{1}; // Industrial texture mapping always prefers the original RGB resolution

    // Virtual source: <dataDir>/img/c + <dataDir>/img/p. If calibrationPath is empty,
    // <dataDir>/calib.txt is used for backward compatibility.
    QString dataDir;

    // Production dual-camera source.
    QString cameraModelJsonPath;
    QString cameraADeviceId;
    QString cameraBDeviceId;
    double cameraAExposure{-6.0};
    double cameraBExposure{-6.0};
    double cameraABacklight{10.0};
    double cameraBBacklight{10.0};
    double cameraSyncToleranceMs{50.0};
    int cameraQueueDepth{3};
};

class ScanFlowController final : public QObject {
  public:
    enum class State {
        Idle,
        Initializing,
        Scanning,
        Stopping,
        ReadyForReconstruction,
        Reconstructing,
        Error
    };

    using PointChunk = std::vector<JMEngine::Point>;
    using PointChunkPtr = std::shared_ptr<PointChunk>;
    using CloudPtr = std::shared_ptr<JMEngine::PointCloud>;
    using StateCallback = std::function<void(State)>;
    using MessageCallback = std::function<void(const QString&)>;
    using PreviewCallback = std::function<void(PointChunkPtr)>;
    using CurrentFrameCallback = std::function<void(PointChunkPtr, bool, int)>;
    using ReconstructionCallback = std::function<void(CloudPtr)>;
    using LiveFrameCallback = std::function<void(PointChunkPtr, const std::array<float,16>&, int)>;
    using LivePoseUpdatesCallback = std::function<void(std::shared_ptr<std::vector<LiveFramePoseUpdate>>) >;
    using CameraListCallback = std::function<void(const QVector<CameraDeviceInfo>&, const QString&)>;
    using CameraPreviewCallback = std::function<void(const QImage&)>;
    using PoseCallback = std::function<void(const ScanPoseState&)>;
    using MarkerFrameCallback = std::function<void(const ScanMarkerFrame&)>;
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    using TextureFrames = std::vector<JMEngine::texture::CameraFrame>;
    using TextureFramesPtr = std::shared_ptr<TextureFrames>;
    using TextureFramesCallback = std::function<void(TextureFramesPtr)>;
#endif

    explicit ScanFlowController(QObject* parent = nullptr);
    ~ScanFlowController() override;

    void setConfig(const ScanConfig& config);
    ScanConfig config() const;
    State state() const noexcept { return state_; }

    void startScan();
    void stopScan();
    void offlineReconstruct();
    void reset();

    // Camera operations are always posted to sourceThread_. They never touch cv::VideoCapture on UI thread.
    void refreshCameras();
    void setCameraExposure(ScanCameraRole role, double value);
    void setCameraBacklight(ScanCameraRole role, double value);

    void setStateCallback(StateCallback cb) { stateCallback_ = std::move(cb); }
    void setMessageCallback(MessageCallback cb) { messageCallback_ = std::move(cb); }
    void setPreviewCallback(PreviewCallback cb) { previewCallback_ = std::move(cb); }
    void setCurrentFrameCallback(CurrentFrameCallback cb) { currentFrameCallback_ = std::move(cb); }
    void setReconstructionCallback(ReconstructionCallback cb) { reconstructionCallback_ = std::move(cb); }
    void setLiveFrameCallback(LiveFrameCallback cb) { liveFrameCallback_ = std::move(cb); }
    void setLivePoseUpdatesCallback(LivePoseUpdatesCallback cb) { livePoseUpdatesCallback_ = std::move(cb); }
    void setCameraListCallback(CameraListCallback cb) { cameraListCallback_ = std::move(cb); }
    void setCameraPreviewCallback(CameraPreviewCallback cb) { cameraPreviewCallback_ = std::move(cb); }
    void setPoseCallback(PoseCallback cb) { poseCallback_ = std::move(cb); }
    void setMarkerFrameCallback(MarkerFrameCallback cb) { markerFrameCallback_ = std::move(cb); }
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    void setTextureFramesCallback(TextureFramesCallback cb) { textureFramesCallback_ = std::move(cb); }
#endif

    static QString stateText(State state);

  private:
    class ScanSourceWorker;
    class RulerMvsWorker;

    void setState(State state);
    void postMessage(const QString& message);
    void requestNextFrame();
    void beginStopping(bool sourceExhausted);
    void shutdownThreads();
    void scheduleRenderDispatch();
    void flushRenderMailbox();

    ScanConfig config_;
    State state_{State::Idle};
    QThread sourceThread_;
    QThread pipelineThread_;
    ScanSourceWorker* source_{nullptr};
    RulerMvsWorker* pipeline_{nullptr};
    bool sourceExhausted_{false};
    bool terminalError_{false};
    quint64 runEpoch_{0};

    StateCallback stateCallback_;
    MessageCallback messageCallback_;
    PreviewCallback previewCallback_;
    CurrentFrameCallback currentFrameCallback_;
    ReconstructionCallback reconstructionCallback_;
    LiveFrameCallback liveFrameCallback_;
    LivePoseUpdatesCallback livePoseUpdatesCallback_;
    CameraListCallback cameraListCallback_;
    CameraPreviewCallback cameraPreviewCallback_;
    PoseCallback poseCallback_;
    MarkerFrameCallback markerFrameCallback_;
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    TextureFramesCallback textureFramesCallback_;
#endif

    // Latest-frame-only camera preview.
    std::mutex cameraPreviewMutex_;
    QImage latestCameraPreview_;
    std::atomic<bool> cameraPreviewDispatchPending_{false};

    // Render mailbox: worker threads never flood the GUI event queue. Persistent live frames
    // are batched (not dropped); transient overlays/pose are latest-only; pose optimization
    // updates are coalesced by frameId. This keeps rendering responsive even when SLAM bursts.
    struct PendingLiveFrame {
        PointChunkPtr points;
        std::array<float,16> pose{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        int frameId{-1};
    };
    std::mutex renderMailboxMutex_;
    std::deque<PendingLiveFrame> pendingLiveFrames_;
    PointChunkPtr latestPreviewChunk_;
    PointChunkPtr latestCurrentFrame_;
    bool latestCurrentTrackingOk_{false};
    int latestCurrentFrameId_{-1};
    bool hasCurrentFrame_{false};
    ScanPoseState latestPose_{};
    bool hasPose_{false};
    std::unordered_map<int, std::array<float,16>> pendingPoseUpdates_;
    std::atomic<bool> renderDispatchPending_{false};
};
