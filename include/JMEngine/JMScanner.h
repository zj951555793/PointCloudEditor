#pragma once

#include "CameraSource.h"
#include "ISlam.h"
#include "ScanTypes.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace JMEngine {

class JMScanner {
  public:
    using StateCallback = std::function<void(ScanState)>;
    using FrameCallback = std::function<void(
        int, const Pose&, std::shared_ptr<PointCloud>,
        std::shared_ptr<PointCloud>, bool)>;
    using MessageCallback = std::function<void(const std::string&)>;
    using ProgressCallback = std::function<void(int)>;
    using MarkerCallback = std::function<void(const ScanMarkerFrame&)>;
    using PreviewCallback = std::function<void(
        std::shared_ptr<std::vector<std::uint8_t>>, int, int)>;

    JMScanner();
    explicit JMScanner(std::unique_ptr<ISlam> backend);
    ~JMScanner();

    JMScanner(const JMScanner&) = delete;
    JMScanner& operator=(const JMScanner&) = delete;

    bool initialize(const ScanConfig& config);
    bool start();
    bool submit(CameraFrame frame);
    void stop();
    bool reconstruct();
    void reset();

    bool startCameras(const DualCameraConfig& config);
    bool startDataset(const std::string& dataDirectory);
    void setCameraExposure(int role, double value);
    void setCameraBacklight(int role, double value);
    void setCameraPreviewCallback(PreviewCallback callback);

    ScanState state() const noexcept;
    Pose pose() const;
    std::shared_ptr<PointCloud> liveCloud() const;
    std::shared_ptr<PointCloud> resultCloud() const;
    ScanStatistics statistics() const;
    std::string lastError() const;
    std::vector<TextureKeyframe> takeTextureKeyframes();

    void setStateCallback(StateCallback callback);
    void setFrameCallback(FrameCallback callback);
    void setMessageCallback(MessageCallback callback);
    void setProgressCallback(ProgressCallback callback);
    void setMarkerCallback(MarkerCallback callback);

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace JMEngine
