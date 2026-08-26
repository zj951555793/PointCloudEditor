#pragma once

#include "ScanTypes.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace JMEngine {

struct CameraDeviceInfo {
    int index{-1};
    std::string id;
    std::string name;
    std::string vid;
    std::string pid;
};

struct CameraDeviceConfig {
    int index{-1};
    int width{1920};
    int height{1200};
    int fps{10};
    std::string fourcc{"MJPG"};
    std::string model;
    // OpenCV cv::flip code loaded from camera_models.json:
    //   -1 = horizontal + vertical, 0 = vertical, 1 = horizontal.
    // Any other value means no image transform.
    int rotate{2};
    double exposure{-6.0};
    double backlight{25.0};
};

struct DualCameraConfig {
    CameraDeviceConfig cameraA;
    CameraDeviceConfig cameraB;
    double syncToleranceMs{10.0};
    int queueDepth{3};
    bool recordRawData{false};
    std::string rawDataDirectory;
    std::string calibrationPath;
};

class ICameraSource {
  public:
    using FrameCallback = std::function<void(CameraFrame)>;
    using ErrorCallback = std::function<void(const std::string&)>;
    using PreviewCallback = std::function<void(
        std::shared_ptr<std::vector<std::uint8_t>>, int, int)>;

    virtual ~ICameraSource() = default;

    virtual bool start(const DualCameraConfig& config,
                       FrameCallback frameCallback,
                       ErrorCallback errorCallback,
                       std::string* error) = 0;
    virtual void stop() = 0;
    virtual void setExposure(int role, double value) {}
    virtual void setBacklight(int role, double value) {}
    virtual void setPreviewCallback(PreviewCallback callback) {}
};

std::unique_ptr<ICameraSource> createPlatformCameraSource();
std::unique_ptr<ICameraSource> createDatasetCameraSource(
    const std::string& dataDirectory);
std::vector<CameraDeviceInfo> enumerateCameraDevices(
    std::string* error = nullptr);

} // namespace JMEngine
