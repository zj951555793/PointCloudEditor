#include "JMEngine/CameraSource.h"

// Source revision: JMEngine-CameraSource-V21.
// If Visual Studio still reports cfg_, frame_, opened_, a_, b_ or m_ here,
// it is compiling a stale copy of this file.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <limits>
#include <mutex>
#include <thread>
#include <utility>

#if defined(JMENGINE_HAS_RULERMVS)
#include <opencv2/opencv.hpp>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <dshow.h>
#include <windows.h>
#endif
#endif

namespace JMEngine {

#if defined(JMENGINE_HAS_RULERMVS)
namespace {

class OpenCvDualCamera final : public ICameraSource {
    struct CapturedFrame {
        cv::Mat image;
        std::uint64_t timestampUs{0};
    };

  public:
    ~OpenCvDualCamera() override {
        stop();
    }

    bool start(const DualCameraConfig& config,
               FrameCallback frameCallback,
               ErrorCallback errorCallback,
               std::string* error) override {
        stop();
        config_ = config;
        if (!prepareRawRecorder(error))
            return false;
        desiredExposure_[0].store(config.cameraA.exposure);
        desiredExposure_[1].store(config.cameraB.exposure);
        desiredBacklight_[0].store(config.cameraA.backlight);
        desiredBacklight_[1].store(config.cameraB.backlight);
        frameCallback_ = std::move(frameCallback);
        errorCallback_ = std::move(errorCallback);
        stopping_.store(false);
        openedCameraCount_ = 0;
        openError_.clear();
        if (config_.recordRawData) {
            recordStopping_.store(false);
            recordThread_ = std::thread([this] { rawRecordLoop(); });
        }

        cameraAThread_ = std::thread([this] {
            captureLoop(config_.cameraA, true);
        });
        cameraBThread_ = std::thread([this] {
            captureLoop(config_.cameraB, false);
        });

        std::unique_lock<std::mutex> lock(mutex_);
        cameraOpenCondition_.wait_for(
            lock, std::chrono::seconds(6),
            [this] { return openedCameraCount_ == 2; });
        if (openedCameraCount_ != 2 || !openError_.empty()) {
            if (error) {
                *error = openError_.empty() ? "camera open timeout" : openError_;
            }
            lock.unlock();
            stop();
            return false;
        }

        pairingThread_ = std::thread([this] { pairingLoop(); });
        return true;
    }

    void stop() override {
        stopping_.store(true);
        frameCondition_.notify_all();
        if (cameraAThread_.joinable())
            cameraAThread_.join();
        if (cameraBThread_.joinable())
            cameraBThread_.join();
        if (pairingThread_.joinable())
            pairingThread_.join();

        recordStopping_.store(true);
        recordCondition_.notify_all();
        if (recordThread_.joinable())
            recordThread_.join();

        {
            std::lock_guard<std::mutex> recordLock(recordMutex_);
            recordQueue_.clear();
        }
        std::lock_guard<std::mutex> lock(mutex_);
        cameraAQueue_.clear();
        cameraBQueue_.clear();
    }

    void setExposure(int role, double value) override {
        if (role >= 0 && role < 2)
            desiredExposure_[std::size_t(role)].store(value);
    }

    void setBacklight(int role, double value) override {
        if (role >= 0 && role < 2)
            desiredBacklight_[std::size_t(role)].store(value);
    }

    void setPreviewCallback(PreviewCallback callback) override {
        std::lock_guard<std::mutex> lock(previewMutex_);
        previewCallback_ = std::move(callback);
    }

  private:
    static std::uint64_t nowUs() {
        return std::uint64_t(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }

    void captureLoop(const CameraDeviceConfig& cameraConfig, bool cameraA) {
        cv::VideoCapture capture;
        const bool opened = capture.open(
            cameraConfig.index,
#if defined(_WIN32)
            cv::CAP_DSHOW
#else
            cv::CAP_ANY
#endif
        );

        const int role = cameraA ? 0 : 1;
        double exposure = desiredExposure_[std::size_t(role)].load();
        double backlight = desiredBacklight_[std::size_t(role)].load();
        if (opened) {
            std::string fourcc = cameraConfig.fourcc.empty() ? "MJPG" : cameraConfig.fourcc;
            while (fourcc.size() < 4) fourcc.push_back(' ');
            capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(
                fourcc[0], fourcc[1], fourcc[2], fourcc[3]));
            capture.set(cv::CAP_PROP_FRAME_WIDTH, cameraConfig.width);
            capture.set(cv::CAP_PROP_FRAME_HEIGHT, cameraConfig.height);
            capture.set(cv::CAP_PROP_FPS, cameraConfig.fps);
            capture.set(cv::CAP_PROP_EXPOSURE, exposure);
            capture.set(cv::CAP_PROP_BACKLIGHT, backlight);
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!opened && openError_.empty()) {
                openError_ = "failed to open camera index " +
                             std::to_string(cameraConfig.index);
            }
            ++openedCameraCount_;
        }
        cameraOpenCondition_.notify_all();
        if (!opened)
            return;

        while (!stopping_.load()) {
            const double nextExposure =
                desiredExposure_[std::size_t(role)].load();
            const double nextBacklight =
                desiredBacklight_[std::size_t(role)].load();
            if (nextExposure != exposure) {
                capture.set(cv::CAP_PROP_EXPOSURE, nextExposure);
                exposure = nextExposure;
            }
            if (!std::isfinite(backlight) || nextBacklight != backlight) {
                if (capture.set(cv::CAP_PROP_BACKLIGHT, nextBacklight))
                    backlight = nextBacklight;
            }

            cv::Mat image;
            if (!capture.grab()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            const std::uint64_t timestampUs = nowUs();
            if (!capture.retrieve(image) || image.empty())
                continue;

            // Camera B drives the Qt preview and render cadence directly.
            // It must not wait for A/B pairing or for the SLAM worker.
            if (!cameraA)
                publishPreview(image);

            CapturedFrame frame{std::move(image), timestampUs};
            {
                std::lock_guard<std::mutex> lock(mutex_);
                auto& queue = cameraA ? cameraAQueue_ : cameraBQueue_;
                const auto queueDepth =
                    std::size_t(std::max(1, config_.queueDepth));
                while (queue.size() >= queueDepth)
                    queue.pop_front();
                queue.push_back(std::move(frame));
            }
            frameCondition_.notify_one();
        }
        capture.release();
    }

    void pairingLoop() {
        int frameId = 0;
        const auto toleranceUs = std::uint64_t(
            std::max(0.0, config_.syncToleranceMs) * 1000.0);

        while (!stopping_.load()) {
            CapturedFrame cameraA;
            CapturedFrame cameraB;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                frameCondition_.wait(lock, [this] {
                    return stopping_.load() ||
                           (!cameraAQueue_.empty() && !cameraBQueue_.empty());
                });
                if (stopping_.load())
                    break;

                while (!cameraAQueue_.empty() && !cameraBQueue_.empty()) {
                    const auto timestampA = cameraAQueue_.front().timestampUs;
                    const auto timestampB = cameraBQueue_.front().timestampUs;
                    const auto delta = timestampA > timestampB
                        ? timestampA - timestampB
                        : timestampB - timestampA;
                    if (delta <= toleranceUs) {
                        cameraA = std::move(cameraAQueue_.front());
                        cameraB = std::move(cameraBQueue_.front());
                        cameraAQueue_.pop_front();
                        cameraBQueue_.pop_front();
                        break;
                    }
                    if (timestampA < timestampB)
                        cameraAQueue_.pop_front();
                    else
                        cameraBQueue_.pop_front();
                }
            }

            if (cameraA.image.empty() || cameraB.image.empty())
                continue;

            // Camera B stays BGR for RGBDFusion.
            cv::Mat rgb;
            cv::Mat code;
            if (cameraB.image.channels() == 3)
                rgb = cameraB.image;
            else
                cv::cvtColor(cameraB.image, rgb, cv::COLOR_GRAY2BGR);
            if (cameraA.image.channels() == 1)
                code = cameraA.image;
            else
                cv::cvtColor(cameraA.image, code, cv::COLOR_BGR2GRAY);

            if (rgb.size() != code.size()) {
                if (errorCallback_)
                    errorCallback_("paired camera frame sizes differ");
                continue;
            }

            CameraFrame frame;
            frame.frameId = frameId++;
            frame.timestampUs =
                std::max(cameraA.timestampUs, cameraB.timestampUs);
            frame.width = rgb.cols;
            frame.height = rgb.rows;
            frame.codeWidth = code.cols;
            frame.codeHeight = code.rows;
            const auto pixelCount =
                std::size_t(frame.width) * std::size_t(frame.height);
            frame.rgb = std::make_shared<std::vector<std::uint8_t>>(
                rgb.data, rgb.data + pixelCount * 3u);
            frame.code = std::make_shared<std::vector<std::uint8_t>>(
                code.data, code.data + pixelCount);

            enqueueRawRecord(frame);
            if (frameCallback_)
                frameCallback_(std::move(frame));
        }
    }

    bool prepareRawRecorder(std::string* error) {
        if (!config_.recordRawData)
            return true;
        if (config_.rawDataDirectory.empty()) {
            if (error) *error = "rawDataDirectory is empty";
            return false;
        }
        std::error_code ec;
        rawRoot_ = std::filesystem::path(config_.rawDataDirectory);
        rawColorDir_ = rawRoot_ / "img" / "c";
        rawCodeDir_ = rawRoot_ / "img" / "p";
        std::filesystem::create_directories(rawColorDir_, ec);
        if (ec) {
            if (error) *error = "failed to create raw color directory: " + ec.message();
            return false;
        }
        std::filesystem::create_directories(rawCodeDir_, ec);
        if (ec) {
            if (error) *error = "failed to create raw code directory: " + ec.message();
            return false;
        }
        if (!config_.calibrationPath.empty()) {
            const auto source = std::filesystem::path(config_.calibrationPath);
            const auto target = rawRoot_ / "calib.txt";
            ec.clear();
            std::filesystem::copy_file(source, target,
                                       std::filesystem::copy_options::overwrite_existing, ec);
            if (ec) {
                if (error) *error = "failed to copy calibration file: " + ec.message();
                return false;
            }
        }
        return true;
    }

    static std::string rawFrameName(int frameId) {
        std::ostringstream stream;
        stream << std::setw(6) << std::setfill('0') << frameId << ".png";
        return stream.str();
    }

    void enqueueRawRecord(const CameraFrame& frame) {
        if (!config_.recordRawData || recordStopping_.load() ||
            !frame.rgb || !frame.code)
            return;
        {
            std::lock_guard<std::mutex> lock(recordMutex_);
            constexpr std::size_t kMaxQueuedFrames = 256;
            if (recordQueue_.size() >= kMaxQueuedFrames) {
                recordQueue_.pop_front();
                ++droppedRawFrames_;
            }
            recordQueue_.push_back(frame);
        }
        recordCondition_.notify_one();
    }

    void rawRecordLoop() {
        for (;;) {
            CameraFrame frame;
            {
                std::unique_lock<std::mutex> lock(recordMutex_);
                recordCondition_.wait(lock, [this] {
                    return recordStopping_.load() || !recordQueue_.empty();
                });
                if (recordQueue_.empty()) {
                    if (recordStopping_.load())
                        break;
                    continue;
                }
                frame = std::move(recordQueue_.front());
                recordQueue_.pop_front();
            }

            if (!frame.rgb || !frame.code || frame.width <= 0 || frame.height <= 0)
                continue;
            const int codeWidth = frame.codeWidth > 0 ? frame.codeWidth : frame.width;
            const int codeHeight = frame.codeHeight > 0 ? frame.codeHeight : frame.height;
            cv::Mat color(frame.height, frame.width, CV_8UC3, frame.rgb->data());
            cv::Mat code(codeHeight, codeWidth, CV_8UC1, frame.code->data());
            const auto name = rawFrameName(frame.frameId);
            try {
                cv::imwrite((rawColorDir_ / name).string(), color);
                cv::imwrite((rawCodeDir_ / name).string(), code);
            } catch (const cv::Exception& e) {
                if (errorCallback_)
                    errorCallback_(std::string("failed to save raw scan frame: ") + e.what());
            }
        }
    }

    void publishPreview(const cv::Mat& source) {
        PreviewCallback callback;
        {
            std::lock_guard<std::mutex> lock(previewMutex_);
            callback = previewCallback_;
        }
        if (!callback)
            return;

        cv::Mat preview = source;
        if (source.cols > 384) {
            const double scale = 384.0 / double(source.cols);
            cv::resize(
                source, preview, cv::Size(), scale, scale, cv::INTER_AREA);
        }

        cv::Mat rgb;
        if (preview.channels() == 3)
            cv::cvtColor(preview, rgb, cv::COLOR_BGR2RGB);
        else
            cv::cvtColor(preview, rgb, cv::COLOR_GRAY2RGB);
        if (!rgb.isContinuous())
            rgb = rgb.clone();

        const std::size_t byteCount = rgb.total() * rgb.elemSize();
        auto pixels = std::make_shared<std::vector<std::uint8_t>>(
            rgb.data, rgb.data + byteCount);
        callback(std::move(pixels), rgb.cols, rgb.rows);
    }

    DualCameraConfig config_;
    FrameCallback frameCallback_;
    ErrorCallback errorCallback_;
    PreviewCallback previewCallback_;
    std::mutex previewMutex_;
    std::filesystem::path rawRoot_;
    std::filesystem::path rawColorDir_;
    std::filesystem::path rawCodeDir_;
    std::thread recordThread_;
    std::mutex recordMutex_;
    std::condition_variable recordCondition_;
    std::deque<CameraFrame> recordQueue_;
    std::atomic<bool> recordStopping_{true};
    std::uint64_t droppedRawFrames_{0};
    std::atomic<bool> stopping_{true};
    std::array<std::atomic<double>, 2> desiredExposure_{};
    std::array<std::atomic<double>, 2> desiredBacklight_{};
    std::thread cameraAThread_;
    std::thread cameraBThread_;
    std::thread pairingThread_;
    std::mutex mutex_;
    std::condition_variable frameCondition_;
    std::condition_variable cameraOpenCondition_;
    std::deque<CapturedFrame> cameraAQueue_;
    std::deque<CapturedFrame> cameraBQueue_;
    int openedCameraCount_{0};
    std::string openError_;
};

class DatasetCamera final : public ICameraSource {
  public:
    explicit DatasetCamera(std::string directory)
        : directory_(std::move(directory)) {}

    ~DatasetCamera() override {
        stop();
    }

    bool start(const DualCameraConfig&, FrameCallback frameCallback,
               ErrorCallback errorCallback, std::string* error) override {
        stop();
        frameCallback_ = std::move(frameCallback);
        errorCallback_ = std::move(errorCallback);
        rgbFiles_ = collectImages(
            std::filesystem::path(directory_) / "img" / "c");
        codeFiles_ = collectImages(
            std::filesystem::path(directory_) / "img" / "p");
        if (rgbFiles_.empty() || codeFiles_.empty()) {
            if (error)
                *error = "dataset must contain img/c and img/p images";
            return false;
        }

        stopping_.store(false);
        worker_ = std::thread([this] { run(); });
        return true;
    }

    void stop() override {
        stopping_.store(true);
        if (worker_.joinable())
            worker_.join();
    }

    void setPreviewCallback(PreviewCallback callback) override {
        previewCallback_ = std::move(callback);
    }

  private:
    static std::vector<std::filesystem::path> collectImages(
        const std::filesystem::path& directory) {
        std::vector<std::filesystem::path> files;
        std::error_code error;
        if (!std::filesystem::exists(directory, error))
            return files;

        for (const auto& entry :
             std::filesystem::directory_iterator(directory, error)) {
            if (!entry.is_regular_file())
                continue;
            auto extension = entry.path().extension().string();
            std::transform(
                extension.begin(), extension.end(), extension.begin(),
                [](unsigned char value) { return char(std::tolower(value)); });
            if (extension == ".jpg" || extension == ".jpeg" ||
                extension == ".png" || extension == ".bmp") {
                files.push_back(entry.path());
            }
        }
        std::sort(files.begin(), files.end());
        return files;
    }

    void run() {
        const std::size_t frameCount =
            std::min(rgbFiles_.size(), codeFiles_.size());
        for (std::size_t index = 0;
             index < frameCount && !stopping_.load(); ++index) {
            cv::Mat bgr = cv::imread(
                rgbFiles_[index].string(), cv::IMREAD_COLOR);
            cv::Mat code = cv::imread(
                codeFiles_[index].string(), cv::IMREAD_GRAYSCALE);
            if (bgr.empty() || code.empty()) {
                if (errorCallback_) {
                    errorCallback_("failed to read dataset frame " +
                                   std::to_string(index));
                }
                continue;
            }
            // Preserve the dataset's native structured-light code resolution.
            // OneShot datasets intentionally use code widths such as 216/343;
            // resizing the code image to the RGB size corrupts the decoder input.

            // Dataset and physical cameras must present the same BGR contract to
            // RulerMvsSlam; UI preview performs its own BGR->RGB conversion.
            cv::Mat rgb = bgr;
            CameraFrame frame;
            frame.frameId = int(index);
            frame.timestampUs = std::uint64_t(index) * 100000u;
            frame.width = rgb.cols;
            frame.height = rgb.rows;
            frame.codeWidth = code.cols;
            frame.codeHeight = code.rows;
            const auto pixelCount =
                std::size_t(frame.width) * std::size_t(frame.height);
            const auto codePixelCount =
                std::size_t(frame.codeWidth) * std::size_t(frame.codeHeight);
            frame.rgb = std::make_shared<std::vector<std::uint8_t>>(
                rgb.data, rgb.data + pixelCount * 3u);
            frame.code = std::make_shared<std::vector<std::uint8_t>>(
                code.data, code.data + codePixelCount);

            // Virtual and real scan share the exact same SLAM->render path.
            // Camera preview is a physical-camera UI feature only and must not be
            // used as a second 3D render clock for dataset scanning.
            if (frameCallback_)
                frameCallback_(std::move(frame));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    std::string directory_;
    FrameCallback frameCallback_;
    ErrorCallback errorCallback_;
    PreviewCallback previewCallback_;
    std::atomic<bool> stopping_{true};
    std::thread worker_;
    std::vector<std::filesystem::path> rgbFiles_;
    std::vector<std::filesystem::path> codeFiles_;
};

#if defined(_WIN32)
std::string wideToUtf8(const wchar_t* text) {
    if (!text)
        return {};
    const int size = WideCharToMultiByte(
        CP_UTF8, 0, text, -1, nullptr, 0, nullptr, nullptr);
    std::string output(size > 0 ? std::size_t(size) : 0u, '\0');
    if (size > 1) {
        WideCharToMultiByte(
            CP_UTF8, 0, text, -1, output.data(), size, nullptr, nullptr);
        output.pop_back();
    }
    return output;
}
#endif

} // namespace

std::unique_ptr<ICameraSource> createPlatformCameraSource() {
    return std::make_unique<OpenCvDualCamera>();
}

std::unique_ptr<ICameraSource> createDatasetCameraSource(
    const std::string& directory) {
    return std::make_unique<DatasetCamera>(directory);
}

std::vector<CameraDeviceInfo> enumerateCameraDevices(std::string* error) {
    std::vector<CameraDeviceInfo> devices;
#if defined(_WIN32)
    const HRESULT initializeResult =
        CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    const bool uninitialize = SUCCEEDED(initializeResult);
    ICreateDevEnum* deviceEnumerator = nullptr;
    IEnumMoniker* monikerEnumerator = nullptr;
    HRESULT result = CoCreateInstance(
        CLSID_SystemDeviceEnum, nullptr, CLSCTX_INPROC_SERVER,
        IID_ICreateDevEnum, reinterpret_cast<void**>(&deviceEnumerator));
    if (SUCCEEDED(result) && deviceEnumerator) {
        result = deviceEnumerator->CreateClassEnumerator(
            CLSID_VideoInputDeviceCategory, &monikerEnumerator, 0);
    }

    if (result == S_OK && monikerEnumerator) {
        IMoniker* moniker = nullptr;
        ULONG fetched = 0;
        int index = 0;
        while (monikerEnumerator->Next(1, &moniker, &fetched) == S_OK) {
            CameraDeviceInfo device;
            device.index = index++;
            IPropertyBag* propertyBag = nullptr;
            if (SUCCEEDED(moniker->BindToStorage(
                    nullptr, nullptr, IID_IPropertyBag,
                    reinterpret_cast<void**>(&propertyBag))) && propertyBag) {
                VARIANT value;
                VariantInit(&value);
                if (SUCCEEDED(propertyBag->Read(
                        L"FriendlyName", &value, nullptr)) &&
                    value.vt == VT_BSTR) {
                    device.name = wideToUtf8(value.bstrVal);
                }
                VariantClear(&value);
                VariantInit(&value);
                if (SUCCEEDED(propertyBag->Read(
                        L"DevicePath", &value, nullptr)) &&
                    value.vt == VT_BSTR) {
                    device.id = wideToUtf8(value.bstrVal);
                }
                VariantClear(&value);
                propertyBag->Release();
            }

            std::string lower = device.id;
            std::transform(
                lower.begin(), lower.end(), lower.begin(),
                [](unsigned char value) { return char(std::tolower(value)); });
            const auto extract = [&lower](const char* key) {
                const auto position = lower.find(key);
                return position == std::string::npos || position + 8 > lower.size()
                    ? std::string{}
                    : lower.substr(position + 4, 4);
            };
            device.vid = extract("vid_");
            device.pid = extract("pid_");
            if (device.id.empty())
                device.id = std::to_string(device.index);
            if (device.name.empty())
                device.name = "Camera " + std::to_string(device.index);
            devices.push_back(std::move(device));
            moniker->Release();
        }
        monikerEnumerator->Release();
    } else if (error) {
        *error = "DirectShow did not enumerate a camera";
    }

    if (deviceEnumerator)
        deviceEnumerator->Release();
    if (uninitialize)
        CoUninitialize();
#else
    for (int index = 0; index < 10; ++index) {
        const std::filesystem::path path =
            "/dev/video" + std::to_string(index);
        std::error_code fileError;
        if (std::filesystem::exists(path, fileError)) {
            devices.push_back({
                index, path.string(), "Camera " + std::to_string(index), {}, {}});
        }
    }
    if (devices.empty() && error)
        *error = "no /dev/video camera devices found";
#endif
    return devices;
}

#else
namespace {

class MissingCameraSource final : public ICameraSource {
  public:
    bool start(const DualCameraConfig&, FrameCallback, ErrorCallback,
               std::string* error) override {
        if (error)
            *error = "JMEngine camera source requires the OpenCV/RulerMVS build";
        return false;
    }

    void stop() override {}
};

} // namespace

std::unique_ptr<ICameraSource> createPlatformCameraSource() {
    return std::make_unique<MissingCameraSource>();
}

std::unique_ptr<ICameraSource> createDatasetCameraSource(const std::string&) {
    return std::make_unique<MissingCameraSource>();
}

std::vector<CameraDeviceInfo> enumerateCameraDevices(std::string* error) {
    if (error)
        *error = "JMEngine camera enumeration requires the OpenCV/RulerMVS build";
    return {};
}

#endif

} // namespace JMEngine
