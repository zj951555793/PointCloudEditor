#include "ScanFlowController.h"

#include <QFileInfo>
#include <QCoreApplication>
#include <QDir>
#include <QImage>
#include <QElapsedTimer>
#include <QTimer>
#include <QMetaObject>
#include <QPointer>
#include <QDebug>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <limits>
#include <unordered_map>
#include <mutex>
#include <thread>

#ifdef JMENGINE_HAS_RULERMVS
#include <opencv2/opencv.hpp>
#include <DBoW3/DBoW3.h>
#include "rulermvs.hpp"
#include "rulermvs/image.hpp"
#include "rulermvs/oneshot.hpp"
#include "rulermvs/rgbdslam.h"
#endif

namespace fs = std::filesystem;

namespace {
#ifdef JMENGINE_HAS_RULERMVS
struct RawScanFrame {
    int index{-1};
    cv::Mat rgb;   // Camera B / virtual img/c: BGR color image.
    cv::Mat code;  // Camera A / virtual img/p: grayscale structured-light code image.
    qint64 timestampAUs{0};
    qint64 timestampBUs{0};
};
using RawScanFramePtr = std::shared_ptr<RawScanFrame>;

std::vector<std::string> sortedJpegs(const fs::path& dir) {
    std::vector<std::string> files;
    std::error_code ec;
    if (!fs::exists(dir, ec)) return files;
    for (const auto& entry : fs::directory_iterator(dir, ec)) {
        if (!entry.is_regular_file()) continue;
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return char(std::tolower(c)); });
        if (ext == ".jpg" || ext == ".jpeg" || ext == ".png") files.push_back(entry.path().string());
    }
    std::sort(files.begin(), files.end());
    return files;
}

std::uint32_t packBgr(const cv::Vec3b& bgr) {
    return std::uint32_t(bgr[2]) | (std::uint32_t(bgr[1]) << 8u) |
           (std::uint32_t(bgr[0]) << 16u) | 0xff000000u;
}

JMEngine::Vec3f transformPoint(const cv::Mat& rt, const cv::Point3f& p) {
    if (rt.empty() || rt.rows < 3 || rt.cols < 4) return {p.x, p.y, p.z};
    auto at = [&rt](int r, int c) -> double {
        return rt.type() == CV_32F ? double(rt.at<float>(r, c)) : rt.at<double>(r, c);
    };
    return {float(at(0,0)*p.x + at(0,1)*p.y + at(0,2)*p.z + at(0,3)),
            float(at(1,0)*p.x + at(1,1)*p.y + at(1,2)*p.z + at(1,3)),
            float(at(2,0)*p.x + at(2,1)*p.y + at(2,2)*p.z + at(2,3))};
}

JMEngine::Vec3f transformDirection(const cv::Mat& rt, const cv::Point3f& v) {
    if (rt.empty() || rt.rows < 3 || rt.cols < 3)
        return {v.x, v.y, v.z};
    auto at = [&rt](int r, int c) -> double {
        return rt.type() == CV_32F ? double(rt.at<float>(r, c)) : rt.at<double>(r, c);
    };
    JMEngine::Vec3f out{float(at(0,0)*v.x + at(0,1)*v.y + at(0,2)*v.z),
                        float(at(1,0)*v.x + at(1,1)*v.y + at(1,2)*v.z),
                        float(at(2,0)*v.x + at(2,1)*v.y + at(2,2)*v.z)};
    const float len = std::sqrt(out.x*out.x + out.y*out.y + out.z*out.z);
    if (len > 1e-8f) { out.x/=len; out.y/=len; out.z/=len; }
    return out;
}

JMEngine::Vec3f transformNormal(const cv::Mat& rt, const cv::Point3f& n) {
    if (rt.empty() || rt.rows < 3 || rt.cols < 3) return {n.x, n.y, n.z};
    auto at = [&rt](int r, int c) -> double {
        return rt.type() == CV_32F ? double(rt.at<float>(r, c)) : rt.at<double>(r, c);
    };
    JMEngine::Vec3f out{float(at(0,0)*n.x + at(0,1)*n.y + at(0,2)*n.z),
                       float(at(1,0)*n.x + at(1,1)*n.y + at(1,2)*n.z),
                       float(at(2,0)*n.x + at(2,1)*n.y + at(2,2)*n.z)};
    const float len = std::sqrt(out.x*out.x + out.y*out.y + out.z*out.z);
    if (len > 1e-8f) { out.x /= len; out.y /= len; out.z /= len; }
    return out;
}

cv::Mat poseToCvMat(const rulermvs::Pose& pose) {
    double m[12]{};
    pose.toMatrix(m);
    cv::Mat rt = cv::Mat::eye(4, 4, CV_64F);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c)
            rt.at<double>(r, c) = m[r * 4 + c];
    return rt;
}

// cv::Mat uses row/column indexing, while JMEngine::Mat4f/OpenGL stores matrices
// in column-major order. Convert the latest SLAM RT once before publishing it to UI.
std::array<float, 16> cvPoseToColumnMajor(const cv::Mat& rt) {
    std::array<float, 16> out{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    if (rt.empty() || rt.rows < 3 || rt.cols < 4)
        return out;

    auto at = [&rt](int r, int c) -> double {
        if (rt.type() == CV_32F) return static_cast<double>(rt.at<float>(r, c));
        return rt.at<double>(r, c);
    };

    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c)
            out[static_cast<std::size_t>(c) * 4u + static_cast<std::size_t>(r)] = static_cast<float>(at(r, c));
    return out;
}

bool poseArrayChanged(const std::array<float, 16>& a,
                      const std::array<float, 16>& b,
                      float epsilon = 1.0e-5f) {
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::fabs(a[i] - b[i]) > epsilon)
            return true;
    }
    return false;
}

// Live preview must never apply a single wildly divergent backend RT immediately.
// Such transient poses create sparse sheets far away from the object even though the
// offline optimizer later converges.  The filter affects DISPLAY ONLY; RGBDFusion data
// and final offline reconstruction are untouched.
bool poseUpdateLooksReasonable(const std::array<float, 16>& oldPose,
                               const std::array<float, 16>& newPose) {
    const float dx = newPose[12] - oldPose[12];
    const float dy = newPose[13] - oldPose[13];
    const float dz = newPose[14] - oldPose[14];
    const float translation = std::sqrt(dx*dx + dy*dy + dz*dz);

    // R_delta = R_old^T * R_new.  Clamp trace before acos for numerical stability.
    float trace = 0.0f;
    for (int i = 0; i < 3; ++i) {
        float d = 0.0f;
        for (int k = 0; k < 3; ++k)
            d += oldPose[std::size_t(i)*4u + std::size_t(k)] *
                 newPose[std::size_t(i)*4u + std::size_t(k)];
        trace += d;
    }
    float c = (trace - 1.0f) * 0.5f;
    c = std::max(-1.0f, std::min(1.0f, c));
    const float angleDeg = std::acos(c) * 57.29577951308232f;

    // Coordinates in the current scanner pipeline are millimetres.  These limits are
    // deliberately generous enough for normal local/global refinement, while rejecting
    // one-shot outliers that visibly explode the Live preview.
    return translation <= 120.0f && angleDeg <= 20.0f;
}

#ifdef JMENGINE_HAS_TEXTURE_MAPPING
JMEngine::texture::ImageRGB8 bgrToTextureRgb(const cv::Mat& image) {
    JMEngine::texture::ImageRGB8 out;
    if (image.empty()) return out;
    cv::Mat rgb;
    if (image.channels() == 3) cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
    else if (image.channels() == 4) cv::cvtColor(image, rgb, cv::COLOR_BGRA2RGB);
    else cv::cvtColor(image, rgb, cv::COLOR_GRAY2RGB);
    if (!rgb.isContinuous()) rgb = rgb.clone();
    out.width = rgb.cols;
    out.height = rgb.rows;
    out.pixels.assign(rgb.data, rgb.data + static_cast<std::size_t>(rgb.total()) * 3u);
    return out;
}
#endif

qint64 steadyUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

QImage bgrToQImage(const cv::Mat& image) {
    if (image.empty()) return {};
    if (image.channels() == 3) {
        cv::Mat rgb;
        cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
        return QImage(rgb.data, rgb.cols, rgb.rows, int(rgb.step), QImage::Format_RGB888).copy();
    }
    if (image.channels() == 4) {
        cv::Mat rgba;
        cv::cvtColor(image, rgba, cv::COLOR_BGRA2RGBA);
        return QImage(rgba.data, rgba.cols, rgba.rows, int(rgba.step), QImage::Format_RGBA8888).copy();
    }
    if (image.channels() == 1)
        return QImage(image.data, image.cols, image.rows, int(image.step), QImage::Format_Grayscale8).copy();
    return {};
}

class DualCameraCapture {
  public:
    struct Captured {
        cv::Mat image;
        qint64 timestampUs{0};
        quint64 sequence{0};
    };

    using PreviewCallback = std::function<void(const QImage&)>;

    void setCameraBPreviewCallback(PreviewCallback cb) {
        std::lock_guard<std::mutex> lock(previewMutex_);
        cameraBPreviewCallback_ = std::move(cb);
    }

    bool start(const CameraDeviceInfo& cameraA, const CameraDeviceInfo& cameraB,
               double exposureA, double exposureB, double backlightA, double backlightB, double syncToleranceMs,
               int queueDepth, QString* error) {
        stop();
        cameraA_ = cameraA;
        cameraB_ = cameraB;
        desiredExposureA_.store(exposureA);
        desiredExposureB_.store(exposureB);
        desiredBacklightA_.store(backlightA);
        desiredBacklightB_.store(backlightB);
        syncToleranceUs_ = std::max<qint64>(0, qint64(std::llround(syncToleranceMs * 1000.0)));
        queueDepth_ = std::max(1, queueDepth);
        stopping_.store(false);
        droppedUnsynced_.store(0);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queueA_.clear();
            queueB_.clear();
            openCount_ = 0;
            openError_.clear();
        }

        threadA_ = std::thread([this] { captureLoop(cameraA_, true); });
        threadB_ = std::thread([this] { captureLoop(cameraB_, false); });

        std::unique_lock<std::mutex> lock(mutex_);
        openCv_.wait_for(lock, std::chrono::seconds(6), [this] { return openCount_ >= 2; });
        const bool ok = openCount_ >= 2 && openError_.isEmpty();
        const QString e = openError_.isEmpty() ? QString::fromUtf8("相机打开超时") : openError_;
        lock.unlock();
        if (!ok) {
            stop();
            if (error) *error = e;
            return false;
        }
        return true;
    }

    void stop() {
        stopping_.store(true);
        frameCv_.notify_all();
        if (threadA_.joinable()) threadA_.join();
        if (threadB_.joinable()) threadB_.join();
        std::lock_guard<std::mutex> lock(mutex_);
        queueA_.clear();
        queueB_.clear();
    }

    ~DualCameraCapture() { stop(); }

    void setExposure(bool cameraA, double value) {
        (cameraA ? desiredExposureA_ : desiredExposureB_).store(value);
    }

    void setBacklight(bool cameraA, double value) {
        (cameraA ? desiredBacklightA_ : desiredBacklightB_).store(value);
    }

    bool nextPair(Captured& a, Captured& b, int waitMs) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(waitMs);
        std::unique_lock<std::mutex> lock(mutex_);
        for (;;) {
            while (!queueA_.empty() && !queueB_.empty()) {
                const qint64 ta = queueA_.front().timestampUs;
                const qint64 tb = queueB_.front().timestampUs;
                const qint64 delta = std::llabs(ta - tb);
                if (delta <= syncToleranceUs_) {
                    a = std::move(queueA_.front());
                    b = std::move(queueB_.front());
                    queueA_.pop_front();
                    queueB_.pop_front();
                    return true;
                }
                // Discard the older frame only. The newer frame is retained and compared with the next frame.
                if (ta < tb) queueA_.pop_front(); else queueB_.pop_front();
                droppedUnsynced_.fetch_add(1);
            }
            if (stopping_.load()) return false;
            if (frameCv_.wait_until(lock, deadline) == std::cv_status::timeout) return false;
        }
    }

    quint64 droppedUnsynced() const { return droppedUnsynced_.load(); }

  private:
    static int fourccValue(const QString& text) {
        const QByteArray s = text.toLatin1();
        if (s.size() < 4) return 0;
        return cv::VideoWriter::fourcc(s[0], s[1], s[2], s[3]);
    }

    void captureLoop(CameraDeviceInfo info, bool isA) {
#ifdef Q_OS_WIN
        constexpr int backend = cv::CAP_DSHOW;
#else
        constexpr int backend = cv::CAP_ANY;
#endif
        cv::VideoCapture cap;
     
        const bool opened = cap.open(info.cvIndex, backend);
        if(opened) 
        {            
            cap.set(cv::CAP_PROP_FOURCC,
                    cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            cap.set(cv::CAP_PROP_FRAME_WIDTH, info.width);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, info.height);
            cap.set(cv::CAP_PROP_FPS, info.fps);
        }    
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!opened && openError_.isEmpty())
                openError_ = QString::fromUtf8("打开相机 %1 失败: index=%2, %3")
                                 .arg(isA ? QStringLiteral("A") : QStringLiteral("B"))
                                 .arg(info.cvIndex).arg(info.displayText());
            ++openCount_;
        }
        openCv_.notify_all();
        if (!opened) return;

        double appliedExposure = isA ? desiredExposureA_.load() : desiredExposureB_.load();
        // Important: desiredBacklight* is only the requested/configured value.
        // It does NOT mean the camera has already received that value.
        // Start with an unknown applied value so the first capture-loop iteration
        // always writes CAP_PROP_BACKLIGHT once after the camera has opened.
        double appliedBacklight = std::numeric_limits<double>::quiet_NaN();
        quint64 sequence = 0;
        while (!stopping_.load()) {
            const double wanted = isA ? desiredExposureA_.load() : desiredExposureB_.load();
            if (std::abs(wanted - appliedExposure) > 1e-9) {
                // Do not touch CAP_PROP_AUTO_EXPOSURE on this camera.  Runtime exposure adjustment is
                // deliberately limited to CAP_PROP_EXPOSURE because AUTO_EXPOSURE writes can break
                // the DirectShow stream on this device.
                cap.set(cv::CAP_PROP_EXPOSURE, wanted);
                appliedExposure = wanted;
            }
            {
                const double wantedBacklight = isA ? desiredBacklightA_.load() : desiredBacklightB_.load();
                if (!std::isfinite(appliedBacklight) ||
                    std::abs(wantedBacklight - appliedBacklight) > 1e-9) {
                    // Only mark the requested value as applied when the backend
                    // accepted the write. This also guarantees the JSON/default
                    // value is written once immediately after open().
                    if (cap.set(cv::CAP_PROP_BACKLIGHT, wantedBacklight)) {
                        appliedBacklight = wantedBacklight;
                    }
                }
            }

            if (!cap.grab()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            const qint64 ts = steadyUs(); // Host arrival/grab timestamp; see integration notes for hardware sync limits.
            cv::Mat image;
            if (!cap.retrieve(image) || image.empty()) continue;

            // Camera B is the color camera. Preview must follow the physical camera
            // continuously and must NOT depend on A/B synchronization or SLAM backpressure.
            if (!isA) {
          
                PreviewCallback previewCb;
                {
                    std::lock_guard<std::mutex> lock(previewMutex_);
                    previewCb = cameraBPreviewCallback_;
                }
                if (previewCb) {
                    // Reduce UI transfer bandwidth while keeping the original aspect ratio.
                    cv::Mat previewImage = image;
                    if (image.cols > 384) {
                        const double scale = 384.0 / double(image.cols);
                        cv::resize(image, previewImage, cv::Size(), scale, scale, cv::INTER_AREA);
                    }
                    QImage qimg = bgrToQImage(previewImage);
                    if (!qimg.isNull()) previewCb(qimg);
                }
            } 

            Captured frame{std::move(image), ts, sequence++};
            {
                std::lock_guard<std::mutex> lock(mutex_);
                auto& q = isA ? queueA_ : queueB_;
                q.push_back(std::move(frame));
                while (int(q.size()) > queueDepth_) q.pop_front();
            }
            frameCv_.notify_one();
        }
        cap.release();
    }

    CameraDeviceInfo cameraA_;
    CameraDeviceInfo cameraB_;
    std::thread threadA_;
    std::thread threadB_;
    std::atomic<bool> stopping_{true};
    std::atomic<double> desiredExposureA_{-6.0};
    std::atomic<double> desiredExposureB_{-6.0};
    std::atomic<double> desiredBacklightA_{10.0};
    std::atomic<double> desiredBacklightB_{10.0};
    std::atomic<quint64> droppedUnsynced_{0};
    qint64 syncToleranceUs_{50000};
    int queueDepth_{3};

    std::mutex previewMutex_;
    PreviewCallback cameraBPreviewCallback_;

    mutable std::mutex mutex_;
    std::condition_variable frameCv_;
    std::condition_variable openCv_;
    std::deque<Captured> queueA_;
    std::deque<Captured> queueB_;
    int openCount_{0};
    QString openError_;
};
#endif
} // namespace

class ScanFlowController::ScanSourceWorker final : public QObject {
  public:
#ifdef JMENGINE_HAS_RULERMVS
    using FrameCallback = std::function<void(quint64, RawScanFramePtr)>;
#else
    using FrameCallback = std::function<void(quint64, std::shared_ptr<void>)>;
#endif
    using DoneCallback = std::function<void(quint64)>;
    using ErrorCallback = std::function<void(quint64, const QString&)>;
    using CameraListCallback = std::function<void(const QVector<CameraDeviceInfo>&, const QString&)>;
    using CameraPreviewCallback = std::function<void(const QImage&)>;

    FrameCallback onFrame;
    DoneCallback onDone;
    ErrorCallback onError;
    MessageCallback onMessage;
    CameraListCallback onCameraList;
    CameraPreviewCallback onCameraPreview;

    void discoverCameras(const QString& modelJsonPath) {
        QString error;
        const auto list = CameraDeviceManager::enumerate(modelJsonPath, &error);
        if (onCameraList) onCameraList(list, error);
    }

    void configure(const ScanConfig& config, quint64 runEpoch) {
        stop();
        runEpoch_ = runEpoch;
        config_ = config;
        index_ = 0;
        stopped_ = false;
        virtualRequestScheduled_ = false;
        lastVirtualEmitMs_ = -1;
        virtualClock_.restart();
#ifdef JMENGINE_HAS_RULERMVS
        if (config_.sourceMode == ScanSourceMode::Virtual) {
            rgbNames_ = sortedJpegs(fs::path(config.dataDir.toStdString()) / "img" / "c");
            codeNames_ = sortedJpegs(fs::path(config.dataDir.toStdString()) / "img" / "p");
            total_ = std::min<int>({config.maxFrames, int(rgbNames_.size()), int(codeNames_.size())});
            if (total_ <= 0) {
                stopped_ = true;
                if (onError) onError(runEpoch_, QString::fromUtf8("虚拟扫描目录没有找到 img/c 与 img/p 图像"));
                return;
            }

            // Source is self-driven. Virtual acquisition runs at its own fixed 10 FPS
            // and never waits for any SLAM result/trace callback.
            QMetaObject::invokeMethod(this, [this, epoch = runEpoch_] {
                if (!stopped_ && epoch == runEpoch_)
                    requestNext();
            }, Qt::QueuedConnection);
            return;
        }

        QString enumError;
        const auto devices = CameraDeviceManager::enumerate(config_.cameraModelJsonPath, &enumError);
        const auto findDevice = [&devices](const QString& id) -> const CameraDeviceInfo* {
            for (const auto& d : devices) if (d.deviceId == id) return &d;
            return nullptr;
        };
        const auto* a = findDevice(config_.cameraADeviceId);
        const auto* b = findDevice(config_.cameraBDeviceId);
        if (!a || !b || a->deviceId == b->deviceId) {
            stopped_ = true;
            if (onError) onError(runEpoch_, QString::fromUtf8("相机 A/B 选择无效，请刷新设备并分别选择两个不同相机。%1").arg(enumError));
            return;
        }
        total_ = std::max(1, config_.maxFrames);
        cameras_.setCameraBPreviewCallback([this](const QImage& image) {
            if (onCameraPreview && !image.isNull())
                onCameraPreview(image);
        });
        QString openError;
        if (!cameras_.start(*a, *b, config_.cameraAExposure, config_.cameraBExposure,
                            config_.cameraABacklight, config_.cameraBBacklight, config_.cameraSyncToleranceMs,
                            config_.cameraQueueDepth, &openError)) {
            stopped_ = true;
            if (onError) onError(runEpoch_, openError);
            return;
        }
        lastDroppedReport_ = 0;
        if (onMessage) {
            onMessage(QString::fromUtf8("双相机采集已启动: A(码图)=%1, B(彩色)=%2, 同步阈值=%3 ms")
                      .arg(a->modelName, b->modelName).arg(config_.cameraSyncToleranceMs, 0, 'f', 2));
        }

        // Camera source is also self-driven and independent from RGBDFusion callbacks.
        QMetaObject::invokeMethod(this, [this, epoch = runEpoch_] {
            if (!stopped_ && epoch == runEpoch_)
                requestNext();
        }, Qt::QueuedConnection);
#else
        total_ = 0;
        stopped_ = true;
        if (onError) onError(runEpoch_, QString::fromUtf8("当前构建未启用 rulermvs/OpenCV 扫描模块"));
#endif
    }

    void requestNext() {
        if (stopped_) return;
#ifdef JMENGINE_HAS_RULERMVS
        if (index_ >= total_) {
            stopped_ = true;
            cameras_.stop();
            if (onDone) onDone(runEpoch_);
            return;
        }

        if (config_.sourceMode == ScanSourceMode::Virtual) {
            // Virtual acquisition emulates a 10 FPS camera and owns its own timing.
            // It is intentionally independent from SLAM callbacks/back-pressure.
            constexpr qint64 kVirtualFramePeriodMs = 100;
            const qint64 nowMs = virtualClock_.isValid() ? virtualClock_.elapsed() : 0;
            if (lastVirtualEmitMs_ >= 0) {
                const qint64 elapsedMs = nowMs - lastVirtualEmitMs_;
                if (elapsedMs < kVirtualFramePeriodMs) {
                    if (!virtualRequestScheduled_) {
                        virtualRequestScheduled_ = true;
                        const quint64 epoch = runEpoch_;
                        const int delayMs = int(kVirtualFramePeriodMs - elapsedMs);
                        QTimer::singleShot(delayMs, this, [this, epoch] {
                            virtualRequestScheduled_ = false;
                            if (!stopped_ && epoch == runEpoch_)
                                requestNext();
                        });
                    }
                    return;
                }
            }

            auto frame = std::make_shared<RawScanFrame>();
            frame->index = index_;
            frame->rgb = cv::imread(rgbNames_[index_], cv::IMREAD_COLOR);
            frame->code = cv::imread(codeNames_[index_], cv::IMREAD_GRAYSCALE);
            ++index_;
            if (frame->rgb.empty() || frame->code.empty()) {
                if (onError) onError(runEpoch_, QString::fromUtf8("读取虚拟扫描图像失败，frame=%1").arg(frame->index));
                return;
            }
            lastVirtualEmitMs_ = virtualClock_.elapsed();
            if (onFrame) onFrame(runEpoch_, std::move(frame));

            // Continue at 10 FPS independently. Do not wait for SLAM availability/result.
            if (!stopped_) {
                const quint64 epoch = runEpoch_;
                QTimer::singleShot(int(kVirtualFramePeriodMs), this, [this, epoch] {
                    if (!stopped_ && epoch == runEpoch_)
                        requestNext();
                });
            }
            return;
        }

        DualCameraCapture::Captured a, b;
        if (!cameras_.nextPair(a, b, 50)) {
            if (!stopped_)
                QMetaObject::invokeMethod(this, [this] { requestNext(); }, Qt::QueuedConnection);
            return;
        }

        const quint64 dropped = cameras_.droppedUnsynced();
        if (dropped >= lastDroppedReport_ + 10) {
            lastDroppedReport_ = dropped;
            if (onMessage) onMessage(QString::fromUtf8("A/B 不同步帧已丢弃: %1").arg(dropped));
        }

        auto frame = std::make_shared<RawScanFrame>();
        frame->index = index_++;
        frame->timestampAUs = a.timestampUs;
        frame->timestampBUs = b.timestampUs;
        if ((frame->index % 30) == 0) {
            qInfo().noquote() << QStringLiteral("[CAM PERF] frame=%1 syncDeltaMs=%2 droppedUnsynced=%3")
                .arg(frame->index)
                .arg(double(std::llabs(frame->timestampAUs - frame->timestampBUs)) / 1000.0, 0, 'f', 3)
                .arg(qulonglong(dropped));
        }

        // Production wiring: Camera A = structured-light code image, Camera B = color image.
        // Keep the algorithm-facing RawScanFrame identical to virtual mode: code=img/p, rgb=img/c.
        if (a.image.channels() == 1) frame->code = std::move(a.image);
        else if (a.image.channels() == 4) cv::cvtColor(a.image, frame->code, cv::COLOR_BGRA2GRAY);
        else cv::cvtColor(a.image, frame->code, cv::COLOR_BGR2GRAY);

        if (b.image.channels() == 1) cv::cvtColor(b.image, frame->rgb, cv::COLOR_GRAY2BGR);
        else if (b.image.channels() == 4) cv::cvtColor(b.image, frame->rgb, cv::COLOR_BGRA2BGR);
        else frame->rgb = std::move(b.image);

        // Camera B preview is emitted directly by Camera B's capture thread.
        // Do not tie UI video to synchronization success or SLAM consumption.
        if (onFrame) onFrame(runEpoch_, std::move(frame));

        // Production camera acquisition is a continuous producer. Keep acquiring even if
        // RGBDFusion is temporarily full; submitFrame() may drop that pair, but acquisition
        // and synchronization must never stall waiting for a trace callback.
        if (!stopped_)
            QMetaObject::invokeMethod(this, [this] { requestNext(); }, Qt::QueuedConnection);
#else
        if (onError) onError(runEpoch_, QString::fromUtf8("当前构建未启用 rulermvs/OpenCV 扫描模块"));
#endif
    }

    void setExposure(ScanCameraRole role, double value) {
        if (role == ScanCameraRole::CameraA) config_.cameraAExposure = value;
        else config_.cameraBExposure = value;
#ifdef JMENGINE_HAS_RULERMVS
        cameras_.setExposure(role == ScanCameraRole::CameraA, value);
#endif
    }

    void setBacklight(ScanCameraRole role, double value) {
        if (role == ScanCameraRole::CameraA) config_.cameraABacklight = value;
        else config_.cameraBBacklight = value;
#ifdef JMENGINE_HAS_RULERMVS
        cameras_.setBacklight(role == ScanCameraRole::CameraA, value);
#endif
    }

    void stop() {
        stopped_ = true;
#ifdef JMENGINE_HAS_RULERMVS
        cameras_.stop();
#endif
    }
    void reset() {
        stop();
        virtualRequestScheduled_ = false;
        lastVirtualEmitMs_ = -1;
        index_ = total_ = 0;
#ifdef JMENGINE_HAS_RULERMVS
        rgbNames_.clear();
        codeNames_.clear();
#endif
    }

  private:
    ScanConfig config_;
    int index_{0};
    int total_{0};
    bool stopped_{true};
    quint64 lastDroppedReport_{0};
    quint64 runEpoch_{0};
    QElapsedTimer virtualClock_;
    qint64 lastVirtualEmitMs_{-1};
    bool virtualRequestScheduled_{false};
#ifdef JMENGINE_HAS_RULERMVS
    std::vector<std::string> rgbNames_;
    std::vector<std::string> codeNames_;
    DualCameraCapture cameras_;
#endif
};

class ScanFlowController::RulerMvsWorker final : public QObject {
  public:
    using ReadyCallback = std::function<void(bool, const QString&)>;
    using PreviewCallback = std::function<void(PointChunkPtr)>;
    using CurrentFrameCallback = ScanFlowController::CurrentFrameCallback;
    using StoppedCallback = std::function<void()>;
    using ReconstructionCallback = std::function<void(CloudPtr)>;
    using ProgressCallback = std::function<void(int)>;
    using PoseCallback = ScanFlowController::PoseCallback;
    using LiveFrameCallback = ScanFlowController::LiveFrameCallback;
    using LivePoseUpdatesCallback = ScanFlowController::LivePoseUpdatesCallback;
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    using TextureFramesCallback = ScanFlowController::TextureFramesCallback;
#endif

    ReadyCallback onInitialized;
    PreviewCallback onPreview;
    CurrentFrameCallback onCurrentFrame;
    StoppedCallback onStopped;
    ReconstructionCallback onReconstructed;
    LiveFrameCallback onLiveFrame;
    LivePoseUpdatesCallback onLivePoseUpdates;
    ProgressCallback onReconstructProgress;
    MessageCallback onMessage;
    PoseCallback onPose;
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    TextureFramesCallback onTextureFrames;
#endif

    void initialize(const ScanConfig& config, quint64 runEpoch) {
        config_ = config;
#ifdef JMENGINE_HAS_RULERMVS
        activeEpoch_ = runEpoch;
        resetInternal();
        QString calibQ = config.calibrationPath.trimmed();
        if (calibQ.isEmpty() && !config.dataDir.trimmed().isEmpty())
            calibQ = config.dataDir + QStringLiteral("/calib.txt");
        const std::string calibPath = calibQ.toStdString();
        if (calibQ.isEmpty() || !fs::exists(calibPath)) {
            if (onInitialized) onInitialized(false, QString::fromUtf8("找不到标定文件: %1").arg(calibQ));
            return;
        }
        if (!QFileInfo::exists(config.vocabularyPath)) {
            if (onInitialized) onInitialized(false, QString::fromUtf8("找不到词典文件: %1").arg(config.vocabularyPath));
            return;
        }

        oneshot_ = rulermvs::IOneShot::create();
        if (!oneshot_ || oneshot_->init(calibPath)) {
            if (onInitialized) onInitialized(false, QString::fromUtf8("rulermvs 标定初始化失败"));
            return;
        }
        rulermvs::IOneShot::DevicePara devicePara;
        if (rulermvs::IOneShot::loadDeviceFile(calibPath, devicePara)) {
            baseRt_ = poseToCvMat(devicePara.colorRT);
            baseRtInv_ = baseRt_.inv();
        } else {
            baseRt_ = cv::Mat::eye(4, 4, CV_64F);
            baseRtInv_ = baseRt_.clone();
            if (onMessage) onMessage(QString::fromUtf8("警告: calib.txt 外参读取失败，实时姿态将使用单位基准"));
        }

        oneshot_->getColorCamera(cam_);
        rgbSize_ = {cam_.width, cam_.height};
        depthSize_ = rgbSize_ / scaleValue_;
        const double sx = double(depthSize_.width) / double(rgbSize_.width);
        const double sy = double(depthSize_.height) / double(rgbSize_.height);
        depthK_ = cv::Mat::eye(3, 3, CV_64F);
        depthK_.at<double>(0,0) = cam_.fx * sx;
        depthK_.at<double>(0,2) = cam_.cx * sx;
        depthK_.at<double>(1,1) = cam_.fy * sy;
        depthK_.at<double>(1,2) = cam_.cy * sy;
        rulermvs::createUndistorRectifyMap(cam_, {}, cam_.nodistor().noskew() / 4, mapX_, mapY_);
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
        textureScaleDivisor_ = 1; // Final texturing must use the original undistorted RGB resolution.
        textureSize_ = rgbSize_ / textureScaleDivisor_;
        textureK_ = cv::Mat::eye(3, 3, CV_64F);
        textureK_.at<double>(0, 0) = cam_.fx / double(textureScaleDivisor_);
        textureK_.at<double>(0, 2) = cam_.cx / double(textureScaleDivisor_);
        textureK_.at<double>(1, 1) = cam_.fy / double(textureScaleDivisor_);
        textureK_.at<double>(1, 2) = cam_.cy / double(textureScaleDivisor_);
        rulermvs::createUndistorRectifyMap(cam_, {}, cam_.nodistor().noskew() / textureScaleDivisor_,
                                           textureMapX_, textureMapY_);
#endif

        vocab_.load(config.vocabularyPath.toStdString());
        db_ = std::make_unique<DBoW3::Database>(vocab_, false, 0);
        std::vector<double> maxDists{3.0};
        std::vector<int> maxIters{5};
        fusion_ = std::make_unique<rgbdslam::RGBDFusion>(depthK_, vocab_, *db_, depthSize_.width, depthSize_.height,
                                                         maxDists.data(), maxIters.data(), int(maxDists.size()),
                                                         8, 8, true, true);
        auto& p = fusion_->para();
        p.is_use_dbow = true;
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

        accepting_.store(true);
        inputFinished_.store(false);
        submitted_.store(0);
        completed_.store(0);
        lastPublishedPoseByFrame_.clear();
        inflight_.store(0);
        fusion_->setTraceCallBack([this](const rgbdslam::IRGBDResult& result) { handleFusionResult(result); });
        fusion_->start();
        if (onInitialized) onInitialized(true, QString::fromUtf8("rulermvs 扫描流水线初始化完成"));
#else
        Q_UNUSED(runEpoch);
        if (onInitialized) onInitialized(false, QString::fromUtf8("当前构建未启用 rulermvs"));
#endif
    }

#ifdef JMENGINE_HAS_RULERMVS
    // Camera/source input is latest-frame-only when RGBDFusion is busy.  The input credit is
    // released by completion of the SDK decode callback, NOT by trace/result callbacks: LOST
    // tracking is therefore still allowed to receive fresh observations for relocalization.
    void offerLatestFrame(quint64 epoch, RawScanFramePtr frame) {
        if (epoch != activeEpoch_ || !fusion_ || !accepting_.load() || !frame) return;
        const int maxInflight = std::max(1, config_.maxInflightFrames);
        if (inflight_.load(std::memory_order_acquire) >= maxInflight) {
            pendingFrame_ = std::move(frame); // replace stale pending input with the newest pair
            const auto dropped = pendingReplaced_.fetch_add(1) + 1;
            if ((dropped % 30u) == 0u && onMessage)
                onMessage(QString::fromUtf8("SLAM 忙，已用最新帧替换等待帧 %1 次（inflight=%2）")
                          .arg(dropped).arg(inflight_.load()));
            return;
        }
        submitFrame(std::move(frame));
    }

    void trySubmitPendingFrame() {
        if (!fusion_ || !accepting_.load() || !pendingFrame_) return;
        const int maxInflight = std::max(1, config_.maxInflightFrames);
        if (inflight_.load(std::memory_order_acquire) >= maxInflight) return;
        auto newest = std::move(pendingFrame_);
        pendingFrame_.reset();
        submitFrame(std::move(newest));
    }

    void submitFrame(RawScanFramePtr frame) {
        if (!fusion_ || !accepting_.load() || !frame) return;

        // Input admission is bounded by decode work rather than SLAM trace callbacks.
        // This prevents an ever-growing RGBDFusion callback queue without blocking LOST recovery.
        if (frame->rgb.size() != rgbSize_ || frame->code.size() != rgbSize_) {
            if (onMessage) {
                onMessage(QString::fromUtf8("输入尺寸与标定不一致，丢弃 frame=%1: A=%2x%3 B=%4x%5 calib=%6x%7")
                          .arg(frame->index).arg(frame->rgb.cols).arg(frame->rgb.rows)
                          .arg(frame->code.cols).arg(frame->code.rows).arg(rgbSize_.width).arg(rgbSize_.height));
            }
            return;
        }

        rulermvs::IOneShot::DecodePara decode;
        decode.sigma = 2.0f;
        decode.darkness = 1.0f;
        decode.smoothX = 5;
        decode.smoothY = 5;
        decode.lineThreshold = 0.75f;
        decode.linkInterval = 20;
        decode.minGroup = 200;
        decode.bFuzzyDecode = true;

        const auto rgb = frame->rgb;
        const auto code = frame->code;
        const int frameIndex = frame->index;
        inflight_.fetch_add(1, std::memory_order_acq_rel);
        auto decodeFunc = [this, rgb, code, frameIndex, decode](int& userID, int64_t& nTime, cv::Mat& depth,
                                                                cv::Mat& color, cv::Mat& mask, cv::Mat& gray) mutable {
            Q_UNUSED(mask);
            QElapsedTimer perf;
            perf.start();
            userID = frameIndex;
            nTime = fusion_->getCurrentTime();
            rulermvs::Image8u codeImage;
            rulermvs::convertTo(code, codeImage);
            const qint64 tConvert = perf.nsecsElapsed();
            rulermvs::Imagef depthImage;
            rulermvs::SimpleTriMesh mesh;
            oneshot_->decode(codeImage, mesh, decode);
            const qint64 tDecode = perf.nsecsElapsed();
            rulermvs::rasterDepth(mesh, cam_.nodistor().noskew() / scaleValue_, depthImage);
            const qint64 tRaster = perf.nsecsElapsed();
            depth = depthImage.to<cv::Mat>().clone();
            const qint64 tDepthClone = perf.nsecsElapsed();
            cv::Mat mapX = mapX_.to<cv::Mat>();
            cv::Mat mapY = mapY_.to<cv::Mat>();
            cv::remap(rgb, color, mapX, mapY, cv::INTER_LINEAR);
            const qint64 tRemap = perf.nsecsElapsed();
            cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
            const qint64 tGray = perf.nsecsElapsed();
            cv::resize(color, color, depthSize_);
            const qint64 tResize = perf.nsecsElapsed();
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
            const int textureStride = std::max(1, config_.textureKeyframeStride);
            if ((frameIndex % textureStride) == 0) {
                cv::Mat textureColor;
                cv::Mat texMapX = textureMapX_.to<cv::Mat>();
                cv::Mat texMapY = textureMapY_.to<cv::Mat>();
                cv::remap(rgb, textureColor, texMapX, texMapY, cv::INTER_LINEAR);
                std::lock_guard<std::mutex> lock(textureFrameMutex_);
                const std::size_t hardLimit = static_cast<std::size_t>(std::max(1, config_.textureMaxKeyframes)) * 2u;
                if (textureImagesByFrame_.size() < hardLimit || textureImagesByFrame_.count(frameIndex) != 0u)
                    textureImagesByFrame_[frameIndex] = std::move(textureColor);
            }
#endif

            const double nsToMs = 1.0 / 1000000.0;
            const double totalMs = double(tResize) * nsToMs;
            if ((frameIndex % 30) == 0 || totalMs > 50.0) {
                qInfo().noquote() << QStringLiteral(
                    "[DECODE PERF] frame=%1 total=%2ms convert=%3 decode=%4 raster=%5 depthClone=%6 remap=%7 gray=%8 resize=%9 inflight=%10")
                    .arg(frameIndex)
                    .arg(totalMs, 0, 'f', 2)
                    .arg(double(tConvert) * nsToMs, 0, 'f', 2)
                    .arg(double(tDecode - tConvert) * nsToMs, 0, 'f', 2)
                    .arg(double(tRaster - tDecode) * nsToMs, 0, 'f', 2)
                    .arg(double(tDepthClone - tRaster) * nsToMs, 0, 'f', 2)
                    .arg(double(tRemap - tDepthClone) * nsToMs, 0, 'f', 2)
                    .arg(double(tGray - tRemap) * nsToMs, 0, 'f', 2)
                    .arg(double(tResize - tGray) * nsToMs, 0, 'f', 2)
                    .arg(inflight_.load(std::memory_order_relaxed));
            }

            // Release one input credit as soon as this expensive decode callback finishes.
            // Schedule admission on pipelineThread_ so pendingFrame_ remains single-thread owned.
            inflight_.fetch_sub(1, std::memory_order_acq_rel);
            QMetaObject::invokeMethod(this, [this] { trySubmitPendingFrame(); }, Qt::QueuedConnection);
        };

        submitted_.fetch_add(1);
        fusion_->addFrameInCallBack(std::move(decodeFunc));
    }
#endif

    void finishInput() {
#ifdef JMENGINE_HAS_RULERMVS
        accepting_.store(false);
        inputFinished_.store(true);
        pendingFrame_.reset();

        // Do NOT wait for inflight_ == 0 here. addFrameInCallBack() does not guarantee that
        // every accepted input produces a trace callback, so callback-count based draining can
        // wait forever. SlameProducer also stops RGBDFusion directly before optimization.
        // This method already runs on pipelineThread_, therefore the blocking SDK stop never
        // blocks the UI thread. RGBDFusion::stop() is the authoritative queue-drain boundary.
        if (stopped_.exchange(true)) return;
        if (onMessage)
            onMessage(QString::fromUtf8("正在停止 SLAM（SDK 内部排空），submitted=%1 traceResults=%2")
                      .arg(submitted_.load()).arg(completed_.load()));
        if (fusion_) fusion_->stop();
        inflight_.store(0);
        if (onStopped) onStopped();
#else
        if (onStopped) onStopped();
#endif
    }

    void offlineReconstruct() {
#ifdef JMENGINE_HAS_RULERMVS
        if (!fusion_) {
            if (onMessage) onMessage(QString::fromUtf8("没有可用于离线重建的扫描数据"));
            return;
        }
        fusion_->optimizePointMap(config_.offlineVoxel, config_.offlineIterations,
                                  [this](int progress, bool& stop) {
                                      stop = false;
                                      if (onReconstructProgress) onReconstructProgress(progress);
                                  });
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
        if (onTextureFrames) {
            std::unordered_map<int, cv::Mat> finalWorldFromCamera;
            fusion_->getResults([&](const rgbdslam::IRGBDResult& r) {
                if (r.getFlag() != 0) return;
                cv::Mat pose = r.getRT();
                if (!baseRtInv_.empty()) pose = baseRtInv_ * pose;
                finalWorldFromCamera[r.getFrameID()] = pose.clone();
            });

            std::vector<std::pair<int, cv::Mat>> savedImages;
            {
                std::lock_guard<std::mutex> lock(textureFrameMutex_);
                savedImages.reserve(textureImagesByFrame_.size());
                for (const auto& kv : textureImagesByFrame_) savedImages.push_back({kv.first, kv.second});
            }
            std::sort(savedImages.begin(), savedImages.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
            const std::size_t maxFrames = static_cast<std::size_t>(std::max(1, config_.textureMaxKeyframes));
            auto frames = std::make_shared<ScanFlowController::TextureFrames>();
            frames->reserve(std::min(maxFrames, savedImages.size()));
            const std::size_t step = savedImages.size() > maxFrames ?
                static_cast<std::size_t>(std::ceil(double(savedImages.size()) / double(maxFrames))) : 1u;
            for (std::size_t i = 0; i < savedImages.size(); i += step) {
                const int frameId = savedImages[i].first;
                const auto pit = finalWorldFromCamera.find(frameId);
                if (pit == finalWorldFromCamera.end()) continue;
                JMEngine::texture::CameraFrame tf;
                tf.frameId = frameId;
                tf.image = bgrToTextureRgb(savedImages[i].second);
                tf.fx = static_cast<float>(textureK_.at<double>(0, 0));
                tf.fy = static_cast<float>(textureK_.at<double>(1, 1));
                tf.cx = static_cast<float>(textureK_.at<double>(0, 2));
                tf.cy = static_cast<float>(textureK_.at<double>(1, 2));
                const cv::Mat worldToCamera = pit->second.inv();
                tf.worldToCamera.m = cvPoseToColumnMajor(worldToCamera);
                if (tf.image.valid()) frames->push_back(std::move(tf));
                if (frames->size() >= maxFrames) break;
            }
            savedImages.clear();
            {
                std::lock_guard<std::mutex> lock(textureFrameMutex_);
                textureImagesByFrame_.clear();
                textureImagesByFrame_.rehash(0);
            }
            if (onMessage) onMessage(QString::fromUtf8("纹理关键帧准备完成: %1").arg(frames->size()));
            onTextureFrames(std::move(frames));
        }
#endif
        std::vector<cv::Point3f> pts, nls;
        std::vector<cv::Vec3b> rgbs;
        fusion_->fusePoints(pts, nls, rgbs);
        JMEngine::PointCloud::Container out;
        out.reserve(pts.size());
        for (std::size_t i = 0; i < pts.size(); ++i) {
            JMEngine::Point p;
            p.position = {pts[i].x, pts[i].y, pts[i].z};
            if (i < nls.size()) p.normal = {nls[i].x, nls[i].y, nls[i].z};
            if (i < rgbs.size()) p.rgba = packBgr(rgbs[i]);
            out.push_back(p);
        }
        if (onReconstructed) onReconstructed(std::make_shared<JMEngine::PointCloud>(std::move(out)));
#else
        if (onMessage) onMessage(QString::fromUtf8("当前构建未启用 rulermvs"));
#endif
    }

    void reset() {
#ifdef JMENGINE_HAS_RULERMVS
        resetInternal();
#endif
    }

  private:
#ifdef JMENGINE_HAS_RULERMVS
    void handleFusionResult(const rgbdslam::IRGBDResult& result) {
        // Result callbacks are OUTPUT ONLY. They never release/enable input submission.
        // Some accepted frames may not produce a trace callback, especially while tracking is lost.
        const int completedNow = completed_.fetch_add(1) + 1;
        if ((completedNow % 30) == 0) {
            qInfo().noquote() << QStringLiteral("[SLAM PERF] results=%1 submitted=%2 inflight=%3 pendingReplaced=%4 frameId=%5 flag=%6")
                .arg(completedNow)
                .arg(submitted_.load(std::memory_order_relaxed))
                .arg(inflight_.load(std::memory_order_relaxed))
                .arg(qulonglong(pendingReplaced_.load(std::memory_order_relaxed)))
                .arg(result.getFrameID())
                .arg(result.getFlag());
        }

        // 保持与当前项目/用户提供的 SlameProducer 实时回调语义一致：
        // result.getFlag() == 0 表示当前帧可用于实时累计显示。
        // 上一版擅自按 SDK 枚举名重新解释 flag，导致正常帧全部被过滤，Live 完全不显示。
        // 这里不再改变既有业务语义；后续若要重新定义 flag，必须以实际 SDK 运行结果验证。
        const auto flag = result.getFlag();
        const bool trackingOk = (flag == 0);

        // OpenCV 相机约定：+X 向右、+Y 向下、+Z 向前，因此 3D 视图 Up 使用局部 -Y。
        cv::Mat measuredPose = result.getRT();
        if (!baseRtInv_.empty()) measuredPose = baseRtInv_ * measuredPose;

        // A failed tracker pose is not trustworthy. For recovery comparison, render the
        // CURRENT lost local cloud using the last valid world pose. Then the yellow last-valid
        // frame and green current-lost frame share the same reference frame; moving the scanner
        // back to the previous view makes the two clouds visibly overlap.
        cv::Mat framePose = measuredPose;
        if (trackingOk)
            lastValidFramePose_ = measuredPose.clone();
        else if (!lastValidFramePose_.empty())
            framePose = lastValidFramePose_;

        if (onPose) {
            const auto pos = transformPoint(framePose, cv::Point3f(0.0f, 0.0f, 0.0f));
            const auto right = transformDirection(framePose, cv::Point3f(1.0f, 0.0f, 0.0f));
            const auto up = transformDirection(framePose, cv::Point3f(0.0f, -1.0f, 0.0f));
            const auto forward = transformDirection(framePose, cv::Point3f(0.0f, 0.0f, 1.0f));
            ScanPoseState pose;
            pose.position = {pos.x, pos.y, pos.z};
            pose.right = {right.x, right.y, right.z};
            pose.up = {up.x, up.y, up.z};
            pose.forward = {forward.x, forward.y, forward.z};
            pose.trackingOk = trackingOk;
            pose.frameId = result.getFrameID();
            onPose(pose);
        }

        if (onPreview || onCurrentFrame || onLiveFrame) {
            std::vector<cv::Point3f> pts, nls;
            std::vector<cv::Vec3b> colors;
            result.toCloud(pts, nls, colors);
            // Keep the total Live history bounded across the full configured scan instead of
            // spending the whole budget on the first few dozen frames.
            const int budgetPerFrame = std::max(250, config_.previewPointLimit / std::max(1, config_.maxFrames));
            const int limit = std::max(1, std::min(config_.previewPointsPerFrame, budgetPerFrame));
            const std::size_t stride = std::max<std::size_t>(1, pts.size() / std::size_t(limit));
            auto localChunk = std::make_shared<PointChunk>();
            auto worldChunk = std::make_shared<PointChunk>();
            localChunk->reserve(std::min<std::size_t>(pts.size(), std::size_t(limit)));
            worldChunk->reserve(localChunk->capacity());
            for (std::size_t i = 0; i < pts.size(); i += stride) {
                JMEngine::Point local;
                local.position = {pts[i].x, pts[i].y, pts[i].z};
                if (i < nls.size()) local.normal = {nls[i].x, nls[i].y, nls[i].z};
                if (i < colors.size()) local.rgba = packBgr(colors[i]);
                localChunk->push_back(local);

                JMEngine::Point world = local;
                world.position = transformPoint(framePose, pts[i]);
                if (i < nls.size()) world.normal = transformNormal(framePose, nls[i]);
                worldChunk->push_back(world);
                if (int(localChunk->size()) >= limit) break;
            }

            if (trackingOk && onLiveFrame && !localChunk->empty()) {
                const auto poseArray = cvPoseToColumnMajor(measuredPose);
                lastPublishedPoseByFrame_[result.getFrameID()] = poseArray;
                onLiveFrame(localChunk, poseArray, result.getFrameID());
            }

            // Current/recovery overlays remain in world coordinates and are independent from
            // the persistent local-coordinate history VBO.
            if (onCurrentFrame && !worldChunk->empty())
                onCurrentFrame(worldChunk, trackingOk, result.getFrameID());

            if (!onCurrentFrame && trackingOk && onPreview && !worldChunk->empty())
                onPreview(std::move(worldChunk));
        }

        // Periodically refresh historical frames from RGBDFusion's CURRENT optimized RTs.
        // Queue it onto pipelineThread_ rather than doing getResults() inside the trace callback.
        // At most one refresh can be pending, preventing optimization display work from piling up.
        if (config_.liveOptimizationEnabled && trackingOk &&
            (completed_.load() % livePoseRefreshInterval_ == 0) &&
            !liveRefreshPending_.exchange(true)) {
            QMetaObject::invokeMethod(this, [this] { refreshLiveOptimizedPreview(); }, Qt::QueuedConnection);
        }

    }

    void refreshLiveOptimizedPreview() {
        liveRefreshPending_.store(false);
        if (!config_.liveOptimizationEnabled || !fusion_ || !accepting_.load() || !onLivePoseUpdates)
            return;

        auto updates = std::make_shared<std::vector<LiveFramePoseUpdate>>();
        int rejected = 0;
        fusion_->getResults([&](const rgbdslam::IRGBDResult& r) {
            if (r.getFlag() != 0) return;
            cv::Mat pose = r.getRT();
            if (!baseRtInv_.empty()) pose = baseRtInv_ * pose;
            const std::array<float, 16> next = cvPoseToColumnMajor(pose);
            auto it = lastPublishedPoseByFrame_.find(r.getFrameID());
            if (it == lastPublishedPoseByFrame_.end())
                return; // Live history has not published this frame yet.
            if (!poseArrayChanged(it->second, next))
                return;
            if (!poseUpdateLooksReasonable(it->second, next)) {
                ++rejected;
                return;
            }

            it->second = next;
            LiveFramePoseUpdate update;
            update.frameId = r.getFrameID();
            update.pose = next;
            updates->push_back(std::move(update));
        });
        if (!updates->empty() && onLivePoseUpdates)
            onLivePoseUpdates(std::move(updates));
        if (rejected > 0 && onMessage)
            onMessage(QString::fromUtf8("Live实时优化过滤了 %1 个异常姿态更新").arg(rejected));
    }

    void resetInternal() {
        accepting_.store(false);
        inputFinished_.store(false);
        if (fusion_) fusion_->stop();
        fusion_.reset();
        db_.reset();
        oneshot_ = nullptr;
        stopped_.store(false);
        inflight_.store(0);
        pendingFrame_.reset();
        pendingReplaced_.store(0);
        submitted_.store(0);
        completed_.store(0);
        lastPublishedPoseByFrame_.clear();
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
        {
            std::lock_guard<std::mutex> lock(textureFrameMutex_);
            textureImagesByFrame_.clear();
        }
#endif
        lastValidFramePose_.release();
        liveRefreshPending_.store(false);
    }

    ScanConfig config_;
    rulermvs::CameraSkewPB cam_;
    rulermvs::Imagef mapX_, mapY_;
    cv::Size depthSize_, rgbSize_;
    cv::Mat depthK_;
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    rulermvs::Imagef textureMapX_, textureMapY_;
    cv::Size textureSize_;
    cv::Mat textureK_;
    int textureScaleDivisor_{2};
#endif
    cv::Mat baseRt_;
    cv::Mat baseRtInv_;
    cv::Mat lastValidFramePose_;
    rulermvs::IOneShot::Ptr oneshot_{nullptr};
    DBoW3::Vocabulary vocab_;
    std::unique_ptr<DBoW3::Database> db_;
    std::unique_ptr<rgbdslam::RGBDFusion> fusion_;
    int scaleValue_{16};
    quint64 activeEpoch_{0};
    RawScanFramePtr pendingFrame_;
    std::atomic<unsigned long long> pendingReplaced_{0};
    std::atomic<int> inflight_{0};
    std::atomic<int> submitted_{0};
    std::atomic<int> completed_{0};
    std::atomic<bool> accepting_{false};
    std::atomic<bool> inputFinished_{false};
    std::atomic<bool> stopped_{false};
    std::atomic<bool> liveRefreshPending_{false};
    std::unordered_map<int, std::array<float,16>> lastPublishedPoseByFrame_;
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    std::mutex textureFrameMutex_;
    std::unordered_map<int, cv::Mat> textureImagesByFrame_;
#endif
    int livePoseRefreshInterval_{30};
#else
    ScanConfig config_;
#endif
};

ScanFlowController::ScanFlowController(QObject* parent) : QObject(parent) {
    source_ = new ScanSourceWorker();
    pipeline_ = new RulerMvsWorker();
    source_->moveToThread(&sourceThread_);
    pipeline_->moveToThread(&pipelineThread_);

    QPointer<ScanFlowController> self(this);
    source_->onFrame = [self](quint64 epoch, auto frame) {
        if (!self || !frame) return;
#ifdef JMENGINE_HAS_RULERMVS
        // Raw scan frames go straight SourceThread -> PipelineThread.  Do not route 10 FPS
        // image traffic through the GUI event queue; the pipeline validates runEpoch itself.
        QMetaObject::invokeMethod(self->pipeline_,
                                  [p = self->pipeline_, epoch, frame = std::move(frame)]() mutable {
                                      p->offerLatestFrame(epoch, std::move(frame));
                                  },
                                  Qt::QueuedConnection);
#else
        Q_UNUSED(epoch);
#endif
    };
    source_->onDone = [self](quint64 epoch) {
        if (!self) return;
        QMetaObject::invokeMethod(self, [self, epoch] {
            if (self && epoch == self->runEpoch_) self->beginStopping(true);
        }, Qt::QueuedConnection);
    };
    source_->onError = [self](quint64 epoch, const QString& error) {
        if (!self) return;
        QMetaObject::invokeMethod(self, [self, epoch, error] {
            if (!self || epoch != self->runEpoch_) return;
            self->postMessage(error);
            self->terminalError_ = true;
            QMetaObject::invokeMethod(self->source_, [s = self->source_] { s->stop(); }, Qt::QueuedConnection);
            QMetaObject::invokeMethod(self->pipeline_, [p = self->pipeline_] { p->finishInput(); }, Qt::QueuedConnection);
            self->setState(State::Error);
        }, Qt::QueuedConnection);
    };
    source_->onMessage = [self](const QString& message) {
        if (!self) return;
        QMetaObject::invokeMethod(self, [self, message] { if (self) self->postMessage(message); }, Qt::QueuedConnection);
    };
    source_->onCameraList = [self](const QVector<CameraDeviceInfo>& list, const QString& error) {
        if (!self) return;
        QMetaObject::invokeMethod(self, [self, list, error] {
            if (self && self->cameraListCallback_) self->cameraListCallback_(list, error);
        }, Qt::QueuedConnection);
    };
    source_->onCameraPreview = [self](const QImage& image) {
        if (!self || image.isNull()) return;
        {
            std::lock_guard<std::mutex> lock(self->cameraPreviewMutex_);
            self->latestCameraPreview_ = image;
        }
        // Keep at most one queued UI preview delivery. New camera frames overwrite the
        // pending image so the operator always sees the newest frame instead of an old queue.
        if (self->cameraPreviewDispatchPending_.exchange(true)) return;
        QMetaObject::invokeMethod(self, [self] {
            if (!self) return;
            QImage latest;
            {
                std::lock_guard<std::mutex> lock(self->cameraPreviewMutex_);
                latest = self->latestCameraPreview_;
            }
            self->cameraPreviewDispatchPending_.store(false);
            if (self->cameraPreviewCallback_ && !latest.isNull())
                self->cameraPreviewCallback_(latest);
        }, Qt::QueuedConnection);
    };

    pipeline_->onInitialized = [self](bool ok, const QString& message) {
        if (!self) return;
        QMetaObject::invokeMethod(self, [self, ok, message] {
            if (!self) return;
            self->postMessage(message);
            if (!ok) { self->setState(State::Error); return; }
            self->sourceExhausted_ = false;
            self->terminalError_ = false;
            self->setState(State::Scanning);
            // Source starts itself. No SLAM callback/requestNext dependency remains.
            QMetaObject::invokeMethod(self->source_, [s = self->source_, cfg = self->config_, epoch = self->runEpoch_] {
                s->configure(cfg, epoch);
            }, Qt::QueuedConnection);
        }, Qt::QueuedConnection);
    };
    pipeline_->onPreview = [self](PointChunkPtr chunk) {
        if (!self || !chunk) return;
        {
            std::lock_guard<std::mutex> lock(self->renderMailboxMutex_);
            self->latestPreviewChunk_ = std::move(chunk); // transient preview: newest wins
        }
        self->scheduleRenderDispatch();
    };
    pipeline_->onLiveFrame = [self](PointChunkPtr points, const std::array<float,16>& pose, int frameId) {
        if (!self || !points) return;
        {
            std::lock_guard<std::mutex> lock(self->renderMailboxMutex_);
            // Persistent history is never replaced: accepted SLAM frames are batched into one GUI wake-up.
            self->pendingLiveFrames_.push_back({std::move(points), pose, frameId});
        }
        self->scheduleRenderDispatch();
    };
    pipeline_->onLivePoseUpdates = [self](std::shared_ptr<std::vector<LiveFramePoseUpdate>> updates) {
        if (!self || !updates) return;
        {
            std::lock_guard<std::mutex> lock(self->renderMailboxMutex_);
            for (const auto& u : *updates)
                self->pendingPoseUpdates_[u.frameId] = u.pose; // only latest RT for each frame matters
        }
        self->scheduleRenderDispatch();
    };
    pipeline_->onCurrentFrame = [self](PointChunkPtr chunk, bool trackingOk, int frameId) {
        if (!self || !chunk) return;
        {
            std::lock_guard<std::mutex> lock(self->renderMailboxMutex_);
            self->latestCurrentFrame_ = std::move(chunk);
            self->latestCurrentTrackingOk_ = trackingOk;
            self->latestCurrentFrameId_ = frameId;
            self->hasCurrentFrame_ = true;
        }
        self->scheduleRenderDispatch();
    };
    pipeline_->onPose = [self](const ScanPoseState& pose) {
        if (!self) return;
        {
            std::lock_guard<std::mutex> lock(self->renderMailboxMutex_);
            self->latestPose_ = pose;
            self->hasPose_ = true;
        }
        self->scheduleRenderDispatch();
    };
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    pipeline_->onTextureFrames = [self](ScanFlowController::TextureFramesPtr frames) {
        if (!self) return;
        QMetaObject::invokeMethod(self, [self, frames = std::move(frames)]() mutable {
            if (self && self->textureFramesCallback_) self->textureFramesCallback_(std::move(frames));
        }, Qt::QueuedConnection);
    };
#endif
    pipeline_->onStopped = [self] {
        if (!self) return;
        QMetaObject::invokeMethod(self, [self] {
            if (!self) return;
            if (self->terminalError_) self->setState(State::Error);
            else {
                self->setState(State::ReadyForReconstruction);
                self->postMessage(QString::fromUtf8("扫描已结束，流水线已排空，可执行离线重建"));
            }
        }, Qt::QueuedConnection);
    };
    pipeline_->onReconstructProgress = [self](int p) {
        if (!self) return;
        QMetaObject::invokeMethod(self, [self, p] { if (self) self->postMessage(QString::fromUtf8("离线重建 %1%").arg(p)); }, Qt::QueuedConnection);
    };
    pipeline_->onReconstructed = [self](CloudPtr cloud) {
        if (!self) return;
        QMetaObject::invokeMethod(self, [self, cloud = std::move(cloud)]() mutable {
            if (!self) return;
            if (self->reconstructionCallback_) self->reconstructionCallback_(std::move(cloud));
            self->setState(State::ReadyForReconstruction);
            self->postMessage(QString::fromUtf8("离线重建完成"));
        }, Qt::QueuedConnection);
    };
    pipeline_->onMessage = [self](const QString& msg) {
        if (!self) return;
        QMetaObject::invokeMethod(self, [self, msg] { if (self) self->postMessage(msg); }, Qt::QueuedConnection);
    };

    sourceThread_.setObjectName(QStringLiteral("ScanSource"));
    pipelineThread_.setObjectName(QStringLiteral("RulerMvsPipeline"));
    sourceThread_.start();
    pipelineThread_.start();
}

ScanFlowController::~ScanFlowController() { shutdownThreads(); }

void ScanFlowController::setConfig(const ScanConfig& config) { config_ = config; }
ScanConfig ScanFlowController::config() const { return config_; }

void ScanFlowController::refreshCameras() {
    const QString jsonPath = config_.cameraModelJsonPath;
    QMetaObject::invokeMethod(source_, [s = source_, jsonPath] { s->discoverCameras(jsonPath); }, Qt::QueuedConnection);
}

void ScanFlowController::setCameraExposure(ScanCameraRole role, double value) {
    if (role == ScanCameraRole::CameraA) config_.cameraAExposure = value;
    else config_.cameraBExposure = value;
    QMetaObject::invokeMethod(source_, [s = source_, role, value] { s->setExposure(role, value); }, Qt::QueuedConnection);
}

void ScanFlowController::setCameraBacklight(ScanCameraRole role, double value) {
    if (role == ScanCameraRole::CameraA) config_.cameraABacklight = value;
    else config_.cameraBBacklight = value;
    QMetaObject::invokeMethod(source_, [s = source_, role, value] { s->setBacklight(role, value); }, Qt::QueuedConnection);
}

void ScanFlowController::startScan() {
    if (state_ == State::Scanning || state_ == State::Initializing || state_ == State::Stopping || state_ == State::Reconstructing)
        return;
    // A new epoch invalidates delayed source callbacks from any previous/reset run.
    ++runEpoch_;

    // Default vocabulary: first *.yml.gz in the executable directory.
    if (config_.vocabularyPath.trimmed().isEmpty()) {
        QDir appDir(QCoreApplication::applicationDirPath());
        const QStringList files = appDir.entryList(QStringList() << QStringLiteral("*.yml.gz"), QDir::Files, QDir::Name);
        if (!files.isEmpty())
            config_.vocabularyPath = appDir.filePath(files.first());
    }
    if (config_.vocabularyPath.trimmed().isEmpty()) {
        postMessage(QString::fromUtf8("运行目录未找到 *.yml.gz Vocabulary"));
        setState(State::Error);
        return;
    }
    if (config_.calibrationPath.trimmed().isEmpty() && config_.dataDir.trimmed().isEmpty()) {
        postMessage(QString::fromUtf8("请配置标定文件 calib.txt"));
        setState(State::Error);
        return;
    }
    if (config_.sourceMode == ScanSourceMode::Virtual && config_.dataDir.trimmed().isEmpty()) {
        postMessage(QString::fromUtf8("虚拟采集模式需要配置虚拟扫描数据目录"));
        setState(State::Error);
        return;
    }
    if (config_.sourceMode == ScanSourceMode::Camera) {
        if (config_.cameraModelJsonPath.trimmed().isEmpty() ||
            config_.cameraADeviceId.trimmed().isEmpty() || config_.cameraBDeviceId.trimmed().isEmpty() ||
            config_.cameraADeviceId == config_.cameraBDeviceId) {
            postMessage(QString::fromUtf8("相机采集模式需要型号 JSON，并分别选择相机 A/B"));
            setState(State::Error);
            return;
        }
    }
    terminalError_ = false;
    setState(State::Initializing);
    QMetaObject::invokeMethod(pipeline_, [p = pipeline_, cfg = config_, epoch = runEpoch_] { p->initialize(cfg, epoch); }, Qt::QueuedConnection);
}

void ScanFlowController::stopScan() {
    if (state_ != State::Scanning) return;
    beginStopping(false);
}

void ScanFlowController::beginStopping(bool sourceExhausted) {
    if (state_ != State::Scanning) return;
    sourceExhausted_ = sourceExhausted;
    setState(State::Stopping);
    QMetaObject::invokeMethod(source_, [s = source_] { s->stop(); }, Qt::QueuedConnection);
    QMetaObject::invokeMethod(pipeline_, [p = pipeline_] { p->finishInput(); }, Qt::QueuedConnection);
    postMessage(sourceExhausted ? QString::fromUtf8("采集数据已提交完，正在排空 SLAM 队列…")
                                : QString::fromUtf8("正在结束采集并排空 SLAM 队列…"));
}

void ScanFlowController::offlineReconstruct() {
    if (state_ != State::ReadyForReconstruction) return;
    setState(State::Reconstructing);
    QMetaObject::invokeMethod(pipeline_, [p = pipeline_] { p->offlineReconstruct(); }, Qt::QueuedConnection);
}

void ScanFlowController::reset() {
    // Invalidate already-posted onFrame/onDone/onError callbacks before resetting workers.
    ++runEpoch_;
    {
        std::lock_guard<std::mutex> lock(renderMailboxMutex_);
        pendingLiveFrames_.clear();
        latestPreviewChunk_.reset();
        latestCurrentFrame_.reset();
        hasCurrentFrame_ = false;
        hasPose_ = false;
        pendingPoseUpdates_.clear();
    }
    if (state_ == State::Scanning || state_ == State::Stopping || state_ == State::Initializing)
        QMetaObject::invokeMethod(source_, [s = source_] { s->stop(); }, Qt::QueuedConnection);
    setState(State::Idle);
    QMetaObject::invokeMethod(source_, [s = source_] { s->reset(); }, Qt::QueuedConnection);
    QMetaObject::invokeMethod(pipeline_, [p = pipeline_] { p->reset(); }, Qt::QueuedConnection);
    postMessage(QString::fromUtf8("扫描流程已重置"));
}

void ScanFlowController::requestNextFrame() {
    // Kept for source compatibility only. Normal scan acquisition is now self-driven
    // and must not be triggered by RGBDFusion callbacks.
    if (state_ != State::Scanning) return;
    QMetaObject::invokeMethod(source_, [s = source_] { s->requestNext(); }, Qt::QueuedConnection);
}

void ScanFlowController::setState(State state) {
    if (state_ == state) return;
    state_ = state;
    if (stateCallback_) stateCallback_(state_);
}

void ScanFlowController::postMessage(const QString& message) {
    if (messageCallback_) messageCallback_(message);
}

void ScanFlowController::scheduleRenderDispatch() {
    // One queued GUI event is enough. Worker callbacks keep filling/replacing the mailbox while
    // the UI is busy; this prevents a 10 FPS scan + optimization burst from becoming hundreds
    // of stale queued invokeMethod calls.
    if (renderDispatchPending_.exchange(true, std::memory_order_acq_rel)) return;
    QPointer<ScanFlowController> self(this);
    QMetaObject::invokeMethod(this, [self] {
        if (self) self->flushRenderMailbox();
    }, Qt::QueuedConnection);
}

void ScanFlowController::flushRenderMailbox() {
    std::deque<PendingLiveFrame> liveFrames;
    PointChunkPtr preview;
    PointChunkPtr current;
    bool currentTrackingOk = false;
    int currentFrameId = -1;
    bool hasCurrent = false;
    ScanPoseState pose;
    bool hasPose = false;
    std::unordered_map<int, std::array<float,16>> poseMap;
    {
        std::lock_guard<std::mutex> lock(renderMailboxMutex_);
        liveFrames.swap(pendingLiveFrames_);
        preview = std::move(latestPreviewChunk_);
        current = std::move(latestCurrentFrame_);
        currentTrackingOk = latestCurrentTrackingOk_;
        currentFrameId = latestCurrentFrameId_;
        hasCurrent = hasCurrentFrame_;
        hasCurrentFrame_ = false;
        hasPose = hasPose_;
        pose = latestPose_;
        hasPose_ = false;
        poseMap.swap(pendingPoseUpdates_);
        renderDispatchPending_.store(false, std::memory_order_release);
    }

    if (state_ != State::Idle && previewCallback_ && preview)
        previewCallback_(std::move(preview));

    // Append accepted history frames in order. PointCloudWidget uploads each local point block once;
    // future optimization changes only its RT, so no whole-cloud VBO rebuild is introduced here.
    if (state_ == State::Scanning && liveFrameCallback_) {
        for (auto& f : liveFrames)
            if (f.points) liveFrameCallback_(std::move(f.points), f.pose, f.frameId);
    }

    if (state_ == State::Scanning && livePoseUpdatesCallback_ && !poseMap.empty()) {
        auto updates = std::make_shared<std::vector<LiveFramePoseUpdate>>();
        updates->reserve(poseMap.size());
        for (auto& kv : poseMap) updates->push_back({kv.first, kv.second});
        livePoseUpdatesCallback_(std::move(updates));
    }

    if (state_ == State::Scanning && currentFrameCallback_ && hasCurrent && current)
        currentFrameCallback_(std::move(current), currentTrackingOk, currentFrameId);
    if (state_ != State::Idle && poseCallback_ && hasPose)
        poseCallback_(pose);

    // A producer may have filled the mailbox after we released the lock but before callbacks
    // completed. Schedule exactly one more pass if anything arrived meanwhile.
    bool more = false;
    {
        std::lock_guard<std::mutex> lock(renderMailboxMutex_);
        more = !pendingLiveFrames_.empty() || latestPreviewChunk_ || hasCurrentFrame_ || hasPose_ || !pendingPoseUpdates_.empty();
    }
    if (more) scheduleRenderDispatch();
}

QString ScanFlowController::stateText(State state) {
    switch (state) {
    case State::Idle: return QString::fromUtf8("空闲");
    case State::Initializing: return QString::fromUtf8("初始化中");
    case State::Scanning: return QString::fromUtf8("扫描中");
    case State::Stopping: return QString::fromUtf8("结束中");
    case State::ReadyForReconstruction: return QString::fromUtf8("待离线重建");
    case State::Reconstructing: return QString::fromUtf8("离线重建中");
    case State::Error: return QString::fromUtf8("错误");
    }
    return {};
}

void ScanFlowController::shutdownThreads() {
    if (source_) {
        auto* s = source_;
        source_ = nullptr;
        QMetaObject::invokeMethod(s, [s] { s->stop(); delete s; }, Qt::BlockingQueuedConnection);
    }
    if (pipeline_) {
        auto* p = pipeline_;
        pipeline_ = nullptr;
        QMetaObject::invokeMethod(p, [p] { p->reset(); delete p; }, Qt::BlockingQueuedConnection);
    }
    sourceThread_.quit();
    pipelineThread_.quit();
    sourceThread_.wait();
    pipelineThread_.wait();
}
