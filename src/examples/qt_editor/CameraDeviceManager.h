#pragma once

#include <QString>
#include <QVector>

struct CameraExposureProfile {
    double minimum{-13.0};
    double maximum{0.0};
    double step{1.0};
    double defaultValue{-6.0};
    double manualAutoValue{0.25};
};

struct CameraBacklightProfile {
    double minimum{0.0};
    double maximum{10.0};
    double step{1.0};
    double defaultValue{10.0};
};

struct CameraDeviceInfo {
    int cvIndex{-1};
    QString deviceId;      // Windows DirectShow moniker/device path. Stable enough for selection.
    QString friendlyName;
    QString vid;
    QString pid;
    QString modelName;
    int width{1920};
    int height{1200};
    double fps{10.0};
    QString fourcc{QStringLiteral("MJPG")};
    CameraExposureProfile exposure;
    CameraBacklightProfile backlight;

    QString displayText() const;
};

struct CameraScanSettings {
    QString lastCalibrationPath;
};

class CameraDeviceManager {
  public:
    // Runs on the scan source worker thread; never call from UI for production use.
    static QVector<CameraDeviceInfo> enumerate(const QString& modelJsonPath, QString* error = nullptr);

    // Persistent scan UI settings live in the same camera_models.json so deployment has
    // only one camera-related configuration file. These helpers preserve the cameras array.
    static CameraScanSettings loadScanSettings(const QString& modelJsonPath, QString* error = nullptr);
    static bool saveScanSettings(const QString& modelJsonPath, const CameraScanSettings& settings,
                                 QString* error = nullptr);
};
