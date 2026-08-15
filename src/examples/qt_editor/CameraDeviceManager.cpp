#include "CameraDeviceManager.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>

#include <algorithm>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <dshow.h>
#endif

namespace {
struct ModelProfile {
    QString vid;
    QString pid;
    QString model;
    int width{1920};
    int height{1200};
    double fps{10.0};
    QString fourcc{QStringLiteral("MJPG")};
    CameraExposureProfile exposure;
    CameraBacklightProfile backlight;
};

QString normalizedHex(QString value) {
    value = value.trimmed().toUpper();
    if (value.startsWith(QStringLiteral("0X"))) value.remove(0, 2);
    return value.rightJustified(4, QLatin1Char('0')).right(4);
}

QVector<ModelProfile> loadProfiles(const QString& path, QString* error) {
    QVector<ModelProfile> out;
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) *error = QString::fromUtf8("无法打开相机型号 JSON: %1").arg(path);
        return out;
    }
    QJsonParseError parseError{};
    const auto doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (error) *error = QString::fromUtf8("相机型号 JSON 格式错误: %1").arg(parseError.errorString());
        return out;
    }
    const auto array = doc.object().value(QStringLiteral("cameras")).toArray();
    for (const auto& value : array) {
        if (!value.isObject()) continue;
        const auto o = value.toObject();
        ModelProfile p;
        p.vid = normalizedHex(o.value(QStringLiteral("vid")).toString());
        p.pid = normalizedHex(o.value(QStringLiteral("pid")).toString());
        p.model = o.value(QStringLiteral("model")).toString(QStringLiteral("Unknown Camera"));
        p.width = o.value(QStringLiteral("width")).toInt(1920);
        p.height = o.value(QStringLiteral("height")).toInt(1200);
        p.fps = o.value(QStringLiteral("fps")).toDouble(10.0);
        p.fourcc = o.value(QStringLiteral("fourcc")).toString(QStringLiteral("MJPG"));
        const auto e = o.value(QStringLiteral("exposure")).toObject();
        p.exposure.minimum = e.value(QStringLiteral("min")).toDouble(-13.0);
        p.exposure.maximum = e.value(QStringLiteral("max")).toDouble(0.0);
        p.exposure.step = e.value(QStringLiteral("step")).toDouble(1.0);
        p.exposure.defaultValue = e.value(QStringLiteral("default")).toDouble(-6.0);
        p.exposure.manualAutoValue = e.value(QStringLiteral("manualAutoValue")).toDouble(0.25);
        const auto b = o.value(QStringLiteral("backlight")).toObject();
        p.backlight.minimum = b.value(QStringLiteral("min")).toDouble(0.0);
        p.backlight.maximum = b.value(QStringLiteral("max")).toDouble(10.0);
        p.backlight.step = b.value(QStringLiteral("step")).toDouble(1.0);
        p.backlight.defaultValue = b.value(QStringLiteral("default")).toDouble(10.0);
        if (!p.vid.isEmpty() && !p.pid.isEmpty()) out.push_back(p);
    }
    return out;
}

void applyProfile(CameraDeviceInfo& info, const QVector<ModelProfile>& profiles) {
    for (const auto& p : profiles) {
        if (info.vid == p.vid && info.pid == p.pid) {
            info.modelName = p.model;
            info.width = p.width;
            info.height = p.height;
            info.fps = p.fps;
            info.fourcc = p.fourcc;
            info.exposure = p.exposure;
            info.backlight = p.backlight;
            return;
        }
    }
    info.modelName = QString::fromUtf8("未配置型号");
}

void parseVidPid(const QString& text, QString& vid, QString& pid) {
    static const QRegularExpression re(QStringLiteral("vid_([0-9a-fA-F]{4}).*pid_([0-9a-fA-F]{4})"),
                                       QRegularExpression::CaseInsensitiveOption);
    const auto m = re.match(text);
    if (m.hasMatch()) {
        vid = normalizedHex(m.captured(1));
        pid = normalizedHex(m.captured(2));
    }
}

#ifdef Q_OS_WIN
QString variantString(const VARIANT& v) {
    return v.vt == VT_BSTR && v.bstrVal ? QString::fromWCharArray(v.bstrVal) : QString{};
}
#endif
} // namespace

QString CameraDeviceInfo::displayText() const {
    return QStringLiteral("%1 | %2 | %3:%4")
        .arg(modelName.isEmpty() ? QString::fromUtf8("未配置型号") : modelName,
             friendlyName.isEmpty() ? QString::fromUtf8("Camera") : friendlyName,
             vid.isEmpty() ? QStringLiteral("????") : vid,
             pid.isEmpty() ? QStringLiteral("????") : pid);
}

QVector<CameraDeviceInfo> CameraDeviceManager::enumerate(const QString& modelJsonPath, QString* error) {
    QString profileError;
    const auto profiles = loadProfiles(modelJsonPath, &profileError);
    if (profiles.isEmpty() && !profileError.isEmpty() && error) *error = profileError;

    QVector<CameraDeviceInfo> devices;
#ifdef Q_OS_WIN
    const HRESULT initHr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    const bool uninit = SUCCEEDED(initHr);

    ICreateDevEnum* devEnum = nullptr;
    IEnumMoniker* enumMoniker = nullptr;
    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, nullptr, CLSCTX_INPROC_SERVER,
                                  IID_ICreateDevEnum, reinterpret_cast<void**>(&devEnum));
    if (SUCCEEDED(hr) && devEnum) {
        hr = devEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &enumMoniker, 0);
    }
    if (hr == S_OK && enumMoniker) {
        IMoniker* moniker = nullptr;
        ULONG fetched = 0;
        int index = 0;
        while (enumMoniker->Next(1, &moniker, &fetched) == S_OK) {
            CameraDeviceInfo info;
            info.cvIndex = index++;

            IPropertyBag* bag = nullptr;
            if (SUCCEEDED(moniker->BindToStorage(nullptr, nullptr, IID_IPropertyBag,
                                                 reinterpret_cast<void**>(&bag))) && bag) {
                VARIANT v;
                VariantInit(&v);
                if (SUCCEEDED(bag->Read(L"FriendlyName", &v, nullptr))) info.friendlyName = variantString(v);
                VariantClear(&v);
                VariantInit(&v);
                if (SUCCEEDED(bag->Read(L"DevicePath", &v, nullptr))) info.deviceId = variantString(v);
                VariantClear(&v);
                bag->Release();
            }

            if (info.deviceId.isEmpty()) {
                IBindCtx* bindCtx = nullptr;
                LPOLESTR name = nullptr;
                if (SUCCEEDED(CreateBindCtx(0, &bindCtx)) && bindCtx) {
                    if (SUCCEEDED(moniker->GetDisplayName(bindCtx, nullptr, &name)) && name) {
                        info.deviceId = QString::fromWCharArray(name);
                        CoTaskMemFree(name);
                    }
                    bindCtx->Release();
                }
            }
            parseVidPid(info.deviceId, info.vid, info.pid);
            applyProfile(info, profiles);
            devices.push_back(std::move(info));
            moniker->Release();
        }
        enumMoniker->Release();
    } else if (error && error->isEmpty()) {
        *error = QString::fromUtf8("DirectShow 未枚举到视频设备");
    }
    if (devEnum) devEnum->Release();
    if (uninit) CoUninitialize();
#else
    Q_UNUSED(profiles);
    if (error && error->isEmpty())
        *error = QString::fromUtf8("当前相机 VID/PID 自动枚举实现面向 Windows DirectShow");
#endif
    return devices;
}

CameraScanSettings CameraDeviceManager::loadScanSettings(const QString& modelJsonPath, QString* error) {
    CameraScanSettings settings;
    QFile file(modelJsonPath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) *error = QString::fromUtf8("无法打开相机型号 JSON: %1").arg(modelJsonPath);
        return settings;
    }
    QJsonParseError parseError{};
    const auto doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (error) *error = QString::fromUtf8("相机型号 JSON 格式错误: %1").arg(parseError.errorString());
        return settings;
    }
    const auto scan = doc.object().value(QStringLiteral("scan")).toObject();
    settings.lastCalibrationPath = scan.value(QStringLiteral("lastCalibrationPath")).toString();
    return settings;
}

bool CameraDeviceManager::saveScanSettings(const QString& modelJsonPath,
                                           const CameraScanSettings& settings,
                                           QString* error) {
    QFile file(modelJsonPath);
    QJsonObject root;
    if (file.exists()) {
        if (!file.open(QIODevice::ReadOnly)) {
            if (error) *error = QString::fromUtf8("无法读取相机型号 JSON: %1").arg(modelJsonPath);
            return false;
        }
        QJsonParseError parseError{};
        const auto doc = QJsonDocument::fromJson(file.readAll(), &parseError);
        file.close();
        if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
            if (error) *error = QString::fromUtf8("相机型号 JSON 格式错误: %1").arg(parseError.errorString());
            return false;
        }
        root = doc.object();
    } else {
        root.insert(QStringLiteral("version"), 1);
        root.insert(QStringLiteral("cameras"), QJsonArray{});
    }

    QJsonObject scan = root.value(QStringLiteral("scan")).toObject();
    scan.insert(QStringLiteral("lastCalibrationPath"), settings.lastCalibrationPath);
    root.insert(QStringLiteral("scan"), scan);

    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (error) *error = QString::fromUtf8("无法写入相机型号 JSON: %1").arg(modelJsonPath);
        return false;
    }
    const auto bytes = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (file.write(bytes) != bytes.size()) {
        if (error) *error = QString::fromUtf8("写入相机型号 JSON 失败: %1").arg(modelJsonPath);
        return false;
    }
    return true;
}
