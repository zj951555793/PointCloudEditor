#include "MainWindow.h"

#include <QApplication>
#include <QCoreApplication>
#include <QDebug>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QSurfaceFormat>

namespace {

QSurfaceFormat makeDesktopFormat(int major, int minor) {
    QSurfaceFormat fmt;
    fmt.setRenderableType(QSurfaceFormat::OpenGL);
    fmt.setVersion(major, minor);
    // 2.1 没有 profile 概念；3.2 选择 Compatibility，让 GLSL 1.20 的普通渲染
    // 与现代 R32UI Picking 可以共存。
    fmt.setProfile(major >= 3 ? QSurfaceFormat::CompatibilityProfile : QSurfaceFormat::NoProfile);
    fmt.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    fmt.setRedBufferSize(8);
    fmt.setGreenBufferSize(8);
    fmt.setBlueBufferSize(8);
    fmt.setAlphaBufferSize(8);
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    fmt.setSamples(0);
    return fmt;
}

bool canCreateDesktopContext(const QSurfaceFormat& requested) {
    QOpenGLContext probe;
    probe.setFormat(requested);
    if (!probe.create())
        return false;

    QOffscreenSurface surface;
    surface.setFormat(probe.format());
    surface.create();
    if (!surface.isValid() || !probe.makeCurrent(&surface))
        return false;

    const auto actual = probe.format();
    const bool versionOk =
        actual.majorVersion() > requested.majorVersion() ||
        (actual.majorVersion() == requested.majorVersion() &&
         actual.minorVersion() >= requested.minorVersion());
    const bool profileOk = requested.majorVersion() < 3 || actual.profile() != QSurfaceFormat::CoreProfile;
    const bool ok = !probe.isOpenGLES() && versionOk && profileOk;
    probe.doneCurrent();
    return ok;
}

} // namespace

int main(int argc, char* argv[]) {
    // 已处理的 TouchEvent 不再额外合成为 MouseEvent，避免一次触摸触发两套交互。
    QCoreApplication::setAttribute(Qt::AA_SynthesizeMouseForUnhandledTouchEvents, false);

#ifdef JMENGINE_RENDER_GLES31
    QSurfaceFormat fmt;
    fmt.setRenderableType(QSurfaceFormat::OpenGLES);
    fmt.setVersion(3, 1);
    fmt.setProfile(QSurfaceFormat::NoProfile);
    fmt.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    fmt.setRedBufferSize(8);
    fmt.setGreenBufferSize(8);
    fmt.setBlueBufferSize(8);
    fmt.setAlphaBufferSize(8);
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    fmt.setSamples(0);
    QSurfaceFormat::setDefaultFormat(fmt);
    QApplication app(argc, argv);
#else
    // 先用最低支持的 Desktop GL2.1 作为安全默认值创建 QApplication。
    // 在任何 QOpenGLWidget 出现前，再用离屏 Context 探测 3.2；能创建就升级，
    // 不能创建则保留 2.1。这样老显卡不会因强制 3.2 在启动阶段直接失败。
    const QSurfaceFormat legacy = makeDesktopFormat(2, 1);
    QSurfaceFormat::setDefaultFormat(legacy);
    QApplication app(argc, argv);

    const bool forceLegacy = qEnvironmentVariableIntValue("JMENGINE_FORCE_GL21") != 0;
    const QSurfaceFormat modern = makeDesktopFormat(3, 2);
    const bool modernAvailable = !forceLegacy && canCreateDesktopContext(modern);
    const QSurfaceFormat selected = modernAvailable ? modern : legacy;
    QSurfaceFormat::setDefaultFormat(selected);
    qInfo() << "[JMEngine GL] selected="
            << (modernAvailable ? "OpenGL 3.2 compatibility" : "OpenGL 2.1 legacy")
            << "forceLegacy=" << forceLegacy;
#endif

    const QString initial = argc > 1 ? QString::fromLocal8Bit(argv[1]) : QString{};

    MainWindow window(initial);
#ifdef Q_OS_ANDROID
    // Android is touch-first; use the full application window while preserving the same Widgets UI.
    window.showMaximized();
#else
    window.show();
#endif
    return app.exec();
}
