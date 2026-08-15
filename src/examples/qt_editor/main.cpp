#include "MainWindow.h"

#include <QApplication>
#include <QCoreApplication>
#include <QSurfaceFormat>

int main(int argc, char* argv[]) {
    // 已处理的 TouchEvent 不再额外合成为 MouseEvent，避免一次触摸触发两套交互。
    QCoreApplication::setAttribute(Qt::AA_SynthesizeMouseForUnhandledTouchEvents, false);

    // 同一套 Qt 编辑器代码，只根据 CMake 选择的渲染后端请求不同 Context。
    // Windows/x86 Desktop：OpenGL 3.2 Compatibility（现代 R32UI GPU Picking）
    // RK3588/ARM64 Linux：OpenGL ES 3.1
    QSurfaceFormat fmt;
#ifdef PCEDITOR_RENDER_GLES31
    fmt.setRenderableType(QSurfaceFormat::OpenGLES);
    fmt.setVersion(3, 1);
    fmt.setProfile(QSurfaceFormat::NoProfile);
#else
    fmt.setRenderableType(QSurfaceFormat::OpenGL);
    fmt.setVersion(3, 2);
    // Render shader 仍兼容 GLSL 1.20，因此明确请求 Compatibility Profile；
    // 同时 OpenGL 3.2 提供现代 R32UI / Geometry Shader GPU Picking。
    fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
#endif
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
    const QString initial = argc > 1 ? QString::fromLocal8Bit(argv[1]) : QString{};

    MainWindow window(initial);
    window.show();
    return app.exec();
}
