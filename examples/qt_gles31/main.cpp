#include "GlesMainWindow.h"

#include <QApplication>
#include <QSurfaceFormat>

int main(int argc, char* argv[])
{
    // 已处理的 TouchEvent 不再额外合成为 MouseEvent，避免一次触摸触发两套交互。
    QCoreApplication::setAttribute(Qt::AA_SynthesizeMouseForUnhandledTouchEvents, false);
    // RK3588 / Mali-G610：明确请求 OpenGL ES 3.1。
    // 必须在 QApplication 创建之前设置默认格式。
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
    const QString initial = argc > 1 ? QString::fromLocal8Bit(argv[1]) : QString{};

    GlesMainWindow window(initial);
    window.show();
    return app.exec();
}
