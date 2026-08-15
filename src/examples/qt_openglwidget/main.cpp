#include "MainWindow.h"
#include "../common/ExampleUtils.h"

#include <QApplication>
#include <QSurfaceFormat>

int main(int argc, char* argv[]) {
    pceditor::example::enableUtf8Console();

    // R32UI GPU Picking 和整数顶点属性要求桌面 OpenGL 2.1 Core。
    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setVersion(2, 1);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setSamples(4); // 正常网格显示开启 MSAA；Picking FBO 仍使用单采样整数纹理。
    QSurfaceFormat::setDefaultFormat(format);

    QApplication app(argc, argv);
    const QString initial = argc > 1 ? QString::fromLocal8Bit(argv[1]) : QString{};

    MainWindow window(initial);
    window.show();
    return app.exec();
}
