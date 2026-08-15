#pragma once

#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QString>

#include <memory>

// 渲染后端统一只使用 VAO。
// Desktop OpenGL 2.1 必须提供 GL_ARB_vertex_array_object 扩展；GLES 3.1 原生支持 VAO。
// Qt 编辑器唯一维护的一套渲染后端接口。
// 上层 MainWindow / PointCloudWidget / ModelManager 完全不区分 Windows 与 RK3588。
// 差异只保留在这里：
//   - Desktop OpenGL 2.1：VBO 必选；运行时检测 GL_ARB_vertex_array_object，支持时可使用 VAO；
//   - OpenGL ES 3.1：原生支持 VBO + VAO；
//   - Desktop GL2.1 GPU Picking 依赖 EXT_gpu_shader4/EXT_texture_integer/EXT_geometry_shader4；
//     不支持则由上层自动使用 CPU Picking；Picking 不上传第二份几何。
class IRenderBackend {
public:
    struct Buffers {
        GLuint vao{0};
        GLuint positionVbo{0};
        GLuint colorVbo{0};
        GLuint normalVbo{0};
        GLuint flagsVbo{0};
        GLuint selectionVbo{0};
        GLuint meshEbo{0};
    };

    virtual ~IRenderBackend() = default;

    virtual QString name() const = 0;
    virtual bool isGles() const = 0;
    virtual bool validateContext(QString* error) const = 0;
    virtual void configureContextState(QOpenGLExtraFunctions& gl) = 0;

    // 统一要求 VAO。Desktop GL2.1 若缺少 ARB VAO 扩展，后端初始化失败。
    virtual bool vaoSupported() const = 0;
    // GPU Picking 仅在当前后端/驱动真正支持所需扩展时启用；否则上层自动回退 CPU。
    virtual bool gpuPickingSupported() const = 0;
    bool usesVao() const { return true; }

    // Shader 文本由后端提供。算法完全一致，仅 GLSL 方言不同。
    virtual const char* renderVertexShader() const = 0;
    virtual const char* renderFragmentShader() const = 0;
    virtual const char* pointPickVertexShader() const = 0;
    virtual const char* pointPickFragmentShader() const = 0;
    virtual const char* meshPickVertexShader() const = 0;
    virtual const char* meshPickFragmentShader() const = 0;
    virtual const char* meshPickGeometryShader() const = 0;

    // GLSL 1.20 没有 layout(location=...)，必须在 link 前绑定属性位置。
    virtual void bindRenderAttributeLocations(QOpenGLShaderProgram& program) const = 0;
    virtual void bindPointPickAttributeLocations(QOpenGLShaderProgram& program) const = 0;
    virtual void bindMeshPickAttributeLocations(QOpenGLShaderProgram& program) const = 0;

    // 创建/销毁 VAO。所有平台都固定使用 VAO。
    virtual void createVertexArrays(QOpenGLExtraFunctions& gl, Buffers& b) const = 0;
    virtual void destroyVertexArrays(QOpenGLExtraFunctions& gl, Buffers& b) const = 0;

    // Picking 与普通渲染复用同一套几何 VBO/EBO，不创建/上传第二份 Position。
    // 绑定普通渲染/点 Picking/网格 Picking 所需的顶点布局。
    virtual void bindRenderLayout(QOpenGLExtraFunctions& gl, const Buffers& b) const = 0;
    virtual void bindPointPickLayout(QOpenGLExtraFunctions& gl, const Buffers& b) const = 0;
    virtual void bindMeshPickLayout(QOpenGLExtraFunctions& gl, const Buffers& b) const = 0;
    virtual void unbindLayout(QOpenGLExtraFunctions& gl) const = 0;
};

// 根据 CMake 选定的目标平台创建后端。
// PCEDITOR_RENDER_GLES31：RK3588 / ARM64 Linux
// PCEDITOR_RENDER_DESKTOP_GL21：Windows / x86 Desktop
std::unique_ptr<IRenderBackend> createRenderBackend();
