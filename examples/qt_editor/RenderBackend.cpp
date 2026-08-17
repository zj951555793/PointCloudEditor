#include "RenderBackend.h"

#include <QOpenGLContext>
#include <QSurfaceFormat>

namespace {

// 为了让 Desktop GL2.1 与 GLES3.1 真正共用同一套 GPU Buffer，
// 顶点属性只使用两边都稳定支持的 float/normalized-u8 类型：
// 0 position(vec3), 1 color(RGBA8 normalized), 2 normal(vec3),
// 4 flags(u8 -> float), 5 selected(u8 -> float)。
// Picking 使用 0 position + 1 pickColor(RGBA8 normalized)。

constexpr const char* kDesktopRenderVs = R"GLSL(#version 120
attribute vec3 aPosition;
attribute vec4 aColor;
attribute vec3 aNormal;
attribute float aFlags;
attribute float aSelected;
uniform mat4 uMVP;
uniform mat3 uNormalMatrix;
uniform float uPointSize;
uniform float uPointMode;
varying vec4 vColor;
varying vec3 vNormal;
varying float vSelected;
void main() {
    // PointDeleted = 1 << 2。GLSL 1.20 无整数位运算，用除法+mod 判断第2位。
    float deleted = mod(floor(aFlags / 4.0), 2.0);
    if (deleted > 0.5) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        gl_PointSize = 1.0;
    } else {
        gl_Position = uMVP * vec4(aPosition, 1.0);
        gl_PointSize = uPointSize;
    }
    vColor = aColor;
    vNormal = uNormalMatrix * aNormal;
    vSelected = aSelected;
}
)GLSL";

constexpr const char* kDesktopRenderFs = R"GLSL(#version 120
varying vec4 vColor;
varying vec3 vNormal;
varying float vSelected;
uniform float uPointMode;
void main() {
    if (uPointMode > 0.5) {
        vec2 q = gl_PointCoord * 2.0 - vec2(1.0);
        if (dot(q, q) > 1.0) discard;
    }
    vec3 base = vColor.rgb;
    float nl = length(vNormal);
    if (nl > 0.001) {
        vec3 n = normalize(vNormal);
        // View-space headlight: camera looks along -Z, so a surface facing the camera has +Z normal.
        // Point clouds frequently contain unoriented / locally flipped normals. For point rendering use
        // two-sided diffuse so rotating the camera never turns a valid surface into an artificially dark side.
        // Mesh rendering stays one-sided to preserve solid-surface shape cues.
        float ndl = dot(n, vec3(0.0, 0.0, 1.0));
        float d = (uPointMode > 0.5) ? abs(ndl) : max(ndl, 0.0);
        base *= (0.58 + 0.42 * d);
    }
    if (vSelected > 0.5) base = mix(base, vec3(1.0, 0.48, 0.03), 0.70);
    gl_FragColor = vec4(base, 1.0);
}
)GLSL";

// Desktop GL2.1 扩展 Picking：不上传任何第二份 Picking 几何。
// PointId 直接使用 GL_EXT_gpu_shader4 提供的 gl_VertexID；
// TriangleId 使用 GL_EXT_geometry_shader4 提供的 gl_PrimitiveIDIn。
constexpr const char* kDesktopPointPickVs = R"GLSL(#version 120
#extension GL_EXT_gpu_shader4 : require
attribute vec3 aPosition;
attribute float aFlags;
uniform mat4 uMVP;
uniform float uPointSize;
flat varying uint vObjectId;
void main() {
    float deleted = mod(floor(aFlags / 4.0), 2.0);
    if (deleted > 0.5) {
        gl_Position = vec4(2.0,2.0,2.0,1.0);
        gl_PointSize = 1.0;
    } else {
        gl_Position = uMVP * vec4(aPosition,1.0);
        gl_PointSize = uPointSize;
    }
    // 0 留给背景，因此实际写入 VertexId + 1。
    vObjectId = uint(gl_VertexID) + 1u;
}
)GLSL";
constexpr const char* kDesktopPointPickFs = R"GLSL(#version 120
#extension GL_EXT_gpu_shader4 : require
flat varying uint vObjectId;
varying out uvec4 outId;
void main() {
    vec2 q = gl_PointCoord * 2.0 - vec2(1.0);
    if (dot(q,q) > 1.0) discard;
    outId = uvec4(vObjectId, 0u, 0u, 1u);
}
)GLSL";
constexpr const char* kDesktopMeshPickVs = R"GLSL(#version 120
attribute vec3 aPosition;
attribute float aFlags;
uniform mat4 uMVP;
varying float vFlags;
void main() {
    gl_Position = uMVP * vec4(aPosition,1.0);
    vFlags = aFlags;
}
)GLSL";
constexpr const char* kDesktopMeshPickGs = R"GLSL(#version 120
#extension GL_EXT_geometry_shader4 : require
#extension GL_EXT_gpu_shader4 : require
varying in float vFlags[];
flat varying out uint gObjectId;
void main() {
    if (mod(floor(vFlags[0] / 4.0), 2.0) > 0.5 ||
        mod(floor(vFlags[1] / 4.0), 2.0) > 0.5 ||
        mod(floor(vFlags[2] / 4.0), 2.0) > 0.5) return;
    // 0 留给背景。gl_PrimitiveIDIn 对一次 glDrawElements 从 0 连续编号。
    gObjectId = uint(gl_PrimitiveIDIn) + 1u;
    for (int i = 0; i < gl_VerticesIn; ++i) {
        gl_Position = gl_PositionIn[i];
        EmitVertex();
    }
    EndPrimitive();
}
)GLSL";
constexpr const char* kDesktopMeshPickFs = R"GLSL(#version 120
#extension GL_EXT_gpu_shader4 : require
flat varying uint gObjectId;
varying out uvec4 outId;
void main() { outId = uvec4(gObjectId, 0u, 0u, 1u); }
)GLSL";

constexpr const char* kGlesRenderVs = R"GLSL(#version 310 es
precision highp float;
layout(location=0) in vec3 aPosition;
layout(location=1) in vec4 aColor;
layout(location=2) in vec3 aNormal;
layout(location=4) in float aFlags;
layout(location=5) in float aSelected;
uniform mat4 uMVP;
uniform mat3 uNormalMatrix;
uniform float uPointSize;
uniform float uPointMode;
out vec4 vColor;
out vec3 vNormal;
out float vSelected;
void main() {
    float deleted = mod(floor(aFlags / 4.0), 2.0);
    if (deleted > 0.5) {
        gl_Position = vec4(2.0,2.0,2.0,1.0);
        gl_PointSize = 1.0;
    } else {
        gl_Position = uMVP * vec4(aPosition,1.0);
        gl_PointSize = uPointSize;
    }
    vColor = aColor;
    vNormal = uNormalMatrix * aNormal;
    vSelected = aSelected;
}
)GLSL";
constexpr const char* kGlesRenderFs = R"GLSL(#version 310 es
precision highp float;
in vec4 vColor;
in vec3 vNormal;
in float vSelected;
uniform float uPointMode;
layout(location=0) out vec4 outColor;
void main() {
    if (uPointMode > 0.5) {
        vec2 q = gl_PointCoord * 2.0 - vec2(1.0);
        if (dot(q,q) > 1.0) discard;
    }
    vec3 base = vColor.rgb;
    float nl = length(vNormal);
    if (nl > 0.001) {
        vec3 n = normalize(vNormal);
        // View-space headlight: camera looks along -Z, so a surface facing the camera has +Z normal.
        // Point clouds frequently contain unoriented / locally flipped normals. For point rendering use
        // two-sided diffuse so rotating the camera never turns a valid surface into an artificially dark side.
        // Mesh rendering stays one-sided to preserve solid-surface shape cues.
        float ndl = dot(n, vec3(0.0, 0.0, 1.0));
        float d = (uPointMode > 0.5) ? abs(ndl) : max(ndl, 0.0);
        base *= (0.58 + 0.42 * d);
    }
    if (vSelected > 0.5) base = mix(base, vec3(1.0, 0.48, 0.03), 0.70);
    outColor = vec4(base,1.0);
}
)GLSL";
constexpr const char* kGlesPointPickVs = R"GLSL(#version 310 es
precision highp float;
layout(location=0) in vec3 aPosition;
layout(location=1) in vec4 aPickColor;
layout(location=4) in float aFlags;
uniform mat4 uMVP;
uniform float uPointSize;
out vec4 vPickColor;
void main() {
    float deleted = mod(floor(aFlags / 4.0), 2.0);
    if (deleted > 0.5) {
        gl_Position = vec4(2.0,2.0,2.0,1.0);
        gl_PointSize = 1.0;
    } else {
        gl_Position = uMVP * vec4(aPosition,1.0);
        gl_PointSize = uPointSize;
    }
    vPickColor = aPickColor;
}
)GLSL";
constexpr const char* kGlesPointPickFs = R"GLSL(#version 310 es
precision highp float;
in vec4 vPickColor;
layout(location=0) out vec4 outColor;
void main() {
    vec2 q = gl_PointCoord * 2.0 - vec2(1.0);
    if (dot(q,q) > 1.0) discard;
    outColor = vPickColor;
}
)GLSL";
constexpr const char* kGlesMeshPickVs = R"GLSL(#version 310 es
precision highp float;
layout(location=0) in vec3 aPosition;
layout(location=1) in vec4 aPickColor;
uniform mat4 uMVP;
out vec4 vPickColor;
void main() { gl_Position = uMVP * vec4(aPosition,1.0); vPickColor = aPickColor; }
)GLSL";
constexpr const char* kGlesMeshPickFs = R"GLSL(#version 310 es
precision highp float;
in vec4 vPickColor;
layout(location=0) out vec4 outColor;
void main() { outColor = vPickColor; }
)GLSL";

class BackendBase : public IRenderBackend {
public:
    void bindRenderAttributeLocations(QOpenGLShaderProgram& p) const override {
        p.bindAttributeLocation("aPosition", 0);
        p.bindAttributeLocation("aColor", 1);
        p.bindAttributeLocation("aNormal", 2);
        p.bindAttributeLocation("aFlags", 4);
        p.bindAttributeLocation("aSelected", 5);
    }
    void bindPointPickAttributeLocations(QOpenGLShaderProgram& p) const override {
        p.bindAttributeLocation("aPosition", 0);
        p.bindAttributeLocation("aFlags", 4);
    }
    void bindMeshPickAttributeLocations(QOpenGLShaderProgram& p) const override {
        p.bindAttributeLocation("aPosition", 0);
    }

    bool vaoSupported() const override { return vaoSupported_; }

protected:
    bool vaoSupported_{false};
    bool gpuPickingSupported_{false};

    static void setupRender(QOpenGLExtraFunctions& gl, const Buffers& b) {
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.positionVbo);
        gl.glEnableVertexAttribArray(0);
        gl.glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(float)*3,nullptr);
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.colorVbo);
        gl.glEnableVertexAttribArray(1);
        gl.glVertexAttribPointer(1,4,GL_UNSIGNED_BYTE,GL_TRUE,4,nullptr);
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.normalVbo);
        gl.glEnableVertexAttribArray(2);
        gl.glVertexAttribPointer(2,3,GL_FLOAT,GL_FALSE,sizeof(float)*3,nullptr);
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.flagsVbo);
        gl.glEnableVertexAttribArray(4);
        gl.glVertexAttribPointer(4,1,GL_UNSIGNED_BYTE,GL_FALSE,1,nullptr);
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.selectionVbo);
        gl.glEnableVertexAttribArray(5);
        gl.glVertexAttribPointer(5,1,GL_UNSIGNED_BYTE,GL_FALSE,1,nullptr);
        gl.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, b.meshEbo);
    }
    static void setupPointPick(QOpenGLExtraFunctions& gl, const Buffers& b) {
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.positionVbo);
        gl.glEnableVertexAttribArray(0);
        gl.glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(float)*3,nullptr);
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.flagsVbo);
        gl.glEnableVertexAttribArray(4);
        gl.glVertexAttribPointer(4,1,GL_UNSIGNED_BYTE,GL_FALSE,1,nullptr);
    }
    static void setupMeshPick(QOpenGLExtraFunctions& gl, const Buffers& b) {
        // 网格 Picking 直接复用正常 Position VBO + EBO。
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.positionVbo);
        gl.glEnableVertexAttribArray(0);
        gl.glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(float)*3,nullptr);
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.flagsVbo);
        gl.glEnableVertexAttribArray(4);
        gl.glVertexAttribPointer(4,1,GL_UNSIGNED_BYTE,GL_FALSE,1,nullptr);
        gl.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, b.meshEbo);
    }

    void createVaos(QOpenGLExtraFunctions& gl, Buffers& b) const {
        gl.glGenVertexArrays(1,&b.vao);
        gl.glBindVertexArray(b.vao);
        setupRender(gl,b);
        gl.glBindVertexArray(0);
    }
    void destroyVaos(QOpenGLExtraFunctions& gl, Buffers& b) const {
        if(b.vao) gl.glDeleteVertexArrays(1,&b.vao);
        b.vao=0;
    }
    void bindRender(QOpenGLExtraFunctions& gl,const Buffers& b) const { gl.glBindVertexArray(b.vao); }
    void bindPointPick(QOpenGLExtraFunctions& gl,const Buffers& b) const {
        gl.glBindVertexArray(b.vao);
        setupPointPick(gl,b);
    }
    void bindMeshPick(QOpenGLExtraFunctions& gl,const Buffers& b) const { gl.glBindVertexArray(b.vao); }
    void unbind(QOpenGLExtraFunctions& gl) const { gl.glBindVertexArray(0); }
};

class DesktopGl21Backend final : public BackendBase {
public:
    QString name() const override {
        return QStringLiteral("Desktop OpenGL 2.1 / VAO / EXT Integer Picking");
    }
    bool isGles() const override { return false; }
    bool gpuPickingSupported() const override { return gpuPickingSupported_; }
    bool validateContext(QString* error) const override {
        auto* c=QOpenGLContext::currentContext();
        if(!c){ if(error)*error=QStringLiteral("没有当前 OpenGL Context"); return false; }
        const auto f=c->format();
        if(c->isOpenGLES() || f.majorVersion()<2 || (f.majorVersion()==2 && f.minorVersion()<1)) {
            if(error)*error=QStringLiteral("Desktop 后端需要 OpenGL 2.1+ Context"); return false;
        }
        const bool hasVao = f.majorVersion() >= 3 || c->hasExtension(QByteArrayLiteral("GL_ARB_vertex_array_object"));
        if (!hasVao) {
            if (error) *error = QStringLiteral("Desktop OpenGL 2.1 驱动缺少 GL_ARB_vertex_array_object，当前版本只支持 VAO");
            return false;
        }
        return true;
    }
    void configureContextState(QOpenGLExtraFunctions& gl) override {
        auto* c = QOpenGLContext::currentContext();
        // OpenGL 3.x+ 核心具有 VAO；2.1 则检测 ARB 扩展。ARB 扩展使用与核心相同的 glGenVertexArrays 名称。
        vaoSupported_ = c && (c->format().majorVersion() >= 3 ||
            c->hasExtension(QByteArrayLiteral("GL_ARB_vertex_array_object")));
        gpuPickingSupported_ = c &&
            c->hasExtension(QByteArrayLiteral("GL_EXT_gpu_shader4")) &&
            c->hasExtension(QByteArrayLiteral("GL_EXT_texture_integer")) &&
            c->hasExtension(QByteArrayLiteral("GL_EXT_geometry_shader4"));
#ifdef GL_POINT_SPRITE
        gl.glEnable(GL_POINT_SPRITE);
#endif
#ifdef GL_VERTEX_PROGRAM_POINT_SIZE
        gl.glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
#endif
    }
    const char* renderVertexShader() const override { return kDesktopRenderVs; }
    const char* renderFragmentShader() const override { return kDesktopRenderFs; }
    const char* pointPickVertexShader() const override { return kDesktopPointPickVs; }
    const char* pointPickFragmentShader() const override { return kDesktopPointPickFs; }
    const char* meshPickVertexShader() const override { return kDesktopMeshPickVs; }
    const char* meshPickFragmentShader() const override { return kDesktopMeshPickFs; }
    const char* meshPickGeometryShader() const override { return kDesktopMeshPickGs; }
    void createVertexArrays(QOpenGLExtraFunctions& gl, Buffers& b) const override { createVaos(gl,b); }
    void destroyVertexArrays(QOpenGLExtraFunctions& gl, Buffers& b) const override { destroyVaos(gl,b); }
    void bindRenderLayout(QOpenGLExtraFunctions& gl,const Buffers& b) const override { bindRender(gl,b); }
    void bindPointPickLayout(QOpenGLExtraFunctions& gl,const Buffers& b) const override { bindPointPick(gl,b); }
    void bindMeshPickLayout(QOpenGLExtraFunctions& gl,const Buffers& b) const override { bindMeshPick(gl,b); }
    void unbindLayout(QOpenGLExtraFunctions& gl) const override { unbind(gl); }
};

class Gles31Backend final : public BackendBase {
public:
    QString name() const override {
        return QStringLiteral("OpenGL ES 3.1 / VAO / RGB24 Block Picking");
    }
    bool isGles() const override { return true; }
    bool gpuPickingSupported() const override { return false; }
    bool validateContext(QString* error) const override {
        auto* c=QOpenGLContext::currentContext();
        if(!c){ if(error)*error=QStringLiteral("没有当前 OpenGL Context"); return false; }
        const auto f=c->format();
        if(!c->isOpenGLES() || f.majorVersion()<3 || (f.majorVersion()==3 && f.minorVersion()<1)) {
            if(error)*error=QStringLiteral("GLES 后端需要 OpenGL ES 3.1+ Context"); return false;
        }
        return true;
    }
    void configureContextState(QOpenGLExtraFunctions&) override {
        // GLES 3.1 原生支持 VAO。
        vaoSupported_ = true;
    }
    const char* renderVertexShader() const override { return kGlesRenderVs; }
    const char* renderFragmentShader() const override { return kGlesRenderFs; }
    const char* pointPickVertexShader() const override { return kGlesPointPickVs; }
    const char* pointPickFragmentShader() const override { return kGlesPointPickFs; }
    const char* meshPickVertexShader() const override { return kGlesMeshPickVs; }
    const char* meshPickFragmentShader() const override { return kGlesMeshPickFs; }
    const char* meshPickGeometryShader() const override { return nullptr; }
    void createVertexArrays(QOpenGLExtraFunctions& gl, Buffers& b) const override { createVaos(gl,b); }
    void destroyVertexArrays(QOpenGLExtraFunctions& gl, Buffers& b) const override { destroyVaos(gl,b); }
    void bindRenderLayout(QOpenGLExtraFunctions& gl,const Buffers& b) const override { bindRender(gl,b); }
    void bindPointPickLayout(QOpenGLExtraFunctions& gl,const Buffers& b) const override { bindPointPick(gl,b); }
    void bindMeshPickLayout(QOpenGLExtraFunctions& gl,const Buffers& b) const override { bindMeshPick(gl,b); }
    void unbindLayout(QOpenGLExtraFunctions& gl) const override { unbind(gl); }
};

} // namespace

std::unique_ptr<IRenderBackend> createRenderBackend()
{
#ifdef JMENGINE_RENDER_GLES31
    return std::make_unique<Gles31Backend>();
#else
    return std::make_unique<DesktopGl21Backend>();
#endif
}
