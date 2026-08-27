#include <JMEngine/render/RenderBackend.h>

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
attribute vec2 aTexCoord;
attribute float aFlags;
attribute float aSelected;
uniform mat4 uMVP;
uniform float uPointSize;
uniform float uPointMode;
uniform float uForceSelected;
varying vec4 vColor;
varying vec3 vNormal;
varying vec2 vTexCoord;
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
    vNormal = aNormal;
    vTexCoord = aTexCoord;
    vSelected = max(aSelected, uForceSelected);
}
)GLSL";

constexpr const char* kDesktopRenderFs = R"GLSL(#version 120
varying vec4 vColor;
varying vec3 vNormal;
varying vec2 vTexCoord;
varying float vSelected;
uniform vec3 uLightDir;
uniform sampler2D uTexture;
uniform float uUseTexture;
uniform float uPointMode;
uniform float uWireframe;
void main() {
    if (uPointMode > 0.5) {
        vec2 q = gl_PointCoord * 2.0 - vec2(1.0);
        if (dot(q, q) > 1.0) discard;
    }
    vec3 base = (uUseTexture > 0.5) ? texture2D(uTexture, vTexCoord).rgb : vColor.rgb;
    float nl = length(vNormal);
    if (nl > 0.001) {
        vec3 n = normalize(vNormal);
        float ndl = dot(n, normalize(uLightDir));
        // 点云法线常常没有做全局朝向一致化，因此点模式使用双面 Headlight。
        // 网格保持单面 Lambert，继续保留实体表面的明暗关系。
        float d = (uPointMode > 0.5) ? abs(ndl) : max(ndl, 0.0);
        // 工业查看器优先保证任意视角下可读，同时保留足够的形状明暗层次。
        float ambient = (uPointMode > 0.5) ? 0.62 : 0.20;
        float diffuse = (uPointMode > 0.5) ? 0.34 : 0.68;
        float spec = ((uPointMode > 0.5) ? 0.04 : 0.08) * pow(d, 32.0);
        base = base * (ambient + diffuse * d) + vec3(spec);
        base = min(base, vec3(0.96));
    }
    if (uWireframe > 0.5) base = vec3(0.06);
    if (vSelected > 0.5) base = mix(base, vec3(1.0, 0.48, 0.03), 0.70);
    gl_FragColor = vec4(base, 1.0);
}
)GLSL";

// Desktop 现代整数 Picking：只在 OpenGL 3.2+ 上启用。
// 旧 EXT_gpu_shader4 路径已移除；现代 shader/FBO 任一失败时上层自动使用 CPU Picking。
constexpr const char* kDesktopPointPickVs = R"GLSL(#version 150
in vec3 aPosition;
in float aFlags;
uniform mat4 uMVP;
uniform float uPointSize;
flat out uint vObjectId;
void main() {
    float deleted = mod(floor(aFlags / 4.0), 2.0);
    if (deleted > 0.5) {
        gl_Position = vec4(2.0,2.0,2.0,1.0);
        gl_PointSize = 1.0;
    } else {
        gl_Position = uMVP * vec4(aPosition,1.0);
        gl_PointSize = uPointSize;
    }
    vObjectId = uint(gl_VertexID) + 1u;
}
)GLSL";
constexpr const char* kDesktopPointPickFs = R"GLSL(#version 150
flat in uint vObjectId;
out uint outId;
void main() {
    vec2 q = gl_PointCoord * 2.0 - vec2(1.0);
    if (dot(q,q) > 1.0) discard;
    outId = vObjectId;
}
)GLSL";
constexpr const char* kDesktopMeshPickVs = R"GLSL(#version 150
in vec3 aPosition;
uniform mat4 uMVP;
void main() { gl_Position = uMVP * vec4(aPosition,1.0); }
)GLSL";
constexpr const char* kDesktopMeshPickGs = R"GLSL(#version 150
layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;
flat out uint gObjectId;
void main() {
    gObjectId = uint(gl_PrimitiveIDIn) + 1u;
    for (int i = 0; i < 3; ++i) {
        gl_Position = gl_in[i].gl_Position;
        EmitVertex();
    }
    EndPrimitive();
}
)GLSL";
constexpr const char* kDesktopMeshPickFs = R"GLSL(#version 150
flat in uint gObjectId;
out uint outId;
void main() { outId = gObjectId; }
)GLSL";

constexpr const char* kGlesRenderVs = R"GLSL(#version 310 es
precision highp float;
layout(location=0) in vec3 aPosition;
layout(location=1) in vec4 aColor;
layout(location=2) in vec3 aNormal;
layout(location=3) in vec2 aTexCoord;
layout(location=4) in float aFlags;
layout(location=5) in float aSelected;
uniform mat4 uMVP;
uniform float uPointSize;
uniform float uPointMode;
uniform float uForceSelected;
out vec4 vColor;
out vec3 vNormal;
out vec2 vTexCoord;
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
    vNormal = aNormal;
    vTexCoord = aTexCoord;
    vSelected = max(aSelected, uForceSelected);
}
)GLSL";
constexpr const char* kGlesRenderFs = R"GLSL(#version 310 es
precision highp float;
in vec4 vColor;
in vec3 vNormal;
in vec2 vTexCoord;
in float vSelected;
uniform vec3 uLightDir;
uniform sampler2D uTexture;
uniform float uUseTexture;
uniform float uPointMode;
uniform float uWireframe;
layout(location=0) out vec4 outColor;
void main() {
    if (uPointMode > 0.5) {
        vec2 q = gl_PointCoord * 2.0 - vec2(1.0);
        if (dot(q,q) > 1.0) discard;
    }
    vec3 base = (uUseTexture > 0.5) ? texture(uTexture, vTexCoord).rgb : vColor.rgb;
    float nl = length(vNormal);
    if (nl > 0.001) {
        vec3 n = normalize(vNormal);
        float ndl = dot(n, normalize(uLightDir));
        // 点云法线常常没有做全局朝向一致化，因此点模式使用双面 Headlight。
        // 网格保持单面 Lambert，继续保留实体表面的明暗关系。
        float d = (uPointMode > 0.5) ? abs(ndl) : max(ndl, 0.0);
        // 工业查看器优先保证任意视角下可读，同时保留足够的形状明暗层次。
        float ambient = (uPointMode > 0.5) ? 0.62 : 0.20;
        float diffuse = (uPointMode > 0.5) ? 0.34 : 0.68;
        float spec = ((uPointMode > 0.5) ? 0.04 : 0.08) * pow(d, 32.0);
        base = base * (ambient + diffuse * d) + vec3(spec);
        base = min(base, vec3(0.96));
    }
    if (uWireframe > 0.5) base = vec3(0.06);
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
        p.bindAttributeLocation("aTexCoord", 3);
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

    bool vaoSupported() const override {
        return vaoSupported_;
    }

  protected:
    bool vaoSupported_{false};
    bool gpuPickingSupported_{false};

    static void setupRender(QOpenGLExtraFunctions& gl, const Buffers& b) {
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.positionVbo);
        gl.glEnableVertexAttribArray(0);
        gl.glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, nullptr);
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.colorVbo);
        gl.glEnableVertexAttribArray(1);
        gl.glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, 4, nullptr);
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.normalVbo);
        gl.glEnableVertexAttribArray(2);
        gl.glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, nullptr);
        if (b.texcoordVbo) {
            gl.glBindBuffer(GL_ARRAY_BUFFER, b.texcoordVbo);
            gl.glEnableVertexAttribArray(3);
            gl.glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, nullptr);
        } else {
            gl.glDisableVertexAttribArray(3);
            gl.glVertexAttrib2f(3, 0.0f, 0.0f);
        }
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.flagsVbo);
        gl.glEnableVertexAttribArray(4);
        gl.glVertexAttribPointer(4, 1, GL_UNSIGNED_BYTE, GL_FALSE, 1, nullptr);
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.selectionVbo);
        gl.glEnableVertexAttribArray(5);
        gl.glVertexAttribPointer(5, 1, GL_UNSIGNED_BYTE, GL_FALSE, 1, nullptr);
        gl.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, b.meshEbo);
    }
    static void setupPointPick(QOpenGLExtraFunctions& gl, const Buffers& b) {
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.positionVbo);
        gl.glEnableVertexAttribArray(0);
        gl.glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, nullptr);
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.flagsVbo);
        gl.glEnableVertexAttribArray(4);
        gl.glVertexAttribPointer(4, 1, GL_UNSIGNED_BYTE, GL_FALSE, 1, nullptr);
    }
    static void setupMeshSelection(QOpenGLExtraFunctions& gl, const Buffers& b) {
        // 与普通 Render 完全复用顶点 VBO，只把 EBO 换成“选中三角形索引”。
        setupRender(gl, b);
        gl.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, b.selectedMeshEbo);
    }
    static void setupMeshPick(QOpenGLExtraFunctions& gl, const Buffers& b) {
        // 网格 Picking 直接复用正常 Position VBO + EBO。
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.positionVbo);
        gl.glEnableVertexAttribArray(0);
        gl.glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, nullptr);
        gl.glBindBuffer(GL_ARRAY_BUFFER, b.flagsVbo);
        gl.glEnableVertexAttribArray(4);
        gl.glVertexAttribPointer(4, 1, GL_UNSIGNED_BYTE, GL_FALSE, 1, nullptr);
        gl.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, b.meshEbo);
    }

    void createVaos(QOpenGLExtraFunctions& gl, Buffers& b) const {
        // Legacy OpenGL 2.1 hardware may not expose GL_ARB_vertex_array_object.
        // In that case Buffers keep vao=0 and every bind*Layout() replays the VBO attribute state.
        if (!vaoSupported_)
            return;

        gl.glGenVertexArrays(1, &b.vao);
        gl.glBindVertexArray(b.vao);
        setupRender(gl, b);
        gl.glBindVertexArray(0);

        gl.glGenVertexArrays(1, &b.selectionVao);
        gl.glBindVertexArray(b.selectionVao);
        setupMeshSelection(gl, b);
        gl.glBindVertexArray(0);
    }
    void destroyVaos(QOpenGLExtraFunctions& gl, Buffers& b) const {
        if (vaoSupported_) {
            if (b.vao)
                gl.glDeleteVertexArrays(1, &b.vao);
            if (b.selectionVao)
                gl.glDeleteVertexArrays(1, &b.selectionVao);
        }
        b.vao = 0;
        b.selectionVao = 0;
    }
    void bindRender(QOpenGLExtraFunctions& gl, const Buffers& b) const {
        if (vaoSupported_ && b.vao)
            gl.glBindVertexArray(b.vao);
        else
            setupRender(gl, b);
    }
    void bindMeshSelection(QOpenGLExtraFunctions& gl, const Buffers& b) const {
        if (vaoSupported_ && b.selectionVao)
            gl.glBindVertexArray(b.selectionVao);
        else
            setupMeshSelection(gl, b);
    }
    void bindPointPick(QOpenGLExtraFunctions& gl, const Buffers& b) const {
        if (vaoSupported_ && b.vao)
            gl.glBindVertexArray(b.vao);
        setupPointPick(gl, b);
    }
    void bindMeshPick(QOpenGLExtraFunctions& gl, const Buffers& b) const {
        if (vaoSupported_ && b.vao)
            gl.glBindVertexArray(b.vao);
        else
            setupMeshPick(gl, b);
    }
    void unbind(QOpenGLExtraFunctions& gl) const {
        if (vaoSupported_) {
            gl.glBindVertexArray(0);
            return;
        }

        // Pure-VBO fallback: no VAO owns these bindings, so leave a clean global state.
        for (GLuint attr = 0; attr <= 5; ++attr)
            gl.glDisableVertexAttribArray(attr);
        gl.glBindBuffer(GL_ARRAY_BUFFER, 0);
        gl.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }
};

class DesktopGl21Backend final : public BackendBase {
  public:
    QString name() const override {
        return QStringLiteral("Desktop OpenGL 2.1+ / VBO / Optional VAO / Auto Picking");
    }
    bool isGles() const override {
        return false;
    }
    bool gpuPickingSupported() const override {
        return gpuPickingSupported_;
    }
    bool validateContext(QString* error) const override {
        auto* c = QOpenGLContext::currentContext();
        if (!c) {
            if (error)
                *error = QStringLiteral("没有当前 OpenGL Context");
            return false;
        }
        const auto f = c->format();
        if (c->isOpenGLES() || f.majorVersion() < 2 || (f.majorVersion() == 2 && f.minorVersion() < 1)) {
            if (error)
                *error = QStringLiteral("Desktop 后端需要 OpenGL 2.1+ Context");
            return false;
        }
        // VAO is optional on the legacy path. OpenGL 2.1 + VBO + GLSL 1.20 is enough
        // for normal point/mesh rendering; selection falls back to CPU when modern picking
        // capabilities are unavailable.
        return true;
    }
    void configureContextState(QOpenGLExtraFunctions& gl) override {
        auto* c = QOpenGLContext::currentContext();
        // OpenGL 3.x+ has core VAO; 2.1 uses ARB_vertex_array_object when present.
        // No VAO is not fatal: bind*Layout() switches to pure VBO state replay.
        vaoSupported_ =
            c && (c->format().majorVersion() >= 3 || c->hasExtension(QByteArrayLiteral("GL_ARB_vertex_array_object")));

        // R32UI + geometry-shader mesh picking stays on the modern 3.2+ path only.
        // Legacy 2.1 keeps full rendering and automatically uses CPU selection.
        gpuPickingSupported_ =
            c && !c->isOpenGLES() &&
            (c->format().majorVersion() > 3 || (c->format().majorVersion() == 3 && c->format().minorVersion() >= 2));
#ifdef GL_POINT_SPRITE
        gl.glEnable(GL_POINT_SPRITE);
#endif
#ifdef GL_VERTEX_PROGRAM_POINT_SIZE
        gl.glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
#endif
    }
    const char* renderVertexShader() const override {
        return kDesktopRenderVs;
    }
    const char* renderFragmentShader() const override {
        return kDesktopRenderFs;
    }
    const char* pointPickVertexShader() const override {
        return kDesktopPointPickVs;
    }
    const char* pointPickFragmentShader() const override {
        return kDesktopPointPickFs;
    }
    const char* meshPickVertexShader() const override {
        return kDesktopMeshPickVs;
    }
    const char* meshPickFragmentShader() const override {
        return kDesktopMeshPickFs;
    }
    const char* meshPickGeometryShader() const override {
        return kDesktopMeshPickGs;
    }
    void createVertexArrays(QOpenGLExtraFunctions& gl, Buffers& b) const override {
        createVaos(gl, b);
    }
    void destroyVertexArrays(QOpenGLExtraFunctions& gl, Buffers& b) const override {
        destroyVaos(gl, b);
    }
    void bindRenderLayout(QOpenGLExtraFunctions& gl, const Buffers& b) const override {
        bindRender(gl, b);
    }
    void bindPointPickLayout(QOpenGLExtraFunctions& gl, const Buffers& b) const override {
        bindPointPick(gl, b);
    }
    void bindMeshPickLayout(QOpenGLExtraFunctions& gl, const Buffers& b) const override {
        bindMeshPick(gl, b);
    }
    void bindMeshSelectionLayout(QOpenGLExtraFunctions& gl, const Buffers& b) const override {
        bindMeshSelection(gl, b);
    }
    void unbindLayout(QOpenGLExtraFunctions& gl) const override {
        unbind(gl);
    }
};

class Gles31Backend final : public BackendBase {
  public:
    QString name() const override {
        return QStringLiteral("OpenGL ES 3.1 / VAO / RGB24 Block Picking");
    }
    bool isGles() const override {
        return true;
    }
    bool gpuPickingSupported() const override {
        return false;
    }
    bool validateContext(QString* error) const override {
        auto* c = QOpenGLContext::currentContext();
        if (!c) {
            if (error)
                *error = QStringLiteral("没有当前 OpenGL Context");
            return false;
        }
        const auto f = c->format();
        if (!c->isOpenGLES() || f.majorVersion() < 3 || (f.majorVersion() == 3 && f.minorVersion() < 1)) {
            if (error)
                *error = QStringLiteral("GLES 后端需要 OpenGL ES 3.1+ Context");
            return false;
        }
        return true;
    }
    void configureContextState(QOpenGLExtraFunctions&) override {
        // GLES 3.1 原生支持 VAO。
        vaoSupported_ = true;
    }
    const char* renderVertexShader() const override {
        return kGlesRenderVs;
    }
    const char* renderFragmentShader() const override {
        return kGlesRenderFs;
    }
    const char* pointPickVertexShader() const override {
        return kGlesPointPickVs;
    }
    const char* pointPickFragmentShader() const override {
        return kGlesPointPickFs;
    }
    const char* meshPickVertexShader() const override {
        return kGlesMeshPickVs;
    }
    const char* meshPickFragmentShader() const override {
        return kGlesMeshPickFs;
    }
    const char* meshPickGeometryShader() const override {
        return nullptr;
    }
    void createVertexArrays(QOpenGLExtraFunctions& gl, Buffers& b) const override {
        createVaos(gl, b);
    }
    void destroyVertexArrays(QOpenGLExtraFunctions& gl, Buffers& b) const override {
        destroyVaos(gl, b);
    }
    void bindRenderLayout(QOpenGLExtraFunctions& gl, const Buffers& b) const override {
        bindRender(gl, b);
    }
    void bindPointPickLayout(QOpenGLExtraFunctions& gl, const Buffers& b) const override {
        bindPointPick(gl, b);
    }
    void bindMeshPickLayout(QOpenGLExtraFunctions& gl, const Buffers& b) const override {
        bindMeshPick(gl, b);
    }
    void bindMeshSelectionLayout(QOpenGLExtraFunctions& gl, const Buffers& b) const override {
        bindMeshSelection(gl, b);
    }
    void unbindLayout(QOpenGLExtraFunctions& gl) const override {
        unbind(gl);
    }
};

} // namespace

std::unique_ptr<IRenderBackend> createRenderBackend() {
#ifdef JMENGINE_RENDER_GLES31
    return std::make_unique<Gles31Backend>();
#else
    return std::make_unique<DesktopGl21Backend>();
#endif
}
