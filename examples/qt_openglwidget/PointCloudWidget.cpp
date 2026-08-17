#include "PointCloudWidget.h"

#include <JMEngine/CpuSelector.h>
#include <JMEngine/PixelIdPicker.h>
#include <JMEngine/PointCloudIO.h>

#include <QImage>
#include <QFileInfo>
#include <QKeyEvent>
#include <QMetaObject>
#include <QMouseEvent>
#include <QPainter>
#include <QPointer>
#include <QTimer>
#include <QString>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <limits>
#include <numeric>
#include <utility>

namespace {

// Core 只返回 UV/纹理路径；Qt 示例用 QImage 解码 PNG/JPG。
// 该步骤运行在后台线程，不会阻塞 UI。
bool bakeTextureWithQImage(const JMEngine::ObjAppearanceData& appearance,
                           JMEngine::PointCloud& cloud,
                           std::string& message)
{
    if (appearance.diffuseTexturePath.empty() || !appearance.hasTextureCoordinates()) return false;

    QImage image(QString::fromUtf8(appearance.diffuseTexturePath.c_str()));
    if (image.isNull()) {
        message += " | 纹理图片加载失败: " + appearance.diffuseTexturePath;
        return false;
    }
    image = image.convertToFormat(QImage::Format_RGBA8888);
    const int width = image.width();
    const int height = image.height();
    const qsizetype stride = image.bytesPerLine();
    const uchar* bits = image.constBits();
    if (width <= 0 || height <= 0 || bits == nullptr) return false;

    const std::size_t count = std::min(cloud.size(), appearance.vertexUv.size());
#ifdef JMENGINE_USE_OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (std::int64_t ii = 0; ii < static_cast<std::int64_t>(count); ++ii) {
        const std::size_t i = static_cast<std::size_t>(ii);
        if (i >= appearance.hasUv.size() || appearance.hasUv[i] == 0u) continue;
        const auto uv = appearance.vertexUv[i];
        const float u = std::clamp(uv.x, 0.0f, 1.0f);
        const float v = std::clamp(uv.y, 0.0f, 1.0f);
        const int x = std::clamp(static_cast<int>(std::lround(u * (width - 1))), 0, width - 1);
        const int y = std::clamp(static_cast<int>(std::lround((1.0f - v) * (height - 1))), 0, height - 1);
        const uchar* px = bits + static_cast<qsizetype>(y) * stride + x * 4;
        cloud.points()[i].rgba = static_cast<std::uint32_t>(px[0]) |
                                 (static_cast<std::uint32_t>(px[1]) << 8u) |
                                 (static_cast<std::uint32_t>(px[2]) << 16u) |
                                 (static_cast<std::uint32_t>(px[3]) << 24u);
    }
    message += " | 已烘焙 OBJ 纹理颜色";
    return true;
}

std::vector<JMEngine::PointId> unionSorted(const std::vector<JMEngine::PointId>& a,
                                           const std::vector<JMEngine::PointId>& b)
{
    std::vector<JMEngine::PointId> out;
    out.reserve(a.size() + b.size());
    std::set_union(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(out));
    return out;
}

std::vector<JMEngine::PointId> differenceSorted(const std::vector<JMEngine::PointId>& a,
                                                const std::vector<JMEngine::PointId>& b)
{
    std::vector<JMEngine::PointId> out;
    out.reserve(a.size());
    std::set_difference(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(out));
    return out;
}

constexpr const char* kColorVertex = R"GLSL(#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in uint aRgba;
layout(location = 2) in vec3 aNormal;
layout(location = 4) in uint aFlags;
layout(location = 5) in uint aSelected;

uniform mat4 uMVP;
uniform float uPointSize;
uniform uint uSelectionOnly;

out vec4 vColor;
out vec3 vNormal;
flat out uint vSelected;

const uint POINT_DELETED = 4u;

void main() {
    if ((aFlags & POINT_DELETED) != 0u || (uSelectionOnly != 0u && aSelected == 0u)) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        gl_PointSize = 1.0;
    } else {
        gl_Position = uMVP * vec4(aPosition, 1.0);
        gl_PointSize = uPointSize;
    }
    vColor = vec4(
        float( aRgba        & 255u) / 255.0,
        float((aRgba >> 8u) & 255u) / 255.0,
        float((aRgba >>16u) & 255u) / 255.0,
        float((aRgba >>24u) & 255u) / 255.0);
    vNormal = aNormal;
    vSelected = aSelected;
}
)GLSL";

constexpr const char* kColorFragment = R"GLSL(#version 330 core
in vec4 vColor;
in vec3 vNormal;
flat in uint vSelected;
out vec4 outColor;
uniform vec3 uLightDir;

void main() {
    vec2 q = gl_PointCoord * 2.0 - vec2(1.0);
    if (dot(q, q) > 1.0) discard;

    vec3 base = vColor.rgb;
    float nlen = length(vNormal);
    if (nlen > 0.001) {
        vec3 n = normalize(vNormal);
        float diffuse = max(dot(n, normalize(uLightDir)), 0.0);
        base *= (0.60 + 0.40 * diffuse);
    }
    if (vSelected != 0u) base = vec3(1.0, 0.48, 0.03);
    outColor = vec4(base, 1.0);
}
)GLSL";

constexpr const char* kMeshVertex = R"GLSL(#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in uint aRgba;
layout(location = 2) in vec3 aNormal;
layout(location = 4) in uint aFlags;
layout(location = 5) in uint aSelected;
uniform mat4 uMVP;

out VS_OUT {
    vec4 color;
    vec3 normal;
    flat uint flags;
    flat uint selected;
} vs;

void main() {
    gl_Position = uMVP * vec4(aPosition, 1.0);
    vs.color = vec4(
        float( aRgba        & 255u) / 255.0,
        float((aRgba >> 8u) & 255u) / 255.0,
        float((aRgba >>16u) & 255u) / 255.0,
        float((aRgba >>24u) & 255u) / 255.0);
    vs.normal = aNormal;
    vs.flags = aFlags;
    vs.selected = aSelected;
}
)GLSL";

constexpr const char* kMeshGeometry = R"GLSL(#version 330 core
layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

in VS_OUT {
    vec4 color;
    vec3 normal;
    flat uint flags;
    flat uint selected;
} gsIn[];

out vec4 gColor;
out vec3 gNormal;
out float gSelected;
const uint POINT_DELETED = 4u;

void main() {
    if ((gsIn[0].flags & POINT_DELETED) != 0u ||
        (gsIn[1].flags & POINT_DELETED) != 0u ||
        (gsIn[2].flags & POINT_DELETED) != 0u) return;

    float selected = float(gsIn[0].selected | gsIn[1].selected | gsIn[2].selected);
    for (int i = 0; i < 3; ++i) {
        gl_Position = gl_in[i].gl_Position;
        gColor = gsIn[i].color;
        gNormal = gsIn[i].normal;
        gSelected = selected;
        EmitVertex();
    }
    EndPrimitive();
}
)GLSL";

constexpr const char* kMeshFragment = R"GLSL(#version 330 core
in vec4 gColor;
in vec3 gNormal;
in float gSelected;
out vec4 outColor;
uniform vec3 uLightDir;

void main() {
    vec3 base = gColor.rgb;
    float nlen = length(gNormal);
    if (nlen > 0.001) {
        vec3 n = normalize(gNormal);
        float diffuse = max(dot(n, normalize(uLightDir)), 0.0);
        base *= (0.58 + 0.42 * diffuse);
    }
    if (gSelected > 0.15) base = mix(base, vec3(1.0, 0.48, 0.03), 0.55);
    outColor = vec4(base, 1.0);
}
)GLSL";

// 点云表面选择：每个可见点写入 PointId。
constexpr const char* kPointPickVertex = R"GLSL(#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 3) in uint aPointId;
layout(location = 4) in uint aFlags;
uniform mat4 uMVP;
uniform float uPointSize;
flat out uint vPointId;
const uint POINT_DELETED = 4u;
void main() {
    if ((aFlags & POINT_DELETED) != 0u) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        gl_PointSize = 1.0;
    } else {
        gl_Position = uMVP * vec4(aPosition, 1.0);
        gl_PointSize = uPointSize;
    }
    vPointId = aPointId;
}
)GLSL";

constexpr const char* kPointPickFragment = R"GLSL(#version 330 core
flat in uint vPointId;
layout(location = 0) out uint outId;
void main() {
    vec2 q = gl_PointCoord * 2.0 - vec2(1.0);
    if (dot(q, q) > 1.0) discard;
    outId = vPointId;
}
)GLSL";

// OBJ 表面选择：不是再画“顶点圆点”，而是真实 rasterize 三角面。
// 每个像素保存 TriangleId，随后 CPU 把选中的三角面展开成完整 3 个顶点。
constexpr const char* kMeshPickVertex = R"GLSL(#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 4) in uint aFlags;
uniform mat4 uMVP;
flat out uint vFlags;
void main() {
    gl_Position = uMVP * vec4(aPosition, 1.0);
    vFlags = aFlags;
}
)GLSL";

constexpr const char* kMeshPickGeometry = R"GLSL(#version 330 core
layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;
flat in uint vFlags[];
flat out uint gTriangleId;
const uint POINT_DELETED = 4u;
void main() {
    if ((vFlags[0] & POINT_DELETED) != 0u ||
        (vFlags[1] & POINT_DELETED) != 0u ||
        (vFlags[2] & POINT_DELETED) != 0u) return;
    gTriangleId = uint(gl_PrimitiveIDIn);
    for (int i = 0; i < 3; ++i) {
        gl_Position = gl_in[i].gl_Position;
        EmitVertex();
    }
    EndPrimitive();
}
)GLSL";

constexpr const char* kMeshPickFragment = R"GLSL(#version 330 core
flat in uint gTriangleId;
layout(location = 0) out uint outId;
void main() { outId = gTriangleId; }
)GLSL";

} // namespace

PointCloudWidget::SceneModel::SceneModel(QString modelPath,
                                         std::shared_ptr<JMEngine::PointCloud> modelCloud,
                                         JMEngine::ObjMeshData modelMesh,
                                         bool isMesh)
    : path(std::move(modelPath)),
      cloud(std::move(modelCloud)),
      editor(cloud),
      mesh(std::move(modelMesh)),
      meshMode(isMesh)
{
    if (cloud) {
        selectedMask.assign(cloud->size(), 0u);
        cachedActiveCount = cloud->activeCount();
    }
}

PointCloudWidget::PointCloudWidget(const std::string& fileName, QWidget* parent)
    : QOpenGLWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    setMinimumSize(960, 640);
    workerPool_.setMaxThreadCount(1);
    statusText_ = "空场景：请打开 OBJ 或 PLY 模型";

    if (!fileName.empty()) {
        QTimer::singleShot(0, this, [this, fileName] {
            loadModelAsync(QString::fromStdString(fileName));
        });
    }
}

PointCloudWidget::~PointCloudWidget() {
    workerPool_.waitForDone();
    makeCurrent();
    for (auto& m : models_) destroySceneModelGl(*m);
    destroyPickingObjects();
    doneCurrent();
}

PointCloudWidget::SceneModel* PointCloudWidget::activeModel() {
    if (activeModel_ < 0 || activeModel_ >= static_cast<int>(models_.size())) return nullptr;
    return models_[static_cast<std::size_t>(activeModel_)].get();
}

const PointCloudWidget::SceneModel* PointCloudWidget::activeModel() const {
    if (activeModel_ < 0 || activeModel_ >= static_cast<int>(models_.size())) return nullptr;
    return models_[static_cast<std::size_t>(activeModel_)].get();
}

int PointCloudWidget::findModel(const QString& path) const {
    const QString key = QFileInfo(path).absoluteFilePath();
    for (int i = 0; i < static_cast<int>(models_.size()); ++i) {
        if (QFileInfo(models_[static_cast<std::size_t>(i)]->path).absoluteFilePath() == key) return i;
    }
    return -1;
}

QString PointCloudWidget::activeModelPath() const {
    const auto* m = activeModel();
    return m ? m->path : QString{};
}

void PointCloudWidget::setInteractionMode(InteractionMode mode) { setMode(mode); }

void PointCloudWidget::setSelectionDepthMode(SelectionDepthMode mode) {
    depthMode_ = mode;
    statusText_ = std::string("选择深度: ") + depthModeName();
    update();
}

void PointCloudWidget::deleteSelection() {
    auto* m = activeModel();
    if (!m) return;
    deleteIdsAsync(m->selectedIds);
}

void PointCloudWidget::clearCurrentSelection() {
    auto* m = activeModel();
    if (!m || editBusy_ || pickBusy_) return;
    m->editor.clearSelection();
    clearSelectionVisual(*m);
    statusText_ = "已清除选择";
}

void PointCloudWidget::undoEdit() { undoAsync(); }
void PointCloudWidget::redoEdit() { redoAsync(); }

void PointCloudWidget::fitView() {
    auto* m = activeModel();
    if (!m || !m->cloud || editBusy_) return;
    camera_.fit(*m->cloud);
    update();
}

void PointCloudWidget::saveModel() { saveAsync(); }

bool PointCloudWidget::activateModel(const QString& fileName) {
    if (editBusy_ || pickBusy_) return false;
    const int index = findModel(fileName);
    if (index < 0) return false;
    activeModel_ = index;
    auto* m = activeModel();
    if (m && m->cloud) camera_.fit(*m->cloud);
    statusText_ = "已激活模型: " + m->path.toStdString();
    update();
    return true;
}

void PointCloudWidget::setModelVisible(const QString& fileName, bool visible) {
    const int index = findModel(fileName);
    if (index < 0) return;
    models_[static_cast<std::size_t>(index)]->visible = visible;
    update();
}

void PointCloudWidget::removeModel(const QString& fileName) {
    if (editBusy_ || pickBusy_) return;
    const int index = findModel(fileName);
    if (index < 0) return;

    if (isValid()) makeCurrent();
    destroySceneModelGl(*models_[static_cast<std::size_t>(index)]);
    if (isValid()) doneCurrent();

    models_.erase(models_.begin() + index);
    if (models_.empty()) {
        activeModel_ = -1;
        statusText_ = "空场景：请打开 OBJ 或 PLY 模型";
    } else {
        if (activeModel_ > index) --activeModel_;
        else if (activeModel_ == index) activeModel_ = std::min(index, static_cast<int>(models_.size()) - 1);
        if (auto* m = activeModel(); m && m->cloud) camera_.fit(*m->cloud);
    }
    update();
}

void PointCloudWidget::clearModel() {
    if (editBusy_ || pickBusy_) return;
    if (isValid()) makeCurrent();
    for (auto& m : models_) destroySceneModelGl(*m);
    if (isValid()) doneCurrent();
    models_.clear();
    activeModel_ = -1;
    statusText_ = "空场景：请打开 OBJ 或 PLY 模型";
    update();
}

void PointCloudWidget::loadModelAsync(const QString& fileName) {
    if (fileName.isEmpty()) return;
    const QString absolute = QFileInfo(fileName).absoluteFilePath();
    if (activateModel(absolute)) return;
    if (std::find(loadingPaths_.begin(), loadingPaths_.end(), absolute) != loadingPaths_.end()) {
        statusText_ = "模型仍在后台加载: " + absolute.toStdString();
        update();
        return;
    }
    loadingPaths_.push_back(absolute);

    loadBusy_ = true;
    statusText_ = "后台解析模型(OpenMP): " + absolute.toStdString();
    update();

    const std::string path = absolute.toStdString();
    QPointer<PointCloudWidget> self(this);
    workerPool_.start([self, path, absolute]() {
        std::shared_ptr<JMEngine::PointCloud> loaded;
        JMEngine::ObjMeshData mesh;
        bool meshMode = false;
        std::string error;
        std::string message;

        std::string ext = std::filesystem::path(path).extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });

        if (ext == ".obj") {
            JMEngine::ObjModelData model;
            if (!JMEngine::ObjModelLoader::load(path, model, &message)) {
                error = message;
            } else {
                loaded = std::move(model.cloud);
                mesh = std::move(model.mesh);
                meshMode = !mesh.empty();
                if (loaded) bakeTextureWithQImage(model.appearance, *loaded, message);
            }
        } else {
            loaded = JMEngine::PointCloudIO::load(path, &error);
            if (loaded) message = "PLY 点云模式";
        }

        if (!self) return;
        QMetaObject::invokeMethod(self,
            [self, absolute, loaded = std::move(loaded), mesh = std::move(mesh), meshMode,
             error = std::move(error), message = std::move(message)]() mutable {
                if (!self) return;
                self->loadingPaths_.erase(
                    std::remove(self->loadingPaths_.begin(), self->loadingPaths_.end(), absolute),
                    self->loadingPaths_.end());
                self->loadBusy_ = !self->loadingPaths_.empty();
                if (!loaded) {
                    self->statusText_ = "加载失败: " + error;
                    self->update();
                    return;
                }

                auto doc = std::make_unique<SceneModel>(absolute, std::move(loaded), std::move(mesh), meshMode);
                if (self->isValid()) {
                    self->makeCurrent();
                    self->createSceneModelGl(*doc);
                    self->doneCurrent();
                }
                self->models_.push_back(std::move(doc));
                self->activeModel_ = static_cast<int>(self->models_.size()) - 1;
                if (auto* m = self->activeModel(); m && m->cloud) self->camera_.fit(*m->cloud);
                self->statusText_ = message.empty() ? "模型加载完成" : message;
                self->update();
            }, Qt::QueuedConnection);
    });
}

bool PointCloudWidget::createPrograms() {
    if (!colorProgram_.addShaderFromSourceCode(QOpenGLShader::Vertex, kColorVertex) ||
        !colorProgram_.addShaderFromSourceCode(QOpenGLShader::Fragment, kColorFragment) ||
        !colorProgram_.link()) {
        std::cerr << "颜色 Shader 编译失败: " << colorProgram_.log().toStdString() << '\n';
        return false;
    }
    if (!meshProgram_.addShaderFromSourceCode(QOpenGLShader::Vertex, kMeshVertex) ||
        !meshProgram_.addShaderFromSourceCode(QOpenGLShader::Geometry, kMeshGeometry) ||
        !meshProgram_.addShaderFromSourceCode(QOpenGLShader::Fragment, kMeshFragment) ||
        !meshProgram_.link()) {
        std::cerr << "网格 Shader 编译失败: " << meshProgram_.log().toStdString() << '\n';
        return false;
    }
    if (!pointPickProgram_.addShaderFromSourceCode(QOpenGLShader::Vertex, kPointPickVertex) ||
        !pointPickProgram_.addShaderFromSourceCode(QOpenGLShader::Fragment, kPointPickFragment) ||
        !pointPickProgram_.link()) {
        std::cerr << "点 Picking Shader 编译失败: " << pointPickProgram_.log().toStdString() << '\n';
        return false;
    }
    if (!meshPickProgram_.addShaderFromSourceCode(QOpenGLShader::Vertex, kMeshPickVertex) ||
        !meshPickProgram_.addShaderFromSourceCode(QOpenGLShader::Geometry, kMeshPickGeometry) ||
        !meshPickProgram_.addShaderFromSourceCode(QOpenGLShader::Fragment, kMeshPickFragment) ||
        !meshPickProgram_.link()) {
        std::cerr << "网格 Picking Shader 编译失败: " << meshPickProgram_.log().toStdString() << '\n';
        return false;
    }
    return true;
}

void PointCloudWidget::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_BLEND);
    if (!createPrograms()) return;

    glGenBuffers(1, &pickPbo_);
    for (auto& m : models_) createSceneModelGl(*m);

    const qreal dpr = devicePixelRatioF();
    createPickingFramebuffer(
        std::max(1, static_cast<int>(width() * dpr)),
        std::max(1, static_cast<int>(height() * dpr)));
}

void PointCloudWidget::resizeGL(int w, int h) {
    const qreal dpr = devicePixelRatioF();
    createPickingFramebuffer(
        std::max(1, static_cast<int>(w * dpr)),
        std::max(1, static_cast<int>(h * dpr)));
}

bool PointCloudWidget::createPickingFramebuffer(int width, int height) {
    if (width <= 0 || height <= 0) return false;
    if (pickFbo_ != 0 && width == pickWidth_ && height == pickHeight_) return true;

    if (pickDepth_) glDeleteRenderbuffers(1, &pickDepth_);
    if (pickTexture_) glDeleteTextures(1, &pickTexture_);
    if (pickFbo_) glDeleteFramebuffers(1, &pickFbo_);
    pickDepth_ = pickTexture_ = pickFbo_ = 0;
    pickWidth_ = width;
    pickHeight_ = height;

    glGenFramebuffers(1, &pickFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, pickFbo_);
    glGenTextures(1, &pickTexture_);
    glBindTexture(GL_TEXTURE_2D, pickTexture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32UI, width, height, 0,
                 GL_RED_INTEGER, GL_UNSIGNED_INT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, pickTexture_, 0);

    glGenRenderbuffers(1, &pickDepth_);
    glBindRenderbuffer(GL_RENDERBUFFER, pickDepth_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                              GL_RENDERBUFFER, pickDepth_);

    const bool ok = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());
    if (!ok) std::cerr << "创建 R32UI Picking FBO 失败。\n";
    return ok;
}

void PointCloudWidget::createSceneModelGl(SceneModel& m) {
    if (!m.cloud || m.vao != 0) return;
    const std::size_t count = m.cloud->size();
    m.gpuPointCapacity = static_cast<GLsizei>(std::min<std::size_t>(
        count, static_cast<std::size_t>(std::numeric_limits<GLsizei>::max())));
    m.gpuPointCount = 0;
    m.gpuMeshIndexCount = 0;
    m.gpuUploadCursor = 0;
    m.meshUploadCursor = 0;

    glGenVertexArrays(1, &m.vao);
    glGenBuffers(1, &m.positionVbo);
    glGenBuffers(1, &m.colorVbo);
    glGenBuffers(1, &m.normalVbo);
    glGenBuffers(1, &m.pointIdVbo);
    glGenBuffers(1, &m.flagsVbo);
    glGenBuffers(1, &m.selectionVbo);
    glGenBuffers(1, &m.meshEbo);

    glBindVertexArray(m.vao);

    glBindBuffer(GL_ARRAY_BUFFER, m.positionVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(count * 3 * sizeof(float)), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, m.colorVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(count * sizeof(std::uint32_t)), nullptr, GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribIPointer(1, 1, GL_UNSIGNED_INT, 0, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, m.normalVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(count * 3 * sizeof(float)), nullptr, GL_STATIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, m.pointIdVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(count * sizeof(std::uint32_t)), nullptr, GL_STATIC_DRAW);
    glEnableVertexAttribArray(3);
    glVertexAttribIPointer(3, 1, GL_UNSIGNED_INT, 0, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, m.flagsVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(count * sizeof(std::uint32_t)), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(4);
    glVertexAttribIPointer(4, 1, GL_UNSIGNED_INT, 0, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, m.selectionVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(count * sizeof(std::uint32_t)), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(5);
    glVertexAttribIPointer(5, 1, GL_UNSIGNED_INT, 0, nullptr);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.meshEbo);
    const std::size_t indexCount = m.meshMode ? m.mesh.triangleIndices.size() : 0u;
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(indexCount * sizeof(std::uint32_t)), nullptr, GL_STATIC_DRAW);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PointCloudWidget::destroySceneModelGl(SceneModel& m) {
    if (!m.vao) return;
    const GLuint buffers[] = {m.positionVbo, m.colorVbo, m.normalVbo, m.pointIdVbo,
                              m.flagsVbo, m.selectionVbo, m.meshEbo};
    glDeleteBuffers(7, buffers);
    glDeleteVertexArrays(1, &m.vao);
    m.vao = m.positionVbo = m.colorVbo = m.normalVbo = m.pointIdVbo = 0;
    m.flagsVbo = m.selectionVbo = m.meshEbo = 0;
}

void PointCloudWidget::destroyPickingObjects() {
    if (pickFence_) { glDeleteSync(pickFence_); pickFence_ = nullptr; }
    if (pickPbo_) glDeleteBuffers(1, &pickPbo_);
    if (pickDepth_) glDeleteRenderbuffers(1, &pickDepth_);
    if (pickTexture_) glDeleteTextures(1, &pickTexture_);
    if (pickFbo_) glDeleteFramebuffers(1, &pickFbo_);
    pickPbo_ = pickDepth_ = pickTexture_ = pickFbo_ = 0;
}

void PointCloudWidget::processInitialGpuUpload(SceneModel& m, std::size_t& byteBudget) {
    if (!m.cloud || !m.vao || byteBudget == 0) return;
    const std::size_t total = m.cloud->size();

    if (m.gpuUploadCursor < total) {
        // 每点约 44 字节上传数据，按预算决定本帧 chunk，最低保证 1 点。
        constexpr std::size_t bytesPerPoint = 44;
        const std::size_t maxPoints = std::max<std::size_t>(1, byteBudget / bytesPerPoint);
        const std::size_t n = std::min(maxPoints, total - m.gpuUploadCursor);
        const std::size_t first = m.gpuUploadCursor;

        std::vector<float> pos(n * 3), normal(n * 3);
        std::vector<std::uint32_t> color(n), ids(n), flags(n), selected(n);
        for (std::size_t i = 0; i < n; ++i) {
            const auto& p = m.cloud->points()[first + i];
            pos[i*3+0] = p.position.x; pos[i*3+1] = p.position.y; pos[i*3+2] = p.position.z;
            normal[i*3+0] = p.normal.x; normal[i*3+1] = p.normal.y; normal[i*3+2] = p.normal.z;
            color[i] = p.rgba;
            ids[i] = static_cast<std::uint32_t>(first + i);
            flags[i] = p.flags;
            selected[i] = first + i < m.selectedMask.size() ? m.selectedMask[first+i] : 0u;
        }

        auto upload = [&](GLuint buffer, std::size_t offset, std::size_t bytes, const void* data) {
            glBindBuffer(GL_ARRAY_BUFFER, buffer);
            glBufferSubData(GL_ARRAY_BUFFER, static_cast<GLintptr>(offset), static_cast<GLsizeiptr>(bytes), data);
        };
        upload(m.positionVbo, first*3*sizeof(float), pos.size()*sizeof(float), pos.data());
        upload(m.colorVbo, first*sizeof(std::uint32_t), color.size()*sizeof(std::uint32_t), color.data());
        upload(m.normalVbo, first*3*sizeof(float), normal.size()*sizeof(float), normal.data());
        upload(m.pointIdVbo, first*sizeof(std::uint32_t), ids.size()*sizeof(std::uint32_t), ids.data());
        upload(m.flagsVbo, first*sizeof(std::uint32_t), flags.size()*sizeof(std::uint32_t), flags.data());
        upload(m.selectionVbo, first*sizeof(std::uint32_t), selected.size()*sizeof(std::uint32_t), selected.data());
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        m.gpuUploadCursor += n;
        m.gpuPointCount = static_cast<GLsizei>(m.gpuUploadCursor);
        const std::size_t used = n * bytesPerPoint;
        byteBudget = used >= byteBudget ? 0 : byteBudget - used;
    }

    if (m.meshMode && m.meshUploadCursor < m.mesh.triangleIndices.size() && byteBudget > 0) {
        const std::size_t bytesPerIndex = sizeof(std::uint32_t);
        std::size_t n = std::max<std::size_t>(3, byteBudget / bytesPerIndex);
        n = std::min(n, m.mesh.triangleIndices.size() - m.meshUploadCursor);
        n -= n % 3; // 只提交完整三角形
        if (n >= 3) {
            // Core Profile 中 GL_ELEMENT_ARRAY_BUFFER 是 VAO 状态，上传 EBO 前必须绑定所属 VAO。
            glBindVertexArray(m.vao);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.meshEbo);
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,
                            static_cast<GLintptr>(m.meshUploadCursor * sizeof(std::uint32_t)),
                            static_cast<GLsizeiptr>(n * sizeof(std::uint32_t)),
                            m.mesh.triangleIndices.data() + m.meshUploadCursor);
            glBindVertexArray(0);
            m.meshUploadCursor += n;
            m.gpuMeshIndexCount = static_cast<GLsizei>(m.meshUploadCursor);
            const std::size_t used = n * bytesPerIndex;
            byteBudget = used >= byteBudget ? 0 : byteBudget - used;
        }
    }
}

void PointCloudWidget::queueDirtyIds(SceneModel& m,
                                     const std::vector<JMEngine::PointId>& ids,
                                     DirtyBuffer buffer)
{
    if (ids.empty()) return;
    std::vector<JMEngine::PointId> sorted = ids;
    std::sort(sorted.begin(), sorted.end());
    sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
    JMEngine::PointId begin = sorted.front();
    JMEngine::PointId prev = begin;
    for (std::size_t i = 1; i < sorted.size(); ++i) {
        if (sorted[i] <= prev + 1u) { prev = sorted[i]; continue; }
        m.dirtyRanges.push_back({begin, prev, buffer});
        begin = prev = sorted[i];
    }
    m.dirtyRanges.push_back({begin, prev, buffer});
}

void PointCloudWidget::processDirtyGpuUpdates(SceneModel& m, std::size_t& byteBudget) {
    if (!m.cloud || !m.vao) return;
    while (!m.dirtyRanges.empty() && byteBudget > 0) {
        auto& range = m.dirtyRanges.front();
        if (range.begin >= m.cloud->size()) { m.dirtyRanges.pop_front(); continue; }
        const std::size_t first = range.begin;
        const std::size_t available = std::min<std::size_t>(
            static_cast<std::size_t>(range.end - range.begin) + 1u, m.cloud->size() - first);
        const std::size_t bytesPerPoint = range.buffer == DirtyBuffer::Position ? 3*sizeof(float) : sizeof(std::uint32_t);
        const std::size_t n = std::min<std::size_t>(available, std::max<std::size_t>(1, byteBudget / bytesPerPoint));

        if (range.buffer == DirtyBuffer::Flags) {
            std::vector<std::uint32_t> data(n);
            for (std::size_t i=0;i<n;++i) data[i]=m.cloud->points()[first+i].flags;
            glBindBuffer(GL_ARRAY_BUFFER, m.flagsVbo);
            glBufferSubData(GL_ARRAY_BUFFER, static_cast<GLintptr>(first*sizeof(std::uint32_t)),
                            static_cast<GLsizeiptr>(n*sizeof(std::uint32_t)), data.data());
        } else if (range.buffer == DirtyBuffer::Selection) {
            glBindBuffer(GL_ARRAY_BUFFER, m.selectionVbo);
            glBufferSubData(GL_ARRAY_BUFFER, static_cast<GLintptr>(first*sizeof(std::uint32_t)),
                            static_cast<GLsizeiptr>(n*sizeof(std::uint32_t)), m.selectedMask.data()+first);
        } else {
            std::vector<float> data(n*3);
            for (std::size_t i=0;i<n;++i) {
                const auto& p=m.cloud->points()[first+i].position;
                data[i*3]=p.x; data[i*3+1]=p.y; data[i*3+2]=p.z;
            }
            glBindBuffer(GL_ARRAY_BUFFER, m.positionVbo);
            glBufferSubData(GL_ARRAY_BUFFER, static_cast<GLintptr>(first*3*sizeof(float)),
                            static_cast<GLsizeiptr>(data.size()*sizeof(float)), data.data());
        }
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        const std::size_t used=n*bytesPerPoint;
        byteBudget=used>=byteBudget?0:byteBudget-used;
        if (n==available) m.dirtyRanges.pop_front();
        else range.begin=static_cast<JMEngine::PointId>(range.begin+n);
    }
}

JMEngine::Mat4f PointCloudWidget::currentMvp() const {
    const qreal dpr=devicePixelRatioF();
    return camera_.mvp(std::max(1,static_cast<int>(width()*dpr)),
                       std::max(1,static_cast<int>(height()*dpr)));
}

void PointCloudWidget::drawModel(SceneModel& m, bool active) {
    if (!m.visible || !m.vao || m.gpuPointCount<=0) return;
    const auto mvp=currentMvp();
    glBindVertexArray(m.vao);
    if (m.meshMode && m.gpuMeshIndexCount>0) {
        meshProgram_.bind();
        glUniformMatrix4fv(meshProgram_.uniformLocation("uMVP"),1,GL_FALSE,mvp.m.data());
        glUniform3f(meshProgram_.uniformLocation("uLightDir"),-0.35f,0.72f,0.62f);
        glDisable(GL_CULL_FACE);
        glDrawElements(GL_TRIANGLES,m.gpuMeshIndexCount,GL_UNSIGNED_INT,nullptr);
        meshProgram_.release();
    } else {
        colorProgram_.bind();
        glUniformMatrix4fv(colorProgram_.uniformLocation("uMVP"),1,GL_FALSE,mvp.m.data());
        glUniform1f(colorProgram_.uniformLocation("uPointSize"),2.6f*static_cast<float>(devicePixelRatioF()));
        glUniform1ui(colorProgram_.uniformLocation("uSelectionOnly"),0u);
        glUniform3f(colorProgram_.uniformLocation("uLightDir"),-0.3f,0.7f,0.65f);
        glDrawArrays(GL_POINTS,0,m.gpuPointCount);
        colorProgram_.release();
    }
    glBindVertexArray(0);
    if (active) drawSelectedOverlay(m);
}

void PointCloudWidget::drawSelectedOverlay(SceneModel& m) {
    if (m.selectedIds.empty() || !m.vao) return;
    // 网格已经通过 mesh shader 把选中三角面染成橙色；再叠加小点可明确顶点范围。
    const auto mvp=currentMvp();
    colorProgram_.bind();
    glUniformMatrix4fv(colorProgram_.uniformLocation("uMVP"),1,GL_FALSE,mvp.m.data());
    glUniform1f(colorProgram_.uniformLocation("uPointSize"),5.5f*static_cast<float>(devicePixelRatioF()));
    glUniform1ui(colorProgram_.uniformLocation("uSelectionOnly"),1u);
    glUniform3f(colorProgram_.uniformLocation("uLightDir"),-0.3f,0.7f,0.65f);
    glBindVertexArray(m.vao);
    glDrawArrays(GL_POINTS,0,m.gpuPointCount);
    glBindVertexArray(0);
    colorProgram_.release();
}

void PointCloudWidget::drawScene() {
    glBindFramebuffer(GL_FRAMEBUFFER,defaultFramebufferObject());
    const qreal dpr=devicePixelRatioF();
    glViewport(0,0,std::max(1,static_cast<int>(width()*dpr)),std::max(1,static_cast<int>(height()*dpr)));
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glClearColor(0.075f,0.075f,0.085f,1.0f);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    for (int i=0;i<static_cast<int>(models_.size());++i)
        drawModel(*models_[static_cast<std::size_t>(i)], i==activeModel_);
}

void PointCloudWidget::paintGL() {
    // 总预算而不是“每模型一个预算”，防止同时加载多个千万点模型时一帧上传数百 MB。
    std::size_t uploadBudget=12u*1024u*1024u;
    for (auto& m:models_) {
        if (uploadBudget==0) break;
        processInitialGpuUpload(*m,uploadBudget);
    }
    std::size_t dirtyBudget=4u*1024u*1024u;
    for (auto& m:models_) {
        if (dirtyBudget==0) break;
        processDirtyGpuUpdates(*m,dirtyBudget);
    }
    drawScene();
    drawOverlay();

    bool more=false;
    for (const auto& m:models_) {
        if ((m->cloud && m->gpuUploadCursor<m->cloud->size()) ||
            (m->meshMode && m->meshUploadCursor<m->mesh.triangleIndices.size()) ||
            !m->dirtyRanges.empty()) { more=true; break; }
    }
    if (more) update();
}

void PointCloudWidget::drawPointPickingPass(SceneModel& m) {
    const auto mvp=currentMvp();
    pointPickProgram_.bind();
    glUniformMatrix4fv(pointPickProgram_.uniformLocation("uMVP"),1,GL_FALSE,mvp.m.data());
    glUniform1f(pointPickProgram_.uniformLocation("uPointSize"),3.2f*static_cast<float>(devicePixelRatioF()));
    glBindVertexArray(m.vao);
    glDrawArrays(GL_POINTS,0,m.gpuPointCount);
    glBindVertexArray(0);
    pointPickProgram_.release();
}

void PointCloudWidget::drawMeshPickingPass(SceneModel& m) {
    const auto mvp=currentMvp();
    meshPickProgram_.bind();
    glUniformMatrix4fv(meshPickProgram_.uniformLocation("uMVP"),1,GL_FALSE,mvp.m.data());
    glDisable(GL_CULL_FACE);
    glBindVertexArray(m.vao);
    glDrawElements(GL_TRIANGLES,m.gpuMeshIndexCount,GL_UNSIGNED_INT,nullptr);
    glBindVertexArray(0);
    meshPickProgram_.release();
}

void PointCloudWidget::drawOverlay() {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing,true);
    painter.setPen(QColor(235,235,235));
    painter.setBrush(QColor(0,0,0,150));
    painter.drawRoundedRect(QRect(12,12,720,92),6,6);

    const auto* m=activeModel();
    const qulonglong activeCount=m?static_cast<qulonglong>(m->cachedActiveCount):0;
    const qulonglong selected=m?static_cast<qulonglong>(m->selectedIds.size()):0;
    painter.drawText(24,36,QString::fromUtf8("活动模型: %1 | 模式: %2 | 深度: %3 | 点: %4 | 选择: %5")
        .arg(m?QFileInfo(m->path).fileName():QString::fromUtf8("无"))
        .arg(QString::fromUtf8(modeName())).arg(QString::fromUtf8(depthModeName()))
        .arg(activeCount).arg(selected));
    painter.drawText(24,60,QString::fromUtf8("左键绕模型中心旋转 | 右/中键平移 | 滚轮缩放 | Del 删除高亮选择"));
    painter.drawText(24,84,QString::fromUtf8(statusText_.c_str()));

    // 普通左键 Orbit 也会设置 dragging_，所以不能仅凭 dragging_ 绘制选择框。
    // 只有 Ctrl+左键已经锁定为编辑手势时，才允许显示选择预览。
    if (!dragging_ || !editGestureActive_ || mode_==InteractionMode::View) return;
    QPen pen(QColor(255,205,32)); pen.setWidth(2);
    painter.setPen(pen); painter.setBrush(QColor(255,205,32,30));
    if (mode_==InteractionMode::Rectangle) {
        painter.drawRect(QRect(pressPos_,currentPos_).normalized());
    } else if (mode_==InteractionMode::Circle) {
        const int dx=currentPos_.x()-pressPos_.x(); const int dy=currentPos_.y()-pressPos_.y();
        const int r=static_cast<int>(std::sqrt(static_cast<double>(dx*dx+dy*dy)));
        painter.drawEllipse(pressPos_,r,r);
    } else if (mode_==InteractionMode::Lasso && stroke_.size()>=2) {
        QPolygon poly; for (const auto& p:stroke_) poly<<p; painter.drawPolyline(poly);
    } else if (mode_==InteractionMode::BrushSelect && !stroke_.empty()) {
        QPen bp(QColor(255,80,80,210)); bp.setWidth(std::max(2,brushRadiusPixels_*2));
        bp.setCapStyle(Qt::RoundCap); bp.setJoinStyle(Qt::RoundJoin); painter.setPen(bp);
        QPolygon poly; for (const auto& p:stroke_) poly<<p;
        if (stroke_.size()==1) painter.drawPoint(stroke_.front()); else painter.drawPolyline(poly);
    }
}

QPoint PointCloudWidget::toPhysical(const QPoint& logical) const {
    const qreal dpr=devicePixelRatioF();
    return {static_cast<int>(std::lround(logical.x()*dpr)),static_cast<int>(std::lround(logical.y()*dpr))};
}

PointCloudWidget::PendingPick PointCloudWidget::buildPendingPick(Qt::KeyboardModifiers modifiers,
                                                                 bool physical) const {
    PendingPick p;
    p.mode=mode_; p.modifiers=modifiers;
    p.framebufferWidth=physical?pickWidth_:std::max(1,width());
    p.framebufferHeight=physical?pickHeight_:std::max(1,height());
    p.brushRadiusPhysical=physical?std::max(1,static_cast<int>(std::lround(brushRadiusPixels_*devicePixelRatioF()))):brushRadiusPixels_;
    auto cvt=[this,physical](const QPoint& q){ return physical?toPhysical(q):q; };
    p.press=cvt(pressPos_); p.current=cvt(currentPos_);
    p.stroke.reserve(stroke_.size()); for(const auto& q:stroke_) p.stroke.push_back(cvt(q));

    int left=0,right=-1,top=0,bottom=-1;
    if(mode_==InteractionMode::Rectangle){
        left=std::min(p.press.x(),p.current.x()); right=std::max(p.press.x(),p.current.x());
        top=std::min(p.press.y(),p.current.y()); bottom=std::max(p.press.y(),p.current.y());
    } else if(mode_==InteractionMode::Circle){
        const int dx=p.current.x()-p.press.x(),dy=p.current.y()-p.press.y();
        const int r=static_cast<int>(std::lround(std::sqrt(static_cast<double>(dx*dx+dy*dy))));
        left=p.press.x()-r; right=p.press.x()+r; top=p.press.y()-r; bottom=p.press.y()+r;
    } else if(!p.stroke.empty()){
        left=right=p.stroke.front().x(); top=bottom=p.stroke.front().y();
        for(const auto&q:p.stroke){left=std::min(left,q.x());right=std::max(right,q.x());top=std::min(top,q.y());bottom=std::max(bottom,q.y());}
        if(mode_==InteractionMode::BrushSelect){const int r=p.brushRadiusPhysical;left-=r;right+=r;top-=r;bottom+=r;}
    }
    left=std::clamp(left,0,p.framebufferWidth-1); right=std::clamp(right,0,p.framebufferWidth-1);
    top=std::clamp(top,0,p.framebufferHeight-1); bottom=std::clamp(bottom,0,p.framebufferHeight-1);
    if(right>=left&&bottom>=top){p.left=left;p.top=top;p.width=right-left+1;p.height=bottom-top+1;p.readY=p.framebufferHeight-top-p.height;}
    return p;
}

void PointCloudWidget::startSelection(Qt::KeyboardModifiers modifiers) {
    if(depthMode_==SelectionDepthMode::Surface) startSurfacePickAsync(modifiers);
    else startThroughPickAsync(modifiers);
}

void PointCloudWidget::startSurfacePickAsync(Qt::KeyboardModifiers modifiers) {
    auto* m=activeModel();
    if(!m||!pickFbo_||!pickPbo_||pickBusy_||editBusy_) return;
    if(m->cloud && m->gpuUploadCursor<m->cloud->size()) {statusText_="活动模型仍在 GPU 分块上传";update();return;}
    if(m->meshMode && m->meshUploadCursor<m->mesh.triangleIndices.size()) {statusText_="网格索引仍在 GPU 分块上传";update();return;}

    PendingPick p=buildPendingPick(modifiers,true);
    if(p.width<=0||p.height<=0)return;
    p.meshIds=m->meshMode;
    const std::size_t pixelCount=static_cast<std::size_t>(p.width)*p.height;

    makeCurrent();
    std::size_t dirtyBudget=32u*1024u*1024u; processDirtyGpuUpdates(*m,dirtyBudget);
    glBindFramebuffer(GL_FRAMEBUFFER,pickFbo_);
    glDrawBuffer(GL_COLOR_ATTACHMENT0); glReadBuffer(GL_COLOR_ATTACHMENT0);
    glViewport(0,0,pickWidth_,pickHeight_);
    glDisable(GL_BLEND); glDisable(GL_SCISSOR_TEST); glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LEQUAL); glDepthMask(GL_TRUE);
    const GLuint clearId=JMEngine::kInvalidPointId; glClearBufferuiv(GL_COLOR,0,&clearId); glClear(GL_DEPTH_BUFFER_BIT);
    if(m->meshMode) drawMeshPickingPass(*m); else drawPointPickingPass(*m);

    glPixelStorei(GL_PACK_ALIGNMENT,4);
    glBindBuffer(GL_PIXEL_PACK_BUFFER,pickPbo_);
    glBufferData(GL_PIXEL_PACK_BUFFER,static_cast<GLsizeiptr>(pixelCount*sizeof(std::uint32_t)),nullptr,GL_STREAM_READ);
    glReadPixels(p.left,p.readY,p.width,p.height,GL_RED_INTEGER,GL_UNSIGNED_INT,nullptr);
    if(pickFence_) glDeleteSync(pickFence_);
    pickFence_=glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE,0); glFlush();
    glBindBuffer(GL_PIXEL_PACK_BUFFER,0); glBindFramebuffer(GL_FRAMEBUFFER,defaultFramebufferObject()); doneCurrent();

    pendingPick_=std::move(p); pickBusy_=true; statusText_=m->meshMode?"GPU 表面三角面选择中...":"GPU 表面点选择中..."; update();
    QTimer::singleShot(0,this,[this]{pollPickReadback();});
}

void PointCloudWidget::pollPickReadback() {
    if(!pickBusy_||!pickFence_||!pickPbo_)return;
    makeCurrent();
    const GLenum wait=glClientWaitSync(pickFence_,0,0);
    if(wait==GL_TIMEOUT_EXPIRED){doneCurrent();QTimer::singleShot(1,this,[this]{pollPickReadback();});return;}
    if(wait==GL_WAIT_FAILED){glDeleteSync(pickFence_);pickFence_=nullptr;doneCurrent();pickBusy_=false;statusText_="GPU Picking fence 失败";update();return;}

    const PendingPick p=pendingPick_;
    const std::size_t count=static_cast<std::size_t>(p.width)*p.height;
    std::vector<std::uint32_t> pixels(count,JMEngine::kInvalidPointId);
    glBindBuffer(GL_PIXEL_PACK_BUFFER,pickPbo_);
    const void* mapped=glMapBufferRange(GL_PIXEL_PACK_BUFFER,0,static_cast<GLsizeiptr>(count*sizeof(std::uint32_t)),GL_MAP_READ_BIT);
    if(mapped){std::memcpy(pixels.data(),mapped,count*sizeof(std::uint32_t));glUnmapBuffer(GL_PIXEL_PACK_BUFFER);} glBindBuffer(GL_PIXEL_PACK_BUFFER,0);
    glDeleteSync(pickFence_);pickFence_=nullptr;doneCurrent();

    SceneModel* model=activeModel();
    if(!model){pickBusy_=false;return;}
    QPointer<PointCloudWidget> self(this);
    workerPool_.start([self,model,p,pixels=std::move(pixels)]() mutable {
        if(!self)return;
        auto ids=self->filterSurfacePixels(p,pixels);
        if(p.meshIds) ids=self->expandTriangleIdsToVertices(*model,ids);
        QMetaObject::invokeMethod(self,[self,p,ids=std::move(ids)]() mutable {
            if(!self)return; self->pickBusy_=false; self->statusText_=ids.empty()?"未选中几何":"表面选择完成";
            self->applySelection(std::move(ids),p.modifiers); self->update();
        },Qt::QueuedConnection);
    });
}

std::vector<JMEngine::PointId> PointCloudWidget::filterSurfacePixels(
    const PendingPick& p,const std::vector<std::uint32_t>& pixels) const
{
    JMEngine::PixelIdPicker picker(p.framebufferWidth,p.framebufferHeight,
        [&p,&pixels](int x,int y,int w,int h,std::uint32_t* dst){
            if(x!=p.left||y!=p.readY||w!=p.width||h!=p.height||!dst)return false;
            std::memcpy(dst,pixels.data(),pixels.size()*sizeof(std::uint32_t));return true;
        },true);
    if(p.mode==InteractionMode::Rectangle)return picker.pickRectangle(p.press.x(),p.press.y(),p.current.x(),p.current.y());
    if(p.mode==InteractionMode::Circle){const int dx=p.current.x()-p.press.x(),dy=p.current.y()-p.press.y();const int r=static_cast<int>(std::lround(std::sqrt(static_cast<double>(dx*dx+dy*dy))));return picker.pickCircle(p.press.x(),p.press.y(),r);}
    std::vector<JMEngine::Point2i> path;path.reserve(p.stroke.size());for(const auto&q:p.stroke)path.push_back({q.x(),q.y()});
    if(p.mode==InteractionMode::Lasso)return picker.pickLasso(path);
    if(p.mode==InteractionMode::BrushSelect)return picker.pickBrushStroke(path,p.brushRadiusPhysical);
    return{};
}

std::vector<JMEngine::PointId> PointCloudWidget::expandTriangleIdsToVertices(
    const SceneModel& model,const std::vector<std::uint32_t>& triangleIds) const
{
    std::vector<JMEngine::PointId> out; out.reserve(triangleIds.size()*3);
    const std::size_t triCount=model.mesh.triangleIndices.size()/3;
    for(const auto tid:triangleIds){
        if(static_cast<std::size_t>(tid)>=triCount)continue;
        const std::size_t b=static_cast<std::size_t>(tid)*3;
        out.push_back(model.mesh.triangleIndices[b]);out.push_back(model.mesh.triangleIndices[b+1]);out.push_back(model.mesh.triangleIndices[b+2]);
    }
    std::sort(out.begin(),out.end());out.erase(std::unique(out.begin(),out.end()),out.end());return out;
}

std::vector<JMEngine::PointId> PointCloudWidget::expandVertexSelectionToWholeTriangles(
    const SceneModel& model,const std::vector<JMEngine::PointId>& vertexIds) const
{
    if(!model.meshMode||vertexIds.empty())return vertexIds;
    std::vector<std::uint8_t> mark(model.cloud->size(),0u);
    for(auto id:vertexIds)if(id<mark.size())mark[id]=1u;
    const auto& idx=model.mesh.triangleIndices;
    for(std::size_t i=0;i+2<idx.size();i+=3){
        const auto a=idx[i],b=idx[i+1],c=idx[i+2];
        if((a<mark.size()&&mark[a])||(b<mark.size()&&mark[b])||(c<mark.size()&&mark[c])){
            if(a<mark.size())mark[a]=1;if(b<mark.size())mark[b]=1;if(c<mark.size())mark[c]=1;
        }
    }
    std::vector<JMEngine::PointId> out;out.reserve(vertexIds.size()*2);
    for(std::size_t i=0;i<mark.size();++i)if(mark[i])out.push_back(static_cast<JMEngine::PointId>(i));
    return out;
}

std::vector<JMEngine::PointId> PointCloudWidget::runThroughSelection(
    const PendingPick& p,const JMEngine::Mat4f& mvp,const JMEngine::PointCloud& cloud,const JMEngine::ObjMeshData* mesh) const
{
    const JMEngine::Viewport vp{p.framebufferWidth,p.framebufferHeight};
    std::vector<JMEngine::PointId> ids;
    if(p.mode==InteractionMode::Rectangle){ids=JMEngine::CpuSelector::rectangle(cloud,mvp,vp,{p.press.x(),p.press.y(),p.current.x(),p.current.y()});}
    else if(p.mode==InteractionMode::Circle){const int dx=p.current.x()-p.press.x(),dy=p.current.y()-p.press.y();const int r=static_cast<int>(std::lround(std::sqrt(static_cast<double>(dx*dx+dy*dy))));ids=JMEngine::CpuSelector::circle(cloud,mvp,vp,{p.press.x(),p.press.y()},r);}
    else {std::vector<JMEngine::Point2i> path;path.reserve(p.stroke.size());for(const auto&q:p.stroke)path.push_back({q.x(),q.y()});if(p.mode==InteractionMode::Lasso)ids=JMEngine::CpuSelector::lasso(cloud,mvp,vp,path);else if(p.mode==InteractionMode::BrushSelect)ids=JMEngine::CpuSelector::brushStroke(cloud,mvp,vp,path,p.brushRadiusPhysical);}
    (void)mesh;
    return ids;
}

void PointCloudWidget::startThroughPickAsync(Qt::KeyboardModifiers modifiers) {
    auto* m=activeModel(); if(!m||!m->cloud||pickBusy_||editBusy_)return;
    PendingPick p=buildPendingPick(modifiers,false); if(p.width<=0||p.height<=0)return;
    const auto mvp=camera_.mvp(std::max(1,width()),std::max(1,height()));
    SceneModel* model=m; pickBusy_=true; statusText_="OpenMP 穿透选择中...";update();
    QPointer<PointCloudWidget> self(this);
    workerPool_.start([self,model,p,mvp]() mutable {
        if(!self||!model->cloud)return;
        // 穿透选择直接使用精确投影命中的顶点，不再做“命中一个顶点就扩展整三角面”。
        // 旧逻辑会把边界附近的共享顶点不断扩展到相邻三角形，看起来像选择范围向外扩散。
        auto ids=self->runThroughSelection(p,mvp,*model->cloud,model->meshMode?&model->mesh:nullptr);
        QMetaObject::invokeMethod(self,[self,p,ids=std::move(ids)]() mutable {
            if(!self)return;self->pickBusy_=false;self->statusText_=ids.empty()?"未选中几何":"穿透选择完成";self->applySelection(std::move(ids),p.modifiers);self->update();
        },Qt::QueuedConnection);
    });
}

void PointCloudWidget::setSelectionVisual(SceneModel& m,const std::vector<JMEngine::PointId>& ids) {
    std::vector<JMEngine::PointId> changed;changed.reserve(m.selectedIds.size()+ids.size());
    for(auto id:m.selectedIds)if(id<m.selectedMask.size()){m.selectedMask[id]=0;changed.push_back(id);}
    for(auto id:ids)if(id<m.selectedMask.size()){m.selectedMask[id]=1;changed.push_back(id);}
    m.selectedIds=ids;queueDirtyIds(m,changed,DirtyBuffer::Selection);update();
}
void PointCloudWidget::clearSelectionVisual(SceneModel& m){setSelectionVisual(m,{});}

void PointCloudWidget::applySelection(std::vector<JMEngine::PointId> ids,Qt::KeyboardModifiers modifiers) {
    auto* m=activeModel();if(!m||editBusy_)return;
    std::sort(ids.begin(),ids.end());ids.erase(std::unique(ids.begin(),ids.end()),ids.end());
    std::vector<JMEngine::PointId> next;
    if(modifiers.testFlag(Qt::ShiftModifier))next=unionSorted(m->selectedIds,ids);
    else if(modifiers.testFlag(Qt::AltModifier))next=differenceSorted(m->selectedIds,ids);
    else next=std::move(ids);
    m->editor.select(next);setSelectionVisual(*m,next);
}

void PointCloudWidget::deleteIdsAsync(std::vector<JMEngine::PointId> ids) {
    auto* m=activeModel();if(!m||editBusy_||ids.empty())return;
    editBusy_=true;statusText_="后台删除中";clearSelectionVisual(*m);m->editor.select(std::move(ids));
    SceneModel* model=m;QPointer<PointCloudWidget> self(this);
    workerPool_.start([self,model]{
        const bool changed=model->editor.deleteSelection();const auto changedIds=model->editor.lastChangedIds();const auto kind=model->editor.lastChangeKind();const auto count=model->cloud?model->cloud->activeCount():0;
        if(!self)return;QMetaObject::invokeMethod(self,[self,model,changed,changedIds,kind,count]() mutable {
            if(!self)return;self->editBusy_=false;model->cachedActiveCount=count;if(changed){if(kind==JMEngine::ChangeKind::Flags)self->queueDirtyIds(*model,changedIds,DirtyBuffer::Flags);else if(kind==JMEngine::ChangeKind::Position)self->queueDirtyIds(*model,changedIds,DirtyBuffer::Position);}self->statusText_=changed?"删除完成":"没有发生变化";self->update();
        },Qt::QueuedConnection);
    });
}

void PointCloudWidget::undoAsync(){auto*m=activeModel();if(!m||editBusy_||!m->editor.canUndo())return;editBusy_=true;clearSelectionVisual(*m);SceneModel*model=m;QPointer<PointCloudWidget>self(this);workerPool_.start([self,model]{const bool changed=model->editor.undo();const auto ids=model->editor.lastChangedIds();const auto kind=model->editor.lastChangeKind();const auto count=model->cloud->activeCount();if(!self)return;QMetaObject::invokeMethod(self,[self,model,changed,ids,kind,count]()mutable{if(!self)return;self->editBusy_=false;model->cachedActiveCount=count;if(changed){if(kind==JMEngine::ChangeKind::Flags)self->queueDirtyIds(*model,ids,DirtyBuffer::Flags);else if(kind==JMEngine::ChangeKind::Position)self->queueDirtyIds(*model,ids,DirtyBuffer::Position);}self->statusText_="撤销完成";self->update();},Qt::QueuedConnection);});}
void PointCloudWidget::redoAsync(){auto*m=activeModel();if(!m||editBusy_||!m->editor.canRedo())return;editBusy_=true;clearSelectionVisual(*m);SceneModel*model=m;QPointer<PointCloudWidget>self(this);workerPool_.start([self,model]{const bool changed=model->editor.redo();const auto ids=model->editor.lastChangedIds();const auto kind=model->editor.lastChangeKind();const auto count=model->cloud->activeCount();if(!self)return;QMetaObject::invokeMethod(self,[self,model,changed,ids,kind,count]()mutable{if(!self)return;self->editBusy_=false;model->cachedActiveCount=count;if(changed){if(kind==JMEngine::ChangeKind::Flags)self->queueDirtyIds(*model,ids,DirtyBuffer::Flags);else if(kind==JMEngine::ChangeKind::Position)self->queueDirtyIds(*model,ids,DirtyBuffer::Position);}self->statusText_="重做完成";self->update();},Qt::QueuedConnection);});}

void PointCloudWidget::saveAsync(){auto*m=activeModel();if(!m||editBusy_||!m->cloud)return;editBusy_=true;SceneModel*model=m;QPointer<PointCloudWidget>self(this);workerPool_.start([self,model]{std::string error;const bool ok=JMEngine::PointCloudIO::savePly(*model->cloud,"edited_output.ply",&error);if(!self)return;QMetaObject::invokeMethod(self,[self,ok,error]{if(!self)return;self->editBusy_=false;self->statusText_=ok?"已保存 edited_output.ply":"保存失败: "+error;self->update();},Qt::QueuedConnection);});}

void PointCloudWidget::setMode(InteractionMode mode){if(editBusy_)return;mode_=mode;dragging_=false;editGestureActive_=false;viewDragButton_=Qt::NoButton;stroke_.clear();update();}
const char* PointCloudWidget::modeName() const noexcept{switch(mode_){case InteractionMode::View:return"浏览";case InteractionMode::Rectangle:return"矩形选择(R)";case InteractionMode::Lasso:return"套索(L)";case InteractionMode::Circle:return"圆形选择(C)";case InteractionMode::BrushSelect:return"画刷(B)";}return"未知";}
const char* PointCloudWidget::depthModeName() const noexcept{return depthMode_==SelectionDepthMode::Surface?"表面":"穿透";}

void PointCloudWidget::mousePressEvent(QMouseEvent* event) {
    const QPoint pos = event->position().toPoint();

    // 1.8.0 交互规则：只有“Ctrl + 左键”才真正进入编辑/选择。
    // 即使菜单已经选择了矩形/Lasso/圆/Brush，只要没有按 Ctrl，左键仍然是 Orbit。
    const bool editGesture = event->button() == Qt::LeftButton &&
                             event->modifiers().testFlag(Qt::ControlModifier) &&
                             mode_ != InteractionMode::View;

    if (editGesture && !editBusy_ && !pickBusy_) {
        pressPos_ = currentPos_ = pos;
        stroke_.clear();
        stroke_.push_back(pos);
        dragging_ = true;
        editGestureActive_ = true;
        viewDragButton_ = Qt::NoButton;
        update();
        return;
    }

    // 非 Ctrl 编辑手势一律按浏览操作处理。
    if (event->button() == Qt::LeftButton ||
        event->button() == Qt::RightButton ||
        event->button() == Qt::MiddleButton) {
        viewDragButton_ = event->button();
        editGestureActive_ = false;
        lastViewPos_ = pos;
        dragging_ = true;
    }
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent* event) {
    const QPoint pos = event->position().toPoint();
    if (!dragging_) return;

    // viewDragButton_ 有值表示当前手势是浏览，而不是 Ctrl 编辑。
    if (viewDragButton_ != Qt::NoButton) {
        const QPoint d = pos - lastViewPos_;
        lastViewPos_ = pos;
        if (viewDragButton_ == Qt::LeftButton)
            camera_.orbit(static_cast<float>(d.x()), static_cast<float>(d.y()), width(), height());
        else
            camera_.pan(static_cast<float>(d.x()), static_cast<float>(d.y()), width(), height());
        update();
        return;
    }

    // Ctrl 选择手势已经在 mousePress 时锁定；中途松开 Ctrl 也不会把这次拖动改成 Orbit。
    currentPos_ = pos;
    if (mode_ == InteractionMode::Lasso || mode_ == InteractionMode::BrushSelect) {
        if (stroke_.empty() || (stroke_.back() - pos).manhattanLength() >= 3)
            stroke_.push_back(pos);
    }
    update();
}

void PointCloudWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (!dragging_) return;

    if (viewDragButton_ != Qt::NoButton) {
        if (event->button() == viewDragButton_) {
            dragging_ = false;
            editGestureActive_ = false;
            viewDragButton_ = Qt::NoButton;
            update();
        }
        return;
    }

    if (event->button() != Qt::LeftButton) return;
    currentPos_ = event->position().toPoint();
    if ((mode_ == InteractionMode::Lasso || mode_ == InteractionMode::BrushSelect) &&
        (stroke_.empty() || stroke_.back() != currentPos_)) {
        stroke_.push_back(currentPos_);
    }
    dragging_ = false;
    editGestureActive_ = false;

    // Ctrl 在这里仅表示“这是一次编辑手势”，不再作为减选修饰键传入 Selection。
    // Shift = 追加；Alt = 减选；无修饰键 = 替换。
    Qt::KeyboardModifiers selectionMods = event->modifiers();
    selectionMods &= ~Qt::ControlModifier;
    startSelection(selectionMods);
    update();
}
void PointCloudWidget::wheelEvent(QWheelEvent* event){if(mode_==InteractionMode::BrushSelect&&event->modifiers().testFlag(Qt::AltModifier)){const int step=event->angleDelta().y()>0?2:-2;brushRadiusPixels_=std::clamp(brushRadiusPixels_+step,2,200);}else camera_.zoom(static_cast<float>(event->angleDelta().y())/120.0f);update();}
void PointCloudWidget::keyPressEvent(QKeyEvent* event){switch(event->key()){case Qt::Key_V:case Qt::Key_Escape:setMode(InteractionMode::View);clearCurrentSelection();break;case Qt::Key_R:setMode(InteractionMode::Rectangle);break;case Qt::Key_L:setMode(InteractionMode::Lasso);break;case Qt::Key_C:setMode(InteractionMode::Circle);break;case Qt::Key_B:setMode(InteractionMode::BrushSelect);break;case Qt::Key_Delete:case Qt::Key_Backspace:deleteSelection();break;case Qt::Key_Z:undoAsync();break;case Qt::Key_Y:redoAsync();break;case Qt::Key_F:fitView();break;case Qt::Key_S:saveAsync();break;default:QOpenGLWidget::keyPressEvent(event);break;}}

