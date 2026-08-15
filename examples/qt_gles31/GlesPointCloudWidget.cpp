#include "GlesPointCloudWidget.h"

#include <pceditor/CpuSelector.h>
#include <pceditor/PointCloudIO.h>

#include <QDebug>
#include <QEvent>
#include <QFileInfo>
#include <QImage>
#include <QLineF>
#include <QMatrix4x4>
#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <QVector3D>
#include <QKeyEvent>
#include <QMetaObject>
#include <QMouseEvent>
#include <QPainter>
#include <QPointer>
#include <QTimer>
#include <QTouchEvent>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <iterator>
#include <limits>
#include <numeric>

namespace {

// GLES 3.1 版本不使用 geometry shader。
// Point 和 Mesh 共用一个 VS/FS；Mesh 删除通过后台重建可见 EBO 实现。
constexpr const char* kRenderVs = R"GLSL(#version 310 es
precision highp float;
precision highp int;
layout(location=0) in vec3 aPosition;
layout(location=1) in uint aRgba;
layout(location=2) in vec3 aNormal;
layout(location=4) in uint aFlags;
layout(location=5) in uint aSelected;

uniform mat4 uMVP;
uniform float uPointSize;
uniform uint uPointMode;

out vec4 vColor;
out vec3 vNormal;
flat out uint vSelected;

const uint POINT_DELETED = 4u;

void main() {
    if ((aFlags & POINT_DELETED) != 0u) {
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

constexpr const char* kRenderFs = R"GLSL(#version 310 es
precision highp float;
precision highp int;
in vec4 vColor;
in vec3 vNormal;
flat in uint vSelected;
uniform vec3 uLightDir;
uniform uint uPointMode;
out vec4 outColor;

void main() {
    if (uPointMode != 0u) {
        vec2 q = gl_PointCoord * 2.0 - vec2(1.0);
        if (dot(q,q) > 1.0) discard;
    }
    vec3 base = vColor.rgb;
    float nl = length(vNormal);
    if (nl > 0.001) {
        vec3 n = normalize(vNormal);
        float d = max(dot(n, normalize(uLightDir)), 0.0);
        base *= (0.58 + 0.42 * d);
    }
    if (vSelected != 0u) base = mix(base, vec3(1.0, 0.48, 0.03), 0.70);
    outColor = vec4(base, 1.0);
}
)GLSL";

constexpr const char* kPointPickVs = R"GLSL(#version 310 es
precision highp float;
precision highp int;
layout(location=0) in vec3 aPosition;
layout(location=3) in uint aPointId;
layout(location=4) in uint aFlags;
uniform mat4 uMVP;
uniform float uPointSize;
flat out uint vId;
const uint POINT_DELETED = 4u;
void main() {
    if ((aFlags & POINT_DELETED) != 0u) {
        gl_Position = vec4(2.0,2.0,2.0,1.0);
        gl_PointSize = 1.0;
    } else {
        gl_Position = uMVP * vec4(aPosition,1.0);
        gl_PointSize = uPointSize;
    }
    vId = aPointId;
}
)GLSL";

constexpr const char* kPointPickFs = R"GLSL(#version 310 es
precision highp float;
precision highp int;
flat in uint vId;
layout(location=0) out highp uint outId;
void main() {
    vec2 q = gl_PointCoord * 2.0 - vec2(1.0);
    if (dot(q,q) > 1.0) discard;
    outId = vId;
}
)GLSL";

// OBJ 表面 Picking：CPU 在加载阶段展开三角形，三个顶点都携带相同 TriangleId。
// 因此完全不依赖 geometry shader / gl_PrimitiveID。
constexpr const char* kMeshPickVs = R"GLSL(#version 310 es
precision highp float;
precision highp int;
layout(location=0) in vec3 aPosition;
layout(location=1) in uint aTriangleId;
uniform mat4 uMVP;
flat out uint vTriangleId;
void main() {
    gl_Position = uMVP * vec4(aPosition,1.0);
    vTriangleId = aTriangleId;
}
)GLSL";

constexpr const char* kMeshPickFs = R"GLSL(#version 310 es
precision highp float;
precision highp int;
flat in uint vTriangleId;
layout(location=0) out highp uint outId;
void main() { outId = vTriangleId; }
)GLSL";

bool bakeTexture(const pceditor::ObjAppearanceData& appearance,
                 pceditor::PointCloud& cloud)
{
    if (appearance.diffuseTexturePath.empty() || !appearance.hasTextureCoordinates()) return false;
    QImage image(QString::fromUtf8(appearance.diffuseTexturePath.c_str()));
    if (image.isNull()) return false;
    image = image.convertToFormat(QImage::Format_RGBA8888);
    const int width = image.width();
    const int height = image.height();
    const qsizetype stride = image.bytesPerLine();
    const uchar* bits = image.constBits();
    if (width <= 0 || height <= 0 || !bits) return false;

    const std::size_t count = std::min(cloud.size(), appearance.vertexUv.size());
#ifdef PCEDITOR_USE_OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (std::int64_t ii = 0; ii < static_cast<std::int64_t>(count); ++ii) {
        const std::size_t i = static_cast<std::size_t>(ii);
        if (i >= appearance.hasUv.size() || appearance.hasUv[i] == 0u) continue;
        const auto uv = appearance.vertexUv[i];
        const int x = std::clamp(static_cast<int>(std::lround(std::clamp(uv.x,0.0f,1.0f) * (width-1))), 0, width-1);
        const int y = std::clamp(static_cast<int>(std::lround((1.0f-std::clamp(uv.y,0.0f,1.0f)) * (height-1))), 0, height-1);
        const uchar* px = bits + static_cast<qsizetype>(y) * stride + x * 4;
        cloud.points()[i].rgba = static_cast<std::uint32_t>(px[0]) |
                                 (static_cast<std::uint32_t>(px[1]) << 8u) |
                                 (static_cast<std::uint32_t>(px[2]) << 16u) |
                                 (static_cast<std::uint32_t>(px[3]) << 24u);
    }
    return true;
}

std::vector<pceditor::PointId> sortedUnique(std::vector<pceditor::PointId> ids)
{
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    return ids;
}

bool pointInPolygon(const QPoint& p, const std::vector<QPoint>& polygon)
{
    if (polygon.size() < 3) return false;
    bool inside = false;
    for (std::size_t i=0, j=polygon.size()-1; i<polygon.size(); j=i++) {
        const QPoint& a = polygon[i];
        const QPoint& b = polygon[j];
        const bool crossing = ((a.y() > p.y()) != (b.y() > p.y())) &&
            (p.x() < (b.x()-a.x()) * (p.y()-a.y()) / double(b.y()-a.y() == 0 ? 1 : b.y()-a.y()) + a.x());
        if (crossing) inside = !inside;
    }
    return inside;
}

} // namespace

GlesPointCloudWidget::Model::Model(QString p,
                                   std::shared_ptr<pceditor::PointCloud> c,
                                   pceditor::ObjMeshData m,
                                   bool mesh)
    : path(std::move(p)), meshMode(mesh), cloud(std::move(c)), editor(cloud), mesh(std::move(m))
{
    if (cloud) selectionMask.assign(cloud->size(), 0u);
}

GlesPointCloudWidget::GlesPointCloudWidget(QWidget* parent)
    : QOpenGLWidget(parent)
{
    setAttribute(Qt::WA_AcceptTouchEvents, true);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    setMinimumSize(800, 480);
    workerPool_.setMaxThreadCount(2);
    statusText_ = QString::fromUtf8("空场景：打开 OBJ / PLY");
}

GlesPointCloudWidget::~GlesPointCloudWidget()
{
    workerPool_.waitForDone();
    makeCurrent();
    for (auto& m : models_) destroyModelGl(*m);
    destroyPickingFramebuffer();
    doneCurrent();
}

GlesPointCloudWidget::Model* GlesPointCloudWidget::activeModel()
{
    if (activeModelIndex_ < 0 || activeModelIndex_ >= static_cast<int>(models_.size())) return nullptr;
    return models_[static_cast<std::size_t>(activeModelIndex_)].get();
}
const GlesPointCloudWidget::Model* GlesPointCloudWidget::activeModel() const
{
    if (activeModelIndex_ < 0 || activeModelIndex_ >= static_cast<int>(models_.size())) return nullptr;
    return models_[static_cast<std::size_t>(activeModelIndex_)].get();
}

int GlesPointCloudWidget::findModel(const QString& path) const
{
    const QString abs = QFileInfo(path).absoluteFilePath();
    for (int i=0; i<static_cast<int>(models_.size()); ++i) {
        if (QFileInfo(models_[static_cast<std::size_t>(i)]->path).absoluteFilePath() == abs) return i;
    }
    return -1;
}

QString GlesPointCloudWidget::activeModelPath() const
{
    const auto* m = activeModel();
    return m ? m->path : QString{};
}

void GlesPointCloudWidget::loadModelAsync(const QString& path)
{
    const QString abs = QFileInfo(path).absoluteFilePath();
    if (abs.isEmpty()) return;
    const int existing = findModel(abs);
    if (existing >= 0) { activeModelIndex_ = existing; fitView(); update(); return; }
    if (std::find(loadingPaths_.begin(), loadingPaths_.end(), abs) != loadingPaths_.end()) return;
    loadingPaths_.push_back(abs);

    QPointer<GlesPointCloudWidget> self(this);
    workerPool_.start([self, abs] {
        if (!self) return;
        const std::string file = abs.toStdString();
        const std::string ext = std::filesystem::path(file).extension().string();
        std::shared_ptr<pceditor::PointCloud> cloud;
        pceditor::ObjMeshData mesh;
        bool meshMode = false;
        QString message;

        if (ext == ".obj" || ext == ".OBJ") {
            pceditor::ObjModelData obj;
            std::string msg;
            if (pceditor::ObjModelLoader::load(file, obj, &msg)) {
                cloud = obj.cloud;
                mesh = std::move(obj.mesh);
                meshMode = !mesh.empty();
                if (cloud) bakeTexture(obj.appearance, *cloud);
                message = QString::fromUtf8(msg.c_str());
            }
        } else {
            std::string msg;
            cloud = pceditor::PointCloudIO::load(file, &msg);
            message = QString::fromUtf8(msg.c_str());
        }

        // 大 OBJ 的可见 EBO 和 TriangleId Picking 顶点也在后台构造，避免 UI 线程线性展开几百万三角形。
        std::vector<std::uint32_t> initialVisible;
        std::vector<PickVertex> initialPick;
        if (cloud && meshMode) {
            initialVisible = mesh.triangleIndices;
            const std::size_t triCount = mesh.triangleCount();
            initialPick.resize(triCount * 3u);
#ifdef PCEDITOR_USE_OPENMP
#pragma omp parallel for schedule(static)
#endif
            for (std::int64_t tt = 0; tt < static_cast<std::int64_t>(triCount); ++tt) {
                const std::size_t t = static_cast<std::size_t>(tt);
                const std::uint32_t tid = static_cast<std::uint32_t>(t);
                for (int k = 0; k < 3; ++k) {
                    const auto vid = mesh.triangleIndices[t*3u + static_cast<std::size_t>(k)];
                    if (vid < cloud->size()) initialPick[t*3u + static_cast<std::size_t>(k)] = {cloud->points()[vid].position, tid};
                }
            }
        }

        QMetaObject::invokeMethod(self, [self, abs, cloud, mesh=std::move(mesh), meshMode, message,
                                         initialVisible=std::move(initialVisible), initialPick=std::move(initialPick)]() mutable {
            if (!self) return;
            auto& lp = self->loadingPaths_;
            lp.erase(std::remove(lp.begin(), lp.end(), abs), lp.end());
            if (!cloud) {
                self->statusText_ = QString::fromUtf8("加载失败：") + message;
                self->update();
                return;
            }

            auto model = std::make_unique<Model>(abs, cloud, std::move(mesh), meshMode);
            model->visibleMeshIndices = std::move(initialVisible);
            model->expandedPickVertices = std::move(initialPick);
            self->models_.push_back(std::move(model));
            self->activeModelIndex_ = static_cast<int>(self->models_.size()) - 1;
            self->statusText_ = QString::fromUtf8("已加载：") + QFileInfo(abs).fileName();
            self->fitView();
            self->update();
        }, Qt::QueuedConnection);
    });
}

bool GlesPointCloudWidget::activateModel(const QString& path)
{
    const int i = findModel(path);
    if (i < 0) return false;
    activeModelIndex_ = i;
    clearSelection();
    fitView();
    update();
    return true;
}

void GlesPointCloudWidget::setModelVisible(const QString& path, bool visible)
{
    const int i = findModel(path);
    if (i < 0) return;
    models_[static_cast<std::size_t>(i)]->visible = visible;
    update();
}

void GlesPointCloudWidget::removeModel(const QString& path)
{
    const int i = findModel(path);
    if (i < 0) return;
    makeCurrent();
    destroyModelGl(*models_[static_cast<std::size_t>(i)]);
    doneCurrent();
    models_.erase(models_.begin() + i);
    if (models_.empty()) activeModelIndex_ = -1;
    else activeModelIndex_ = std::clamp(i, 0, static_cast<int>(models_.size()) - 1);
    fitView();
    update();
}

void GlesPointCloudWidget::setInteractionMode(InteractionMode mode) { interactionMode_ = mode; cancelEditGesture(); update(); }
void GlesPointCloudWidget::setSelectionDepthMode(SelectionDepthMode mode) { selectionDepthMode_ = mode; }
void GlesPointCloudWidget::setTouchEditMode(bool enabled) { touchEditMode_ = enabled; cancelEditGesture(); update(); }

void GlesPointCloudWidget::initializeGL()
{
    initializeOpenGLFunctions();
    const auto* ctx = QOpenGLContext::currentContext();
    const auto fmt = ctx ? ctx->format() : QSurfaceFormat{};
    glesReady_ = ctx && ctx->isOpenGLES() &&
                 (fmt.majorVersion() > 3 || (fmt.majorVersion() == 3 && fmt.minorVersion() >= 1));

    const char* vendor = reinterpret_cast<const char*>(glGetString(GL_VENDOR));
    const char* renderer = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    const char* version = reinterpret_cast<const char*>(glGetString(GL_VERSION));
    qInfo() << "[PointCloudEditor/RK3588] GLES=" << (ctx && ctx->isOpenGLES())
            << "format=" << fmt.majorVersion() << fmt.minorVersion()
            << "vendor=" << (vendor ? vendor : "?")
            << "renderer=" << (renderer ? renderer : "?")
            << "version=" << (version ? version : "?");

    if (!glesReady_) {
        statusText_ = QString::fromUtf8("需要 OpenGL ES 3.1+，当前 Context=%1.%2")
            .arg(fmt.majorVersion()).arg(fmt.minorVersion());
        qWarning() << statusText_;
        return;
    }

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glClearColor(0.02f, 0.02f, 0.025f, 1.0f);

    if (!createPrograms()) {
        glesReady_ = false;
        statusText_ = QString::fromUtf8("GLES 3.1 Shader 编译失败，请查看控制台日志");
        qWarning() << pointProgram_.log() << meshProgram_.log() << pointPickProgram_.log() << meshPickProgram_.log();
        return;
    }
    pickingReady_ = createPickingFramebuffer(std::max(1, int(width()*devicePixelRatioF())),
                                              std::max(1, int(height()*devicePixelRatioF())));
    statusText_ = QString::fromUtf8("GLES %1.%2 | %3 | GPU Picking=%4")
        .arg(fmt.majorVersion()).arg(fmt.minorVersion())
        .arg(QString::fromLatin1(renderer ? renderer : "unknown"))
        .arg(pickingReady_ ? QString::fromUtf8("R32UI") : QString::fromUtf8("不可用/将使用CPU"));
}

void GlesPointCloudWidget::resizeGL(int w, int h)
{
    if (!glesReady_) return;
    const qreal dpr = devicePixelRatioF();
    pickingReady_ = createPickingFramebuffer(std::max(1,int(w*dpr)), std::max(1,int(h*dpr)));
}

bool GlesPointCloudWidget::createPrograms()
{
    auto build = [](QOpenGLShaderProgram& p, const char* vs, const char* fs) {
        p.removeAllShaders();
        if (!p.addShaderFromSourceCode(QOpenGLShader::Vertex, vs)) return false;
        if (!p.addShaderFromSourceCode(QOpenGLShader::Fragment, fs)) return false;
        return p.link();
    };
    const bool ok1 = build(pointProgram_, kRenderVs, kRenderFs);
    const bool ok2 = build(meshProgram_, kRenderVs, kRenderFs);
    const bool ok3 = build(pointPickProgram_, kPointPickVs, kPointPickFs);
    const bool ok4 = build(meshPickProgram_, kMeshPickVs, kMeshPickFs);
    return ok1 && ok2 && ok3 && ok4;
}

bool GlesPointCloudWidget::createPickingFramebuffer(int w, int h)
{
    if (pickFbo_ && pickWidth_ == w && pickHeight_ == h) return true;
    destroyPickingFramebuffer();
    pickWidth_ = w; pickHeight_ = h;

    glGenFramebuffers(1, &pickFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, pickFbo_);

    glGenTextures(1, &pickTexture_);
    glBindTexture(GL_TEXTURE_2D, pickTexture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32UI, w, h, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, nullptr);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, pickTexture_, 0);

    glGenRenderbuffers(1, &pickDepth_);
    glBindRenderbuffer(GL_RENDERBUFFER, pickDepth_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, pickDepth_);

    const bool ok = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());
    return ok;
}

void GlesPointCloudWidget::destroyPickingFramebuffer()
{
    if (pickDepth_) glDeleteRenderbuffers(1, &pickDepth_);
    if (pickTexture_) glDeleteTextures(1, &pickTexture_);
    if (pickFbo_) glDeleteFramebuffers(1, &pickFbo_);
    pickDepth_=pickTexture_=pickFbo_=0;
    pickWidth_=pickHeight_=0;
}

void GlesPointCloudWidget::createModelGl(Model& m)
{
    if (m.glCreated || !m.cloud) return;
    glGenVertexArrays(1, &m.vao);
    glBindVertexArray(m.vao);

    const GLsizeiptr n = static_cast<GLsizeiptr>(m.cloud->size());
    glGenBuffers(1,&m.positionVbo); glBindBuffer(GL_ARRAY_BUFFER,m.positionVbo); glBufferData(GL_ARRAY_BUFFER,n*sizeof(pceditor::Vec3f),nullptr,GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0); glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(pceditor::Vec3f),nullptr);
    glGenBuffers(1,&m.colorVbo); glBindBuffer(GL_ARRAY_BUFFER,m.colorVbo); glBufferData(GL_ARRAY_BUFFER,n*sizeof(std::uint32_t),nullptr,GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(1); glVertexAttribIPointer(1,1,GL_UNSIGNED_INT,sizeof(std::uint32_t),nullptr);
    glGenBuffers(1,&m.normalVbo); glBindBuffer(GL_ARRAY_BUFFER,m.normalVbo); glBufferData(GL_ARRAY_BUFFER,n*sizeof(pceditor::Vec3f),nullptr,GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(2); glVertexAttribPointer(2,3,GL_FLOAT,GL_FALSE,sizeof(pceditor::Vec3f),nullptr);

    // PointId 直接等于原始数组下标，编辑期间软删除保证 ID 稳定。
    glGenBuffers(1,&m.pointIdVbo); glBindBuffer(GL_ARRAY_BUFFER,m.pointIdVbo);
    std::vector<std::uint32_t> ids(m.cloud->size()); std::iota(ids.begin(),ids.end(),0u);
    glBufferData(GL_ARRAY_BUFFER,n*sizeof(std::uint32_t),ids.data(),GL_STATIC_DRAW);
    glEnableVertexAttribArray(3); glVertexAttribIPointer(3,1,GL_UNSIGNED_INT,sizeof(std::uint32_t),nullptr);

    glGenBuffers(1,&m.flagsVbo); glBindBuffer(GL_ARRAY_BUFFER,m.flagsVbo); glBufferData(GL_ARRAY_BUFFER,n*sizeof(std::uint32_t),nullptr,GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(4); glVertexAttribIPointer(4,1,GL_UNSIGNED_INT,sizeof(std::uint32_t),nullptr);
    glGenBuffers(1,&m.selectionVbo); glBindBuffer(GL_ARRAY_BUFFER,m.selectionVbo); glBufferData(GL_ARRAY_BUFFER,n*sizeof(std::uint32_t),nullptr,GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(5); glVertexAttribIPointer(5,1,GL_UNSIGNED_INT,sizeof(std::uint32_t),nullptr);

    if (m.meshMode) {
        glGenBuffers(1,&m.meshEbo); glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m.meshEbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,static_cast<GLsizeiptr>(m.visibleMeshIndices.size()*sizeof(std::uint32_t)),nullptr,GL_DYNAMIC_DRAW);

        glGenVertexArrays(1,&m.pickVao); glBindVertexArray(m.pickVao);
        glGenBuffers(1,&m.pickPositionVbo); glBindBuffer(GL_ARRAY_BUFFER,m.pickPositionVbo);
        glBufferData(GL_ARRAY_BUFFER,static_cast<GLsizeiptr>(m.expandedPickVertices.size()*sizeof(pceditor::Vec3f)),nullptr,GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0); glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(pceditor::Vec3f),nullptr);
        glGenBuffers(1,&m.pickIdVbo); glBindBuffer(GL_ARRAY_BUFFER,m.pickIdVbo);
        glBufferData(GL_ARRAY_BUFFER,static_cast<GLsizeiptr>(m.expandedPickVertices.size()*sizeof(std::uint32_t)),nullptr,GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(1); glVertexAttribIPointer(1,1,GL_UNSIGNED_INT,sizeof(std::uint32_t),nullptr);
    }
    glBindVertexArray(0);
    m.glCreated = true;
}

void GlesPointCloudWidget::destroyModelGl(Model& m)
{
    if (!m.glCreated) return;
    GLuint bufs[] = {m.positionVbo,m.colorVbo,m.normalVbo,m.pointIdVbo,m.flagsVbo,m.selectionVbo,m.meshEbo,m.pickPositionVbo,m.pickIdVbo};
    glDeleteBuffers(9, bufs);
    if (m.vao) glDeleteVertexArrays(1,&m.vao);
    if (m.pickVao) glDeleteVertexArrays(1,&m.pickVao);
    m.vao = m.positionVbo = m.colorVbo = m.normalVbo = m.pointIdVbo = 0;
    m.flagsVbo = m.selectionVbo = m.meshEbo = 0;
    m.pickVao = m.pickPositionVbo = m.pickIdVbo = 0;
    m.drawPointCount = m.drawIndexCount = m.pickVertexCount = 0;
    m.glCreated = false;
}

void GlesPointCloudWidget::uploadModelIncremental(Model& m, std::size_t& budget)
{
    if (!m.cloud) return;
    if (!m.glCreated) createModelGl(m);
    constexpr std::size_t chunkPoints = 150000;
    const auto& pts = m.cloud->points();
    if (m.uploadPointCursor < pts.size() && budget > 0) {
        const std::size_t count = std::min(chunkPoints, pts.size()-m.uploadPointCursor);
        std::vector<pceditor::Vec3f> pos(count), normal(count);
        std::vector<std::uint32_t> color(count), flags(count), sel(count);
        for (std::size_t i=0;i<count;++i) {
            const auto& p=pts[m.uploadPointCursor+i]; pos[i]=p.position; normal[i]=p.normal; color[i]=p.rgba; flags[i]=p.flags; sel[i]=m.selectionMask[m.uploadPointCursor+i];
        }
        const GLintptr off = static_cast<GLintptr>(m.uploadPointCursor);
        glBindBuffer(GL_ARRAY_BUFFER,m.positionVbo); glBufferSubData(GL_ARRAY_BUFFER,off*sizeof(pceditor::Vec3f),count*sizeof(pceditor::Vec3f),pos.data());
        glBindBuffer(GL_ARRAY_BUFFER,m.normalVbo); glBufferSubData(GL_ARRAY_BUFFER,off*sizeof(pceditor::Vec3f),count*sizeof(pceditor::Vec3f),normal.data());
        glBindBuffer(GL_ARRAY_BUFFER,m.colorVbo); glBufferSubData(GL_ARRAY_BUFFER,off*sizeof(std::uint32_t),count*sizeof(std::uint32_t),color.data());
        glBindBuffer(GL_ARRAY_BUFFER,m.flagsVbo); glBufferSubData(GL_ARRAY_BUFFER,off*sizeof(std::uint32_t),count*sizeof(std::uint32_t),flags.data());
        glBindBuffer(GL_ARRAY_BUFFER,m.selectionVbo); glBufferSubData(GL_ARRAY_BUFFER,off*sizeof(std::uint32_t),count*sizeof(std::uint32_t),sel.data());
        m.uploadPointCursor += count;
        m.drawPointCount = static_cast<GLsizei>(m.uploadPointCursor);
        budget = budget > count*40u ? budget-count*40u : 0;
    }

    if (m.meshMode && m.uploadIndexCursor < m.visibleMeshIndices.size() && budget>0) {
        const std::size_t count=std::min<std::size_t>(450000,m.visibleMeshIndices.size()-m.uploadIndexCursor);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m.meshEbo);
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,static_cast<GLintptr>(m.uploadIndexCursor*sizeof(std::uint32_t)),count*sizeof(std::uint32_t),m.visibleMeshIndices.data()+m.uploadIndexCursor);
        m.uploadIndexCursor+=count; m.drawIndexCount=static_cast<GLsizei>(m.uploadIndexCursor);
        budget = budget > count*4u ? budget-count*4u : 0;
    }

    if (m.meshMode && m.uploadPickCursor < m.expandedPickVertices.size() && budget>0) {
        const std::size_t count=std::min<std::size_t>(300000,m.expandedPickVertices.size()-m.uploadPickCursor);
        std::vector<pceditor::Vec3f> pos(count); std::vector<std::uint32_t> ids(count);
        for(std::size_t i=0;i<count;++i){ const auto& v=m.expandedPickVertices[m.uploadPickCursor+i]; pos[i]=v.position; ids[i]=v.id; }
        glBindBuffer(GL_ARRAY_BUFFER,m.pickPositionVbo); glBufferSubData(GL_ARRAY_BUFFER,static_cast<GLintptr>(m.uploadPickCursor*sizeof(pceditor::Vec3f)),count*sizeof(pceditor::Vec3f),pos.data());
        glBindBuffer(GL_ARRAY_BUFFER,m.pickIdVbo); glBufferSubData(GL_ARRAY_BUFFER,static_cast<GLintptr>(m.uploadPickCursor*sizeof(std::uint32_t)),count*sizeof(std::uint32_t),ids.data());
        m.uploadPickCursor+=count; m.pickVertexCount=static_cast<GLsizei>(m.uploadPickCursor);
        budget = budget > count*16u ? budget-count*16u : 0;
    }
}

void GlesPointCloudWidget::paintGL()
{
    if (!glesReady_) {
        QPainter painter(this);
        painter.fillRect(rect(), QColor(12,12,14));
        painter.setPen(Qt::red);
        painter.drawText(20, 40, statusText_);
        return;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());
    glViewport(0,0,std::max(1,int(width()*devicePixelRatioF())),std::max(1,int(height()*devicePixelRatioF())));
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    // 每帧最多上传约 12MB，避免大模型首次进入场景时冻结触摸/UI。
    std::size_t budget=12u*1024u*1024u;
    for(auto& m:models_) uploadModelIncremental(*m,budget);
    drawScene();
    drawGestureOverlay();

    if (!statusText_.isEmpty()) {
        QPainter p(this); p.setPen(Qt::white); p.drawText(12,24,statusText_);
    }
    bool uploading=false;
    for(const auto& m:models_) if(m->cloud && (m->uploadPointCursor<m->cloud->size() || (m->meshMode && (m->uploadIndexCursor<m->visibleMeshIndices.size() || m->uploadPickCursor<m->expandedPickVertices.size())))) uploading=true;
    if(uploading) update();
}

void GlesPointCloudWidget::drawScene()
{
    for(auto& m:models_) if(m->visible && m->cloud) drawModel(*m);
}

void GlesPointCloudWidget::drawModel(Model& m)
{
    if (!m.glCreated || m.drawPointCount<=0) return;
    const auto mvp=currentMvp();
    QOpenGLShaderProgram& prog = m.meshMode ? meshProgram_ : pointProgram_;
    prog.bind();
    prog.setUniformValue("uMVP", QMatrix4x4(mvp.m.data()).transposed());
    prog.setUniformValue("uLightDir", QVector3D(0.3f,0.7f,0.6f));
    prog.setUniformValue("uPointSize", 3.0f*float(devicePixelRatioF()));
    prog.setUniformValue("uPointMode", m.meshMode ? 0u : 1u);
    glBindVertexArray(m.vao);
    if(m.meshMode && m.drawIndexCount>0) glDrawElements(GL_TRIANGLES,m.drawIndexCount,GL_UNSIGNED_INT,nullptr);
    else glDrawArrays(GL_POINTS,0,m.drawPointCount);
    glBindVertexArray(0); prog.release();
}

void GlesPointCloudWidget::drawGestureOverlay()
{
    if(!editGestureActive_) return;
    QPainter painter(this);
    QPen pen(QColor(255,190,0)); pen.setWidth(2); painter.setPen(pen); painter.setBrush(Qt::NoBrush);
    if(interactionMode_==InteractionMode::Rectangle) painter.drawRect(QRect(pressPos_,currentPos_).normalized());
    else if(interactionMode_==InteractionMode::Circle) {
        const int r=int(std::hypot(currentPos_.x()-pressPos_.x(),currentPos_.y()-pressPos_.y())); painter.drawEllipse(pressPos_,r,r);
    } else if(stroke_.size()>1) {
        QPolygon poly; for(const auto& p:stroke_) poly<<p; painter.drawPolyline(poly);
    }
}

pceditor::Mat4f GlesPointCloudWidget::currentMvp() const
{
    return camera_.mvp(std::max(1,int(width()*devicePixelRatioF())),std::max(1,int(height()*devicePixelRatioF())));
}

void GlesPointCloudWidget::fitView()
{
    if(auto* m=activeModel(); m && m->cloud) camera_.fit(*m->cloud);
    update();
}

void GlesPointCloudWidget::uploadSelectionMask(Model& m)
{
    if(!m.glCreated) return;
    glBindBuffer(GL_ARRAY_BUFFER,m.selectionVbo);
    glBufferSubData(GL_ARRAY_BUFFER,0,static_cast<GLsizeiptr>(m.selectionMask.size()*sizeof(std::uint32_t)),m.selectionMask.data());
}

void GlesPointCloudWidget::uploadChangedFlags(Model& m, const std::vector<pceditor::PointId>& ids)
{
    if(!m.glCreated || !m.cloud) return;
    // 合并连续 ID，减少 glBufferSubData 调用次数。
    auto sorted=ids; std::sort(sorted.begin(),sorted.end()); sorted.erase(std::unique(sorted.begin(),sorted.end()),sorted.end());
    std::size_t i=0;
    while(i<sorted.size()) {
        std::size_t j=i+1; while(j<sorted.size() && sorted[j]==sorted[j-1]+1u) ++j;
        std::vector<std::uint32_t> flags(j-i);
        for(std::size_t k=i;k<j;++k) flags[k-i]=m.cloud->points()[sorted[k]].flags;
        glBindBuffer(GL_ARRAY_BUFFER,m.flagsVbo);
        glBufferSubData(GL_ARRAY_BUFFER,static_cast<GLintptr>(sorted[i]*sizeof(std::uint32_t)),static_cast<GLsizeiptr>(flags.size()*sizeof(std::uint32_t)),flags.data());
        i=j;
    }
}

void GlesPointCloudWidget::clearSelection()
{
    auto* m=activeModel(); if(!m) return;
    std::fill(m->selectionMask.begin(),m->selectionMask.end(),0u); m->selectedIds.clear(); m->editor.clearSelection();
    makeCurrent(); uploadSelectionMask(*m); doneCurrent(); update();
}

void GlesPointCloudWidget::applySelection(Model& m, std::vector<pceditor::PointId> ids, Qt::KeyboardModifiers modifiers)
{
    ids=sortedUnique(std::move(ids));
    ids.erase(std::remove_if(ids.begin(),ids.end(),[&](auto id){return id>=m.cloud->size() || (m.cloud->points()[id].flags&pceditor::PointDeleted)!=0;}),ids.end());
    if(modifiers.testFlag(Qt::ShiftModifier)) {
        auto all=m.selectedIds; all.insert(all.end(),ids.begin(),ids.end()); m.selectedIds=sortedUnique(std::move(all));
    } else if(modifiers.testFlag(Qt::AltModifier)) {
        std::vector<pceditor::PointId> out; std::set_difference(m.selectedIds.begin(),m.selectedIds.end(),ids.begin(),ids.end(),std::back_inserter(out)); m.selectedIds=std::move(out);
    } else m.selectedIds=std::move(ids);
    std::fill(m.selectionMask.begin(),m.selectionMask.end(),0u); for(auto id:m.selectedIds) if(id<m.selectionMask.size()) m.selectionMask[id]=1u;
    m.editor.select(m.selectedIds);
    makeCurrent(); uploadSelectionMask(m); doneCurrent(); update();
}

std::vector<pceditor::PointId> GlesPointCloudWidget::filterPickedIds(const Model& m,const std::vector<std::uint32_t>& raw,bool triangleIds) const
{
    std::vector<pceditor::PointId> out;
    if(!triangleIds){ for(auto id:raw) if(id!=pceditor::kInvalidPointId && id<m.cloud->size()) out.push_back(id); return sortedUnique(std::move(out)); }
    for(auto tid:raw){ if(tid==pceditor::kInvalidPointId || tid>=m.mesh.triangleCount()) continue; const std::size_t b=std::size_t(tid)*3u; out.push_back(m.mesh.triangleIndices[b]); out.push_back(m.mesh.triangleIndices[b+1]); out.push_back(m.mesh.triangleIndices[b+2]); }
    return sortedUnique(std::move(out));
}

void GlesPointCloudWidget::performSurfaceSelection(Qt::KeyboardModifiers modifiers)
{
    auto* m=activeModel(); if(!m || !m->glCreated) return;
    if (!pickingReady_) {
        // 某些定制驱动若整数 FBO 不可用，不让选择功能失效，直接退化为 CPU 穿透选择。
        performThroughSelection(modifiers);
        return;
    }
    makeCurrent();
    createPickingFramebuffer(std::max(1,int(width()*devicePixelRatioF())),std::max(1,int(height()*devicePixelRatioF())));
    glBindFramebuffer(GL_FRAMEBUFFER,pickFbo_); glViewport(0,0,pickWidth_,pickHeight_);
    const GLuint clearId=pceditor::kInvalidPointId; glClearBufferuiv(GL_COLOR,0,&clearId); glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST); glDisable(GL_BLEND); glDisable(GL_DITHER);
    const auto mvp=currentMvp();
    bool tri=false;
    if(m->meshMode && m->pickVertexCount>0){ tri=true; meshPickProgram_.bind(); meshPickProgram_.setUniformValue("uMVP",QMatrix4x4(mvp.m.data()).transposed()); glBindVertexArray(m->pickVao); glDrawArrays(GL_TRIANGLES,0,m->pickVertexCount); glBindVertexArray(0); meshPickProgram_.release(); }
    else { pointPickProgram_.bind(); pointPickProgram_.setUniformValue("uMVP",QMatrix4x4(mvp.m.data()).transposed()); pointPickProgram_.setUniformValue("uPointSize",4.0f*float(devicePixelRatioF())); glBindVertexArray(m->vao); glDrawArrays(GL_POINTS,0,m->drawPointCount); glBindVertexArray(0); pointPickProgram_.release(); }

    const qreal dpr=devicePixelRatioF();
    QRect r(QPoint(std::min(pressPos_.x(),currentPos_.x()),std::min(pressPos_.y(),currentPos_.y())),QPoint(std::max(pressPos_.x(),currentPos_.x()),std::max(pressPos_.y(),currentPos_.y())));
    if(interactionMode_==InteractionMode::Brush) r=QRect(currentPos_.x()-brushRadiusPixels_,currentPos_.y()-brushRadiusPixels_,brushRadiusPixels_*2+1,brushRadiusPixels_*2+1);
    int x=std::clamp(int(r.left()*dpr),0,pickWidth_-1), yTop=std::clamp(int(r.top()*dpr),0,pickHeight_-1);
    int rw=std::clamp(int(r.width()*dpr),1,pickWidth_-x), rh=std::clamp(int(r.height()*dpr),1,pickHeight_-yTop);
    int readY=pickHeight_-(yTop+rh); readY=std::max(0,readY);
    std::vector<std::uint32_t> pixels(std::size_t(rw)*std::size_t(rh),clearId);
    glReadPixels(x,readY,rw,rh,GL_RED_INTEGER,GL_UNSIGNED_INT,pixels.data());
    glBindFramebuffer(GL_FRAMEBUFFER,defaultFramebufferObject()); doneCurrent();

    // 对圆形/套索/Brush 再按实际 2D 形状过滤像素，避免只取 bounding box。
    std::vector<std::uint32_t> filtered; filtered.reserve(pixels.size());
    for(int py=0;py<rh;++py) for(int px=0;px<rw;++px){
        const std::uint32_t id=pixels[std::size_t(py)*rw+px]; if(id==clearId) continue;
        const int logicalX=int((x+px)/dpr), logicalY=int((yTop+(rh-1-py))/dpr); QPoint lp(logicalX,logicalY); bool hit=true;
        if(interactionMode_==InteractionMode::Circle){ const double dx=lp.x()-pressPos_.x(),dy=lp.y()-pressPos_.y(); const double rr=std::hypot(currentPos_.x()-pressPos_.x(),currentPos_.y()-pressPos_.y()); hit=dx*dx+dy*dy<=rr*rr; }
        else if(interactionMode_==InteractionMode::Lasso) hit=pointInPolygon(lp,stroke_);
        else if(interactionMode_==InteractionMode::Brush){ const double dx=lp.x()-currentPos_.x(),dy=lp.y()-currentPos_.y(); hit=dx*dx+dy*dy<=double(brushRadiusPixels_*brushRadiusPixels_); }
        if(hit) filtered.push_back(id);
    }
    applySelection(*m,filterPickedIds(*m,filtered,tri),modifiers);
}

void GlesPointCloudWidget::performThroughSelection(Qt::KeyboardModifiers modifiers)
{
    auto* m=activeModel(); if(!m || !m->cloud) return;
    const auto mvp=currentMvp(); const pceditor::Viewport vp{width(),height()}; std::vector<pceditor::PointId> ids;
    if(interactionMode_==InteractionMode::Rectangle){ ids=pceditor::CpuSelector::rectangle(*m->cloud,mvp,vp,{pressPos_.x(),pressPos_.y(),currentPos_.x(),currentPos_.y()}); }
    else if(interactionMode_==InteractionMode::Circle){ const int r=int(std::hypot(currentPos_.x()-pressPos_.x(),currentPos_.y()-pressPos_.y())); ids=pceditor::CpuSelector::circle(*m->cloud,mvp,vp,{pressPos_.x(),pressPos_.y()},r); }
    else { std::vector<pceditor::Point2i> path; path.reserve(stroke_.size()); for(const auto& p:stroke_)path.push_back({p.x(),p.y()}); ids=(interactionMode_==InteractionMode::Lasso)?pceditor::CpuSelector::lasso(*m->cloud,mvp,vp,path):pceditor::CpuSelector::brushStroke(*m->cloud,mvp,vp,path,brushRadiusPixels_); }
    applySelection(*m,std::move(ids),modifiers);
}

void GlesPointCloudWidget::beginEditGesture(const QPoint& pos, Qt::KeyboardModifiers)
{
    editGestureActive_=true; pressPos_=currentPos_=pos; stroke_.clear(); stroke_.push_back(pos); update();
}
void GlesPointCloudWidget::updateEditGesture(const QPoint& pos){ if(!editGestureActive_)return; currentPos_=pos; if(interactionMode_==InteractionMode::Lasso||interactionMode_==InteractionMode::Brush) stroke_.push_back(pos); update(); }
void GlesPointCloudWidget::finishEditGesture(const QPoint& pos, Qt::KeyboardModifiers modifiers){ if(!editGestureActive_)return; updateEditGesture(pos); editGestureActive_=false; if(selectionDepthMode_==SelectionDepthMode::Surface)performSurfaceSelection(modifiers);else performThroughSelection(modifiers); stroke_.clear(); update(); }
void GlesPointCloudWidget::cancelEditGesture(){editGestureActive_=false;stroke_.clear();update();}

void GlesPointCloudWidget::mousePressEvent(QMouseEvent* e)
{
    if(e->button()==Qt::LeftButton && (e->modifiers().testFlag(Qt::ControlModifier)||touchEditMode_)){ beginEditGesture(e->position().toPoint(),e->modifiers()); return; }
    viewDragging_=true; viewButton_=e->button(); lastViewPos_=e->position().toPoint();
}
void GlesPointCloudWidget::mouseMoveEvent(QMouseEvent* e)
{
    if(editGestureActive_){updateEditGesture(e->position().toPoint());return;}
    if(!viewDragging_)return; const QPoint p=e->position().toPoint(),d=p-lastViewPos_; lastViewPos_=p;
    if(viewButton_==Qt::LeftButton)camera_.orbit(float(d.x()),float(d.y()),width(),height()); else camera_.pan(float(d.x()),float(d.y()),width(),height()); update();
}
void GlesPointCloudWidget::mouseReleaseEvent(QMouseEvent* e)
{
    if(editGestureActive_){finishEditGesture(e->position().toPoint(),e->modifiers());return;} viewDragging_=false;viewButton_=Qt::NoButton;
}
void GlesPointCloudWidget::wheelEvent(QWheelEvent* e){camera_.zoom(float(e->angleDelta().y())/120.0f);update();}

bool GlesPointCloudWidget::event(QEvent* event)
{
    switch(event->type()){
    case QEvent::TouchBegin: case QEvent::TouchUpdate: case QEvent::TouchEnd: case QEvent::TouchCancel:
        return handleTouchEvent(static_cast<QTouchEvent*>(event));
    default: break;
    }
    return QOpenGLWidget::event(event);
}

bool GlesPointCloudWidget::handleTouchEvent(QTouchEvent* e)
{
    const auto pts=e->points();
    if(e->type()==QEvent::TouchCancel){twoFingerActive_=false;cancelEditGesture();return true;}
    if(pts.size()>=2){
        // 两指始终优先作为浏览手势：即使“编辑模式”开启，也能随时缩放/平移模型。
        if(editGestureActive_)cancelEditGesture();
        const QPointF p0=pts[0].position(),p1=pts[1].position(); const QPointF center=(p0+p1)*0.5; const qreal dist=QLineF(p0,p1).length();
        if(!twoFingerActive_){twoFingerActive_=true;lastTouchCenter_=center;lastTouchDistance_=std::max<qreal>(dist,1.0);}
        else { const QPointF d=center-lastTouchCenter_; camera_.pan(float(d.x()),float(d.y()),width(),height()); if(dist>1.0&&lastTouchDistance_>1.0){const qreal ratio=dist/lastTouchDistance_; camera_.zoom(float(std::log(ratio)/std::log(1.18)));} lastTouchCenter_=center; lastTouchDistance_=dist; }
        update(); if(e->type()==QEvent::TouchEnd)twoFingerActive_=false; return true;
    }
    twoFingerActive_=false;
    if(pts.size()==1){
        const QPoint p=pts[0].position().toPoint();
        if(e->type()==QEvent::TouchBegin){ if(touchEditMode_)beginEditGesture(p); else {viewDragging_=true;viewButton_=Qt::LeftButton;lastViewPos_=p;} }
        else if(e->type()==QEvent::TouchUpdate){ if(editGestureActive_)updateEditGesture(p); else if(viewDragging_){const QPoint d=p-lastViewPos_;lastViewPos_=p;camera_.orbit(float(d.x()),float(d.y()),width(),height());update();} }
        else if(e->type()==QEvent::TouchEnd){ if(editGestureActive_)finishEditGesture(p); viewDragging_=false; }
        return true;
    }
    if(e->type()==QEvent::TouchEnd){ if(editGestureActive_)finishEditGesture(currentPos_); viewDragging_=false; twoFingerActive_=false; }
    return true;
}

void GlesPointCloudWidget::deleteSelection()
{
    auto* m=activeModel(); if(!m||m->selectedIds.empty()||editBusy_)return; editBusy_=true; const auto ids=m->selectedIds; QPointer<GlesPointCloudWidget> self(this); const QString path=m->path;
    workerPool_.start([self,path,ids]{ if(!self)return; QMetaObject::invokeMethod(self,[self,path,ids]{ if(!self)return; int i=self->findModel(path); if(i<0)return; auto& model=*self->models_[std::size_t(i)]; model.editor.select(ids); model.editor.deleteSelection(); const auto changed=model.editor.lastChangedIds(); std::fill(model.selectionMask.begin(),model.selectionMask.end(),0u); model.selectedIds.clear(); self->makeCurrent(); self->uploadChangedFlags(model,changed); self->uploadSelectionMask(model); self->doneCurrent(); self->rebuildVisibleMeshAsync(model); self->editBusy_=false; self->update(); },Qt::QueuedConnection); });
}

void GlesPointCloudWidget::undoEdit(){auto* m=activeModel();if(!m||editBusy_)return; if(m->editor.undo()){makeCurrent();uploadChangedFlags(*m,m->editor.lastChangedIds());doneCurrent();rebuildVisibleMeshAsync(*m);update();}}
void GlesPointCloudWidget::redoEdit(){auto* m=activeModel();if(!m||editBusy_)return; if(m->editor.redo()){makeCurrent();uploadChangedFlags(*m,m->editor.lastChangedIds());doneCurrent();rebuildVisibleMeshAsync(*m);update();}}

void GlesPointCloudWidget::rebuildVisibleMeshAsync(Model& model)
{
    if(!model.meshMode||!model.cloud)return; const QString path=model.path; const auto mesh=model.mesh; auto cloud=model.cloud; QPointer<GlesPointCloudWidget> self(this);
    workerPool_.start([self,path,mesh,cloud]{
        std::vector<std::uint32_t> visible; visible.reserve(mesh.triangleIndices.size()); std::vector<PickVertex> pick; pick.reserve(mesh.triangleIndices.size());
        const std::size_t triCount=mesh.triangleCount();
        for(std::size_t t=0;t<triCount;++t){const auto a=mesh.triangleIndices[t*3],b=mesh.triangleIndices[t*3+1],c=mesh.triangleIndices[t*3+2]; if(a>=cloud->size()||b>=cloud->size()||c>=cloud->size())continue; if((cloud->points()[a].flags&pceditor::PointDeleted)||(cloud->points()[b].flags&pceditor::PointDeleted)||(cloud->points()[c].flags&pceditor::PointDeleted))continue; visible.insert(visible.end(),{a,b,c}); const auto tid=std::uint32_t(t); pick.push_back({cloud->points()[a].position,tid});pick.push_back({cloud->points()[b].position,tid});pick.push_back({cloud->points()[c].position,tid});}
        QMetaObject::invokeMethod(self,[self,path,visible=std::move(visible),pick=std::move(pick)]()mutable{if(!self)return;int i=self->findModel(path);if(i<0)return;auto& m=*self->models_[std::size_t(i)];m.visibleMeshIndices=std::move(visible);m.expandedPickVertices=std::move(pick);m.uploadIndexCursor=0;m.drawIndexCount=0;m.uploadPickCursor=0;m.pickVertexCount=0;self->makeCurrent();self->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m.meshEbo);self->glBufferData(GL_ELEMENT_ARRAY_BUFFER,static_cast<GLsizeiptr>(m.visibleMeshIndices.size()*sizeof(std::uint32_t)),nullptr,GL_DYNAMIC_DRAW);self->glBindBuffer(GL_ARRAY_BUFFER,m.pickPositionVbo);self->glBufferData(GL_ARRAY_BUFFER,static_cast<GLsizeiptr>(m.expandedPickVertices.size()*sizeof(pceditor::Vec3f)),nullptr,GL_DYNAMIC_DRAW);self->glBindBuffer(GL_ARRAY_BUFFER,m.pickIdVbo);self->glBufferData(GL_ARRAY_BUFFER,static_cast<GLsizeiptr>(m.expandedPickVertices.size()*sizeof(std::uint32_t)),nullptr,GL_DYNAMIC_DRAW);self->doneCurrent();self->update();},Qt::QueuedConnection);
    });
}

void GlesPointCloudWidget::saveActiveModel()
{
    auto* m=activeModel(); if(!m||!m->cloud)return; std::filesystem::path p(m->path.toStdString()); p.replace_filename(p.stem().string()+"_edited.ply"); std::string msg; if(pceditor::PointCloudIO::savePly(p.string(),*m->cloud,&msg))statusText_=QString::fromUtf8("已保存：")+QString::fromStdString(p.string());else statusText_=QString::fromUtf8("保存失败：")+QString::fromUtf8(msg.c_str()); update();
}

void GlesPointCloudWidget::keyPressEvent(QKeyEvent* e)
{
    if(e->key()==Qt::Key_Delete)deleteSelection(); else if(e->matches(QKeySequence::Undo))undoEdit(); else if(e->matches(QKeySequence::Redo))redoEdit(); else if(e->key()==Qt::Key_F)fitView(); else if(e->key()==Qt::Key_Escape)clearSelection(); else QOpenGLWidget::keyPressEvent(e);
}

QPoint GlesPointCloudWidget::toPhysical(const QPointF& p) const { const qreal d=devicePixelRatioF();return QPoint(int(p.x()*d),int(p.y()*d)); }
void GlesPointCloudWidget::drawSelectionOverlay(Model&) {}

