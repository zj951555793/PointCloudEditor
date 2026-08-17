#include "PointCloudWidget.h"

#include <JMEngine/CpuSelector.h>
#include <JMEngine/Alignment.h>
#include <JMEngine/Measurement.h>
#include <JMEngine/PointCloudIO.h>
#include <JMEngine/ModelIO.h>
#include <JMEngine/processing/Parallel.h>

#include <QDebug>
#include <QDir>
#include <QEvent>
#include <QFileInfo>
#include <QImage>
#include <QLineF>
#include <QMatrix4x4>
#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <QSize>
#include <QVector3D>
#include <QVector2D>
#include <QVector4D>
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
#include <exception>
#include <iterator>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

namespace {

JMEngine::example::OrbitCamera::Quat quatFromBasis(const JMEngine::Vec3f& rightIn,
                                                     const JMEngine::Vec3f& upIn,
                                                     const JMEngine::Vec3f& backIn) {
    using Cam = JMEngine::example::OrbitCamera;
    const auto r = JMEngine::example::normalize(rightIn);
    const auto u = JMEngine::example::normalize(upIn);
    const auto b = JMEngine::example::normalize(backIn);
    // Rotation matrix columns are local +X/+Y/+Z expressed in world space.
    const float m00=r.x, m01=u.x, m02=b.x;
    const float m10=r.y, m11=u.y, m12=b.y;
    const float m20=r.z, m21=u.z, m22=b.z;
    Cam::Quat q{};
    const float tr = m00 + m11 + m22;
    if (tr > 0.0f) {
        const float ss = std::sqrt(tr + 1.0f) * 2.0f;
        q.w = 0.25f * ss; q.x = (m21-m12)/ss; q.y = (m02-m20)/ss; q.z = (m10-m01)/ss;
    } else if (m00 > m11 && m00 > m22) {
        const float ss = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        q.w=(m21-m12)/ss; q.x=0.25f*ss; q.y=(m01+m10)/ss; q.z=(m02+m20)/ss;
    } else if (m11 > m22) {
        const float ss = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        q.w=(m02-m20)/ss; q.x=(m01+m10)/ss; q.y=0.25f*ss; q.z=(m12+m21)/ss;
    } else {
        const float ss = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        q.w=(m10-m01)/ss; q.x=(m02+m20)/ss; q.y=(m12+m21)/ss; q.z=0.25f*ss;
    }
    return Cam::normalizeQuat(q);
}


bool bakeTexture(const JMEngine::ObjAppearanceData& appearance, JMEngine::PointCloud& cloud) {
    if (appearance.diffuseTexturePath.empty() || !appearance.hasTextureCoordinates())
        return false;
    QImage image(QString::fromUtf8(appearance.diffuseTexturePath.c_str()));
    if (image.isNull())
        return false;
    image = image.convertToFormat(QImage::Format_RGBA8888);
    const int width = image.width();
    const int height = image.height();
    const qsizetype stride = image.bytesPerLine();
    const uchar* bits = image.constBits();
    if (width <= 0 || height <= 0 || !bits)
        return false;

    const std::size_t count = std::min(cloud.size(), appearance.vertexUv.size());
#ifdef JMENGINE_USE_OPENMP
#pragma omp parallel for schedule(static) num_threads(JMEngine::processing::processingThreadCount())
#endif
    for (std::int64_t ii = 0; ii < static_cast<std::int64_t>(count); ++ii) {
        const std::size_t i = static_cast<std::size_t>(ii);
        if (i >= appearance.hasUv.size() || appearance.hasUv[i] == 0u)
            continue;
        const auto uv = appearance.vertexUv[i];
        const int x =
            std::clamp(static_cast<int>(std::lround(std::clamp(uv.x, 0.0f, 1.0f) * (width - 1))), 0, width - 1);
        const int y = std::clamp(static_cast<int>(std::lround((1.0f - std::clamp(uv.y, 0.0f, 1.0f)) * (height - 1))), 0,
                                 height - 1);
        const uchar* px = bits + static_cast<qsizetype>(y) * stride + x * 4;
        cloud.points()[i].rgba = static_cast<std::uint32_t>(px[0]) | (static_cast<std::uint32_t>(px[1]) << 8u) |
                                 (static_cast<std::uint32_t>(px[2]) << 16u) |
                                 (static_cast<std::uint32_t>(px[3]) << 24u);
    }
    return true;
}

std::vector<JMEngine::PointId> sortedUnique(std::vector<JMEngine::PointId> ids) {
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    return ids;
}

std::uint32_t packedColor(const QColor& c) {
    return static_cast<std::uint32_t>(c.red()) | (static_cast<std::uint32_t>(c.green()) << 8u) |
           (static_cast<std::uint32_t>(c.blue()) << 16u) | (static_cast<std::uint32_t>(c.alpha()) << 24u);
}


// Export must never operate on the scene-owned PointCloud/TriangleMesh instances directly.
// The scene uses shared_ptr heavily (mesh vertices share the PointCloud), so merely copying
// shared_ptrs is NOT a snapshot.  Keep this deep-copy helper deliberately boring: no compact,
// remap, cleanup or topology conversion is allowed here.
struct ExportSnapshot {
    std::shared_ptr<JMEngine::PointCloud> cloud;
    std::shared_ptr<JMEngine::TriangleMesh> mesh;
    bool meshMode{false};
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    std::shared_ptr<JMEngine::texture::Result> textureResult;
#endif
};

std::shared_ptr<JMEngine::PointCloud> deepCopyPointCloud(const std::shared_ptr<JMEngine::PointCloud>& src) {
    if (!src)
        return {};
    return std::make_shared<JMEngine::PointCloud>(src->points());
}

std::shared_ptr<JMEngine::TriangleMesh> deepCopyTriangleMesh(
    const std::shared_ptr<JMEngine::TriangleMesh>& src,
    const std::shared_ptr<JMEngine::PointCloud>& preferredVertices) {
    if (!src)
        return {};

    auto vertices = preferredVertices;
    if (!vertices) {
        vertices = deepCopyPointCloud(src->vertices());
    }
    auto out = std::make_shared<JMEngine::TriangleMesh>(vertices, src->indices());
    // Constructor initializes all flags to valid.  Preserve the exact scene deletion/selection
    // state without compacting or changing TriangleId ordering.
    out->triangleFlags() = src->triangleFlags();
    return out;
}

#ifdef JMENGINE_HAS_TEXTURE_MAPPING
std::shared_ptr<JMEngine::texture::Result> deepCopyTextureResult(
    const std::shared_ptr<JMEngine::texture::Result>& src) {
    if (!src)
        return {};
    auto out = std::make_shared<JMEngine::texture::Result>(*src);
    // Result is mostly value-owned, except its split texture vertex cloud.
    out->vertices = deepCopyPointCloud(src->vertices);
    return out;
}
#endif

ExportSnapshot makeExportSnapshot(
    const std::shared_ptr<JMEngine::PointCloud>& sceneCloud,
    const std::shared_ptr<JMEngine::TriangleMesh>& sceneMesh,
    bool meshMode
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    , const std::shared_ptr<JMEngine::texture::Result>& sceneTextureResult
#endif
) {
    ExportSnapshot out;
    out.cloud = deepCopyPointCloud(sceneCloud);
    out.meshMode = meshMode;
    out.mesh = deepCopyTriangleMesh(sceneMesh, out.cloud);
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    out.textureResult = deepCopyTextureResult(sceneTextureResult);
#endif
    return out;
}



JMEngine::Vec3f smallestEigenVector3x3(float a00, float a01, float a02, float a11, float a12, float a22) {
    float a[3][3]{{a00,a01,a02},{a01,a11,a12},{a02,a12,a22}};
    float v[3][3]{{1,0,0},{0,1,0},{0,0,1}};
    for (int it=0; it<16; ++it) {
        int p=0,q=1;
        float m=std::fabs(a[0][1]);
        if (std::fabs(a[0][2])>m){p=0;q=2;m=std::fabs(a[0][2]);}
        if (std::fabs(a[1][2])>m){p=1;q=2;m=std::fabs(a[1][2]);}
        if (m<1e-10f) break;
        const float phi=0.5f*std::atan2(2.0f*a[p][q], a[q][q]-a[p][p]);
        const float c=std::cos(phi), sn=std::sin(phi);
        for(int k=0;k<3;++k){
            const float apk=a[p][k], aqk=a[q][k];
            a[p][k]=c*apk-sn*aqk; a[q][k]=sn*apk+c*aqk;
        }
        for(int k=0;k<3;++k){
            const float akp=a[k][p], akq=a[k][q];
            a[k][p]=c*akp-sn*akq; a[k][q]=sn*akp+c*akq;
        }
        for(int k=0;k<3;++k){
            const float vkp=v[k][p], vkq=v[k][q];
            v[k][p]=c*vkp-sn*vkq; v[k][q]=sn*vkp+c*vkq;
        }
    }
    int k=0; if(a[1][1]<a[k][k])k=1; if(a[2][2]<a[k][k])k=2;
    JMEngine::Vec3f n{v[0][k],v[1][k],v[2][k]};
    const float l=std::sqrt(n.x*n.x+n.y*n.y+n.z*n.z);
    if(l<1e-12f) return {0,1,0};
    return {n.x/l,n.y/l,n.z/l};
}

bool fitPlanePca(const std::vector<JMEngine::Vec3f>& pts, JMEngine::Vec3f& center, JMEngine::Vec3f& normal) {
    if (pts.size() < 3) return false;
    double sx=0,sy=0,sz=0;
    for(const auto&p:pts){sx+=p.x;sy+=p.y;sz+=p.z;}
    center={float(sx/pts.size()),float(sy/pts.size()),float(sz/pts.size())};
    double xx=0,xy=0,xz=0,yy=0,yz=0,zz=0;
    for(const auto&p:pts){const double x=p.x-center.x,y=p.y-center.y,z=p.z-center.z;xx+=x*x;xy+=x*y;xz+=x*z;yy+=y*y;yz+=y*z;zz+=z*z;}
    normal=smallestEigenVector3x3(float(xx),float(xy),float(xz),float(yy),float(yz),float(zz));
    return std::isfinite(normal.x)&&std::isfinite(normal.y)&&std::isfinite(normal.z);
}

float dot3(const JMEngine::Vec3f&a,const JMEngine::Vec3f&b){return a.x*b.x+a.y*b.y+a.z*b.z;}
JMEngine::Vec3f sub3(const JMEngine::Vec3f&a,const JMEngine::Vec3f&b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
JMEngine::Vec3f add3(const JMEngine::Vec3f&a,const JMEngine::Vec3f&b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
JMEngine::Vec3f mul3(const JMEngine::Vec3f&a,float s){return {a.x*s,a.y*s,a.z*s};}
JMEngine::Vec3f cross3(const JMEngine::Vec3f&a,const JMEngine::Vec3f&b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
JMEngine::Vec3f norm3(const JMEngine::Vec3f&a){const float l=std::sqrt(dot3(a,a));return l>1e-12f?mul3(a,1.0f/l):JMEngine::Vec3f{0,1,0};}

// Build the geometry that is actually visible in the renderer.  This is export-only data:
// the scene-owned topology/PointIds are never compacted or remapped.
ExportSnapshot makeRenderedExportSnapshot(const ExportSnapshot& raw) {
    ExportSnapshot out;
    out.meshMode = raw.meshMode;
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    out.textureResult = raw.textureResult;
#endif
    if (!raw.cloud) return out;
    if (!raw.meshMode || !raw.mesh) {
        JMEngine::PointCloud::Container pts;
        pts.reserve(raw.cloud->activeCount());
        for (const auto& p : raw.cloud->points()) if ((p.flags & JMEngine::PointDeleted)==0) {
            auto q=p; q.flags = static_cast<std::uint8_t>(q.flags & ~JMEngine::PointSelected); pts.push_back(q);
        }
        out.cloud=std::make_shared<JMEngine::PointCloud>(std::move(pts));
        return out;
    }
    const auto& oldPts=raw.cloud->points();
    const auto& idx=raw.mesh->indices();
    std::vector<JMEngine::PointId> map(oldPts.size(), JMEngine::kInvalidPointId);
    JMEngine::PointCloud::Container pts;
    std::vector<std::uint32_t> newIdx;
    newIdx.reserve(raw.mesh->activeTriangleCount()*3u);
    for(std::size_t t=0;t+2<idx.size();t+=3){
        const auto tid=static_cast<JMEngine::TriangleId>(t/3u);
        if(!raw.mesh->triangleActive(tid)) continue;
        const std::uint32_t ids[3]{idx[t],idx[t+1],idx[t+2]};
        bool ok=true; for(auto id:ids) if(id>=oldPts.size() || (oldPts[id].flags&JMEngine::PointDeleted)) ok=false;
        if(!ok) continue;
        for(auto id:ids){
            if(map[id]==JMEngine::kInvalidPointId){auto q=oldPts[id];q.flags=static_cast<std::uint8_t>(q.flags&~JMEngine::PointSelected);map[id]=static_cast<JMEngine::PointId>(pts.size());pts.push_back(q);} 
            newIdx.push_back(map[id]);
        }
    }
    out.cloud=std::make_shared<JMEngine::PointCloud>(std::move(pts));
    out.mesh=std::make_shared<JMEngine::TriangleMesh>(out.cloud,std::move(newIdx));
    return out;
}

bool pointInPolygon(const QPoint& p, const std::vector<QPoint>& polygon) {
    if (polygon.size() < 3)
        return false;
    bool inside = false;
    for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const QPoint& a = polygon[i];
        const QPoint& b = polygon[j];
        const bool crossing =
            ((a.y() > p.y()) != (b.y() > p.y())) &&
            (p.x() < (b.x() - a.x()) * (p.y() - a.y()) / double(b.y() - a.y() == 0 ? 1 : b.y() - a.y()) + a.x());
        if (crossing)
            inside = !inside;
    }
    return inside;
}

QRect gestureBounds(PointCloudWidget::InteractionMode mode, const QPoint& press, const QPoint& current,
                    const std::vector<QPoint>& stroke, int brushRadius) {
    if (mode == PointCloudWidget::InteractionMode::Rectangle)
        return QRect(press, current).normalized();
    if (mode == PointCloudWidget::InteractionMode::Circle) {
        const int r = int(std::ceil(std::hypot(current.x() - press.x(), current.y() - press.y())));
        return QRect(press.x() - r, press.y() - r, r * 2 + 1, r * 2 + 1);
    }
    if (stroke.empty())
        return QRect(current, QSize(1, 1));
    int l = stroke.front().x(), r = l, t = stroke.front().y(), b = t;
    for (const auto& q : stroke) {
        l = std::min(l, q.x());
        r = std::max(r, q.x());
        t = std::min(t, q.y());
        b = std::max(b, q.y());
    }
    if (mode == PointCloudWidget::InteractionMode::Brush) {
        l -= brushRadius;
        r += brushRadius;
        t -= brushRadius;
        b += brushRadius;
    }
    return QRect(QPoint(l, t), QPoint(r, b)).normalized();
}

double pointSegmentDistanceSquared(const QPoint& p, const QPoint& a, const QPoint& b) {
    const double vx = b.x() - a.x(), vy = b.y() - a.y();
    const double wx = p.x() - a.x(), wy = p.y() - a.y();
    const double den = vx * vx + vy * vy;
    const double u = den > 1.0e-12 ? std::clamp((wx * vx + wy * vy) / den, 0.0, 1.0) : 0.0;
    const double dx = p.x() - (a.x() + u * vx), dy = p.y() - (a.y() + u * vy);
    return dx * dx + dy * dy;
}

bool gestureContains(PointCloudWidget::InteractionMode mode, const QPoint& p, const QPoint& press,
                     const QPoint& current, const std::vector<QPoint>& stroke, int brushRadius) {
    if (mode == PointCloudWidget::InteractionMode::Rectangle)
        return QRect(press, current).normalized().contains(p);
    if (mode == PointCloudWidget::InteractionMode::Circle) {
        const double dx = p.x() - press.x(), dy = p.y() - press.y();
        const double r = std::hypot(current.x() - press.x(), current.y() - press.y());
        return dx * dx + dy * dy <= r * r;
    }
    if (mode == PointCloudWidget::InteractionMode::Lasso)
        return pointInPolygon(p, stroke);
    if (stroke.empty())
        return false;
    const double r2 = double(brushRadius) * double(brushRadius);
    if (stroke.size() == 1) {
        const double dx = p.x() - stroke[0].x(), dy = p.y() - stroke[0].y();
        return dx * dx + dy * dy <= r2;
    }
    for (std::size_t i = 1; i < stroke.size(); ++i)
        if (pointSegmentDistanceSquared(p, stroke[i - 1], stroke[i]) <= r2)
            return true;
    return false;
}


bool projectPointToScreen(const JMEngine::Vec3f& position, const JMEngine::Mat4f& mvp, int viewportWidth,
                          int viewportHeight, float& screenX, float& screenY, float& depth01) {
    const float x = position.x, y = position.y, z = position.z;
    const float clipX = mvp.m[0] * x + mvp.m[4] * y + mvp.m[8] * z + mvp.m[12];
    const float clipY = mvp.m[1] * x + mvp.m[5] * y + mvp.m[9] * z + mvp.m[13];
    const float clipZ = mvp.m[2] * x + mvp.m[6] * y + mvp.m[10] * z + mvp.m[14];
    const float clipW = mvp.m[3] * x + mvp.m[7] * y + mvp.m[11] * z + mvp.m[15];
    if (clipW <= 0.0f)
        return false;

    const float ndcX = clipX / clipW;
    const float ndcY = clipY / clipW;
    const float ndcZ = clipZ / clipW;
    if (ndcX < -1.0f || ndcX > 1.0f || ndcY < -1.0f || ndcY > 1.0f || ndcZ < -1.0f || ndcZ > 1.0f)
        return false;

    screenX = (ndcX * 0.5f + 0.5f) * float(viewportWidth);
    screenY = (1.0f - (ndcY * 0.5f + 0.5f)) * float(viewportHeight);
    depth01 = ndcZ * 0.5f + 0.5f;
    return true;
}

float adaptiveDepthTolerance(const std::vector<float>& depthPixels, int width, int height, int x, int y,
                             float frontDepth) {
    if (!(frontDepth >= 0.0f && frontDepth < 1.0f) || width <= 0 || height <= 0)
        return 0.0f;

    // Surface picking must never turn into a depth slab / Through selection.
    // The previous V11.3 distance-based shell (up to 0.0025 in depth01) becomes extremely thick
    // under perspective at long range and can include geometry behind the visible surface.
    //
    // Visible dense points are recovered instead by testing the complete rendered point footprint
    // (see surfaceDepthVisible below), not by widening the depth threshold.
    // Keep only a tiny tolerance for depth-buffer quantization and rasterization round-off.
    const float quantization = 8.0f / 16777215.0f; // conservative 24-bit depth allowance
    const float floating = 16.0f * std::numeric_limits<float>::epsilon() *
                           std::max(1.0f, std::abs(frontDepth));

    // A very small local allowance handles depth interpolation / float readback differences while
    // remaining orders of magnitude below the old 0.0025 shell.
    float local = 0.0f;
    for (int dy = -1; dy <= 1; ++dy) {
        const int yy = y + dy;
        if (yy < 0 || yy >= height)
            continue;
        for (int dx = -1; dx <= 1; ++dx) {
            const int xx = x + dx;
            if (xx < 0 || xx >= width || (dx == 0 && dy == 0))
                continue;
            const float d = depthPixels[std::size_t(yy) * std::size_t(width) + std::size_t(xx)];
            if (d >= 0.0f && d < 1.0f)
                local = std::max(local, std::min(std::abs(d - frontDepth), 2.0e-5f));
        }
    }
    return std::min(5.0e-5f, quantization + floating + local);
}

bool surfaceDepthVisible(const std::vector<float>& depthPixels, int width, int height,
                         int centerX, int centerY, float pointDepth, int footprintRadius = 2) {
    if (width <= 0 || height <= 0 || depthPixels.empty())
        return false;

    // A point rendered as a ~3 px sprite is visible if any pixel in its actual raster footprint
    // contains the same front depth. This recovers dense visible surface points without accepting
    // deeper layers at the projected center pixel.
    for (int dy = -footprintRadius; dy <= footprintRadius; ++dy) {
        const int y = centerY + dy;
        if (y < 0 || y >= height)
            continue;
        for (int dx = -footprintRadius; dx <= footprintRadius; ++dx) {
            const int x = centerX + dx;
            if (x < 0 || x >= width)
                continue;
            if (dx * dx + dy * dy > footprintRadius * footprintRadius)
                continue;
            const float front = depthPixels[std::size_t(y) * std::size_t(width) + std::size_t(x)];
            if (!(front >= 0.0f && front < 1.0f))
                continue;
            const float tol = adaptiveDepthTolerance(depthPixels, width, height, x, y, front);
            if (std::abs(pointDepth - front) <= tol)
                return true;
        }
    }
    return false;
}

std::vector<JMEngine::PointId> selectCandidatePointsAgainstGpuDepth(
    const JMEngine::PointCloud& cloud, const std::vector<JMEngine::PointId>& candidates, const JMEngine::Mat4f& mvp,
    int viewportWidth, int viewportHeight, qreal dpr, int pickWidth, int pickHeight, int readX, int readY,
    int readWidth, int readHeight, const std::vector<float>& depthPixels, PointCloudWidget::InteractionMode mode,
    const QPoint& press, const QPoint& current, const std::vector<QPoint>& stroke, int brushRadius) {
    std::vector<JMEngine::PointId> selected;
    if (viewportWidth <= 0 || viewportHeight <= 0 || depthPixels.empty() || candidates.empty())
        return selected;

    auto depthVisible = [&](float sx, float sy, float depth) {
        const int physicalX = std::clamp(int(std::floor(sx * float(dpr))), 0, pickWidth - 1);
        const int physicalYTop = std::clamp(int(std::floor(sy * float(dpr))), 0, pickHeight - 1);
        const int physicalYGl = pickHeight - 1 - physicalYTop;
        const int lx = physicalX - readX;
        const int ly = physicalYGl - readY;
        if (lx < 0 || lx >= readWidth || ly < 0 || ly >= readHeight)
            return false;
        return surfaceDepthVisible(depthPixels, readWidth, readHeight, lx, ly, depth, 2);
    };

#ifdef JMENGINE_USE_OPENMP
    const int threadCount = JMEngine::processing::processingThreadCount();
    std::vector<std::vector<JMEngine::PointId>> perThread(static_cast<std::size_t>(threadCount));
#pragma omp parallel num_threads(threadCount)
    {
        auto& local = perThread[static_cast<std::size_t>(omp_get_thread_num())];
        local.reserve(candidates.size() / static_cast<std::size_t>(threadCount * 16) + 16u);
#pragma omp for schedule(static)
        for (std::int64_t ii = 0; ii < static_cast<std::int64_t>(candidates.size()); ++ii) {
            const auto id = candidates[static_cast<std::size_t>(ii)];
            if (static_cast<std::size_t>(id) >= cloud.size())
                continue;
            const auto& point = cloud.points()[static_cast<std::size_t>(id)];
            if ((point.flags & JMEngine::PointDeleted) != 0)
                continue;
            float sx = 0.0f, sy = 0.0f, depth = 1.0f;
            if (!projectPointToScreen(point.position, mvp, viewportWidth, viewportHeight, sx, sy, depth))
                continue;
            const QPoint logicalPos(int(std::floor(sx)), int(std::floor(sy)));
            if (gestureContains(mode, logicalPos, press, current, stroke, brushRadius) && depthVisible(sx, sy, depth))
                local.push_back(id);
        }
    }
    std::size_t total = 0;
    for (const auto& v : perThread)
        total += v.size();
    selected.reserve(total);
    for (auto& v : perThread)
        selected.insert(selected.end(), v.begin(), v.end());
#else
    selected.reserve(candidates.size() / 16u + 1u);
    for (const auto id : candidates) {
        if (static_cast<std::size_t>(id) >= cloud.size())
            continue;
        const auto& point = cloud.points()[static_cast<std::size_t>(id)];
        if ((point.flags & JMEngine::PointDeleted) != 0)
            continue;
        float sx = 0.0f, sy = 0.0f, depth = 1.0f;
        if (!projectPointToScreen(point.position, mvp, viewportWidth, viewportHeight, sx, sy, depth))
            continue;
        const QPoint logicalPos(int(std::floor(sx)), int(std::floor(sy)));
        if (gestureContains(mode, logicalPos, press, current, stroke, brushRadius) && depthVisible(sx, sy, depth))
            selected.push_back(id);
    }
#endif
    return sortedUnique(std::move(selected));
}

std::vector<JMEngine::PointId> selectCandidatePointsCpuSurface(
    const JMEngine::PointCloud& cloud, const std::vector<JMEngine::PointId>& candidates, const JMEngine::Mat4f& mvp,
    int viewportWidth, int viewportHeight, const QRect& logicalBounds, PointCloudWidget::InteractionMode mode,
    const QPoint& press, const QPoint& current, const std::vector<QPoint>& stroke, int brushRadius) {
    std::vector<JMEngine::PointId> selected;
    if (candidates.empty() || viewportWidth <= 0 || viewportHeight <= 0)
        return selected;

    const QRect r = logicalBounds.normalized().intersected(QRect(0, 0, viewportWidth, viewportHeight));
    if (r.isEmpty())
        return selected;
    const int rw = r.width();
    const int rh = r.height();
    std::vector<float> zbuf(static_cast<std::size_t>(rw) * static_cast<std::size_t>(rh), 1.0f);

    // 与正常点渲染一致采用 3px footprint；仅在手势包围盒内建立软件深度，避免整屏 width*height 内存和工作量。
    constexpr float kPointDiameter = 3.0f;
    constexpr float kRadius = kPointDiameter * 0.5f;
    constexpr int kRadiusCeil = 2;
    constexpr float kRadius2 = kRadius * kRadius;
    for (const auto id : candidates) {
        if (static_cast<std::size_t>(id) >= cloud.size()) continue;
        const auto& point = cloud.points()[static_cast<std::size_t>(id)];
        if ((point.flags & JMEngine::PointDeleted) != 0) continue;
        float sx=0.0f, sy=0.0f, depth=1.0f;
        if (!projectPointToScreen(point.position, mvp, viewportWidth, viewportHeight, sx, sy, depth)) continue;
        const int cx=int(std::floor(sx)), cy=int(std::floor(sy));
        for (int dy=-kRadiusCeil; dy<=kRadiusCeil; ++dy) {
            const int py=cy+dy;
            if (py < r.top() || py > r.bottom()) continue;
            for (int dx=-kRadiusCeil; dx<=kRadiusCeil; ++dx) {
                const int px=cx+dx;
                if (px < r.left() || px > r.right()) continue;
                const float fx=(float(px)+0.5f)-sx, fy=(float(py)+0.5f)-sy;
                if (fx*fx+fy*fy > kRadius2) continue;
                auto& dst=zbuf[static_cast<std::size_t>(py-r.top())*static_cast<std::size_t>(rw)+static_cast<std::size_t>(px-r.left())];
                dst=std::min(dst, depth);
            }
        }
    }

    auto visible = [&](float sx,float sy,float depth) {
        const int px=int(std::floor(sx)), py=int(std::floor(sy));
        if (!r.contains(px,py)) return false;
        const int lx=px-r.left(), ly=py-r.top();
        return surfaceDepthVisible(zbuf, rw, rh, lx, ly, depth, 2);
    };
#ifdef JMENGINE_USE_OPENMP
    const int threadCount=JMEngine::processing::processingThreadCount();
    std::vector<std::vector<JMEngine::PointId>> perThread(static_cast<std::size_t>(threadCount));
#pragma omp parallel num_threads(threadCount)
    {
        auto& local=perThread[static_cast<std::size_t>(omp_get_thread_num())];
#pragma omp for schedule(static)
        for (std::int64_t ii=0; ii<static_cast<std::int64_t>(candidates.size()); ++ii) {
            const auto id=candidates[static_cast<std::size_t>(ii)];
            if (static_cast<std::size_t>(id)>=cloud.size()) continue;
            const auto& point=cloud.points()[static_cast<std::size_t>(id)];
            if ((point.flags&JMEngine::PointDeleted)!=0) continue;
            float sx=0.0f,sy=0.0f,depth=1.0f;
            if (!projectPointToScreen(point.position,mvp,viewportWidth,viewportHeight,sx,sy,depth)) continue;
            const QPoint qp(int(std::floor(sx)),int(std::floor(sy)));
            if (gestureContains(mode,qp,press,current,stroke,brushRadius) && visible(sx,sy,depth)) local.push_back(id);
        }
    }
    for (auto& v:perThread) selected.insert(selected.end(),v.begin(),v.end());
#else
    for (const auto id:candidates) {
        if (static_cast<std::size_t>(id)>=cloud.size()) continue;
        const auto& point=cloud.points()[static_cast<std::size_t>(id)];
        if ((point.flags&JMEngine::PointDeleted)!=0) continue;
        float sx=0.0f,sy=0.0f,depth=1.0f;
        if (!projectPointToScreen(point.position,mvp,viewportWidth,viewportHeight,sx,sy,depth)) continue;
        const QPoint qp(int(std::floor(sx)),int(std::floor(sy)));
        if (gestureContains(mode,qp,press,current,stroke,brushRadius) && visible(sx,sy,depth)) selected.push_back(id);
    }
#endif
    return sortedUnique(std::move(selected));
}

std::vector<JMEngine::PointId> selectCandidatePointsThrough(
    const JMEngine::PointCloud& cloud, const std::vector<JMEngine::PointId>& candidates, const JMEngine::Mat4f& mvp,
    int viewportWidth, int viewportHeight, PointCloudWidget::InteractionMode mode, const QPoint& press,
    const QPoint& current, const std::vector<QPoint>& stroke, int brushRadius) {
    std::vector<JMEngine::PointId> selected;
    if (candidates.empty())
        return selected;
#ifdef JMENGINE_USE_OPENMP
    const int threadCount = JMEngine::processing::processingThreadCount();
    std::vector<std::vector<JMEngine::PointId>> perThread(static_cast<std::size_t>(threadCount));
#pragma omp parallel num_threads(threadCount)
    {
        auto& local = perThread[static_cast<std::size_t>(omp_get_thread_num())];
#pragma omp for schedule(static)
        for (std::int64_t ii = 0; ii < static_cast<std::int64_t>(candidates.size()); ++ii) {
            const auto id = candidates[static_cast<std::size_t>(ii)];
            if (static_cast<std::size_t>(id) >= cloud.size())
                continue;
            const auto& point = cloud.points()[static_cast<std::size_t>(id)];
            if ((point.flags & JMEngine::PointDeleted) != 0)
                continue;
            float sx = 0.0f, sy = 0.0f, depth = 1.0f;
            if (!projectPointToScreen(point.position, mvp, viewportWidth, viewportHeight, sx, sy, depth))
                continue;
            if (gestureContains(mode, QPoint(int(std::floor(sx)), int(std::floor(sy))), press, current, stroke,
                                brushRadius))
                local.push_back(id);
        }
    }
    for (auto& v : perThread)
        selected.insert(selected.end(), v.begin(), v.end());
#else
    for (const auto id : candidates) {
        if (static_cast<std::size_t>(id) >= cloud.size())
            continue;
        const auto& point = cloud.points()[static_cast<std::size_t>(id)];
        if ((point.flags & JMEngine::PointDeleted) != 0)
            continue;
        float sx = 0.0f, sy = 0.0f, depth = 1.0f;
        if (!projectPointToScreen(point.position, mvp, viewportWidth, viewportHeight, sx, sy, depth))
            continue;
        if (gestureContains(mode, QPoint(int(std::floor(sx)), int(std::floor(sy))), press, current, stroke,
                            brushRadius))
            selected.push_back(id);
    }
#endif
    return sortedUnique(std::move(selected));
}

} // namespace

PointCloudWidget::Model::Model(QString p, std::shared_ptr<JMEngine::PointCloud> c, JMEngine::ObjMeshData m,
                               bool meshModeFlag)
    : path(std::move(p)), meshMode(meshModeFlag), cloud(std::move(c)), editor(cloud) {
    displayMode = meshMode ? DisplayMode::Solid : DisplayMode::Points;
    if (meshMode) {
        this->mesh = std::make_shared<JMEngine::TriangleMesh>(cloud, std::move(m.triangleIndices));
        meshEditor.setMesh(this->mesh);
    }
    if (cloud)
        selectionMask.assign(cloud->size(), 0u);
}

PointCloudWidget::Model::Model(QString p, std::shared_ptr<JMEngine::TriangleMesh> meshValue)
    : path(std::move(p)), meshMode(true), displayMode(DisplayMode::Solid),
      cloud(meshValue ? meshValue->vertices() : nullptr), editor(cloud), mesh(std::move(meshValue)), meshEditor(mesh) {
    if (cloud)
        selectionMask.assign(cloud->size(), 0u);
    // 算法生成的新 Mesh 默认使用中性灰材质，便于观察表面细节。
    displayColor = QColor(184, 184, 184);
    useDisplayColor = true;
}


void PointCloudWidget::ensurePointPickIndex(Model& model) {
    if (!model.cloud || model.cloud->empty()) {
        model.pickGridIds.clear();
        model.pickBlocks.clear();
        model.pickIndexedCloud = model.cloud.get();
        model.pickIndexedPointCount = model.cloud ? model.cloud->size() : 0u;
        return;
    }
    if (model.pickIndexedCloud == model.cloud.get() && model.pickIndexedPointCount == model.cloud->size() &&
        !model.pickBlocks.empty())
        return;

    const auto& pts = model.cloud->points();
    JMEngine::Vec3f bmin = pts.front().position;
    JMEngine::Vec3f bmax = pts.front().position;
    for (const auto& p : pts) {
        bmin.x = std::min(bmin.x, p.position.x); bmin.y = std::min(bmin.y, p.position.y); bmin.z = std::min(bmin.z, p.position.z);
        bmax.x = std::max(bmax.x, p.position.x); bmax.y = std::max(bmax.y, p.position.y); bmax.z = std::max(bmax.z, p.position.z);
    }

    const float ex = std::max(1.0e-12f, bmax.x - bmin.x);
    const float ey = std::max(1.0e-12f, bmax.y - bmin.y);
    const float ez = std::max(1.0e-12f, bmax.z - bmin.z);

    // 约 32K 点/空间单元。10M 点通常只有数百个 block；索引 O(N) 构建，选择阶段只扫描候选块。
    constexpr std::size_t kTargetPointsPerCell = 32768u;
    const std::size_t targetCells = std::max<std::size_t>(1u, (pts.size() + kTargetPointsPerCell - 1u) / kTargetPointsPerCell);
    int nx = 1, ny = 1, nz = 1;
    while (static_cast<std::size_t>(nx) * ny * nz < targetCells) {
        const float sx = ex / float(nx), sy = ey / float(ny), sz = ez / float(nz);
        if (sx >= sy && sx >= sz && nx < 64) ++nx;
        else if (sy >= sx && sy >= sz && ny < 64) ++ny;
        else if (nz < 64) ++nz;
        else break;
    }

    const std::size_t cellCount = static_cast<std::size_t>(nx) * ny * nz;
    std::vector<std::size_t> counts(cellCount, 0u);
    auto cellOf = [&](const JMEngine::Vec3f& v) {
        const int ix = std::clamp(int((v.x - bmin.x) / ex * float(nx)), 0, nx - 1);
        const int iy = std::clamp(int((v.y - bmin.y) / ey * float(ny)), 0, ny - 1);
        const int iz = std::clamp(int((v.z - bmin.z) / ez * float(nz)), 0, nz - 1);
        return (static_cast<std::size_t>(iz) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(iy)) *
                   static_cast<std::size_t>(nx) + static_cast<std::size_t>(ix);
    };
    for (const auto& p : pts)
        ++counts[cellOf(p.position)];

    std::vector<std::size_t> offsets(cellCount + 1u, 0u);
    for (std::size_t i = 0; i < cellCount; ++i)
        offsets[i + 1u] = offsets[i] + counts[i];
    std::vector<std::size_t> cursors = offsets;
    model.pickGridIds.resize(pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const auto c = cellOf(pts[i].position);
        model.pickGridIds[cursors[c]++] = static_cast<JMEngine::PointId>(i);
    }

    model.pickBlocks.clear();
    model.pickBlocks.reserve(cellCount);
    const float inf = std::numeric_limits<float>::infinity();
    for (std::size_t c = 0; c < cellCount; ++c) {
        if (counts[c] == 0u)
            continue;
        Model::PickBlock block;
        block.offset = offsets[c];
        block.count = counts[c];
        block.min = {inf, inf, inf};
        block.max = {-inf, -inf, -inf};
        for (std::size_t k = block.offset; k < block.offset + block.count; ++k) {
            const auto& v = pts[static_cast<std::size_t>(model.pickGridIds[k])].position;
            block.min.x = std::min(block.min.x, v.x); block.min.y = std::min(block.min.y, v.y); block.min.z = std::min(block.min.z, v.z);
            block.max.x = std::max(block.max.x, v.x); block.max.y = std::max(block.max.y, v.y); block.max.z = std::max(block.max.z, v.z);
        }
        model.pickBlocks.push_back(block);
    }
    model.pickIndexedCloud = model.cloud.get();
    model.pickIndexedPointCount = model.cloud->size();
    qInfo() << "[PickingIndex] points=" << static_cast<qulonglong>(model.cloud->size())
            << "grid=" << nx << ny << nz << "blocks=" << static_cast<qulonglong>(model.pickBlocks.size());
}

std::vector<JMEngine::PointId> PointCloudWidget::pointPickCandidates(Model& model, const JMEngine::Mat4f& mvp,
                                                                     const QRect& logicalBounds) {
    ensurePointPickIndex(model);
    std::vector<JMEngine::PointId> out;
    if (!model.cloud || model.pickBlocks.empty())
        return out;

    const QRectF target = QRectF(logicalBounds.normalized()).adjusted(-3.0, -3.0, 3.0, 3.0);
    auto blockIntersects = [&](const Model::PickBlock& b) {
        float minX = std::numeric_limits<float>::infinity(), minY = minX;
        float maxX = -minX, maxY = -minX;
        bool conservative = false;
        for (int i = 0; i < 8; ++i) {
            const JMEngine::Vec3f v{(i & 1) ? b.max.x : b.min.x, (i & 2) ? b.max.y : b.min.y,
                                    (i & 4) ? b.max.z : b.min.z};
            const float x=v.x,y=v.y,z=v.z;
            const float cx=mvp.m[0]*x+mvp.m[4]*y+mvp.m[8]*z+mvp.m[12];
            const float cy=mvp.m[1]*x+mvp.m[5]*y+mvp.m[9]*z+mvp.m[13];
            const float cw=mvp.m[3]*x+mvp.m[7]*y+mvp.m[11]*z+mvp.m[15];
            if (cw <= 1.0e-8f) { conservative = true; continue; }
            const float sx=(cx/cw*0.5f+0.5f)*float(width());
            const float sy=(1.0f-(cy/cw*0.5f+0.5f))*float(height());
            minX=std::min(minX,sx); maxX=std::max(maxX,sx); minY=std::min(minY,sy); maxY=std::max(maxY,sy);
        }
        if (conservative)
            return true; // AABB 穿越近裁剪面时宁可多选候选块，不能漏选。
        if (!std::isfinite(minX))
            return false;
        return QRectF(QPointF(minX,minY), QPointF(maxX,maxY)).normalized().intersects(target);
    };

    std::size_t reserveCount = 0;
    for (const auto& b : model.pickBlocks)
        if (blockIntersects(b)) reserveCount += b.count;
    out.reserve(reserveCount);
    for (const auto& b : model.pickBlocks) {
        if (!blockIntersects(b)) continue;
        out.insert(out.end(), model.pickGridIds.begin() + static_cast<std::ptrdiff_t>(b.offset),
                   model.pickGridIds.begin() + static_cast<std::ptrdiff_t>(b.offset + b.count));
    }
    return out;
}

const std::vector<std::uint32_t>& PointCloudWidget::meshDrawIndices(const Model& model) const {
    static const std::vector<std::uint32_t> empty;
    if (!model.meshMode || !model.mesh)
        return empty;
    return model.meshFiltered ? model.visibleMeshIndices : model.mesh->indices();
}

PointCloudWidget::PointCloudWidget(QWidget* parent) : QOpenGLWidget(parent) {
    setAttribute(Qt::WA_AcceptTouchEvents, true);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    setMinimumSize(800, 480);
    workerPool_.setMaxThreadCount(2);
    statusText_ = QString::fromUtf8("空场景：打开 OBJ / PLY / TXT / ASC");
}

PointCloudWidget::~PointCloudWidget() {
    workerPool_.waitForDone();
    makeCurrent();
    for (auto& m : models_)
        destroyModelGl(*m);
    destroyPickingFramebuffer();
    doneCurrent();
}

PointCloudWidget::Model* PointCloudWidget::activeModel() {
    if (activeModelIndex_ < 0 || activeModelIndex_ >= static_cast<int>(models_.size()))
        return nullptr;
    return models_[static_cast<std::size_t>(activeModelIndex_)].get();
}
const PointCloudWidget::Model* PointCloudWidget::activeModel() const {
    if (activeModelIndex_ < 0 || activeModelIndex_ >= static_cast<int>(models_.size()))
        return nullptr;
    return models_[static_cast<std::size_t>(activeModelIndex_)].get();
}

int PointCloudWidget::findModel(const QString& path) const {
    const QString abs = QFileInfo(path).absoluteFilePath();
    for (int i = 0; i < static_cast<int>(models_.size()); ++i) {
        if (QFileInfo(models_[static_cast<std::size_t>(i)]->path).absoluteFilePath() == abs)
            return i;
    }
    return -1;
}

QString PointCloudWidget::activeModelPath() const {
    const auto* m = activeModel();
    return m ? m->path : QString{};
}

void PointCloudWidget::loadModelAsync(const QString& path) {
    const QString abs = QFileInfo(path).absoluteFilePath();
    if (abs.isEmpty())
        return;
    const int existing = findModel(abs);
    if (existing >= 0) {
        activeModelIndex_ = existing;
        // Switching models must preserve the operator camera. F remains the explicit fit command.
        update();
        return;
    }
    if (std::find(loadingPaths_.begin(), loadingPaths_.end(), abs) != loadingPaths_.end())
        return;
    loadingPaths_.push_back(abs);

    QPointer<PointCloudWidget> self(this);
    workerPool_.start([self, abs] {
        if (!self)
            return;
        const std::string file = abs.toStdString();
        const std::string ext = std::filesystem::path(file).extension().string();
        std::shared_ptr<JMEngine::PointCloud> cloud;
        JMEngine::ObjMeshData mesh;
        bool meshMode = false;
        QString message;

        if (ext == ".obj" || ext == ".OBJ") {
            JMEngine::ObjModelData obj;
            std::string msg;
            if (JMEngine::ObjModelLoader::load(file, obj, &msg)) {
                cloud = obj.cloud;
                mesh = std::move(obj.mesh);
                meshMode = !mesh.empty();
                if (cloud)
                    bakeTexture(obj.appearance, *cloud);
                message = QString::fromUtf8(msg.c_str());
            }
        } else {
            std::string msg;
            const std::filesystem::path fp(file);
            const auto ext2 = fp.extension().string();
            if (ext2 == ".txt" || ext2 == ".TXT")
                cloud = JMEngine::ModelIO::loadTxt(file, &msg);
            else if (ext2 == ".asc" || ext2 == ".ASC")
                cloud = JMEngine::ModelIO::loadAsc(file, &msg);
            else
                cloud = JMEngine::PointCloudIO::load(file, &msg);
            message = QString::fromUtf8(msg.c_str());
        }

        // OBJ 初始加载只保留原始 Mesh 索引一份。GPU Picking 与正常渲染
        // 直接复用同一个 VAO/VBO/EBO，不再复制一份 visibleMeshIndices。
        QMetaObject::invokeMethod(
            self,
            [self, abs, cloud, mesh = std::move(mesh), meshMode, message]() mutable {
                if (!self)
                    return;
                auto& lp = self->loadingPaths_;
                lp.erase(std::remove(lp.begin(), lp.end(), abs), lp.end());
                if (!cloud) {
                    self->statusText_ = QString::fromUtf8("加载失败：") + message;
                    self->update();
                    return;
                }

                const bool firstModel = self->models_.empty();
                auto model = std::make_unique<Model>(abs, cloud, std::move(mesh), meshMode);
                self->models_.push_back(std::move(model));
                self->activeModelIndex_ = static_cast<int>(self->models_.size()) - 1;
                self->statusText_ = QString::fromUtf8("已加载：") + QFileInfo(abs).fileName();
                if (firstModel) self->fitView();
                else self->update();
            },
            Qt::QueuedConnection);
    });
}

QString PointCloudWidget::beginScanPreview(std::size_t reservePoints) {
    if (!scanPreviewPath_.isEmpty())
        clearScanPreview();

    // 使用稳定的绝对路径作为模型 key；它只是场景标识，不会实际写文件。
    scanPreviewPath_ = QDir(QDir::tempPath()).absoluteFilePath(QStringLiteral("JMEngine_live_scan.ply"));
    auto cloud = std::make_shared<JMEngine::PointCloud>();
    cloud->points().reserve(reservePoints);
    JMEngine::ObjMeshData emptyMesh;
    auto model = std::make_unique<Model>(scanPreviewPath_, cloud, std::move(emptyMesh), false);
    model->displayMode = DisplayMode::Points;
    model->gpuReservedPointCapacity = reservePoints;
    model->selectionMask.reserve(reservePoints);
    models_.push_back(std::move(model));
    activeModelIndex_ = static_cast<int>(models_.size()) - 1;
    statusText_ = QString::fromUtf8("实时扫描预览");
    scanPreviewViewInitialized_ = false;
    scanPreviewFrameCount_ = 0;
    scanPreviewPointLimit_ = reservePoints;
    if (modelAddedCallback_)
        modelAddedCallback_(scanPreviewPath_);
    update();
    return scanPreviewPath_;
}

void PointCloudWidget::appendScanPreview(const std::shared_ptr<std::vector<JMEngine::Point>>& points,
                                         std::size_t pointLimit) {
    if (!points || points->empty())
        return;
    if (scanPreviewPath_.isEmpty())
        beginScanPreview(pointLimit);
    const int index = findModel(scanPreviewPath_);
    if (index < 0)
        return;
    auto& model = *models_[static_cast<std::size_t>(index)];
    if (!model.cloud)
        return;

    auto& dst = model.cloud->points();

    // Never stop live visualization just because the preview budget is full.
    // The old implementation returned here permanently, which meant only the first part of
    // a long scan was ever visible.  Keep a spatially/temporally representative history by
    // uniformly compacting the accumulated preview when we need room for newer frames.
    // Full-resolution scan data remains owned by RGBDFusion; this only affects the UI preview.
    bool compacted = false;
    if (pointLimit > 0 && dst.size() + points->size() > pointLimit) {
        const std::size_t targetHistory = std::max<std::size_t>(1, pointLimit / 2u);
        if (dst.size() > targetHistory) {
            const std::size_t stride = std::max<std::size_t>(2u, (dst.size() + targetHistory - 1u) / targetHistory);
            std::size_t write = 0;
            for (std::size_t read = 0; read < dst.size(); read += stride)
                dst[write++] = dst[read];
            dst.resize(write);
            compacted = true;
        }
    }

    const std::size_t oldSize = dst.size();
    const std::size_t room = pointLimit > dst.size() ? pointLimit - dst.size() : 0u;
    const std::size_t count = std::min(points->size(), room);
    if (count == 0)
        return;
    dst.insert(dst.end(), points->begin(), points->begin() + static_cast<std::ptrdiff_t>(count));
    model.selectionMask.resize(dst.size(), 0u);
    if (model.liveBackCloud) {
        model.livePostSwapPoints.insert(model.livePostSwapPoints.end(), points->begin(),
                                        points->begin() + static_cast<std::ptrdiff_t>(count));
    }

    if (compacted) {
        // 历史压缩改变了全部索引，不能覆盖正在显示的 front。复制一份 CPU 快照交给 back VBO，
        // front 保持完整直到 back 上传完成后 swap。
        model.liveBackCloud = std::make_shared<JMEngine::PointCloud>(model.cloud->points());
        model.liveBackUploadCursor = 0;
    }

    // 实时扫描相机策略：
    // 1) 第一批有效点必须 fit，否则 OrbitCamera 默认 sceneRadius=1/distance=3，
    //    对毫米坐标点云很容易被 near/far 全部裁掉，表现为黑屏。
    // 2) 后续跟随“最新一帧”的包围盒中心，而不是每帧 fit 整个累计模型。
    //    这样视口会跟着扫描镜头所在区域移动，同时保持用户当前观察方向和缩放，
    //    每帧只 O(previewPointsPerFrame)，不会随累计点数增长而拖慢 UI。
    if (count > 0) {
        ++scanPreviewFrameCount_;
        if (!scanPreviewViewInitialized_) {
            activeModelIndex_ = index;
            // First valid preview chunk must establish the real scene scale first.
            // Pose callbacks can arrive before point-cloud callbacks; following a pose while
            // sceneRadius is still the OrbitCamera default (1.0) can place the observer with
            // an invalid near/far range and make the realtime cloud appear completely blank.
            camera_.fit(*model.cloud);
            scanPreviewViewInitialized_ = true;

            // If a valid SLAM pose arrived before the first cloud, re-apply it now that the
            // point-cloud scale / clipping range is known.
            if (scanCameraPose_ && scanCameraPose_->trackingOk) {
                const ScanCameraViewPose pendingPose = *scanCameraPose_;
                updateScanCameraPose(pendingPose);
            }
        } else if (!scanCameraPose_) {
            JMEngine::Vec3f bmin = dst[oldSize].position;
            JMEngine::Vec3f bmax = dst[oldSize].position;
            for (std::size_t i = oldSize + 1; i < oldSize + count; ++i) {
                const auto& v = dst[i].position;
                bmin.x = std::min(bmin.x, v.x); bmin.y = std::min(bmin.y, v.y); bmin.z = std::min(bmin.z, v.z);
                bmax.x = std::max(bmax.x, v.x); bmax.y = std::max(bmax.y, v.y); bmax.z = std::max(bmax.z, v.z);
            }
            const JMEngine::Vec3f frameCenter{
                (bmin.x + bmax.x) * 0.5f,
                (bmin.y + bmax.y) * 0.5f,
                (bmin.z + bmax.z) * 0.5f
            };

            // 平滑跟随，避免 SLAM pose 微抖导致画面跳动。
            constexpr float followAlpha = 0.22f;
            camera_.target.x += (frameCenter.x - camera_.target.x) * followAlpha;
            camera_.target.y += (frameCenter.y - camera_.target.y) * followAlpha;
            camera_.target.z += (frameCenter.z - camera_.target.z) * followAlpha;

            // 最新帧尺寸偶尔增大时扩大裁剪范围；不主动缩小，防止点云闪烁/裁切。
            const float dx = bmax.x - bmin.x;
            const float dy = bmax.y - bmin.y;
            const float dz = bmax.z - bmin.z;
            const float frameRadius = 0.5f * std::sqrt(dx * dx + dy * dy + dz * dz);
            if (frameRadius > camera_.sceneRadius)
                camera_.sceneRadius = frameRadius;
        }
    }
    model.pickBlocks.clear();
    model.pickGridIds.clear();
    model.pickIndexedCloud = nullptr;
    model.pickIndexedPointCount = 0;

    // 正常情况下 beginScanPreview 已一次性预留足够 GPU 空间。若调用方提高 pointLimit，
    // 仅在真正越界时重建一次，之后仍按 paintGL 的固定 byte budget 分块上传。
    if (model.glCreated && dst.size() > model.gpuReservedPointCapacity) {
        // 容量不足也不销毁正在显示的 front，交给 back VBO 完整建立后切换。
        model.gpuReservedPointCapacity = std::max(dst.size(), model.gpuReservedPointCapacity * 2u);
        model.liveBackCloud = std::make_shared<JMEngine::PointCloud>(model.cloud->points());
        model.liveBackUploadCursor = 0;
    } else if (model.glCreated && !compacted && count > 0) {
        // 正常 10FPS 实时扫描：新增帧点数很小，整帧一次性追加到 front VBO，不分块。
        makeCurrent();
        uploadPointRangeNow(model, oldSize, count);
        doneCurrent();
    }
    update();
}

void PointCloudWidget::appendScanLocalFrame(int frameId,
                                             const std::shared_ptr<std::vector<JMEngine::Point>>& localPoints,
                                             const std::array<float,16>& poseArray) {
    if (!localPoints || localPoints->empty()) return;
    if (scanPreviewPath_.isEmpty()) beginScanPreview(scanPreviewPointLimit_);
    const int index = findModel(scanPreviewPath_);
    if (index < 0) return;
    auto& model = *models_[static_cast<std::size_t>(index)];
    if (!model.cloud) return;

    JMEngine::Mat4f pose;
    pose.m = poseArray;
    auto existing = model.liveFrameIndex.find(frameId);
    if (existing != model.liveFrameIndex.end()) {
        model.liveFrames[existing->second].pose = pose;
        update();
        return;
    }

    auto& dst = model.cloud->points();
    const std::size_t first = dst.size();
    const std::size_t count = localPoints->size();
    // The source already budgets points across maxFrames. If a caller exceeds the original
    // reserve, grow once rather than dropping the tail of the scan.
    if (first + count > model.gpuReservedPointCapacity)
        model.gpuReservedPointCapacity = std::max(first + count, std::max<std::size_t>(1, model.gpuReservedPointCapacity) * 2u);
    dst.insert(dst.end(), localPoints->begin(), localPoints->end());
    model.selectionMask.resize(dst.size(), 0u);
    model.liveFramePoseMode = true;
    model.liveFrameIndex[frameId] = model.liveFrames.size();
    model.liveFrames.push_back({frameId, first, count, pose});

    // Establish clipping/scene scale from the first frame in WORLD coordinates. The stored
    // history remains local, so optimization never requires point re-upload.
    if (!scanPreviewViewInitialized_) {
        JMEngine::PointCloud::Container world;
        world.reserve(localPoints->size());
        for (const auto& src : *localPoints) {
            auto q = src;
            q.position = JMEngine::transformPoint(pose, src.position);
            world.push_back(q);
        }
        if (!world.empty()) {
            JMEngine::PointCloud tmp(std::move(world));
            camera_.fit(tmp);
            scanPreviewViewInitialized_ = true;
            activeModelIndex_ = index;
            if (scanCameraPose_ && scanCameraPose_->trackingOk) {
                const ScanCameraViewPose pending = *scanCameraPose_;
                updateScanCameraPose(pending);
            }
        }
    }

    if (model.glCreated && first + count <= model.gpuReservedPointCapacity) {
        makeCurrent();
        uploadPointRangeNow(model, first, count);
        doneCurrent();
    } else if (model.glCreated) {
        // Capacity changed after GL creation. This is rare because beginScanPreview reserves the
        // full budget. Recreate only once on true overflow; normal optimization never enters here.
        makeCurrent();
        destroyModelGl(model);
        doneCurrent();
        model.uploadPointCursor = 0;
        model.drawPointCount = 0;
    }
    model.pickBlocks.clear();
    model.pickGridIds.clear();
    model.pickIndexedCloud = nullptr;
    model.pickIndexedPointCount = 0;
    update();
}

void PointCloudWidget::updateScanFramePoses(const std::shared_ptr<std::vector<LiveFramePoseUpdate>>& updates) {
    if (!updates || updates->empty() || scanPreviewPath_.isEmpty()) return;
    const int index = findModel(scanPreviewPath_);
    if (index < 0) return;
    auto& model = *models_[static_cast<std::size_t>(index)];
    if (!model.liveFramePoseMode) return;
    for (const auto& u : *updates) {
        const auto it = model.liveFrameIndex.find(u.frameId);
        if (it == model.liveFrameIndex.end()) continue;
        model.liveFrames[it->second].pose.m = u.pose;
    }
    // Only CPU pose data changed. No point VBO upload, no buffer rebuild, no flicker.
    update();
}

void PointCloudWidget::upsertScanStatusLayer(
    QString& path,
    const QString& fileName,
    const std::shared_ptr<std::vector<JMEngine::Point>>& points,
    std::uint32_t rgba) {
    if (!points || points->empty()) {
        clearScanStatusLayer(path);
        return;
    }
    if (path.isEmpty())
        path = QDir(QDir::tempPath()).absoluteFilePath(fileName);

    JMEngine::PointCloud::Container colored;
    colored.reserve(points->size());
    for (const auto& src : *points) {
        auto p = src;
        p.rgba = rgba;
        colored.push_back(p);
    }
    auto cloud = std::make_shared<JMEngine::PointCloud>(std::move(colored));

    int index = findModel(path);
    if (index < 0) {
        JMEngine::ObjMeshData emptyMesh;
        auto model = std::make_unique<Model>(path, cloud, std::move(emptyMesh), false);
        model->displayMode = DisplayMode::Points;
        model->gpuReservedPointCapacity = cloud->size();
        model->selectionMask.assign(cloud->size(), 0u);
        models_.push_back(std::move(model));
        return;
    }

    auto& model = *models_[static_cast<std::size_t>(index)];
    bool canDirectUpload = model.glCreated && cloud->size() <= model.gpuReservedPointCapacity;
    if (model.glCreated && cloud->size() > model.gpuReservedPointCapacity) {
        // 当前/找回帧通常只有几千点；尺寸真的变大时一次性扩容即可，不走历史分块逻辑。
        makeCurrent();
        destroyModelGl(model);
        doneCurrent();
        model.gpuReservedPointCapacity = cloud->size();
        canDirectUpload = false;
    }
    model.cloud = cloud;
    model.editor.setPointCloud(cloud);
    model.mesh.reset();
    model.meshMode = false;
    model.displayMode = DisplayMode::Points;
    model.selectionMask.assign(cloud->size(), 0u);
    model.selectedIds.clear();
    model.selectedTriangleIds.clear();
    model.uploadPointCursor = canDirectUpload ? cloud->size() : 0;
    model.uploadIndexCursor = 0;
    model.drawPointCount = canDirectUpload ? static_cast<GLsizei>(cloud->size()) : 0;
    model.gpuReservedPointCapacity = std::max(model.gpuReservedPointCapacity, cloud->size());
    if (canDirectUpload) {
        makeCurrent();
        uploadPointRangeNow(model, 0, cloud->size());
        doneCurrent();
    }
    model.pickBlocks.clear();
    model.pickGridIds.clear();
    model.pickIndexedCloud = nullptr;
    model.pickIndexedPointCount = 0;
}

void PointCloudWidget::clearScanStatusLayer(QString& path) {
    if (path.isEmpty()) return;
    const int index = findModel(path);
    if (index >= 0) {
        makeCurrent();
        destroyModelGl(*models_[static_cast<std::size_t>(index)]);
        doneCurrent();
        models_.erase(models_.begin() + index);
        if (activeModelIndex_ > index) --activeModelIndex_;
        if (models_.empty()) activeModelIndex_ = -1;
        else if (activeModelIndex_ >= int(models_.size())) activeModelIndex_ = int(models_.size()) - 1;
    }
    path.clear();
}

void PointCloudWidget::setCurrentScanFrame(const std::shared_ptr<std::vector<JMEngine::Point>>& points,
                                           bool trackingOk) {
    // Persistent RGB history is now uploaded independently in frame-local coordinates.
    // These two layers are status-only overlays and never mutate/re-upload history.
    constexpr std::uint32_t kGreen = 0xff00ff00u;
    constexpr std::uint32_t kYellow = 0xff00ffffu;

    if (trackingOk) {
        recoveryScanFrameSource_.reset();
        clearScanStatusLayer(recoveryScanFramePath_);
        currentScanFrameSource_ = points;
        currentScanFrameTrackingOk_ = true;
        if (points && !points->empty())
            upsertScanStatusLayer(currentScanFramePath_, QStringLiteral("JMEngine_current_scan_frame.ply"), points, kGreen);
        else
            clearScanStatusLayer(currentScanFramePath_);
        update();
        return;
    }

    if (!recoveryScanFrameSource_ && currentScanFrameSource_ && currentScanFrameTrackingOk_) {
        recoveryScanFrameSource_ = currentScanFrameSource_;
        upsertScanStatusLayer(recoveryScanFramePath_, QStringLiteral("JMEngine_recovery_reference_frame.ply"),
                              recoveryScanFrameSource_, kYellow);
    }
    currentScanFrameSource_ = points;
    currentScanFrameTrackingOk_ = false;
    if (points && !points->empty())
        upsertScanStatusLayer(currentScanFramePath_, QStringLiteral("JMEngine_current_scan_frame.ply"), points, kGreen);
    else
        clearScanStatusLayer(currentScanFramePath_);
    update();
}

void PointCloudWidget::finalizeCurrentScanFrame() {
    currentScanFrameSource_.reset();
    currentScanFrameTrackingOk_ = false;
    recoveryScanFrameSource_.reset();
    clearScanStatusLayer(currentScanFramePath_);
    clearScanStatusLayer(recoveryScanFramePath_);
    update();
}

void PointCloudWidget::clearCurrentScanFrame() {
    currentScanFrameSource_.reset();
    currentScanFrameTrackingOk_ = false;
    recoveryScanFrameSource_.reset();
    clearScanStatusLayer(currentScanFramePath_);
    clearScanStatusLayer(recoveryScanFramePath_);
    update();
}

void PointCloudWidget::updateScanCameraPose(const ScanCameraViewPose& pose) {
    scanCameraPose_ = pose;

    // SLAM pose can be reported before the first realtime point chunk.  Keep the pose for
    // drawing/status, but do not move the observer until appendScanPreview() has fitted the
    // first cloud and established a meaningful sceneRadius / clipping scale.
    if (!scanPreviewViewInitialized_) {
        if (pose.trackingOk)
            lastValidScanCameraPose_ = pose;
        update();
        return;
    }

    if (pose.trackingOk) {
        lastValidScanCameraPose_ = pose;

        // Third-person scan view: follow the physical color-camera pose, with the viewer
        // slightly behind it and looking in exactly the same direction.  This avoids the
        // old behavior of following the latest point-cloud centroid, which can lag or jump.
        if (scanCameraFollowEnabled_) {
            const auto forward = JMEngine::example::normalize(pose.forward);
            const auto right = JMEngine::example::normalize(pose.right);
            const auto up = JMEngine::example::normalize(pose.up);
            const float base = std::max(camera_.sceneRadius, 1.0f);
            const float behind = std::max(base * 0.10f, 1.0f);
            const float lookAhead = std::max(base * 0.24f, behind * 2.0f);
            camera_.target = JMEngine::example::add(pose.position, JMEngine::example::mul(forward, lookAhead));
            camera_.distance = behind + lookAhead;
            camera_.orientation = quatFromBasis(right, up, JMEngine::example::mul(forward, -1.0f));
            camera_.panNdcX = 0.0f;
            camera_.panNdcY = 0.0f;
            camera_.sceneRadius = std::max(camera_.sceneRadius, camera_.distance * 1.5f);
        }
    }
    // Tracking lost: keep the last valid observer pose so the 3D view does not jump.
    // The current SLAM camera/frustum is still drawn in yellow to guide reacquisition.
    update();
}

void PointCloudWidget::clearScanCameraPose() {
    scanCameraPose_.reset();
    lastValidScanCameraPose_.reset();
    scanCameraFollowEnabled_ = true;
    update();
}


void PointCloudWidget::updateOptimizedScanPreview(const std::shared_ptr<JMEngine::PointCloud>&) {
    // Deprecated: live optimization now updates only per-frame RT through updateScanFramePoses().
    // Keeping this method avoids breaking older callers while guaranteeing no second history VBO.
}

void PointCloudWidget::replaceScanPreview(const std::shared_ptr<JMEngine::PointCloud>& cloud) {
    if (!cloud)
        return;
    if (scanPreviewPath_.isEmpty())
        beginScanPreview(cloud->size());
    const int index = findModel(scanPreviewPath_);
    if (index < 0)
        return;
    auto& model = *models_[static_cast<std::size_t>(index)];
    if (model.glCreated) {
        makeCurrent();
        destroyModelGl(model);
        doneCurrent();
    }
    model.liveFramePoseMode = false;
    model.liveFrames.clear();
    model.liveFrameIndex.clear();
    model.cloud = cloud;
    model.editor.setPointCloud(cloud);
    model.mesh.reset();
    model.meshMode = false;
    model.displayMode = DisplayMode::Points;
    model.selectionMask.assign(cloud->size(), 0u);
    model.selectedIds.clear();
    model.selectedTriangleIds.clear();
    model.uploadPointCursor = 0;
    model.uploadIndexCursor = 0;
    model.drawPointCount = 0;
    model.gpuReservedPointCapacity = cloud->size();
    model.pickBlocks.clear();
    model.pickGridIds.clear();
    model.pickIndexedCloud = nullptr;
    model.pickIndexedPointCount = 0;
    activeModelIndex_ = index;
    statusText_ = QString::fromUtf8("离线重建结果：%1 点").arg(qulonglong(cloud->size()));
    // Keep the operator's current camera after offline reconstruction.
    update();
}

void PointCloudWidget::clearScanPreview() {
    clearScanCameraPose();
    clearCurrentScanFrame();
    if (scanPreviewPath_.isEmpty())
        return;
    const QString path = scanPreviewPath_;
    scanPreviewPath_.clear();
    scanPreviewViewInitialized_ = false;
    scanPreviewFrameCount_ = 0;
    scanPreviewPointLimit_ = 2000000;
    removeModel(path);
    update();
}

bool PointCloudWidget::activateModel(const QString& path) {
    const int i = findModel(path);
    if (i < 0)
        return false;
    activeModelIndex_ = i;
    basePlane_.active = false;
    basePlane_.dragging = false;
    clearSelection();
    // Preserve camera orientation/distance, but make the selected model the orbit pivot.
    updateOrbitPivotForActiveModel();
    update();
    return true;
}

void PointCloudWidget::setModelVisible(const QString& path, bool visible) {
    const int i = findModel(path);
    if (i < 0)
        return;
    models_[static_cast<std::size_t>(i)]->visible = visible;
    update();
}

void PointCloudWidget::removeModel(const QString& path) {
    const int i = findModel(path);
    if (i < 0)
        return;
    makeCurrent();
    destroyModelGl(*models_[static_cast<std::size_t>(i)]);
    doneCurrent();
    models_.erase(models_.begin() + i);
    if (models_.empty())
        activeModelIndex_ = -1;
    else
        activeModelIndex_ = std::clamp(i, 0, static_cast<int>(models_.size()) - 1);
    update();
}

void PointCloudWidget::setInteractionMode(InteractionMode mode) {
    interactionMode_ = mode;
    cancelEditGesture();
    update();
}
void PointCloudWidget::setSelectionDepthMode(SelectionDepthMode mode) {
    // Surface / Through are two independent user-selectable modes.
    // Surface must always run depth visibility filtering; Through deliberately skips depth filtering.
    selectionDepthMode_ = mode;
    const QString depth =
        mode == SelectionDepthMode::Surface ? QString::fromUtf8("Surface") : QString::fromUtf8("Through");
    const QString backend = (pickingMode_ == PickingMode::Gpu && mode == SelectionDepthMode::Surface && pickingReady_)
                                ? QString::fromUtf8("GPU/Block")
                                : QString::fromUtf8("CPU/Block");
    statusText_ = QString::fromUtf8("Picking：%1 / %2").arg(backend, depth);
    update();
}
void PointCloudWidget::setPickingMode(PickingMode mode) {
    pickingMode_ = mode;
    const QString depth = selectionDepthMode_ == SelectionDepthMode::Surface ? QString::fromUtf8("Surface")
                                                                             : QString::fromUtf8("Through");
    const QString backend =
        (mode == PickingMode::Gpu && selectionDepthMode_ == SelectionDepthMode::Surface && pickingReady_)
            ? QString::fromUtf8("GPU/Block")
            : QString::fromUtf8("CPU/Block");
    statusText_ = QString::fromUtf8("Picking：%1 / %2").arg(backend, depth);
    update();
}

void PointCloudWidget::setDisplayMode(DisplayMode mode) {
    auto* m = activeModel();
    if (!m)
        return;
    if (!m->meshMode && mode != DisplayMode::Points)
        mode = DisplayMode::Points;

    // 点显示与网格显示的编辑单位不同：
    //   Points -> PointId
    //   Solid/Wireframe -> TriangleId
    // 跨语义切换时必须清掉旧选择，避免残留 TriangleId/PointId 让 Delete 走错分支。
    const bool oldPointMode = !m->meshMode || m->displayMode == DisplayMode::Points;
    const bool newPointMode = !m->meshMode || mode == DisplayMode::Points;
    if (oldPointMode != newPointMode)
        clearSelection();

    m->displayMode = mode;
    if (mode == DisplayMode::Wireframe || mode == DisplayMode::SolidWireframe)
        m->wireDirty = true;
    update();
}

void PointCloudWidget::setModelDisplayColor(const QString& path, const QColor& color) {
    const int i = findModel(path);
    if (i < 0 || !color.isValid())
        return;
    auto& m = *models_[static_cast<std::size_t>(i)];
    m.displayColor = color;
    m.useDisplayColor = true;
    if (m.glCreated && m.cloud) {
        const std::uint32_t rgba = packedColor(color);
        constexpr std::size_t chunk = 262144;
        std::vector<std::uint32_t> colors;
        makeCurrent();
        glBindBuffer(GL_ARRAY_BUFFER, m.gpu.colorVbo);
        for (std::size_t off = 0; off < m.cloud->size(); off += chunk) {
            const std::size_t count = std::min(chunk, m.cloud->size() - off);
            colors.assign(count, rgba);
            glBufferSubData(GL_ARRAY_BUFFER, static_cast<GLintptr>(off * 4u), static_cast<GLsizeiptr>(count * 4u),
                            colors.data());
        }
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        doneCurrent();
    }
    update();
}

QColor PointCloudWidget::modelDisplayColor(const QString& path) const {
    const int i = findModel(path);
    return i >= 0 ? models_[static_cast<std::size_t>(i)]->displayColor : QColor{};
}

void PointCloudWidget::setTouchEditMode(bool enabled) {
    touchEditMode_ = enabled;
    cancelEditGesture();
    update();
}

void PointCloudWidget::setObjectMoveMode(bool enabled) {
    objectMoveMode_ = enabled;
    if (!enabled && objectDragging_) {
        objectDragging_ = false;
        unsetCursor();
    }
    update();
}

void PointCloudWidget::initializeGL() {
    initializeOpenGLFunctions();
    backend_ = createRenderBackend();

    QString backendError;
    renderReady_ = backend_ && backend_->validateContext(&backendError);

    const auto* ctx = QOpenGLContext::currentContext();
    const auto fmt = ctx ? ctx->format() : QSurfaceFormat{};
    const char* vendor = reinterpret_cast<const char*>(glGetString(GL_VENDOR));
    const char* renderer = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    const char* version = reinterpret_cast<const char*>(glGetString(GL_VERSION));

    qInfo() << "[JMEngine] backend=" << (backend_ ? backend_->name() : QStringLiteral("null"))
            << "GLES=" << (ctx && ctx->isOpenGLES()) << "format=" << fmt.majorVersion() << fmt.minorVersion()
            << "vendor=" << (vendor ? vendor : "?") << "renderer=" << (renderer ? renderer : "?")
            << "version=" << (version ? version : "?");

    if (!renderReady_) {
        statusText_ = QString::fromUtf8("渲染后端初始化失败：") + backendError;
        qWarning() << statusText_;
        return;
    }

    backend_->configureContextState(*this);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glDisable(GL_DITHER);
    glClearColor(0.02f, 0.02f, 0.025f, 1.0f);

    if (!createPrograms()) {
        renderReady_ = false;
        statusText_ = QString::fromUtf8("Shader 编译失败，请查看控制台日志");
        qWarning() << pointProgram_.log() << meshProgram_.log() << pointPickProgram_.log() << meshPickProgram_.log();
        return;
    }

    pickingReady_ = backend_->gpuPickingSupported() && pointPickProgram_.isLinked() && meshPickProgram_.isLinked() &&
                    createPickingFramebuffer(std::max(1, int(width() * devicePixelRatioF())),
                                             std::max(1, int(height() * devicePixelRatioF())));

    statusText_ = QString::fromUtf8("%1 | %2 | Mesh Picking=%3 | Point Picking=%4")
                      .arg(backend_->name())
                      .arg(QString::fromLatin1(renderer ? renderer : "unknown"))
                      .arg(pickingReady_ ? QString::fromUtf8("R32UI GPU") : QString::fromUtf8("CPU"))
                      .arg(pickingReady_ ? QString::fromUtf8("GPU Depth + Block Exact") : QString::fromUtf8("CPU"));
}

void PointCloudWidget::resizeGL(int w, int h) {
    if (!renderReady_)
        return;
    const qreal dpr = devicePixelRatioF();
    pickingReady_ = backend_ && backend_->gpuPickingSupported() && pointPickProgram_.isLinked() &&
                    meshPickProgram_.isLinked() &&
                    createPickingFramebuffer(std::max(1, int(w * dpr)), std::max(1, int(h * dpr)));
}

bool PointCloudWidget::createPrograms() {
    if (!backend_)
        return false;

    auto build = [](QOpenGLShaderProgram& p, const char* vs, const char* fs, const auto& bindLocations) {
        p.removeAllShaders();
        if (!p.addShaderFromSourceCode(QOpenGLShader::Vertex, vs))
            return false;
        if (!p.addShaderFromSourceCode(QOpenGLShader::Fragment, fs))
            return false;
        bindLocations(p);
        return p.link();
    };

    const bool ok1 = build(pointProgram_, backend_->renderVertexShader(), backend_->renderFragmentShader(),
                           [this](auto& p) { backend_->bindRenderAttributeLocations(p); });
    const bool ok2 = build(meshProgram_, backend_->renderVertexShader(), backend_->renderFragmentShader(),
                           [this](auto& p) { backend_->bindRenderAttributeLocations(p); });
    if (!ok1 || !ok2)
        return false;

    // GPU Picking 是可选能力。Desktop 只使用 OpenGL 3.2+ / GLSL 150 / R32UI 现代路径。
    // Context 或 shader/FBO 不满足时不影响正常渲染，选择自动回退 CPU。
    if (!backend_->gpuPickingSupported())
        return true;

    const bool pointOk =
        build(pointPickProgram_, backend_->pointPickVertexShader(), backend_->pointPickFragmentShader(),
              [this](auto& p) { backend_->bindPointPickAttributeLocations(p); });
    if (!pointOk)
        return true;

    meshPickProgram_.removeAllShaders();
    meshPickProgram_.create();
    if (!meshPickProgram_.addShaderFromSourceCode(QOpenGLShader::Vertex, backend_->meshPickVertexShader()))
        return true;
    const char* gs = backend_->meshPickGeometryShader();
    if (!gs || !meshPickProgram_.addShaderFromSourceCode(QOpenGLShader::Geometry, gs))
        return true;
    if (!meshPickProgram_.addShaderFromSourceCode(QOpenGLShader::Fragment, backend_->meshPickFragmentShader()))
        return true;
    backend_->bindMeshPickAttributeLocations(meshPickProgram_);

    // 现代 Desktop 路径使用 GLSL 150 core geometry shader，primitive layout 已写在 shader 内。
    // 编译/链接失败不影响正常渲染，initializeGL 会把 Picking 自动降级到 CPU。
    meshPickProgram_.link();
    return true;
}

bool PointCloudWidget::createPickingFramebuffer(int w, int h) {
    if (!backend_ || !backend_->gpuPickingSupported())
        return false;
    if (pickFbo_ && pickWidth_ == w && pickHeight_ == h)
        return true;
    destroyPickingFramebuffer();

    glGenFramebuffers(1, &pickFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, pickFbo_);

    glGenTextures(1, &pickTexture_);
    glBindTexture(GL_TEXTURE_2D, pickTexture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32UI, w, h, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, nullptr);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, pickTexture_, 0);

    // R32UI Surface Picking 使用同一 FBO 的深度附件，只保留当前视角最前表面。
    glGenTextures(1, &pickDepth_);
    glBindTexture(GL_TEXTURE_2D, pickDepth_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, w, h, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, pickDepth_, 0);

    const bool ok = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());
    if (!ok)
        destroyPickingFramebuffer();
    else {
        pickWidth_ = w;
        pickHeight_ = h;
    }
    return ok;
}

void PointCloudWidget::destroyPickingFramebuffer() {
    if (pickDepth_)
        glDeleteTextures(1, &pickDepth_);
    if (pickTexture_)
        glDeleteTextures(1, &pickTexture_);
    if (pickFbo_)
        glDeleteFramebuffers(1, &pickFbo_);
    pickFbo_ = pickTexture_ = pickDepth_ = 0;
    pickWidth_ = pickHeight_ = 0;
}

void PointCloudWidget::createModelGl(Model& m) {
    if (m.glCreated || !m.cloud || !backend_)
        return;

    const GLsizeiptr n = static_cast<GLsizeiptr>(std::max(m.cloud->size(), m.gpuReservedPointCapacity));
    glGenBuffers(1, &m.gpu.positionVbo);
    glBindBuffer(GL_ARRAY_BUFFER, m.gpu.positionVbo);
    glBufferData(GL_ARRAY_BUFFER, n * sizeof(JMEngine::Vec3f), nullptr, GL_DYNAMIC_DRAW);

    glGenBuffers(1, &m.gpu.colorVbo);
    glBindBuffer(GL_ARRAY_BUFFER, m.gpu.colorVbo);
    glBufferData(GL_ARRAY_BUFFER, n * 4, nullptr, GL_DYNAMIC_DRAW);

    glGenBuffers(1, &m.gpu.normalVbo);
    glBindBuffer(GL_ARRAY_BUFFER, m.gpu.normalVbo);
    glBufferData(GL_ARRAY_BUFFER, n * sizeof(JMEngine::Vec3f), nullptr, GL_DYNAMIC_DRAW);

#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    if (m.textureResult && m.textureResult->texcoords.size() == m.cloud->size() && !m.textureResult->texcoords.empty()) {
        glGenBuffers(1, &m.gpu.texcoordVbo);
        glBindBuffer(GL_ARRAY_BUFFER, m.gpu.texcoordVbo);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(m.textureResult->texcoords.size() * sizeof(JMEngine::Vec2f)),
                     m.textureResult->texcoords.data(), GL_STATIC_DRAW);
    }
#endif

    glGenBuffers(1, &m.gpu.flagsVbo);
    glBindBuffer(GL_ARRAY_BUFFER, m.gpu.flagsVbo);
    glBufferData(GL_ARRAY_BUFFER, n, nullptr, GL_DYNAMIC_DRAW);

    glGenBuffers(1, &m.gpu.selectionVbo);
    glBindBuffer(GL_ARRAY_BUFFER, m.gpu.selectionVbo);
    glBufferData(GL_ARRAY_BUFFER, n, nullptr, GL_DYNAMIC_DRAW);

    if (m.meshMode) {
        glGenBuffers(1, &m.gpu.meshEbo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.gpu.meshEbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(meshDrawIndices(m).size() * sizeof(std::uint32_t)), nullptr,
                     GL_DYNAMIC_DRAW);

        // 仅保存“当前选中三角形”的索引。高亮复用原 Position/Normal/Color VBO，
        // 不再把 Triangle selection 映射成 vertex selection，避免共享顶点造成扩散噪声。
        glGenBuffers(1, &m.gpu.selectedMeshEbo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.gpu.selectedMeshEbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(meshDrawIndices(m).size() * sizeof(std::uint32_t)), nullptr,
                     GL_DYNAMIC_DRAW);
    }

#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    if (m.textureResult && m.textureResult->atlas.valid()) {
        glGenTextures(1, &m.textureGl);
        glBindTexture(GL_TEXTURE_2D, m.textureGl);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m.textureResult->atlas.width, m.textureResult->atlas.height,
                     0, GL_RGB, GL_UNSIGNED_BYTE, m.textureResult->atlas.pixels.data());
        glBindTexture(GL_TEXTURE_2D, 0);
    }
#endif

    backend_->createVertexArrays(*this, m.gpu);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    m.glCreated = true;
}

void PointCloudWidget::destroyModelGl(Model& m) {
    if (!m.glCreated)
        return;
    if (backend_)
        backend_->destroyVertexArrays(*this, m.gpu);

    GLuint bufs[] = {m.gpu.positionVbo,  m.gpu.colorVbo,     m.gpu.normalVbo, m.gpu.texcoordVbo,
                     m.gpu.flagsVbo,     m.gpu.selectionVbo, m.gpu.meshEbo,   m.gpu.selectedMeshEbo,
                     m.wireEbo};
    glDeleteBuffers(9, bufs);
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    if (m.textureGl) glDeleteTextures(1, &m.textureGl);
    m.textureGl = 0;
#endif
    m.gpu = {};
    m.wireEbo = 0;
    m.wireIndexCount = 0;
    m.wireDirty = true;
    m.drawPointCount = m.drawIndexCount = m.selectedMeshIndexCount = 0;
    m.glCreated = false;
    // 主模型销毁时后台 Live RT VBO 也必须释放。
    destroyLiveBackGl(m);
    m.liveBackCloud.reset();
    m.livePostSwapPoints.clear();
}


void PointCloudWidget::destroyLiveBackGl(Model& m) {
    if (!m.liveBackCreated)
        return;
    if (backend_)
        backend_->destroyVertexArrays(*this, m.liveBackGpu);
    GLuint bufs[] = {m.liveBackGpu.positionVbo, m.liveBackGpu.colorVbo, m.liveBackGpu.normalVbo,
                     m.liveBackGpu.texcoordVbo, m.liveBackGpu.flagsVbo, m.liveBackGpu.selectionVbo,
                     m.liveBackGpu.meshEbo};
    glDeleteBuffers(7, bufs);
    m.liveBackGpu = {};
    m.liveBackCreated = false;
    m.liveBackCapacity = 0;
    m.liveBackUploadCursor = 0;
}

void PointCloudWidget::uploadPointRangeNow(Model& m, std::size_t first, std::size_t count) {
    if (!m.glCreated || !m.cloud || count == 0 || first >= m.cloud->size())
        return;
    count = std::min(count, m.cloud->size() - first);
    std::vector<JMEngine::Vec3f> pos(count), normal(count);
    std::vector<std::uint32_t> color(count);
    std::vector<std::uint8_t> flags(count), sel(count);
    const auto& pts = m.cloud->points();
    for (std::size_t i = 0; i < count; ++i) {
        const auto& point = pts[first + i];
        pos[i] = point.position;
        normal[i] = point.normal;
        color[i] = m.useDisplayColor ? packedColor(m.displayColor) : point.rgba;
        flags[i] = static_cast<std::uint8_t>(point.flags & 0xFFu);
        sel[i] = (first + i < m.selectionMask.size() && m.selectionMask[first + i]) ? 1u : 0u;
    }
    const GLintptr off = static_cast<GLintptr>(first);
    glBindBuffer(GL_ARRAY_BUFFER, m.gpu.positionVbo);
    glBufferSubData(GL_ARRAY_BUFFER, off * sizeof(JMEngine::Vec3f), count * sizeof(JMEngine::Vec3f), pos.data());
    glBindBuffer(GL_ARRAY_BUFFER, m.gpu.normalVbo);
    glBufferSubData(GL_ARRAY_BUFFER, off * sizeof(JMEngine::Vec3f), count * sizeof(JMEngine::Vec3f), normal.data());
    glBindBuffer(GL_ARRAY_BUFFER, m.gpu.colorVbo);
    glBufferSubData(GL_ARRAY_BUFFER, off * 4, count * 4, color.data());
    glBindBuffer(GL_ARRAY_BUFFER, m.gpu.flagsVbo);
    glBufferSubData(GL_ARRAY_BUFFER, off, count, flags.data());
    glBindBuffer(GL_ARRAY_BUFFER, m.gpu.selectionVbo);
    glBufferSubData(GL_ARRAY_BUFFER, off, count, sel.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    m.uploadPointCursor = std::max(m.uploadPointCursor, first + count);
    m.drawPointCount = static_cast<GLsizei>(std::max<std::size_t>(m.drawPointCount, first + count));
}

void PointCloudWidget::uploadLiveBackIncremental(Model& m, std::size_t& budget) {
    if (!m.liveBackCloud || m.liveBackCloud->empty() || !backend_ || budget == 0)
        return;
    const std::size_t required = std::max<std::size_t>(m.liveBackCloud->size(), scanPreviewPointLimit_);
    if (!m.liveBackCreated || m.liveBackCapacity < required) {
        if (m.liveBackCreated)
            destroyLiveBackGl(m);
        const GLsizeiptr n = static_cast<GLsizeiptr>(required);
        glGenBuffers(1, &m.liveBackGpu.positionVbo);
        glBindBuffer(GL_ARRAY_BUFFER, m.liveBackGpu.positionVbo);
        glBufferData(GL_ARRAY_BUFFER, n * sizeof(JMEngine::Vec3f), nullptr, GL_DYNAMIC_DRAW);
        glGenBuffers(1, &m.liveBackGpu.colorVbo);
        glBindBuffer(GL_ARRAY_BUFFER, m.liveBackGpu.colorVbo);
        glBufferData(GL_ARRAY_BUFFER, n * 4, nullptr, GL_DYNAMIC_DRAW);
        glGenBuffers(1, &m.liveBackGpu.normalVbo);
        glBindBuffer(GL_ARRAY_BUFFER, m.liveBackGpu.normalVbo);
        glBufferData(GL_ARRAY_BUFFER, n * sizeof(JMEngine::Vec3f), nullptr, GL_DYNAMIC_DRAW);
        glGenBuffers(1, &m.liveBackGpu.flagsVbo);
        glBindBuffer(GL_ARRAY_BUFFER, m.liveBackGpu.flagsVbo);
        glBufferData(GL_ARRAY_BUFFER, n, nullptr, GL_DYNAMIC_DRAW);
        glGenBuffers(1, &m.liveBackGpu.selectionVbo);
        glBindBuffer(GL_ARRAY_BUFFER, m.liveBackGpu.selectionVbo);
        glBufferData(GL_ARRAY_BUFFER, n, nullptr, GL_DYNAMIC_DRAW);
        backend_->createVertexArrays(*this, m.liveBackGpu);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        m.liveBackCreated = true;
        m.liveBackCapacity = required;
        m.liveBackUploadCursor = 0;
    }

    constexpr std::size_t bytesPerPoint = sizeof(JMEngine::Vec3f) * 2u + 6u;
    const std::size_t maxByBudget = std::max<std::size_t>(1u, budget / bytesPerPoint);
    const std::size_t count = std::min(maxByBudget, m.liveBackCloud->size() - m.liveBackUploadCursor);
    if (count == 0)
        return;
    std::vector<JMEngine::Vec3f> pos(count), normal(count);
    std::vector<std::uint32_t> color(count);
    std::vector<std::uint8_t> flags(count, 0u), sel(count, 0u);
    const auto& pts = m.liveBackCloud->points();
    for (std::size_t i = 0; i < count; ++i) {
        const auto& point = pts[m.liveBackUploadCursor + i];
        pos[i] = point.position;
        normal[i] = point.normal;
        color[i] = point.rgba;
        flags[i] = static_cast<std::uint8_t>(point.flags & 0xFFu);
    }
    const GLintptr off = static_cast<GLintptr>(m.liveBackUploadCursor);
    glBindBuffer(GL_ARRAY_BUFFER, m.liveBackGpu.positionVbo);
    glBufferSubData(GL_ARRAY_BUFFER, off * sizeof(JMEngine::Vec3f), count * sizeof(JMEngine::Vec3f), pos.data());
    glBindBuffer(GL_ARRAY_BUFFER, m.liveBackGpu.normalVbo);
    glBufferSubData(GL_ARRAY_BUFFER, off * sizeof(JMEngine::Vec3f), count * sizeof(JMEngine::Vec3f), normal.data());
    glBindBuffer(GL_ARRAY_BUFFER, m.liveBackGpu.colorVbo);
    glBufferSubData(GL_ARRAY_BUFFER, off * 4, count * 4, color.data());
    glBindBuffer(GL_ARRAY_BUFFER, m.liveBackGpu.flagsVbo);
    glBufferSubData(GL_ARRAY_BUFFER, off, count, flags.data());
    glBindBuffer(GL_ARRAY_BUFFER, m.liveBackGpu.selectionVbo);
    glBufferSubData(GL_ARRAY_BUFFER, off, count, sel.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    m.liveBackUploadCursor += count;
    const std::size_t bytes = count * bytesPerPoint;
    budget = bytes >= budget ? 0u : budget - bytes;

    if (m.liveBackUploadCursor >= m.liveBackCloud->size()) {
        // Back 已完整：同一帧内交换。front 从未被部分覆盖，所以没有半新半旧闪烁。
        std::swap(m.gpu, m.liveBackGpu);
        m.liveBackCreated = true; // 交换后的 liveBackGpu 是旧 front，继续作为下一轮 back 复用。
        m.liveBackCapacity = m.gpuReservedPointCapacity;
        m.gpuReservedPointCapacity = std::max<std::size_t>(m.liveBackCloud->size(), scanPreviewPointLimit_);
        m.cloud = m.liveBackCloud;
        m.editor.setPointCloud(m.cloud);
        m.selectionMask.assign(m.cloud->size(), 0u);
        m.uploadPointCursor = m.cloud->size();
        m.drawPointCount = static_cast<GLsizei>(m.cloud->size());
        m.liveBackCloud.reset();
        m.liveBackUploadCursor = 0;

        // 优化快照生成后仍到来的正常帧补到新 front，保证最新扫描历史不倒退。
        if (!m.livePostSwapPoints.empty()) {
            auto& dst = m.cloud->points();
            const std::size_t first = dst.size();
            const std::size_t room = scanPreviewPointLimit_ > first ? scanPreviewPointLimit_ - first : 0u;
            const std::size_t add = std::min(room, m.livePostSwapPoints.size());
            dst.insert(dst.end(), m.livePostSwapPoints.begin(), m.livePostSwapPoints.begin() + static_cast<std::ptrdiff_t>(add));
            m.selectionMask.resize(dst.size(), 0u);
            if (add > 0)
                uploadPointRangeNow(m, first, add);
            m.livePostSwapPoints.clear();
        }
        statusText_ = QString::fromUtf8("Live姿态优化已更新：%1 点").arg(qulonglong(m.cloud->size()));
    }
}

void PointCloudWidget::uploadModelIncremental(Model& m, std::size_t& budget) {
    if (!m.cloud)
        return;
    if (!m.glCreated)
        createModelGl(m);

    // 保留旧版已经验证稳定的固定 VBO chunk。每帧只上传一部分顶点，
    // 并把剩余 budget 立即交给 EBO，让两者在同一帧同步向前推进。
    // 关键是：EBO 不再等待所有 Position VBO 完成后才开始上传。
    constexpr std::size_t chunkPoints = 150000;
    constexpr std::size_t bytesPerPoint = sizeof(JMEngine::Vec3f) * 2u + 6u;

    const auto& pts = m.cloud->points();
    const auto& indices = meshDrawIndices(m);
    if (m.uploadPointCursor < pts.size() && budget > 0) {
        const std::size_t count = std::min(chunkPoints, pts.size() - m.uploadPointCursor);
        std::vector<JMEngine::Vec3f> pos(count), normal(count);
        std::vector<std::uint32_t> color(count);
        std::vector<std::uint8_t> flags(count), sel(count);

        for (std::size_t i = 0; i < count; ++i) {
            const auto& point = pts[m.uploadPointCursor + i];
            pos[i] = point.position;
            normal[i] = point.normal;
            color[i] =
                m.useDisplayColor ? packedColor(m.displayColor) : point.rgba; // 只覆盖 GPU 显示色，不修改 Core RGB。
            flags[i] = static_cast<std::uint8_t>(point.flags & 0xFFu);
            sel[i] = static_cast<std::uint8_t>(m.selectionMask[m.uploadPointCursor + i] ? 1u : 0u);
        }

        const GLintptr off = static_cast<GLintptr>(m.uploadPointCursor);
        glBindBuffer(GL_ARRAY_BUFFER, m.gpu.positionVbo);
        glBufferSubData(GL_ARRAY_BUFFER, off * sizeof(JMEngine::Vec3f), count * sizeof(JMEngine::Vec3f), pos.data());
        glBindBuffer(GL_ARRAY_BUFFER, m.gpu.normalVbo);
        glBufferSubData(GL_ARRAY_BUFFER, off * sizeof(JMEngine::Vec3f), count * sizeof(JMEngine::Vec3f), normal.data());
        glBindBuffer(GL_ARRAY_BUFFER, m.gpu.colorVbo);
        glBufferSubData(GL_ARRAY_BUFFER, off * 4, count * 4, color.data());
        glBindBuffer(GL_ARRAY_BUFFER, m.gpu.flagsVbo);
        glBufferSubData(GL_ARRAY_BUFFER, off, count, flags.data());
        glBindBuffer(GL_ARRAY_BUFFER, m.gpu.selectionVbo);
        glBufferSubData(GL_ARRAY_BUFFER, off, count, sel.data());

        m.uploadPointCursor += count;
        m.drawPointCount = static_cast<GLsizei>(m.uploadPointCursor);

        const std::size_t bytes = count * bytesPerPoint;
        budget = bytes >= budget ? 0u : budget - bytes;
    }

    // 与旧稳定版本一致：只要本帧仍有上传预算，就立即推进 Mesh EBO。
    // 不再使用 `m.uploadPointCursor >= pts.size()` 作为门槛，否则会形成：
    // 先完整显示点 -> 再开始显示 Mesh 的明显“两次加载”视觉效果。
    if (m.meshMode && m.uploadIndexCursor < indices.size() && budget >= 3u * sizeof(std::uint32_t)) {
        constexpr std::size_t maxIndexChunk = 450000;
        const std::size_t bytesPerIndex = sizeof(std::uint32_t);

        std::size_t count = std::min<std::size_t>(maxIndexChunk, indices.size() - m.uploadIndexCursor);
        count = std::min<std::size_t>(count, budget / bytesPerIndex);
        count -= count % 3u; // 只提交完整三角形。

        if (count >= 3u) {
            // VAO-only：EBO 是 VAO 状态的一部分。上传时绑定所属 Render VAO，
            // 不创建第二份 EBO，也不创建 Picking 几何缓冲。
            backend_->bindRenderLayout(*this, m.gpu);
            // VAO-only：meshEbo 已经记录在 VAO 中，这里只更新当前 VAO 的 EBO 内容，
            // 不再额外修改 GL_ELEMENT_ARRAY_BUFFER 绑定。
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLintptr>(m.uploadIndexCursor * sizeof(std::uint32_t)),
                            static_cast<GLsizeiptr>(count * sizeof(std::uint32_t)),
                            indices.data() + m.uploadIndexCursor);
            backend_->unbindLayout(*this);

            m.uploadIndexCursor += count;

            const std::size_t bytes = count * bytesPerIndex;
            budget = bytes >= budget ? 0u : budget - bytes;
        }
    }

    // Mesh 仍然分帧上传以避免大模型阻塞 UI，但不再显示中间态。
    // 只有 Position VBO 和 EBO 都完整提交后，才一次性开放 drawIndexCount。
    // 这样加载过程中不会先冒出一个三角形，也不会经历 Point -> Mesh 切换。
    if (m.meshMode) {
        const bool verticesComplete = (m.uploadPointCursor >= pts.size());
        const bool indicesComplete = (m.uploadIndexCursor >= indices.size());
        m.meshUploadComplete = verticesComplete && indicesComplete;
        m.drawIndexCount = m.meshUploadComplete ? static_cast<GLsizei>(indices.size()) : 0;
    }
}

void PointCloudWidget::paintGL() {
    if (!renderReady_) {
        QPainter painter(this);
        painter.fillRect(rect(), QColor(12, 12, 14));
        painter.setPen(Qt::red);
        painter.drawText(20, 40, statusText_);
        return;
    }
    if (!renderFpsClock_.isValid())
        renderFpsClock_.start();
    ++renderFpsFrameCount_;
    const qint64 fpsElapsedMs = renderFpsClock_.elapsed();
    if (fpsElapsedMs >= 500) {
        const int tenths = int((qint64(renderFpsFrameCount_) * 10000 + fpsElapsedMs / 2) / fpsElapsedMs);
        renderFpsTenths_.store(tenths, std::memory_order_relaxed);
        renderFpsFrameCount_ = 0;
        renderFpsClock_.restart();
    }
    // QOpenGLWidget 和 Picking 共用同一个 OpenGL Context。Picking 会修改
    // framebuffer / viewport / depth / blend 等全局状态，因此正常渲染每一帧
    // 都显式恢复，不能依赖上一帧留下的状态。
    glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());
    glViewport(0, 0, std::max(1, int(width() * devicePixelRatioF())), std::max(1, int(height() * devicePixelRatioF())));
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 每帧最多上传约 12MB，避免大模型首次进入场景时冻结触摸/UI。
    std::size_t budget = 12u * 1024u * 1024u;
    for (auto& m : models_) {
        uploadModelIncremental(*m, budget);
        uploadLiveBackIncremental(*m, budget);
    }
    drawScene();
    drawScanCameraOverlay();
    drawGestureOverlay();
    drawUtilityOverlay();
    drawBasePlaneOverlay();

    if (!statusText_.isEmpty()) {
        QPainter p(this);
        p.setPen(Qt::white);
        p.drawText(12, 24, statusText_);
    }
    bool uploading = false;
    for (const auto& m : models_) {
        if (!m->cloud)
            continue;
        const auto& indices = meshDrawIndices(*m);
        if (m->uploadPointCursor < m->cloud->size() || (m->meshMode && m->uploadIndexCursor < indices.size()) ||
            (m->liveBackCloud && m->liveBackUploadCursor < m->liveBackCloud->size())) {
            uploading = true;
        }
    }
    if (uploading)
        update();
}

void PointCloudWidget::drawScene() {
    for (auto& m : models_) {
        if (!m->visible || !m->cloud)
            continue;
        const bool scanStatusLayer =
            (!currentScanFramePath_.isEmpty() && m->path == currentScanFramePath_) ||
            (!recoveryScanFramePath_.isEmpty() && m->path == recoveryScanFramePath_);
        if (scanStatusLayer)
            glDepthFunc(GL_LEQUAL); // green current + yellow recovery reference overlay RGB history
        drawModel(*m);
        drawSelectionOverlay(*m);
        if (scanStatusLayer)
            glDepthFunc(GL_LESS);
    }
}

void PointCloudWidget::rebuildWireframeBuffer(Model& m) {
    if (!m.meshMode || !m.mesh || !m.glCreated || !backend_)
        return;
    const auto& tri = meshDrawIndices(m);
    std::vector<std::uint32_t> edges;
    edges.reserve(tri.size() * 2u);
    for (std::size_t i = 0; i + 2 < tri.size(); i += 3) {
        const auto a = tri[i], b = tri[i + 1], c = tri[i + 2];
        edges.push_back(a);
        edges.push_back(b);
        edges.push_back(b);
        edges.push_back(c);
        edges.push_back(c);
        edges.push_back(a);
    }
    if (!m.wireEbo)
        glGenBuffers(1, &m.wireEbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.wireEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(edges.size() * sizeof(std::uint32_t)),
                 edges.empty() ? nullptr : edges.data(), GL_STATIC_DRAW);
    m.wireIndexCount = static_cast<GLsizei>(edges.size());
    m.wireDirty = false;
}

void PointCloudWidget::drawModel(Model& m) {
    if (!m.glCreated || m.drawPointCount <= 0 || !backend_)
        return;
    const auto mvp = modelMvp(m);
    const bool pointMode = !m.meshMode || m.displayMode == DisplayMode::Points;
    QOpenGLShaderProgram& prog = pointMode ? pointProgram_ : meshProgram_;
    prog.bind();
    prog.setUniformValue("uMVP", QMatrix4x4(mvp.m.data()).transposed());

    // 相机 Headlight。RenderBackend 中法线保持在模型局部坐标，因此这里把
    // “模型 -> 相机”的世界方向变换回模型局部坐标，再传入 shader。
    // modelTransform 当前是刚体变换（手工/自动对齐均如此），其逆旋转等于 R^T。
    const auto eye = camera_.eye();
    const auto lightWorld = JMEngine::example::normalize(JMEngine::example::sub(eye, camera_.target));
    const auto& mm = m.modelTransform.m;
    JMEngine::Vec3f lightLocal{
        mm[0] * lightWorld.x + mm[1] * lightWorld.y + mm[2] * lightWorld.z,
        mm[4] * lightWorld.x + mm[5] * lightWorld.y + mm[6] * lightWorld.z,
        mm[8] * lightWorld.x + mm[9] * lightWorld.y + mm[10] * lightWorld.z};
    lightLocal = JMEngine::example::normalize(lightLocal);
    prog.setUniformValue("uLightDir", QVector3D(lightLocal.x, lightLocal.y, lightLocal.z));
    prog.setUniformValue("uPointSize", 3.0f * float(devicePixelRatioF()));
    prog.setUniformValue("uPointMode", pointMode ? 1.0f : 0.0f);
    prog.setUniformValue("uForceSelected", 0.0f);
    prog.setUniformValue("uWireframe", 0.0f);
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    const bool useTexture = !pointMode && m.textureGl != 0 && m.textureResult && m.textureResult->atlas.valid();
    prog.setUniformValue("uUseTexture", useTexture ? 1.0f : 0.0f);
    prog.setUniformValue("uTexture", 0);
    if (useTexture) {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m.textureGl);
    }
#else
    prog.setUniformValue("uUseTexture", 0.0f);
#endif

    backend_->bindRenderLayout(*this, m.gpu);
    if (pointMode && m.liveFramePoseMode && !m.liveFrames.empty()) {
        // One shared local-point VBO; each frame is drawn with its latest optimized RT.
        // getResults() therefore changes only tiny CPU pose records, never historical point data.
        const auto vp = currentMvp();
        for (const auto& frame : m.liveFrames) {
            if (frame.count == 0 || frame.first >= std::size_t(m.drawPointCount)) continue;
            const auto world = JMEngine::example::multiply(m.modelTransform, frame.pose);
            const auto frameMvp = JMEngine::example::multiply(vp, world);
            prog.setUniformValue("uMVP", QMatrix4x4(frameMvp.m.data()).transposed());

            QMatrix3x3 normalMatrix;
            normalMatrix(0,0)=world.m[0]; normalMatrix(0,1)=world.m[4]; normalMatrix(0,2)=world.m[8];
            normalMatrix(1,0)=world.m[1]; normalMatrix(1,1)=world.m[5]; normalMatrix(1,2)=world.m[9];
            normalMatrix(2,0)=world.m[2]; normalMatrix(2,1)=world.m[6]; normalMatrix(2,2)=world.m[10];
            prog.setUniformValue("uNormalMatrix", normalMatrix);
            const GLsizei count = static_cast<GLsizei>(std::min<std::size_t>(frame.count, std::size_t(m.drawPointCount) - frame.first));
            glDrawArrays(GL_POINTS, static_cast<GLint>(frame.first), count);
        }
    } else if (pointMode) {
        glDrawArrays(GL_POINTS, 0, m.drawPointCount);
    } else if (m.drawIndexCount > 0) {
        if (m.displayMode == DisplayMode::Solid || m.displayMode == DisplayMode::SolidWireframe)
            glDrawElements(GL_TRIANGLES, m.drawIndexCount, GL_UNSIGNED_INT, nullptr);
        if (m.displayMode == DisplayMode::Wireframe || m.displayMode == DisplayMode::SolidWireframe) {
            if (m.wireDirty)
                rebuildWireframeBuffer(m);
            if (m.wireEbo && m.wireIndexCount > 0) {
                prog.setUniformValue("uWireframe", 1.0f);
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.wireEbo);
                glLineWidth(1.0f);
                glDrawElements(GL_LINES, m.wireIndexCount, GL_UNSIGNED_INT, nullptr);
                prog.setUniformValue("uWireframe", 0.0f);
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.gpu.meshEbo);
            }
        }
    }
    backend_->unbindLayout(*this);
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    if (!pointMode && m.textureGl) glBindTexture(GL_TEXTURE_2D, 0);
#endif
    prog.release();
}

void PointCloudWidget::drawBasePlaneOverlay() {
    if(!basePlane_.active) return;
    const auto* m=activeModel(); if(!m) return;
    const auto p0=add3(basePlane_.point,mul3(basePlane_.normal,basePlane_.offset));
    JMEngine::Vec3f ref = std::fabs(basePlane_.normal.y)<0.9f?JMEngine::Vec3f{0,1,0}:JMEngine::Vec3f{1,0,0};
    const auto u=norm3(cross3(basePlane_.normal,ref));
    const auto v=norm3(cross3(basePlane_.normal,u));
    const float r=basePlane_.visualRadius;
    std::array<JMEngine::Vec3f,4> local{
        add3(add3(p0,mul3(u,-r)),mul3(v,-r)), add3(add3(p0,mul3(u,r)),mul3(v,-r)),
        add3(add3(p0,mul3(u,r)),mul3(v,r)), add3(add3(p0,mul3(u,-r)),mul3(v,r))};
    QPolygonF poly; bool ok=true;
    const auto mvp=modelMvp(*m);
    for(const auto& q:local){float x=0,y=0,d=0;if(!projectPointToScreen(q,mvp,width(),height(),x,y,d)){ok=false;break;}poly<<QPointF(x,y);}
    float cx=0,cy=0,cd=0; if(!projectPointToScreen(p0,mvp,width(),height(),cx,cy,cd)) ok=false;
    if(!ok) return;
    QPainter painter(this); painter.setRenderHint(QPainter::Antialiasing,true);
    painter.setPen(QPen(QColor(0,220,255,230),2)); painter.setBrush(QColor(0,180,255,45)); painter.drawPolygon(poly);
    painter.setBrush(QColor(255,220,0,230)); painter.setPen(QPen(Qt::black,1)); painter.drawEllipse(QPointF(cx,cy),8,8);
    painter.setPen(Qt::white); painter.drawText(QPointF(cx+12,cy-10),QString::fromUtf8("拖动上下调整基底"));
}

void PointCloudWidget::drawScanCameraOverlay() {
    if (!scanCameraPose_)
        return;

    const auto& pose = *scanCameraPose_;
    const auto right = JMEngine::example::normalize(pose.right);
    const auto up = JMEngine::example::normalize(pose.up);
    const auto forward = JMEngine::example::normalize(pose.forward);
    if (JMEngine::example::length(forward) < 1.0e-6f)
        return;

    // Draw a small 3D camera frustum in world coordinates.  Green = tracking, yellow = lost.
    const float scale = std::max(camera_.sceneRadius * 0.022f, 0.5f);
    const float depth = scale;
    const float halfW = scale * 0.58f;
    const float halfH = scale * 0.36f;
    const auto center = JMEngine::example::add(pose.position, JMEngine::example::mul(forward, depth));
    const auto rw = JMEngine::example::mul(right, halfW);
    const auto uh = JMEngine::example::mul(up, halfH);
    std::array<JMEngine::Vec3f, 5> v{
        pose.position,
        JMEngine::example::add(JMEngine::example::add(center, rw), uh),
        JMEngine::example::add(JMEngine::example::sub(center, rw), uh),
        JMEngine::example::sub(JMEngine::example::sub(center, rw), uh),
        JMEngine::example::sub(JMEngine::example::add(center, rw), uh)};

    const auto mvp = camera_.mvp(std::max(1, width()), std::max(1, height()));
    std::array<QPointF, 5> screen{};
    std::array<bool, 5> valid{};
    for (std::size_t i=0;i<v.size();++i) {
        float x=0.0f,y=0.0f,d=0.0f;
        valid[i] = projectPointToScreen(v[i], mvp, width(), height(), x, y, d);
        if (valid[i]) screen[i] = QPointF(x,y);
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const QColor stateColor = pose.trackingOk ? QColor(0, 230, 110) : QColor(255, 210, 0);

    QPen pen(stateColor); pen.setWidth(2);
    painter.setPen(pen); painter.setBrush(Qt::NoBrush);
    for (int i=1;i<=4;++i) if (valid[0] && valid[i]) painter.drawLine(screen[0], screen[i]);
    for (int i=1;i<=4;++i) { const int j=(i==4)?1:i+1; if (valid[i]&&valid[j]) painter.drawLine(screen[i], screen[j]); }

    // Optical axis makes the current scanning direction obvious.
    const auto axisEnd = JMEngine::example::add(pose.position, JMEngine::example::mul(forward, depth * 1.8f));
    float ax=0.0f, ay=0.0f, ad=0.0f;
    if (valid[0] && projectPointToScreen(axisEnd, mvp, width(), height(), ax, ay, ad))
        painter.drawLine(screen[0], QPointF(ax, ay));

}

void PointCloudWidget::drawGestureOverlay() {
    if (!editGestureActive_)
        return;
    QPainter painter(this);
    QPen pen(QColor(255, 190, 0));
    pen.setWidth(2);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    if (interactionMode_ == InteractionMode::Rectangle)
        painter.drawRect(QRect(pressPos_, currentPos_).normalized());
    else if (interactionMode_ == InteractionMode::Circle) {
        const int r = int(std::hypot(currentPos_.x() - pressPos_.x(), currentPos_.y() - pressPos_.y()));
        painter.drawEllipse(pressPos_, r, r);
    } else if (stroke_.size() > 1) {
        QPolygon poly;
        for (const auto& p : stroke_)
            poly << p;
        painter.drawPolyline(poly);
    }
}


void PointCloudWidget::drawUtilityOverlay() {
    if (utilityPoints_.empty() && utilityMode_ == UtilityMode::None)
        return;

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    QPen pen(QColor(0, 220, 255));
    pen.setWidth(2);
    painter.setPen(pen);
    painter.setBrush(QColor(0, 220, 255));

    std::vector<QPointF> screen;
    screen.reserve(utilityPoints_.size());
    const auto mvp = currentMvp();
    for (const auto& p : utilityPoints_) {
        float x=0.0f, y=0.0f, d=0.0f;
        if (projectPointToScreen(p, mvp, width(), height(), x, y, d)) {
            screen.emplace_back(x,y);
            painter.drawEllipse(QPointF(x,y), 5.0, 5.0);
        }
    }
    if (screen.size() >= 2) {
        painter.setBrush(Qt::NoBrush);
        for (std::size_t i=1;i<screen.size();++i)
            painter.drawLine(screen[i-1], screen[i]);
    }
}

void PointCloudWidget::startDistanceMeasurement() {
    utilityMode_ = UtilityMode::MeasureDistance;
    utilityPoints_.clear();
    alignmentSourcePath_.clear();
    alignmentSourceCount_ = 0;
    statusText_ = QString::fromUtf8("距离测量：点击第 1 个点");
    update();
}

void PointCloudWidget::startAngleMeasurement() {
    utilityMode_ = UtilityMode::MeasureAngle;
    utilityPoints_.clear();
    alignmentSourcePath_.clear();
    alignmentSourceCount_ = 0;
    statusText_ = QString::fromUtf8("角度测量：点击 A 点");
    update();
}

void PointCloudWidget::startThreePointAlignment() {
    if (!activeModel()) {
        statusText_ = QString::fromUtf8("三点对齐：请先激活移动模型");
        return;
    }
    utilityMode_ = UtilityMode::AlignThreePoint;
    utilityPoints_.clear();
    alignmentSourcePath_ = activeModelPath();
    alignmentSourceCount_ = 0;
    statusText_ = QString::fromUtf8("三点对齐：在移动模型上点击源点 1/3");
    update();
}

void PointCloudWidget::measureActiveSurfaceArea() {
    const auto* model = activeModel();
    if (!model || !model->meshMode || !model->mesh) {
        statusText_ = QString::fromUtf8("面积测量仅对三角网格有效；点云请先执行可靠的表面重建");
        update();
        return;
    }
    const auto r = JMEngine::measureSurfaceArea(*model->mesh);
    if (!r.valid) {
        statusText_ = QString::fromUtf8("面积测量失败：当前网格没有有效三角形");
    } else {
        statusText_ = QString::fromUtf8("表面积 = %1（模型单位²） | 有效三角形 %2 | 退化 %3")
                          .arg(r.area, 0, 'g', 12).arg(r.triangleCount).arg(r.degenerateTriangleCount);
    }
    update();
}

void PointCloudWidget::measureActiveVolume() {
    const auto* model = activeModel();
    if (!model || !model->meshMode || !model->mesh) {
        statusText_ = QString::fromUtf8("体积测量仅对封闭三角网格有效；点云不能直接给出工业可信体积");
        update();
        return;
    }
    const auto r = JMEngine::measureVolume(*model->mesh);
    if (!r.valid) {
        statusText_ = QString::fromUtf8("体积无效：网格未闭合/非流形 | 边界边 %1 | 非流形边 %2 | 退化三角形 %3")
                          .arg(r.boundaryEdgeCount).arg(r.nonManifoldEdgeCount).arg(r.degenerateTriangleCount);
    } else {
        statusText_ = QString::fromUtf8("体积 = %1（模型单位³） | 闭合拓扑通过 | 有效三角形 %2")
                          .arg(r.volume, 0, 'g', 12).arg(r.triangleCount);
    }
    update();
}

void PointCloudWidget::startOrRunAutoAlignment() {
    if (autoAlignmentBusy_) {
        statusText_ = QString::fromUtf8("自动对齐正在后台计算，请勿重复提交");
        update();
        return;
    }
    auto* current = activeModel();
    if (!current || !current->cloud) {
        statusText_ = QString::fromUtf8("自动对齐：请先激活移动模型");
        update();
        return;
    }
    if (autoAlignmentSourcePath_.isEmpty()) {
        autoAlignmentSourcePath_ = current->path;
        utilityMode_ = UtilityMode::AutoAlign;
        statusText_ = QString::fromUtf8("自动对齐：已锁定移动模型 %1；请激活基准模型，再次执行自动对齐")
                          .arg(QFileInfo(current->path).fileName());
        update();
        return;
    }
    if (current->path == autoAlignmentSourcePath_) {
        statusText_ = QString::fromUtf8("自动对齐：基准模型必须与移动模型不同，请先切换激活模型");
        update();
        return;
    }
    const int sourceIndex = findModel(autoAlignmentSourcePath_);
    if (sourceIndex < 0) {
        autoAlignmentSourcePath_.clear();
        utilityMode_ = UtilityMode::None;
        statusText_ = QString::fromUtf8("自动对齐失败：移动模型已被移除，请重新开始");
        update();
        return;
    }
    auto& source = *models_[static_cast<std::size_t>(sourceIndex)];
    const QString sourcePath = source.path;
    const QString targetPath = current->path;
    const auto sourceCloud = source.cloud;
    const auto targetCloud = current->cloud;
    const auto sourceTf = source.modelTransform;
    const auto targetTf = current->modelTransform;
    autoAlignmentBusy_ = true;
    editBusy_ = true;
    statusText_ = QString::fromUtf8("自动对齐：后台执行 PCA 多初值粗配准 + Trimmed ICP 精配准...");
    update();
    QPointer<PointCloudWidget> self(this);
    workerPool_.start([self, sourcePath, targetPath, sourceCloud, targetCloud, sourceTf, targetTf] {
        JMEngine::AutoAlignmentOptions opt;
        const auto result = JMEngine::alignPointClouds(*sourceCloud, sourceTf, *targetCloud, targetTf, opt);
        QMetaObject::invokeMethod(self, [self, sourcePath, targetPath, result] {
            if (!self) return;
            self->autoAlignmentBusy_ = false;
            self->editBusy_ = false;
            const int si = self->findModel(sourcePath);
            const int ti = self->findModel(targetPath);
            if (si < 0 || ti < 0) {
                self->statusText_ = QString::fromUtf8("自动对齐结果已丢弃：源模型或基准模型已被移除");
            } else if (!result.success) {
                QString reason;
                switch (result.status) {
                case JMEngine::AutoAlignmentStatus::NotEnoughPoints: reason = QString::fromUtf8("有效采样点不足"); break;
                case JMEngine::AutoAlignmentStatus::NoCorrespondence: reason = QString::fromUtf8("没有稳定对应关系"); break;
                case JMEngine::AutoAlignmentStatus::NotConverged: reason = QString::fromUtf8("ICP 未收敛"); break;
                case JMEngine::AutoAlignmentStatus::QualityRejected: reason = QString::fromUtf8("质量门槛未通过"); break;
                default: reason = QString::fromUtf8("输入无效"); break;
                }
                self->statusText_ = QString::fromUtf8("自动对齐拒绝应用：%1 | RMS=%2 | 内点率=%3% | 对应=%4")
                    .arg(reason).arg(result.rmsError,0,'g',6).arg(result.inlierRatio*100.0f,0,'f',1).arg(result.correspondenceCount);
            } else {
                auto& sm = *self->models_[static_cast<std::size_t>(si)];
                sm.modelTransform = JMEngine::example::multiply(result.transform, sm.modelTransform);
                self->statusText_ = QString::fromUtf8("自动对齐完成 | RMS=%1 | 内点率=%2% | 对应=%3 | 迭代=%4")
                    .arg(result.rmsError,0,'g',6).arg(result.inlierRatio*100.0f,0,'f',1)
                    .arg(result.correspondenceCount).arg(result.iterations);
            }
            self->autoAlignmentSourcePath_.clear();
            self->utilityMode_ = UtilityMode::None;
            self->update();
        }, Qt::QueuedConnection);
    });
}

void PointCloudWidget::cancelUtilityMode() {
    utilityMode_ = UtilityMode::None;
    utilityPoints_.clear();
    alignmentSourcePath_.clear();
    alignmentSourceCount_ = 0;
    if (!autoAlignmentBusy_) autoAlignmentSourcePath_.clear();
    statusText_ = autoAlignmentBusy_ ? QString::fromUtf8("自动对齐已在计算中；当前版本不会强制中止后台算法，结果仍会经过质量门槛")
                                     : QString::fromUtf8("已取消测量/对齐");
    update();
}

std::optional<JMEngine::Vec3f> PointCloudWidget::pickActiveWorldPoint(const QPoint& pos) {
    auto* model = activeModel();
    if (!model || !model->cloud || model->cloud->empty())
        return std::nullopt;

    ensurePointPickIndex(*model);
    constexpr int radius = 10;
    const QRect bounds(pos.x()-radius, pos.y()-radius, radius*2+1, radius*2+1);
    const auto mvp = modelMvp(*model);
    auto candidates = pointPickCandidates(*model, mvp, bounds);
    if (candidates.empty())
        return std::nullopt;

    float bestScore = std::numeric_limits<float>::max();
    JMEngine::PointId bestId = JMEngine::kInvalidPointId;
    for (const auto id : candidates) {
        const auto* point = model->cloud->tryGet(id);
        if (!point || (point->flags & JMEngine::PointDeleted))
            continue;
        float sx=0.0f, sy=0.0f, depth=0.0f;
        if (!projectPointToScreen(point->position, mvp, width(), height(), sx, sy, depth))
            continue;
        const float dx=sx-float(pos.x()), dy=sy-float(pos.y());
        const float d2=dx*dx+dy*dy;
        if (d2 > float(radius*radius))
            continue;
        // 先保证点击位置接近，再优先最前表面，避免穿透到背面。
        const float score = d2 * 0.02f + depth;
        if (score < bestScore) {
            bestScore=score;
            bestId=id;
        }
    }
    if (bestId == JMEngine::kInvalidPointId)
        return std::nullopt;
    const auto* point=model->cloud->tryGet(bestId);
    return JMEngine::transformPoint(model->modelTransform, point->position);
}

void PointCloudWidget::handleUtilityClick(const QPoint& pos) {
    const auto picked = pickActiveWorldPoint(pos);
    if (!picked) {
        statusText_ = QString::fromUtf8("未拾取到表面点，请点击更靠近点云的位置");
        update();
        return;
    }

    if (utilityMode_ == UtilityMode::MeasureDistance) {
        utilityPoints_.push_back(*picked);
        if (utilityPoints_.size() == 1) {
            statusText_ = QString::fromUtf8("距离测量：点击第 2 个点");
        } else {
            const auto r = JMEngine::measureDistance(utilityPoints_[0], utilityPoints_[1]);
            statusText_ = QString::fromUtf8("距离 = %1（模型坐标单位）").arg(r.distance, 0, 'f', 6);
            utilityMode_ = UtilityMode::None;
        }
        update();
        return;
    }

    if (utilityMode_ == UtilityMode::MeasureAngle) {
        utilityPoints_.push_back(*picked);
        if (utilityPoints_.size() == 1)
            statusText_ = QString::fromUtf8("角度测量：点击顶点 B");
        else if (utilityPoints_.size() == 2)
            statusText_ = QString::fromUtf8("角度测量：点击 C 点");
        else {
            const auto r = JMEngine::measureAngle(utilityPoints_[0], utilityPoints_[1], utilityPoints_[2]);
            statusText_ = QString::fromUtf8("夹角 = %1°").arg(r.degrees, 0, 'f', 3);
            utilityMode_ = UtilityMode::None;
        }
        update();
        return;
    }

    if (utilityMode_ != UtilityMode::AlignThreePoint)
        return;

    const QString currentPath = activeModelPath();
    if (alignmentSourceCount_ < 3) {
        if (currentPath != alignmentSourcePath_) {
            statusText_ = QString::fromUtf8("请先完成移动模型的 3 个源点；当前源模型：%1").arg(alignmentSourcePath_);
            return;
        }
        alignmentSourceWorld_[static_cast<std::size_t>(alignmentSourceCount_)] = *picked;
        ++alignmentSourceCount_;
        utilityPoints_.push_back(*picked);
        if (alignmentSourceCount_ < 3) {
            statusText_ = QString::fromUtf8("三点对齐：在移动模型上点击源点 %1/3").arg(alignmentSourceCount_+1);
        } else {
            utilityPoints_.clear();
            statusText_ = QString::fromUtf8("源点完成。请在模型管理器激活基准模型，再依次点击对应目标点 1/3");
        }
        update();
        return;
    }

    if (currentPath == alignmentSourcePath_) {
        statusText_ = QString::fromUtf8("请先在模型管理器中激活另一个基准模型");
        return;
    }
    utilityPoints_.push_back(*picked);
    if (utilityPoints_.size() < 3) {
        statusText_ = QString::fromUtf8("三点对齐：点击目标点 %1/3").arg(utilityPoints_.size()+1);
        update();
        return;
    }

    std::array<JMEngine::Vec3f,3> target{utilityPoints_[0],utilityPoints_[1],utilityPoints_[2]};
    const auto result = JMEngine::alignThreePoints(alignmentSourceWorld_, target);
    if (!result.success) {
        statusText_ = QString::fromUtf8("三点对齐失败：三点过近或近似共线，请重新选择");
        utilityMode_ = UtilityMode::None;
        update();
        return;
    }
    const int sourceIndex = findModel(alignmentSourcePath_);
    if (sourceIndex < 0) {
        statusText_ = QString::fromUtf8("三点对齐失败：移动模型已被移除");
        utilityMode_ = UtilityMode::None;
        return;
    }
    auto& sourceModel=*models_[static_cast<std::size_t>(sourceIndex)];
    sourceModel.modelTransform = JMEngine::example::multiply(result.transform, sourceModel.modelTransform);
    statusText_ = QString::fromUtf8("三点对齐完成，RMS = %1（模型坐标单位）").arg(result.rmsError,0,'g',6);
    utilityMode_ = UtilityMode::None;
    utilityPoints_.clear();
    alignmentSourcePath_.clear();
    alignmentSourceCount_=0;
    update();
}

JMEngine::Mat4f PointCloudWidget::currentMvp() const {
    return camera_.mvp(std::max(1, int(width() * devicePixelRatioF())),
                       std::max(1, int(height() * devicePixelRatioF())));
}

JMEngine::Mat4f PointCloudWidget::modelMvp(const Model& model) const {
    return JMEngine::example::multiply(currentMvp(), model.modelTransform);
}

void PointCloudWidget::moveActiveModelByPixels(float dxPixels, float dyPixels) {
    auto* model = activeModel();
    if (!model || !model->cloud) {
        return;
    }

    // 在当前相机视平面上移动。每像素对应的世界尺寸取当前相机距离处的透视尺度，
    // 因而无论窗口分辨率如何，拖动速度都保持稳定。
    const float viewportHeight = static_cast<float>(std::max(1, height()));
    const float worldPerPixel = 2.0f * camera_.distance * std::tan(camera_.fovYRadians * 0.5f) / viewportHeight;
    const auto right = camera_.right();
    const auto up = camera_.up();
    const auto delta = JMEngine::example::add(JMEngine::example::mul(right, dxPixels * worldPerPixel),
                                              JMEngine::example::mul(up, -dyPixels * worldPerPixel));

    model->modelTransform.m[12] += delta.x;
    model->modelTransform.m[13] += delta.y;
    model->modelTransform.m[14] += delta.z;
    update();
}

void PointCloudWidget::fitView() {
    if (auto* model = activeModel(); model && model->cloud) {
        camera_.fit(*model->cloud);
        camera_.target = JMEngine::transformPoint(model->modelTransform, camera_.target);
    }
    update();
}
void PointCloudWidget::updateOrbitPivotForActiveModel() {
    auto* model = activeModel();
    if (!model || !model->cloud || model->cloud->empty()) return;
    bool valid=false;
    JMEngine::Vec3f mn{},mx{};
    auto consume=[&](const JMEngine::Point& p){
        if(p.flags&JMEngine::PointDeleted) return;
        const auto w=JMEngine::transformPoint(model->modelTransform,p.position);
        if(!valid){mn=mx=w;valid=true;return;}
        mn.x=std::min(mn.x,w.x);mn.y=std::min(mn.y,w.y);mn.z=std::min(mn.z,w.z);
        mx.x=std::max(mx.x,w.x);mx.y=std::max(mx.y,w.y);mx.z=std::max(mx.z,w.z);
    };
    if(model->meshMode && model->mesh){
        const auto& idx=model->mesh->indices();
        std::unordered_set<std::uint32_t> used;
        used.reserve(model->mesh->activeTriangleCount()*2u);
        for(std::size_t i=0;i+2<idx.size();i+=3){
            if(!model->mesh->triangleActive(static_cast<JMEngine::TriangleId>(i/3u))) continue;
            used.insert(idx[i]);used.insert(idx[i+1]);used.insert(idx[i+2]);
        }
        for(auto id:used) if(id<model->cloud->size()) consume(model->cloud->points()[id]);
    } else for(const auto&p:model->cloud->points()) consume(p);
    if(!valid) return;
    camera_.target={(mn.x+mx.x)*0.5f,(mn.y+mx.y)*0.5f,(mn.z+mx.z)*0.5f};
    const auto d=sub3(mx,mn);
    camera_.sceneRadius=std::max(0.5f,0.5f*std::sqrt(dot3(d,d)));
}

bool PointCloudWidget::fitBasePlaneFromSelection(QString* message) {
    auto* m=activeModel();
    if(!m || !m->cloud){if(message)*message=QString::fromUtf8("没有激活模型");return false;}
    std::vector<JMEngine::Vec3f> pts;
    if(m->meshMode && m->mesh && !m->selectedTriangleIds.empty()){
        std::unordered_set<std::uint32_t> ids;
        const auto& idx=m->mesh->indices();
        for(auto tid:m->selectedTriangleIds){const std::size_t b=std::size_t(tid)*3u;if(b+2>=idx.size()||!m->mesh->triangleActive(tid))continue;ids.insert(idx[b]);ids.insert(idx[b+1]);ids.insert(idx[b+2]);}
        pts.reserve(ids.size());
        for(auto id:ids) if(id<m->cloud->size() && !(m->cloud->points()[id].flags&JMEngine::PointDeleted)) pts.push_back(m->cloud->points()[id].position);
    } else {
        pts.reserve(m->selectedIds.size());
        for(auto id:m->selectedIds) if(id<m->cloud->size() && !(m->cloud->points()[id].flags&JMEngine::PointDeleted)) pts.push_back(m->cloud->points()[id].position);
    }
    if(pts.size()<3){if(message)*message=QString::fromUtf8("请先框选/套索选择一块基底平面（至少 3 个点或三角形）");return false;}
    JMEngine::Vec3f c,n;
    if(!fitPlanePca(pts,c,n)){if(message)*message=QString::fromUtf8("平面拟合失败");return false;}
    // 将法向指向模型主体一侧：模型中心在平面哪一侧，就把那一侧定义为“保留侧”。
    JMEngine::Vec3f modelCenter=c; bool bv=false; JMEngine::Vec3f mn{},mx{};
    for(const auto&p:m->cloud->points()) if(!(p.flags&JMEngine::PointDeleted)) {if(!bv){mn=mx=p.position;bv=true;}else{mn.x=std::min(mn.x,p.position.x);mn.y=std::min(mn.y,p.position.y);mn.z=std::min(mn.z,p.position.z);mx.x=std::max(mx.x,p.position.x);mx.y=std::max(mx.y,p.position.y);mx.z=std::max(mx.z,p.position.z);}}
    if(bv) modelCenter={(mn.x+mx.x)*.5f,(mn.y+mx.y)*.5f,(mn.z+mx.z)*.5f};
    if(dot3(n,sub3(modelCenter,c))<0.0f)n=mul3(n,-1.0f);
    float radius=0.0f;for(const auto&p:pts){const auto q=sub3(p,c);const auto tang=sub3(q,mul3(n,dot3(q,n)));radius=std::max(radius,std::sqrt(dot3(tang,tang)));}
    basePlane_.active=true;basePlane_.dragging=false;basePlane_.point=c;basePlane_.normal=n;basePlane_.offset=0.0f;basePlane_.visualRadius=std::max(radius*1.25f,camera_.sceneRadius*.08f);
    statusText_=QString::fromUtf8("基底平面已拟合：拖动平面中心圆点可沿法向上下微调；应用后删除平面以下内容");
    if(message)*message=statusText_;
    update();return true;
}

bool PointCloudWidget::applyBasePlaneCut(QString* message) {
    auto* m=activeModel(); if(!m||!m->cloud||!basePlane_.active){if(message)*message=QString::fromUtf8("没有有效的基底拟合平面");return false;}
    const auto planePoint=add3(basePlane_.point,mul3(basePlane_.normal,basePlane_.offset));
    if(m->meshMode && m->mesh){
        std::vector<JMEngine::TriangleId> del; const auto& idx=m->mesh->indices();
        del.reserve(m->mesh->triangleCount()/8u);
        for(std::size_t i=0;i+2<idx.size();i+=3){const auto tid=static_cast<JMEngine::TriangleId>(i/3u);if(!m->mesh->triangleActive(tid))continue;const auto ia=idx[i],ib=idx[i+1],ic=idx[i+2];if(ia>=m->cloud->size()||ib>=m->cloud->size()||ic>=m->cloud->size())continue;const auto&a=m->cloud->points()[ia].position;const auto&b=m->cloud->points()[ib].position;const auto&c=m->cloud->points()[ic].position;const JMEngine::Vec3f center{(a.x+b.x+c.x)/3.0f,(a.y+b.y+c.y)/3.0f,(a.z+b.z+c.z)/3.0f};if(dot3(sub3(center,planePoint),basePlane_.normal)<0.0f)del.push_back(tid);}
        m->meshEditor.select(del); const auto r=m->meshEditor.deleteSelection(); if(r.changed)rebuildVisibleMeshAsync(*m);
        statusText_=QString::fromUtf8("已删除基底平面以下 %1 个三角形").arg(qulonglong(del.size()));
    } else {
        std::vector<JMEngine::PointId> del;del.reserve(m->cloud->size()/8u);for(JMEngine::PointId id=0;std::size_t(id)<m->cloud->size();++id){const auto&p=m->cloud->points()[id];if((p.flags&JMEngine::PointDeleted)==0 && dot3(sub3(p.position,planePoint),basePlane_.normal)<0.0f)del.push_back(id);}m->editor.select(del);m->editor.deleteSelection();const auto changed=m->editor.lastChangedIds();makeCurrent();uploadChangedFlags(*m,changed);doneCurrent();statusText_=QString::fromUtf8("已删除基底平面以下 %1 个点").arg(qulonglong(del.size()));
    }
    basePlane_.active=false;basePlane_.dragging=false;clearSelection();updateOrbitPivotForActiveModel();
    if(message)*message=statusText_;update();return true;
}

void PointCloudWidget::cancelBasePlaneCut(){basePlane_.active=false;basePlane_.dragging=false;statusText_=QString::fromUtf8("已取消基底平面裁剪");update();}


void PointCloudWidget::uploadSelectionMask(Model& m) {
    if (!m.glCreated)
        return;
    glBindBuffer(GL_ARRAY_BUFFER, m.gpu.selectionVbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(m.selectionMask.size()), m.selectionMask.data());
}

void PointCloudWidget::uploadChangedFlags(Model& m, const std::vector<JMEngine::PointId>& ids) {
    if (!m.glCreated || !m.cloud)
        return;
    auto sorted = ids;
    std::sort(sorted.begin(), sorted.end());
    sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
    std::size_t i = 0;
    while (i < sorted.size()) {
        std::size_t j = i + 1;
        while (j < sorted.size() && sorted[j] == sorted[j - 1] + 1u)
            ++j;
        std::vector<std::uint8_t> flags(j - i);
        for (std::size_t k = i; k < j; ++k)
            flags[k - i] = static_cast<std::uint8_t>(m.cloud->points()[sorted[k]].flags & 0xFFu);
        glBindBuffer(GL_ARRAY_BUFFER, m.gpu.flagsVbo);
        glBufferSubData(GL_ARRAY_BUFFER, static_cast<GLintptr>(sorted[i]), static_cast<GLsizeiptr>(flags.size()),
                        flags.data());
        i = j;
    }
}

void PointCloudWidget::clearSelection() {
    auto* m = activeModel();
    if (!m)
        return;
    std::fill(m->selectionMask.begin(), m->selectionMask.end(), 0u);
    m->selectedIds.clear();
    m->selectedTriangleIds.clear();
    m->selectedMeshIndexCount = 0;
    m->editor.clearSelection();
    m->meshEditor.clearSelection();
    makeCurrent();
    uploadSelectionMask(*m);
    doneCurrent();
    update();
}

void PointCloudWidget::applySelection(Model& m, std::vector<JMEngine::PointId> ids, Qt::KeyboardModifiers modifiers) {
    ids = sortedUnique(std::move(ids));
    ids.erase(std::remove_if(ids.begin(), ids.end(),
                             [&](auto id) {
                                 return id >= m.cloud->size() ||
                                        (m.cloud->points()[id].flags & JMEngine::PointDeleted) != 0;
                             }),
              ids.end());
    if (modifiers.testFlag(Qt::ShiftModifier)) {
        auto all = m.selectedIds;
        all.insert(all.end(), ids.begin(), ids.end());
        m.selectedIds = sortedUnique(std::move(all));
    } else if (modifiers.testFlag(Qt::AltModifier)) {
        std::vector<JMEngine::PointId> out;
        std::set_difference(m.selectedIds.begin(), m.selectedIds.end(), ids.begin(), ids.end(),
                            std::back_inserter(out));
        m.selectedIds = std::move(out);
    } else
        m.selectedIds = std::move(ids);
    std::fill(m.selectionMask.begin(), m.selectionMask.end(), 0u);
    for (auto id : m.selectedIds)
        if (id < m.selectionMask.size())
            m.selectionMask[id] = 1u;
    // PointId 选择与 TriangleId 选择互斥，避免显示模式切换后 Delete 读取旧网格选择。
    m.selectedTriangleIds.clear();
    m.selectedMeshIndexCount = 0;
    m.meshEditor.clearSelection();
    m.editor.select(m.selectedIds);
    makeCurrent();
    uploadSelectionMask(m);
    doneCurrent();
    update();
}

void PointCloudWidget::applyTriangleSelection(Model& m, std::vector<JMEngine::TriangleId> ids,
                                              Qt::KeyboardModifiers modifiers) {
    if (!m.mesh || !m.cloud)
        return;
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    ids.erase(std::remove_if(ids.begin(), ids.end(),
                             [&](auto id) {
                                 return static_cast<std::size_t>(id) >= m.mesh->triangleCount() ||
                                        !m.mesh->triangleActive(id);
                             }),
              ids.end());

    if (modifiers.testFlag(Qt::ShiftModifier)) {
        auto all = m.selectedTriangleIds;
        all.insert(all.end(), ids.begin(), ids.end());
        std::sort(all.begin(), all.end());
        all.erase(std::unique(all.begin(), all.end()), all.end());
        m.selectedTriangleIds = std::move(all);
    } else if (modifiers.testFlag(Qt::AltModifier)) {
        std::vector<JMEngine::TriangleId> out;
        std::set_difference(m.selectedTriangleIds.begin(), m.selectedTriangleIds.end(), ids.begin(), ids.end(),
                            std::back_inserter(out));
        m.selectedTriangleIds = std::move(out);
    } else {
        m.selectedTriangleIds = std::move(ids);
    }

    m.meshEditor.select(m.selectedTriangleIds);
    m.editor.clearSelection();

    // Mesh 高亮不再复用 point selectionMask。
    // 一个 Point 可能被多个 Triangle 共享，按顶点高亮会把未选中的相邻面也插值染色。
    // 这里仅构造“选中三角形索引 EBO”，Position/Normal/Color 仍复用原 VBO。
    std::fill(m.selectionMask.begin(), m.selectionMask.end(), 0u);
    m.selectedIds.clear();
    std::vector<std::uint32_t> selectedIndices;
    selectedIndices.reserve(m.selectedTriangleIds.size() * 3u);
    // 注意：selectedTriangleIds 保存的是 TriangleId；下面这份数组只是该 TriangleId
    // 对应的 3 个顶点 EBO 索引，用于绘制高亮，不会写回 Selection。
    const auto& triangleVertexIndices = m.mesh->indices();
    for (auto tid : m.selectedTriangleIds) {
        const std::size_t b = static_cast<std::size_t>(tid) * 3u;
        if (b + 2u >= triangleVertexIndices.size() || !m.mesh->triangleActive(tid))
            continue;
        selectedIndices.push_back(triangleVertexIndices[b]);
        selectedIndices.push_back(triangleVertexIndices[b + 1u]);
        selectedIndices.push_back(triangleVertexIndices[b + 2u]);
    }
    m.selectedMeshIndexCount = static_cast<GLsizei>(selectedIndices.size());
    makeCurrent();
    if (m.gpu.selectedMeshEbo && !selectedIndices.empty()) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.gpu.selectedMeshEbo);
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0,
                        static_cast<GLsizeiptr>(selectedIndices.size() * sizeof(std::uint32_t)),
                        selectedIndices.data());
    }
    // Mesh 模式 point-selection VBO 必须保持全 0。
    uploadSelectionMask(m);
    doneCurrent();
    update();
}

std::vector<JMEngine::PointId> PointCloudWidget::filterPickedPointIds(const Model& m,
                                                                      const std::vector<std::uint32_t>& raw) const {
    std::vector<JMEngine::PointId> out;
    if (!m.cloud)
        return out;
    out.reserve(raw.size());
    for (auto id : raw) {
        if (id != JMEngine::kInvalidPointId && id < m.cloud->size())
            out.push_back(static_cast<JMEngine::PointId>(id));
    }
    return sortedUnique(std::move(out));
}

void PointCloudWidget::performSurfaceSelection(Qt::KeyboardModifiers modifiers) {
    auto* m = activeModel();
    if (!m || !m->glCreated || !backend_)
        return;

    // 显示什么就编辑什么：点显示使用 PointId；网格显示使用 TriangleId。
    const bool triangleIds = m->meshMode && m->mesh && m->displayMode != DisplayMode::Points;

    // Surface 与 Through 必须保持严格语义。
    // GPU 不可用或用户明确选择 CPU 时，只能回退到 CPU Surface，绝不能调用 Through。
    auto performCpuSurfaceFallback = [&]() {
        const auto mvp = modelMvp(*m);
        const JMEngine::Viewport vp{width(), height()};

        if (triangleIds && m->meshMode && m->mesh) {
            std::vector<JMEngine::TriangleId> tids;
            if (interactionMode_ == InteractionMode::Rectangle) {
                tids = JMEngine::CpuMeshSelector::surfaceRectangle(
                    *m->mesh, mvp, vp, {pressPos_.x(), pressPos_.y(), currentPos_.x(), currentPos_.y()});
            } else if (interactionMode_ == InteractionMode::Circle) {
                const int radius = int(std::hypot(currentPos_.x() - pressPos_.x(), currentPos_.y() - pressPos_.y()));
                tids =
                    JMEngine::CpuMeshSelector::surfaceCircle(*m->mesh, mvp, vp, {pressPos_.x(), pressPos_.y()}, radius);
            } else {
                std::vector<JMEngine::Point2i> path;
                path.reserve(stroke_.size());
                for (const auto& p : stroke_)
                    path.push_back({p.x(), p.y()});
                tids = (interactionMode_ == InteractionMode::Lasso)
                           ? JMEngine::CpuMeshSelector::surfaceLasso(*m->mesh, mvp, vp, path)
                           : JMEngine::CpuMeshSelector::surfaceBrushStroke(*m->mesh, mvp, vp, path, brushRadiusPixels_);
            }
            applyTriangleSelection(*m, std::move(tids), modifiers);
            return;
        }

        // CPU Surface 工业路径：空间 Block 粗筛 + 局部软件 Z + 自适应深度坡度容差。
        // 不再使用固定 NDC tolerance，也不再每次扫描整云两遍。
        const QRect bounds = gestureBounds(interactionMode_, pressPos_, currentPos_, stroke_, brushRadiusPixels_);
        auto candidates = pointPickCandidates(*m, mvp, bounds);
        auto ids = selectCandidatePointsCpuSurface(*m->cloud, candidates, mvp, width(), height(), bounds,
                                                   interactionMode_, pressPos_, currentPos_, stroke_,
                                                   brushRadiusPixels_);
        applySelection(*m, std::move(ids), modifiers);
    };

    if (pickingMode_ == PickingMode::Cpu || !pickingReady_ || !backend_->gpuPickingSupported()) {
        performCpuSurfaceFallback();
        return;
    }

    if ((!triangleIds && m->drawPointCount <= 0) || (triangleIds && m->drawIndexCount <= 0))
        return;

    makeCurrent();
    if (!createPickingFramebuffer(std::max(1, int(width() * devicePixelRatioF())),
                                  std::max(1, int(height() * devicePixelRatioF())))) {
        doneCurrent();
        performCpuSurfaceFallback();
        return;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, pickFbo_);
    glViewport(0, 0, pickWidth_, pickHeight_);
    glDisable(GL_BLEND);
    glDisable(GL_DITHER);
#ifdef GL_MULTISAMPLE
    glDisable(GL_MULTISAMPLE);
#endif
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);

    // 点云 Surface 只需要手势包围盒内的深度。启用 scissor 后 GPU 仍会执行顶点变换，
    // 但清屏和 point-sprite raster/depth 写入只发生在小区域，能显著降低高 DPI + 千万点的 fragment 压力。
    const qreal pointDpr = devicePixelRatioF();
    const QRect pointLogicalBounds = gestureBounds(interactionMode_, pressPos_, currentPos_, stroke_, brushRadiusPixels_);
    int pointReadX = 0, pointReadY = 0, pointReadW = pickWidth_, pointReadH = pickHeight_;
    if (!triangleIds) {
        pointReadX = std::clamp(int(std::floor(pointLogicalBounds.left() * pointDpr)), 0, pickWidth_ - 1);
        const int yTop = std::clamp(int(std::floor(pointLogicalBounds.top() * pointDpr)), 0, pickHeight_ - 1);
        const int right = std::clamp(int(std::ceil((pointLogicalBounds.right() + 1) * pointDpr)), pointReadX + 1, pickWidth_);
        const int bottom = std::clamp(int(std::ceil((pointLogicalBounds.bottom() + 1) * pointDpr)), yTop + 1, pickHeight_);
        pointReadW = std::max(1, right - pointReadX);
        pointReadH = std::max(1, bottom - yTop);
        pointReadY = std::max(0, pickHeight_ - (yTop + pointReadH));
        glEnable(GL_SCISSOR_TEST);
        glScissor(pointReadX, pointReadY, pointReadW, pointReadH);
    }

    // Surface Picking：
    // - 点云：GPU depth prepass + Block candidate 精筛；
    // - Mesh：继续使用 R32UI TriangleId 两遍 Picking。
    const GLuint clearId[4] = {0u, 0u, 0u, 0u};
    glClearBufferuiv(GL_COLOR, 0, clearId);
    glClear(GL_DEPTH_BUFFER_BIT);

    const auto mvp = modelMvp(*m);
    const QMatrix4x4 qtMvp(mvp.m.data());

    auto drawPicking = [&](bool depthPrepass) {
        if (triangleIds) {
            meshPickProgram_.bind();
            meshPickProgram_.setUniformValue("uMVP", qtMvp.transposed());
            backend_->bindMeshPickLayout(*this, m->gpu);
            glDrawElements(GL_TRIANGLES, m->drawIndexCount, GL_UNSIGNED_INT, nullptr);
            backend_->unbindLayout(*this);
            meshPickProgram_.release();
        } else {
            pointPickProgram_.bind();
            pointPickProgram_.setUniformValue("uMVP", qtMvp.transposed());
            // 点云 Hybrid Picking 的 depth prepass 与正常显示保持一致的 3px footprint。
            // 最终 PointId 不再来自单像素 R32UI，而由 CPU 对 GPU depth 做精确筛选。
            const float pointSize = 3.0f * float(devicePixelRatioF());
            pointPickProgram_.setUniformValue("uPointSize", pointSize);
            backend_->bindPointPickLayout(*this, m->gpu);
            glDrawArrays(GL_POINTS, 0, m->drawPointCount);
            backend_->unbindLayout(*this);
            pointPickProgram_.release();
        }
    };

    // 第一遍：只写 depth。
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    drawPicking(true);

    // 大点云点选择：GPU 只建立前表面 depth，CPU/OpenMP 遍历可见点并与该 depth 比较。
    // 这样同一像素落入多个前表面点时，可以返回全部 PointId，不再受 R32UI “1 pixel = 1 id” 限制。
    if (!triangleIds) {
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glDisable(GL_SCISSOR_TEST);
        std::vector<float> depthPixels(std::size_t(pointReadW) * std::size_t(pointReadH), 1.0f);
        glReadPixels(pointReadX, pointReadY, pointReadW, pointReadH, GL_DEPTH_COMPONENT, GL_FLOAT, depthPixels.data());
        glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
        doneCurrent();

        // GPU Surface：GPU 负责真实前表面 Z，CPU 只精筛与手势相交的空间块。
        // 大点云下 CPU 工作量从 O(N) 降为 O(candidate points)。
        auto candidates = pointPickCandidates(*m, mvp, pointLogicalBounds);
        auto ids = selectCandidatePointsAgainstGpuDepth(*m->cloud, candidates, mvp, width(), height(), pointDpr, pickWidth_,
                                                        pickHeight_, pointReadX, pointReadY, pointReadW, pointReadH, depthPixels, interactionMode_,
                                                        pressPos_, currentPos_, stroke_, brushRadiusPixels_);
        applySelection(*m, std::move(ids), modifiers);
        return;
    }

    // Mesh 仍使用原 R32UI TriangleId 两遍 Picking。
    // 第二遍：保留 prepass depth，只清 ID；只允许位于前表面的 primitive 写 ID。
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glClearBufferuiv(GL_COLOR, 0, clearId);
    glDepthMask(GL_FALSE);
    glDepthFunc(GL_LEQUAL);
    drawPicking(false);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);

    const qreal dpr = devicePixelRatioF();
    // 每种手势都读取真实包围盒：
    // Circle 是完整圆的外接正方形；Lasso/Brush 使用整条 stroke，而不是 press/current 两点。
    const QRect r = gestureBounds(interactionMode_, pressPos_, currentPos_, stroke_, brushRadiusPixels_);
    const int x = std::clamp(int(r.left() * dpr), 0, pickWidth_ - 1);
    const int yTop = std::clamp(int(r.top() * dpr), 0, pickHeight_ - 1);
    const int rw = std::clamp(int(r.width() * dpr), 1, pickWidth_ - x);
    const int rh = std::clamp(int(r.height() * dpr), 1, pickHeight_ - yTop);
    const int readY = std::max(0, pickHeight_ - (yTop + rh));

    std::vector<std::uint32_t> pixels(std::size_t(rw) * std::size_t(rh), 0u);
    glReadPixels(x, readY, rw, rh, GL_RED_INTEGER, GL_UNSIGNED_INT, pixels.data());
    glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());
    doneCurrent();

    std::vector<std::uint32_t> picked;
    picked.reserve(pixels.size() / 8u + 1u);
    for (int py = 0; py < rh; ++py) {
        for (int px = 0; px < rw; ++px) {
            const auto encoded = pixels[std::size_t(py) * std::size_t(rw) + std::size_t(px)];
            if (encoded == 0u)
                continue;
            const std::uint32_t id = encoded - 1u;
            const int logicalX = int((x + px) / dpr);
            const int logicalY = int((yTop + (rh - 1 - py)) / dpr);
            const QPoint lp(logicalX, logicalY);
            if (gestureContains(interactionMode_, lp, pressPos_, currentPos_, stroke_, brushRadiusPixels_))
                picked.push_back(id);
        }
    }

    picked = sortedUnique(std::move(picked));
    if (triangleIds && m->meshMode && m->mesh) {
        std::vector<JMEngine::TriangleId> tids;
        tids.reserve(picked.size());
        for (auto id : picked) {
            if (id == JMEngine::kInvalidPointId)
                continue;
            if (m->meshFiltered) {
                if (static_cast<std::size_t>(id) < m->visibleTriangleIds.size())
                    tids.push_back(m->visibleTriangleIds[static_cast<std::size_t>(id)]);
            } else if (static_cast<std::size_t>(id) < m->mesh->triangleCount()) {
                tids.push_back(static_cast<JMEngine::TriangleId>(id));
            }
        }
        // 2.0.5：远距离时亚像素三角形可能没有写入 Picking FBO。
        // GPU 返回的 tids 只作为“真正可见的种子”；随后仅在当前手势区域内，
        // 沿共享边做有限拓扑闭合，把同一表面上漏掉的零碎 TriangleId 补齐。
        // 注意：候选集合仍是 TriangleId，绝不退回按共享 PointId 删除。
        if (!tids.empty()) {
            const JMEngine::Viewport vp{width(), height()};
            std::vector<JMEngine::TriangleId> candidates;
            if (interactionMode_ == InteractionMode::Rectangle) {
                candidates = JMEngine::CpuMeshSelector::rectangle(
                    *m->mesh, mvp, vp, {pressPos_.x(), pressPos_.y(), currentPos_.x(), currentPos_.y()});
            } else if (interactionMode_ == InteractionMode::Circle) {
                const int radius = int(std::hypot(currentPos_.x() - pressPos_.x(), currentPos_.y() - pressPos_.y()));
                candidates =
                    JMEngine::CpuMeshSelector::circle(*m->mesh, mvp, vp, {pressPos_.x(), pressPos_.y()}, radius);
            } else {
                std::vector<JMEngine::Point2i> path;
                path.reserve(stroke_.size());
                for (const auto& p : stroke_)
                    path.push_back({p.x(), p.y()});
                candidates = (interactionMode_ == InteractionMode::Lasso)
                                 ? JMEngine::CpuMeshSelector::lasso(*m->mesh, mvp, vp, path)
                                 : JMEngine::CpuMeshSelector::brushStroke(*m->mesh, mvp, vp, path, brushRadiusPixels_);
            }

            JMEngine::MeshSelectionClosureOptions closureOptions;
            closureOptions.maxRings = 16;
            closureOptions.minAdjacentNormalDot = 0.30f;
            tids = JMEngine::MeshSelectionClosure::expandSurfaceSelection(*m->mesh, tids, candidates, closureOptions);
        }
        applyTriangleSelection(*m, std::move(tids), modifiers);
    } else {
        applySelection(*m, filterPickedPointIds(*m, picked), modifiers);
    }
}

void PointCloudWidget::performThroughSelection(Qt::KeyboardModifiers modifiers) {
    auto* m = activeModel();
    if (!m || !m->cloud)
        return;
    const auto mvp = modelMvp(*m);
    const JMEngine::Viewport vp{width(), height()};

    // Mesh 模式的 CPU fallback 仍然保持 TriangleId 语义。
    // Core 使用屏幕空间“三角形与选择区域相交”测试，不再用命中顶点后扩散到相邻面的旧逻辑。
    if (m->meshMode && m->mesh && m->displayMode != DisplayMode::Points) {
        std::vector<JMEngine::TriangleId> tids;
        if (interactionMode_ == InteractionMode::Rectangle) {
            tids = JMEngine::CpuMeshSelector::rectangle(
                *m->mesh, mvp, vp, {pressPos_.x(), pressPos_.y(), currentPos_.x(), currentPos_.y()});
        } else if (interactionMode_ == InteractionMode::Circle) {
            const int r = int(std::hypot(currentPos_.x() - pressPos_.x(), currentPos_.y() - pressPos_.y()));
            tids = JMEngine::CpuMeshSelector::circle(*m->mesh, mvp, vp, {pressPos_.x(), pressPos_.y()}, r);
        } else {
            std::vector<JMEngine::Point2i> path;
            path.reserve(stroke_.size());
            for (const auto& p : stroke_)
                path.push_back({p.x(), p.y()});
            tids = (interactionMode_ == InteractionMode::Lasso)
                       ? JMEngine::CpuMeshSelector::lasso(*m->mesh, mvp, vp, path)
                       : JMEngine::CpuMeshSelector::brushStroke(*m->mesh, mvp, vp, path, brushRadiusPixels_);
        }
        applyTriangleSelection(*m, std::move(tids), modifiers);
        return;
    }

    // Through 的语义是“手势投影范围内全部点”，完全不做深度测试。
    // CPU/GPU 菜单在 Through 下共享同一个 Block accelerated exact path，避免 R32UI 一像素一 ID 的先天漏选。
    const QRect bounds = gestureBounds(interactionMode_, pressPos_, currentPos_, stroke_, brushRadiusPixels_);
    auto candidates = pointPickCandidates(*m, mvp, bounds);
    auto ids = selectCandidatePointsThrough(*m->cloud, candidates, mvp, width(), height(), interactionMode_, pressPos_,
                                            currentPos_, stroke_, brushRadiusPixels_);
    applySelection(*m, std::move(ids), modifiers);
}

void PointCloudWidget::beginEditGesture(const QPoint& pos, Qt::KeyboardModifiers) {
    editGestureActive_ = true;
    pressPos_ = currentPos_ = pos;
    stroke_.clear();
    stroke_.push_back(pos);
    update();
}
void PointCloudWidget::updateEditGesture(const QPoint& pos) {
    if (!editGestureActive_)
        return;
    currentPos_ = pos;
    if (interactionMode_ == InteractionMode::Lasso || interactionMode_ == InteractionMode::Brush)
        stroke_.push_back(pos);
    update();
}
void PointCloudWidget::finishEditGesture(const QPoint& pos, Qt::KeyboardModifiers modifiers) {
    if (!editGestureActive_)
        return;
    updateEditGesture(pos);
    editGestureActive_ = false;
    if (selectionDepthMode_ == SelectionDepthMode::Surface)
        performSurfaceSelection(modifiers);
    else
        performThroughSelection(modifiers);
    stroke_.clear();
    update();
}
void PointCloudWidget::cancelEditGesture() {
    editGestureActive_ = false;
    stroke_.clear();
    update();
}

void PointCloudWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button()==Qt::LeftButton && basePlane_.active) {
        if (const auto* m=activeModel()) {
            const auto p0=add3(basePlane_.point,mul3(basePlane_.normal,basePlane_.offset));
            float x=0,y=0,d=0;
            if(projectPointToScreen(p0,modelMvp(*m),width(),height(),x,y,d)) {
                const QPointF q=event->position();
                if(QLineF(q,QPointF(x,y)).length()<=24.0) {
                    basePlane_.dragging=true; basePlane_.dragLast=q.toPoint(); setCursor(Qt::SizeVerCursor); return;
                }
            }
        }
    }

    if (event->button() == Qt::LeftButton && utilityMode_ != UtilityMode::None && utilityMode_ != UtilityMode::AutoAlign) {
        handleUtilityClick(event->position().toPoint());
        return;
    }
    const bool controlDown = event->modifiers().testFlag(Qt::ControlModifier);
    const bool altDown = event->modifiers().testFlag(Qt::AltModifier);

    // Ctrl 的编辑选择优先级最高。Ctrl+Alt 仍可沿用 Alt=减选语义。
    if (event->button() == Qt::LeftButton && controlDown) {
        beginEditGesture(event->position().toPoint(), event->modifiers());
        return;
    }

    // Alt+左键是临时对象移动；开启“对象移动模式”后无需按 Alt。
    // 对象移动优先于 touchEditMode_，避免两个菜单模式同时开启时无法移动。
    if (event->button() == Qt::LeftButton && (altDown || objectMoveMode_) && activeModel()) {
        objectDragging_ = true;
        lastObjectDragPos_ = event->position().toPoint();
        setCursor(Qt::ClosedHandCursor);
        return;
    }

    if (event->button() == Qt::LeftButton && touchEditMode_) {
        beginEditGesture(event->position().toPoint(), event->modifiers());
        return;
    }

    viewDragging_ = true;
    viewButton_ = event->button();
    lastViewPos_ = event->position().toPoint();
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent* event) {
    if(basePlane_.dragging){
        const QPoint p=event->position().toPoint(); const int dy=p.y()-basePlane_.dragLast.y(); basePlane_.dragLast=p;
        const float worldPerPixel=2.0f*camera_.distance*std::tan(camera_.fovYRadians*0.5f)/float(std::max(1,height()));
        basePlane_.offset += -float(dy)*worldPerPixel;
        statusText_=QString::fromUtf8("基底平面偏移：%1（模型单位）").arg(basePlane_.offset,0,'f',4); update(); return;
    }
    if (editGestureActive_) {
        updateEditGesture(event->position().toPoint());
        return;
    }
    if (objectDragging_) {
        const QPoint current = event->position().toPoint();
        const QPoint delta = current - lastObjectDragPos_;
        lastObjectDragPos_ = current;
        moveActiveModelByPixels(static_cast<float>(delta.x()), static_cast<float>(delta.y()));
        return;
    }
    if (!viewDragging_) {
        return;
    }

    const QPoint current = event->position().toPoint();
    const QPoint delta = current - lastViewPos_;
    lastViewPos_ = current;
    if (viewButton_ == Qt::LeftButton) {
        camera_.orbit(static_cast<float>(delta.x()), static_cast<float>(delta.y()), width(), height());
    } else {
        camera_.pan(static_cast<float>(delta.x()), static_cast<float>(delta.y()), width(), height());
    }
    update();
}

void PointCloudWidget::mouseReleaseEvent(QMouseEvent* event) {
    if(basePlane_.dragging){basePlane_.dragging=false;unsetCursor();update();return;}
    if (editGestureActive_) {
        finishEditGesture(event->position().toPoint(), event->modifiers());
        return;
    }
    if (objectDragging_) {
        objectDragging_ = false;
        unsetCursor();
        return;
    }
    viewDragging_ = false;
    viewButton_ = Qt::NoButton;
}
void PointCloudWidget::wheelEvent(QWheelEvent* e) {
    camera_.zoom(float(e->angleDelta().y()) / 120.0f);
    update();
}

bool PointCloudWidget::event(QEvent* event) {
    switch (event->type()) {
    case QEvent::TouchBegin:
    case QEvent::TouchUpdate:
    case QEvent::TouchEnd:
    case QEvent::TouchCancel:
        return handleTouchEvent(static_cast<QTouchEvent*>(event));
    default:
        break;
    }
    return QOpenGLWidget::event(event);
}

bool PointCloudWidget::handleTouchEvent(QTouchEvent* e) {
    const auto pts = e->points();
    if (e->type() == QEvent::TouchCancel) {
        twoFingerActive_ = false;
        cancelEditGesture();
        return true;
    }
    if (pts.size() >= 2) {
        // 两指始终优先作为浏览手势：即使“编辑模式”开启，也能随时缩放/平移模型。
        if (editGestureActive_)
            cancelEditGesture();
        const QPointF p0 = pts[0].position(), p1 = pts[1].position();
        const QPointF center = (p0 + p1) * 0.5;
        const qreal dist = QLineF(p0, p1).length();
        if (!twoFingerActive_) {
            twoFingerActive_ = true;
            lastTouchCenter_ = center;
            lastTouchDistance_ = std::max<qreal>(dist, 1.0);
        } else {
            const QPointF d = center - lastTouchCenter_;
            camera_.pan(float(d.x()), float(d.y()), width(), height());
            if (dist > 1.0 && lastTouchDistance_ > 1.0) {
                const qreal ratio = dist / lastTouchDistance_;
                camera_.zoom(float(std::log(ratio) / std::log(1.18)));
            }
            lastTouchCenter_ = center;
            lastTouchDistance_ = dist;
        }
        update();
        if (e->type() == QEvent::TouchEnd)
            twoFingerActive_ = false;
        return true;
    }
    twoFingerActive_ = false;
    if (pts.size() == 1) {
        const QPoint p = pts[0].position().toPoint();
        if (e->type() == QEvent::TouchBegin) {
            if (touchEditMode_)
                beginEditGesture(p);
            else {
                viewDragging_ = true;
                viewButton_ = Qt::LeftButton;
                lastViewPos_ = p;
            }
        } else if (e->type() == QEvent::TouchUpdate) {
            if (editGestureActive_)
                updateEditGesture(p);
            else if (viewDragging_) {
                const QPoint d = p - lastViewPos_;
                lastViewPos_ = p;
                camera_.orbit(float(d.x()), float(d.y()), width(), height());
                update();
            }
        } else if (e->type() == QEvent::TouchEnd) {
            if (editGestureActive_)
                finishEditGesture(p);
            viewDragging_ = false;
        }
        return true;
    }
    if (e->type() == QEvent::TouchEnd) {
        if (editGestureActive_)
            finishEditGesture(currentPos_);
        viewDragging_ = false;
        twoFingerActive_ = false;
    }
    return true;
}

void PointCloudWidget::deleteSelection() {
    auto* m = activeModel();
    if (!m || editBusy_)
        return;

    // Mesh 模式：真正以 TriangleId 为编辑单位，不删除共享 PointId。
    if (m->meshMode && m->mesh && !m->selectedTriangleIds.empty()) {
        m->meshEditor.select(m->selectedTriangleIds);
        const auto result = m->meshEditor.deleteSelection();
        if (!result.changed)
            return;
        m->selectedTriangleIds.clear();
        m->selectedIds.clear();
        m->selectedMeshIndexCount = 0;
        std::fill(m->selectionMask.begin(), m->selectionMask.end(), 0u);
        makeCurrent();
        uploadSelectionMask(*m);
        doneCurrent();
        rebuildVisibleMeshAsync(*m);
        update();
        return;
    }

    if (m->selectedIds.empty())
        return;
    m->editor.select(m->selectedIds);
    if (!m->editor.deleteSelection())
        return;
    const auto changed = m->editor.lastChangedIds();
    std::fill(m->selectionMask.begin(), m->selectionMask.end(), 0u);
    m->selectedIds.clear();
    makeCurrent();
    uploadChangedFlags(*m, changed);
    uploadSelectionMask(*m);
    doneCurrent();
    if (m->meshMode && m->mesh)
        rebuildVisibleMeshAsync(*m);
    update();
}

void PointCloudWidget::keepSelectionOnly() {
    auto* m = activeModel();
    if (!m || editBusy_)
        return;
    if (m->meshMode && m->mesh && !m->selectedTriangleIds.empty()) {
        m->meshEditor.select(m->selectedTriangleIds);
        const auto r = m->meshEditor.keepSelection();
        if (!r.changed)
            return;
        m->selectedTriangleIds.clear();
        m->selectedIds.clear();
        m->selectedMeshIndexCount = 0;
        std::fill(m->selectionMask.begin(), m->selectionMask.end(), 0u);
        makeCurrent();
        uploadSelectionMask(*m);
        doneCurrent();
        rebuildVisibleMeshAsync(*m);
        update();
        return;
    }
    if (m->selectedIds.empty())
        return;
    m->editor.select(m->selectedIds);
    if (!m->editor.keepSelection())
        return;
    const auto changed = m->editor.lastChangedIds();
    m->selectedIds.clear();
    std::fill(m->selectionMask.begin(), m->selectionMask.end(), 0u);
    makeCurrent();
    uploadChangedFlags(*m, changed);
    uploadSelectionMask(*m);
    doneCurrent();
    if (m->meshMode && m->mesh)
        rebuildVisibleMeshAsync(*m);
    update();
}

void PointCloudWidget::invertSelection() {
    auto* m = activeModel();
    if (!m)
        return;
    if (m->meshMode && m->mesh && m->displayMode != DisplayMode::Points) {
        m->meshEditor.select(m->selectedTriangleIds);
        m->meshEditor.invertSelection();
        applyTriangleSelection(*m, m->meshEditor.selection(), {});
        return;
    }
    m->editor.select(m->selectedIds);
    m->editor.invertSelection();
    applySelection(*m, m->editor.selection().ids(), {});
}

void PointCloudWidget::compactActiveModel() {
    auto* m = activeModel();
    if (!m || editBusy_)
        return;
    if (m->meshMode && m->mesh) {
        m->meshEditor.compactTriangles();
        m->selectedTriangleIds.clear();
        m->selectedIds.clear();
        m->selectedMeshIndexCount = 0;
        std::fill(m->selectionMask.begin(), m->selectionMask.end(), 0u);
        // compact 后原始 EBO 已改变，重新走完整 EBO cache 更新。
        rebuildVisibleMeshAsync(*m);
        update();
        return;
    }
    if (!m->cloud)
        return;
    makeCurrent();
    destroyModelGl(*m);
    doneCurrent();
    m->editor.compact();
    m->selectionMask.assign(m->cloud->size(), 0u);
    m->selectedIds.clear();
    m->uploadPointCursor = m->uploadIndexCursor = 0;
    m->drawPointCount = m->drawIndexCount = 0;
    m->glCreated = false;
    update();
}

JMEngine::processing::OperationDescriptor PointCloudWidget::processingDescriptor(const std::string& operationId) const {
    auto operation = JMEngine::processing::createOperation(operationId);
    if (!operation)
        return {};
    JMEngine::processing::ProcessInput input;
    if (const auto* model = activeModel()) {
        input.cloud = model->cloud;
        input.mesh = model->mesh;
    }
    return JMEngine::processing::estimateOperationDescriptor(*operation, input);
}

JMEngine::processing::ModelDiagnostics PointCloudWidget::activeModelDiagnostics() const {
    JMEngine::processing::ProcessInput input;
    if (const auto* model = activeModel()) {
        input.cloud = model->cloud;
        input.mesh = model->meshMode ? model->mesh : nullptr;
    }
    return JMEngine::processing::analyzeModel(input);
}

JMEngine::processing::ProcessingPreflight
PointCloudWidget::processingPreflight(const std::string& operationId,
                                      const JMEngine::processing::ParameterMap& params) const {
    auto operation = JMEngine::processing::createOperation(operationId);
    if (!operation)
        return {};
    JMEngine::processing::ProcessInput input;
    if (const auto* model = activeModel()) {
        input.cloud = model->cloud;
        input.mesh = model->meshMode ? model->mesh : nullptr;
    }
    return JMEngine::processing::preflightOperation(*operation, input, params);
}

bool PointCloudWidget::analyzeActiveModelAsync(DiagnosticsFinishedCallback finished) {
    auto* model = activeModel();
    if (!model || !model->cloud || processingBusy_ || diagnosticsBusy_ || editBusy_) {
        if (finished)
            finished(false, {}, QString::fromUtf8("当前没有可诊断模型，或已有重处理任务正在运行"));
        return false;
    }

    diagnosticsBusy_ = true;
    editBusy_ = true;
    const QString path = model->path;
    const auto cloud = model->cloud;
    const auto mesh = model->meshMode ? model->mesh : nullptr;
    QPointer<PointCloudWidget> self(this);
    workerPool_.start([self, path, cloud, mesh, finished = std::move(finished)]() mutable {
        JMEngine::processing::ProcessInput input;
        input.cloud = cloud;
        input.mesh = mesh;
        const auto diagnostics = JMEngine::processing::analyzeModel(input);
        QMetaObject::invokeMethod(
            self,
            [self, path, diagnostics, finished = std::move(finished)]() mutable {
                if (!self)
                    return;
                self->diagnosticsBusy_ = false;
                self->editBusy_ = false;
                const bool sameModel = self->activeModelPath() == path;
                if (finished)
                    finished(sameModel, diagnostics,
                             sameModel ? QString{} : QString::fromUtf8("诊断期间激活模型已切换，结果未绑定到当前模型"));
            },
            Qt::QueuedConnection);
    });
    return true;
}

PointCloudWidget::ProcessingSnapshot PointCloudWidget::captureProcessingSnapshot(const Model& model) const {
    ProcessingSnapshot snapshot;
    snapshot.path = model.path;
    snapshot.cloud = model.cloud;
    snapshot.mesh = model.mesh;
    snapshot.meshMode = model.meshMode;
    snapshot.modelTransform = model.modelTransform;
    return snapshot;
}

bool PointCloudWidget::restoreProcessingSnapshot(const ProcessingSnapshot& snapshot) {
    const int index = findModel(snapshot.path);
    if (index < 0)
        return false;
    auto& model = *models_[static_cast<std::size_t>(index)];

    makeCurrent();
    destroyModelGl(model);
    doneCurrent();

    model.cloud = snapshot.cloud;
    model.mesh = snapshot.mesh;
    model.meshMode = snapshot.meshMode;
    model.modelTransform = snapshot.modelTransform;
    model.editor.setPointCloud(model.cloud);
    model.meshEditor.setMesh(model.meshMode ? model.mesh : nullptr);
    model.selectionMask.assign(model.cloud ? model.cloud->size() : 0u, 0u);
    model.selectedIds.clear();
    model.selectedTriangleIds.clear();
    model.selectedMeshIndexCount = 0;
    model.pickGridIds.clear();
    model.pickBlocks.clear();
    model.pickIndexedCloud = nullptr;
    model.pickIndexedPointCount = 0;
    model.visibleMeshIndices.clear();
    model.visibleTriangleIds.clear();
    model.meshFiltered = false;
    model.uploadPointCursor = 0;
    model.uploadIndexCursor = 0;
    model.drawPointCount = 0;
    model.drawIndexCount = 0;
    model.meshUploadComplete = false;
    model.glCreated = false;
    activeModelIndex_ = index;
    update();
    return true;
}

void PointCloudWidget::undoEdit() {
    auto* m = activeModel();
    if (!m || editBusy_ || processingBusy_)
        return;
    if (m->meshMode && m->mesh && m->meshEditor.canUndo()) {
        const auto r = m->meshEditor.undo();
        if (r.changed) {
            m->selectedTriangleIds.clear();
            m->selectedMeshIndexCount = 0;
            m->meshEditor.clearSelection();
            rebuildVisibleMeshAsync(*m);
            update();
        }
        return;
    }
    if (m->editor.undo()) {
        const auto changed = m->editor.lastChangedIds();
        makeCurrent();
        uploadChangedFlags(*m, changed);
        doneCurrent();
        update();
        return;
    }
    if (!m->processingUndo.empty()) {
        const auto previous = m->processingUndo.back();
        m->processingUndo.pop_back();
        m->processingRedo.push_back(captureProcessingSnapshot(*m));
        if (m->processingRedo.size() > 4u)
            m->processingRedo.erase(m->processingRedo.begin());
        if (restoreProcessingSnapshot(previous))
            statusText_ = QString::fromUtf8("已撤销处理操作");
    }
}

void PointCloudWidget::redoEdit() {
    auto* m = activeModel();
    if (!m || editBusy_ || processingBusy_)
        return;
    if (m->meshMode && m->mesh && m->meshEditor.canRedo()) {
        const auto r = m->meshEditor.redo();
        if (r.changed) {
            m->selectedTriangleIds.clear();
            m->selectedMeshIndexCount = 0;
            m->meshEditor.clearSelection();
            rebuildVisibleMeshAsync(*m);
            update();
        }
        return;
    }
    if (m->editor.redo()) {
        const auto changed = m->editor.lastChangedIds();
        makeCurrent();
        uploadChangedFlags(*m, changed);
        doneCurrent();
        update();
        return;
    }
    if (!m->processingRedo.empty()) {
        const auto next = m->processingRedo.back();
        m->processingRedo.pop_back();
        m->processingUndo.push_back(captureProcessingSnapshot(*m));
        if (m->processingUndo.size() > 4u)
            m->processingUndo.erase(m->processingUndo.begin());
        if (restoreProcessingSnapshot(next))
            statusText_ = QString::fromUtf8("已重做处理操作");
    }
}

void PointCloudWidget::rebuildVisibleMeshAsync(Model& model) {
    if (!model.meshMode || !model.mesh || editBusy_)
        return;
    editBusy_ = true;
    const QString path = model.path;
    auto mesh = model.mesh;
    QPointer<PointCloudWidget> self(this);
    workerPool_.start([self, path, mesh] {
        // 可见 EBO + packed primitive -> original TriangleId 映射都由 Core 生成。
        auto visible = mesh->buildVisibleBuffer();
        QMetaObject::invokeMethod(
            self,
            [self, path, visible = std::move(visible)]() mutable {
                if (!self)
                    return;
                int i = self->findModel(path);
                if (i < 0) {
                    self->editBusy_ = false;
                    return;
                }
                auto& m = *self->models_[std::size_t(i)];

                // 2.0.4：Visible EBO 与 primitive->TriangleId 映射作为同一个 active snapshot 提交。
                // 先准备 CPU pending 数据；在 GPU meshEbo 更新完成前，绝不切换 active mapping。
                auto pendingIndices = std::move(visible.indices);
                auto pendingTriangleIds = std::move(visible.triangleIds);

                self->makeCurrent();
                // 与 1.9.0 的 VAO/EBO 路径一致：绑定正常 Render VAO 后更新它持有的 meshEbo。
                // Picking 与正常渲染复用这个 VAO/EBO，因此 primitiveId 与 active EBO 严格一致。
                self->backend_->bindRenderLayout(*self, m.gpu);
                if (!pendingIndices.empty()) {
                    self->glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0,
                                          static_cast<GLsizeiptr>(pendingIndices.size() * sizeof(std::uint32_t)),
                                          pendingIndices.data());
                }
                self->backend_->unbindLayout(*self);
                self->doneCurrent();

                // GPU EBO 提交完成后再原子切换 CPU 侧 active snapshot。
                m.visibleMeshIndices = std::move(pendingIndices);
                m.visibleTriangleIds = std::move(pendingTriangleIds);
                m.meshFiltered = true;
                m.meshUploadComplete = true;
                m.uploadIndexCursor = m.visibleMeshIndices.size();
                m.drawIndexCount = static_cast<GLsizei>(m.visibleMeshIndices.size());
                m.wireDirty = true;
                self->editBusy_ = false;
                self->update();
            },
            Qt::QueuedConnection);
    });
}

void PointCloudWidget::saveActiveModel() {
    auto* m = activeModel();
    if (!m || !m->cloud)
        return;
    std::filesystem::path p(m->path.toStdString());
    p.replace_filename(p.stem().string() + "_edited.ply");
    const QString out = QString::fromStdString(p.string());
    exportActiveModelAsync(out, [this](bool, const QString& message) {
        statusText_ = message;
        update();
    });
}

bool PointCloudWidget::exportActiveModel(const QString& path, QString* message) {
    auto* m = activeModel();
    if (!m || !m->cloud) {
        if (message) *message = QString::fromUtf8("当前没有可导出的模型");
        return false;
    }
    auto snapshot = makeExportSnapshot(m->cloud, m->mesh, m->meshMode
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
                                       , m->textureResult
#endif
    );
    std::string msg;
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    if (snapshot.textureResult && QFileInfo(path).suffix().compare(QStringLiteral("obj"), Qt::CaseInsensitive) == 0) {
        const bool ok = JMEngine::texture::saveObj(*snapshot.textureResult, path.toStdString(), &msg, snapshot.mesh.get());
        if (message) *message = ok ? QString::fromUtf8("已导出当前渲染纹理模型：") + path
                                   : QString::fromUtf8("纹理模型导出失败：") + QString::fromUtf8(msg.c_str());
        return ok;
    }
#endif
    auto rendered = makeRenderedExportSnapshot(snapshot);
    const bool ok = rendered.cloud && JMEngine::ModelIO::save(*rendered.cloud, rendered.meshMode ? rendered.mesh.get() : nullptr,
                                                               path.toStdString(), &msg);
    if (message) *message = ok ? QString::fromUtf8("已导出当前渲染数据：") + path
                               : QString::fromUtf8("导出失败：") + QString::fromUtf8(msg.c_str());
    return ok;
}

bool PointCloudWidget::exportActiveModelAsync(const QString& path,
                                               std::function<void(bool, const QString&)> finished) {
    auto* m = activeModel();
    if (!m || !m->cloud) {
        if (finished) finished(false, QString::fromUtf8("当前没有可导出的模型"));
        return false;
    }
    if (exportBusy_) {
        if (finished) finished(false, QString::fromUtf8("已有导出任务正在运行"));
        return false;
    }
    // Never snapshot a model while a Poisson/texture/diagnostics job can still replace or edit
    // its arrays.  This also makes the snapshot represent one well-defined scene state.
    if (processingBusy_ || diagnosticsBusy_ || editBusy_) {
        if (finished) finished(false, QString::fromUtf8("模型正在处理，请处理完成后再导出"));
        return false;
    }

    exportBusy_ = true;
    editBusy_ = true;

    // Capture only owning pointers here.  The worker immediately DEEP-copies them before any
    // exporter is called.  Export code never receives a scene-owned cloud/mesh/result pointer.
    const auto sceneCloud = m->cloud;
    const auto sceneMesh = m->mesh;
    const bool sceneMeshMode = m->meshMode;
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    const auto sceneTextureResult = m->textureResult;
#endif
    const QString outPath = path;
    QPointer<PointCloudWidget> self(this);
    workerPool_.start([self, sceneCloud, sceneMesh, sceneMeshMode,
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
                       sceneTextureResult,
#endif
                       outPath, finished = std::move(finished)]() mutable {
        std::string msg;
        bool ok = false;
        try {
            // IMPORTANT: makeExportSnapshot performs no compact/remap/cleanup.  It preserves
            // vertex IDs, triangle IDs, flags, UV topology and texture result exactly.
            auto snapshot = makeExportSnapshot(sceneCloud, sceneMesh, sceneMeshMode
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
                                               , sceneTextureResult
#endif
            );
            if (!snapshot.cloud) {
                msg = "failed to create export snapshot";
            } else {
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
                if (snapshot.textureResult && outPath.endsWith(QStringLiteral(".obj"), Qt::CaseInsensitive)) {
                    // Textured OBJ keeps the texture-result vertex/UV topology, but saveObj is given
                    // the scene snapshot so it can skip every face hidden by triangle/point deletion.
                    ok = JMEngine::texture::saveObj(*snapshot.textureResult, outPath.toStdString(), &msg,
                                                    snapshot.mesh.get());
                } else
#endif
                {
                    // Non-textured export must match what the renderer shows, not the original backing arrays.
                    // Compact/remap ONLY this private snapshot.
                    auto rendered = makeRenderedExportSnapshot(snapshot);
                    ok = rendered.cloud && JMEngine::ModelIO::save(*rendered.cloud,
                                                 rendered.meshMode ? rendered.mesh.get() : nullptr,
                                                 outPath.toStdString(), &msg);
                }
            }
        } catch (const std::exception& e) {
            msg = std::string("export snapshot failed: ") + e.what();
            ok = false;
        } catch (...) {
            msg = "export snapshot failed: unknown error";
            ok = false;
        }
        const QString text = ok ? QString::fromUtf8("已导出：") + outPath
                                : QString::fromUtf8("导出失败：") + QString::fromUtf8(msg.c_str());
        QMetaObject::invokeMethod(self, [self, ok, text, finished]() mutable {
            if (!self) return;
            self->exportBusy_ = false;
            self->editBusy_ = false;
            self->statusText_ = text;
            self->update();
            if (finished) finished(ok, text);
        }, Qt::QueuedConnection);
    });
    return true;
}

#ifdef JMENGINE_HAS_TEXTURE_MAPPING
void PointCloudWidget::setTextureFrames(TextureFramesPtr frames) {
    textureFrames_ = std::move(frames);
    const std::size_t count = textureFrames_ ? textureFrames_->size() : 0u;
    statusText_ = QString::fromUtf8("纹理关键帧：%1").arg(static_cast<qulonglong>(count));
    update();
}

bool PointCloudWidget::startTextureMappingAsync(JMEngine::texture::Backend backend, ProcessingFinishedCallback finished) {
    auto* m = activeModel();
    if (!m || !m->meshMode || !m->mesh) {
        if (finished) finished(false, QString::fromUtf8("纹理映射需要当前激活模型为三角网格"));
        return false;
    }
    if (!textureFrames_ || textureFrames_->empty()) {
        if (finished) finished(false, QString::fromUtf8("没有纹理关键帧，请先完成一次扫描和离线优化"));
        return false;
    }
    if (processingBusy_ || editBusy_) {
        if (finished) finished(false, QString::fromUtf8("已有处理任务正在运行"));
        return false;
    }

    processingBusy_ = true;
    editBusy_ = true;
    const QString sourcePath = m->path;
    const auto sourceMesh = m->mesh;
    const auto frames = textureFrames_;
    const auto sourceTransform = m->modelTransform;
    QPointer<PointCloudWidget> self(this);
    workerPool_.start([self, sourcePath, sourceMesh, frames, sourceTransform, backend, finished = std::move(finished)]() mutable {
        JMEngine::texture::Config cfg;
        cfg.backend = backend;
        cfg.quality = JMEngine::texture::Quality::OpenMVS;
        cfg.maxKeyframes = static_cast<int>(frames->size());
        cfg.maxAtlasSize = 16384;
        cfg.visibilityWidth = 1280;
        cfg.visibilityHeight = 800;
        cfg.visibilityTolerance = 4.0f;
        cfg.borderMarginRatio = 0.002f;
        cfg.candidateCameraCount = 8;
        cfg.globalViewIterations = 10;
        cfg.patchAtlas = true;
        cfg.patchBorderPixels = 6;
        cfg.patchSmoothness = 0.22f;
        cfg.keepUntexturedFaces = true;
        cfg.rejectBlurredFrames = false; // never hard-drop the only view of a recessed surface
        cfg.buildVisibilityDepth = true;
        cfg.bakePreviewVertexColors = true;
        JMEngine::texture::TextureMapper mapper;
        auto result = mapper.map(*sourceMesh, *frames, cfg);
        auto resultPtr = std::make_shared<JMEngine::texture::Result>(std::move(result));

        QMetaObject::invokeMethod(self, [self, sourcePath, sourceMesh, sourceTransform, resultPtr, finished]() mutable {
            if (!self) return;
            self->processingBusy_ = false;
            self->editBusy_ = false;
            const int sourceIndex = self->findModel(sourcePath);
            if (sourceIndex < 0 || self->models_[static_cast<std::size_t>(sourceIndex)]->mesh != sourceMesh) {
                if (finished) finished(false, QString::fromUtf8("源网格已变化，纹理结果已丢弃"));
                return;
            }
            if (!resultPtr->ok || !resultPtr->vertices || resultPtr->indices.empty()) {
                if (finished) finished(false, QString::fromUtf8(resultPtr->message.c_str()));
                return;
            }

            QFileInfo srcInfo(sourcePath);
            QString base = srcInfo.completeBaseName();
            if (base.isEmpty()) base = QString::fromUtf8("model");
            QString generated = srcInfo.absolutePath() + QLatin1Char('/') + base + QString::fromUtf8("_textured");
            int suffix = 2;
            while (self->findModel(generated) >= 0)
                generated = srcInfo.absolutePath() + QLatin1Char('/') + base + QString::fromUtf8("_textured_%1").arg(suffix++);

            auto mesh = std::make_shared<JMEngine::TriangleMesh>(resultPtr->vertices, resultPtr->indices);
            auto newModel = std::make_unique<Model>(generated, mesh);
            newModel->textureResult = resultPtr;
            newModel->modelTransform = sourceTransform;
            // Current renderer previews mapped imagery through baked per-corner RGB.
            // The full UV + atlas are retained in textureResult and exported as OBJ/MTL/TGA.
            newModel->displayMode = DisplayMode::Solid;
            self->models_.push_back(std::move(newModel));
            self->activeModelIndex_ = static_cast<int>(self->models_.size()) - 1;
            if (self->modelAddedCallback_) self->modelAddedCallback_(generated);
            const QString backendName = resultPtr->backendUsed == JMEngine::texture::Backend::Cuda
                                            ? QStringLiteral("CUDA") : QStringLiteral("CPU");
            self->statusText_ = QString::fromUtf8("纹理映射完成：%1，%2 ms，%3 三角形")
                                    .arg(backendName)
                                    .arg(resultPtr->elapsedMs, 0, 'f', 1)
                                    .arg(static_cast<qulonglong>(resultPtr->triangleCameraIds.size()));
            self->update();
            if (finished)
                finished(true, QString::fromUtf8("纹理映射完成（%1）：%2 ms；导出 OBJ 时会同时生成 MTL/纹理图")
                                   .arg(backendName)
                                   .arg(resultPtr->elapsedMs, 0, 'f', 1));
        }, Qt::QueuedConnection);
    });
    return true;
}


bool PointCloudWidget::startScanTextureMappingAsync(JMEngine::texture::Backend backend,
                                                     ProcessingProgressCallback progress,
                                                     ProcessingFinishedCallback finished) {
    if (!textureFrames_ || textureFrames_->empty()) {
        if (finished)
            finished(false, QString::fromUtf8("没有纹理关键帧，请先点击“离线重建”完成姿态优化和纹理帧准备"));
        return false;
    }

    auto* model = activeModel();
    if (!model || !model->cloud) {
        if (finished)
            finished(false, QString::fromUtf8("当前没有扫描模型"));
        return false;
    }

    // Already a mesh: texture directly.
    if (model->meshMode && model->mesh)
        return startTextureMappingAsync(backend, std::move(finished));

    // A normal scan finishes as a point cloud.  TextureMapper requires triangles, therefore
    // build a surface first.  Use the same data-driven defaults shown by ProcessingDialog so
    // the scan button does not hide a second parameter dialog from the operator.
    auto operation = JMEngine::processing::createOperation("poisson_octree");
    if (!operation) {
        if (finished)
            finished(false, QString::fromUtf8("工业泊松重建模块不可用，无法从扫描点云生成纹理网格"));
        return false;
    }

    JMEngine::processing::ProcessInput input;
    input.cloud = model->cloud;
    input.mesh = model->mesh;
    const auto desc = JMEngine::processing::estimateOperationDescriptor(*operation, input);
    JMEngine::processing::ParameterMap params;
    for (const auto& spec : desc.parameters) {
        switch (spec.kind) {
        case JMEngine::processing::ParameterKind::Integer:
            params[spec.key] = static_cast<std::int64_t>(std::llround(spec.defaultValue));
            break;
        case JMEngine::processing::ParameterKind::Boolean:
            params[spec.key] = spec.defaultValue >= 0.5;
            break;
        case JMEngine::processing::ParameterKind::Real:
            params[spec.key] = spec.defaultValue;
            break;
        case JMEngine::processing::ParameterKind::Choice:
            // Choice defaults are represented by their numeric index in the generic descriptor.
            params[spec.key] = static_cast<std::int64_t>(std::llround(spec.defaultValue));
            break;
        }
    }

    QPointer<PointCloudWidget> self(this);
    return startProcessingOperation(
        "poisson_octree", std::move(params), std::move(progress),
        [self, backend, finished = std::move(finished)](bool ok, const QString& message) mutable {
            if (!self)
                return;
            if (!ok) {
                if (finished)
                    finished(false, QString::fromUtf8("纹理映射前的网格重建失败：") + message);
                return;
            }

            // startProcessingOperation activates the newly-created Poisson mesh before this
            // callback, so mapping can safely start immediately without polling or a timer.
            if (!self->startTextureMappingAsync(backend, std::move(finished))) {
                // startTextureMappingAsync reports the concrete reason through finished().
                return;
            }
        });
}
#endif

bool PointCloudWidget::startProcessingOperation(const std::string& operationId,
                                                JMEngine::processing::ParameterMap params,
                                                ProcessingProgressCallback progress,
                                                ProcessingFinishedCallback finished) {
    auto* m = activeModel();
    if (!m || !m->cloud || processingBusy_ || editBusy_) {
        if (finished)
            finished(false, QString::fromUtf8("当前没有可处理模型，或已有任务正在运行"));
        return false;
    }

    auto operation = JMEngine::processing::createOperation(operationId);
    if (!operation) {
        if (finished)
            finished(false, QString::fromUtf8("未知处理算法"));
        return false;
    }
    const auto desc = operation->descriptor();
    if (desc.inputKind == JMEngine::processing::ModelKind::TriangleMesh && (!m->meshMode || !m->mesh)) {
        if (finished)
            finished(false, QString::fromUtf8("该算法需要三角网格模型"));
        return false;
    }
    if (desc.inputKind == JMEngine::processing::ModelKind::PointCloud && m->meshMode &&
        operationId != "poisson_octree") {
        if (finished)
            finished(false, QString::fromUtf8("该算法当前只对纯点云模型开放"));
        return false;
    }

    processingBusy_ = true;
    editBusy_ = true;
    processingCancel_ = JMEngine::processing::CancelToken{};

    const QString path = m->path;
    // Keep the scene-owned pointers only for stale-result validation on the UI thread.
    // Background geometry processing must NEVER read them directly: live scan / pose
    // optimization can still update the scene cloud even while editBusy_ is set.  Reading
    // the same PointCloud from Poisson while another thread appends/replaces points is a
    // data race and can produce a collapsed / tangled reconstruction.
    const auto sourceCloud = m->cloud;
    const auto sourceMesh = m->mesh;

    // Take a true immutable processing snapshot BEFORE entering the worker thread.
    // This is intentionally the same deep-copy policy used by export: no compact, remap,
    // normal rebuild, cleanup, transform or topology conversion is allowed here.
    const auto processingCloud = deepCopyPointCloud(sourceCloud);
    const auto processingMesh = deepCopyTriangleMesh(sourceMesh, processingCloud);
    if (!processingCloud || (sourceMesh && !processingMesh)) {
        processingBusy_ = false;
        editBusy_ = false;
        if (finished)
            finished(false, QString::fromUtf8("无法创建稳定的处理快照"));
        return false;
    }

    const auto outputPolicy = desc.outputPolicy;
    const auto cancel = processingCancel_;

    // Poisson's "使用输入颜色" writes RGB into the generated mesh vertices. Algorithm-generated
    // meshes used to force a neutral gray display override, so the RGB existed in Core but was
    // never visible. Carry this display intent to the UI-thread result application.
    bool preferResultVertexColors = false;
    if (operationId == "poisson_octree") {
        preferResultVertexColors = true;
        const auto it = params.find("use_input_color");
        if (it != params.end()) {
            if (const auto* v = std::get_if<bool>(&it->second)) preferResultVertexColors = *v;
            else if (const auto* v = std::get_if<std::int64_t>(&it->second)) preferResultVertexColors = *v != 0;
            else if (const auto* v = std::get_if<double>(&it->second)) preferResultVertexColors = *v != 0.0;
        }
    }
    QPointer<PointCloudWidget> self(this);

    workerPool_.start([self, path, sourceCloud, sourceMesh, processingCloud, processingMesh, operationId, outputPolicy,
                       preferResultVertexColors, params = std::move(params), cancel,
                       progress = std::move(progress), finished = std::move(finished)]() mutable {
        if (!self)
            return;
        auto op = JMEngine::processing::createOperation(operationId);
        if (!op)
            return;

        JMEngine::processing::ProcessInput input;
        // Worker reads only the immutable snapshot.  sourceCloud/sourceMesh are identity
        // tokens used later to reject stale results; they are never processing input.
        input.cloud = processingCloud;
        input.mesh = processingMesh;

        auto result = op->run(
            input, params,
            [self, progress](const JMEngine::processing::ProgressInfo& info) {
                if (!self || !progress)
                    return;
                const QString stage = QString::fromUtf8(info.stage.c_str());
                QMetaObject::invokeMethod(
                    self,
                    [self, progress, value = info.progress, stage] {
                        if (self && progress)
                            progress(value, stage);
                    },
                    Qt::QueuedConnection);
            },
            cancel);

        QMetaObject::invokeMethod(
            self,
            [self, path, sourceCloud, sourceMesh, outputPolicy, preferResultVertexColors, result = std::move(result), finished]() mutable {
                if (!self)
                    return;
                self->processingBusy_ = false;
                self->editBusy_ = false;

                const int index = self->findModel(path);
                if (index < 0) {
                    if (finished)
                        finished(false, QString::fromUtf8("模型已被移除，处理结果已丢弃"));
                    return;
                }
                auto& model = *self->models_[static_cast<std::size_t>(index)];
                if (model.cloud != sourceCloud || model.mesh != sourceMesh) {
                    if (finished)
                        finished(false, QString::fromUtf8("模型已发生变化，旧任务结果已丢弃"));
                    return;
                }
                if (result.cancelled) {
                    if (finished)
                        finished(false, QString::fromUtf8("处理已取消"));
                    return;
                }
                if (!result.success) {
                    if (finished)
                        finished(false, QString::fromUtf8(result.message.c_str()));
                    return;
                }

                const bool inputWasMesh = static_cast<bool>(sourceMesh);
                const bool outputIsMesh = static_cast<bool>(result.mesh);
                const bool addOnKindChange = outputPolicy == JMEngine::processing::OutputPolicy::AddModelOnKindChange &&
                                             inputWasMesh != outputIsMesh;

                if (addOnKindChange && result.mesh) {
                    // PointCloud -> Mesh（Poisson 等）不覆盖原始扫描点云，而是在模型管理器新增 Mesh。
                    QFileInfo srcInfo(path);
                    QString base = srcInfo.completeBaseName();
                    if (base.isEmpty())
                        base = QString::fromUtf8("model");
                    QString generated =
                        srcInfo.absolutePath() + QLatin1Char('/') + base + QString::fromUtf8("_poisson_mesh");
                    int suffix = 2;
                    while (self->findModel(generated) >= 0)
                        generated = srcInfo.absolutePath() + QLatin1Char('/') + base +
                                    QString::fromUtf8("_poisson_mesh_%1").arg(suffix++);

                    auto newModel = std::make_unique<Model>(generated, result.mesh);
                    newModel->modelTransform = model.modelTransform;
                    if (preferResultVertexColors) {
                        // Show the colors transferred by IndustrialPoisson instead of the Model
                        // constructor's neutral-gray GPU override.
                        newModel->useDisplayColor = false;
                    } else if (model.useDisplayColor) {
                        newModel->displayColor = model.displayColor;
                        newModel->useDisplayColor = true;
                    }
                    self->models_.push_back(std::move(newModel));
                    self->activeModelIndex_ = static_cast<int>(self->models_.size()) - 1;
                    self->statusText_ =
                        QString::fromUtf8("处理完成：已新增网格模型 ") + QFileInfo(generated).fileName();
                    if (self->modelAddedCallback_)
                        self->modelAddedCallback_(generated);
                    self->update();
                    if (finished) {
                        finished(true, QString::fromUtf8("完成：保留原点云，并新增 Mesh；%1 顶点，%2 三角形")
                                           .arg(static_cast<qulonglong>(result.outputPoints))
                                           .arg(static_cast<qulonglong>(result.outputTriangles)));
                    }
                    return;
                }

                model.processingUndo.push_back(self->captureProcessingSnapshot(model));
                if (model.processingUndo.size() > 4u)
                    model.processingUndo.erase(model.processingUndo.begin());
                model.processingRedo.clear();

                self->makeCurrent();
                self->destroyModelGl(model);
                self->doneCurrent();

                if (result.mesh) {
                    model.mesh = result.mesh;
                    model.cloud = result.mesh->vertices();
                    model.meshMode = true;
                    model.displayMode = DisplayMode::Solid;
                    if (preferResultVertexColors)
                        model.useDisplayColor = false;
                    model.meshEditor.setMesh(model.mesh);
                    model.visibleMeshIndices.clear();
                    model.visibleTriangleIds.clear();
                    model.meshFiltered = false;
                } else if (result.cloud) {
                    model.cloud = result.cloud;
                    model.mesh.reset();
                    model.meshMode = false;
                    model.displayMode = DisplayMode::Points;
                    model.meshEditor.setMesh(nullptr);
                }

                model.editor.setPointCloud(model.cloud);
                model.selectionMask.assign(model.cloud ? model.cloud->size() : 0u, 0u);
                model.selectedIds.clear();
                model.selectedTriangleIds.clear();
                model.selectedMeshIndexCount = 0;
                model.pickGridIds.clear();
                model.pickBlocks.clear();
                model.pickIndexedCloud = nullptr;
                model.pickIndexedPointCount = 0;
                model.uploadPointCursor = 0;
                model.uploadIndexCursor = 0;
                model.drawPointCount = 0;
                model.drawIndexCount = 0;
                model.meshUploadComplete = false;
                model.wireDirty = true;
                model.glCreated = false;

                self->statusText_ = QString::fromUtf8("处理完成：") +
                                    QString::fromUtf8(result.message.empty() ? "OK" : result.message.c_str());
                self->update();

                if (finished) {
                    QString summary;
                    if (model.meshMode) {
                        summary = QString::fromUtf8("完成：%1 -> %2 三角形，%3 -> %4 顶点")
                                      .arg(static_cast<qulonglong>(result.inputTriangles))
                                      .arg(static_cast<qulonglong>(result.outputTriangles))
                                      .arg(static_cast<qulonglong>(result.inputPoints))
                                      .arg(static_cast<qulonglong>(result.outputPoints));
                        if (result.holesDetected > 0)
                            summary += QString::fromUtf8("；检测孔洞 %1 个")
                                           .arg(static_cast<qulonglong>(result.holesDetected));
                        if (result.degenerateTriangles > 0 || result.nonManifoldEdges > 0 ||
                            result.connectedComponents > 0) {
                            summary += QString::fromUtf8("；退化面 %1，非流形边 %2，边界边 %3，连通域 %4")
                                           .arg(static_cast<qulonglong>(result.degenerateTriangles))
                                           .arg(static_cast<qulonglong>(result.nonManifoldEdges))
                                           .arg(static_cast<qulonglong>(result.boundaryEdges))
                                           .arg(static_cast<qulonglong>(result.connectedComponents));
                        }
                    } else {
                        summary = QString::fromUtf8("完成：%1 -> %2 点")
                                      .arg(static_cast<qulonglong>(result.inputPoints))
                                      .arg(static_cast<qulonglong>(result.outputPoints));
                    }
                    finished(true, summary);
                }
            },
            Qt::QueuedConnection);
    });
    return true;
}

void PointCloudWidget::cancelProcessing() {
    if (!processingBusy_)
        return;
    processingCancel_.cancel();
    statusText_ = QString::fromUtf8("正在取消处理任务...");
    update();
}

void PointCloudWidget::keyPressEvent(QKeyEvent* e) {
    if (e->key() == Qt::Key_Delete)
        deleteSelection();
    else if (e->matches(QKeySequence::Undo))
        undoEdit();
    else if (e->matches(QKeySequence::Redo))
        redoEdit();
    else if (e->key() == Qt::Key_F)
        fitView();
    else if (e->key() == Qt::Key_Escape)
        clearSelection();
    else
        QOpenGLWidget::keyPressEvent(e);
}

QPoint PointCloudWidget::toPhysical(const QPointF& p) const {
    const qreal d = devicePixelRatioF();
    return QPoint(int(p.x() * d), int(p.y() * d));
}
void PointCloudWidget::drawSelectionOverlay(Model& m) {
    if (!backend_ || !m.meshMode || !m.glCreated || m.selectedMeshIndexCount <= 0)
        return;

    const auto mvp = modelMvp(m);
    meshProgram_.bind();
    meshProgram_.setUniformValue("uMVP", QMatrix4x4(mvp.m.data()).transposed());
    const auto eye = camera_.eye();
    const auto lightWorld = JMEngine::example::normalize(JMEngine::example::sub(eye, camera_.target));
    const auto& mm = m.modelTransform.m;
    JMEngine::Vec3f lightLocal{
        mm[0] * lightWorld.x + mm[1] * lightWorld.y + mm[2] * lightWorld.z,
        mm[4] * lightWorld.x + mm[5] * lightWorld.y + mm[6] * lightWorld.z,
        mm[8] * lightWorld.x + mm[9] * lightWorld.y + mm[10] * lightWorld.z};
    lightLocal = JMEngine::example::normalize(lightLocal);
    meshProgram_.setUniformValue("uLightDir", QVector3D(lightLocal.x, lightLocal.y, lightLocal.z));
    meshProgram_.setUniformValue("uPointSize", 1.0f);
    meshProgram_.setUniformValue("uPointMode", 0.0f);
    meshProgram_.setUniformValue("uForceSelected", 1.0f);
    meshProgram_.setUniformValue("uWireframe", 0.0f);
    meshProgram_.setUniformValue("uUseTexture", 0.0f);
    meshProgram_.setUniformValue("uTexture", 0);

    // 与原网格共用 depth。轻微向相机方向偏移，避免完全共面导致 Z-fighting。
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_FALSE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-1.0f, -1.0f);

    backend_->bindMeshSelectionLayout(*this, m.gpu);
    glDrawElements(GL_TRIANGLES, m.selectedMeshIndexCount, GL_UNSIGNED_INT, nullptr);
    backend_->unbindLayout(*this);

    glDisable(GL_POLYGON_OFFSET_FILL);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    meshProgram_.release();
}
