#include <JMEngine/CpuMeshSelector.h>
#include <JMEngine/processing/Parallel.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#ifdef JMENGINE_USE_OPENMP
#include <omp.h>
#endif

namespace JMEngine {
namespace {

struct P2 {
    float x{}, y{};
};
struct ScreenVertex {
    float x{}, y{}, z{};
};

bool project2(const Vec3f& p, const Mat4f& m, const Viewport& vp, P2& out) {
    const float cx = m.m[0] * p.x + m.m[4] * p.y + m.m[8] * p.z + m.m[12];
    const float cy = m.m[1] * p.x + m.m[5] * p.y + m.m[9] * p.z + m.m[13];
    const float cz = m.m[2] * p.x + m.m[6] * p.y + m.m[10] * p.z + m.m[14];
    const float cw = m.m[3] * p.x + m.m[7] * p.y + m.m[11] * p.z + m.m[15];
    if (cw <= 1.0e-8f)
        return false;
    const float nx = cx / cw, ny = cy / cw, nz = cz / cw;
    if (nz < -1.0f || nz > 1.0f)
        return false;
    out.x = (nx * 0.5f + 0.5f) * float(vp.width);
    out.y = (1.0f - (ny * 0.5f + 0.5f)) * float(vp.height);
    return true;
}

bool project3(const Vec3f& p, const Mat4f& m, const Viewport& vp, ScreenVertex& out) {
    const float cx = m.m[0] * p.x + m.m[4] * p.y + m.m[8] * p.z + m.m[12];
    const float cy = m.m[1] * p.x + m.m[5] * p.y + m.m[9] * p.z + m.m[13];
    const float cz = m.m[2] * p.x + m.m[6] * p.y + m.m[10] * p.z + m.m[14];
    const float cw = m.m[3] * p.x + m.m[7] * p.y + m.m[11] * p.z + m.m[15];
    if (cw <= 1.0e-8f)
        return false;
    const float nx = cx / cw, ny = cy / cw, nz = cz / cw;
    if (nz < -1.0f || nz > 1.0f)
        return false;
    out.x = (nx * 0.5f + 0.5f) * float(vp.width);
    out.y = (1.0f - (ny * 0.5f + 0.5f)) * float(vp.height);
    out.z = nz * 0.5f + 0.5f; // 与 OpenGL depth range [0,1] 同方向：越小越靠前。
    return true;
}

float orient(P2 a, P2 b, P2 c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}
bool onSeg(P2 a, P2 b, P2 p) {
    return p.x >= std::min(a.x, b.x) - 1e-4f && p.x <= std::max(a.x, b.x) + 1e-4f &&
           p.y >= std::min(a.y, b.y) - 1e-4f && p.y <= std::max(a.y, b.y) + 1e-4f;
}
bool segInter(P2 a, P2 b, P2 c, P2 d) {
    float o1 = orient(a, b, c), o2 = orient(a, b, d), o3 = orient(c, d, a), o4 = orient(c, d, b);
    if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0)))
        return true;
    if (std::fabs(o1) < 1e-5f && onSeg(a, b, c))
        return true;
    if (std::fabs(o2) < 1e-5f && onSeg(a, b, d))
        return true;
    if (std::fabs(o3) < 1e-5f && onSeg(c, d, a))
        return true;
    if (std::fabs(o4) < 1e-5f && onSeg(c, d, b))
        return true;
    return false;
}
bool pointTri(P2 p, P2 a, P2 b, P2 c) {
    const float d1 = orient(p, a, b), d2 = orient(p, b, c), d3 = orient(p, c, a);
    const bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0), pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    return !(neg && pos);
}
bool pointPoly(P2 p, const std::vector<Point2i>& poly) {
    bool inside = false;
    if (poly.size() < 3)
        return false;
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const float xi = float(poly[i].x), yi = float(poly[i].y), xj = float(poly[j].x), yj = float(poly[j].y);
        const bool cross =
            ((yi > p.y) != (yj > p.y)) && (p.x < (xj - xi) * (p.y - yi) / ((yj - yi) == 0 ? 1e-20f : (yj - yi)) + xi);
        if (cross)
            inside = !inside;
    }
    return inside;
}
float dist2Seg(P2 p, P2 a, P2 b) {
    float x = b.x - a.x, y = b.y - a.y, den = x * x + y * y,
          t = den > 1e-12f ? std::clamp(((p.x - a.x) * x + (p.y - a.y) * y) / den, 0.0f, 1.0f) : 0.0f;
    float dx = p.x - (a.x + t * x), dy = p.y - (a.y + t * y);
    return dx * dx + dy * dy;
}

bool triRect(P2 a, P2 b, P2 c, const RectI& rr) {
    const float l = float(std::min(rr.x1, rr.x2)), r = float(std::max(rr.x1, rr.x2)), t = float(std::min(rr.y1, rr.y2)),
                bt = float(std::max(rr.y1, rr.y2));
    auto in = [&](P2 p) { return p.x >= l && p.x <= r && p.y >= t && p.y <= bt; };
    if (in(a) || in(b) || in(c))
        return true;
    P2 q[4] = {{l, t}, {r, t}, {r, bt}, {l, bt}};
    for (auto p : q)
        if (pointTri(p, a, b, c))
            return true;
    P2 e1[3] = {a, b, c}, e2[3] = {b, c, a};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j)
            if (segInter(e1[i], e2[i], q[j], q[(j + 1) % 4]))
                return true;
    return false;
}
bool triCircle(P2 a, P2 b, P2 c, P2 ctr, float rad) {
    float r2 = rad * rad;
    auto d = [&](P2 p) {
        float x = p.x - ctr.x, y = p.y - ctr.y;
        return x * x + y * y;
    };
    if (d(a) <= r2 || d(b) <= r2 || d(c) <= r2 || pointTri(ctr, a, b, c))
        return true;
    return dist2Seg(ctr, a, b) <= r2 || dist2Seg(ctr, b, c) <= r2 || dist2Seg(ctr, c, a) <= r2;
}
bool triPoly(P2 a, P2 b, P2 c, const std::vector<Point2i>& poly) {
    if (poly.size() < 3)
        return false;
    if (pointPoly(a, poly) || pointPoly(b, poly) || pointPoly(c, poly))
        return true;
    for (const auto& p : poly)
        if (pointTri({float(p.x), float(p.y)}, a, b, c))
            return true;
    P2 e1[3] = {a, b, c}, e2[3] = {b, c, a};
    for (int i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < poly.size(); ++j) {
            P2 p{float(poly[j].x), float(poly[j].y)},
                q{float(poly[(j + 1) % poly.size()].x), float(poly[(j + 1) % poly.size()].y)};
            if (segInter(e1[i], e2[i], p, q))
                return true;
        }
    return false;
}

template <class Hit>
std::vector<TriangleId> selectThrough(const TriangleMesh& mesh, const Mat4f& m, const Viewport& vp, bool includeDeleted,
                                      Hit hit) {
    std::vector<TriangleId> out;
    auto cloud = mesh.vertices();
    if (!cloud || vp.width <= 0 || vp.height <= 0)
        return out;
    const auto& idx = mesh.indices();
    const std::size_t n = mesh.triangleCount();
#ifdef JMENGINE_USE_OPENMP
    const int tc = processing::processingThreadCount();
    std::vector<std::vector<TriangleId>> per(static_cast<std::size_t>(tc));
#pragma omp parallel num_threads(tc)
    {
        auto& loc = per[static_cast<std::size_t>(omp_get_thread_num())];
#pragma omp for schedule(static)
        for (std::int64_t tt = 0; tt < static_cast<std::int64_t>(n); ++tt) {
            TriangleId t = static_cast<TriangleId>(tt);
            if (!includeDeleted && !mesh.triangleActive(t))
                continue;
            std::size_t b = std::size_t(t) * 3u;
            auto i0 = idx[b], i1 = idx[b + 1], i2 = idx[b + 2];
            if (i0 >= cloud->size() || i1 >= cloud->size() || i2 >= cloud->size())
                continue;
            P2 a, bp, c;
            if (!project2(cloud->points()[i0].position, m, vp, a) ||
                !project2(cloud->points()[i1].position, m, vp, bp) || !project2(cloud->points()[i2].position, m, vp, c))
                continue;
            if (hit(a, bp, c))
                loc.push_back(t);
        }
    }
    std::size_t total = 0;
    for (auto& v : per)
        total += v.size();
    out.reserve(total);
    for (auto& v : per)
        out.insert(out.end(), v.begin(), v.end());
    std::sort(out.begin(), out.end());
#else
    out.reserve(n / 32 + 1);
    for (TriangleId t = 0; std::size_t(t) < n; ++t) {
        if (!includeDeleted && !mesh.triangleActive(t))
            continue;
        std::size_t b = std::size_t(t) * 3u;
        auto i0 = idx[b], i1 = idx[b + 1], i2 = idx[b + 2];
        if (i0 >= cloud->size() || i1 >= cloud->size() || i2 >= cloud->size())
            continue;
        P2 a, bp, c;
        if (project2(cloud->points()[i0].position, m, vp, a) && project2(cloud->points()[i1].position, m, vp, bp) &&
            project2(cloud->points()[i2].position, m, vp, c) && hit(a, bp, c))
            out.push_back(t);
    }
#endif
    return out;
}

struct PixelRegion {
    int left{}, top{}, right{}, bottom{}; // inclusive, logical pixel coordinates
};

PixelRegion clampRegion(PixelRegion r, const Viewport& vp) {
    r.left = std::clamp(r.left, 0, std::max(0, vp.width - 1));
    r.right = std::clamp(r.right, 0, std::max(0, vp.width - 1));
    r.top = std::clamp(r.top, 0, std::max(0, vp.height - 1));
    r.bottom = std::clamp(r.bottom, 0, std::max(0, vp.height - 1));
    if (r.left > r.right)
        std::swap(r.left, r.right);
    if (r.top > r.bottom)
        std::swap(r.top, r.bottom);
    return r;
}

template <class PixelHit>
std::vector<TriangleId> selectSurface(const TriangleMesh& mesh, const Mat4f& m, const Viewport& vp, PixelRegion region,
                                      PixelHit pixelHit) {
    std::vector<TriangleId> out;
    auto cloud = mesh.vertices();
    if (!cloud || vp.width <= 0 || vp.height <= 0 || mesh.empty())
        return out;
    region = clampRegion(region, vp);
    const int rw = region.right - region.left + 1, rh = region.bottom - region.top + 1;
    if (rw <= 0 || rh <= 0)
        return out;

    const std::size_t pixelCount = std::size_t(rw) * std::size_t(rh);
    std::vector<float> depth(pixelCount, std::numeric_limits<float>::infinity());
    std::vector<TriangleId> owner(pixelCount, kInvalidTriangleId);
    const auto& idx = mesh.indices();

    for (TriangleId tid = 0; std::size_t(tid) < mesh.triangleCount(); ++tid) {
        if (!mesh.triangleActive(tid))
            continue;
        const std::size_t b = std::size_t(tid) * 3u;
        const auto i0 = idx[b], i1 = idx[b + 1], i2 = idx[b + 2];
        if (i0 >= cloud->size() || i1 >= cloud->size() || i2 >= cloud->size())
            continue;
        ScreenVertex a, bv, c;
        if (!project3(cloud->points()[i0].position, m, vp, a) || !project3(cloud->points()[i1].position, m, vp, bv) ||
            !project3(cloud->points()[i2].position, m, vp, c))
            continue;

        const float area = (bv.x - a.x) * (c.y - a.y) - (bv.y - a.y) * (c.x - a.x);
        if (std::fabs(area) < 1.0e-8f)
            continue;
        const int minX = std::max(region.left, int(std::floor(std::min({a.x, bv.x, c.x}))));
        const int maxX = std::min(region.right, int(std::ceil(std::max({a.x, bv.x, c.x}))));
        const int minY = std::max(region.top, int(std::floor(std::min({a.y, bv.y, c.y}))));
        const int maxY = std::min(region.bottom, int(std::ceil(std::max({a.y, bv.y, c.y}))));
        if (minX > maxX || minY > maxY)
            continue;

        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                const P2 p{float(x) + 0.5f, float(y) + 0.5f};
                if (!pixelHit(p))
                    continue;
                const float w0 = ((bv.x - p.x) * (c.y - p.y) - (bv.y - p.y) * (c.x - p.x)) / area;
                const float w1 = ((c.x - p.x) * (a.y - p.y) - (c.y - p.y) * (a.x - p.x)) / area;
                const float w2 = 1.0f - w0 - w1;
                constexpr float eps = -1.0e-5f;
                if (w0 < eps || w1 < eps || w2 < eps)
                    continue;
                const float z = w0 * a.z + w1 * bv.z + w2 * c.z;
                const std::size_t pi = std::size_t(y - region.top) * std::size_t(rw) + std::size_t(x - region.left);
                if (z < depth[pi]) {
                    depth[pi] = z;
                    owner[pi] = tid;
                }
            }
        }
    }

    for (auto tid : owner)
        if (tid != kInvalidTriangleId)
            out.push_back(tid);
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

PixelRegion rectRegion(const RectI& r) {
    return {std::min(r.x1, r.x2), std::min(r.y1, r.y2), std::max(r.x1, r.x2), std::max(r.y1, r.y2)};
}
PixelRegion circleRegion(const Point2i& c, int rad) {
    rad = std::max(0, rad);
    return {c.x - rad, c.y - rad, c.x + rad, c.y + rad};
}
PixelRegion polyRegion(const std::vector<Point2i>& p, int pad = 0) {
    if (p.empty())
        return {};
    int l = p[0].x, r = l, t = p[0].y, b = t;
    for (const auto& q : p) {
        l = std::min(l, q.x);
        r = std::max(r, q.x);
        t = std::min(t, q.y);
        b = std::max(b, q.y);
    }
    return {l - pad, t - pad, r + pad, b + pad};
}
bool brushHit(P2 p, const std::vector<Point2i>& path, float radius) {
    if (path.empty())
        return false;
    const float r2 = radius * radius;
    if (path.size() == 1) {
        const float dx = p.x - path[0].x, dy = p.y - path[0].y;
        return dx * dx + dy * dy <= r2;
    }
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (dist2Seg(p, {float(path[i - 1].x), float(path[i - 1].y)}, {float(path[i].x), float(path[i].y)}) <= r2)
            return true;
    }
    return false;
}

} // namespace

std::vector<TriangleId> CpuMeshSelector::rectangle(const TriangleMesh& mesh, const Mat4f& m, const Viewport& vp,
                                                   const RectI& r, bool d) {
    return selectThrough(mesh, m, vp, d, [&](P2 a, P2 b, P2 c) { return triRect(a, b, c, r); });
}
std::vector<TriangleId> CpuMeshSelector::circle(const TriangleMesh& mesh, const Mat4f& m, const Viewport& vp,
                                                const Point2i& ctr, int rad, bool d) {
    P2 c{float(ctr.x), float(ctr.y)};
    return selectThrough(mesh, m, vp, d,
                         [=](P2 a, P2 b, P2 cc) { return triCircle(a, b, cc, c, float(std::max(0, rad))); });
}
std::vector<TriangleId> CpuMeshSelector::lasso(const TriangleMesh& mesh, const Mat4f& m, const Viewport& vp,
                                               const std::vector<Point2i>& poly, bool d) {
    return selectThrough(mesh, m, vp, d, [&](P2 a, P2 b, P2 c) { return triPoly(a, b, c, poly); });
}
std::vector<TriangleId> CpuMeshSelector::brushStroke(const TriangleMesh& mesh, const Mat4f& m, const Viewport& vp,
                                                     const std::vector<Point2i>& path, int rad, bool d) {
    if (path.empty())
        return {};
    const float rr = float(std::max(0, rad));
    return selectThrough(mesh, m, vp, d, [&](P2 a, P2 b, P2 c) {
        for (const auto& p : path)
            if (triCircle(a, b, c, {float(p.x), float(p.y)}, rr))
                return true;
        return false;
    });
}

std::vector<TriangleId> CpuMeshSelector::surfaceRectangle(const TriangleMesh& mesh, const Mat4f& m, const Viewport& vp,
                                                          const RectI& r) {
    const auto region = rectRegion(r);
    const float l = float(region.left), rr = float(region.right), t = float(region.top), b = float(region.bottom);
    return selectSurface(mesh, m, vp, region,
                         [=](P2 p) { return p.x >= l && p.x <= rr + 1.0f && p.y >= t && p.y <= b + 1.0f; });
}
std::vector<TriangleId> CpuMeshSelector::surfaceCircle(const TriangleMesh& mesh, const Mat4f& m, const Viewport& vp,
                                                       const Point2i& c, int rad) {
    rad = std::max(0, rad);
    const float r2 = float(rad * rad);
    const auto region = circleRegion(c, rad);
    return selectSurface(mesh, m, vp, region, [=](P2 p) {
        const float dx = p.x - float(c.x), dy = p.y - float(c.y);
        return dx * dx + dy * dy <= r2;
    });
}
std::vector<TriangleId> CpuMeshSelector::surfaceLasso(const TriangleMesh& mesh, const Mat4f& m, const Viewport& vp,
                                                      const std::vector<Point2i>& poly) {
    if (poly.size() < 3)
        return {};
    const auto region = polyRegion(poly);
    return selectSurface(mesh, m, vp, region, [&](P2 p) { return pointPoly(p, poly); });
}
std::vector<TriangleId> CpuMeshSelector::surfaceBrushStroke(const TriangleMesh& mesh, const Mat4f& m,
                                                            const Viewport& vp, const std::vector<Point2i>& path,
                                                            int rad) {
    if (path.empty())
        return {};
    rad = std::max(0, rad);
    const auto region = polyRegion(path, rad);
    return selectSurface(mesh, m, vp, region, [&](P2 p) { return brushHit(p, path, float(rad)); });
}

} // namespace JMEngine
