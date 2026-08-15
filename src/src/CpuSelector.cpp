#include <pceditor/CpuSelector.h>
#include <pceditor/processing/Parallel.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <cstdint>
#include <limits>

#ifdef PCEDITOR_USE_OPENMP
#include <omp.h>
#endif

namespace pceditor {
namespace {

// 把一个三维点投影到 UI 屏幕坐标。
// 返回 false 表示点位于相机后方或裁剪空间之外。
bool projectToScreen(const Vec3f& position, const Mat4f& mvp, const Viewport& viewport, float& screenX, float& screenY,
                     float* depth01 = nullptr) {
    const float x = position.x;
    const float y = position.y;
    const float z = position.z;

    const float clipX = mvp.m[0] * x + mvp.m[4] * y + mvp.m[8] * z + mvp.m[12];
    const float clipY = mvp.m[1] * x + mvp.m[5] * y + mvp.m[9] * z + mvp.m[13];
    const float clipZ = mvp.m[2] * x + mvp.m[6] * y + mvp.m[10] * z + mvp.m[14];
    const float clipW = mvp.m[3] * x + mvp.m[7] * y + mvp.m[11] * z + mvp.m[15];

    if (clipW <= 0.0f)
        return false;

    const float ndcX = clipX / clipW;
    const float ndcY = clipY / clipW;
    const float ndcZ = clipZ / clipW;
    if (ndcX < -1.0f || ndcX > 1.0f || ndcY < -1.0f || ndcY > 1.0f || ndcZ < -1.0f || ndcZ > 1.0f) {
        return false;
    }

    screenX = (ndcX * 0.5f + 0.5f) * static_cast<float>(viewport.width);
    screenY = (1.0f - (ndcY * 0.5f + 0.5f)) * static_cast<float>(viewport.height);
    if (depth01)
        *depth01 = ndcZ * 0.5f + 0.5f;
    return true;
}

// 多边形点内测试，采用奇偶规则。对凹多边形同样有效。
bool pointInPolygon(float x, float y, const std::vector<Point2i>& polygon) {
    bool inside = false;
    const std::size_t n = polygon.size();
    if (n < 3)
        return false;

    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const float xi = static_cast<float>(polygon[i].x);
        const float yi = static_cast<float>(polygon[i].y);
        const float xj = static_cast<float>(polygon[j].x);
        const float yj = static_cast<float>(polygon[j].y);

        const bool crosses =
            ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / ((yj - yi) == 0.0f ? 1.0e-20f : (yj - yi)) + xi);
        if (crosses)
            inside = !inside;
    }
    return inside;
}

float distanceSquaredToSegment(float px, float py, float ax, float ay, float bx, float by) {
    const float abx = bx - ax;
    const float aby = by - ay;
    const float apx = px - ax;
    const float apy = py - ay;
    const float denom = abx * abx + aby * aby;

    float t = 0.0f;
    if (denom > 1.0e-12f) {
        t = std::clamp((apx * abx + apy * aby) / denom, 0.0f, 1.0f);
    }

    const float dx = px - (ax + t * abx);
    const float dy = py - (ay + t * aby);
    return dx * dx + dy * dy;
}

// 选择器公共遍历逻辑：只负责投影，具体屏幕形状由 hitTest 决定。
template <typename HitTest>
std::vector<PointId> selectSurfaceProjected(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                            bool includeDeleted, float depthToleranceNdc, HitTest&& hitTest) {
    std::vector<PointId> selected;
    if (viewport.width <= 0 || viewport.height <= 0 || cloud.empty())
        return selected;

    const std::size_t pixelCount = static_cast<std::size_t>(viewport.width) * static_cast<std::size_t>(viewport.height);
    std::vector<float> minDepth(pixelCount, std::numeric_limits<float>::infinity());

    // Surface Picking 使用 4px 的 point sprite。Surface 软件深度必须按点的屏幕 footprint
    // 写入，而不能只写中心 1 像素；否则后层点中心落在相邻像素时会被误判为可见，表现得像 Through。
    constexpr float kRenderedPointDiameterPixels = 4.0f;
    const float radius = kRenderedPointDiameterPixels * 0.5f;
    const int radiusCeil = std::max(1, static_cast<int>(std::ceil(radius)));
    const float radius2 = radius * radius;

    // 第一遍建立与 GL_POINTS 近似一致的软件 Z-buffer。这里故意顺序写，避免原子 float 平台差异；
    // 第二遍的大量投影/筛选仍然使用 OpenMP。
    for (PointId id = 0; static_cast<std::size_t>(id) < cloud.size(); ++id) {
        const auto& point = cloud.points()[id];
        if (!includeDeleted && (point.flags & PointDeleted) != 0)
            continue;
        float sx = 0.0f, sy = 0.0f, depth = 1.0f;
        if (!projectToScreen(point.position, mvp, viewport, sx, sy, &depth))
            continue;
        const int cx = static_cast<int>(std::floor(sx));
        const int cy = static_cast<int>(std::floor(sy));
        for (int dy = -radiusCeil; dy <= radiusCeil; ++dy) {
            const int py = cy + dy;
            if (py < 0 || py >= viewport.height)
                continue;
            for (int dx = -radiusCeil; dx <= radiusCeil; ++dx) {
                const int px = cx + dx;
                if (px < 0 || px >= viewport.width)
                    continue;
                const float fx = (static_cast<float>(px) + 0.5f) - sx;
                const float fy = (static_cast<float>(py) + 0.5f) - sy;
                if (fx * fx + fy * fy > radius2)
                    continue;
                const std::size_t pi = static_cast<std::size_t>(py) * static_cast<std::size_t>(viewport.width) +
                                       static_cast<std::size_t>(px);
                minDepth[pi] = std::min(minDepth[pi], depth);
            }
        }
    }

    const float tolerance = std::max(1.0e-6f, depthToleranceNdc);

    // 与 GPU Compute Surface 保持一致：可见性在“当前点中心”采样深度，
    // 但前表面 Z-buffer 已按真实 point sprite footprint 写入。这样后层点中心只要被前点覆盖就会被剔除，
    // 同时不会因为检查后层 sprite 边缘的未覆盖像素而误判为可见。
    auto isSurfaceVisible = [&](float sx, float sy, float depth) {
        const int px = std::clamp(static_cast<int>(std::floor(sx)), 0, viewport.width - 1);
        const int py = std::clamp(static_cast<int>(std::floor(sy)), 0, viewport.height - 1);
        const std::size_t pi =
            static_cast<std::size_t>(py) * static_cast<std::size_t>(viewport.width) + static_cast<std::size_t>(px);
        return depth <= minDepth[pi] + tolerance;
    };
#ifdef PCEDITOR_USE_OPENMP
    const int threadCount = processing::processingThreadCount();
    std::vector<std::vector<PointId>> perThread(static_cast<std::size_t>(threadCount));
#pragma omp parallel num_threads(threadCount)
    {
        auto& local = perThread[static_cast<std::size_t>(omp_get_thread_num())];
#pragma omp for schedule(static)
        for (std::int64_t ii = 0; ii < static_cast<std::int64_t>(cloud.size()); ++ii) {
            const PointId id = static_cast<PointId>(ii);
            const auto& point = cloud.points()[static_cast<std::size_t>(ii)];
            if (!includeDeleted && (point.flags & PointDeleted) != 0)
                continue;
            float sx = 0.0f, sy = 0.0f, depth = 1.0f;
            if (!projectToScreen(point.position, mvp, viewport, sx, sy, &depth) || !hitTest(sx, sy))
                continue;
            if (isSurfaceVisible(sx, sy, depth))
                local.push_back(id);
        }
    }
    std::size_t total = 0;
    for (const auto& v : perThread)
        total += v.size();
    selected.reserve(total);
    for (auto& v : perThread)
        selected.insert(selected.end(), v.begin(), v.end());
    std::sort(selected.begin(), selected.end());
#else
    for (PointId id = 0; static_cast<std::size_t>(id) < cloud.size(); ++id) {
        const auto& point = cloud.points()[id];
        if (!includeDeleted && (point.flags & PointDeleted) != 0)
            continue;
        float sx = 0.0f, sy = 0.0f, depth = 1.0f;
        if (!projectToScreen(point.position, mvp, viewport, sx, sy, &depth) || !hitTest(sx, sy))
            continue;
        if (isSurfaceVisible(sx, sy, depth))
            selected.push_back(id);
    }
#endif
    return selected;
}

template <typename HitTest>
std::vector<PointId> selectProjected(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                     bool includeDeleted, HitTest&& hitTest) {
    std::vector<PointId> selected;
    if (viewport.width <= 0 || viewport.height <= 0 || cloud.empty())
        return selected;

#ifdef PCEDITOR_USE_OPENMP
    // 穿透选择必须遍历全部点。千万点情况下如果仍然单线程投影，
    // 鼠标松开后会明显卡住，因此这里对 CPU 投影阶段做 OpenMP 并行。
    // 每个线程使用自己的结果数组，最后再顺序合并，避免 push_back 锁竞争。
    const int threadCount = processing::processingThreadCount();
    std::vector<std::vector<PointId>> perThread(static_cast<std::size_t>(threadCount));

#pragma omp parallel num_threads(threadCount)
    {
        const int tid = omp_get_thread_num();
        auto& local = perThread[static_cast<std::size_t>(tid)];
        local.reserve(cloud.size() / static_cast<std::size_t>(threadCount * 32) + 16);

#pragma omp for schedule(static)
        for (std::int64_t ii = 0; ii < static_cast<std::int64_t>(cloud.size()); ++ii) {
            const PointId id = static_cast<PointId>(ii);
            const auto& point = cloud.points()[static_cast<std::size_t>(ii)];
            if (!includeDeleted && (point.flags & PointDeleted) != 0)
                continue;

            float sx = 0.0f;
            float sy = 0.0f;
            if (!projectToScreen(point.position, mvp, viewport, sx, sy))
                continue;
            if (hitTest(sx, sy))
                local.push_back(id);
        }
    }

    std::size_t total = 0;
    for (const auto& v : perThread)
        total += v.size();
    selected.reserve(total);
    for (auto& v : perThread) {
        selected.insert(selected.end(), v.begin(), v.end());
    }
    // 并行分段合并后的 ID 通常已经近似有序，但线程分块顺序不应作为 API 保证。
    std::sort(selected.begin(), selected.end());
#else
    selected.reserve(cloud.size() / 32 + 1);
    for (PointId id = 0; static_cast<std::size_t>(id) < cloud.size(); ++id) {
        const auto& point = cloud.points()[id];
        if (!includeDeleted && (point.flags & PointDeleted) != 0)
            continue;

        float sx = 0.0f;
        float sy = 0.0f;
        if (!projectToScreen(point.position, mvp, viewport, sx, sy))
            continue;
        if (hitTest(sx, sy))
            selected.push_back(id);
    }
#endif
    return selected;
}

} // namespace

std::vector<PointId> CpuSelector::rectangle(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                            const RectI& rect, bool includeDeleted) {
    const float left = static_cast<float>(std::min(rect.x1, rect.x2));
    const float right = static_cast<float>(std::max(rect.x1, rect.x2));
    const float top = static_cast<float>(std::min(rect.y1, rect.y2));
    const float bottom = static_cast<float>(std::max(rect.y1, rect.y2));

    return selectProjected(cloud, mvp, viewport, includeDeleted,
                           [=](float x, float y) { return x >= left && x <= right && y >= top && y <= bottom; });
}

std::vector<PointId> CpuSelector::circle(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                         const Point2i& center, int radiusPixels, bool includeDeleted) {
    const float cx = static_cast<float>(center.x);
    const float cy = static_cast<float>(center.y);
    const float radius = static_cast<float>(std::max(0, radiusPixels));
    const float radius2 = radius * radius;

    return selectProjected(cloud, mvp, viewport, includeDeleted, [=](float x, float y) {
        const float dx = x - cx;
        const float dy = y - cy;
        return dx * dx + dy * dy <= radius2;
    });
}

std::vector<PointId> CpuSelector::lasso(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                        const std::vector<Point2i>& polygon, bool includeDeleted) {
    if (polygon.size() < 3)
        return {};
    return selectProjected(cloud, mvp, viewport, includeDeleted,
                           [&](float x, float y) { return pointInPolygon(x, y, polygon); });
}

std::vector<PointId> CpuSelector::rectangleSurface(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                                   const RectI& rect, float depthToleranceNdc, bool includeDeleted) {
    const float left = static_cast<float>(std::min(rect.x1, rect.x2));
    const float right = static_cast<float>(std::max(rect.x1, rect.x2));
    const float top = static_cast<float>(std::min(rect.y1, rect.y2));
    const float bottom = static_cast<float>(std::max(rect.y1, rect.y2));
    return selectSurfaceProjected(cloud, mvp, viewport, includeDeleted, depthToleranceNdc,
                                  [=](float x, float y) { return x >= left && x <= right && y >= top && y <= bottom; });
}

std::vector<PointId> CpuSelector::circleSurface(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                                const Point2i& center, int radiusPixels, float depthToleranceNdc,
                                                bool includeDeleted) {
    const float cx = static_cast<float>(center.x), cy = static_cast<float>(center.y);
    const float radius = static_cast<float>(std::max(0, radiusPixels));
    const float radius2 = radius * radius;
    return selectSurfaceProjected(cloud, mvp, viewport, includeDeleted, depthToleranceNdc, [=](float x, float y) {
        const float dx = x - cx, dy = y - cy;
        return dx * dx + dy * dy <= radius2;
    });
}

std::vector<PointId> CpuSelector::lassoSurface(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                               const std::vector<Point2i>& polygon, float depthToleranceNdc,
                                               bool includeDeleted) {
    if (polygon.size() < 3)
        return {};
    return selectSurfaceProjected(cloud, mvp, viewport, includeDeleted, depthToleranceNdc,
                                  [&](float x, float y) { return pointInPolygon(x, y, polygon); });
}

std::vector<PointId> CpuSelector::brushStrokeSurface(const PointCloud& cloud, const Mat4f& mvp,
                                                     const Viewport& viewport, const std::vector<Point2i>& path,
                                                     int radiusPixels, float depthToleranceNdc, bool includeDeleted) {
    if (path.empty())
        return {};
    const float radius = static_cast<float>(std::max(0, radiusPixels));
    const float radius2 = radius * radius;
    return selectSurfaceProjected(cloud, mvp, viewport, includeDeleted, depthToleranceNdc, [&](float x, float y) {
        if (path.size() == 1) {
            const float dx = x - static_cast<float>(path.front().x);
            const float dy = y - static_cast<float>(path.front().y);
            return dx * dx + dy * dy <= radius2;
        }
        for (std::size_t i = 1; i < path.size(); ++i)
            if (distanceSquaredToSegment(x, y, static_cast<float>(path[i - 1].x), static_cast<float>(path[i - 1].y),
                                         static_cast<float>(path[i].x), static_cast<float>(path[i].y)) <= radius2)
                return true;
        return false;
    });
}

std::vector<PointId> CpuSelector::brushStroke(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                              const std::vector<Point2i>& path, int radiusPixels, bool includeDeleted) {
    if (path.empty())
        return {};
    const float radius = static_cast<float>(std::max(0, radiusPixels));
    const float radius2 = radius * radius;

    return selectProjected(cloud, mvp, viewport, includeDeleted, [&](float x, float y) {
        if (path.size() == 1) {
            const float dx = x - static_cast<float>(path.front().x);
            const float dy = y - static_cast<float>(path.front().y);
            return dx * dx + dy * dy <= radius2;
        }

        for (std::size_t i = 1; i < path.size(); ++i) {
            if (distanceSquaredToSegment(x, y, static_cast<float>(path[i - 1].x), static_cast<float>(path[i - 1].y),
                                         static_cast<float>(path[i].x), static_cast<float>(path[i].y)) <= radius2) {
                return true;
            }
        }
        return false;
    });
}

} // namespace pceditor
