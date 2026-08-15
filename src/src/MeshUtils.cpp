#include <pceditor/MeshUtils.h>

#include <cmath>

namespace pceditor {
namespace {
Vec3f cross(const Vec3f& a, const Vec3f& b) noexcept {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
Vec3f sub(const Vec3f& a, const Vec3f& b) noexcept {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
} // namespace

bool recomputeVertexNormals(TriangleMesh& mesh) noexcept {
    auto cloud = mesh.vertices();
    if (!cloud || cloud->empty())
        return false;
    auto& points = cloud->points();
    for (auto& p : points)
        p.normal = {};

    const auto& idx = mesh.indices();
    std::size_t validFaces = 0;
    for (std::size_t i = 0; i + 2 < idx.size(); i += 3) {
        const auto triId = static_cast<TriangleId>(i / 3);
        if (!mesh.triangleActive(triId))
            continue;
        const auto a = idx[i], b = idx[i + 1], c = idx[i + 2];
        if (a >= points.size() || b >= points.size() || c >= points.size())
            continue;
        const Vec3f n = cross(sub(points[b].position, points[a].position), sub(points[c].position, points[a].position));
        const float l2 = n.x * n.x + n.y * n.y + n.z * n.z;
        if (!(l2 > 1e-24f) || !std::isfinite(l2))
            continue;
        points[a].normal.x += n.x;
        points[a].normal.y += n.y;
        points[a].normal.z += n.z;
        points[b].normal.x += n.x;
        points[b].normal.y += n.y;
        points[b].normal.z += n.z;
        points[c].normal.x += n.x;
        points[c].normal.y += n.y;
        points[c].normal.z += n.z;
        ++validFaces;
    }
    std::size_t validNormals = 0;
    for (auto& p : points) {
        const float l2 = p.normal.x * p.normal.x + p.normal.y * p.normal.y + p.normal.z * p.normal.z;
        if (l2 > 1e-24f && std::isfinite(l2)) {
            const float inv = 1.0f / std::sqrt(l2);
            p.normal.x *= inv;
            p.normal.y *= inv;
            p.normal.z *= inv;
            ++validNormals;
        } else {
            p.normal = {};
        }
    }
    return validFaces > 0 && validNormals > 0;
}

} // namespace pceditor
