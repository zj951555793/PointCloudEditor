#include <JMEngine/MeshSelectionClosure.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

namespace JMEngine {
namespace {

struct EdgeKey {
    std::uint32_t a{};
    std::uint32_t b{};
    bool operator==(const EdgeKey& o) const noexcept {
        return a == o.a && b == o.b;
    }
};

struct EdgeHash {
    std::size_t operator()(const EdgeKey& e) const noexcept {
        return (static_cast<std::size_t>(e.a) << 32u) ^ static_cast<std::size_t>(e.b);
    }
};

EdgeKey makeEdge(std::uint32_t a, std::uint32_t b) noexcept {
    return a < b ? EdgeKey{a, b} : EdgeKey{b, a};
}

Vec3f sub(const Vec3f& a, const Vec3f& b) noexcept {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3f cross(const Vec3f& a, const Vec3f& b) noexcept {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

float dot(const Vec3f& a, const Vec3f& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3f triangleNormal(const TriangleMesh& mesh, TriangleId tid) noexcept {
    const auto cloud = mesh.vertices();
    if (!cloud)
        return {};
    const auto& idx = mesh.indices();
    const std::size_t b = static_cast<std::size_t>(tid) * 3u;
    if (b + 2u >= idx.size())
        return {};
    const auto i0 = idx[b], i1 = idx[b + 1u], i2 = idx[b + 2u];
    if (i0 >= cloud->size() || i1 >= cloud->size() || i2 >= cloud->size())
        return {};
    auto n = cross(sub(cloud->points()[i1].position, cloud->points()[i0].position),
                   sub(cloud->points()[i2].position, cloud->points()[i0].position));
    const float len = std::sqrt(dot(n, n));
    if (len <= 1.0e-12f)
        return {};
    n.x /= len;
    n.y /= len;
    n.z /= len;
    return n;
}

bool normalCompatible(const Vec3f& a, const Vec3f& b, float minDot) noexcept {
    const float la = dot(a, a), lb = dot(b, b);
    if (la <= 1.0e-12f || lb <= 1.0e-12f)
        return true;
    return dot(a, b) >= minDot;
}

} // namespace

std::vector<TriangleId> MeshSelectionClosure::expandSurfaceSelection(const TriangleMesh& mesh,
                                                                     const std::vector<TriangleId>& seeds,
                                                                     const std::vector<TriangleId>& regionCandidates,
                                                                     const MeshSelectionClosureOptions& options) {
    if (seeds.empty() || regionCandidates.empty() || mesh.empty())
        return seeds;

    const std::size_t triCount = mesh.triangleCount();
    std::vector<std::uint8_t> candidateMask(triCount, 0u);
    for (auto tid : regionCandidates) {
        if (static_cast<std::size_t>(tid) < triCount && mesh.triangleActive(tid))
            candidateMask[tid] = 1u;
    }
    // GPU seed 可能因边界数值误差没有出现在 CPU 候选中；种子本身永远保留。
    for (auto tid : seeds) {
        if (static_cast<std::size_t>(tid) < triCount && mesh.triangleActive(tid))
            candidateMask[tid] = 1u;
    }

    const auto& idx = mesh.indices();
    std::unordered_map<EdgeKey, std::vector<TriangleId>, EdgeHash> edgeToTriangles;
    edgeToTriangles.reserve(regionCandidates.size() * 2u + 16u);
    for (TriangleId tid = 0; static_cast<std::size_t>(tid) < triCount; ++tid) {
        if (!candidateMask[tid])
            continue;
        const std::size_t b = static_cast<std::size_t>(tid) * 3u;
        if (b + 2u >= idx.size())
            continue;
        edgeToTriangles[makeEdge(idx[b], idx[b + 1u])].push_back(tid);
        edgeToTriangles[makeEdge(idx[b + 1u], idx[b + 2u])].push_back(tid);
        edgeToTriangles[makeEdge(idx[b + 2u], idx[b])].push_back(tid);
    }

    std::vector<std::uint8_t> selected(triCount, 0u);
    std::vector<Vec3f> normals(triCount);
    std::vector<std::uint8_t> normalReady(triCount, 0u);
    auto getNormal = [&](TriangleId tid) -> const Vec3f& {
        if (!normalReady[tid]) {
            normals[tid] = triangleNormal(mesh, tid);
            normalReady[tid] = 1u;
        }
        return normals[tid];
    };

    std::deque<std::pair<TriangleId, std::size_t>> queue;
    for (auto tid : seeds) {
        if (static_cast<std::size_t>(tid) >= triCount || !mesh.triangleActive(tid))
            continue;
        if (!selected[tid]) {
            selected[tid] = 1u;
            queue.push_back({tid, 0u});
        }
    }

    const std::size_t maxRings = std::max<std::size_t>(1u, options.maxRings);
    const float minDot = std::clamp(options.minAdjacentNormalDot, -1.0f, 1.0f);
    while (!queue.empty()) {
        const auto [tid, ring] = queue.front();
        queue.pop_front();
        if (ring >= maxRings)
            continue;
        const std::size_t b = static_cast<std::size_t>(tid) * 3u;
        if (b + 2u >= idx.size())
            continue;
        const EdgeKey edges[3] = {makeEdge(idx[b], idx[b + 1u]), makeEdge(idx[b + 1u], idx[b + 2u]),
                                  makeEdge(idx[b + 2u], idx[b])};
        const Vec3f& n0 = getNormal(tid);
        for (const auto& edge : edges) {
            const auto it = edgeToTriangles.find(edge);
            if (it == edgeToTriangles.end())
                continue;
            for (auto nb : it->second) {
                if (nb == tid || selected[nb] || !candidateMask[nb])
                    continue;
                if (!normalCompatible(n0, getNormal(nb), minDot))
                    continue;
                selected[nb] = 1u;
                queue.push_back({nb, ring + 1u});
            }
        }
    }

    std::vector<TriangleId> out;
    out.reserve(seeds.size() * 2u + 16u);
    for (TriangleId tid = 0; static_cast<std::size_t>(tid) < triCount; ++tid)
        if (selected[tid])
            out.push_back(tid);
    return out;
}

} // namespace JMEngine
