#include <pceditor/TriangleMesh.h>

#include <algorithm>

namespace pceditor {

TriangleMesh::TriangleMesh() : vertices_(std::make_shared<PointCloud>()) {}

TriangleMesh::TriangleMesh(std::shared_ptr<PointCloud> vertices, std::vector<std::uint32_t> indices)
    : vertices_(vertices ? std::move(vertices) : std::make_shared<PointCloud>()), indices_(std::move(indices)) {
    indices_.resize((indices_.size() / 3u) * 3u);
    normalizeFlags();
}

void TriangleMesh::setVertices(std::shared_ptr<PointCloud> vertices) {
    vertices_ = vertices ? std::move(vertices) : std::make_shared<PointCloud>();
}

void TriangleMesh::setIndices(std::vector<std::uint32_t> indices) {
    indices_ = std::move(indices);
    indices_.resize((indices_.size() / 3u) * 3u);
    normalizeFlags();
}

void TriangleMesh::normalizeFlags() {
    triangleFlags_.assign(triangleCount(), TriangleValid);
}

bool TriangleMesh::triangleActive(TriangleId id) const noexcept {
    const auto i = static_cast<std::size_t>(id);
    if (i >= triangleFlags_.size() || (triangleFlags_[i] & TriangleDeleted) != 0)
        return false;

    // 网格顶点与 PointCloud 共享。点显示模式删除 PointId 后，任何引用已删除点的三角形
    // 都必须视为不可见，否则切回实体/线框时会把已经删除的点重新连成面。
    if (!vertices_)
        return false;
    const auto base = i * 3u;
    if (base + 2u >= indices_.size())
        return false;
    const auto& points = vertices_->points();
    for (std::size_t k = 0; k < 3u; ++k) {
        const auto vertexId = static_cast<std::size_t>(indices_[base + k]);
        if (vertexId >= points.size() || (points[vertexId].flags & PointDeleted) != 0)
            return false;
    }
    return true;
}

std::size_t TriangleMesh::activeTriangleCount() const noexcept {
    std::size_t active = 0;
    for (std::size_t i = 0; i < triangleCount(); ++i)
        if (triangleActive(static_cast<TriangleId>(i)))
            ++active;
    return active;
}

std::size_t TriangleMesh::deletedTriangleCount() const noexcept {
    return triangleCount() - activeTriangleCount();
}

VisibleTriangleBuffer TriangleMesh::buildVisibleBuffer() const {
    VisibleTriangleBuffer out;
    const auto active = activeTriangleCount();
    out.indices.reserve(active * 3u);
    out.triangleIds.reserve(active);
    const auto count = triangleCount();
    for (std::size_t t = 0; t < count; ++t) {
        if (!triangleActive(static_cast<TriangleId>(t)))
            continue;
        const auto b = t * 3u;
        out.indices.push_back(indices_[b]);
        out.indices.push_back(indices_[b + 1u]);
        out.indices.push_back(indices_[b + 2u]);
        out.triangleIds.push_back(static_cast<TriangleId>(t));
    }
    return out;
}

std::vector<std::uint32_t> TriangleMesh::buildVisibleIndices() const {
    return buildVisibleBuffer().indices;
}

std::vector<TriangleId> TriangleMesh::compactTriangles() {
    const auto oldCount = triangleCount();
    std::vector<TriangleId> oldToNew(oldCount, kInvalidTriangleId);
    std::vector<std::uint32_t> compacted;
    compacted.reserve(activeTriangleCount() * 3u);

    TriangleId newId = 0;
    for (std::size_t t = 0; t < oldCount; ++t) {
        if ((triangleFlags_[t] & TriangleDeleted) != 0)
            continue;
        oldToNew[t] = newId++;
        const auto b = t * 3u;
        compacted.insert(compacted.end(), {indices_[b], indices_[b + 1u], indices_[b + 2u]});
    }
    indices_.swap(compacted);
    normalizeFlags();
    return oldToNew;
}

} // namespace pceditor
