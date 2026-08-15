#pragma once

#include "PointCloud.h"
#include "Types.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace pceditor {

enum TriangleFlags : std::uint8_t { TriangleValid = 1u << 0, TriangleSelected = 1u << 1, TriangleDeleted = 1u << 2 };

// Core 层三角网格。
// 顶点直接共享 PointCloud，避免 OBJ 加载后再复制一份 positions/colors/normals。
struct VisibleTriangleBuffer {
    std::vector<std::uint32_t> indices;
    std::vector<TriangleId> triangleIds; // packed primitive -> original TriangleId
};

class TriangleMesh {
  public:
    TriangleMesh();
    TriangleMesh(std::shared_ptr<PointCloud> vertices, std::vector<std::uint32_t> indices);

    void setVertices(std::shared_ptr<PointCloud> vertices);
    std::shared_ptr<PointCloud> vertices() const noexcept {
        return vertices_;
    }

    void setIndices(std::vector<std::uint32_t> indices);
    std::vector<std::uint32_t>& indices() noexcept {
        return indices_;
    }
    const std::vector<std::uint32_t>& indices() const noexcept {
        return indices_;
    }

    std::size_t triangleCount() const noexcept {
        return indices_.size() / 3u;
    }
    bool empty() const noexcept {
        return triangleCount() == 0;
    }

    std::vector<std::uint8_t>& triangleFlags() noexcept {
        return triangleFlags_;
    }
    const std::vector<std::uint8_t>& triangleFlags() const noexcept {
        return triangleFlags_;
    }

    bool triangleActive(TriangleId id) const noexcept;
    std::size_t activeTriangleCount() const noexcept;
    std::size_t deletedTriangleCount() const noexcept;

    // 生成 Renderer 可直接上传的可见 EBO；只过滤 TriangleDeleted，不复制顶点。
    VisibleTriangleBuffer buildVisibleBuffer() const;
    std::vector<std::uint32_t> buildVisibleIndices() const;

    // 只压缩三角形，不改变 PointId。返回 old TriangleId -> new TriangleId。
    std::vector<TriangleId> compactTriangles();

  private:
    void normalizeFlags();

    std::shared_ptr<PointCloud> vertices_;
    std::vector<std::uint32_t> indices_;
    std::vector<std::uint8_t> triangleFlags_;
};

} // namespace pceditor
