#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace pceditor {

// OBJ 三角网格拓扑。
// triangleIndices 每 3 个索引构成一个三角形，索引指向 PointCloud 中的顶点。
struct ObjMeshData {
    std::vector<std::uint32_t> triangleIndices;

    bool empty() const noexcept {
        return triangleIndices.empty();
    }
    std::size_t triangleCount() const noexcept {
        return triangleIndices.size() / 3u;
    }
};

// 轻量 OBJ 网格拓扑加载器。
// 完全不依赖 Qt/VTK/PCL。启用 PCEDITOR_USE_OPENMP 时，面解析阶段使用 OpenMP。
class ObjMeshLoader {
  public:
    static bool load(const std::string& objFile, std::size_t vertexCount, ObjMeshData& out,
                     std::string* message = nullptr);
};

} // namespace pceditor
