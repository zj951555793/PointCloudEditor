#pragma once

#include "TriangleMesh.h"
#include <cstddef>
#include <vector>

namespace JMEngine {

// Surface Picking 在远距离时，部分亚像素三角形可能没有写入 Picking FBO。
// 本工具只在当前选择区域候选集合内，从真正可见的 TriangleId 种子沿“共享边”做有限扩展，
// 用于补齐同一表面的零碎漏选；不会退回 PointId 删除，也不会无界穿透到模型背面。
struct MeshSelectionClosureOptions {
    std::size_t maxRings{12};
    // 相邻三角形法向夹角限制。0.35 ~= 69.5 度。
    float minAdjacentNormalDot{0.35f};
};

class MeshSelectionClosure {
  public:
    static std::vector<TriangleId> expandSurfaceSelection(const TriangleMesh& mesh,
                                                          const std::vector<TriangleId>& seeds,
                                                          const std::vector<TriangleId>& regionCandidates,
                                                          const MeshSelectionClosureOptions& options = {});
};

} // namespace JMEngine
