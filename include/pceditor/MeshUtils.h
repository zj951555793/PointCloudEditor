#pragma once

#include "TriangleMesh.h"

namespace pceditor {

// 在最终网格拓扑上重建面积加权顶点法线。
// 必须在 Trim/Cleanup/删除小连通域等拓扑操作之后调用。
bool recomputeVertexNormals(TriangleMesh& mesh) noexcept;

} // namespace pceditor
