#pragma once

#include "CpuSelector.h"
#include "TriangleMesh.h"
#include <vector>

namespace pceditor {

// CPU 三角面选择器。
// 与 CpuSelector 的“点落在区域内”不同，这里按屏幕空间三角形与选择区域是否相交判断，
// 因此不会再出现“只命中一个共享顶点就把相邻整片面扩散选中”的问题。
class CpuMeshSelector {
  public:
    static std::vector<TriangleId> rectangle(const TriangleMesh& mesh, const Mat4f& mvp, const Viewport& viewport,
                                             const RectI& rect, bool includeDeleted = false);

    static std::vector<TriangleId> circle(const TriangleMesh& mesh, const Mat4f& mvp, const Viewport& viewport,
                                          const Point2i& center, int radiusPixels, bool includeDeleted = false);

    static std::vector<TriangleId> lasso(const TriangleMesh& mesh, const Mat4f& mvp, const Viewport& viewport,
                                         const std::vector<Point2i>& polygon, bool includeDeleted = false);

    static std::vector<TriangleId> brushStroke(const TriangleMesh& mesh, const Mat4f& mvp, const Viewport& viewport,
                                               const std::vector<Point2i>& path, int radiusPixels,
                                               bool includeDeleted = false);

    // 真正的“表面选择”：软件 Z-buffer 只保留每个像素最前面的 TriangleId。
    // 与上面的 rectangle/circle/lasso/brushStroke（穿透选择）严格区分。
    static std::vector<TriangleId> surfaceRectangle(const TriangleMesh& mesh, const Mat4f& mvp,
                                                    const Viewport& viewport, const RectI& rect);

    static std::vector<TriangleId> surfaceCircle(const TriangleMesh& mesh, const Mat4f& mvp, const Viewport& viewport,
                                                 const Point2i& center, int radiusPixels);

    static std::vector<TriangleId> surfaceLasso(const TriangleMesh& mesh, const Mat4f& mvp, const Viewport& viewport,
                                                const std::vector<Point2i>& polygon);

    static std::vector<TriangleId> surfaceBrushStroke(const TriangleMesh& mesh, const Mat4f& mvp,
                                                      const Viewport& viewport, const std::vector<Point2i>& path,
                                                      int radiusPixels);
};

} // namespace pceditor
