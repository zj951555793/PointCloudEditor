#pragma once

#include "PointCloud.h"
#include <vector>

namespace pceditor {

// CPU 选择器所需的视口大小。
struct Viewport {
    int width{1};
    int height{1};
};

// 屏幕矩形，输入坐标原点约定为左上角。
struct RectI {
    int x1{0};
    int y1{0};
    int x2{0};
    int y2{0};
};

// 屏幕整数点。套索、画刷路径都使用该结构，避免核心库依赖 Qt 的 QPoint。
struct Point2i {
    int x{0};
    int y{0};
};

// 纯 CPU 屏幕选择工具。
//
// 说明：
// - 不依赖 OpenGL / Qt / VTK，Windows 原生测试和离线工具可直接使用；
// - 每次调用都会遍历点云，因此千万点实时编辑优先使用 GPU ID Picking；
// - 屏幕坐标统一约定左上角为原点，与 Qt/Win32 鼠标坐标一致。
class CpuSelector {
  public:
    static std::vector<PointId> rectangle(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                          const RectI& rect, bool includeDeleted = false);

    // 真正的 CPU 表面选择：先建立软件深度图，再只返回位于最前表面附近的点。
    // depthToleranceNdc 是 [0,1] 深度空间容差，用于把同一物理表面的亚像素密集点一起选中。
    static std::vector<PointId> rectangleSurface(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                                 const RectI& rect, float depthToleranceNdc = 0.0025f,
                                                 bool includeDeleted = false);

    static std::vector<PointId> circleSurface(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                              const Point2i& center, int radiusPixels,
                                              float depthToleranceNdc = 0.0025f, bool includeDeleted = false);

    static std::vector<PointId> lassoSurface(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                             const std::vector<Point2i>& polygon, float depthToleranceNdc = 0.0025f,
                                             bool includeDeleted = false);

    static std::vector<PointId> brushStrokeSurface(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                                   const std::vector<Point2i>& path, int radiusPixels,
                                                   float depthToleranceNdc = 0.0025f, bool includeDeleted = false);

    // 圆形选择：center 为屏幕圆心，radiusPixels 为像素半径。
    static std::vector<PointId> circle(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                       const Point2i& center, int radiusPixels, bool includeDeleted = false);

    // 套索选择：polygon 至少 3 个点，内部使用奇偶规则判断点是否在多边形内。
    static std::vector<PointId> lasso(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                      const std::vector<Point2i>& polygon, bool includeDeleted = false);

    // 画刷路径选择：命中距离任意路径线段 <= radiusPixels 的屏幕点。
    // Brush 删除通常先调用该函数得到 PointId[]，再交给 PointCloudEditor 删除。
    static std::vector<PointId> brushStroke(const PointCloud& cloud, const Mat4f& mvp, const Viewport& viewport,
                                            const std::vector<Point2i>& path, int radiusPixels,
                                            bool includeDeleted = false);
};

} // namespace pceditor
