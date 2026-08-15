#pragma once

#include "Types.h"
#include <vector>

namespace pceditor {

// 与渲染后端无关的 GPU Picking 接口。
// OpenGL/GLES 可用 R32UI 附件实现；Vulkan 可使用整数颜色附件实现。
class IGpuPicker {
  public:
    virtual ~IGpuPicker() = default;

    // 获取单个屏幕像素对应的 PointId；背景返回 kInvalidPointId。
    virtual PointId pickPoint(int x, int y) = 0;

    // 获取矩形区域内出现过的所有 PointId；实现应返回去重后的结果。
    virtual std::vector<PointId> pickRectangle(int x1, int y1, int x2, int y2) = 0;
};

} // namespace pceditor
