#pragma once

#include <array>
#include <cstdint>
#include <limits>

namespace JMEngine {

// 点云内部统一使用 32 位无符号整数作为点 ID。
// 在 compact() 之前，PointId 与 points() 数组下标保持一致。
using PointId = std::uint32_t;
using TriangleId = std::uint32_t;

// 无效点 ID。GPU Picking 的背景像素也使用这个值，便于过滤。
constexpr PointId kInvalidPointId = std::numeric_limits<PointId>::max();
constexpr TriangleId kInvalidTriangleId = std::numeric_limits<TriangleId>::max();

// 二维浮点向量。主要用于屏幕坐标、纹理坐标等轻量场景。
struct Vec2f {
    float x{0.0f};
    float y{0.0f};
};

// 三维浮点向量。点云坐标统一使用 float，可降低千万点场景内存占用。
struct Vec3f {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

// 4x4 矩阵，采用 OpenGL 风格的列主序存储。
// m[12]、m[13]、m[14] 分别对应 X/Y/Z 平移分量。
struct Mat4f {
    std::array<float, 16> m{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

    static Mat4f identity() {
        return {};
    }
};

// 轴对齐包围盒 AABB。
struct Box3f {
    Vec3f min;
    Vec3f max;

    // 判断点是否位于包围盒内部；边界上的点也视为内部点。
    bool contains(const Vec3f& p) const noexcept {
        return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y && p.z >= min.z && p.z <= max.z;
    }
};

// 使用齐次坐标执行 4x4 矩阵变换。
// 对透视矩阵也兼容：当 w 不为 0/1 时会自动进行齐次除法。
inline Vec3f transformPoint(const Mat4f& a, const Vec3f& p) noexcept {
    const float x = a.m[0] * p.x + a.m[4] * p.y + a.m[8] * p.z + a.m[12];
    const float y = a.m[1] * p.x + a.m[5] * p.y + a.m[9] * p.z + a.m[13];
    const float z = a.m[2] * p.x + a.m[6] * p.y + a.m[10] * p.z + a.m[14];
    const float w = a.m[3] * p.x + a.m[7] * p.y + a.m[11] * p.z + a.m[15];

    if (w != 0.0f && w != 1.0f) {
        return {x / w, y / w, z / w};
    }
    return {x, y, z};
}

} // namespace JMEngine
