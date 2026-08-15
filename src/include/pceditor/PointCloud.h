#pragma once

#include "Types.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace pceditor {

// 每个点的状态位。
// 使用 bit flag 是为了让删除/选择操作只修改少量字节，不频繁搬移大数组。
enum PointFlags : std::uint8_t {
    PointValid = 1u << 0,    // 点数据有效。
    PointSelected = 1u << 1, // 可选状态位；当前 Selection 类才是正式选择集合。
    PointDeleted = 1u << 2   // 软删除标记；真正物理删除由 compact() 完成。
};

// 单个点的数据结构。
// rgba 采用 32 位打包颜色，具体通道顺序由渲染端自行约定。
struct Point {
    Vec3f position{};
    std::uint32_t rgba{0xffffffffu};
    std::uint8_t flags{PointValid};

    // 可选法向量。很多扫描 PLY 会直接携带 nx/ny/nz。
    // 全 0 表示“文件没有提供有效法向”；渲染端应退化为纯顶点颜色显示。
    // 把 normal 放在末尾是为了兼容旧代码中 {position, rgba, flags} 的聚合初始化写法。
    Vec3f normal{};
};

// 点云数据容器。
// 设计原则：编辑期间 PointId 尽量稳定，因此删除采用软删除；只有 compact() 会改变 ID。
class PointCloud {
  public:
    using Container = std::vector<Point>;

    PointCloud() = default;
    explicit PointCloud(Container points);

    std::size_t size() const noexcept {
        return points_.size();
    }
    bool empty() const noexcept {
        return points_.empty();
    }

    // 带边界检查访问；越界会抛 std::out_of_range。
    Point& at(PointId id);
    const Point& at(PointId id) const;

    // 不抛异常访问；PointId 无效时返回 nullptr。
    Point* tryGet(PointId id) noexcept;
    const Point* tryGet(PointId id) const noexcept;

    // 如需高性能批量遍历可直接访问底层数组。
    Container& points() noexcept {
        return points_;
    }
    const Container& points() const noexcept {
        return points_;
    }

    // 当前未被软删除的点数。
    std::size_t activeCount() const noexcept;
    std::size_t deletedCount() const noexcept;

    // 物理移除所有 PointDeleted 点并压缩数组。
    // 注意：调用后旧 PointId 会失效。
    // 返回 oldId -> newId 映射；已删除点对应 kInvalidPointId。
    std::vector<PointId> compact();

  private:
    Container points_;
};

} // namespace pceditor
