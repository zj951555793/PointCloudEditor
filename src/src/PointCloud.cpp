#include <pceditor/PointCloud.h>

#include <stdexcept>
#include <utility>

namespace pceditor {

// 直接接管调用方传入的点数组，避免一次无意义的大点云复制。
PointCloud::PointCloud(Container points) : points_(std::move(points)) {}

Point& PointCloud::at(PointId id) {
    // PointId 本质上是数组下标，但对外接口仍做边界保护。
    if (static_cast<std::size_t>(id) >= points_.size()) {
        throw std::out_of_range("PointId out of range");
    }
    return points_[id];
}

const Point& PointCloud::at(PointId id) const {
    if (static_cast<std::size_t>(id) >= points_.size()) {
        throw std::out_of_range("PointId out of range");
    }
    return points_[id];
}

Point* PointCloud::tryGet(PointId id) noexcept {
    // 编辑命令批量遍历时更适合 tryGet：无效 ID 直接返回 nullptr，不使用异常做流程控制。
    return static_cast<std::size_t>(id) < points_.size() ? &points_[id] : nullptr;
}

const Point* PointCloud::tryGet(PointId id) const noexcept {
    return static_cast<std::size_t>(id) < points_.size() ? &points_[id] : nullptr;
}

std::size_t PointCloud::activeCount() const noexcept {
    std::size_t count = 0;
    for (const auto& point : points_) {
        if ((point.flags & PointDeleted) == 0)
            ++count;
    }
    return count;
}

std::size_t PointCloud::deletedCount() const noexcept {
    return points_.size() - activeCount();
}

std::vector<PointId> PointCloud::compact() {
    // mapping 保存 compact 前后的 PointId 对应关系。
    // 删除点保持 kInvalidPointId，外部缓存可据此清理旧引用。
    std::vector<PointId> mapping(points_.size(), kInvalidPointId);

    Container compacted;
    compacted.reserve(activeCount());

    PointId newId = 0;
    for (PointId oldId = 0; static_cast<std::size_t>(oldId) < points_.size(); ++oldId) {
        auto& point = points_[oldId];
        if ((point.flags & PointDeleted) != 0)
            continue;

        // Selection 类才是正式的选择集合；物理压缩时清掉 PointSelected 临时位。
        point.flags = static_cast<std::uint8_t>(point.flags & ~PointSelected);
        mapping[oldId] = newId++;
        compacted.push_back(point);
    }

    // 一次 swap 完成物理替换，避免逐点 erase 导致 O(N^2) 搬移。
    points_.swap(compacted);
    return mapping;
}

} // namespace pceditor
