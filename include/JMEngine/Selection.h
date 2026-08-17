#pragma once

#include "Types.h"
#include <algorithm>
#include <iterator> // std::back_inserter 定义在此头文件中，不能依赖其它头文件间接包含。
#include <utility>
#include <vector>

namespace JMEngine {

// 选择集合。
// 内部始终保持“有序 + 去重”，这样集合差集和后续批量操作都更稳定。
class Selection {
  public:
    const std::vector<PointId>& ids() const noexcept {
        return ids_;
    }
    bool empty() const noexcept {
        return ids_.empty();
    }
    std::size_t size() const noexcept {
        return ids_.size();
    }

    void clear() {
        ids_.clear();
    }

    // 完全替换选择集。
    void set(std::vector<PointId> ids) {
        ids_ = std::move(ids);
        normalize();
    }

    // 增量加入点 ID；重复 ID 会被自动去除。
    void add(const std::vector<PointId>& ids) {
        ids_.insert(ids_.end(), ids.begin(), ids.end());
        normalize();
    }

    // 从当前选择集中移除指定 ID。
    void remove(const std::vector<PointId>& ids) {
        std::vector<PointId> rhs = ids;
        std::sort(rhs.begin(), rhs.end());
        rhs.erase(std::unique(rhs.begin(), rhs.end()), rhs.end());

        std::vector<PointId> out;
        out.reserve(ids_.size());
        std::set_difference(ids_.begin(), ids_.end(), rhs.begin(), rhs.end(), std::back_inserter(out));
        ids_.swap(out);
    }

    // compact() 后根据 oldId -> newId 映射更新选择集。
    // 已被删除的点会自动从选择集中移除。
    void remap(const std::vector<PointId>& oldToNew) {
        std::vector<PointId> out;
        out.reserve(ids_.size());
        for (auto id : ids_) {
            if (id < oldToNew.size()) {
                const auto newId = oldToNew[id];
                if (newId != kInvalidPointId)
                    out.push_back(newId);
            }
        }
        ids_.swap(out);
        normalize();
    }

  private:
    // 统一排序并去重，保证 Selection 的内部不变量。
    void normalize() {
        // GPU Picking 返回值通常已经有序；先判断可避免百万级选择再次 O(N log N) 排序。
        if (!std::is_sorted(ids_.begin(), ids_.end())) {
            std::sort(ids_.begin(), ids_.end());
        }
        ids_.erase(std::unique(ids_.begin(), ids_.end()), ids_.end());
    }

    std::vector<PointId> ids_;
};

} // namespace JMEngine
