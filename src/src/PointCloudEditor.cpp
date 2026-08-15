#include <pceditor/PointCloudEditor.h>

#include <algorithm>
#include <unordered_set>
#include <utility>

namespace pceditor {
namespace {

// 一次 flags 修改的最小历史记录。
// 对删除/保留/裁剪而言，只需要保存“修改前 flags”和“修改后 flags”，
// 不需要复制 position/color，因此大点云 Undo 的内存开销明显更低。
struct FlagChange {
    PointId id{};
    std::uint8_t before{};
    std::uint8_t after{};
};

// 通用 flags 修改命令。
// deleteSelection()/keepSelection()/crop() 都可以复用它。
class FlagCommand final : public ICommand {
  public:
    explicit FlagCommand(std::vector<FlagChange> changes) : changes_(std::move(changes)) {}

    void execute(PointCloud& cloud) override {
        for (const auto& change : changes_) {
            if (auto* point = cloud.tryGet(change.id)) {
                point->flags = change.after;
            }
        }
    }

    void undo(PointCloud& cloud) override {
        for (const auto& change : changes_) {
            if (auto* point = cloud.tryGet(change.id)) {
                point->flags = change.before;
            }
        }
    }

    std::vector<PointId> affectedIds() const override {
        std::vector<PointId> ids;
        ids.reserve(changes_.size());
        for (const auto& change : changes_)
            ids.push_back(change.id);
        return ids;
    }

    ChangeKind changeKind() const noexcept override {
        return ChangeKind::Flags;
    }

  private:
    std::vector<FlagChange> changes_;
};

// 点坐标变换命令。
// 与 FlagCommand 不同，变换必须保存旧 position，Undo 才能精确恢复。
class TransformCommand final : public ICommand {
  public:
    TransformCommand(std::vector<PointId> ids, Mat4f matrix) : ids_(std::move(ids)), matrix_(matrix) {}

    void execute(PointCloud& cloud) override {
        // Redo 时 execute() 会再次调用，所以必须先清掉上一次保存的旧坐标。
        old_.clear();
        old_.reserve(ids_.size());

        for (const PointId id : ids_) {
            if (auto* point = cloud.tryGet(id)) {
                // 已删除点不参与几何变换。
                if ((point->flags & PointDeleted) != 0)
                    continue;

                old_.push_back({id, point->position});
                point->position = transformPoint(matrix_, point->position);
            }
        }
    }

    void undo(PointCloud& cloud) override {
        for (const auto& item : old_) {
            if (auto* point = cloud.tryGet(item.id)) {
                point->position = item.position;
            }
        }
    }

    std::vector<PointId> affectedIds() const override {
        std::vector<PointId> ids;
        ids.reserve(old_.size());
        for (const auto& item : old_)
            ids.push_back(item.id);
        // 首次 execute 之前 old_ 为空，这时使用计划变换的 ids_。
        if (ids.empty())
            ids = ids_;
        return ids;
    }

    ChangeKind changeKind() const noexcept override {
        return ChangeKind::Position;
    }

  private:
    struct OldPosition {
        PointId id{};
        Vec3f position{};
    };

    std::vector<PointId> ids_;
    Mat4f matrix_{};
    std::vector<OldPosition> old_;
};

// 收集所有未软删除点的 PointId。
// 当 transform() 没有 selection 时，默认对整片有效点云进行变换。
std::vector<PointId> activeIds(const PointCloud& cloud) {
    std::vector<PointId> ids;
    ids.reserve(cloud.activeCount());

    for (PointId id = 0; static_cast<std::size_t>(id) < cloud.size(); ++id) {
        const auto& point = cloud.points()[id];
        if ((point.flags & PointDeleted) == 0)
            ids.push_back(id);
    }
    return ids;
}

} // namespace

// 默认创建一份空点云，保证 pointCloud() 正常情况下不返回空指针。
PointCloudEditor::PointCloudEditor() : cloud_(std::make_shared<PointCloud>()) {}

PointCloudEditor::PointCloudEditor(std::shared_ptr<PointCloud> cloud) {
    setPointCloud(std::move(cloud));
}

void PointCloudEditor::setPointCloud(std::shared_ptr<PointCloud> cloud) {
    cloud_ = cloud ? std::move(cloud) : std::make_shared<PointCloud>();

    // 新旧点云的 PointId 没有任何关系，因此必须清理选择和历史。
    selection_.clear();
    clearHistory();
    lastChangedIds_.clear();
    lastChangeKind_ = ChangeKind::None;
}

void PointCloudEditor::select(std::vector<PointId> ids) {
    selection_.set(std::move(ids));
}

void PointCloudEditor::addSelection(const std::vector<PointId>& ids) {
    selection_.add(ids);
}

void PointCloudEditor::removeSelection(const std::vector<PointId>& ids) {
    selection_.remove(ids);
}

void PointCloudEditor::clearSelection() {
    selection_.clear();
}

void PointCloudEditor::invertSelection() {
    if (!cloud_)
        return;
    const auto current = selection_.ids();
    std::vector<PointId> out;
    out.reserve(cloud_->activeCount());
    std::size_t s = 0;
    for (PointId id = 0; static_cast<std::size_t>(id) < cloud_->size(); ++id) {
        if ((cloud_->points()[id].flags & PointDeleted) != 0)
            continue;
        while (s < current.size() && current[s] < id)
            ++s;
        if (s >= current.size() || current[s] != id)
            out.push_back(id);
    }
    selection_.set(std::move(out));
}

bool PointCloudEditor::execute(std::unique_ptr<ICommand> cmd) {
    if (!cmd || !cloud_)
        return false;

    // 新命令成功执行后进入 Undo 栈。
    cmd->execute(*cloud_);
    lastChangedIds_ = cmd->affectedIds();
    lastChangeKind_ = cmd->changeKind();
    undo_.push_back(std::move(cmd));

    // 一旦用户在 Undo 之后执行了新操作，旧 Redo 分支就失效。
    redo_.clear();
    return true;
}

bool PointCloudEditor::deleteSelection() {
    if (!cloud_ || selection_.empty())
        return false;

    std::vector<FlagChange> changes;
    changes.reserve(selection_.size());

    for (const PointId id : selection_.ids()) {
        if (auto* point = cloud_->tryGet(id)) {
            // 重复删除同一个已删除点没有意义，也不应该产生空历史命令。
            if ((point->flags & PointDeleted) != 0)
                continue;

            const auto before = point->flags;
            const auto after = static_cast<std::uint8_t>((before | PointDeleted) & ~PointSelected);
            changes.push_back({id, before, after});
        }
    }

    // 删除完成后 UI 一般应取消选择，避免视觉上保留“已选但已删除”的状态。
    selection_.clear();

    if (changes.empty())
        return false;
    return execute(std::make_unique<FlagCommand>(std::move(changes)));
}

bool PointCloudEditor::keepSelection() {
    if (!cloud_ || selection_.empty())
        return false;

    // unordered_set 用于 O(1) 平均复杂度判断“当前点是否属于保留集合”。
    std::unordered_set<PointId> keep(selection_.ids().begin(), selection_.ids().end());

    std::vector<FlagChange> changes;
    changes.reserve(cloud_->size());

    for (PointId id = 0; static_cast<std::size_t>(id) < cloud_->size(); ++id) {
        auto& point = cloud_->points()[id];
        if ((point.flags & PointDeleted) != 0)
            continue;
        if (keep.find(id) != keep.end())
            continue;

        const auto before = point.flags;
        const auto after = static_cast<std::uint8_t>((before | PointDeleted) & ~PointSelected);
        changes.push_back({id, before, after});
    }

    selection_.clear();
    if (changes.empty())
        return false;
    return execute(std::make_unique<FlagCommand>(std::move(changes)));
}

bool PointCloudEditor::crop(const Box3f& box) {
    if (!cloud_)
        return false;

    std::vector<FlagChange> changes;

    // 常规裁剪通常只删除一部分点，预留 1/8 避免一开始直接按整片点云分配。
    changes.reserve(cloud_->size() / 8 + 1);

    for (PointId id = 0; static_cast<std::size_t>(id) < cloud_->size(); ++id) {
        auto& point = cloud_->points()[id];
        if ((point.flags & PointDeleted) != 0)
            continue;
        if (box.contains(point.position))
            continue;

        const auto before = point.flags;
        const auto after = static_cast<std::uint8_t>((before | PointDeleted) & ~PointSelected);
        changes.push_back({id, before, after});
    }

    selection_.clear();
    if (changes.empty())
        return false;
    return execute(std::make_unique<FlagCommand>(std::move(changes)));
}

bool PointCloudEditor::transform(const Mat4f& matrix) {
    if (!cloud_)
        return false;

    // 有选择集时只变换选择集；无选择集时默认变换全部有效点。
    auto ids = selection_.empty() ? activeIds(*cloud_) : selection_.ids();
    if (ids.empty())
        return false;

    return execute(std::make_unique<TransformCommand>(std::move(ids), matrix));
}

bool PointCloudEditor::undo() {
    if (!cloud_ || undo_.empty())
        return false;

    auto cmd = std::move(undo_.back());
    undo_.pop_back();
    cmd->undo(*cloud_);
    lastChangedIds_ = cmd->affectedIds();
    lastChangeKind_ = cmd->changeKind();
    redo_.push_back(std::move(cmd));
    return true;
}

bool PointCloudEditor::redo() {
    if (!cloud_ || redo_.empty())
        return false;

    auto cmd = std::move(redo_.back());
    redo_.pop_back();
    cmd->execute(*cloud_);
    lastChangedIds_ = cmd->affectedIds();
    lastChangeKind_ = cmd->changeKind();
    undo_.push_back(std::move(cmd));
    return true;
}

void PointCloudEditor::clearHistory() {
    // unique_ptr 清空时会自动释放所有命令对象及其增量历史数据。
    undo_.clear();
    redo_.clear();
    lastChangedIds_.clear();
    lastChangeKind_ = ChangeKind::None;
}

std::vector<PointId> PointCloudEditor::compact() {
    if (!cloud_)
        return {};

    auto mapping = cloud_->compact();

    // Selection 可根据 old->new 映射自动修复。
    selection_.remap(mapping);

    // Undo/Redo 命令保存的是 compact 前 PointId，无法安全继续使用，所以必须清空。
    clearHistory();
    // compact 之后所有点都可能换了 ID，渲染端必须做一次完整重建。
    lastChangedIds_.clear();
    lastChangeKind_ = ChangeKind::Full;
    return mapping;
}

} // namespace pceditor
