#pragma once

#include "PointCloud.h"
#include "Selection.h"

#include <memory>
#include <vector>

namespace JMEngine {

// 最近一次编辑改变了哪类 GPU 数据。
// 渲染后端据此只同步 flags 或 position，避免无意义的大块上传。
enum class ChangeKind { None, Flags, Position, Full };

// 所有可撤销编辑命令的统一接口。
class ICommand {
  public:
    virtual ~ICommand() = default;

    // 首次执行或 Redo 时调用。
    virtual void execute(PointCloud& cloud) = 0;

    // Undo 时恢复执行前状态。
    virtual void undo(PointCloud& cloud) = 0;

    // 返回本命令影响到的 PointId。
    // 渲染端可以据此只更新 GPU 的脏块，而不是重建整片 VBO。
    virtual std::vector<PointId> affectedIds() const = 0;
    virtual ChangeKind changeKind() const noexcept = 0;
};

// 点云编辑库的主入口。
//
// 重要设计约束：
// 1. 不依赖 Qt / VTK / PCL / OpenGL；
// 2. 图形前端只负责把用户操作转换成 PointId[]；
// 3. 删除采用软删除，普通编辑不会改变 PointId；
// 4. compact() 才真正压缩数组并改变 PointId；
// 5. lastChangedIds() 用于千万点场景 GPU 局部同步。
class Engine {
  public:
    Engine();
    explicit Engine(std::shared_ptr<PointCloud> cloud);

    void setPointCloud(std::shared_ptr<PointCloud> cloud);
    std::shared_ptr<PointCloud> pointCloud() const noexcept {
        return cloud_;
    }

    // -----------------------------
    // 选择集操作
    // -----------------------------
    const Selection& selection() const noexcept {
        return selection_;
    }
    void select(std::vector<PointId> ids);
    void addSelection(const std::vector<PointId>& ids);
    void removeSelection(const std::vector<PointId>& ids);
    void clearSelection();
    void invertSelection();

    // -----------------------------
    // 编辑操作
    // -----------------------------
    bool deleteSelection();
    bool keepSelection();
    bool crop(const Box3f& box);
    bool transform(const Mat4f& matrix);

    // -----------------------------
    // Undo / Redo
    // -----------------------------
    bool undo();
    bool redo();
    bool canUndo() const noexcept {
        return !undo_.empty();
    }
    bool canRedo() const noexcept {
        return !redo_.empty();
    }
    void clearHistory();

    // 最近一次 execute/undo/redo 实际影响到的 PointId。
    // 该列表始终按命令真实影响范围返回，渲染线程可转换成 dirty ranges。
    const std::vector<PointId>& lastChangedIds() const noexcept {
        return lastChangedIds_;
    }
    ChangeKind lastChangeKind() const noexcept {
        return lastChangeKind_;
    }

    // -----------------------------
    // 物理压缩
    // -----------------------------
    std::vector<PointId> compact();

  private:
    bool execute(std::unique_ptr<ICommand> cmd);

    std::shared_ptr<PointCloud> cloud_;
    Selection selection_;
    std::vector<std::unique_ptr<ICommand>> undo_;
    std::vector<std::unique_ptr<ICommand>> redo_;
    std::vector<PointId> lastChangedIds_;
    ChangeKind lastChangeKind_{ChangeKind::None};
};

} // namespace JMEngine
