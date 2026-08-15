#pragma once

#include <pceditor/TriangleMesh.h>
#include <pceditor/edit/EditResult.h>

#include <memory>
#include <vector>

namespace pceditor {

// Mesh 编辑选择集专门使用 TriangleId，避免“选中一个面 -> 删除三个共享顶点”的误伤。
class MeshEditSession {
  public:
    MeshEditSession();
    explicit MeshEditSession(std::shared_ptr<TriangleMesh> mesh);
    ~MeshEditSession();

    void setMesh(std::shared_ptr<TriangleMesh> mesh);
    std::shared_ptr<TriangleMesh> mesh() const noexcept {
        return mesh_;
    }

    const std::vector<TriangleId>& selection() const noexcept {
        return selection_;
    }
    void select(std::vector<TriangleId> ids);
    void addSelection(const std::vector<TriangleId>& ids);
    void removeSelection(const std::vector<TriangleId>& ids);
    void clearSelection();
    void invertSelection();

    EditResult deleteSelection();
    EditResult keepSelection();
    EditResult transformSelection(const Mat4f& matrix);

    EditResult undo();
    EditResult redo();
    bool canUndo() const noexcept {
        return !undo_.empty();
    }
    bool canRedo() const noexcept {
        return !redo_.empty();
    }
    void clearHistory();

    // 物理压缩已删除三角形；PointId 不变化。
    // 返回 old TriangleId -> new TriangleId。
    std::vector<TriangleId> compactTriangles();

  public:
    struct ICommand;

  private:
    EditResult execute(std::unique_ptr<ICommand> cmd);
    void normalizeSelection();

    std::shared_ptr<TriangleMesh> mesh_;
    std::vector<TriangleId> selection_;
    std::vector<std::unique_ptr<ICommand>> undo_;
    std::vector<std::unique_ptr<ICommand>> redo_;
};

} // namespace pceditor
