#include <JMEngine/edit/MeshEditSession.h>
#include <JMEngine/edit/DirtyRange.h>

#include <algorithm>
#include <iterator>
#include <unordered_set>
#include <utility>

namespace JMEngine {

struct MeshEditSession::ICommand {
    virtual ~ICommand() = default;
    virtual EditResult execute(TriangleMesh& mesh) = 0;
    virtual EditResult undo(TriangleMesh& mesh) = 0;
};

namespace {

struct TriangleFlagChange {
    TriangleId id{};
    std::uint8_t before{};
    std::uint8_t after{};
};

class TriangleFlagCommand final : public MeshEditSession::ICommand {
  public:
    explicit TriangleFlagCommand(std::vector<TriangleFlagChange> changes) : changes_(std::move(changes)) {}

    EditResult execute(TriangleMesh& mesh) override {
        std::vector<std::uint32_t> ids;
        ids.reserve(changes_.size());
        auto& flags = mesh.triangleFlags();
        for (const auto& c : changes_) {
            const auto i = static_cast<std::size_t>(c.id);
            if (i >= flags.size())
                continue;
            flags[i] = c.after;
            ids.push_back(c.id);
        }
        EditResult r;
        r.changed = !ids.empty();
        r.topologyChanged = r.changed;
        r.triangleFlagRanges = makeDirtyRanges(std::move(ids));
        return r;
    }

    EditResult undo(TriangleMesh& mesh) override {
        std::vector<std::uint32_t> ids;
        ids.reserve(changes_.size());
        auto& flags = mesh.triangleFlags();
        for (const auto& c : changes_) {
            const auto i = static_cast<std::size_t>(c.id);
            if (i >= flags.size())
                continue;
            flags[i] = c.before;
            ids.push_back(c.id);
        }
        EditResult r;
        r.changed = !ids.empty();
        r.topologyChanged = r.changed;
        r.triangleFlagRanges = makeDirtyRanges(std::move(ids));
        return r;
    }

  private:
    std::vector<TriangleFlagChange> changes_;
};

class MeshTransformCommand final : public MeshEditSession::ICommand {
  public:
    MeshTransformCommand(std::vector<PointId> ids, Mat4f matrix) : ids_(std::move(ids)), matrix_(matrix) {}

    EditResult execute(TriangleMesh& mesh) override {
        old_.clear();
        auto cloud = mesh.vertices();
        if (!cloud)
            return {};
        old_.reserve(ids_.size());
        for (auto id : ids_) {
            auto* p = cloud->tryGet(id);
            if (!p || (p->flags & PointDeleted))
                continue;
            old_.push_back({id, p->position});
            p->position = transformPoint(matrix_, p->position);
        }
        return result();
    }

    EditResult undo(TriangleMesh& mesh) override {
        auto cloud = mesh.vertices();
        if (!cloud)
            return {};
        for (const auto& v : old_) {
            if (auto* p = cloud->tryGet(v.id))
                p->position = v.pos;
        }
        return result();
    }

  private:
    struct Old {
        PointId id;
        Vec3f pos;
    };
    EditResult result() const {
        std::vector<std::uint32_t> ids;
        ids.reserve(old_.size());
        for (const auto& v : old_)
            ids.push_back(v.id);
        EditResult r;
        r.changed = !ids.empty();
        r.geometryChanged = r.changed;
        r.pointPositionRanges = makeDirtyRanges(std::move(ids));
        return r;
    }
    std::vector<PointId> ids_;
    Mat4f matrix_{};
    std::vector<Old> old_;
};

} // namespace

MeshEditSession::MeshEditSession() : mesh_(std::make_shared<TriangleMesh>()) {}

MeshEditSession::MeshEditSession(std::shared_ptr<TriangleMesh> mesh) {
    setMesh(std::move(mesh));
}

MeshEditSession::~MeshEditSession() = default;

void MeshEditSession::setMesh(std::shared_ptr<TriangleMesh> mesh) {
    mesh_ = mesh ? std::move(mesh) : std::make_shared<TriangleMesh>();
    selection_.clear();
    clearHistory();
}

void MeshEditSession::normalizeSelection() {
    if (!std::is_sorted(selection_.begin(), selection_.end()))
        std::sort(selection_.begin(), selection_.end());
    selection_.erase(std::unique(selection_.begin(), selection_.end()), selection_.end());
    const auto n = mesh_ ? mesh_->triangleCount() : 0u;
    selection_.erase(std::remove_if(selection_.begin(), selection_.end(),
                                    [n](TriangleId id) { return static_cast<std::size_t>(id) >= n; }),
                     selection_.end());
}

void MeshEditSession::select(std::vector<TriangleId> ids) {
    selection_ = std::move(ids);
    normalizeSelection();
}
void MeshEditSession::addSelection(const std::vector<TriangleId>& ids) {
    selection_.insert(selection_.end(), ids.begin(), ids.end());
    normalizeSelection();
}
void MeshEditSession::removeSelection(const std::vector<TriangleId>& ids) {
    std::vector<TriangleId> rhs = ids;
    std::sort(rhs.begin(), rhs.end());
    rhs.erase(std::unique(rhs.begin(), rhs.end()), rhs.end());
    std::vector<TriangleId> out;
    out.reserve(selection_.size());
    std::set_difference(selection_.begin(), selection_.end(), rhs.begin(), rhs.end(), std::back_inserter(out));
    selection_.swap(out);
}
void MeshEditSession::clearSelection() {
    selection_.clear();
}
void MeshEditSession::invertSelection() {
    if (!mesh_)
        return;
    std::vector<TriangleId> out;
    out.reserve(mesh_->activeTriangleCount());
    std::size_t s = 0;
    for (TriangleId id = 0; static_cast<std::size_t>(id) < mesh_->triangleCount(); ++id) {
        if (!mesh_->triangleActive(id))
            continue;
        while (s < selection_.size() && selection_[s] < id)
            ++s;
        if (s >= selection_.size() || selection_[s] != id)
            out.push_back(id);
    }
    selection_.swap(out);
}

EditResult MeshEditSession::execute(std::unique_ptr<ICommand> cmd) {
    if (!mesh_ || !cmd)
        return {};
    auto r = cmd->execute(*mesh_);
    if (!r.changed)
        return r;
    undo_.push_back(std::move(cmd));
    redo_.clear();
    return r;
}

EditResult MeshEditSession::deleteSelection() {
    if (!mesh_ || selection_.empty())
        return {};
    std::vector<TriangleFlagChange> changes;
    changes.reserve(selection_.size());
    auto& flags = mesh_->triangleFlags();
    for (auto id : selection_) {
        const auto i = static_cast<std::size_t>(id);
        if (i >= flags.size() || (flags[i] & TriangleDeleted))
            continue;
        changes.push_back({id, flags[i], static_cast<std::uint8_t>((flags[i] | TriangleDeleted) & ~TriangleSelected)});
    }
    selection_.clear();
    if (changes.empty())
        return {};
    return execute(std::make_unique<TriangleFlagCommand>(std::move(changes)));
}

EditResult MeshEditSession::keepSelection() {
    if (!mesh_ || selection_.empty())
        return {};
    std::unordered_set<TriangleId> keep(selection_.begin(), selection_.end());
    std::vector<TriangleFlagChange> changes;
    auto& flags = mesh_->triangleFlags();
    for (TriangleId id = 0; static_cast<std::size_t>(id) < flags.size(); ++id) {
        if (flags[id] & TriangleDeleted)
            continue;
        if (keep.find(id) != keep.end())
            continue;
        changes.push_back(
            {id, flags[id], static_cast<std::uint8_t>((flags[id] | TriangleDeleted) & ~TriangleSelected)});
    }
    selection_.clear();
    if (changes.empty())
        return {};
    return execute(std::make_unique<TriangleFlagCommand>(std::move(changes)));
}

EditResult MeshEditSession::transformSelection(const Mat4f& matrix) {
    if (!mesh_ || selection_.empty())
        return {};
    const auto& idx = mesh_->indices();
    std::vector<PointId> vertexIds;
    vertexIds.reserve(selection_.size() * 3u);
    for (auto tid : selection_) {
        const auto b = static_cast<std::size_t>(tid) * 3u;
        if (b + 2u >= idx.size() || !mesh_->triangleActive(tid))
            continue;
        vertexIds.push_back(idx[b]);
        vertexIds.push_back(idx[b + 1u]);
        vertexIds.push_back(idx[b + 2u]);
    }
    std::sort(vertexIds.begin(), vertexIds.end());
    vertexIds.erase(std::unique(vertexIds.begin(), vertexIds.end()), vertexIds.end());
    if (vertexIds.empty())
        return {};
    return execute(std::make_unique<MeshTransformCommand>(std::move(vertexIds), matrix));
}

EditResult MeshEditSession::undo() {
    if (!mesh_ || undo_.empty())
        return {};
    auto cmd = std::move(undo_.back());
    undo_.pop_back();
    auto r = cmd->undo(*mesh_);
    redo_.push_back(std::move(cmd));
    return r;
}
EditResult MeshEditSession::redo() {
    if (!mesh_ || redo_.empty())
        return {};
    auto cmd = std::move(redo_.back());
    redo_.pop_back();
    auto r = cmd->execute(*mesh_);
    undo_.push_back(std::move(cmd));
    return r;
}
void MeshEditSession::clearHistory() {
    undo_.clear();
    redo_.clear();
}

std::vector<TriangleId> MeshEditSession::compactTriangles() {
    if (!mesh_)
        return {};
    auto map = mesh_->compactTriangles();
    std::vector<TriangleId> remapped;
    remapped.reserve(selection_.size());
    for (auto id : selection_) {
        if (id < map.size() && map[id] != kInvalidTriangleId)
            remapped.push_back(map[id]);
    }
    selection_.swap(remapped);
    clearHistory();
    return map;
}

} // namespace JMEngine
