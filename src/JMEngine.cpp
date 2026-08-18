#include "JMEngine/JMEngine.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace JMEngine {
namespace {

class FlagsCommand final : public ICommand {
  public:
    FlagsCommand(PointCloud& cloud, std::vector<PointId> ids, bool deleted)
        : cloud_(cloud), ids_(std::move(ids)), deleted_(deleted) {
        previousFlags_.reserve(ids_.size());
        for (const auto id : ids_) {
            const auto* point = cloud_.tryGet(id);
            previousFlags_.push_back(point ? point->flags : 0);
        }
    }

    void execute(PointCloud&) override {
        for (const auto id : ids_) {
            auto* point = cloud_.tryGet(id);
            if (!point)
                continue;
            if (deleted_)
                point->flags = std::uint8_t(point->flags | PointDeleted);
            else
                point->flags = std::uint8_t(point->flags & ~PointDeleted);
        }
    }

    void undo(PointCloud&) override {
        for (std::size_t index = 0; index < ids_.size(); ++index) {
            if (auto* point = cloud_.tryGet(ids_[index]))
                point->flags = previousFlags_[index];
        }
    }

    std::vector<PointId> affectedIds() const override {
        return ids_;
    }

    ChangeKind changeKind() const noexcept override {
        return ChangeKind::Flags;
    }

  private:
    PointCloud& cloud_;
    std::vector<PointId> ids_;
    std::vector<std::uint8_t> previousFlags_;
    bool deleted_{false};
};

class TransformCommand final : public ICommand {
  public:
    TransformCommand(PointCloud& cloud, std::vector<PointId> ids,
                     const Mat4f& matrix)
        : cloud_(cloud), ids_(std::move(ids)), matrix_(matrix) {
        previousPositions_.reserve(ids_.size());
        for (const auto id : ids_) {
            const auto* point = cloud_.tryGet(id);
            previousPositions_.push_back(point ? point->position : Vec3f{});
        }
    }

    void execute(PointCloud&) override {
        for (const auto id : ids_) {
            if (auto* point = cloud_.tryGet(id))
                point->position = transformPoint(matrix_, point->position);
        }
    }

    void undo(PointCloud&) override {
        for (std::size_t index = 0; index < ids_.size(); ++index) {
            if (auto* point = cloud_.tryGet(ids_[index]))
                point->position = previousPositions_[index];
        }
    }

    std::vector<PointId> affectedIds() const override {
        return ids_;
    }

    ChangeKind changeKind() const noexcept override {
        return ChangeKind::Position;
    }

  private:
    PointCloud& cloud_;
    std::vector<PointId> ids_;
    std::vector<Vec3f> previousPositions_;
    Mat4f matrix_;
};

} // namespace

Engine::Engine() = default;

Engine::Engine(std::shared_ptr<PointCloud> cloud)
    : cloud_(std::move(cloud)) {}

void Engine::setPointCloud(std::shared_ptr<PointCloud> cloud) {
    selection_.clear();
    clearHistory();
    cloud_ = std::move(cloud);
}

void Engine::select(std::vector<PointId> ids) {
    selection_.set(std::move(ids));
}

void Engine::addSelection(const std::vector<PointId>& ids) {
    selection_.add(ids);
}

void Engine::removeSelection(const std::vector<PointId>& ids) {
    selection_.remove(ids);
}

void Engine::clearSelection() {
    selection_.clear();
}

void Engine::invertSelection() {
    if (!cloud_)
        return;

    std::vector<PointId> ids;
    ids.reserve(cloud_->activeCount());
    const auto& selected = selection_.ids();
    for (PointId id = 0; std::size_t(id) < cloud_->size(); ++id) {
        const auto& point = cloud_->points()[id];
        if ((point.flags & PointDeleted) == 0 &&
            !std::binary_search(selected.begin(), selected.end(), id)) {
            ids.push_back(id);
        }
    }
    selection_.set(std::move(ids));
}

bool Engine::execute(std::unique_ptr<ICommand> command) {
    if (!cloud_ || !command)
        return false;

    command->execute(*cloud_);
    lastChangedIds_ = command->affectedIds();
    lastChangeKind_ = command->changeKind();
    undo_.push_back(std::move(command));
    redo_.clear();
    return !lastChangedIds_.empty();
}

bool Engine::deleteSelection() {
    if (!cloud_ || selection_.empty())
        return false;

    std::vector<PointId> ids;
    for (const auto id : selection_.ids()) {
        const auto* point = cloud_->tryGet(id);
        if (point && (point->flags & PointDeleted) == 0)
            ids.push_back(id);
    }
    return !ids.empty() && execute(
        std::make_unique<FlagsCommand>(*cloud_, std::move(ids), true));
}

bool Engine::keepSelection() {
    if (!cloud_)
        return false;

    std::vector<PointId> ids;
    const auto& keep = selection_.ids();
    for (PointId id = 0; std::size_t(id) < cloud_->size(); ++id) {
        if ((cloud_->points()[id].flags & PointDeleted) == 0 &&
            !std::binary_search(keep.begin(), keep.end(), id)) {
            ids.push_back(id);
        }
    }
    return !ids.empty() && execute(
        std::make_unique<FlagsCommand>(*cloud_, std::move(ids), true));
}

bool Engine::crop(const Box3f& box) {
    if (!cloud_)
        return false;

    std::vector<PointId> ids;
    for (PointId id = 0; std::size_t(id) < cloud_->size(); ++id) {
        const auto& point = cloud_->points()[id];
        if ((point.flags & PointDeleted) == 0 &&
            !box.contains(point.position)) {
            ids.push_back(id);
        }
    }
    return !ids.empty() && execute(
        std::make_unique<FlagsCommand>(*cloud_, std::move(ids), true));
}

bool Engine::transform(const Mat4f& matrix) {
    if (!cloud_ || selection_.empty())
        return false;

    std::vector<PointId> ids;
    for (const auto id : selection_.ids()) {
        const auto* point = cloud_->tryGet(id);
        if (point && (point->flags & PointDeleted) == 0)
            ids.push_back(id);
    }
    return !ids.empty() && execute(
        std::make_unique<TransformCommand>(*cloud_, std::move(ids), matrix));
}

bool Engine::undo() {
    if (!cloud_ || undo_.empty())
        return false;

    auto command = std::move(undo_.back());
    undo_.pop_back();
    command->undo(*cloud_);
    lastChangedIds_ = command->affectedIds();
    lastChangeKind_ = command->changeKind();
    redo_.push_back(std::move(command));
    return true;
}

bool Engine::redo() {
    if (!cloud_ || redo_.empty())
        return false;

    auto command = std::move(redo_.back());
    redo_.pop_back();
    command->execute(*cloud_);
    lastChangedIds_ = command->affectedIds();
    lastChangeKind_ = command->changeKind();
    undo_.push_back(std::move(command));
    return true;
}

void Engine::clearHistory() {
    undo_.clear();
    redo_.clear();
    lastChangedIds_.clear();
    lastChangeKind_ = ChangeKind::None;
}

std::vector<PointId> Engine::compact() {
    if (!cloud_)
        return {};

    auto mapping = cloud_->compact();
    selection_.remap(mapping);
    clearHistory();
    return mapping;
}

} // namespace JMEngine
