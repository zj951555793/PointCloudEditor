#include "JMEngine/FrameQueue.h"

#include <algorithm>
#include <utility>

namespace JMEngine {

FrameQueue::FrameQueue(std::size_t capacity)
    : capacity_(std::max<std::size_t>(1, capacity)) {}

void FrameQueue::setCapacity(std::size_t capacity) {
    std::lock_guard<std::mutex> lock(mutex_);
    capacity_ = std::max<std::size_t>(1, capacity);
    while (queue_.size() > capacity_)
        queue_.pop_front();
    condition_.notify_all();
}


bool FrameQueue::pushSequential(CameraFrame frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_)
        return false;

    if (queue_.size() >= capacity_)
        return false;

    queue_.push_back(std::move(frame));
    condition_.notify_one();
    return true;
}


bool FrameQueue::pushBlocking(CameraFrame frame) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] {
        return closed_ || queue_.size() < capacity_;
    });
    if (closed_)
        return false;

    queue_.push_back(std::move(frame));
    condition_.notify_all();
    return true;
}

bool FrameQueue::pushLatest(CameraFrame frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_)
        return false;

    bool replaced = false;
    while (queue_.size() >= capacity_) {
        queue_.pop_front();
        replaced = true;
    }
    queue_.push_back(std::move(frame));
    condition_.notify_one();
    return replaced;
}

bool FrameQueue::pop(CameraFrame& frame) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] { return closed_ || !queue_.empty(); });
    if (queue_.empty())
        return false;

    frame = std::move(queue_.front());
    queue_.pop_front();
    // Wake a lossless dataset producer waiting for queue capacity.
    condition_.notify_all();
    return true;
}

void FrameQueue::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
    condition_.notify_all();
}

void FrameQueue::reopen() {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = false;
}

void FrameQueue::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    condition_.notify_all();
}

std::size_t FrameQueue::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
}

} // namespace JMEngine
