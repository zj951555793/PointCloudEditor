#pragma once

#include "ScanTypes.h"

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>

namespace JMEngine {

class FrameQueue {
  public:
    explicit FrameQueue(std::size_t capacity = 1);

    void setCapacity(std::size_t capacity);
    // Preserve chronological continuity: when full, reject the incoming
    // newest frame instead of evicting an older frame already waiting.
    bool pushSequential(CameraFrame frame);
    bool pushLatest(CameraFrame frame);
    bool pop(CameraFrame& frame);
    void close();
    void reopen();
    void clear();
    std::size_t size() const;

  private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<CameraFrame> queue_;
    std::size_t capacity_{1};
    bool closed_{false};
};

} // namespace JMEngine
