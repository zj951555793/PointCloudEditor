#ifndef _RULERMVS_CORE_THREAD_POOL_HPP_
#define _RULERMVS_CORE_THREAD_POOL_HPP_
#include <queue>
#include <atomic>
#include <future>
#include <thread>
#include <stdexcept>
#include <functional>
#include <condition_variable>
namespace rulermvs
{
/// thread pool class for multi thread operate
class ThreadPool {
public:
    using Task = std::function<void()>;
    ThreadPool(size_t thread_num = std::thread::hardware_concurrency())
        : stoped_ {false}, idle_num_ {thread_num}
    {
        for (size_t i = 0; i < thread_num; ++i)
            pool_.emplace_back([this]() {
                while (!this->stoped_) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock {this->lock_};
                        this->cond_task_.wait(lock, [this]() {
                            return this->stoped_.load() ||
                                   !(this->tasks_.empty() &&
                                       this->front_tasks_.empty());
                        });
                        if (this->stoped_ && this->tasks_.empty() &&
                            this->front_tasks_.empty())
                            return;
                        task = std::move(!front_tasks_.empty() ?
                                             this->front_tasks_.front() :
                                             this->tasks_.front());
                        !front_tasks_.empty() ? this->front_tasks_.pop() :
                                                this->tasks_.pop();
                    }

                    idle_num_--;
                    task();
                    idle_num_++;
                }
            });
    }
    virtual ~ThreadPool()
    {
        stoped_.store(true);
        cond_task_.notify_all();
        for (std::thread& thread : pool_)
            if (thread.joinable()) thread.join();
        std::queue<Task> empty;
        std::swap(tasks_, empty);
    }
    /// 有两种方法可以实现调用类成员 ： std::bind 和 std::mem_fn
    template <class F, class... Args> auto commit(F&& f, Args&&... args)
        -> std::future<decltype(f(args...))>
    {
        if (stoped_.load()) throw std::runtime_error("ThreadPool is stopped.");
        using RetType = decltype(f(args...));
        auto task     = std::make_shared<std::packaged_task<RetType()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...));
        std::future<RetType> future = task->get_future();
        {
            std::lock_guard<std::mutex> lock {lock_};
            tasks_.emplace([task]() { (*task)(); });
        }
        cond_task_.notify_one();
        return future;
    }
    /// 有两种方法可以实现调用类成员 ： std::bind 和 std::mem_fn
    template <class F, class... Args> auto commit_front(F&& f, Args&&... args)
        -> std::future<decltype(f(args...))>
    {
        if (stoped_.load()) throw std::runtime_error("ThreadPool is stopped.");

        using RetType = decltype(f(args...));
        auto task     = std::make_shared<std::packaged_task<RetType()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...));
        std::future<RetType> future = task->get_future();
        {
            std::lock_guard<std::mutex> lock {lock_};
            front_tasks_.emplace([task]() { (*task)(); });
        }
        cond_task_.notify_one();
        return future;
    }
    /// 判断当前是否有任务在运行和等待
    bool is_running() const
    {
        return !tasks_.empty() || !front_tasks_.empty() ||
               idle_num_ != pool_.size();
    }
    /// 线程id列表
    inline std::vector<std::thread::id> getThreadID() const
    {
        std::vector<std::thread::id> ids;
        if (!pool_.empty()) {
            ids.resize(pool_.size());
            for (size_t i = 0; i < pool_.size(); ++i)
                ids[i] = pool_[i].get_id();
        }
        return ids;
    }
    inline size_t getIdleNum() const { return idle_num_; }

private:
    std::vector<std::thread> pool_;         ///< 线程池
    std::queue<Task>         tasks_;        ///< 任务队列
    std::queue<Task>         front_tasks_;  ///< 优先队列
    std::mutex               lock_;         ///< 锁
    std::condition_variable  cond_task_;    ///< 条件阻塞
    std::atomic<bool>        stoped_;       ///< 是否关闭提交
    std::atomic<size_t>      idle_num_;     ///< 空闲线程数量
};
}  // namespace rulermvs
#endif