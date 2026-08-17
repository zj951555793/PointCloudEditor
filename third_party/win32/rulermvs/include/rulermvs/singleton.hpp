#ifndef _RULERMVS_CORE_SINGLETON_HPP_
#define _RULERMVS_CORE_SINGLETON_HPP_
#include <mutex>
namespace rulermvs
{
/// @brief 单例
template <typename T> class Singleton {
public:
    template <typename... Args> static T* instance(Args&&... args)
    {
        if (instance_ == nullptr) {
            std::lock_guard<std::mutex> lock {lock_};
            // std::lock_guard<std::mutex> lock {globalLock()};
            if (instance_ == nullptr) {
                // 占位，否则内存释放不能动态执行
                singleton_auto_delete_.nothing_need_to_do();
                // 借助临时指针变量实现线程安全
                T* ptmp   = new T(std::forward<Args>(args)...);
                instance_ = ptmp;
            }
        }
        return instance_;
    }

    static void destroy()
    {
        if (!instance_) return;
        delete instance_;
        instance_ = nullptr;
    }

protected:
    Singleton() = default;
    virtual ~Singleton() {}

private:
    static std::mutex lock_;
    static T*         instance_;

    Singleton(const Singleton&)            = delete;
    Singleton& operator=(const Singleton&) = delete;

    static struct _SingletonAutoDelete {
        inline void nothing_need_to_do() const {}
        virtual ~_SingletonAutoDelete() { destroy(); }
    } singleton_auto_delete_;
};
template <typename T> std::mutex Singleton<T>::lock_;
template <typename T> T*         Singleton<T>::instance_ = nullptr;
template <typename T> typename Singleton<T>::_SingletonAutoDelete
    Singleton<T>::singleton_auto_delete_;
}  // namespace rulermvs
#endif