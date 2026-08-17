#ifndef _RULERMVS_CORE_LOGGER_HPP_
#define _RULERMVS_CORE_LOGGER_HPP_
#include <set>
#include <time.h>
#include <string>
#include <thread>
#include <sstream>
#include <istream>
#include <iomanip>
#include <stdarg.h>
#include <algorithm>
namespace rulermvs
{
/// @brief 日志类型
enum LogType { LogNone = 0, LogFatal, LogError, LogWarn, LogInfo, LogDebug };

/// 日志接口类
struct ILogSink {
    /// @brief 占位析构函数
    virtual ~ILogSink() {}
    /// @brief 发送消息到IO
    virtual void send(int, const std::string&, const std::string&, int,
        const struct tm*, const std::string&) = 0;
    /// @brief 刷新
    virtual void wait_until() = 0;
};

/// @brief 日志实例,实现文件IO
class LogIOSink: public ILogSink {
public:
    LogIOSink() : level_(LogInfo), stream_(stdout) {}
    LogIOSink(int level) : level_(level), stream_(stdout) {}
    LogIOSink(const char* file_name, const char* mode, int level = LogInfo)
        : level_(level)
    {
#if defined(_MSC_VER)
        fopen_s(&stream_, file_name, mode);
#else
        stream_ = fopen(file_name, mode);
#endif
        if (stream_ == nullptr) stream_ = stdout;
    }
    virtual ~LogIOSink()
    {
        if (stream_ && stream_ != stdout) fclose(stream_);
    }
    void log(int, const char* fmt, ...)
    {
        va_list arglist;
        va_start(arglist, fmt);
        vfprintf(stream_, fmt, arglist);
        va_end(arglist);
    }
    void send(int        level, const std::string&, const std::string&, int,
        const struct tm* timeinfo, const std::string& message)
    {
        std::stringstream ss;
        ss << std::setw(6) << std::this_thread::get_id();
        auto time = std::put_time(timeinfo, "%b %d %Y %H:%M:%S");
        ss << " " << time << " " << getLogName(level) << message;
        log(level, "%s", ss.str().c_str());
    }
    void wait_until() { fflush(stream_); }
    // @brief 获取日志类型打印名称
    static inline std::string getLogName(int level)
    {
        if (level == LogNone) return "NONE  ";
        if (level == LogWarn) return "WARN  ";
        if (level == LogInfo) return "INFO  ";
        if (level == LogFatal) return "FATAL ";
        if (level == LogError) return "ERROR ";
        return "DEBUG ";
    }

protected:
    int   level_;
    FILE* stream_;
};

/// @brief 日志管理类
class Logger {
public:
    /// @brief 返回全局日志实例
    /// @return 返回Logger引用
    static Logger& instance()
    {
        static Logger logger;
        return logger;
    }
    void wait_for()
    {
        for_each(this->log_sinks_.begin(), this->log_sinks_.end(),
            [=](ILogSink* sink) { sink->wait_until(); });
    }
    void init_logging(char*) {}
    void add_log_sink(ILogSink* sink) { log_sinks_.insert(sink); }
    void remove_log_sink() { log_sinks_.clear(); }
    void remove_log_sink(ILogSink* sink) { log_sinks_.erase(sink); }
    void send_message(int level, const std::string& path,
        const std::string& name, int line, const struct tm* timeinfo,
        const std::string& message)
    {
        for_each(this->log_sinks_.begin(), this->log_sinks_.end(),
            [&](ILogSink* sink) -> void {
                sink->send(level, path, name, line, timeinfo, message);
            });
    }

protected:
    Logger() : log_sinks_({new LogIOSink()}) {}
    Logger(const Logger&) = delete;
    virtual ~Logger()
    {
        for_each(log_sinks_.begin(), log_sinks_.end(), [](ILogSink* sink) {
            if (sink) delete sink;
        });
        log_sinks_.clear();
    }

    std::set<ILogSink*> log_sinks_;
};

/// 消息类
class Message {
public:
    // construct function
    Message(const std::string& path, int line, const char* tag, int level)
        : tag_(tag), path_(path), line_(line), level_(level)
    {
        stream_ << getFileName(path_) << ":" << std::setw(4) << line_ << "   ";
    }
    // Output the contents of the stream to the proper channel on destruction.
    virtual ~Message()
    {
        stream_ << std::endl;
        /// 获取时间戳
        struct tm timeinfo;
        // consider thread safe
        time_t timeseconds = time(0);
#if defined(_MSC_VER) || defined(__MINGW32__)
        localtime_s(&timeinfo, &timeseconds);
#else
        localtime_r(&timeseconds, &timeinfo);
#endif
        Logger::instance().send_message(
            level_, path_, getFileName(path_), line_, &timeinfo, stream_.str());
        Logger::instance().wait_for();
    }
    static inline std::string getFileName(const std::string& path)
    {
        std::string name = path;
        std::replace(name.begin(), name.end(), '\\', '/');
        return name.substr(name.rfind('/') + 1, std::string::npos);
    }
    // Return the stream associated with the logger object.
    std::stringstream& stream() { return stream_; }

private:
    Message()               = delete;
    Message(const Message&) = delete;

    std::string       tag_;
    std::string       path_;
    int               line_;
    int               level_;
    std::stringstream stream_;
};
// This class is used to explicitly ignore values in the conditional
// logging macros.  This avoids compiler warnings like "value computed
// is not used" and "statement has no effect".
class LoggerVoidify {
public:
    LoggerVoidify() {}

    // This has to be an operator with a precedence lower than << but
    // higher than ?:
    void operator&(const std::ostream&) {}
};

// Log a message and terminate.
template <class T>
void logMessageFatal(const char* file, int line, const T& message)
{
    Message(file, line, "native", LogFatal).stream() << message;
}

// Helpers for CHECK_NOTNULL(). Two are necessary to support both raw pointers
// and smart pointers.
template <typename T>
T& checkNotNullCommon(const char* file, int line, const char* names, T& t)
{
    if (t == NULL) logMessageFatal(file, line, std::string(names));
    return t;
}
template <typename T>
T* checkNotNull(const char* file, int line, const char* names, T* t)
{
    return checkNotNullCommon(file, line, names, t);
}
template <typename T>
T& checkNotNull(const char* file, int line, const char* names, T& t)
{
    return checkNotNullCommon(file, line, names, t);
}
/** @} */
}  // namespace rulermvs

// ---------------------- Logging Macro definitions --------------------------
// LG is a convenient shortcut for LOG(INFO). Its use is in new
// google3 code is discouraged and the following shortcut exists for
// backward compatibility with existing code.
#define MVS_LOG(n) rulermvs::Message(__FILE__, __LINE__, "native", n).stream()
#define MVS_WLOG MVS_LOG(rulermvs::LogWarn)
#define MVS_ILOG MVS_LOG(rulermvs::LogInfo)

#ifndef NDEBUG
#define MVS_DLOG MVS_LOG(rulermvs::LogDebug)
#else
#define MVS_DLOG \
    if (false) MVS_LOG(rulermvs::LogDebug)
#endif

// Log only if condition is met.  Otherwise evaluates to void.
#define MVS_LOG_IF(n, condition) \
    !(condition) ? (void)0 : rulermvs::LoggerVoidify() & MVS_LOG(n)

// Log only if condition is NOT met.  Otherwise evaluates to void.
#define MVS_LOG_IF_FALSE(n, condition) MVS_LOG_IF(n, !(condition))
#define MVS_VLOG_IF(n, condition) MVS_LOG_IF(n, condition)
#define MVS_VLOG_IS_ON(x) (1)
// ---------------------------- CHECK macros ---------------------------------
// Check for a given boolean condition.
#define MVS_CHECK(condition)                        \
    MVS_LOG_IF_FALSE(rulermvs::LogFatal, condition) \
        << "Check Failed: " #condition " "

// Debug only version of CHECK
#ifndef NDEBUG
#define MVS_DCHECK(condition)                       \
    MVS_LOG_IF_FALSE(rulermvs::LogFatal, condition) \
        << "Check Failed: " #condition " "
#else
#define MVS_DCHECK(condition)                       \
    if (false)                                      \
    MVS_LOG_IF_FALSE(rulermvs::LogFatal, condition) \
        << "Check Failed: " #condition " "
#endif

// ------------------------- CHECK_OP macros ---------------------------------
// Generic binary operator check macro. This should not be directly invoked,
// instead use the binary comparison macros defined below.
#define MVS_CHECK_OP(val1, val2, op)                      \
    MVS_LOG_IF_FALSE(rulermvs::LogInfo, ((val1)op(val2))) \
        << "Check failed: " #val1 " " #op " " #val2 " "

// Check_op macro definitions
#define MVS_CHECK_EQ(val1, val2) MVS_CHECK_OP(val1, val2, ==)
#define MVS_CHECK_NE(val1, val2) MVS_CHECK_OP(val1, val2, !=)
#define MVS_CHECK_LE(val1, val2) MVS_CHECK_OP(val1, val2, <=)
#define MVS_CHECK_LT(val1, val2) MVS_CHECK_OP(val1, val2, <)
#define MVS_CHECK_GE(val1, val2) MVS_CHECK_OP(val1, val2, >=)
#define MVS_CHECK_GT(val1, val2) MVS_CHECK_OP(val1, val2, >)

/////////////////////////////////////////////////
/// Check_op macro definitions and return/////////
/////////////////////////////////////////////////
#define MVS_CHECK_OP_RETURN(val1, val2, op, ret, sw)                      \
    if (!((val1)op(val2))) {                                              \
        if (sw) {                                                         \
            LoggerVoidify() &                                             \
                Message(__FILE__, __LINE__, "native", rulermvs::LogFatal) \
                        .stream()                                         \
                    << "Check failed return: " #val1 " " #op " " #val2    \
                       " " #ret " ";                                      \
        }                                                                 \
        return ret;                                                       \
    }
#define MVS_CHECK_EQ_RETURN(val1, val2, ret) \
    MVS_CHECK_OP_RETURN(val1, val2, ==, ret, true)
#define MVS_CHECK_NE_RETURN(val1, val2, ret) \
    MVS_CHECK_OP_RETURN(val1, val2, !=, ret, true)
#define MVS_CHECK_LE_RETURN(val1, val2, ret) \
    MVS_CHECK_OP_RETURN(val1, val2, <=, ret, true)
#define MVS_CHECK_LT_RETURN(val1, val2, ret) \
    MVS_CHECK_OP_RETURN(val1, val2, <, ret, true)
#define MVS_CHECK_GE_RETURN(val1, val2, ret) \
    MVS_CHECK_OP_RETURN(val1, val2, >=, ret, true)
#define MVS_CHECK_GT_RETURN(val1, val2, ret) \
    MVS_CHECK_OP_RETURN(val1, val2, >, ret, true)
#define MVS_DCHECK_EQ_RETURN(val1, val2, ret) \
    MVS_CHECK_OP_RETURN(val1, val2, ==, ret, false)
#define MVS_DCHECK_NE_RETURN(val1, val2, ret) \
    MVS_CHECK_OP_RETURN(val1, val2, !=, ret, false)
#define MVS_DCHECK_LE_RETURN(val1, val2, ret) \
    MVS_CHECK_OP_RETURN(val1, val2, <=, ret, false)
#define MVS_DCHECK_LT_RETURN(val1, val2, ret) \
    MVS_CHECK_OP_RETURN(val1, val2, <, ret, false)
#define MVS_DCHECK_GE_RETURN(val1, val2, ret) \
    MVS_CHECK_OP_RETURN(val1, val2, >=, ret, false)
#define MVS_DCHECK_GT_RETURN(val1, val2, ret) \
    MVS_CHECK_OP_RETURN(val1, val2, >, ret, false)

/////////////////////////////////////////////////
/// Check_op macro definitions and continue/////////
/////////////////////////////////////////////////
#define MVS_CHECK_OP_CONTINUE(val1, val2, op) \
    if (!((val1)op(val2))) continue;
#define MVS_CHECK_EQ_CONTINUE(val1, val2) MVS_CHECK_OP_CONTINUE(val1, val2, ==)
#define MVS_CHECK_NE_CONTINUE(val1, val2) MVS_CHECK_OP_CONTINUE(val1, val2, !=)

/////////////////////////////////////////////////
/// Check_op macro definitions and break/////////
/////////////////////////////////////////////////
#define MVS_CHECK_OP_BREAK(val1, val2, op, sw)                             \
    if (!((val1)op(val2))) {                                               \
        if (sw) {                                                          \
            LoggerVoidify() & Message((char*)__FILE__, __LINE__, "native", \
                                  rulermvs::LogType::LogFatal)             \
                                      .stream()                            \
                                  << "Check failed break: " #val1 " " #op  \
                                     " " #val2 " ";                        \
        }                                                                  \
        break;                                                             \
    }
#define MVS_CHECK_EQ_BREAK(val1, val2) MVS_CHECK_OP_BREAK(val1, val2, ==, true)
#define MVS_CHECK_NE_BREAK(val1, val2) MVS_CHECK_OP_BREAK(val1, val2, !=, true)
#define MVS_DCHECK_EQ_BREAK(val1, val2) \
    MVS_CHECK_OP_BREAK(val1, val2, ==, false)
#define MVS_DCHECK_NE_BREAK(val1, val2) \
    MVS_CHECK_OP_BREAK(val1, val2, !=, false)

/////////////////////////////////////////////////
/// Check_op macro definitions and return void/////////
/////////////////////////////////////////////////
#define MVS_CHECK_OP_VOID(val1, val2, op, sw)                              \
    if (!((val1)op(val2))) {                                               \
        if (sw) {                                                          \
            LoggerVoidify() & Message((char*)__FILE__, __LINE__, "native", \
                                  rulermvs::LogType::LogFatal)             \
                                      .stream()                            \
                                  << "Check failed return void: " #val1    \
                                     " " #op " " #val2 " ";                \
        }                                                                  \
        return;                                                            \
    }
#define MVS_CHECK_EQ_VOID(val1, val2) MVS_CHECK_OP_VOID(val1, val2, ==, true)
#define MVS_CHECK_NE_VOID(val1, val2) MVS_CHECK_OP_VOID(val1, val2, !=, true)
#define MVS_DCHECK_EQ_VOID(val1, val2) MVS_CHECK_OP_VOID(val1, val2, ==, false)
#define MVS_DCHECK_NE_VOID(val1, val2) MVS_CHECK_OP_VOID(val1, val2, !=, false)

// qiao.helloworld@gmail.com /tzu.ta.lin@gmail.com add
// Add logging macros which are missing in glog or are not accessible for
// whatever reason.
#define CHECK_NEAR(val1, val2, margin)       \
    do {                                     \
        CHECK_LE((val1), (val2) + (margin)); \
        CHECK_GE((val1), (val2) - (margin)); \
    } while (0)
#ifndef NDEBUG
// Debug only versions of CHECK_OP macros.
#define MVS_DCHECK_EQ(val1, val2) MVS_CHECK_OP(val1, val2, ==)
#define MVS_DCHECK_NE(val1, val2) MVS_CHECK_OP(val1, val2, !=)
#define MVS_DCHECK_LE(val1, val2) MVS_CHECK_OP(val1, val2, <=)
#define MVS_DCHECK_LT(val1, val2) MVS_CHECK_OP(val1, val2, <)
#define MVS_DCHECK_GE(val1, val2) MVS_CHECK_OP(val1, val2, >=)
#define MVS_DCHECK_GT(val1, val2) MVS_CHECK_OP(val1, val2, >)
// qiao.helloworld@gmail.com /tzu.ta.lin@gmail.com add
#define MVS_DCHECK_NEAR(val1, val2, margin) MVS_CHECK_NEAR(val1, val2, margin)
#else
// These versions generate no code in optimized mode.
#define MVS_DCHECK_EQ(val1, val2) \
    if (false) MVS_CHECK_OP(val1, val2, ==)
#define MVS_DCHECK_NE(val1, val2) \
    if (false) MVS_CHECK_OP(val1, val2, !=)
#define MVS_DCHECK_LE(val1, val2) \
    if (false) MVS_CHECK_OP(val1, val2, <=)
#define MVS_DCHECK_LT(val1, val2) \
    if (false) MVS_CHECK_OP(val1, val2, <)
#define MVS_DCHECK_GE(val1, val2) \
    if (false) MVS_CHECK_OP(val1, val2, >=)
#define MVS_DCHECK_GT(val1, val2) \
    if (false) MVS_CHECK_OP(val1, val2, >)
// qiao.helloworld@gmail.com /tzu.ta.lin@gmail.com add
#define MVS_DCHECK_NEAR(val1, val2, margin) \
    if (false) MVS_CHECK_NEAR(val1, val2, margin)
#endif  // NDEBUG

// ---------------------------CHECK_NOTNULL macros ---------------------------
// Check that a pointer is not null.
#define MVS_CHECK_NOTNULL(val) \
    rulermvs::checkNotNull(    \
        __FILE__, __LINE__, "'" #val "' Must be non NULL", (val))

#ifndef NDEBUG
// Debug only version of CHECK_NOTNULL
#define MVS_DCHECK_NOTNULL(val) \
    rulermvs::checkNotNull(     \
        __FILE__, __LINE__, "'" #val "' Must be non NULL", (val))
#else
// Optimized version - generates no code.
#define MVS_DCHECK_NOTNULL(val) \
    if (false)                  \
    rulermvs::checkNotNull(     \
        __FILE__, __LINE__, "'" #val "' Must be non NULL", (val))
#endif  // NDEBUG

/// @internal 支持返回值,不支持返回void
#define MVS_DEXEC_RET(fn)                                            \
    [&]() {                                                          \
        using namespace std::chrono;                                 \
        MVS_ILOG << "[excute " << #fn << " start]";                  \
        steady_clock::time_point begin   = steady_clock::now();      \
        auto                     ret_val = fn;                       \
        steady_clock::time_point end     = steady_clock::now();      \
        MVS_ILOG << "[excute " << #fn << " finished in "             \
                 << duration_cast<milliseconds>(end - begin).count() \
                 << " ms]";                                          \
        return ret_val;                                              \
    }()

/// @internal 无返回值
#define MVS_DEXEC(fn)                                                \
    do {                                                             \
        using namespace std::chrono;                                 \
        MVS_ILOG << "[excute " << #fn << " start]";                  \
        steady_clock::time_point begin = steady_clock::now();        \
        fn;                                                          \
        steady_clock::time_point end = steady_clock::now();          \
        MVS_ILOG << "[excute " << #fn << " finished in "             \
                 << duration_cast<milliseconds>(end - begin).count() \
                 << " ms]";                                          \
    } while (0);

/// @internal 打印函数名
#define MVS_DTRACE MVS_ILOG << "of [" << __FUNCTION__ << "]";

#endif  //_RULERMVS_CORE_LOGGER_HPP_
