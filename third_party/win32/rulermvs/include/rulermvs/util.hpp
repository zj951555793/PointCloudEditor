#ifndef _RULERMVS_CORE_UTIL_HPP_
#define _RULERMVS_CORE_UTIL_HPP_
#include "rulermvs/core.hpp"

namespace rulermvs
{
class Logger;

/// @brief 高性能字符串转整形
MVS_EXPORT int strAtoi(char* str);

/// @brief 高性能将字符串转浮点类型
MVS_EXPORT double strAtof(char* str);

/// @brief 返回全局日志实例
/// @return 返回Logger引用
MVS_EXPORT Logger& globalLogger();

/// @brief 返回全局锁
MVS_EXPORT std::mutex& globalLock();

#if defined(_WIN32) && defined(_MSC_VER)
/// @brief 将UTF8字符串转换到GBK字符串
MVS_EXPORT std::string strUTF8ToGBK(const char* strUTF8);

/// @brief 将GBK字符转到UTF-8字符串
MVS_EXPORT std::string strGBKToUTF8(const char* strGBK);
#endif

/// @brief 格式化字符串
template <typename... Args>
static inline std::string strFormat(const char* format, Args&&... args)
{
    char buf[256];
    MVS_SPRINTF(buf, format, std::forward<Args>(args)...);
    return buf;
}

/// @brief 路径是否带文件夹
static inline bool strHasParent(const std::string& path)
{
    return path.rfind('/') != std::string::npos ||
           path.rfind('\\') != std::string::npos;
}

/// @brief 获取文件名
static inline std::string strFileName(const std::string& path)
{
    std::string tmp = path;
    std::replace(tmp.begin(), tmp.end(), '\\', '/');
    return tmp.substr(tmp.rfind('/') + 1, std::string::npos);
}

/// @brief 获取文件所在的文件夹
static inline std::string strParentDir(const std::string& path)
{
    std::string tmp = path;
    std::replace(tmp.begin(), tmp.end(), '\\', '/');
    return tmp.substr(0, tmp.rfind('/') + 1);
}

/// @brief 去除文件名的后缀
static inline std::string strRemoveExt(const std::string& path)
{
    return path.substr(0, path.rfind('.'));
}

/// @brief 对文件名进行排序
MVS_EXPORT void sortPaths(std::vector<std::string>& paths);

/// @brief 获取文件名后缀
static inline std::string strFileNameExt(const std::string& path)
{
    return path.substr(path.rfind('.') + 1, std::string::npos);
}

/// @brief 判断文件或者文件夹是否存在;
MVS_EXPORT bool isExists(const std::string& path);

/// @brief 迭代创建文件夹;
MVS_EXPORT bool createDirectory(const std::string& dir);

/// @brief 查询文件夹下相应后缀名的文件;
MVS_EXPORT void findPaths(const std::string& dir, const std::string& format,
    std::vector<std::string>& paths);

MVS_EXPORT void findSubDirs(
    const std::string& dir, std::vector<std::string>& dirs);

template <FileType type> struct FindFiles_ {};
template <> struct FindFiles_<FileType::BMP> {
    static void find(const std::string& dir, std::vector<std::string>& paths)
    {
        findPaths(dir, ".bmp", paths);
    }
};
template <> struct FindFiles_<FileType::PNG> {
    static void find(const std::string& _dir, std::vector<std::string>& paths)
    {
        findPaths(_dir, ".png", paths);
    }
};
template <> struct FindFiles_<FileType::JPG> {
    static void find(const std::string& dir, std::vector<std::string>& paths)
    {
        findPaths(dir, ".jpg", paths);
    }
};
template <> struct FindFiles_<FileType::OBJ> {
    static void find(const std::string& dir, std::vector<std::string>& paths)
    {
        findPaths(dir, ".obj", paths);
    }
};
template <> struct FindFiles_<FileType::ASC> {
    static void find(const std::string& dir, std::vector<std::string>& paths)
    {
        findPaths(dir, ".asc", paths);
    }
};
template <FileType type>
void findPaths(const std::string& dir, std::vector<std::string>& paths)
{
    FindFiles_<type>::find(dir, paths);
    sortPaths(paths);
}
template <FileType type>
std::vector<std::string> findPaths(const std::string& dir)
{
    std::vector<std::string> paths;
    findPaths<type>(dir, paths);
    return paths;
}

/// @brief 分割字符串
/// @param source 输入字符串
/// @param delimiter 分隔符
template <class charT, class traits, class allocator>
inline std::vector<std::basic_string<charT, traits, allocator>> strSplit(
    const std::basic_string<charT, traits, allocator>& source,
    const std::basic_string<charT, traits, allocator>& delimiter)
{
    std::vector<std::basic_string<charT, traits, allocator>> result;
    std::basic_string<charT, traits, allocator>              input = source;
    typename std::basic_string<charT, traits, allocator>::size_type index =
        input.find_first_of(delimiter);
    while (index != std::basic_string<charT, traits, allocator>::npos) {
        std::basic_string<charT, traits, allocator> substr =
            input.substr(0, index);
        input.erase(0, index + delimiter.size());
        result.push_back(substr);
        index = input.find_first_of(delimiter);
    }
    result.push_back(input);
    return result;
}
}  // namespace rulermvs
#endif  //__RULERMVS_RULERMVS_UTIL_H__