#ifndef _RULERMVS_CORE_CORE_HPP_
#define _RULERMVS_CORE_CORE_HPP_

#ifdef MVS_EXPORT
#undef MVS_EXPORT
#endif

#ifdef WIN32
/* win32 dll export/import directives */
#ifdef RULERMVS_EXPORTS
#if defined(__GNUC__)
#define MVS_EXPORT __attribute__((visibility("default")))
#elif defined(_MSC_VER)
#define MVS_EXPORT __declspec(dllexport)
#else
#define MVS_EXPORT
#endif
#elif defined(RULERMVS_STATIC)
#define MVS_EXPORT
#else
// #define MVS_EXPORT __declspec(dllimport)
#define MVS_EXPORT
#endif
#else
/* unix needs nothing */
#define MVS_EXPORT
#endif

#ifdef MVS_DEPRECATED
#undef MVS_DEPRECATED
#endif

#if defined(__GNUC__)
#define MVS_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define MVS_DEPRECATED __declspec(deprecated)
#else
#pragma message( \
    "WARNING: You need to implement MVS_DEPRECATED for this compiler")
#define MVS_DEPRECATED
#endif

#undef MVS_PLATFORM_64_BIT
#undef MVS_PLATFORM_32_BIT
#if __amd64__ || __x86_64__ || _WIN64 || _M_X64
#define MVS_PLATFORM_64_BIT
#else
#define MVS_PLATFORM_32_BIT
#endif

#define MVS_E 2.71828182845904523536                    // e
#define MVS_LOG2E 1.44269504088896340736                // log2(e)
#define MVS_LOG10E 0.434294481903251827651              // log10(e)
#define MVS_LN2 0.693147180559945309417                 // ln(2)
#define MVS_LN10 2.30258509299404568402                 // ln(10)
#define MVS_PI 3.1415926535897932384626433832795        // pi
#define MVS_2PI 6.283185307179586476925286766559        // 2pi
#define MVS_PI_2 1.57079632679489661923                 // pi/2
#define MVS_PI_4 0.785398163397448309616                // pi/4
#define MVS_PI_6 0.5236                                 // pi/6
#define MVS_PI_8 0.3927                                 // pi/8
#define MVS_PI_12 0.2618                                // pi/12
#define MVS_1_PI 0.318309886183790671538                // 1/pi
#define MVS_2_PI 0.636619772367581343076                // 2/pi
#define MVS_2_SQRTPI 1.12837916709551257390             // 2/sqrt(pi)
#define MVS_1_SQRT2PI 0.398942280401432677939946059935  // 1/sqrt(2pi)
#define MVS_SQRT2 1.41421356237309504880                // sqrt(2)
#define MVS_SQRT1_2 0.707106781186547524401             // 1/sqrt(2)
#define MVS_EPSILON 0.000000001000000000000             // epsilon

#include <cmath>
#include <mutex>
#include <vector>
#include <string>
#include <memory>
#include <cstring>
#include <assert.h>
#include <iostream>
#include <algorithm>
#include <functional>
#include <type_traits>

#ifdef RULERMVS_USE_SSE
#include <intrin.h>
#endif

#ifdef RULERMVS_USE_OPENMP
#include <omp.h>
#endif

#ifdef _WIN32
#include <stdio.h>
#else
#include <stdio.h>
#define sscanf_s sscanf
#define sprintf_s snprintf
#define fopen_s(pFile, filename, mode) \
    ((*(pFile)) = fopen((filename), (mode))) == NULL
#endif

// per OpenMP standard, _OPENMP defined when compiling with OpenMP.
// https://stackoverflow.com/questions/66747651/pragmaomp-parallel-for-causes-internal-error-in-visual-compiler
#if defined(RULERMVS_USE_OPENMP) && defined(_OPENMP)
#ifdef _MSC_VER
// must use MSVC __pragma here instead of _Pragma otherwise you get an internal
// compiler error. still an issue in Visual Studio 2022
#define MVS_OMP_PARALLEL_FOR __pragma(omp parallel for)
// any other standards-compliant C99/C++11 compiler
#else
#define MVS_OMP_PARALLEL_FOR _Pragma("omp parallel for")
#endif  // _MSC_VER
// no OpenMP support
#else
#define MVS_OMP_PARALLEL_FOR
#endif  // _OPENMP

#if defined(_WIN32) && defined(_MSC_VER)
#define MVS_FSCANF fscanf_s
#define MVS_SPRINTF sprintf_s
#define MVS_FOPEN(f, p, m) fopen_s(&f, p, m)
#else
#define MVS_FSCANF fscanf
#define MVS_SPRINTF sprintf
#define MVS_FOPEN(f, p, m) f = fopen(p, m)
#endif

// 针对某个实现类的Json序列化接口函数
#define RULERMVS_JSON_IO_EXPORT(T)                    \
    MVS_EXPORT bool readJson(const std::string&, T&); \
    MVS_EXPORT bool writeJson(const std::string&, const T&)

namespace rulermvs
{
/**@defgroup basic 基础数据结构
 * @brief 基础数据结构定义
 * @details 基础的数据结构定义
 * @{ */
// 无符号基类型别名
typedef unsigned int uint;
typedef unsigned char uchar;
typedef unsigned short ushort;

// 字符串别名
typedef std::string String;
typedef const std::string ConstStr;
typedef std::vector<std::string> StringVec;

// 数组别名
typedef std::vector<int> IntVec;
typedef std::vector<float> FloatVec;
typedef std::vector<double> DoubleVec;

// 进度条回调函数
typedef std::function<void(int progress)> ProgressBar;

/// @brief 拷贝接口类
template <typename T> struct Clonable {
    virtual std::shared_ptr<T> clone() const = 0;
};

/// @brief 用于辅助变长参数的展开
template <typename... Args> void ignore(Args&&...) {}

/// @brief 判断类型是否一致
template <typename T1, typename T2> struct IsSame {
    enum : bool { value = false };
};
template <typename T> struct IsSame<T, T> {
    enum : bool { value = true };
};

/// @brief 辅助函数，主要用来消除conditional expression warning.
/// @tparam b 布尔类型常量.
template <bool b> struct Condition {
    static inline bool check() { return true; }
};
template <> struct Condition<false> {
    static inline bool check() { return false; }
};

/// @brief 辅助模板类,针对GCC不兼容而实现的类MSVC实现模板.
template <bool, typename Tp = void> struct EnableIf {
    typedef void type;
};
/// @brief Partial specialization for true.
template <typename Tp> struct EnableIf<true, Tp> {
    typedef Tp type;
};
template <bool condition, typename T> using enable_if_t =
    typename EnableIf<condition, T>::type;

/// @brief 辅助模板，判断模板中的布尔类型是否存在false.
template <bool First, bool... Second> struct AnyFalse {
    enum { value = AnyFalse<Second...>::value };
};
template <bool... Second> struct AnyFalse<false, Second...> {
    enum { value = true };
};
template <> struct AnyFalse<true> {
    enum { value = false };
};

/// @brief 基础数据类型为数值类型
/// @tparam T 数值类型
template <typename T,
    class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
struct CoordType {
    typedef T type;
};

/// @brief 辅助函数，查询类的基础数据类型;
/// @tparam T 模板类型
template <typename T> struct CoordTraits: CoordType<T> {};
template <typename T> using coord_traits_t = typename CoordTraits<T>::type;

/// @brief 截断数值到指定范围.
/// @tparam T 数值类型
/// @param val 输入数值
/// @param min 最小值
/// @param max 最大值
/// @return 返回截断后的数值
template <typename T,
    class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
static inline T clamp(T val, T min, T max)
{
    return std::max<T>(min, std::min<T>(val, max));
}

/// @brief 输出增量数组
/// @tparam Tp 模板类型
/// @param start 起始点
/// @param end 终止条件
/// @param step 增量
/// @return 返回数组
template <typename T,
    class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
static inline std::vector<T> rangeVec(T start, T end, T step)
{
    std::vector<T> vec;
    if (start < end && step > 0) {
        for (T i = start; i < end; i += step) vec.emplace_back(i);
    } else if (start > end && step < 0) {
        for (T i = start; i > end; i += step) vec.emplace_back(i);
    }
    return vec;
}

/// @brief 创建实例的辅助类
/// @tparam T 实例类型
template <typename T> struct ObjectCreator_ {
    static inline T* create() { return nullptr; }
};
template <typename T, typename... Args>
std::shared_ptr<T> createObject(Args&&... args)
{
    auto obj = ObjectCreator_<T>::create(std::forward<Args>(args)...);
    return obj ? std::shared_ptr<T>(obj) : nullptr;
}

/// @brief 类型转换
template <typename T1, typename T2, class = void> struct Converter_ {
    static inline void to(const T1& src, T2& dst) { dst = (T2)src; }
};
template <typename T1> struct Converter_<T1, T1> {
    static inline void to(const T1& src, T1& dst) { dst = src; }
};
template <typename Float> struct Converter_<Float, ushort,
    typename std::enable_if<std::is_floating_point<Float>::value>::type> {
    static inline void to(Float src, ushort& dst)
    {
        dst = (ushort)std::round(clamp<Float>(src, 0.f, 65535.f));
    }
};
template <typename Float> struct Converter_<Float, uchar,
    typename std::enable_if<std::is_floating_point<Float>::value>::type> {
    static inline void to(Float src, uchar& dst)
    {
        dst = (uchar)std::round(clamp<Float>(src, 0.f, 255.f));
    }
};
template <typename T1, typename T2, typename... Args>
static inline void convertTo(const T1& src, T2& dst, Args&&... args)
{
    Converter_<T1, T2>::to(src, dst, std::forward<Args>(args)...);
}
/// @note 注意这里的模板参数顺序
template <typename T1, typename T2, typename... Args>
static inline T1 convertTo(const T2& src, Args&&... args)
{
    T1 dst;
    convertTo<T2, T1, Args...>(src, dst, std::forward<Args>(args)...);
    return dst;
}
template <typename T1, typename T2, typename... Args>
static inline void convertTo(const T1& src, T2& to1, T2& to2, Args&&... args)
{
    Converter_<T1, T2, Args...>::to(src, to1, to2, std::forward<Args>(args)...);
}

/// @internal 宏定义,辅助方法，提供自动转换类型的方法
#define RULERMVS_SMART_CONVERT_MEMBER_FUNC(T)                             \
    template <typename Tp, typename... Args>                              \
    void to(Tp& other, Args&&... args) const                              \
    {                                                                     \
        Converter_<T, Tp>::to(*dynamic_cast<const T*>(this), other,       \
            std::forward<Args>(args)...);                                 \
    }                                                                     \
    template <typename Tp, typename... Args> Tp to(Args&&... args) const  \
    {                                                                     \
        Tp other;                                                         \
        Converter_<T, Tp>::to(*dynamic_cast<const T*>(this), other,       \
            std::forward<Args>(args)...);                                 \
        return other;                                                     \
    }                                                                     \
    template <typename Tp, typename... Args>                              \
    T& from(const Tp& other, Args&&... args)                              \
    {                                                                     \
        Converter_<Tp, T>::to(                                            \
            other, *dynamic_cast<T*>(this), std::forward<Args>(args)...); \
        return *dynamic_cast<T*>(this);                                   \
    }

// 前置声明
enum class PixelType;
template <typename T> struct Image_;
/// @brief 图像接口类
struct IImage {
    /// @brief 析构函数
    virtual ~IImage() {}
    /// @brief 返回图像宽
    virtual int getWidth() const = 0;
    /// @brief 返回图像高
    virtual int getHeight() const = 0;
    /// @brief 返回图像内存偏移量
    virtual int getStride() const = 0;
    /// @brief 返回数据类型
    virtual PixelType getType() const = 0;
    /// @brief 返回数据内存指针
    virtual const uchar* getData() const = 0;
};
typedef std::shared_ptr<IImage> ImagePtr;

/// @brief 深度图所对应帧
template <typename Float> struct IRGBDImage_ {
    /// @brief 虚析构函数
    virtual ~IRGBDImage_() {}
    /// @brief 获取纹理图
    virtual const IImage& colorImage() const = 0;
    /// @brief 获取深度图
    virtual const Image_<Float>& rangeImage() const = 0;
};
using IRGBDImage = IRGBDImage_<float>;
template <typename Float> struct CoordTraits<IRGBDImage_<Float>>
    : CoordType<Float> {};

template <typename T> struct Point2_;
template <typename T> struct Point3_;
/// @brief 点云接口类,这里支持数据类型的接口访问;
/// @tparam Float 浮点类型
template <typename Float> struct IPointCloud_ {
    virtual ~IPointCloud_() {}
    /// @brief 返回点云数量
    virtual size_t getPointNum() const = 0;
    /// @brief 返回像素数据实际类型
    virtual PixelType getPixelType() const = 0;
    /// @brief 返回像素内存起始地址
    virtual const uchar* getPixelData() const = 0;
    /// @brief 返回三维点内存起始地址
    virtual const Point3_<Float>* getPointData() const = 0;
    /// @brief 返回法线内存起始地址
    virtual const Point3_<Float>* getNormalData() const = 0;
};
typedef IPointCloud_<float> IPointCloud;
template <typename Float> struct CoordTraits<IPointCloud_<Float>>
    : CoordType<Float> {};

template <typename T, int N> struct Scalar_;
/// @brief 三角网格接口类
/// @tparam Float 浮点类型
/// @tparam Index 索引类型
template <typename Float, typename Index = size_t> struct ITriMesh_
    : IPointCloud_<Float> {
    /// @brief 析构函数
    virtual ~ITriMesh_() {}
    /// @brief 返回纹理影像
    virtual const IImage& getTexture() const = 0;
    /// @brief 返回纹理坐标内存起始地址
    virtual const Point2_<Float>* getTexcoordData() const = 0;
    /// @brief 返回三角面数量
    virtual size_t getTriFaceNum() const = 0;
    /// @brief 返回三角面对应纹理坐标索引数据的指针
    virtual const Scalar_<Index, 3>* getTriUVData() const = 0;
    /// @brief 返回三角面对应三维点索引数据的指针
    virtual const Scalar_<Index, 3>* getTriPointData() const = 0;
    /// @brief 返回三角面对应法向索引数据的指针
    virtual const Scalar_<Index, 3>* getTriNormalData() const = 0;
};
typedef ITriMesh_<float, size_t> ITriMesh;
template <typename Float, typename Index>
struct CoordTraits<ITriMesh_<Float, Index>>: CoordType<Float> {};

/// @brief 导数类型
enum class DerivType : char { R, C, RR, RC, CC };

///@brief 文件类型
enum FileType : int { BMP = 0, JPG, OBJ, PNG, TGA, HDR, ASC, PLY };

///@brief ICP match method，based on the distance of measure function.
enum class ICPMethod { Point = 0, Normal, Color };

///@brief 帧间匹配采用的方法
enum class MatchMode { Both, Texture /*纹理*/, Geometry /*几何*/ };

// 这里屏蔽加速模式的选择，改用宏控制；实际实现的优化是内生性的,外部选择有些冗余;
/// @brief 加速模式
enum class AccelerateMode { None, SIMD, OpenMP, Cuda };

/// @brief 指令集支持
enum class SIMDMode {
    MVS_SSE2,
    MVS_SSE3,
    MVS_SSE4_1,
    MVS_SSE4_2,
    MVS_AVX,
    MVS_AVX2
};

/// @brief 匹配索引
struct DMatch {
    DMatch() : query_idx(-1), train_idx(-1) {}
    DMatch(int query, int train) : query_idx(query), train_idx(train) {}

    int query_idx, train_idx;
};
typedef std::vector<DMatch> DMatchVec;
template <> struct CoordTraits<DMatch>: CoordType<int> {};

/// @brief 结构光类型
enum class LightType { CodedLine, MultiLine, GridConner, PhaseShift };
template <LightType type> struct ICodedLight_ {
    ICodedLight_() = delete;
    ICodedLight_(const ICodedLight_&) = delete;
};
template <LightType type> struct CodedLightWapper_ {
    static inline std::shared_ptr<ICodedLight_<type>> create()
    {
        return std::shared_ptr<ICodedLight_<type>>(nullptr);
    }
};
template <LightType type>
static inline std::shared_ptr<ICodedLight_<type>> createCodedLight()
{
    return CodedLightWapper_<type>::create();
}

/// @brief CPU平台指令集支持信息
struct CPUSupportInfo {
    bool HAS_MMX;
    bool HAS_SSE;
    bool HAS_AVX;
    bool HAS_AVX2;
    bool HAS_SSE2;
    bool HAS_SSE3;
    bool HAS_SSSE3;
    bool HAS_SSE41;
    bool HAS_SSE42;
    bool HAS_SSE4a;
};

MVS_EXPORT bool checkHardwareSupport(SIMDMode mode);

/// @brief 寻找非零值的数量[SSE4加速]
MVS_EXPORT int countNonZero(const uchar* in, size_t num);

/// @brief 寻找非零浮点值的数量[SSE4加速]
MVS_EXPORT int countNonZero(const float* in, size_t num);

/// @brief 统计数组非零个数
template <typename Tp> static inline int countNonZero(const Tp* in, size_t num)
{
    int cnt = 0;
    if (!(in && num > 0)) return 0;
    const Tp *cur = in, *end = cur + num;
    for (; cur != end; ++cur)
        if (*cur != 0) cnt++;
    return cnt;
}

/** 占位函数接口 */
void readJson();
void writeJson();

/// @brief 判断是否存在对应的JSON文件加载函数
template <typename T> struct has_json_reader {
    typedef long No;
    typedef char Yes;
    template <typename Tp> struct helper {
        typedef bool (*func_ptr)(ConstStr&, T&);
    };
    template <typename Tp, Tp> struct TypeCheck;
    template <typename Tp>
    static Yes has_reader(TypeCheck<typename helper<Tp>::func_ptr, &readJson>*);
    template <typename Tp> static No has_reader(...);
    enum { value = (sizeof(has_reader<T>(0)) == sizeof(Yes)) };
};

/// @brief 判断是否存在对应的JSON文件保存函数
template <typename T> struct has_json_writer {
    typedef long No;
    typedef char Yes;
    template <typename Tp> struct helper {
        typedef bool (*func_ptr)(ConstStr&, const T&);
    };
    template <typename Tp, Tp> struct TypeCheck;
    template <typename Tp> static Yes has_writer(
        TypeCheck<typename helper<Tp>::func_ptr, &writeJson>*);
    template <typename Tp> static No has_writer(...);
    enum { value = (sizeof(has_writer<T>(0)) == sizeof(Yes)) };
};

/// @brief JSON文件读取
template <typename Tp, typename = void> struct JSONReader_ {
    static inline bool read(ConstStr&, Tp&) { return false; }
};
template <typename Tp, typename = void> struct JSONWriter_ {
    static inline bool write(ConstStr&, const Tp&) { return false; }
};

}  // namespace rulermvs
#endif  // _RULERMVS_CORE_CORE_HPP_