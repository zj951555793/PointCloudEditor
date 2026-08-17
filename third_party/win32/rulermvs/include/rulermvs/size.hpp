#ifndef _RULERMVS_CORE_SIZE_HPP_
#define _RULERMVS_CORE_SIZE_HPP_
#include "rulermvs/core.hpp"
namespace rulermvs
{
/// @brief 二维尺寸数据类型
/// @tparam T 数值类型
template <typename T,
    class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
struct Size_ {
    Size_() : width(0), height(0) {}
    Size_(T w, T h) : width(w), height(h) {}
    template <typename Tp,
        class = typename std::enable_if<!std::is_same<T, Tp>::value, Tp>::type>
    explicit Size_(Tp w, Tp h) : Size_((T)w, (T)h)
    {}
    Size_(const Size_& sz) : width(sz.width), height(sz.height) {}
    template <typename Tp,
        class = typename std::enable_if<!std::is_same<T, Tp>::value, Tp>::type>
    explicit Size_(const Size_<Tp>& sz) : Size_((T)sz.width, (T)sz.height)
    {}
    /// @brief 判断是否有效
    T    area() const { return width * height; }
    bool valid() const { return width > 0 && height > 0; }
    T    perimeter() const { return 2 * width + 2 * height; }

    T width, height;
};
using Size = Size_<int>;
template <typename T> struct CoordTraits<Size_<T>>: CoordType<T> {};

template <typename T> static inline Size_<T> operator+(const Size_<T>& sz, T s)
{
    return {sz.width + s, sz.height + s};
}
template <typename T> static inline Size_<T> operator-(const Size_<T>& sz, T s)
{
    return {sz.width - s, sz.height - s};
}
template <typename T>
static inline Size_<T> operator+(const Size_<T>& sz1, const Size_<T>& sz2)
{
    return {sz1.width + sz2.width, sz1.height + sz2.height};
}
template <typename T>
static inline Size_<T> operator-(const Size_<T>& sz1, const Size_<T>& sz2)
{
    return {sz1.width - sz2.width, sz1.height - sz2.height};
}
template <typename T>
static inline bool operator==(const Size_<T>& sz1, const Size_<T>& sz2)
{
    return sz1.width == sz2.width && sz1.height == sz2.height;
}
template <typename T>
static inline bool operator!=(const Size_<T>& sz1, const Size_<T>& sz2)
{
    return sz1.width != sz2.width || sz1.height != sz2.height;
}
template <typename T>
static inline std::ostream& operator<<(std::ostream& out, const Size_<T>& sz)
{
    return out << "Size(" << sz.width << "," << sz.height << ")";
}
}  // namespace rulermvs
#endif  // _RULERMVS_CORE_SIZE_HPP_