#ifndef _RULERMVS_CORE_POINT_HPP_
#define _RULERMVS_CORE_POINT_HPP_
#include "rulermvs/core.hpp"
namespace rulermvs
{
/// @brief 二维点类型定义
template <typename T> struct Point2_ {
    Point2_() = default;
    Point2_(T _x, T _y) : x(_x), y(_y) {}
    Point2_(const Point2_&) = default;
    template <typename Tp,
        class = typename std::enable_if<!std::is_same<T, Tp>::value>::type>
    explicit Point2_(Tp _x, Tp _y) : x((T)_x), y((T)_y)
    {}
    template <typename Tp,
        class = typename std::enable_if<!std::is_same<T, Tp>::value>::type>
    explicit Point2_(const Point2_<Tp>& _pt) : Point2_((T)_pt.x, (T)_pt.y)
    {}
    inline T dot(const Point2_& _pt) const { return x * _pt.x + y * _pt.y; }
    inline double ddot(const Point2_& _pt) const
    {
        return (double)x * _pt.x + (double)y * _pt.y;
    }
    inline double norm() const { return std::sqrt(norm2()); }
    inline double norm2() const { return (double)x * x + (double)y * y; }
    inline double cross(const Point2_& _pt) const
    {
        return (double)x * _pt.y - (double)y * _pt.x;
    }
    inline T&       operator[](int ind) { return (&x)[ind]; }
    inline const T& operator[](int ind) const { return (&x)[ind]; }
    /// @brief 序列化
    friend inline std::ostream& operator<<(std::ostream& out, const Point2_& pt)
    {
        return out << "Point2(" << pt.x << "," << pt.y << ")";
    };

    T x, y;
};
using Point2i    = Point2_<int>;
using Point2f    = Point2_<float>;
using Point2d    = Point2_<double>;
using Point2iVec = std::vector<Point2i>;
using Point2fVec = std::vector<Point2f>;
using Point2dVec = std::vector<Point2d>;
/// @brief 二维点向量的类型别名
/// @tparam T 常量类型
template <typename T> using Point2Vec = std::vector<Point2_<T>>;
template <typename T> struct CoordTraits<Point2_<T>> {
    typedef T type;
};

/// @brief 三维点
/// @tparam T 基本数据类型
template <typename T> struct Point3_ {
    Point3_() = default;
    Point3_(T x, T y, T z) : x(x), y(y), z(z) {}
    Point3_(const Point3_&) = default;
    template <typename Tp,
        class = typename std::enable_if<!std::is_same<T, Tp>::value, Tp>::type>
    explicit inline Point3_(Tp x, Tp y, Tp z) : x((T)x), y((T)y), z((T)z)
    {}
    template <typename Tp,
        class = typename std::enable_if<!std::is_same<T, Tp>::value, Tp>::type>
    explicit inline Point3_(const Point3_<Tp>& pt)
        : Point3_((T)pt.x, (T)pt.y, (T)pt.z)
    {}
    inline double norm() const { return std::sqrt(norm2()); }
    inline double norm2() const
    {
        return (double)x * x + (double)y * y + (double)z * z;
    }
    inline T dot(const Point3_& pt) const
    {
        return x * pt.x + y * pt.y + z * pt.z;
    }
    inline double ddot(const Point3_& pt) const
    {
        return (double)x * pt.x + (double)y * pt.y + (double)z * pt.z;
    }
    inline Point3_ cross(const Point3_& pt) const
    {
        return {y * pt.z - z * pt.y, z * pt.x - x * pt.z, x * pt.y - y * pt.x};
    }
    inline T&       operator[](int ind) { return (&x)[ind]; }
    inline const T& operator[](int ind) const { return (&x)[ind]; }
    /// @brief 序列化三维点
    friend inline std::ostream& operator<<(std::ostream& out, const Point3_& pt)
    {
        return out << "Point(" << pt.x << "," << pt.y << "," << pt.z << ")";
    };

    T x, y, z;
};
using Point3i    = Point3_<int>;
using Point3f    = Point3_<float>;
using Point3d    = Point3_<double>;
using Point3iVec = std::vector<Point3i>;
using Point3fVec = std::vector<Point3f>;
using Point3dVec = std::vector<Point3d>;
/// @brief 三维点向量的类型别名
/// @tparam T 常量类型
template <typename T> using Point3Vec = std::vector<Point3_<T>>;
template <typename T> struct CoordTraits<Point3_<T>> {
    typedef T type;
};

template <typename T>
static inline bool operator==(const Point2_<T>& a, const Point2_<T>& b)
{
    return a.x == b.x && a.y == b.y;
}
template <typename T> static inline bool operator==(const Point2_<T>& a, int b)
{
    return a.x == b && a.y == b;
}
template <typename T>
static inline bool operator==(const Point2_<T>& a, float b)
{
    return a.x == b && a.y == b;
}
template <typename T>
static inline bool operator==(const Point2_<T>& a, double b)
{
    return a.x == b && a.y == b;
}
template <typename T>
static inline bool operator!=(const Point2_<T>& a, const Point2_<T>& b)
{
    return a.x != b.x || a.y != b.y;
}
template <typename T> static inline bool operator!=(const Point2_<T>& a, int b)
{
    return a.x != b || a.y != b;
}
template <typename T>
static inline bool operator!=(const Point2_<T>& a, float b)
{
    return a.x != b || a.y != b;
}
template <typename T>
static inline bool operator!=(const Point2_<T>& a, double b)
{
    return a.x != b || a.y != b;
}
template <typename T>
static inline Point2_<T> operator+(const Point2_<T>& a, const Point2_<T>& b)
{
    return Point2_<T>(a.x + b.x, a.y + b.y);
}
template <typename T>
static inline Point2_<T>& operator+=(Point2_<T>& a, const Point2_<T>& b)
{
    a.x += b.x, a.y += b.y;
    return a;
}
template <typename T> static inline Point2_<T> operator-(const Point2_<T>& a)
{
    return Point2_<T>(-a.x, -a.y);
}
template <typename T>
static inline Point2_<T> operator-(const Point2_<T>& a, const Point2_<T>& b)
{
    return Point2_<T>(a.x - b.x, a.y - b.y);
}
template <typename T>
static inline Point2_<T>& operator-=(Point2_<T>& a, const Point2_<T>& b)
{
    a.x -= b.x, a.y -= b.y;
    return a;
}

template <typename T>
static inline Point2_<T> operator+(const Point2_<T>& a, int b)
{
    return Point2_<T>(a.x + b, a.y + b);
}
template <typename T>
static inline Point2_<T> operator+(int a, const Point2_<T>& b)
{
    return Point2_<T>(b.x + a, b.y + a);
}
template <typename T>
static inline Point2_<T> operator+(const Point2_<T>& a, float b)
{
    return Point2_<T>(a.x + b, a.y + b);
}
template <typename T>
static inline Point2_<T> operator+(float a, const Point2_<T>& b)
{
    return Point2_<T>(b.x + a, b.y + a);
}
template <typename T>
static inline Point2_<T> operator+(const Point2_<T>& a, double b)
{
    return Point2_<T>(a.x + b, a.y + b);
}
template <typename T>
static inline Point2_<T> operator+(double a, const Point2_<T>& b)
{
    return Point2_<T>(b.x + a, b.y + a);
}
template <typename T>
static inline Point2_<T> operator-(const Point2_<T>& a, int b)
{
    return Point2_<T>(a.x - b, a.y - b);
}
template <typename T>
static inline Point2_<T> operator-(int a, const Point2_<T>& b)
{
    return Point2_<T>(a - b.x, a - b.y);
}
template <typename T>
static inline Point2_<T> operator-(const Point2_<T>& a, float b)
{
    return Point2_<T>(a.x - b, a.y - b);
}
template <typename T>
static inline Point2_<T> operator-(float a, const Point2_<T>& b)
{
    return Point2_<T>(a - b.x, a - b.y);
}
template <typename T>
static inline Point2_<T> operator-(const Point2_<T>& a, double b)
{
    return Point2_<T>(a.x - b, a.y - b);
}
template <typename T>
static inline Point2_<T> operator-(double a, const Point2_<T>& b)
{
    return Point2_<T>(a - b.x, a - b.y);
}
template <typename T>
static inline Point2_<T> operator*(const Point2_<T>& a, int b)
{
    return Point2_<T>(a.x * b, a.y * b);
}
template <typename T>
static inline Point2_<T> operator*(int a, const Point2_<T>& b)
{
    return Point2_<T>(b.x * a, b.y * a);
}
template <typename T>
static inline Point2_<T> operator*(const Point2_<T>& a, float b)
{
    return Point2_<T>(a.x * b, a.y * b);
}
template <typename T>
static inline Point2_<T> operator*(float a, const Point2_<T>& b)
{
    return Point2_<T>(b.x * a, b.y * a);
}
template <typename T>
static inline Point2_<T> operator*(const Point2_<T>& a, double b)
{
    return Point2_<T>(a.x * b, a.y * b);
}
template <typename T>
static inline Point2_<T> operator*(double a, const Point2_<T>& b)
{
    return Point2_<T>(b.x * a, b.y * a);
}
template <typename T> static inline Point2_<T>& operator*=(Point2_<T>& a, int b)
{
    a.x *= b, a.y *= b;
    return a;
}
template <typename T>
static inline Point2_<T>& operator*=(Point2_<T>& a, float b)
{
    a.x *= b, a.y *= b;
    return a;
}
template <typename T>
static inline Point2_<T>& operator*=(Point2_<T>& a, double b)
{
    a.x *= b, a.y *= b;
    return a;
}
template <typename T>
static inline Point2_<T> operator/(const Point2_<T>& a, int b)
{
    return Point2_<T>(a.x / b, a.y / b);
}
template <typename T>
static inline Point2_<T> operator/(const Point2_<T>& a, float b)
{
    return Point2_<T>(a.x / b, a.y / b);
}
template <typename T>
static inline Point2_<T> operator/(const Point2_<T>& a, double b)
{
    return Point2_<T>(a.x / b, a.y / b);
}
template <typename T> static inline Point2_<T>& operator/=(Point2_<T>& a, int b)
{
    a.x /= b, a.y /= b;
    return a;
}
template <typename T>
static inline Point2_<T>& operator/=(Point2_<T>& a, float b)
{
    a.x /= b, a.y /= b;
    return a;
}
template <typename T>
static inline Point2_<T>& operator/=(Point2_<T>& a, double b)
{
    a.x /= b, a.y /= b;
    return a;
}

template <typename T>
static inline Point3_<T>& operator+=(Point3_<T>& a, const Point3_<T>& b)
{
    a.x += b.x, a.y += b.y, a.z += b.z;
    return a;
}
template <typename T>
static inline Point3_<T>& operator-=(Point3_<T>& a, const Point3_<T>& b)
{
    a.x -= b.x, a.y -= b.y, a.z -= b.z;
    return a;
}
template <typename T> static inline Point3_<T>& operator*=(Point3_<T>& a, int b)
{
    a.x *= (T)b, a.y *= (T)b, a.z *= (T)b;
    return a;
}
template <typename T>
static inline Point3_<T>& operator*=(Point3_<T>& a, float b)
{
    a.x *= (T)b, a.y *= (T)b, a.z *= (T)b;
    return a;
}
template <typename T>
static inline Point3_<T>& operator*=(Point3_<T>& a, double b)
{
    a.x *= (T)b, a.y *= (T)b, a.z *= (T)b;
    return a;
}
template <typename T> static inline Point3_<T>& operator/=(Point3_<T>& a, int b)
{
    a.x /= (T)b, a.y /= (T)b, a.z /= (T)b;
    return a;
}
template <typename T>
static inline Point3_<T>& operator/=(Point3_<T>& a, float b)
{
    a.x /= (T)b, a.y /= (T)b, a.z /= (T)b;
    return a;
}
template <typename T>
static inline Point3_<T>& operator/=(Point3_<T>& a, double b)
{
    a.x /= (T)b, a.y /= (T)b, a.z /= (T)b;
    return a;
}
template <typename T>
static inline bool operator==(const Point3_<T>& a, const Point3_<T>& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}
template <typename T> static inline bool operator==(const Point3_<T>& a, int b)
{
    return a.x == b && a.y == b && a.z == b;
}
template <typename T>
static inline bool operator==(const Point3_<T>& a, float b)
{
    return a.x == b && a.y == b && a.z == b;
}
template <typename T>
static inline bool operator==(const Point3_<T>& a, double b)
{
    return a.x == b && a.y == b && a.z == b;
}
template <typename T>
static inline bool operator!=(const Point3_<T>& a, const Point3_<T>& b)
{
    return a.x != b.x || a.y != b.y || a.z != b.z;
}
template <typename T> static inline bool operator!=(const Point3_<T>& a, int b)
{
    return a.x != b || a.y != b || a.z != b;
}
template <typename T>
static inline bool operator!=(const Point3_<T>& a, float b)
{
    return a.x != b || a.y != b || a.z != b;
}
template <typename T>
static inline bool operator!=(const Point3_<T>& a, double b)
{
    return a.x != b || a.y != b || a.z != b;
}
template <typename T>
static inline Point3_<T> operator+(const Point3_<T>& a, const Point3_<T>& b)
{
    return Point3_<T>(a.x + b.x, a.y + b.y, a.z + b.z);
}
template <typename T>
static inline Point3_<T> operator-(const Point3_<T>& a, const Point3_<T>& b)
{
    return Point3_<T>(a.x - b.x, a.y - b.y, a.z - b.z);
}
template <typename T> static inline Point3_<T> operator-(const Point3_<T>& a)
{
    return Point3_<T>(-a.x, -a.y, -a.z);
}
template <typename T>
static inline Point3_<T> operator*(const Point3_<T>& a, int b)
{
    return Point3_<T>(a.x * b, a.y * b, a.z * b);
}
template <typename T>
static inline Point3_<T> operator*(int a, const Point3_<T>& b)
{
    return Point3_<T>(b.x * a, b.y * a, b.z * a);
}
template <typename T>
static inline Point3_<T> operator*(const Point3_<T>& a, float b)
{
    return Point3_<T>(a.x * b, a.y * b, a.z * b);
}
template <typename T>
static inline Point3_<T> operator*(float a, const Point3_<T>& b)
{
    return Point3_<T>(b.x * a, b.y * a, b.z * a);
}
template <typename T>
static inline Point3_<T> operator*(const Point3_<T>& a, double b)
{
    return Point3_<T>(a.x * b, a.y * b, a.z * b);
}
template <typename T>
static inline Point3_<T> operator*(double a, const Point3_<T>& b)
{
    return Point3_<T>(b.x * a, b.y * a, b.z * a);
}
template <typename T>
static inline Point3_<T> operator/(const Point3_<T>& a, int b)
{
    return Point3_<T>(a.x / b, a.y / b, a.z / b);
}
template <typename T>
static inline Point3_<T> operator/(const Point3_<T>& a, float b)
{
    return Point3_<T>(a.x / b, a.y / b, a.z / b);
}
template <typename T>
static inline Point3_<T> operator/(const Point3_<T>& a, double b)
{
    return Point3_<T>(a.x / b, a.y / b, a.z / b);
}
}  // namespace rulermvs
#endif