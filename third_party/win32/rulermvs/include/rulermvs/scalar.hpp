#ifndef _RULERMVS_CORE_SCALAR_HPP_
#define _RULERMVS_CORE_SCALAR_HPP_
#include "rulermvs/core.hpp"
namespace rulermvs
{
/// @brief 指定长度数组
/// @tparam T 元数据类型
/// @tparam N 数据长度
template <typename T, int N = 2> struct Scalar_ {
    Scalar_() {}
    Scalar_(const Scalar_& _s) { memcpy(s, _s.s, sizeof(T[N])); }
    template <typename Tp,
        class = typename std::enable_if<!std::is_same<T, Tp>::value, T>::type>
    explicit Scalar_(const Scalar_<Tp, N>& sr)
    {
        for (int i = 0; i < N; ++i) s[i] = static_cast<T>(sr.s[i]);
    }
    template <typename... Args> Scalar_(Args... args) { set(args...); }
    template <int Index, typename First, typename... Args>
    void set(First first, Args... args)
    {
        if (Condition<(Index < N)>::check()) s[Index] = (T)first;
        set<Index + 1, Args...>(args...);
    }
    template <typename... Args> void set(Args... args)
    {
        set<0, Args...>(args...);
    }
    template <int Index> void set() {}

    inline T sum() const
    {
        T sr(s[0]);
        for (int i = 1; i < N; ++i) sr += s[i];
        return sr;
    }
    inline T accumulate() const
    {
        T sr(s[0]);
        for (int i = 1; i < N; ++i) sr *= s[i];
        return sr;
    }
    template <int M> Scalar_<T, M>& sub(int pos)
    {
        return *reinterpret_cast<Scalar_<T, M>*>(&s[pos]);
    }
    inline T  operator[](int ind) const { return s[ind]; }
    inline T& operator[](int ind) { return s[ind]; }
    Scalar_&  operator=(const Scalar_& sr)
    {
        for (int i = 0; i < N; ++i) s[i] = sr.s[i];
        return *this;
    }

    T s[N];
};
typedef Scalar_<float, 2>   Scalar2f;
typedef Scalar_<double, 2>  Scalar2d;
typedef Scalar_<int, 3>     Scalar3i;
typedef Scalar_<float, 3>   Scalar3f;
typedef Scalar_<size_t, 3>  Scalar3z;
typedef Scalar_<double, 3>  Scalar3d;
typedef Scalar_<float, 33>  Desc33f;
typedef Scalar_<float, 128> Desc128f;

typedef std::vector<Scalar_<float, 33>>  Desc33fVec;
typedef std::vector<Scalar_<float, 128>> Desc128fVec;
template <typename T> using Scalar2_ = Scalar_<T, 2>;
template <typename T> using Scalar3_ = Scalar_<T, 3>;

template <typename T, int N> struct CoordTraits<Scalar_<T, N>> {
    typedef T type;
};
template <typename T, int N>
static inline Scalar_<T, N>& operator+=(Scalar_<T, N>& s, T v)
{
    for (int i = 0; i < N; ++i) s.s[i] += v;
    return s;
}
template <typename T, int N>
static inline Scalar_<T, N>& operator-=(Scalar_<T, N>& s, T v)
{
    for (int i = 0; i < N; ++i) s.s[i] -= v;
    return s;
}
template <typename T, int N>
static inline Scalar_<T, N>& operator*=(Scalar_<T, N>& s, T v)
{
    for (int i = 0; i < N; ++i) s.s[i] *= v;
    return s;
}
template <typename T, int N>
static inline Scalar_<T, N>& operator/=(Scalar_<T, N>& s, T v)
{
    for (int i = 0; i < N; ++i) s.s[i] /= v;
    return s;
}
template <typename T, int N> static inline Scalar_<T, N> operator*(
    const Scalar_<T, N>& s, coord_traits_t<T> v)
{
    Scalar_<T, N> sr;
    for (int i = 0; i < N; ++i) sr.s[i] = s.s[i] * v;
    return sr;
}
template <typename T, int N> static inline Scalar_<T, N> operator/(
    const Scalar_<T, N>& s, coord_traits_t<T> v)
{
    Scalar_<T, N> sr;
    for (int i = 0; i < N; ++i) sr.s[i] = s.s[i] / v;
    return sr;
}
template <typename T, int N> static inline Scalar_<T, N> operator+(
    const Scalar_<T, N>& s, coord_traits_t<T> v)
{
    Scalar_<T, N> sr;
    for (int i = 0; i < N; ++i) sr.s[i] = s.s[i] + v;
    return sr;
}
template <typename T, int N> static inline Scalar_<T, N> operator-(
    const Scalar_<T, N>& s, coord_traits_t<T> v)
{
    Scalar_<T, N> sr;
    for (int i = 0; i < N; ++i) sr.s[i] = s.s[i] - v;
    return sr;
}
template <typename T, int N> static inline Scalar_<T, N> operator+(
    const Scalar_<T, N>& s1, const Scalar_<T, N>& s2)
{
    Scalar_<T, N> s;
    for (int i = 0; i < N; ++i) s.s[i] = s1.s[i] + s2.s[i];
    return s;
}
template <typename T, int N> static inline Scalar_<T, N> operator-(
    const Scalar_<T, N>& s1, const Scalar_<T, N>& s2)
{
    Scalar_<T, N> s;
    for (int i = 0; i < N; ++i) s.s[i] = s1.s[i] - s2.s[i];
    return s;
}
template <typename T, int N>
static inline bool operator==(const Scalar_<T, N>& s1, const Scalar_<T, N>& s2)
{
    for (int i = 0; i < N; ++i)
        if (s1.s[i] != s2.s[i]) return false;
    return true;
}
}  // namespace rulermvs
#endif