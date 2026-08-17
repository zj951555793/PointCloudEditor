#ifndef _RULERMVS_CORE_MATRIX_HPP_
#define _RULERMVS_CORE_MATRIX_HPP_
#include "rulermvs/core.hpp"
namespace rulermvs
{
template <int R, int C> struct size_at_complie_time {
    enum : int { ret = R * C };
};

template <typename T, int R, int C> struct Matrix_ {
    static_assert(
        R > 0 && C > 0, "rows and cols of matrix should greater than 0.");
    Matrix_() {}
    ~Matrix_() {}

    T data[size_at_complie_time<R, C>::ret];
};
}  // namespace rulermvs
#endif