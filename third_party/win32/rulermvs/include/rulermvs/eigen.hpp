#ifndef _RULERMVS_CORE_EIGEN_HPP_
#define _RULERMVS_CORE_EIGEN_HPP_

#if defined(RULERMVS_USE_MKL)
#define EIGEN_USE_MKL_ALL
#endif

#if defined(RULERMVS_USE_SSE)
#define EIGEN_VECTORIZE_SSE4_2
#endif

// #include "rulermvs/pose.hpp"
//--
// Eigen
// http://eigen.tuxfamily.org/dox-devel/QuickRefPage.html
//--
#include <memory>
#include <vector>
#include <initializer_list>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/StdVector>
#include "rulermvs/pose.hpp"
#include "rulermvs/image.hpp"
#include "rulermvs/camera.hpp"

namespace rulermvs
{
using Eigen::Map;

/// Trait used for double type
using EigenDoubleTraits = Eigen::NumTraits<double>;

/// 2d vector using int internal format
using Vec2i = Eigen::Vector2i;

/// 2d vector using float internal format
using Vec2f = Eigen::Vector2f;

/// 2d vector using double internal format
using Vec2d = Eigen::Vector2d;

/// 3d vector using int internal format
using Vec3i = Eigen::Vector3i;

/// 3d vector using float internal format
using Vec3f = Eigen::Vector3f;

/// 3d vector using double internal format
using Vec3d = Eigen::Vector3d;

/// 4d vector using double internal format
using Vec4d = Eigen::Vector4d;

/// 6d vector using double internal format
using Vec6d = Eigen::Matrix<double, 6, 1>;

/// 9d vector using double internal format
using Vec9d = Eigen::Matrix<double, 9, 1>;

/// Quaternion type
using Quaternion = Eigen::Quaternion<double>;

/// 3x3 matrix using double internal format
using Mat22 = Eigen::Matrix<double, 2, 2>;

/// 3x3 matrix using double internal format
using Mat33 = Eigen::Matrix<double, 3, 3>;

/// 3x4 matrix using double internal format
using Mat34 = Eigen::Matrix<double, 3, 4>;

/// 4x4 matrix using double internal format
using Mat44 = Eigen::Matrix<double, 4, 4>;

/// 6x6 matrix using double internal format
using Mat66 = Eigen::Matrix<double, 6, 6>;

/// generic matrix using unsigned int internal format
using Matu = Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic>;

/// 3x3 matrix using double internal format with RowMajor storage
using RMat33 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

/// 4x4 matrix using double internal format with RowMajor storage
using RMat44 = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

/// 4x4 matrix using double internal format with RowMajor storage
using RMat66 = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;

//-- General purpose Matrix and Vector
/// Unconstrained matrix using double internal format
using Mat = Eigen::MatrixXd;

/// Unconstrained vector using double internal format
using Vec = Eigen::VectorXd;

template <typename Tp> using Vec_ = Eigen::Matrix<Tp, Eigen::Dynamic, 1>;

/// Unconstrained vector using unsigned int internal format
using Vecu = Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>;

/// Unconstrained matrix using float internal format
using Matf = Eigen::MatrixXf;

/// Unconstrained vector using float internal format
using Vecf = Eigen::VectorXf;

/// 2xN matrix using double internal format
using Mat2X = Eigen::Matrix<double, 2, Eigen::Dynamic>;

/// 3xN matrix using double internal format
using Mat3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;

/// 4xN matrix using double internal format
using Mat4X = Eigen::Matrix<double, 4, Eigen::Dynamic>;

/// Nx6 matrix using double internal format
using MatX6 = Eigen::Matrix<double, Eigen::Dynamic, 6>;

/// Nx9 matrix using double internal format
using MatX9 = Eigen::Matrix<double, Eigen::Dynamic, 9>;

//-- Sparse Matrix (Column major, and row major)
/// Sparse unconstrained matrix using double internal format
using sMat = Eigen::SparseMatrix<double>;

/// Sparse unconstrained matrix using double internal format and Row Major
/// storage
using sRMat = Eigen::SparseMatrix<double, Eigen::RowMajor>;

#define SUM_OR_DYNAMIC(x, y) \
    (x == Eigen::Dynamic || y == Eigen::Dynamic) ? Eigen::Dynamic : (x + y)
template <typename Derived1, typename Derived2> struct hstack_return {
    using Scalar = typename Derived1::Scalar;
    enum {
        RowsAtCompileTime = Derived1::RowsAtCompileTime,
        ColsAtCompileTime = SUM_OR_DYNAMIC(
            Derived1::ColsAtCompileTime, Derived2::ColsAtCompileTime),
        Options = Derived1::Flags & Eigen::RowMajorBit ? Eigen::RowMajor : 0,
        MaxRowsAtCompileTime = Derived1::MaxRowsAtCompileTime,
        MaxColsAtCompileTime = SUM_OR_DYNAMIC(
            Derived1::MaxColsAtCompileTime, Derived2::MaxColsAtCompileTime)
    };
    using type = Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime,
        Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>;
};

template <typename Derived1, typename Derived2>
typename hstack_return<Derived1, Derived2>::type HStack(
    const Eigen::MatrixBase<Derived1>& lhs,
    const Eigen::MatrixBase<Derived2>& rhs)
{
    typename hstack_return<Derived1, Derived2>::type res;
    res.resize(lhs.rows(), lhs.cols() + rhs.cols());
    res << lhs, rhs;
    return res;
}

template <typename Derived1, typename Derived2> struct vstack_return {
    using Scalar = typename Derived1::Scalar;
    enum {
        RowsAtCompileTime = SUM_OR_DYNAMIC(
            Derived1::RowsAtCompileTime, Derived2::RowsAtCompileTime),
        ColsAtCompileTime = Derived1::ColsAtCompileTime,
        Options = Derived1::Flags & Eigen::RowMajorBit ? Eigen::RowMajor : 0,
        MaxRowsAtCompileTime = SUM_OR_DYNAMIC(
            Derived1::MaxRowsAtCompileTime, Derived2::MaxRowsAtCompileTime),
        MaxColsAtCompileTime = Derived1::MaxColsAtCompileTime
    };
    using type = Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime,
        Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>;
};

template <typename Derived1, typename Derived2>
typename vstack_return<Derived1, Derived2>::type VStack(
    const Eigen::MatrixBase<Derived1>& lhs,
    const Eigen::MatrixBase<Derived2>& rhs)
{
    typename vstack_return<Derived1, Derived2>::type res;
    res.resize(lhs.rows() + rhs.rows(), lhs.cols());
    res << lhs, rhs;
    return res;
}
#undef SUM_OR_DYNAMIC

template <typename Float> struct Rotation_;
template <> struct Converter_<Rotation_<double>, Mat33> {
    static inline void to(const Rotation_<double>& R, Mat33& mat)
    {
        mat = (Mat33() << R.a1, R.a2, R.a3, R.b1, R.b2, R.b3, R.c1, R.c2, R.c3)
                  .finished();
    }
};
template <typename T> struct Point3_;
template <> struct Converter_<Point3_<double>, Vec3d> {
    static inline void to(const Point3_<double>& pt, Vec3d& O)
    {
        O = (Vec3d() << pt.x, pt.y, pt.z).finished();
    }
};
template <typename Float> struct Pose_;
template <> struct Converter_<Pose_<double>, Mat44> {
    static inline void to(const Pose_<double>& RT, Mat44& mat)
    {
        auto tvec = -RT.rotate(RT.center());
        mat = (Mat44() << RT.a1, RT.a2, RT.a3, tvec.x, RT.b1, RT.b2, RT.b3,
            tvec.y, RT.c1, RT.c2, RT.c3, tvec.z, 0, 0, 0, 1)
                  .finished();
    }
};

}  // namespace rulermvs

// Extend EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION with initializer list support.
#define EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(...)           \
    namespace std                                                              \
    {                                                                          \
    template <> class vector<__VA_ARGS__, allocator<__VA_ARGS__>>              \
        : public vector<__VA_ARGS__, Eigen::aligned_allocator<__VA_ARGS__>> {  \
        typedef vector<__VA_ARGS__, Eigen::aligned_allocator<__VA_ARGS__>>     \
            vector_base;                                                       \
                                                                               \
    public:                                                                    \
        typedef __VA_ARGS__                 value_type;                        \
        typedef vector_base::allocator_type allocator_type;                    \
        typedef vector_base::size_type      size_type;                         \
        typedef vector_base::iterator       iterator;                          \
        explicit vector(const allocator_type& a = allocator_type())            \
            : vector_base(a)                                                   \
        {}                                                                     \
        template <typename InputIterator> explicit vector(InputIterator first, \
            InputIterator last, const allocator_type& a = allocator_type())    \
            : vector_base(first, last, a)                                      \
        {}                                                                     \
        vector(const vector& c) = default;                                     \
        explicit vector(size_type num, const value_type& val = value_type())   \
            : vector_base(num, val)                                            \
        {}                                                                     \
        explicit vector(iterator start, iterator end)                          \
            : vector_base(start, end)                                          \
        {}                                                                     \
        vector& operator=(const vector& x) = default;                          \
        /* Add initializer list constructor support*/                          \
        vector(initializer_list<__VA_ARGS__> list)                             \
            : vector_base(list.begin(), list.end())                            \
        {}                                                                     \
    };                                                                         \
    }  // namespace std

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(rulermvs::Vec2i)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(rulermvs::Vec2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(rulermvs::Vec2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(rulermvs::Vec3i)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(rulermvs::Vec3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(rulermvs::Vec3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(rulermvs::Vec4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(rulermvs::Vec6d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(rulermvs::Vec9d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(rulermvs::Mat33)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(rulermvs::Mat44)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(rulermvs::Mat34)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(rulermvs::RMat33)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(rulermvs::Quaternion)
//////////////////////////////////////////////////////////////////////////
#endif  // _RULERMVS_CORE_EIGEN_HPP_
