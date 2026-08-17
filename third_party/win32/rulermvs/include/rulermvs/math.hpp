#ifndef _RULERMVS_CORE_MATH_HPP_
#define _RULERMVS_CORE_MATH_HPP_
#include "rulermvs/core.hpp"
namespace rulermvs
{
/// 更新雅可比矩阵的权重
template <typename Float>
static inline void updateJacobianValue(Float jac[6], Float weight)
{
    jac[0] *= weight;
    jac[1] *= weight;
    jac[2] *= weight;
    jac[3] *= weight;
    jac[4] *= weight;
    jac[5] *= weight;
}
/// 计算B矩阵
template <typename Float>
static inline void updateBMatrix(double B[6], const Float jac[6], Float res)
{
    B[0] += jac[0] * res;
    B[1] += jac[1] * res;
    B[2] += jac[2] * res;
    B[3] += jac[3] * res;
    B[4] += jac[4] * res;
    B[5] += jac[5] * res;
}
/// 计算海森矩阵的上三角
template <typename Float>
static inline void updateUpperTrigularHessian(double H[36], const Float jac[6])
{
    H[0] += jac[0] * jac[0];
    H[1] += jac[0] * jac[1];
    H[2] += jac[0] * jac[2];
    H[3] += jac[0] * jac[3];
    H[4] += jac[0] * jac[4];
    H[5] += jac[0] * jac[5];
    H[7] += jac[1] * jac[1];
    H[8] += jac[1] * jac[2];
    H[9] += jac[1] * jac[3];
    H[10] += jac[1] * jac[4];
    H[11] += jac[1] * jac[5];
    H[14] += jac[2] * jac[2];
    H[15] += jac[2] * jac[3];
    H[16] += jac[2] * jac[4];
    H[17] += jac[2] * jac[5];
    H[21] += jac[3] * jac[3];
    H[22] += jac[3] * jac[4];
    H[23] += jac[3] * jac[5];
    H[28] += jac[4] * jac[4];
    H[29] += jac[4] * jac[5];
    H[35] += jac[5] * jac[5];
}
/// 扩展海森矩阵
template <typename Float>
static inline void expandUpperTrigularHessian(Float H[36])
{
    H[6]  = H[1];
    H[12] = H[2];
    H[18] = H[3];
    H[24] = H[4];
    H[30] = H[5];
    H[13] = H[8];
    H[19] = H[9];
    H[25] = H[10];
    H[31] = H[11];
    H[20] = H[15];
    H[26] = H[16];
    H[32] = H[17];
    H[27] = H[22];
    H[33] = H[23];
    H[34] = H[29];
}
template <typename Float> inline void computeEigenVals(
    Float _dxx, Float _dxy, Float _dyy, Float& eigval, Float eigvec[2])
{
    /* Compute the eigenvalues and eigenvectors of the Hessian matrix. */
    Float n1, n2, e1, e2;
    if (_dxy) {
        Float theta = 0.5f * (_dxx - _dyy) / _dxy;
        Float t = 1.0f / (std::abs(theta) + std::sqrt(theta * theta + 1.0f));
        t       = theta < 0 ? t : -t;
        n1      = 1.0f / std::sqrt(t * t + 1.0f);
        n2      = -t * n1;
        e1      = _dyy - t * _dxy;
        e2      = _dxx + t * _dxy;
    } else {
        n1 = 1.0f;
        n2 = 0.0f;
        e1 = _dyy;
        e2 = _dxx;
    }

    if (std::abs(e1) > std::abs(e2)) {
        eigval    = e1;
        eigvec[0] = n1;
        eigvec[1] = n2;
    } else if (std::abs(e1) < std::abs(e2)) {
        eigval    = e2;
        eigvec[0] = -n2;
        eigvec[1] = n1;
    } else {
        if (e1 < e2) {
            eigval    = e1;
            eigvec[0] = n1;
            eigvec[1] = n2;
        } else {
            eigval    = e2;
            eigvec[0] = -n2;
            eigvec[1] = n1;
        }
    }
}
/// 计算特征向量和特征值
template <typename Float> inline void computeEigenVals(
    Float _dxx, Float _dxy, Float _dyy, Float eigval[2], Float eigvec[2][2])
{
    /* Compute the eigenvalues and eigenvectors of the Hessian matrix. */
    Float n1, n2, e1, e2;
    if (_dxy) {
        Float theta = 0.5f * (_dxx - _dyy) / _dxy;
        Float t     = 1.0f / (std::abs(theta) + sqrt(theta * theta + 1.0f));
        t           = theta < 0 ? t : -t;
        n1          = 1.0f / sqrt(t * t + 1.0f);
        n2          = -t * n1;
        e1          = _dyy - t * _dxy;
        e2          = _dxx + t * _dxy;
    } else {
        n1 = 1.0f;
        n2 = 0.0f;
        e1 = _dyy;
        e2 = _dxx;
    }

    if (std::abs(e1) > std::abs(e2)) {
        eigval[0]    = e1;
        eigval[1]    = e2;
        eigvec[0][0] = n1;
        eigvec[0][1] = n2;
        eigvec[1][0] = -n2;
        eigvec[1][1] = n1;
    } else if (std::abs(e1) < std::abs(e2)) {
        eigval[0]    = e2;
        eigval[1]    = e1;
        eigvec[0][0] = -n2;
        eigvec[0][1] = n1;
        eigvec[1][0] = n1;
        eigvec[1][1] = n2;
    } else {
        if (e1 < e2) {
            eigval[0]    = e1;
            eigval[1]    = e2;
            eigvec[0][0] = n1;
            eigvec[0][1] = n2;
            eigvec[1][0] = -n2;
            eigvec[1][1] = n1;
        } else {
            eigval[0]    = e2;
            eigval[1]    = e1;
            eigvec[0][0] = -n2;
            eigvec[0][1] = n1;
            eigvec[1][0] = n1;
            eigvec[1][1] = n2;
        }
    }
}

template <typename Float> std::vector<Float> gaussianKernel(Float sigma)
{
    int  n    = (int)(std::ceil(3.09023230616781 * sigma));
    auto phi0 = [sigma](double x) -> double {
        return 0.5 * erfc(-x / (sigma * MVS_SQRT2));
    };
    std::vector<Float> vec(2 * n + 1);
    for (int i = 1; i < 2 * n; ++i)
        vec[i] = (Float)(phi0(n - i + 0.5) - phi0(n - i - 0.5));
    vec[0]     = (Float)(1 - phi0(n - 0.5));
    vec[2 * n] = (Float)phi0(-n + 0.5);
    return vec;
}

template <typename Float> std::vector<Float> gaussianDerivedKernel(Float sigma)
{
    int  n    = (int)(std::ceil(3.46087178201605 * sigma));
    auto phi1 = [sigma](double x) -> double {
        double t = x / sigma;
        return MVS_1_SQRT2PI / sigma * exp(-0.5 * t * t);
    };
    std::vector<Float> vec(2 * n + 1);
    for (int i = 1; i < 2 * n; ++i)
        vec[i] = (Float)(phi1(n - i + 0.5) - phi1(n - i - 0.5));
    vec[0]     = (Float)(-phi1(n - 0.5));
    vec[2 * n] = (Float)phi1(-n + 0.5);
    return vec;
}

template <typename Float> std::vector<Float> gaussianDerived2ndKernel(Float sigma)
{
    int  n    = (int)(std::ceil(3.82922419517181 * sigma));
    auto phi2 = [sigma](double x) -> double {
        double t = x / sigma;
        return -x * MVS_1_SQRT2PI / std::pow(sigma, 3) * exp(-0.5 * t * t);
    };
    std::vector<Float> vec(2 * n + 1);
    for (int i = 1; i < 2 * n; ++i)
        vec[i] = (Float)(phi2(n - i + 0.5) - phi2(n - i - 0.5));
    vec[0]     = (Float)(-phi2(n - 0.5));
    vec[2 * n] = (Float)phi2(-n + 0.5);
    return vec;
}

/// @brief 3*3矩阵的特征值
/// @note 快速实现方案,参考[https://zhuanlan.zhihu.com/p/547080414].
/// @param m 输入矩阵
/// @param x 特征值
extern "C" MVS_EXPORT void solveZ3x3(double m[9], double x[3]);

MVS_EXPORT bool computeEigen3x3(double m[9], double eval[3]);
MVS_EXPORT bool computeEigen3x3(double m[9], double eval[3], double evec[3][3]);
extern "C" MVS_EXPORT bool computeEigen4x4(
    double m[16], double eval[4], double evec[4][4]);
extern "C" MVS_EXPORT bool solveHessianAndBMatrix(
    const double hessian[36], const double b[6], double X[6]);
}  // namespace rulermvs
#endif  // _RULERMVS_CORE_MATH_HPP_