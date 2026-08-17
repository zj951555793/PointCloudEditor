#ifndef _RULERMVS_CORE_POSE_HPP_
#define _RULERMVS_CORE_POSE_HPP_
#include "rulermvs/point.hpp"
namespace rulermvs
{
enum class EulerAxisMode : int { XYZ = 0, XZY, YXZ, YZX, ZXY, ZYX };

/// @brief SO3旋转类
/// @tparam Float 浮点类型
template <typename Float> struct Rotation_ {
    static_assert(std::is_floating_point<Float>::value,
        "struct Rotation can only be instantiated with float point type.");
    /// @brief 初始化，为单位矩阵.
    Rotation_() : a1(1), a2(0), a3(0), b1(0), b2(1), b3(0), c1(0), c2(0), c3(1)
    {}
    /// @brief 通过3x3的矩阵进行初始化;
    Rotation_(Float data[9]) { memcpy(&a1, data, sizeof(Rotation_)); }
    Rotation_(const Rotation_& R) { memcpy(&a1, &R, sizeof(Rotation_)); }
    /// @brief 通过欧拉角初始化,
    /// @param euler_rad 旋转弧度
    /// @param axis [0,1,2]分别对应axis[x,y,z]三个轴.
    Rotation_(Float euler_rad, int axis = 0);
    /// @brief 通过旋转向量初始化
    Rotation_(Float rx, Float ry, Float rz);
    /// @brief 通过四元素初始化
    Rotation_(Float qw, Float qx, Float qy, Float qz);
    Rotation_(Float m1, Float m2, Float m3, Float m4, Float m5, Float m6,
        Float m7, Float m8, Float m9)
        : a1(m1), a2(m2), a3(m3), b1(m4), b2(m5), b3(m6), c1(m7), c2(m8), c3(m9)
    {}
    RULERMVS_SMART_CONVERT_MEMBER_FUNC(Rotation_<Float>)

    Rotation_ t() const { return {a1, b1, c1, a2, b2, c2, a3, b3, c3}; }
    Rotation_ inv() const
    {
        // 这里实现了三维矩阵的逆，不进行判断；
        Float det = a1 * (b2 * c3 - b3 * c2) + b1 * (c2 * a3 - a2 * c3) +
                    c1 * (a2 * b3 - b2 * a3);
        Float A11 = (b2 * c3 - b3 * c2) / det;
        Float A12 = (b3 * c1 - b1 * c3) / det;
        Float A13 = (b1 * c2 - b2 * c1) / det;
        Float A21 = (a3 * c2 - a2 * c3) / det;
        Float A22 = (a1 * c3 - a3 * c1) / det;
        Float A23 = (a2 * c1 - a1 * c2) / det;
        Float A31 = (a2 * b3 - a3 * b2) / det;
        Float A32 = (a3 * b1 - a1 * b3) / det;
        Float A33 = (a1 * b2 - a2 * b1) / det;
        return {A11, A21, A31, A12, A22, A32, A13, A23, A33};
    }
    Rotation_ dot(const Rotation_& R) const
    {
        Float data[9] = {a1 * R.a1 + a2 * R.b1 + a3 * R.c1,
            a1 * R.a2 + a2 * R.b2 + a3 * R.c2,
            a1 * R.a3 + a2 * R.b3 + a3 * R.c3,
            b1 * R.a1 + b2 * R.b1 + b3 * R.c1,
            b1 * R.a2 + b2 * R.b2 + b3 * R.c2,
            b1 * R.a3 + b2 * R.b3 + b3 * R.c3,
            c1 * R.a1 + c2 * R.b1 + c3 * R.c1,
            c1 * R.a2 + c2 * R.b2 + c3 * R.c2,
            c1 * R.a3 + c2 * R.b3 + c3 * R.c3};
        return data;
    }
    template <typename Tp> Point3_<Tp> rotate(const Point3_<Tp>& pt) const
    {
        return {(Tp)(a1 * pt.x + a2 * pt.y + a3 * pt.z),
            (Tp)(b1 * pt.x + b2 * pt.y + b3 * pt.z),
            (Tp)(c1 * pt.x + c2 * pt.y + c3 * pt.z)};
    }
    template <typename Tp>
    void rotate(const Point3_<Tp>* src, Point3_<Tp>* dst, size_t pt_num) const
    {
        //MVS_OMP_PARALLEL_FOR
        for (int i = 0; i < (int)pt_num; ++i) dst[i] = rotate(src[i]);
    }
#ifdef RULERMVS_USE_SSE
    void rotate(const Point3f* src, Point3f* dst, size_t num) const;
#endif
    template <typename Tp>
    void rotate(const Point3Vec<Tp>& src, Point3Vec<Tp>& dst) const
    {
        if (src.empty()) return;
        if (src.size() != dst.size()) dst.resize(src.size());
        rotate<Tp>(&src[0], &dst[0], src.size());
    }
    Rotation_& operator=(const Rotation_& R) noexcept
    {
        a1 = R.a1, a2 = R.a2, a3 = R.a3;
        b1 = R.b1, b2 = R.b2, b3 = R.b3;
        c1 = R.c1, c2 = R.c2, c3 = R.c3;
        return *this;
    }
    Rotation_& operator*=(const Rotation_& R) { return *this = dot(R); }

    friend inline std::ostream& operator<<(std::ostream& O, const Rotation_& R)
    {
        return O << "Rotation(" << R.a1 << "," << R.a2 << "," << R.a3 << ","
                 << R.b1 << "," << R.b2 << "," << R.b3 << "," << R.c1 << ","
                 << R.c2 << "," << R.c3 << ")";
    };

    Float a1, a2, a3, b1, b2, b3, c1, c2, c3;
};
using Rotation = Rotation_<double>;
template <typename T> struct CoordTraits<Rotation_<T>>: CoordType<T> {};
template <typename Float>
static inline Rotation_<Float> operator-(const Rotation_<Float>& R)
{
    return {-R.a1, -R.a2, -R.a3, -R.b1, -R.b2, -R.b3, -R.c1, -R.c2, -R.c3};
}
template <typename Float> static inline Rotation_<Float> operator*(
    const Rotation_<Float>& R1, const Rotation_<Float>& R2)
{
    return R1.dot(R2);
}
template <typename Float> static inline bool operator==(
    const Rotation_<Float> R1, const Rotation_<Float>& R2)
{
    const Float e = std::numeric_limits<Float>::epsilon();
    for (int i = 0; i < 9; ++i)
        if (std::abs((&R1.a1)[i] - (&R2.a1)[i]) > e) return false;
    return true;
}

/// @brief  SE3姿态类,用于表达姿态和位置信息.
/// @tparam Float 浮点类型，一般的我们推荐用双精度double类型.
template <typename Float> struct Pose_: Rotation_<Float>, Point3_<Float> {
    static_assert(std::is_floating_point<Float>::value,
        "struct Pose can only be instantiated with float point type.");
    typedef Point3_<Float>   OBase;
    typedef Rotation_<Float> RBase;
    Pose_() : RBase(), OBase(0, 0, 0) {}
    Pose_(ConstStr& path) { load(path); }
    Pose_(const Pose_& pose) : RBase(pose), OBase(pose) {}
    Pose_(const RBase& R, const OBase& O) : RBase(R), OBase(O) {}
    Pose_(Float rx, Float ry, Float rz, Float tx, Float ty, Float tz)
        : RBase(rx, ry, rz)
        , OBase(-getRotation().inv().rotate(OBase {tx, ty, tz}))
    {}
    Pose_(Float qw, Float qx, Float qy, Float qz, Float tx, Float ty, Float tz)
        : RBase(qw, qx, qy, qz)
        , OBase(-getRotation().inv().rotate(OBase {tx, ty, tz}))
    {}
    Pose_(Float a1, Float a2, Float a3, Float b1, Float b2, Float b3, Float c1,
        Float c2, Float c3, Float tx, Float ty, Float tz)
        : RBase(a1, a2, a3, b1, b2, b3, c1, c2, c3)
        , OBase(-getRotation().inv().rotate(OBase {tx, ty, tz}))
    {}
    Pose_(const Float mat3x4[12])
        : Pose_(mat3x4[0], mat3x4[1], mat3x4[2], mat3x4[4], mat3x4[5],
              mat3x4[6], mat3x4[8], mat3x4[9], mat3x4[10], mat3x4[3], mat3x4[7],
              mat3x4[11])
    {}
    RULERMVS_SMART_CONVERT_MEMBER_FUNC(Pose_<Float>)

    bool  load(ConstStr& path);
    bool  save(ConstStr& path) const;
    OBase center() const { return static_cast<OBase>(*this); }
    RBase getRotation() const { return static_cast<RBase>(*this); }
    OBase getTranslation() const { return -getRotation().rotate(center()); }
    void  setTranslation(Float tx, Float ty, Float tz)
    {
        auto C = -getRotation().inv().rotate(OBase {tx, ty, tz});
        memcpy(&this->x, &C.x, sizeof(Float[3]));
    }
    void setTranslation(const Point3_<Float>& tvec)
    {
        setTranslation(tvec.x, tvec.y, tvec.z);
    }
    template <typename Tp> void toMatrix(Tp rt[12]) const
    {
        auto t = getTranslation();
        for (int i = 0; i < 3; ++i) {
            (&rt[0])[i] = (Tp)((&(RBase::a1))[i]);
            (&rt[4])[i] = (Tp)((&(RBase::b1))[i]);
            (&rt[8])[i] = (Tp)((&(RBase::c1))[i]);
        }
        rt[3] = (Tp)t.x, rt[7] = (Tp)t.y, rt[11] = (Tp)t.z;
    }
    Pose_ dot(const Pose_& RT) const
    {
        return {static_cast<RBase>(*this) * static_cast<RBase>(RT),
            static_cast<RBase>(RT).inv().rotate(static_cast<OBase>(*this)) +
                static_cast<OBase>(RT)};
    }
    Pose_ inv() const
    {
        return {static_cast<RBase>(*this).inv(),
            -RBase::rotate(static_cast<OBase>(*this))};
    }
    template <typename Tp> Tp depth(const Point3_<Tp>& pt) const
    {
        return (Tp)(this->c1 * (pt.x - this->x) + this->c2 * (pt.y - this->y) +
                    this->c3 * (pt.z - this->z));
    }
    template <typename Tp>
    void depth(const Point3_<Tp>* points, Tp* depths, size_t pt_num) const
    {
        //MVS_OMP_PARALLEL_FOR
        for (int i = 0; i < (int)pt_num; ++i) depths[i] = depth(points[i]);
    }
    template <typename Tp> void depth(
        const std::vector<Point3_<Tp>>& points, std::vector<Tp>& depths) const
    {
        if (points.empty()) return;
        if (points.size() != depths.size()) depths.resize(points.size());
        depth(&points[0], &depths[0], points.size());
    }
    template <typename Tp> Point3_<Tp> transform(const Point3_<Tp>& pt) const
    {
        // 考虑到精度影响的问题
        return Point3_<Tp>(RBase::rotate(
            Point3_<Float>(pt.x - OBase::x, pt.y - OBase::y, pt.z - OBase::z)));
    }
    template <typename Tp>
    void transform(const Point3_<Tp>* src, Point3_<Tp>* dst, size_t sz) const
    {
        //MVS_OMP_PARALLEL_FOR
        for (int i = 0; i < (int)sz; ++i) dst[i] = transform(src[i]);
    }
#ifdef RULERMVS_USE_SSE
    void transform(const Point3f* src, Point3f* dst, size_t num) const;
#endif
    template <typename Tp>
    void transform(const std::vector<Point3_<Tp>>& points,
        std::vector<Point3_<Tp>>&                  points_t) const
    {
        if (points.empty()) return;
        if (points.size() != points_t.size()) points_t.resize(points.size());
        transform<Tp>(&points[0], &points_t[0], points.size());
    }
    Pose_& operator=(const Pose_& rt) noexcept
    {
        OBase::x = rt.x, OBase::y = rt.y, OBase::z = rt.z;
        RBase::a1 = rt.a1, RBase::a2 = rt.a2, RBase::a3 = rt.a3;
        RBase::b1 = rt.b1, RBase::b2 = rt.b2, RBase::b3 = rt.b3;
        RBase::c1 = rt.c1, RBase::c2 = rt.c2, RBase::c3 = rt.c3;
        return *this;
    }
    /// @brief 姿态乘法运算
    Pose_& operator*=(const Pose_& rt) { return *this = dot(rt); }
    friend inline std::ostream& operator<<(std::ostream& O, const Pose_& RT)
    {
        return O << "Pose[" << static_cast<RBase>(RT) << ","
                 << static_cast<OBase>(RT) << "]";
    };
};
using Pose    = Pose_<double>;
using PoseVec = std::vector<Pose>;
template <typename T> struct CoordTraits<Pose_<T>>: CoordType<T> {};
template <typename Float> static inline Pose_<Float> operator*(
    const Pose_<Float>& rt1, const Pose_<Float>& rt2)
{
    return rt1.dot(rt2);
}
template <typename Float> static inline Pose_<Float> operator*(
    const Rotation_<Float>& R, const Pose_<Float>& rt)
{
    return {R * rt.getRotation(), rt.center()};
}
template <typename Float> static inline Pose_<Float> operator*(
    const Pose_<Float>& rt, const Rotation_<Float>& R)
{
    return {rt.getRotation() * R, R.inv().rotate(rt.center())};
}
template <typename Float>
static inline bool operator==(const Pose_<Float> RT1, const Pose_<Float>& RT2)
{
    const Float e = std::numeric_limits<Float>::epsilon();
    for (int i = 0; i < 3; ++i)
        if (std::abs((&RT1.x)[i] - (&RT2.x)[i]) > e) return false;
    return static_cast<Rotation_<Float>>(RT1) ==
           static_cast<Rotation_<Float>>(RT2);
}
template <typename Float>
static inline void toRodrigues(const Float m[9], Float rvec[3])
{
    Float a = (m[0] + m[4] + m[8] - 1) / 2;
    Float e = std::numeric_limits<Float>::epsilon();
    if (std::abs(m[1] - m[3]) < e && std::abs(m[5] - m[7]) < e &&
        std::abs(m[2] - m[6]) < e) {
        if (std::abs(m[1] + m[3]) < 0.1 && std::abs(m[5] + m[7]) < 0.1 &&
            std::abs(m[2] + m[6]) < 0.1 && a > 0.9) {
            rvec[0] = rvec[1] = rvec[2] = 0;
        } else {
            Float xx       = (m[0] + 1) / 2;
            Float yy       = (m[4] + 1) / 2;
            Float zz       = (m[8] + 1) / 2;
            Float xy       = (m[1] + m[3]) / 4;
            Float xz       = (m[2] + m[6]) / 4;
            Float yz       = (m[5] + m[7]) / 4;
            Float pi_sqrt2 = (Float)(MVS_SQRT1_2 * MVS_PI);
            if ((xx > yy) && (xx > zz)) {
                if (xx < e) {
                    rvec[0] = 0, rvec[1] = rvec[2] = pi_sqrt2;
                } else {
                    Float t = std::sqrt(xx);
                    rvec[0] = (Float)(t * MVS_PI);
                    rvec[1] = (Float)(xy / t * MVS_PI);
                    rvec[2] = (Float)(xz / t * MVS_PI);
                }
            } else if (yy > zz) {
                if (yy < e) {
                    rvec[1] = 0;
                    rvec[0] = rvec[2] = pi_sqrt2;
                } else {
                    Float t = std::sqrt(yy);
                    rvec[0] = (Float)(xy / t * MVS_PI);
                    rvec[1] = (Float)(t * MVS_PI);
                    rvec[2] = (Float)(yz / t * MVS_PI);
                }
            } else {
                if (zz < e) {
                    rvec[0] = rvec[1] = pi_sqrt2, rvec[2] = 0;
                } else {
                    Float t = std::sqrt(zz);
                    rvec[0] = (Float)(xz / t * MVS_PI);
                    rvec[1] = (Float)(yz / t * MVS_PI);
                    rvec[2] = (Float)(t * MVS_PI);
                }
            }
        }
    } else {
        a       = std::acos(a);
        Float b = (Float)(0.5 * a / std::sin(a));
        rvec[0] = b * (m[7] - m[5]);
        rvec[1] = b * (m[2] - m[6]);
        rvec[2] = b * (m[3] - m[1]);
    }
}
template <typename Float>
static inline void toRodrigues(const Rotation_<Float>& R, Float rvec[3])
{
    toRodrigues<Float>(&R.a1, rvec);
}
template <typename Float>
static inline void fromRodrigues(const Float rvec[3], Float m[9])
{
    Float a =
        std::sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1] + rvec[2] * rvec[2]);
    Float ct = (Float)(a == 0.0 ? 0.5 : (1.0 - std::cos(a)) / a / a);
    Float st = (Float)(a == 0.0 ? 1 : std::sin(a) / a);
    m[0]     = (Float)(1.0 - (rvec[1] * rvec[1] + rvec[2] * rvec[2]) * ct);
    m[1]     = (Float)(rvec[0] * rvec[1] * ct - rvec[2] * st);
    m[2]     = (Float)(rvec[2] * rvec[0] * ct + rvec[1] * st);
    m[3]     = (Float)(rvec[0] * rvec[1] * ct + rvec[2] * st);
    m[4]     = (Float)(1.0 - (rvec[2] * rvec[2] + rvec[0] * rvec[0]) * ct);
    m[5]     = (Float)(rvec[1] * rvec[2] * ct - rvec[0] * st);
    m[6]     = (Float)(rvec[2] * rvec[0] * ct - rvec[1] * st);
    m[7]     = (Float)(rvec[1] * rvec[2] * ct + rvec[0] * st);
    m[8]     = (Float)(1.0 - (rvec[0] * rvec[0] + rvec[1] * rvec[1]) * ct);
}
template <typename Float>
static inline Rotation_<Float> fromRodrigues(const Float rvec[3])
{
    Rotation_<Float> rotation;
    fromRodrigues<Float>(rvec, &rotation.a1);
    return rotation;
}
template <typename Float>
Rotation_<Float>::Rotation_(Float rx, Float ry, Float rz)
{
    Float r[3] = {rx, ry, rz};
    fromRodrigues<Float>(r, &a1);
}
template <typename Float>
static inline void toQuaternian(const Float m[9], Float quat[4])
{
    quat[0] = 1 + m[0] + m[4] + m[8];
    if (quat[0] > std::numeric_limits<Float>::epsilon()) {
        quat[0] = std::sqrt(quat[0]) / 2;
        quat[1] = (m[7] - m[5]) / (4 * quat[0]);
        quat[2] = (m[2] - m[6]) / (4 * quat[0]);
        quat[3] = (m[3] - m[1]) / (4 * quat[0]);
    } else {
        if (m[0] > m[4] && m[0] > m[8]) {
            Float s = 2 * std::sqrt(1 + m[0] - m[4] - m[8]);
            quat[1] = s / 4;
            quat[2] = (m[1] + m[3]) / s;
            quat[3] = (m[2] + m[6]) / s;
            quat[0] = (m[5] - m[7]) / s;
        } else if (m[4] > m[8]) {
            Float s = 2 * std::sqrt(1 + m[4] - m[0] - m[8]);
            quat[1] = (m[1] + m[3]) / s;
            quat[2] = s / 4;
            quat[3] = (m[5] + m[7]) / s;
            quat[0] = (m[2] - m[6]) / s;
        } else {
            Float s = 2 * std::sqrt(1 + m[8] - m[0] - m[4]);
            quat[1] = (m[2] + m[6]) / s;
            quat[2] = (m[5] + m[7]) / s;
            quat[3] = s / 4;
            quat[0] = (m[1] - m[3]) / s;
        }
    }
}
template <typename Float>
static inline void toQuaternian(const Rotation_<Float>& R, Float quat[4])
{
    toQuaternian<Float>(&R.a1, quat);
}
template <typename Float>
static inline void fromQuaternian(const Float quat[4], Float m[9])
{
    Float qq = std::sqrt(quat[0] * quat[0] + quat[1] * quat[1] +
                         quat[2] * quat[2] + quat[3] * quat[3]);
    Float qw, qx, qy, qz;
    if (qq > 0) {
        qw = quat[0] / qq;
        qx = quat[1] / qq;
        qy = quat[2] / qq;
        qz = quat[3] / qq;
    } else {
        qw = 1, qx = qy = qz = 0;
    }
    m[0] = qw * qw + qx * qx - qz * qz - qy * qy;
    m[1] = 2 * qx * qy - 2 * qz * qw;
    m[2] = 2 * qy * qw + 2 * qz * qx;
    m[3] = 2 * qx * qy + 2 * qw * qz;
    m[4] = qy * qy + qw * qw - qz * qz - qx * qx;
    m[5] = 2 * qz * qy - 2 * qx * qw;
    m[6] = 2 * qx * qz - 2 * qy * qw;
    m[7] = 2 * qy * qz + 2 * qw * qx;
    m[8] = qz * qz + qw * qw - qy * qy - qx * qx;
}
template <typename Float>
static inline Rotation_<Float> fromQuaternian(const Float quat[4])
{
    Rotation_<Float> rotation;
    fromQuaternian<Float>(quat, &rotation.a1);
    return rotation;
}
template <typename Float>
Rotation_<Float>::Rotation_(Float qw, Float qx, Float qy, Float qz)
{
    Float q[4] = {qw, qx, qy, qz};
    fromQuaternian<Float>(q, &a1);
}
template <typename Float>
static inline void fromEulerAxisX(Float theta, Float m[9])
{
    m[0] = 1, m[1] = 0, m[2] = 0;
    m[3] = 0, m[4] = cos(theta), m[5] = sin(theta);
    m[6] = 0, m[7] = -sin(theta), m[8] = cos(theta);
}
template <typename Float>
static inline Rotation_<Float> fromEulerAxisX(Float theta)
{
    Rotation_<Float> rotation;
    fromEulerAxisX<Float>(theta, &rotation.a1);
    return rotation;
}
template <typename Float>
static inline void fromEulerAxisY(Float _theta, Float _m[9])
{
    _m[0] = cos(_theta);
    _m[1] = 0;
    _m[2] = -sin(_theta);
    _m[3] = 0;
    _m[4] = 1;
    _m[5] = 0;
    _m[6] = sin(_theta);
    _m[7] = 0;
    _m[8] = cos(_theta);
}
template <typename Float>
static inline Rotation_<Float> fromEulerAxisY(Float _theta)
{
    Rotation_<Float> rotation;
    fromEulerAxisY<Float>(_theta, &rotation.a1);
    return rotation;
}
template <typename Float>
static inline void fromEulerAxisZ(Float _theta, Float _m[9])
{
    _m[0] = cos(_theta);
    _m[1] = sin(_theta);
    _m[2] = 0;
    _m[3] = -sin(_theta);
    _m[4] = cos(_theta);
    _m[5] = 0;
    _m[6] = 0;
    _m[7] = 0;
    _m[8] = 1;
}
template <typename Float>
static inline Rotation_<Float> fromEulerAxisZ(Float _theta)
{
    Rotation_<Float> rotation;
    fromEulerAxisZ<Float>(_theta, &rotation.a1);
    return rotation;
}
template <typename Float> Rotation_<Float>::Rotation_(Float theta, int axis)
{
    if (axis == 2) fromEulerAxisZ(theta, &a1);
    else if (axis == 1)
        fromEulerAxisY(theta, &a1);
    else
        fromEulerAxisX(theta, &a1);
}
template <typename Float> static inline Rotation_<Float> fromEulerAngles(
    const Float _angle[3], EulerAxisMode _mode = EulerAxisMode::XYZ)
{
    Rotation_<Float> rotation;
    switch (_mode) {
        case EulerAxisMode::XZY:
            rotation = fromEulerAxisY(_angle[2]);
            rotation *= fromEulerAxisZ(_angle[1]);
            rotation *= fromEulerAxisX(_angle[0]);
            break;
        case EulerAxisMode::YXZ:
            rotation = fromEulerAxisZ(_angle[2]);
            rotation *= fromEulerAxisX(_angle[1]);
            rotation *= fromEulerAxisY(_angle[0]);
            break;
        case EulerAxisMode::YZX:
            rotation = fromEulerAxisX(_angle[2]);
            rotation *= fromEulerAxisZ(_angle[1]);
            rotation *= fromEulerAxisY(_angle[0]);
            break;
        case EulerAxisMode::ZXY:
            rotation = fromEulerAxisY(_angle[2]);
            rotation *= fromEulerAxisX(_angle[1]);
            rotation *= fromEulerAxisZ(_angle[0]);
            break;
        case EulerAxisMode::ZYX:
            rotation = fromEulerAxisX(_angle[2]);
            rotation *= fromEulerAxisY(_angle[1]);
            rotation *= fromEulerAxisZ(_angle[0]);
            break;
        case EulerAxisMode::XYZ:
        default:
            rotation = fromEulerAxisZ(_angle[2]);
            rotation *= fromEulerAxisY(_angle[1]);
            rotation *= fromEulerAxisX(_angle[0]);
            break;
    }
    return rotation;
}
template <typename Float> static inline void toEulerAngles(
    const Float m[9], Float euler[3], EulerAxisMode mode = EulerAxisMode::XYZ)
{}
template <typename Float>
static inline void fromEulerAxisXYZ(const Float euler[3], Float m[9])
{
    Float alpha = euler[0], beta = euler[1], gamma = euler[2];
    m[0] = cos(gamma) * cos(beta);
    m[1] = -sin(gamma) * cos(alpha) + cos(gamma) * sin(beta) * sin(alpha);
    m[2] = sin(gamma) * sin(alpha) + cos(gamma) * sin(beta) * cos(alpha);
    m[3] = sin(gamma) * cos(beta);
    m[4] = cos(gamma) * cos(alpha) + sin(gamma) * sin(beta) * sin(alpha);
    m[5] = -cos(gamma) * sin(alpha) + sin(gamma) * sin(beta) * cos(alpha);
    m[6] = -sin(beta);
    m[7] = cos(beta) * sin(alpha);
    m[8] = cos(beta) * cos(alpha);
}
template <typename Float>
static inline Rotation_<Float> fromEulerAxisXYZ(const Float _angle[3])
{
    Rotation_<Float> rotation;
    fromEulerAxisXYZ<Float>(_angle, &rotation.a1);
    return rotation;
}
template <typename Float>
static inline void toEulerAxisXYZ(const Float _m[9], Float _angle[3])
{
    auto alpha   = atan2(_m[7], _m[8]);
    auto gamma   = atan2(_m[3], _m[0]);
    auto cosbeta = (_m[0] * cos(gamma) + _m[3] * sin(gamma) +
                       _m[7] * sin(alpha) + _m[8] * cos(alpha)) /
                   2;
    auto beta = atan2(-_m[6], cosbeta);
    _angle[0] = alpha;
    _angle[1] = beta;
    _angle[2] = gamma;
}
template <typename Float> static inline void toEulerAxisXYZ(
    const Rotation_<Float>& rotation, Float angle[3])
{
    toEulerAxisXYZ(&rotation.a1, angle);
}
template <typename Float>
static inline void updatePoseFromX(Pose_<Float>& rt, const Float X[6])
{
    Rotation_<Float> R = fromEulerAxisXYZ(&X[0]);
    rt *= Pose_<Float>(R, -R.inv().rotate(Point3_<Float> {X[3], X[4], X[5]}));
}

template <typename Tp, typename Float>
static inline void rotatePoints(const Rotation_<Float>& r,
    const Point3_<Tp>* src, Point3_<Tp>* dst, size_t num)
{
    if (src && dst && num) r.rotate(src, dst, num);
}

template <typename Tp, typename Float>
static inline void rotatePoints(const Rotation_<Float>& r,
    const std::vector<Point3_<Tp>>& src, std::vector<Point3_<Tp>>& dst)
{
    if (src.empty()) return;
    if (dst.size() != src.size()) dst.resize(src.size());
    rotatePoints(r, src.data(), dst.data(), src.size());
}

template <typename Tp, typename Float> static inline void transformPoints(
    const Pose_<Float>& p, const Point3_<Tp>* src, Point3_<Tp>* dst, size_t num)
{
    if (src && dst && num) p.transform(src, dst, num);
}

template <typename Tp, typename Float>
static inline void transformPoints(const Pose_<Float>& p,
    const std::vector<Point3_<Tp>>& src, std::vector<Point3_<Tp>>& dst)
{
    if (src.empty()) return;
    if (dst.size() != src.size()) dst.resize(src.size());
    transformPoints(p, src.data(), dst.data(), src.size());
}

RULERMVS_JSON_IO_EXPORT(Pose);
RULERMVS_JSON_IO_EXPORT(PoseVec);
RULERMVS_JSON_IO_EXPORT(Rotation);

#ifdef RULERMVS_USE_SSE
/// @brief R*[x,y,z]'的指令集加速
template <typename Float> void Rotation_<Float>::rotate(
    const Point3f* src, Point3f* dst, size_t num) const
{
    // 如果平台不支持指令集加速，则采用一般方法.
    if (!checkHardwareSupport(SIMDMode::MVS_SSE3)) {
        this->rotate<float>(src, dst, num);
        return;
    }
    __m128 R[9];
    for (int i = 0; i < 9; ++i) R[i] = _mm_set1_ps((float)(&(this->a1))[i]);

    // MVS_OMP_PARALLEL_FOR
    for (int i = 0; i < (int)num >> 2; ++i) {
        auto*  src_ptr = &src[i << 2];
        auto*  dst_ptr = &dst[i << 2];
        __m128 X =
            _mm_set_ps(src_ptr[3].x, src_ptr[2].x, src_ptr[1].x, src_ptr[0].x);
        __m128 Y =
            _mm_set_ps(src_ptr[3].y, src_ptr[2].y, src_ptr[1].y, src_ptr[0].y);
        __m128 Z =
            _mm_set_ps(src_ptr[3].z, src_ptr[2].z, src_ptr[1].z, src_ptr[0].z);
        __m128 Xt =
            _mm_add_ps(_mm_add_ps(_mm_mul_ps(X, R[0]), _mm_mul_ps(Y, R[1])),
                _mm_mul_ps(Z, R[2]));
        __m128 Yt =
            _mm_add_ps(_mm_add_ps(_mm_mul_ps(X, R[3]), _mm_mul_ps(Y, R[4])),
                _mm_mul_ps(Z, R[5]));
        __m128 Zt =
            _mm_add_ps(_mm_add_ps(_mm_mul_ps(X, R[6]), _mm_mul_ps(Y, R[7])),
                _mm_mul_ps(Z, R[8]));
        for (int j = 0; j < 4; ++j) {
            dst_ptr[j].x = ((float*)&Xt)[j];
            dst_ptr[j].y = ((float*)&Yt)[j];
            dst_ptr[j].z = ((float*)&Zt)[j];
        }
    }
    for (size_t i = (num >> 2) << 2; i < num; ++i) dst[i] = rotate(src[i]);
}

/// @brief RT*[x,y,z]'的SSE加速
template <typename Float>
void Pose_<Float>::transform(const Point3f* src, Point3f* dst, size_t num) const
{
    // 如果平台不支持指令集加速，则采用一般方法.
    if (!checkHardwareSupport(SIMDMode::MVS_SSE3)) {
        this->transform<float>(src, dst, num);
        return;
    }
    __m128 RT[12];
    for (int i = 0; i < 3; ++i) {
        (&RT[0])[i] = _mm_set1_ps((float)((&(this->a1))[i]));
        (&RT[4])[i] = _mm_set1_ps((float)((&(this->b1))[i]));
        (&RT[8])[i] = _mm_set1_ps((float)((&(this->c1))[i]));
    }
    auto t = this->getTranslation();
    RT[3]  = _mm_set1_ps((float)t.x);
    RT[7]  = _mm_set1_ps((float)t.y);
    RT[11] = _mm_set1_ps((float)t.z);

    // MVS_OMP_PARALLEL_FOR
    for (int i = 0; i < (int)num >> 2; ++i) {
        auto*  src_ptr = &src[i << 2];
        auto*  dst_ptr = &dst[i << 2];
        __m128 X =
            _mm_set_ps(src_ptr[3].x, src_ptr[2].x, src_ptr[1].x, src_ptr[0].x);
        __m128 Y =
            _mm_set_ps(src_ptr[3].y, src_ptr[2].y, src_ptr[1].y, src_ptr[0].y);
        __m128 Z =
            _mm_set_ps(src_ptr[3].z, src_ptr[2].z, src_ptr[1].z, src_ptr[0].z);
        __m128 Xt = _mm_add_ps(
            _mm_add_ps(_mm_add_ps(_mm_mul_ps(X, RT[0]), _mm_mul_ps(Y, RT[1])),
                _mm_mul_ps(Z, RT[2])),
            RT[3]);
        __m128 Yt = _mm_add_ps(
            _mm_add_ps(_mm_add_ps(_mm_mul_ps(X, RT[4]), _mm_mul_ps(Y, RT[5])),
                _mm_mul_ps(Z, RT[6])),
            RT[7]);
        __m128 Zt = _mm_add_ps(
            _mm_add_ps(_mm_add_ps(_mm_mul_ps(X, RT[8]), _mm_mul_ps(Y, RT[9])),
                _mm_mul_ps(Z, RT[10])),
            RT[11]);
        for (int j = 0; j < 4; ++j) {
            dst_ptr[j].x = ((float*)&Xt)[j];
            dst_ptr[j].y = ((float*)&Yt)[j];
            dst_ptr[j].z = ((float*)&Zt)[j];
        }
    }
    for (size_t i = (num >> 2) << 2; i < num; ++i) dst[i] = transform(src[i]);
}
#endif

}  // namespace rulermvs
#endif