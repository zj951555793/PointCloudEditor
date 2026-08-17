#ifndef _RULERMVS_CORE_CAMERA_HPP_
#define _RULERMVS_CORE_CAMERA_HPP_
#include "rulermvs/size.hpp"
#include "rulermvs/point.hpp"
#include "rulermvs/scalar.hpp"
// #include "rulermvs/util.hpp"
namespace rulermvs
{
/// @brief 畸变模型的枚举类型
enum class DistorType : int { None, Radial, Brown };
/// @brief 占位，对应无相机畸变
/// @tparam Float 浮点类型
/// @tparam D 畸变类型
template <DistorType D = DistorType::None, typename Float = double>
struct DistorModel_ {
    DistorModel_() {}
    DistorModel_(Float...) {}
    DistorModel_(const DistorModel_&) {}
    template <typename Tp,
        class = typename std::enable_if<!std::is_same<Float, Tp>::value>::type>
    explicit DistorModel_(const DistorModel_<D, Tp>&)
    {}
    DistorModel_& operator=(const DistorModel_&) { return *this; }
    template <typename Tp>
    const Point3_<Tp>& distor(const Point3_<Tp>& pt) const noexcept
    {
        return static_cast<const Point3_<Tp>&>(pt);
    }
    template <typename Tp>
    const Point3_<Tp>& undistor(const Point3_<Tp>& pt) const noexcept
    {
        return static_cast<const Point3_<Tp>&>(pt);
    }
    friend inline std::ostream& operator<<(std::ostream& o, const DistorModel_&)
    {
        return o << "DistorNone";
    };
    const Float k1 = 0;  ///< 占位符
};
template <typename Float> struct DistorModel_<DistorType::Radial, Float> {
    DistorModel_() : k1(0), k2(0), k3(0) {}
    DistorModel_(const DistorModel_<DistorType::None, Float>& d)
        : k1(d.k1), k2(0), k3(0)
    {}
    DistorModel_(const DistorModel_& d) : k1(d.k1), k2(d.k2), k3(d.k3) {}
    template <typename Tp,
        class = typename std::enable_if<!std::is_same<Float, Tp>::value>::type>
    explicit DistorModel_(const DistorModel_<DistorType::Radial, Tp>& d)
        : k1((Float)d.k1), k2((Float)d.k2), k3((Float)d.k3)
    {}
    DistorModel_(Float k1, Float k2 = 0, Float k3 = 0) : k1(k1), k2(k2), k3(k3)
    {}
    template <typename Tp> Point3_<Tp> distor(const Point3_<Tp>& pt) const
    {
        if (!pt.z) return {0, 0, 0};
        Float x  = pt.x / pt.z;
        Float y  = pt.y / pt.z;
        Float r  = x * x + y * y;
        Float rr = r * r;
        Float kd = 1 + k1 * r + k2 * rr + k3 * rr * r;
        return {(Tp)(x * kd), (Tp)(y * kd), (Tp)1};
    }
    template <typename Tp>
    Point3_<Tp> undistor(const Point3_<Tp>& pt, int max_iter = 5) const
    {
        if (!pt.z) return {0, 0, 0};
        Float x0 = pt.x / pt.z;
        Float y0 = pt.y / pt.z;
        Float x = x0, y = y0;
        for (int i = 0; i < max_iter; ++i) {
            Float r  = x * x + y * y;
            Float rr = r * r;
            Float kd = 1 + k1 * r + k2 * rr + k3 * rr * r;

            Float b1 = kd * x - x0;
            Float b2 = kd * y - y0;
            Float J11 =
                kd + x * (2 * k1 * x + 4 * k2 * x * r + 6 * k3 * x * rr);
            Float J12 = x * (2 * k1 * y + 4 * k2 * y * r + 6 * k3 * y * rr);
            Float J21 = y * (2 * k1 * x + 4 * k2 * x * r + 6 * k3 * x * rr);
            Float J22 =
                kd + y * (2 * k1 * y + 4 * k2 * y * r + 6 * k3 * y * rr);

            Float A11 = J11 * J11 + J21 * J21;
            Float A12 = J11 * J12 + J21 * J22;
            Float A21 = J12 * J11 + J22 * J21;
            Float A22 = J12 * J12 + J22 * J22;
            Float B1  = J11 * b1 + J12 * b2;
            Float B2  = J21 * b1 + J22 * b2;
            Float det = A11 * A22 - A12 * A21;
            Float X1  = (A22 * B1 - A12 * B2) / det;
            Float X2  = (A11 * B2 - A21 * B1) / det;

            x -= X1;
            y -= X2;
        }
        return {(Tp)x, (Tp)y, (Tp)1};
    }
    friend inline std::ostream& operator<<(
        std::ostream& o, const DistorModel_& d)
    {
        return o << "DistorRadial(" << d.k1 << "," << d.k2 << "," << d.k3
                 << ")";
    };

    Float k1;  ///< 径向畸变K1
    Float k2;  ///< 径向畸变K2
    Float k3;  ///< 径向畸变K3
};
template <typename Float> struct DistorModel_<DistorType::Brown, Float> {
    DistorModel_() : k1(0), k2(0), p1(0), p2(0), k3(0) {}
    DistorModel_(const DistorModel_& d)
        : k1(d.k1), k2(d.k2), p1(d.p1), p2(d.p2), k3(d.k3)
    {}
    template <typename Tp,
        class = typename std::enable_if<!std::is_same<Float, Tp>::value>::type>
    explicit DistorModel_(const DistorModel_<DistorType::Brown, Tp>& d)
        : k1((Float)d.k1)
        , k2((Float)d.k2)
        , p1((Float)d.p1)
        , p2((Float)d.p2)
        , k3((Float)d.k3)
    {}
    DistorModel_(
        Float k1, Float k2 = 0, Float p1 = 0, Float p2 = 0, Float k3 = 0)
        : k1(k1), k2(k2), p1(p1), p2(p2), k3(k3)
    {}
    template <typename Tp> Point3_<Tp> distor(const Point3_<Tp>& pt) const
    {
        Float x   = pt.x / pt.z;
        Float y   = pt.y / pt.z;
        Float xx  = x * x;
        Float yy  = y * y;
        Float xy  = x * y;
        Float r   = xx + yy;
        Float rr  = r * r;
        Float kd  = 1 + k1 * r + k2 * rr + k3 * rr * r;
        Float dox = 2 * p1 * xy + p2 * (r + 2 * xx);
        Float doy = p1 * (r + 2 * yy) + 2 * p2 * xy;
        return {(Tp)(x * kd + dox), (Tp)(y * kd + doy), (Tp)1};
    }
    template <typename Tp>
    Point3_<Tp> undistor(const Point3_<Tp>& pt, int max_iter = 5) const
    {
        Float x0 = pt.x / pt.z;
        Float y0 = pt.y / pt.z;
        Float x = x0, y = y0;
        for (int i = 0; i < max_iter; ++i) {
            Float xx  = x * x;
            Float xy  = x * y;
            Float yy  = y * y;
            Float r   = xx + yy;
            Float rr  = r * r;
            Float kd  = 1 + k1 * r + k2 * rr + k3 * rr * r;
            Float dox = 2 * p1 * xy + p2 * (r + 2 * xx);
            Float doy = p1 * (r + 2 * yy) + 2 * p2 * xy;

            Float b1  = x * kd + dox - x0;
            Float b2  = y * kd + doy - y0;
            Float J11 = kd +
                        x * (2 * k1 * x + 4 * k2 * x * r + 6 * k3 * x * rr) +
                        2 * p1 * y + 6 * p2 * x;
            Float J12 = x * (2 * k1 * y + 4 * k2 * y * r + 6 * k3 * y * rr) +
                        2 * p1 * x + 2 * p2 * y;
            Float J21 = y * (2 * k1 * x + 4 * k2 * x * r + 6 * k3 * x * rr) +
                        2 * p1 * x + 2 * p2 * y;
            Float J22 = kd +
                        y * (2 * k1 * y + 4 * k2 * y * r + 6 * k3 * y * rr) +
                        6 * p1 * y + 2 * p2 * x;

            Float A11 = J11 * J11 + J21 * J21;
            Float A12 = J11 * J12 + J21 * J22;
            Float A21 = J12 * J11 + J22 * J21;
            Float A22 = J12 * J12 + J22 * J22;
            Float B1  = J11 * b1 + J12 * b2;
            Float B2  = J21 * b1 + J22 * b2;
            Float det = A11 * A22 - A12 * A21;
            Float X1  = (A22 * B1 - A12 * B2) / det;
            Float X2  = (A11 * B2 - A21 * B1) / det;

            x -= X1;
            y -= X2;
        }
        return {(Tp)x, (Tp)y, (Tp)1};
    }
    friend inline std::ostream& operator<<(
        std::ostream& o, const DistorModel_& d)
    {
        return o << "DistorBrown(" << d.k1 << "," << d.k2 << "," << d.p1 << ","
                 << d.p2 << "," << d.k3 << ")";
    };

    Float k1;  ///< 经向畸变K1
    Float k2;  ///< 经向畸变K2
    Float p1;  ///< 切向畸变P1
    Float p2;  ///< 切向畸变P2
    Float k3;  ///< 经向畸变K3
};
typedef DistorModel_<DistorType::None, double>   NoneDistor;
typedef DistorModel_<DistorType::Brown, double>  BrownDistor;
typedef DistorModel_<DistorType::Radial, double> RadialDistor;

/// @brief 相机类型枚举
enum class CameraType : int {
    SixBox,      ///< 六面体
    Fisheye,     ///< 鱼眼
    Pinhole,     ///< 针孔相机模型
    Spherical,   ///< 全景
    SkewPinhole  ///< 带斜率针孔相机模型
};

/// @brief 相机模型
template <CameraType C = CameraType::Pinhole, typename Float = double>
struct CameraModel_ {
    CameraModel_() : fx(0), fy(0), cx(0), cy(0) {}
    CameraModel_(Float f, Float cx, Float cy) : fx(f), fy(f), cx(cx), cy(cy) {}
    CameraModel_(Float fx, Float fy, Float cx, Float cy)
        : fx(fx), fy(fy), cx(cx), cy(cy)
    {}
    CameraModel_(const CameraModel_& p) : fx(p.fx), fy(p.fy), cx(p.cx), cy(p.cy)
    {}
    template <typename Tp,
        class = typename std::enable_if<!std::is_same<Float, Tp>::value>::type>
    explicit CameraModel_(const CameraModel_<CameraType::Pinhole, Tp>& p)
        : fx((Float)p.fx), fy((Float)p.fy), cx((Float)p.cx), cy((Float)p.cy)
    {}
    template <typename Tp> Point2_<Tp> point2uv(const Point3_<Tp>& pt) const
    {
        return {(Tp)(fx * pt.x / pt.z + cx), (Tp)(fy * pt.y / pt.z + cy)};
    }
    template <typename Tp> Point3_<Tp> uv2point(const Point2_<Tp>& uv) const
    {
        return {(Tp)((uv.x - cx) / fx), (Tp)((uv.y - cy) / fy), (Tp)1};
    }
    // friend CameraModel_ operator/(
    //     const CameraModel_& cam, coord_traits_t<Float> s)
    // {
    //     return {cam.fx / s, cam.fy / s, cam.cx / s, cam.cy / s};
    // }
    friend CameraModel_ operator*(
        const CameraModel_& cam, const Scalar2_<Float>& s)
    {
        return {cam.fx * s[0], cam.fy * s[1], cam.cx * s[0], cam.cy * s[1]};
    }
    friend CameraModel_ operator/(
        const CameraModel_& cam, const Scalar2_<Float>& s)
    {
        return {cam.fx / s[0], cam.fy / s[1], cam.cx / s[0], cam.cy / s[1]};
    }
    friend inline std::ostream& operator<<(
        std::ostream& O, const CameraModel_& cam)
    {
        return O << "Pinhole(" << cam.fx << "," << cam.fy << "," << cam.cx
                 << "," << cam.cy << ")";
    };

    Float fx;  ///< X方向的焦距
    Float fy;  ///< Y方向的焦距
    Float cx;  ///< X方向的像主点坐标
    Float cy;  ///< Y方向的像主点坐标
};

/// @brief 带斜率针孔相机模型
template <typename Float> struct CameraModel_<CameraType::SkewPinhole, Float> {
    CameraModel_() : fx(0), fy(0), cx(0), cy(0), sk(0) {}
    CameraModel_(Float fx, Float fy, Float cx, Float cy, Float sk = 0)
        : fx(fx), fy(fy), cx(cx), cy(cy), sk(sk)
    {}
    CameraModel_(const CameraModel_& p)
        : fx(p.fx), fy(p.fy), cx(p.cx), cy(p.cy), sk(p.sk)
    {}
    template <typename Tp,
        class = typename std::enable_if<!std::is_same<Float, Tp>::value>::type>
    explicit CameraModel_(const CameraModel_<CameraType::Pinhole, Tp>& p)
        : fx((Float)p.fx)
        , fy((Float)p.fy)
        , cx((Float)p.cx)
        , cy((Float)p.cy)
        , sk((Float)p.sk)
    {}
    template <typename Tp> Point2_<Tp> point2uv(const Point3_<Tp>& pt) const
    {
        return {(Tp)(fx * pt.x / pt.z + sk * pt.y / pt.z + cx),
            (Tp)(fy * pt.y / pt.z + cy)};
    }
    template <typename Tp> Point3_<Tp> uv2point(const Point2_<Tp>& uv) const
    {
        double v = (uv.y - cy) / fy;
        double u = (uv.x - sk * v - cx) / fx;
        return {(Tp)u, (Tp)v, (Tp)1};
    }

    // friend CameraModel_ operator*(
    //     const CameraModel_& cam, coord_traits_t<Float> s)
    // {
    //     return {cam.fx * s, cam.fy * s, cam.cx * s, cam.cy * s};
    // }
    // friend CameraModel_ operator/(
    //     const CameraModel_& cam, coord_traits_t<Float> s)
    // {
    //     return {cam.fx / s, cam.fy / s, cam.cx / s, cam.cy / s};
    // }
    friend CameraModel_ operator*(
        const CameraModel_& cam, const Scalar2_<Float>& s)
    {
        return {cam.fx * s[0], cam.fy * s[1], cam.cx * s[0], cam.cy * s[1],
            cam.sk * s[0]};
    }
    friend CameraModel_ operator/(
        const CameraModel_& cam, const Scalar2_<Float>& s)
    {
        assert(s[0] != 0 && s[1] != 0);
        return {cam.fx / s[0], cam.fy / s[1], cam.cx / s[0], cam.cy / s[1],
            cam.sk / s[0]};
    }
    friend inline std::ostream& operator<<(
        std::ostream& O, const CameraModel_& cam)
    {
        return O << "SkewPinhole(" << cam.fx << "," << cam.fy << "," << cam.cx
                 << "," << cam.cy << "," << cam.sk << ")";
    };

    Float fx;  ///< X方向的焦距
    Float fy;  ///< Y方向的焦距
    Float cx;  ///< X方向的像主点坐标
    Float cy;  ///< Y方向的像主点坐标
    Float sk;  ///< 斜率，微距相机中存在,或者标定板不正或提点精度不高容易造成.
};

template <typename Float> struct CameraModel_<CameraType::SixBox, Float> {
    Point2_<Float> point2uv(const Point3_<Float>& pt) const { return {0, 0}; }
    Point3_<Float> uv2point(const Point2_<Float>& uv) const
    {
        return {0, 0, 0};
    }

    Float square;
};
template <typename Float> struct CameraModel_<CameraType::Spherical, Float> {
    CameraModel_() {}
    Point2_<Float> point2uv(const Point3_<Float>& pt) const { return {0, 0}; }
    Point3_<Float> uv2point(const Point2_<Float>& uv) const
    {
        return {0, 0, 0};
    }

    Float ppx;
    Float ppy;
};
typedef CameraModel_<CameraType::SixBox, double>      SixBoxModel;
typedef CameraModel_<CameraType::Pinhole, double>     PinholeModel;
typedef CameraModel_<CameraType::Spherical, double>   SphericalModel;
typedef CameraModel_<CameraType::SkewPinhole, double> SkewPinholeModel;

template <typename Float, CameraType C, DistorType D = DistorType::None>
struct Camera_: CameraModel_<C, Float>, DistorModel_<D, Float> {
    static_assert(std::is_floating_point<Float>::value,
        "struct Camera can only be instantiated with float point type.");
    typedef CameraModel_<C, Float> CameraBase;
    typedef DistorModel_<D, Float> DistorBase;
    Camera_() : CameraBase(), DistorBase(), width(0), height(0) {}
    explicit Camera_(Size sz)
        : CameraBase(), DistorBase(), width(sz.width), height(sz.height)
    {}
    Camera_(Size sz, const CameraBase& c)
        : CameraBase(c), DistorBase(), width(sz.width), height(sz.height)
    {}
    Camera_(Size sz, const CameraBase& c, const DistorBase& d)
        : CameraBase(c), DistorBase(d), width(sz.width), height(sz.height)
    {}
    Camera_(const Camera_& cam)
        : CameraBase(cam), DistorBase(cam), width(cam.width), height(cam.height)
    {}
    Camera_(ConstStr& path) : Camera_() { load(path); }
    template <typename Tp,
        class = typename std::enable_if<std::is_same<Float, Tp>::value>::type>
    explicit Camera_(const Camera_<Tp, C, D>& cam)
        : width(cam.width), height(cam.height), CameraBase(cam), DistorBase(cam)
    {}
    bool load(ConstStr& path);
    bool save(ConstStr& path) const;
    Size size() const { return {width, height}; }
    /// @brief 重置相机对应影像分辨率的参数.
    /// @return 返回自身引用
    Camera_& resize(Size sz);
    /// @brief 返回无畸变内参
    Camera_<Float, C, DistorType::None> nodistor() const
    {
        return {this->size(), static_cast<CameraBase>(*this)};
    }
    template <typename = typename EnableIf<C == CameraType::SkewPinhole>::type>
    Camera_<Float, CameraType::Pinhole, D> noskew() const
    {
        return {this->size(),
            {CameraBase::fx, CameraBase::fy, CameraBase::cx, CameraBase::cy},
            static_cast<DistorBase>(*this)};
    }
    template <typename Tp> void uv2points(
        const Point2_<Tp>* uvs, Point3_<Tp>* pts, size_t pt_num) const
    {
        assert(uvs != nullptr && pts != nullptr && pt_num > 0);
        for (size_t i = 0; i < pt_num; ++i)
            pts[i] = CameraBase::uv2point(uvs[i]);
    }
    template <typename Tp> void uv2points(const std::vector<Point2_<Tp>>& uvs,
        std::vector<Point3_<Tp>>& pts) const
    {
        if (uvs.empty()) return;
        if (uvs.size() != pts.size()) pts.resize(uvs.size());
        uv2points(&uvs[0], &pts[0], uvs.size());
    }
    template <typename Tp> void point2uvs(
        const Point3_<Tp>* pts, Point2_<Tp>* uvs, size_t pt_num) const
    {
        assert(pts != nullptr && uvs != nullptr && pt_num > 0);
        for (size_t i = 0; i < pt_num; ++i)
            uvs[i] = CameraBase::point2uv(pts[i]);
    }
    template <typename Tp> void point2uvs(const std::vector<Point3_<Tp>>& pts,
        std::vector<Point2_<Tp>>& uvs) const
    {
        if (pts.empty()) return;
        if (pts.size() != uvs.size()) uvs.resize(pts.size());
        point2uvs(&pts[0], &uvs[0], pts.size());
    }
    template <typename Tp> Point3_<Tp> project(const Point2_<Tp>& uv) const
    {
        return DistorBase::undistor(CameraBase::uv2point(uv));
    }
    template <typename Tp>
    void project(const Point2_<Tp>* uvs, Point3_<Tp>* pts, size_t pt_num) const
    {
        assert(uvs != nullptr && pts != nullptr && pt_num > 0);
        for (size_t i = 0; i < pt_num; ++i) pts[i] = project(uvs[i]);
    }
    template <typename Tp> void project(const std::vector<Point2_<Tp>>& uvs,
        std::vector<Point3_<Tp>>& pts) const
    {
        if (uvs.empty()) return;
        if (uvs.size() != pts.size()) pts.resize(uvs.size());
        project(&uvs[0], &pts[0], uvs.size());
    }
    template <typename Tp> Point2_<Tp> reproject(const Point3_<Tp>& pt) const
    {
        return CameraBase::point2uv(DistorBase::distor(pt));
    }
    template <typename Tp> void reproject(
        const Point3_<Tp>* pts, Point2_<Tp>* uvs, size_t pt_num) const
    {
        assert(pts != nullptr && uvs != nullptr && pt_num > 0);
        for (size_t i = 0; i < pt_num; ++i) uvs[i] = reproject(pts[i]);
    }
    template <typename Tp> void reproject(const std::vector<Point3_<Tp>>& pts,
        std::vector<Point2_<Tp>>& uvs) const
    {
        if (pts.empty()) return;
        if (uvs.size() != pts.size()) uvs.resize(pts.size());
        reproject(&pts[0], &uvs[0], uvs.size());
    }
    friend inline std::ostream& operator<<(std::ostream& o, const Camera_& cam)
    {
        return o << "Camera[ " << static_cast<CameraBase>(cam) << " "
                 << static_cast<DistorBase>(cam) << " " << cam.size() << " ]";
    };

    int width, height;
};
template <typename Float, DistorType D> using CameraP_ =
    Camera_<Float, CameraType::Pinhole, D>;
template <typename Float, DistorType D> using CameraSkewP_ =
    Camera_<Float, CameraType::SkewPinhole, D>;
typedef CameraP_<double, DistorType::None>       CameraP;
typedef CameraP_<double, DistorType::Brown>      CameraPB;
typedef CameraP_<double, DistorType::Radial>     CameraPR;
typedef CameraSkewP_<double, DistorType::None>   CameraSkewP;
typedef CameraSkewP_<double, DistorType::Brown>  CameraSkewPB;
typedef CameraSkewP_<double, DistorType::Radial> CameraSkewPR;

template <typename Float, CameraType C, DistorType D>
static inline Camera_<Float, C, D> resizeCamera(
    const Camera_<Float, C, D>& cam, Size sz)
{
    using CameraT = Camera_<Float, C, D>;
    if (cam.size() == sz) return CameraT {cam};
    Scalar2_<Float> s = {1.0f, 1.0f};
    if (cam.width > 0 && cam.height > 0) {
        s[0] = static_cast<Float>((double)sz.width / cam.width);
        s[1] = static_cast<Float>((double)sz.height / cam.height);
    }
    return CameraT {sz, static_cast<typename CameraT::CameraBase>(cam) * s,
        static_cast<typename CameraT::DistorBase>(cam)};
}

template <typename Float, CameraType C, DistorType D>
Camera_<Float, C, D>& Camera_<Float, C, D>::resize(Size sz)
{
    return *this = resizeCamera(*this, sz);
}
template <typename Float, CameraType C, DistorType D>
static inline Camera_<Float, C, D> operator*(
    const Camera_<Float, C, D>& cam, double s)
{
    return resizeCamera(cam, Size_<int> {cam.width * s, cam.height * s});
}
// static inline Camera_<Float, C, D> operator*(
//     const Camera_<Float, C, D>& cam, coord_traits_t<Float> s)
// {
//     return resizeCamera(cam, Size_<int> {static_cast<int>(cam.width * s),
//                                  static_cast<int>(cam.height * s)});
// }
template <typename Float, CameraType C, DistorType D>
static inline Camera_<Float, C, D> operator/(
    const Camera_<Float, C, D>& cam, double s)
{
    return resizeCamera(cam, Size_<int> {cam.width / s, cam.height / s});
}

// 畸变矫正
template <typename Tp, typename Float, CameraType C, DistorType D>
void undistorPoints(const Camera_<Float, C, D>& cam, const Point2_<Tp>* uvs,
    Point2_<Tp>* un_uvs, size_t sz)
{
    for (size_t i = 0; i < sz; ++i)
        un_uvs[i] = cam.point2uv(cam.undistor(cam.uv2point(uvs[i])));
}
template <typename Tp, typename Float, CameraType C, DistorType D>
void undistorPoints(const Camera_<Float, C, D>& cam,
    const std::vector<Point2_<Tp>>& uvs, std::vector<Point2_<Tp>>& un_uvs)
{
    if (uvs.empty()) return;
    if (uvs.size() != un_uvs.size()) un_uvs.resize(uvs.size());
    undistorPoints<Tp, Float, C, D>(cam, &uvs[0], &un_uvs[0], uvs.size());
}
template <typename Tp, typename Float, CameraType C, DistorType D>
std::vector<Point2_<Tp>> undistorPoints(
    const Camera_<Float, C, D>& cam, const std::vector<Point2_<Tp>>& uvs)
{
    std::vector<Point2_<Tp>> un_uvs;
    undistorPoints<Tp, Float, C, D>(cam, uvs, un_uvs);
    return un_uvs;
}

// 从JSON文件读取或保存相机内参
RULERMVS_JSON_IO_EXPORT(CameraP);
RULERMVS_JSON_IO_EXPORT(CameraPB);

template <typename Float, CameraType C, DistorType D>
bool Camera_<Float, C, D>::load(ConstStr& path)
{
    return JSONReader_<Camera_<Float, C, D>>::read(path, *this);
}
template <typename Float, CameraType C, DistorType D>
bool Camera_<Float, C, D>::save(ConstStr& path) const
{
    return JSONWriter_<Camera_<Float, C, D>>::write(path, *this);
}
template <typename Float, CameraType C, DistorType D>
struct JSONReader_<Camera_<Float, C, D>,
    typename std::enable_if<
        has_json_reader<Camera_<Float, C, D>>::value>::type> {
    static inline bool read(ConstStr& path, Camera_<Float, C, D>& camera)
    {
        return readJson(path, camera);
    }
};
template <typename Float, CameraType C, DistorType D>
struct JSONWriter_<Camera_<Float, C, D>,
    typename std::enable_if<
        has_json_writer<Camera_<Float, C, D>>::value>::type> {
    static inline bool write(ConstStr& path, const Camera_<Float, C, D>& camera)
    {
        return writeJson(path, camera);
    }
};

}  // namespace rulermvs
#endif  // _RULERMVS_CORE_CAMERA_HPP_