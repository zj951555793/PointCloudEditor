#ifndef _RULERMVS_CORE_RGBD_HPP_
#define _RULERMVS_CORE_RGBD_HPP_
#include "rulermvs/pose.hpp"
#include "rulermvs/math.hpp"
#include "rulermvs/image.hpp"
#include "rulermvs/camera.hpp"
#include "rulermvs/normal.hpp"
#include "rulermvs/logger.hpp"
// #include "rulermvs/trimesh.hpp"
// #include "rulermvs/pointcloud.hpp"
namespace rulermvs
{
/// @brief 深度图帧,这里的实现采用特定类型
template <typename Pixel, typename Float> struct RGBDImage_
    : IRGBDImage_<Float> {
    RGBDImage_() {}
    RGBDImage_(const RGBDImage_& rgbd) : color(rgbd.color), depth(rgbd.depth) {}
    RGBDImage_(RGBDImage_&& rgbd) noexcept
    {
        std::swap(color, rgbd.color);
        std::swap(depth, rgbd.depth);
    }
    RGBDImage_(const Image_<Pixel>& color_img, const Image_<Float>& depth_img)
        : color(color_img), depth(depth_img)
    {}
    RGBDImage_(ConstStr& color_name, ConstStr& depth_name, float a = 0.015625f,
        float b = 0.f)
    {
        Image16u depth16u;
        if (!readImage(color_name, color))
            std::cout << "Read Image Failed.[" << color_name << "]\n";
        if (!readImage(depth_name, depth16u))
            std::cout << "Read Image Failed.[" << depth_name << "]\n";
        // 这里采用常用的定值,建议用户可以自己实现此类.
        convertTo(depth16u, depth, a, b);
    }
    virtual ~RGBDImage_() {}
    /// @brief 获取纹理图
    virtual const IImage& colorImage() const { return color; }
    /// @brief 获取深度图
    virtual const Image_<Float>& rangeImage() const { return depth; }

    Image_<Pixel> color;
    Image_<Float> depth;
};
typedef RGBDImage_<RGBPixel, float>      RGBDImage;
typedef RGBDImage_<RGBAPixel, float>     RGBADImage;
typedef RGBDImage_<unsigned char, float> GRAYDImage;

/// 范围约束接口
template <typename Float> struct IRGBDConstraint_ {
    virtual bool judge(const Point3_<Float>& point) const = 0;
};
/// 转台约束
struct TrunTableConstraint: public IRGBDConstraint_<float> {
    TrunTableConstraint() {}
    TrunTableConstraint(const TrunTableConstraint& constraint)
        : center(constraint.center)
        , normal(constraint.normal)
        , radius(constraint.radius)
        , height_up(constraint.height_up)
        , height_down(constraint.height_down)
    {}
    TrunTableConstraint(Point3f center, Point3f normal, float height_up,
        float height_down, float radius)
        : center(center)
        , normal(normal)
        , radius(radius)
        , height_up(height_up)
        , height_down(height_down)
    {}
    ~TrunTableConstraint() {}
    virtual std::shared_ptr<IRGBDConstraint_<float>> clone() const
    {
        return std::make_shared<TrunTableConstraint>(*this);
    }
    virtual bool judge(const Point3f& pt) const
    {
        float height_dist = (pt - center).dot(normal);

        float lengthN1 = 1.0f;
        float lengthN2 = (float)(pt - center).norm();

        float width_dist = 0.0;

        if (lengthN1 * lengthN2 != 0) {
            float angle = std::acos(height_dist / (lengthN1 * lengthN2));
            width_dist  = std::abs(lengthN2 * std::sin(angle));
        }
        return ((height_dist < height_up && height_dist > height_down) ||
                width_dist > radius);
    }

    Point3f center;       ///< 中心点
    Point3f normal;       ///< 法线
    float   radius;       ///< 半径
    float   height_up;    ///< 上偏移
    float   height_down;  ///< 下偏移
};

template <typename Float>
static inline void depth2Mask(const Image_<Float>& depth, Image_<uchar>& mask,
    coord_traits_t<Float> min_z = 0, coord_traits_t<Float> max_z = INFINITY)
{
    if (depth.empty()) return;
    mask.create(depth.size());
    mask.memsetZero();
    for (int y = 0; y < depth.height; ++y) {
        auto mask_ptr  = mask.ptr(y);
        auto depth_ptr = depth.ptr(y);
        for (int x = 0; x < depth.width; ++x) {
            const Float& z = depth_ptr[x];
            if (min_z < z && z < max_z) mask_ptr[x] = 255;
        }
    }
}
template <typename Float>
static inline Image_<uchar> depth2Mask(const Image_<Float>& depth,
    coord_traits_t<Float> min_z = 0, coord_traits_t<Float> max_z = INFINITY)
{
    Image_<uchar> mask;
    depth2Mask<Float>(depth, mask, min_z, max_z);
    return mask;
}

/// @brief 将深度图转成点云
MVS_EXPORT void depth2Points(
    const Imagef& depth, const CameraP& cam, Point3fVec& pts);

/// @brief 将深度图转成点云
MVS_EXPORT void depth2Points(
    const Imaged& depth, const CameraP& cam, Point3dVec& pts);

/// @brief 将深度图转成有序点云
MVS_EXPORT void depth2Vmap(
    const Imagef& depth, const CameraP& cam, Image3f& vmap);

/// @brief 将深度图转成有序点云
MVS_EXPORT void depth2Vmap(
    const Imaged& depth, const CameraP& cam, Image3d& vmap);

/// @brief 将深度图转成有序点云
MVS_EXPORT void depth2Vmap(const Imagef& depth, const Image8u& mask,
    const CameraP& cam, Image3f& vmap);

/// @brief 将深度图转成有序点云
MVS_EXPORT void depth2Vmap(const Imaged& depth, const Image8u& mask,
    const CameraP& cam, Image3d& vmap);

// template <typename Float> static inline Image_<Point3_<Float>> depth2Vmap(
//    const Image_<Float>& depth, const CameraP& cam)
//{
//    Image_<Point3_<Float>> vmap;
//    depth2Vmap(depth, cam, vmap);
//    return vmap;
//}
//template <typename Float>
//static inline void depth2Vmap(const Image_<Float>& depth, const Image8u& mask,
//    const CameraP& cam, Image_<Point3_<Float>>& vmap)
//{
//    assert(!depth.empty() && cam.size().valid());
//    if (mask.size() != depth.size()) {
//        depth2Vmap(depth, cam, vmap);
//        return;
//    }
//    const auto cam_p = resizeCamera(cam, depth.size());
//
//    std::vector<Float> u(depth.width);
//    for (int j = 0; j < depth.width; ++j)
//        u[j] = (Float)((j - cam_p.cx) / cam_p.fx);
//
//    vmap.create(depth.size());
//    vmap.memsetZero();
//    for (int i = 0; i < depth.height; i++) {
//        auto v = (Float)((i - cam_p.cy) / cam_p.fy);
//        auto vmap_ptr  = vmap.ptr(i);
//        auto mask_ptr  = mask.ptr(i);
//        auto depth_ptr = depth.ptr(i);
//        for (int j = 0; j < depth.width; ++j) {
//            const Float& z = depth_ptr[j];
//            if (mask_ptr[j]) vmap_ptr[j] = {u[j] * z, v * z, z};
//        }
//    }
//}
//template <typename Float> static inline Image_<Point3_<Float>> depth2Vmap(
//    const Image_<Float>& depth, const Image8u& mask, const CameraP& cam)
//{
//    Image_<Point3_<Float>> vmap;
//    depth2Vmap(depth, mask, cam, vmap);
//    return vmap;
//}

//template <typename Float> static inline void depth2Points(
//    const Image_<Float>& depth, const CameraP& cam, Point3Vec<Float>& pts)
//{
//    assert(!depth.empty() && cam.size().valid());
//    int pt_num = countNonZero(depth);
//    if (pt_num <= 0) return;
//    const auto cam_p = resizeCamera(cam, depth.size());
//    pts.resize(pt_num);
//    std::vector<Float> u(depth.width);
//    for (int j = 0; j < depth.width; ++j)
//        u[j] = (Float)((j - cam_p.cx) / cam_p.fx);
//    pt_num = 0;
//    for (int i = 0; i < depth.height; i++) {
//        auto v = (Float)((i - cam_p.cy) / cam_p.fy);
//
//        auto depth_ptr = depth.ptr(i);
//        for (int j = 0; j < depth.width; ++j) {
//            const Float& z = depth_ptr[j];
//            if (z) pts[pt_num++] = {u[j] * z, v * z, z};
//        }
//    }
//}

// template <typename Float>
// static inline std::vector<Point3_<Float>> depth2Points(
//    const Image_<Float>& depth, const CameraP& cam)
//{
//    Point3Vec<Float> pts;
//    depth2Points<Float>(depth, cam, pts);
//    return pts;
//}
template <typename Float> static inline void vmap2Points(
    const Image_<Point3_<Float>>& vmap, Point3Vec<Float>& pts)
{
    assert(!vmap.empty());
    pts.clear();
    for (int y = 0; y < vmap.height; y++) {
        auto vmap_ptr = vmap.ptr(y);
        for (int x = 0; x < vmap.width; ++x) {
            if (vmap_ptr[x].z) pts.emplace_back(vmap_ptr[x]);
        }
    }
}

template <typename Float> static inline std::vector<Point3_<Float>> vmap2Points(
    const Image_<Point3_<Float>>& vmap)
{
    std::vector<Point3_<Float>> points;
    vmap2Points<Float>(vmap, points);
    return points;
}

template <typename Float> MVS_DEPRECATED static inline void vmap2Points(
    const Image_<Point3_<Float>>& vmap, const Image_<uchar>& mask,
    Point3Vec<Float>& points)
{
    cvtImgToVec(vmap, mask, points);
}

template <typename Float>
MVS_DEPRECATED static inline std::vector<Point3_<Float>> vmap2Points(
    const Image_<Point3_<Float>>& vmap, const Image_<uchar>& mask)
{
    return vmap[mask];
}

template <typename Float> static inline void nmap2Normals(
    const Image_<Point3_<Float>>& nmap, Point3Vec<Float>& normals)
{
    assert(!nmap.empty());
    normals.clear();
    for (int y = 0; y < nmap.height; y++) {
        auto nmap_ptr = nmap.ptr(y);
        for (int x = 0; x < nmap.width; ++x) {
            if (nmap_ptr[x].x || nmap_ptr[x].y || nmap_ptr[x].z)
                normals.emplace_back(nmap_ptr[x]);
        }
    }
}
template <typename Float>
static inline Point3Vec<Float> nmap2Normals(const Image_<Point3_<Float>>& nmap)
{
    Point3Vec<Float> normals;
    nmap2Normals<Float>(nmap, normals);
    return normals;
}
template <typename Float> MVS_DEPRECATED static inline void nmap2Normals(
    const Image_<Point3_<Float>>& nmap, const Image_<uchar>& mask,
    Point3Vec<Float>& normals)
{
    cvtImgToVec(nmap, mask, normals);
}
template <typename Float>
MVS_DEPRECATED static inline std::vector<Point3_<Float>> nmap2Normals(
    const Image_<Point3_<Float>>& nmap, const Image_<uchar>& mask)
{
    return nmap[mask];
}

template <typename Float>
static inline void vmapAndNmapToMask(const Image_<Point3_<Float>>& vmap,
    const Image_<Point3_<Float>>& nmap, Image_<uchar>& mask)
{
    assert(!vmap.empty() && vmap.size() == nmap.size());
    mask.create(vmap.size());
    mask.memsetZero();
    for (int y = 0; y < vmap.height; y++) {
        auto vmap_ptr = vmap.ptr(y);
        auto nmap_ptr = nmap.ptr(y);
        auto mask_ptr = mask.ptr(y);
        for (int x = 0; x < vmap.width; ++x) {
            if (vmap_ptr[x].z &&
                (nmap_ptr[x].x || nmap_ptr[x].y || nmap_ptr[x].z))
                mask_ptr[x] = 255;
        }
    }
}
template <typename Float> static inline Image_<uchar> vmapAndNmapToMask(
    const Image_<Point3_<Float>>& vmap, const Image_<Point3_<Float>>& nmap)
{
    Image_<uchar> mask;
    vmapAndNmapToMask<Float>(vmap, nmap, mask);
    return mask;
}

template <typename Float> static inline void vmapToMask(
    const Image_<Point3_<Float>>& vmap, Image_<uchar>& mask)
{
    assert(!vmap.empty());
    mask.create(vmap.size());
    mask.memsetZero();
    for (int y = 0; y < vmap.height; y++) {
        auto vmap_ptr = vmap.ptr(y);
        auto mask_ptr = mask.ptr(y);
        for (int x = 0; x < vmap.width; ++x) {
            if (vmap_ptr[x].z) mask_ptr[x] = 255;
        }
    }
}
template <typename Float>
static inline Image_<uchar> vmapToMask(const Image_<Point3_<Float>>& vmap)
{
    Image_<uchar> mask;
    vmapToMask<Float>(vmap, mask);
    return mask;
}

/// @brief 归一化有序点云法向
/// @tparam Float 浮点类型
/// @param nmap 有序点云法向
template <typename Float>
static inline void normalizeNmap(Image_<Point3_<Float>>& nmap)
{
    if (nmap.empty()) return;
    for (int y = 0; y < nmap.height; ++y) {
        auto nmap_ptr = nmap.ptr(y);
        for (int x = 0; x < nmap.width; ++x) {
            nmap_ptr[x] = normalize(nmap_ptr[x]);
        }
    }
}

/// @brief 快速计算有序点云法向
/// @param vmap 有序点云
/// @param nmap 有序点云对应法向
/// @param max_angle 最大法向夹角
MVS_EXPORT void vmap2Nmap(
    const Image3f& vmap, Image3f& nmap, float max_angle = (float)MVS_PI_2);

/// @brief 快速计算有序点云法向
/// @param vmap 有序点云
/// @param nmap 有序点云对应法向
/// @param max_angle 最大法向夹角
MVS_EXPORT void vmap2Nmap(
    const Image3d& vmap, Image3d& nmap, double max_angle = MVS_PI_2);

template <typename Float>
Image_<Point3_<Float>> vmap2Nmap(const Image_<Point3_<Float>>& vmap)
{
    Image_<Point3_<Float>> nmap;
    vmap2Nmap(vmap, nmap);
    return nmap;
}

/// @brief 将深度图转换成有序点云并计算法向
/// @tparam Float 浮点类型
/// @param depth 深度图
/// @param camera 相机内参
/// @param vmap 有序点云
/// @param nmap 有序点云对应法向
template <typename Float> void depth2VmapAndNmap(const Image_<Float>& depth,
    const CameraP& cam, Image3_<Float>& vmap, Image3_<Float>& nmap)
{
    depth2Vmap(depth, cam, vmap);
    vmap2Nmap(vmap, nmap);
}

template <typename Float> void depth2VmapAndNmap(const Image_<Float>& depth,
    const Image8u& mask, const CameraP& cam, Image3_<Float>& vmap,
    Image3_<Float>& nmap)
{
    depth2Vmap(depth, mask, cam, vmap);
    vmap2Nmap(vmap, nmap);
}

/// @brief 将深度图转换成有序点云并计算法向
/// @tparam Float 浮点类型
/// @param depth 深度图
/// @param camera 相机内参
/// @param vmap 有序点云
/// @param nmap 有序点云对应法向
/// @param mask 有序点云对应的掩模
template <typename Float> void depth2VmapAndNmap(const Image_<Float>& depth,
    const CameraP& cam, Image3_<Float>& vmap, Image3_<Float>& nmap,
    Image_<uchar>& mask)
{
    depth2Vmap(depth, cam, vmap);
    vmap2Nmap(vmap, nmap);
    vmapAndNmapToMask(vmap, nmap, mask);
}

/// @brief 缩放深度图
/// @tparam Float 浮点类型
/// @param src 源深度图
/// @param new_sz 缩放后的尺寸
/// @param dst 输出深度图
/// @param point_z 最大点间距
template <typename Float,
    class = typename std::enable_if<std::is_floating_point<Float>::value>::type>
static inline void resizeDepth(const Image_<Float>& src, Size new_sz,
    Image_<Float>& dst, Float point_z = INFINITY)
{
    assert(!src.empty() && new_sz.valid());
    if (src.size() == new_sz) {
        dst = src.clone();
        return;
    }
    Image_<Float> img;
    if (dst.size() == new_sz && src.data != dst.data) img = dst;
    if (img.size() != new_sz) img.create(new_sz);
    img.memsetZero();
    const double sx = src.width / (double)new_sz.width;
    const double sy = src.height / (double)new_sz.height;
    for (int i = 0; i < img.height; ++i) {
        auto img_ptr = img.ptr(i);
        for (int j = 0; j < img.width; ++j) {
            auto x  = j * sx;
            auto y  = i * sy;
            int  xi = (int)(x), yi = (int)(y);
            int  xc = xi + 1, yc = yi + 1;
            if (xc > 0 && yc > 0 && xc < src.width && yc < src.height) {
                auto& z0 = src.ptr(yi)[xi];
                auto& z1 = (&z0)[1];
                auto& z2 = *(Float*)((uchar*)&z0 + src.stride);
                auto& z3 = (&z2)[1];
                // 判断是否为有效点
                if (z0 > 0 && z1 > 0 && z2 > 0 && z3 > 0) {
                    // 判断深度之间的Z方向距离
                    if (std::abs(z1 - z0) < point_z &&
                        std::abs(z2 - z0) < point_z &&
                        std::abs(z3 - z0) < point_z) {
                        double a0 = xc - x, b0 = x - xi;
                        double a1 = yc - y, b1 = y - yi;
                        img_ptr[j] = (Float)((z0 * a0 + z1 * b0) * a1 +
                                             (z2 * a0 + z3 * b0) * b1);
                    }
                }
            }
        }
    }
    dst = img;
}
template <typename Float,
    class = typename std::enable_if<std::is_floating_point<Float>::value>::type>
static inline Image_<Float> resizeDepth(
    const Image_<Float>& src, Size new_sz, Float point_z = INFINITY)
{
    Image_<Float> dst;
    resizeDepth(src, new_sz, dst, point_z);
    return dst;
}

/// @brief 重映射深度图
/// @tparam Float 浮点类型
/// @param src 源深度图
/// @param mapx 存储X的图像阵列
/// @param mapy 存储Y的图像阵列
/// @param dst 输出深度图
/// @param point_z 点间距
template <typename Float,
    class = typename std::enable_if<std::is_floating_point<Float>::value>::type>
static inline void remapDepth(const Image_<Float>& src, const Imagef& mapx,
    const Imagef& mapy, Image_<Float>& dst, double point_z = INFINITY)
{
    assert(!src.empty() && !mapx.empty() && !mapy.empty() &&
           mapx.size() == mapy.size());
    if (dst.size() != mapx.size()) dst.create(mapx.size());
    dst.memsetZero();
    for (int i = 0; i < mapx.height; ++i) {
        auto* d_ptr  = dst.ptr(i);
        auto* mx_ptr = mapx.ptr(i);
        auto* my_ptr = mapy.ptr(i);
        for (int j = 0; j < mapx.width; ++j) {
            auto x  = mx_ptr[j];
            auto y  = my_ptr[j];
            int  xi = (int)(x), yi = (int)(y);
            int  xc = xi + 1, yc = yi + 1;
            if (xc > 0 && yc > 0 && xc < src.width && yc < src.height) {
                auto& z0 = src.ptr(yi)[xi];
                auto& z1 = (&z0)[1];
                auto& z2 = *(Float*)((uchar*)&z0 + src.stride);
                auto& z3 = (&z2)[1];
                // 判断是否为有效点
                if (z0 > 0 && z1 > 0 && z2 > 0 && z3 > 0) {
                    // 判断深度之间的Z方向距离
                    if (std::abs(z1 - z0) < point_z &&
                        std::abs(z2 - z0) < point_z &&
                        std::abs(z3 - z0) < point_z) {
                        float a0 = xc - x, b0 = x - xi;
                        float a1 = yc - y, b1 = y - yi;
                        d_ptr[j] = (Float)((z0 * a0 + z1 * b0) * a1 +
                                           (z2 * a0 + z3 * b0) * b1);
                    }
                }
            }
        }
    }
}

/// @brief 通过uv坐标和深度图，获取三维点和法向
/// @tparam Float 浮点类型
/// @param depth 深度图
/// @param uvs uv坐标
/// @param camera 相机内参，需要跟UV坐标对应.
/// @param points 三维点坐标
/// @param normals 三维点法向
template <typename Float> void sampleDepthFromUV(const Imagef& depth,
    const std::vector<Point2_<Float>>& uvs, const CameraP& cam,
    Point3Vec<Float>& points, Point3Vec<Float>& normals)
{
    assert(!depth.empty() && cam.size().valid());
    if (uvs.empty()) return;
    const auto cam_p = resizeCamera(cam, depth.size());
    points.resize(uvs.size());
    memset(&points[0].x, 0, sizeof(Point3_<Float>) * points.size());
    normals.resize(uvs.size());
    Point3_<Float> rect_pts[4];
    const int      w_1           = depth.width - 1;
    const int      h_1           = depth.height - 1;
    double         covariance[9] = {0}, X[3] = {0};
    const size_t   covraiance_byte = 9 * sizeof(double);
    for (size_t i = 0; i < uvs.size(); ++i) {
        auto& pt  = points[i];
        auto  uv  = cam_p.point2uv(cam.uv2point(uvs[i]));
        int   x_f = (int)uv.x, y_f = (int)uv.y;
        // 如果都在有效范围内
        if (uv.x > 0 && uv.y > 0 && x_f < w_1 && y_f < h_1) {
            auto& d00 = depth.ptr(y_f)[x_f];
            auto& d01 = (&d00)[1];
            auto& d10 = (&d00)[depth.width];
            auto& d11 = (&d10)[1];
            if (d00 > 0 && d01 > 0 && d10 > 0 && d11 > 0) {
                float b0 = uv.x - (float)x_f, a0 = 1.0f - b0;
                float b1 = uv.y - (float)y_f, a1 = 1.0f - b1;
                float depth_v =
                    a1 * (a0 * d00 + b0 * d01) + b1 * (a0 * d10 + b0 * d11);
                pt          = cam_p.uv2point(uv) * depth_v;
                rect_pts[0] = cam_p.uv2point(Point2_<Float> {x_f, y_f}) * d00;
                rect_pts[1] =
                    cam_p.uv2point(Point2_<Float> {x_f + 1, y_f}) * d01;
                rect_pts[2] =
                    cam_p.uv2point(Point2_<Float> {x_f, y_f + 1}) * d10;
                rect_pts[3] =
                    cam_p.uv2point(Point2_<Float> {x_f + 1, y_f + 1}) * d11;
                memset(&covariance[0], 0, covraiance_byte);
                for (int j = 0; j < 4; ++j) {
                    auto pt_vec = rect_pts[j] - pt;
                    covariance[0] += pt_vec.x * pt_vec.x;
                    covariance[1] += pt_vec.x * pt_vec.y;
                    covariance[2] += pt_vec.x * pt_vec.z;
                    covariance[4] += pt_vec.y * pt_vec.y;
                    covariance[5] += pt_vec.y * pt_vec.z;
                    covariance[8] += pt_vec.z * pt_vec.z;
                }
                covariance[3] = covariance[1];
                covariance[6] = covariance[2];
                covariance[7] = covariance[5];
                solveZ3x3(covariance, X);
                auto nl    = normalize(Point3_<Float>(X[0], X[1], X[2]));
                normals[i] = nl.dot(pt) > 0 ? nl : -nl;
            }
        }
    }
}

/// @brief 基于深度图采样线，用于仿真
/// @tparam Float 浮点类型
/// @param depth 深度图
/// @param camera 针孔相机类型
/// @param theta 旋转角度，起始角度朝向为正上;起始点为相机中心;
/// @param points 输出三维点云，通过双线性插值构造；
/// @param sample_num 采样预期点数量，实际数量会少于此；
template <typename Float>
static void sampleLineFromDepth(const Image_<Float>& depth,
    const CameraP& camera, coord_traits_t<Float> theta,
    Point3Vec<Float>& points, int sample_num = 15000)
{
    if (depth.empty() || sample_num < 1) return;
    const int w_1      = depth.width - 1;
    const int h_1      = depth.height - 1;
    auto      camera_p = resizeCamera(camera, depth.size());
    auto      vec      = Point2_<Float>(std::sin(theta), std::cos(theta));
    Float     cx = (Float)camera_p.cx, cy = (Float)camera_p.cy;
    // 记录交点信息
    Point2Vec<Float> nodes;
    Float            y1 =
        vec.x ? (-cx / vec.x) * vec.y + cy : -std::numeric_limits<Float>::max();
    if (y1 >= 0 && y1 <= (Float)h_1) nodes.emplace_back(0.f, y1);
    Float y2 = vec.x ? (((Float)w_1 - cx) / vec.x) * vec.y + cy :
                       std::numeric_limits<Float>::max();
    if (y2 >= 0 && y2 <= (Float)h_1) nodes.emplace_back((Float)w_1, y2);
    Float x1 =
        vec.y ? (-cy / vec.y) * vec.x + cx : -std::numeric_limits<Float>::max();
    if (x1 > 0 && x1 < (Float)w_1) nodes.emplace_back(x1, 0.f);
    Float x2 = vec.y ? (((Float)h_1 - cy) / vec.y) * vec.x + cx :
                       std::numeric_limits<Float>::max();
    if (x1 > 0 && x1 < (Float)w_1) nodes.emplace_back(x2, (Float)h_1);
    if (nodes.size() != 2)
        throw std::runtime_error("Error: Unexpected condition.");
    // int axis = std::abs(vec.x) > std::abs(vec.y) ? 0 : 1;
    Float c = 0;
    if (std::abs(vec.x) > std::abs(vec.y)) {
        c = (nodes[1].x - nodes[0].x) / vec.x;
    } else {
        c = (nodes[1].y - nodes[0].y) / vec.y;
    }
    int pt_num = 0;
    points.resize(sample_num);
    auto offset = vec * c / (Float)sample_num;
    for (int i = 0; i < sample_num; ++i) {
        auto& pt  = points[pt_num];
        auto  uv  = nodes[0] + offset * (Float)i;
        int   x_f = (int)uv.x, y_f = (int)uv.y;
        // 如果都在有效范围内
        if (uv.x > 0 && uv.y > 0 && x_f < w_1 && y_f < h_1) {
            auto& d00 = depth.ptr(y_f)[x_f];
            auto& d01 = (&d00)[1];
            auto& d10 = *(Float*)((uchar*)&d00 + depth.stride);
            auto& d11 = (&d10)[1];
            if (d00 > 0 && d01 > 0 && d10 > 0 && d11 > 0) {
                float b0 = uv.x - (float)x_f, a0 = 1.0f - b0;
                float b1 = uv.y - (float)y_f, a1 = 1.0f - b1;
                float depth_v =
                    a1 * (a0 * d00 + b0 * d01) + b1 * (a0 * d10 + b0 * d11);
                pt = camera_p.uv2point(uv) * depth_v;
                pt_num++;
            }
        }
    }
    points.resize(pt_num);
}

/// @brief 快速搜索对应点
/// @param src_vmap 源有序三维点云
/// @param dst_vmap 目标有序三维点云
/// @param camera 相机内参,与dst_vmap对应.
/// @param rt 相对位姿
/// @param matchs 匹配索引
/// @param max_dist 对应点的最大欧式距离
/// @return 返回帧间重叠率
MVS_EXPORT double findPointCorres(const Image3f& srcVmap,
    const Image3f& dstVmap, const CameraP& camera, const Pose& RT,
    DMatchVec& matchs, float max_dist);

/// @brief 快速搜索对应点
/// @param src_vmap 源有序点云
/// @param src_nmap 源有序法向
/// @param dst_vmap 目标有序点云
/// @param dst_nmap 目标有序法向
/// @param camera 相机内参
/// @param rt 相对姿态
/// @param matchs 匹配索引
/// @param max_dist 对应的最大距离
/// @param max_angle 对应点的最大法向夹角
/// @return 返回帧间重叠率
MVS_EXPORT double findPointCorres(const Image3f& srcVmap,
    const Image3f& srcNmap, const Image3f& dstVmap, const Image3f& dstNmap,
    const CameraP& camera, const Pose& RT, DMatchVec& matchs, float max_dist,
    float max_angle);

/// @brief 腐蚀深度图
/// @param depth 深度图
/// @param ksize 窗口尺寸
/// @param iter 迭代次数
MVS_EXPORT void erodeDepth(const Imagef& depth, int ksize = 3, int iter = 1);

/// @brief 模糊深度图
/// @param depth 深度图
/// @param ksize 窗口尺寸
MVS_EXPORT void medianBlurDepth(const Imagef& depth, int ksize = 3);

#ifdef RULERMVS_USE_SSE


/// @brief 法向归一化指令集加速.
static inline void normalizeNmap(Image3f& nmap)
{
    if (nmap.empty()) return;
    const auto nloop = nmap.width >> 2;
    const auto zero  = _mm_setzero_ps();
    for (int i = 0; i < nmap.height; ++i) {
        auto nmap_ptr = nmap.ptr(i);
        for (int j = 0; j < nloop; ++j) {
            auto nl   = &nmap_ptr[j << 2];
            auto nx   = _mm_setr_ps(nl[0].x, nl[1].x, nl[2].x, nl[3].x);
            auto ny   = _mm_setr_ps(nl[0].y, nl[1].y, nl[2].y, nl[3].y);
            auto nz   = _mm_setr_ps(nl[0].z, nl[1].z, nl[2].z, nl[3].z);
            auto norm = _mm_sqrt_ps(_mm_add_ps(_mm_mul_ps(nx, nx),
                _mm_add_ps(_mm_mul_ps(ny, ny), _mm_mul_ps(nz, nz))));
            nx        = _mm_div_ps(nx, norm);
            ny        = _mm_div_ps(ny, norm);
            nz        = _mm_div_ps(nz, norm);
            auto mask = _mm_cmpgt_ps(norm, zero);
            nx        = _mm_blendv_ps(zero, nx, mask);
            ny        = _mm_blendv_ps(zero, ny, mask);
            nz        = _mm_blendv_ps(zero, nz, mask);
            nl[0]     = {((float*)&nx)[0], ((float*)&ny)[0], ((float*)&nz)[0]};
            nl[1]     = {((float*)&nx)[1], ((float*)&ny)[1], ((float*)&nz)[1]};
            nl[2]     = {((float*)&nx)[2], ((float*)&ny)[2], ((float*)&nz)[2]};
            nl[3]     = {((float*)&nx)[3], ((float*)&ny)[3], ((float*)&nz)[3]};
        }
        for (int j = nloop << 2; j < nmap.width; ++j)
            nmap_ptr[j] = normalize(nmap_ptr[j]);
    }
}
#endif

}  // namespace rulermvs
#endif  // _RULERMVS_CORE_RGBD_HPP_