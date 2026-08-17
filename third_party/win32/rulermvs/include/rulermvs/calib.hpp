#ifndef _RULERMVS_CALIB_CALIB_HPP_
#define _RULERMVS_CALIB_CALIB_HPP_
#include "rulermvs/core.hpp"
#include "rulermvs/pose.hpp"
#include "rulermvs/rect.hpp"
#include "rulermvs/image.hpp"
#include "rulermvs/camera.hpp"
#include "rulermvs/logger.hpp"
#include "rulermvs/centroid.hpp"
namespace rulermvs
{

/// 反投影
template <typename Tp, typename Float, CameraType C, DistorType D>
void reprojectPoints(const Camera_<Float, C, D>& cam, const Pose_<Float>& rt,
    const Point3_<Tp>* pts, Point2_<Tp>* uvs, size_t pt_num)
{
    for (size_t i = 0; i < pt_num; ++i)
        uvs[i] = cam.reproject(rt.transform(pts[i]));
}
template <typename Tp, typename Float, CameraType C, DistorType D>
void reprojectPoints(const Camera_<Float, C, D>& cam, const Pose_<Float>& rt,
    const std::vector<Point3_<Tp>>& pts, std::vector<Point2_<Tp>>& uvs)
{
    if (pts.empty()) return;
    if (uvs.size() != pts.size()) uvs.resize(pts.size());
    reprojectPoints(cam, rt, &pts[0], &uvs[0], pts.size());
}
template <typename Tp, typename Float, CameraType C, DistorType D>
void undistorPoints(const Camera_<Float, C, D>& cam, const Rotation_<Float>& R,
    const Camera_<Float, C>& new_cam, const Point2_<Tp>* uvs,
    Point2_<Tp>* un_uvs, size_t pt_num)
{
    for (size_t i = 0; i < pt_num; ++i)
        un_uvs[i] =
            new_cam.point2uv(R.rotate(cam.undistor(cam.uv2point(uvs[i]))));
}
template <typename Tp, typename Float, CameraType C, DistorType D>
void undistorPoints(const Camera_<Float, C, D>& cam, const Rotation_<Float>& R,
    const Camera_<Float, C>& cam_new, const std::vector<Point2_<Tp>>& uvs,
    std::vector<Point2_<Tp>>& un_uvs)
{
    if (uvs.empty()) return;
    if (uvs.size() != un_uvs.size()) un_uvs.resize(uvs.size());
    undistorPoints(cam, R, cam_new, &uvs[0], &un_uvs[0], uvs.size());
}
template <typename Tp, typename Float, CameraType C, DistorType D>
std::vector<Point2_<Tp>> undistorPoints(const Camera_<Float, C, D>& cam,
    const Rotation_<Float>& R, const Camera_<Float, C>& new_cam,
    const std::vector<Point2_<Tp>>& uvs)
{
    std::vector<Point2_<Tp>> un_uvs;
    undistorPoints(cam, R, new_cam, uvs, un_uvs);
    return un_uvs;
}
/// @brief 创建立体校正的映射矩阵
/// @tparam Tp 记录坐标的基类型[float-double]
/// @tparam Float 浮点类型
/// @tparam C 相机模型
/// @tparam D 畸变模型
/// @param cam 输入相机参数
/// @param R 旋转矩阵
/// @param new_cam 校正后的相机参数
/// @param mapx 记录x坐标
/// @param mapy 记录y坐标
template <typename Tp, typename Float, CameraType C, DistorType D,
    CameraType C2 = C>
void createUndistorRectifyMap(const Camera_<Float, C, D>& cam,
    const Rotation_<Float>& R, const Camera_<Float, C2>& new_cam,
    Image_<Tp>& mapx, Image_<Tp>& mapy)
{
    if (!new_cam.size().valid()) return;
    auto R_inv = R.inv();
    mapx.create(new_cam.size());
    mapy.create(new_cam.size());
    for (int i = 0; i < new_cam.height; ++i) {
        Tp *ptrx = mapx.ptr(i), *ptry = mapy.ptr(i);
        for (int j = 0; j < new_cam.width; ++j) {
            Point2_<Float> uv = cam.point2uv(
                static_cast<DistorModel_<D, Float>>(cam).distor(R_inv.rotate(
                    new_cam.uv2point(Point2_<Float> {(Float)j, (Float)i}))));
            ptrx[j] = (Tp)uv.x;
            ptry[j] = (Tp)uv.y;
        }
    }
}
/// @brief 创建立体校正的映射矩阵的简化函数
/// @tparam Tp 记录坐标的基类型[float-double]
/// @tparam Float 浮点类型
/// @tparam C 相机模型
/// @tparam D 畸变模型
/// @param cam 输入相机参数
/// @param mapx 记录x坐标
/// @param mapy 记录y坐标
template <typename Tp, typename Float, CameraType C, DistorType D>
void createUndistorRectifyMap(
    const Camera_<Float, C, D>& cam, Image_<Tp>& mapx, Image_<Tp>& mapy)
{
    createUndistorRectifyMap<Tp, Float, C, D>(
        cam, Rotation_<Float>(), cam.nodistor(), mapx, mapy);
}
/// @brief 对影像进行畸变校正
/// @tparam Tp 图像数据类型
/// @tparam Float 浮点类型
/// @tparam C 相机类型
/// @tparam D 畸变类型
/// @param img 输入图像
/// @param cam 相机参数
/// @param R 旋转矩阵
/// @param new_cam 校正后的相机内参
/// @param un_img 校正后的图像
template <typename Tp, typename Float, CameraType C, DistorType D,
    CameraType C2 = C>
void undistorImage(const Image_<Tp>& img, const Camera_<Float, C, D>& cam,
    const Rotation_<Float>& R, const Camera_<Float, C2>& new_cam,
    Image_<Tp>& un_img)
{
    auto R_inv = R.inv();
    un_img.create(new_cam.size());
    if (img.empty() || un_img.empty()) return;
    un_img.memsetZero();
    for (int i = 0; i < un_img.height; ++i) {
        Tp* undistor_ptr = un_img.ptr(i);
        for (int j = 0; j < un_img.width; ++j) {
            auto uv = cam.point2uv(cam.distor(
                R_inv.rotate(new_cam.uv2point(Point2_<Float> {j, i}))));
            interpolate<Tp, Float, SamplerType::MVS_BILINEAR>(
                img, uv.x, uv.y, undistor_ptr[j]);
        }
    }
}
/// @brief 对影像进行基表校正
/// @tparam Tp 图像的基本数据类型
/// @tparam Float 浮点类型
/// @tparam C 相机模型
/// @tparam D 畸变模型
/// @param img 输入图像
/// @param cam 相机内参
/// @param un_img 校正影像
template <typename Tp, typename Float, CameraType C, DistorType D>
void undistorImage(
    const Image_<Tp>& img, const Camera_<Float, C, D>& cam, Image_<Tp>& un_img)
{
    undistorImage<Tp, Float, C, D>(
        img, cam, Rotation_<Float>(), cam.nodistor(), un_img);
}
/// @brief 针孔相机模型的立体校正参数计算
/// @tparam Float 浮点类型
/// @tparam D 畸变模型
/// @param cam1 相机1内参
/// @param cam2 相机2内参
/// @param rt 旋转矩阵[rt2*rt1.inv()]
/// @param new_sz 校正后的相机尺寸
/// @param R1 相机1的旋转矩阵
/// @param new_cam1 相机1的校正后内参
/// @param R2 相机2的旋转矩阵
/// @param new_cam2 相机2的校正后内参
/// @param Q Q矩阵，借鉴OpenCV定义.
/// @param alpha 标识计算新内参时的过程.
template <typename Float, DistorType D = DistorType::Brown>
void stereoRectify(const CameraP_<Float, D>& cam1,
    const CameraP_<Float, D>& cam2, const Pose_<Float>& rt, Size new_sz,
    Rotation_<Float>& R1, CameraP_<Float, DistorType::None>& new_cam1,
    Rotation_<Float>& R2, CameraP_<Float, DistorType::None>& new_cam2,
    double Q[16] = nullptr, double alpha = 0)
{
    // typedef DistorModel_<D, Float> DistorBase;
    // if (alpha != 0) alpha = 0;
    enum RectifyType : int { Inner = 0, Left, Right, Both = 4, Outer = 8 };
    // 计算相机之间各自的旋转角度, 以使主光轴的朝向一致;
    Float rvec[3];
    auto  rot = rt.getRotation();
    toRodrigues((Float*)&rot.a1, rvec);
    *(Point3_<Float>*)rvec *= -0.5f;
    Rotation_<Float> R_2(rvec[0], rvec[1], rvec[2]);
    Point3_<Float>   tvec = R_2.rotate(rt.getTranslation());
    int              idx  = fabs(tvec.x) > fabs(tvec.y) ? 0 : 1;
    Point3_<Float>   uvec(0, 0, 0);
    (&uvec.x)[idx] = (&tvec.x)[idx] > 0 ? 1 : -1;
    double theta =
        (double)std::acos(tvec.dot(uvec) / tvec.norm() / uvec.norm());
    Point3_<Float> wvec = normalize(tvec.cross(uvec));
    wvec *= theta;
    Rotation_<Float> Rw(wvec.x, wvec.y, wvec.z);
    R1   = Rw * R_2.inv();
    R2   = Rw * R_2;
    tvec = R2.rotate(rt.getTranslation());

    // 计算新的相机焦距
    auto  cam01 = resizeCamera(cam1, new_sz);
    auto  cam02 = resizeCamera(cam2, new_sz);
    Float f1    = idx == 0 ? cam01.fy : cam01.fx;
    int   w(new_sz.width), h(new_sz.height);
    f1 *= cam01.k1 < 0 ? 1 + cam01.k1 * (w * w + h * h) / (4 * f1 * f1) : 1;
    Float f2 = idx == 0 ? cam02.fy : cam02.fx;
    f2 *= cam02.k1 < 0 ? 1 + cam02.k1 * (w * w + h * h) / (4 * f2 * f2) : 1;
    Float fn = std::min(f1, f2);

    // 计算新的相机内参和Q矩阵
    Point2_<Float> pts[4] = {Point2_<Float> {0, 0}, Point2_<Float> {w - 1, 0},
        Point2_<Float> {0, h - 1}, Point2_<Float> {w - 1, h - 1}};
    Point2_<Float> un_pts_1[4], un_pts_2[4];
    for (size_t i = 0; i < 4; ++i) {
        auto pt       = R1.rotate(cam01.undistor(cam01.uv2point(pts[i])));
        un_pts_1[i].x = pt.x * fn / pt.z;
        un_pts_1[i].y = pt.y * fn / pt.z;
        pt            = R2.rotate(cam02.undistor(cam02.uv2point(pts[i])));
        un_pts_2[i].x = pt.x * fn / pt.z;
        un_pts_2[i].y = pt.y * fn / pt.z;
    }
    auto C1pt = centroidPoints<Float>(un_pts_1, 4);
    auto C2pt = centroidPoints<Float>(un_pts_2, 4);
    C1pt      = Point2_<Float>((w - 1) * 0.5, (h - 1) * 0.5) - C1pt;
    C2pt      = Point2_<Float>((w - 1) * 0.5, (h - 1) * 0.5) - C2pt;
    if (idx == 0) {
        C1pt.y = C2pt.y = (C1pt.y + C2pt.y) / 2;
    } else {
        C1pt.x = C2pt.x = (C1pt.x + C2pt.x) / 2;
    }
    new_cam1 = {new_sz, {fn, fn, C1pt.x, C1pt.y}};
    new_cam2 = {new_sz, {fn, fn, C2pt.x, C2pt.y}};

    Point2Vec<Float> contour = Rect_<Float>(Size_<Float>(new_sz)).contour();
    if (alpha > 0) {
        Rect_<double> rect1 =
            findMaxInnerRect(undistorPoints(cam01, R1, new_cam1, contour));
        Rect_<double> rect2 =
            findMaxInnerRect(undistorPoints(cam02, R2, new_cam2, contour));

        double sx =
            std::min(new_sz.width / rect1.width, new_sz.width / rect2.width);
        double sy = std::min(
            new_sz.height / rect1.height, new_sz.height / rect2.height);
        double s = std::min(sx, sy) * alpha;
        new_cam1 = {
            new_sz, {fn * s, fn * s, C1pt.x * s + (1 - s) * (w - 1) * 0.5,
                        C1pt.y * s + (1 - s) * (h - 1) * 0.5}};
        new_cam2 = {
            new_sz, {fn * s, fn * s, C2pt.x * s + (1 - s) * (w - 1) * 0.5,
                        C2pt.y * s + (1 - s) * (h - 1) * 0.5}};
    }
    // 输出Q矩阵,参考OpenCV实现.
    if (Q != nullptr) {
        Float tmp[16] = {1, 0, 0, -C1pt.x, 0, 0, 1, -C1pt.y, 0, 0, 0, fn, 0, 0,
            -1 / (&tvec.x)[idx],
            (idx == 0 ? C1pt.x - C2pt.x : C1pt.y - C2pt.y) / (&tvec.x)[idx]};
        memcpy(Q, tmp, 16 * sizeof(Float));
    }

    MVS_ILOG << "stereoRectify: New Left Camera: " << new_cam1 << std::endl;
    MVS_ILOG << "stereoRectify: New Right Camera: " << new_cam2 << std::endl;
    auto rect1 = findMinOuterRect(undistorPoints(cam01, R1, new_cam1, contour));
    auto rect2 = findMinOuterRect(undistorPoints(cam02, R2, new_cam2, contour));
    MVS_ILOG << "stereoRectify: Left Camera Valid ROI " << rect1
             << ", Right Camera Valid ROI" << rect2;
}

template <typename Float, DistorType D = DistorType::Brown>
void stereoRectify(const CameraP_<Float, D>& cam1, const Pose_<Float>& rt1,
    const CameraP_<Float, D>& cam2, const Pose_<Float>& rt2, Size new_sz,
    Rotation_<Float>& R1, CameraP_<Float, DistorType::None>& new_cam1,
    Rotation_<Float>& R2, CameraP_<Float, DistorType::None>& new_cam2,
    double alpha = 0)
{
    stereoRectify<Float, D>(
        cam1, cam2, rt2 * rt1.inv(), new_sz, R1, new_cam1, R2, new_cam2, alpha);
}

/// @brief 将视差图转换成深度图
/// @tparam Float 浮点类型
/// @param disparity 视差图
/// @param Q Q矩阵，通过立体校正得到
/// @param depth 深度图
template <typename Float> static inline void disparityToDepth(
    const Image_<Float>& disparity, const double Q[16], Image_<Float>& depth)
{
    if (disparity.empty() || Q == nullptr) return;
    if (depth.size() != disparity.size()) depth.create(disparity.size());
    for (int i = 0; i < depth.height; ++i) {
        auto depth_ptr     = depth.ptr(i);
        auto disparity_ptr = disparity.ptr(i);
        for (int j = 0; j < depth.width; ++j)
            if (!std::isnan(disparity_ptr[j]) && !std::isinf(disparity_ptr[j]))
                depth_ptr[j] =
                    (Float)(Q[11] / (disparity_ptr[j] * Q[14] + Q[15]));
            else
                depth_ptr[j] = 0;
    }
}

#ifdef RULERMVS_USE_SSE
static inline void cvt_disparity_to_depth_avx(
    const float* disparity, const double Q[16], float* depth, size_t sz)
{
    __m256 Q34 = _mm256_set1_ps((float)Q[11]);
    __m256 Q43 = _mm256_set1_ps((float)Q[14]);
    __m256 Q44 = _mm256_set1_ps((float)Q[15]);

    size_t nloop = sz >> 3;
    for (size_t i = 0; i < nloop; ++i) {
        _mm256_storeu_ps(&depth[i << 3],
            _mm256_div_ps(Q34,
                _mm256_add_ps(
                    _mm256_mul_ps(_mm256_loadu_ps(&disparity[i << 3]), Q43),
                    Q44)));
    }
    for (size_t i = nloop << 3; i < sz; ++i)
        depth[i] = (float)(Q[11] / (disparity[i] * Q[14] + Q[15]));
}

template <> inline void disparityToDepth<float>(
    const Imagef& disparity, const double Q[16], Imagef& depth)
{
    if (disparity.empty() || Q == nullptr) return;
    if (depth.size() != disparity.size()) depth.create(disparity.size());
    if (disparity.unbiased() && depth.unbiased()) {
        cvt_disparity_to_depth_avx(
            disparity.data, Q, depth.data, (size_t)depth.size().area());
    } else {
        for (int i = 0; i < depth.height; ++i) {
            cvt_disparity_to_depth_avx(
                disparity.ptr(i), Q, depth.ptr(i), (size_t)depth.width);
        }
    }
}
#endif

MVS_EXPORT bool detectCircleGrid();
}  // namespace rulermvs
#endif  // _RULERMVS_CALIB_CALIB_HPP_