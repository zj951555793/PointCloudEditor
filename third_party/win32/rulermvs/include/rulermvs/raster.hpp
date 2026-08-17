#ifndef _RULERMVS_CORE_RASTER_HPP_
#define _RULERMVS_CORE_RASTER_HPP_
#include "rulermvs/pose.hpp"
#include "rulermvs/image.hpp"
#include "rulermvs/point.hpp"
#include "rulermvs/scalar.hpp"
#include "rulermvs/camera.hpp"
namespace rulermvs
{
template <typename Float = float, typename Index = size_t>
void rasterDepth(const Scalar3_<Index>* tript_inds, size_t _tript_num,
    const Point3_<Float>* _points, const Point2_<Float>* _uvs,
    size_t _point_num, Image_<Float>& _depth)
{
    if (_depth.empty() || _point_num <= 0) return;
    _depth.memsetZero();
    Float MaxDepthWidth  = (Float)_depth.width - 1;
    Float MaxDepthHeight = (Float)_depth.height - 1;
    for (size_t k = 0; k < _tript_num; ++k) {
        auto& tri_ind = tript_inds[k];
        auto& imgp1   = _uvs[tri_ind[0]];
        auto& imgp2   = _uvs[tri_ind[1]];
        auto& imgp3   = _uvs[tri_ind[2]];

        auto v0    = imgp3 - imgp1;
        auto v1    = imgp2 - imgp1;
        auto dot00 = v0.dot(v0);
        auto dot01 = v0.dot(v1);
        auto dot11 = v1.dot(v1);

        auto minx = std::min(std::min(imgp1.x, imgp2.x), imgp3.x);
        auto miny = std::min(std::min(imgp1.y, imgp2.y), imgp3.y);
        auto maxx = std::max(std::max(imgp1.x, imgp2.x), imgp3.x);
        auto maxy = std::max(std::max(imgp1.y, imgp2.y), imgp3.y);
        minx      = std::max<Float>(minx, 0);
        miny      = std::max<Float>(miny, 0);
        maxx      = std::min(maxx, MaxDepthWidth);
        maxy      = std::min(maxy, MaxDepthHeight);

        Point2_<Float> v2;
        for (int i = (int)minx; i <= (int)maxx; ++i) {
            for (int j = (int)miny; j <= (int)maxy; ++j) {
                v2.x       = (Float)i - imgp1.x;
                v2.y       = (Float)j - imgp1.y;
                auto dot02 = v0.dot(v2);
                auto dot12 = v1.dot(v2);
                auto denom = (dot00 * dot11 - dot01 * dot01);
                auto u     = (dot11 * dot02 - dot01 * dot12) / denom;
                auto v     = (dot00 * dot12 - dot01 * dot02) / denom;
                // 判断此点是否在三角形中
                if (u >= 0 && v >= 0 && u + v <= 1) {
                    auto value =
                        (_points[tri_ind[2]].z - _points[tri_ind[0]].z) * u +
                        (_points[tri_ind[1]].z - _points[tri_ind[0]].z) * v +
                        _points[tri_ind[0]].z;
                    auto& depth_val = _depth.ptr(j)[i];
                    if (value > 0 && (!depth_val || value < depth_val))
                        depth_val = value;
                }
            }
        }
    }
}
template <typename Float = float, typename Index = size_t>
void rasterDepth(const std::vector<Scalar3_<Index>>& tript_inds,
    const std::vector<Point3_<Float>>&               _points,
    const std::vector<Point2_<Float>>& _uvs, Image_<Float>& _depth)
{
    if (tript_inds.empty() || _points.empty() || _points.size() != _uvs.size())
        return;
    rasterDepth<Float, Index>(&tript_inds[0], tript_inds.size(), &_points[0],
        &_uvs[0], _points.size(), _depth);
}
template <typename Float = float, typename Index = size_t>
void rasterDepth(const Scalar3_<Index>* _tript_inds, size_t _tript_num,
    const Point3_<Float>* _points, size_t _point_num,
    const CameraP& _camera, Image_<Float>& _depth)
{
    _depth.create(_camera.size());
    std::vector<Point2_<Float>> reproj_uvs(_point_num);
    _camera.reproject(_points, &reproj_uvs[0], _point_num);
    rasterDepth<Float, Index>(
        _tript_inds, _tript_num, _points, &reproj_uvs[0], _point_num, _depth);
}
template <typename Float = float, typename Index = size_t>
void rasterDepth(const Scalar3_<Index>* _tript_inds, size_t _tript_num,
    const Point3_<Float>* _points, size_t _point_num,
    const CameraP& _camera, const Pose& _pose, Image_<Float>& _depth)
{
    std::vector<Point3_<Float>> trans_pts(_point_num);
    _pose.transform(_points, &trans_pts[0], _point_num);
    rasterDepth<Float, Index>(
        _tript_inds, _tript_num, &trans_pts[0], _point_num, _camera, _depth);
}
template <typename Float = float, typename Index = size_t>
void rasterDepth(const std::vector<Scalar3_<Index>>& _tript_inds,
    const std::vector<Point3_<Float>>& _points, const CameraP& _camera,
    Image_<Float>& _depth)
{
    if (_points.empty() || _tript_inds.empty()) return;
    if (_camera.width <= 0 || _camera.height <= 0) return;
    rasterDepth<Float, Index>(&_tript_inds[0], _tript_inds.size(), &_points[0],
        _points.size(), _camera, _depth);
}
template <typename Float = float, typename Index = size_t>
void rasterDepth(const std::vector<Scalar3_<Index>>& tript_inds,
    const std::vector<Point3_<Float>>& points, const CameraP& camera,
    const Pose& pose, Image_<Float>& depth)
{
    rasterDepth<Float, Index>(&tript_inds[0], tript_inds.size(), &points[0],
        points.size(), camera, pose, depth);
}
}  // namespace rulermvs
#endif