#ifndef _RULERMVS_CORE_VOXEL_HPP_
#define _RULERMVS_CORE_VOXEL_HPP_
#include "rulermvs/point.hpp"
#include "rulermvs/boudingbox.hpp"
namespace rulermvs
{
// 快速体素滤波, 这里只是进行最近邻筛选
template <typename Float> void voxelFilter(const Point3_<Float>* points,
    size_t pt_num, coord_traits_t<Float> voxel_size,
    std::vector<size_t>& pt_inds)
{
    pt_inds.clear(), pt_inds.reserve(pt_num);
    if (!pt_num || !points) return;
    auto           bbox   = findBoundingBox<Float>(points, pt_num);
    Point3_<Float> anchor = *(Point3_<Float>*)&bbox.anchor.s;
    auto   volume = Scalar3_<size_t>(bbox.size() / voxel_size + (Float)1);
    size_t lx     = volume[0];
    size_t ly     = volume[1];
    size_t lxy    = volume[0] * volume[1];
    using PairInd = std::pair<size_t, size_t>;
    std::vector<PairInd> voxel_inds(pt_num);
    for (size_t i = 0; i < pt_num; ++i) {
        auto pt = Point3_<size_t>((points[i] - anchor) / voxel_size);
        voxel_inds[i].first  = pt.z * lxy + pt.y * lx + pt.x;
        voxel_inds[i].second = i;
    }
    // 排序, 此种实现的目的是为了减少内存占用;
    std::sort(voxel_inds.begin(), voxel_inds.end(),
        [&](const PairInd& item1, const PairInd& item2) -> bool {
            return item1.first < item2.first;
        });
    size_t pre_cnt  = 0;
    size_t pre_vind = std::numeric_limits<size_t>::max();
    size_t pre_pind = std::numeric_limits<size_t>::max();
    double pre_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < pt_num; ++i) {
        auto& vind = voxel_inds[i];
        auto& pt   = points[vind.second];
        auto  pt_v = Point3_<Float> {
            vind.first % lx, vind.first / lx % ly, vind.first / lxy};
        auto dist = (pt - anchor - (pt_v + Point3f(0.5, 0.5, 0.5)) * voxel_size)
                        .norm2();
        if (vind.first == pre_vind) {
            if (dist < pre_dist) pre_pind = vind.second, pre_dist = dist;
            pre_cnt++;
        } else {
            if (pre_cnt > 0) pt_inds.emplace_back(pre_pind);
            pre_cnt = 1, pre_pind = vind.second;
            pre_vind = vind.first, pre_dist = dist;
        }
    }
}
template <typename Float> void voxelGridFilter(Point3_<Float>* points,
    Point3_<Float>* normals, size_t pt_num, coord_traits_t<Float> voxel_size,
    std::vector<size_t>& pt_inds)
{
    pt_inds.clear(), pt_inds.reserve(pt_num);
    if (!pt_num || !points || !normals) return;
    auto           bbox   = findBoundingBox<Float>(points, pt_num);
    Point3_<Float> anchor = *(Point3_<Float>*)&bbox.anchor.s;
    auto   volume = Scalar3_<size_t>(bbox.size() / voxel_size + (Float)1);
    size_t lx     = volume[0];
    size_t ly     = volume[1];
    size_t lxy    = volume[0] * volume[1];
    using PairInd = std::pair<size_t, size_t>;
    std::vector<PairInd> voxel_inds(pt_num);
    for (size_t i = 0; i < pt_num; ++i) {
        auto pt = Point3_<size_t>((points[i] - anchor) / voxel_size);
        voxel_inds[i].first  = pt.z * lxy + pt.y * lx + pt.x;
        voxel_inds[i].second = i;
    }
    // 排序, 此种实现的目的是为了减少内存占用;
    std::sort(voxel_inds.begin(), voxel_inds.end(),
        [&](const PairInd& item1, const PairInd& item2) -> bool {
            return item1.first < item2.first;
        });
    size_t           pre_cnt  = 0;
    size_t           pre_vind = std::numeric_limits<size_t>::max();
    size_t           pre_pind = std::numeric_limits<size_t>::max();
    double           pre_dist = std::numeric_limits<double>::max();
    Point3Vec<Float> centroid(2);
    size_t           centroid_byte = 2 * sizeof(Float[3]);
    for (size_t i = 0; i < pt_num; ++i) {
        auto& vind = voxel_inds[i];
        auto& pt   = points[vind.second];
        auto& nl   = normals[vind.second];
        auto  pt_v = Point3_<Float> {
            vind.first % lx, vind.first / lx % ly, vind.first / lxy};
        auto dist = (pt - anchor - (pt_v + Point3f(0.5, 0.5, 0.5)) * voxel_size)
                        .norm2();
        if (vind.first == pre_vind) {
            if (dist < pre_dist) { pre_pind = vind.second, pre_dist = dist; }
            pre_cnt++;
            centroid[0] += pt;
            centroid[1] += nl;
        } else {
            if (pre_cnt > 0) {
                pt_inds.emplace_back(pre_pind);
                centroid[0] /= static_cast<int>(pre_cnt);
                points[pre_pind]  = centroid[0];
                normals[pre_pind] = normalize(centroid[1]);
                memset(&centroid[0].x, 0, centroid_byte);
            }
            centroid[0] += pt;
            centroid[1] += nl;
            pre_cnt = 1, pre_pind = vind.second;
            pre_vind = vind.first, pre_dist = dist;
        }
    }
}
template <typename Float> void voxelFilter(const Point3Vec<Float>& points,
    coord_traits_t<Float> sz, std::vector<size_t>& pt_inds)
{
    if (points.empty()) return;
    voxelFilter<Float>(&points[0], points.size(), sz, pt_inds);
}
template <typename Float> Point3Vec<Float> voxelFilter(
    const Point3_<Float>* src_pts, size_t src_num, coord_traits_t<Float> sz)
{
    Point3Vec<Float> dst_pts;
    if (src_pts && src_num) {
        std::vector<size_t> pt_inds;
        voxelFilter<Float>(src_pts, src_num, sz, pt_inds);
        dst_pts.resize(pt_inds.size());
        for (size_t i = 0; i < pt_inds.size(); ++i)
            dst_pts[i] = src_pts[pt_inds[i]];
    }
    return dst_pts;
}
template <typename Float> std::vector<Point3_<Float>> voxelFilter(
    const Point3Vec<Float>& pts, coord_traits_t<Float> sz)
{
    if (!pts.empty()) return voxelFilter<Float>(&pts[0], pts.size(), sz);
    return std::vector<Point3_<Float>>();
}
}  // namespace rulermvs
#endif