#ifndef _RULERMVS_CORE_KDTREE_HPP_
#define _RULERMVS_CORE_KDTREE_HPP_
#include "rulermvs/math.hpp"
#include "rulermvs/point.hpp"
#include "rulermvs/nanoflann.hpp"
namespace rulermvs
{
// 前置声明
template <typename Float> struct IPointCloud_;

/// @brief 一维数组KDTree
/// @tparam Tp 数据类型
template <typename Tp> struct ArrAdaptor_ {
    const Tp*      data;  //!< A const ref to the data set origin
    const uint32_t num;   //!< array size
    /// The constructor that sets the data set source
    ArrAdaptor_(const Tp* data_ptr, size_t sz) : data(data_ptr), num(sz) {}
    inline uint32_t kdtree_get_point_count() const { return num; }
    inline Tp       kdtree_get_pt(const uint32_t idx, uint32_t) const
    {
        return data[idx];
    }
    template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
};
template <typename Tp> using ArrKDTree =
    KDTreeSingleIndexAdaptor<L1_Adaptor<Tp, ArrAdaptor_<Tp>, Tp, uint32_t>,
        ArrAdaptor_<Tp>, 1, uint32_t>;

/// @brief 三维点数组对应的KDTree
/// @tparam Float 浮点类型
template <typename Float> struct CloudAdaptor_ {
    const Point3_<Float>* points;     //!< A const ref to the data set origin
    const Point3_<Float>* normals;    //!< A const ref to the data set origin
    const size_t          point_num;  //!< point size
    CloudAdaptor_() : points(nullptr), normals(nullptr), point_num(0) {}
    /// @brief 点云KDTree的初始化
    /// @tparam Float 浮点类型
    /// @param cloud 点云接口类引用
    CloudAdaptor_(const IPointCloud_<Float>& cloud)
        : points(cloud.getPointData())
        , normals(cloud.getNormalData())
        , point_num(cloud.getPointNum())
    {}
    /// The constructor that sets the data set source
    CloudAdaptor_(const Point3_<Float>* point_ptr, size_t point_num)
        : points(point_ptr), normals(nullptr), point_num(point_num)
    {}
    CloudAdaptor_(const Point3_<Float>* point_ptr,
        const Point3_<Float>* normal_ptr, size_t point_num)
        : points(point_ptr), normals(normal_ptr), point_num(point_num)
    {}
    inline size_t kdtree_get_point_count() const { return point_num; }
    inline Float  kdtree_get_pt(const size_t idx, size_t dim) const
    {
        if (dim == 0) return points[idx].x;
        else if (dim == 1)
            return points[idx].y;
        else
            return points[idx].z;
    }
    template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
};
template <typename Float> using CloudKDTree = KDTreeSingleIndexAdaptor<
    L2_Simple_Adaptor<Float, CloudAdaptor_<Float>, Float, size_t>,
    CloudAdaptor_<Float>, 3, size_t>;

}  // namespace rulermvs
#endif