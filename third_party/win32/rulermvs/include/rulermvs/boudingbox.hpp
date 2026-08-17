#ifndef _RULERMVS_CORE_BOUNDING_BOX_HPP_
#define _RULERMVS_CORE_BOUNDING_BOX_HPP_
#include "rulermvs/point.hpp"
namespace rulermvs
{
/// @brief 包围盒返回
/// @tparam T 数据类型
/// @tparam N 空间维度
template <typename T, int N = 3> struct BoundingBox_ {
    BoundingBox_() {}
    BoundingBox_(const BoundingBox_& _box)
        : anchor(_box.anchor), boxsize(_box.boxsize)
    {}
    ~BoundingBox_() {}
    Scalar_<T, N> bl() const { return anchor; }
    Scalar_<T, N> tr() const { return anchor + boxsize; }
    Scalar_<T, N> size() const { return boxsize; }
    T             capacity() const { return boxsize.accumulate(); };

    Scalar_<T, N> anchor;
    Scalar_<T, N> boxsize;
};
using BoundingBox = BoundingBox_<float, 3>;
template <typename T, int N> struct CoordTraits<BoundingBox_<T, N>> {
    typedef T type;
};

template <typename T> static inline BoundingBox_<T, 3> findBoundingBox(
    const Point3_<T>* pts, size_t sz)
{
    BoundingBox_<T, 3> bbox;
    if (sz) {
        Point3_<T> bl(pts[0]), tr(pts[0]);
        for (auto iter = &pts[1]; iter != &pts[sz]; ++iter) {
            if (iter->x < bl.x) bl.x = iter->x;
            if (iter->y < bl.y) bl.y = iter->y;
            if (iter->z < bl.z) bl.z = iter->z;
            if (iter->x > tr.x) tr.x = iter->x;
            if (iter->y > tr.y) tr.y = iter->y;
            if (iter->z > tr.z) tr.z = iter->z;
        }
        tr -= bl;
        bbox.anchor  = *((Scalar_<T, 3>*)&bl.x);
        bbox.boxsize = *((Scalar_<T, 3>*)&tr.x);
    }
    return bbox;
}

template <typename T>
static inline BoundingBox_<T, 3> findBoundingBox(const Point3Vec<T>& pts)
{
    BoundingBox_<T, 3> bbox;
    if (!pts.empty()) {
        Point3_<T> bl, tr;
        bl.x = bl.y = bl.z = std::numeric_limits<float>::max();
        tr.x = tr.y = tr.z = std::numeric_limits<float>::min();
        for (auto iter = pts.begin(); iter != pts.end(); ++iter) {
            if (iter->x < bl.x) bl.x = iter->x;
            if (iter->y < bl.y) bl.y = iter->y;
            if (iter->z < bl.z) bl.z = iter->z;
            if (iter->x > tr.x) tr.x = iter->x;
            if (iter->y > tr.y) tr.y = iter->y;
            if (iter->z > tr.z) tr.z = iter->z;
        }
        tr -= bl;
        bbox.anchor  = *((Scalar_<T, 3>*)&bl.x);
        bbox.boxsize = *((Scalar_<T, 3>*)&tr.x);
    }
    return bbox;
}
}  // namespace rulermvs
#endif