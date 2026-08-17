#ifndef _RULERMVS_CORE_CENTROID_HPP_
#define _RULERMVS_CORE_CENTROID_HPP_
#include "rulermvs/point.hpp"
namespace rulermvs
{
template <typename T>
static inline Point2_<T> centroidPoints(const Point2_<T>* points, size_t pt_num)
{
    Point2_<T> c {0, 0};
    for (size_t i = 0; i < pt_num; ++i) c += points[i];
    if (pt_num) c /= (int)pt_num;
    return c;
}
template <typename T>
static inline Point2_<T> centroidPoints(const std::vector<Point2_<T>>& points)
{
    if (points.empty()) return Point2_<T> {0, 0};
    return centroidPoints(&points[0], points.size());
}
template <typename T>
static inline Point3_<T> centroidPoints(const Point3_<T>* points, size_t pt_num)
{
    Point3_<T> c {0, 0, 0};
    for (size_t i = 0; i < pt_num; ++i) c += points[i];
    if (pt_num) c /= (int)pt_num;
    return c;
}
template <typename T>
static inline Point3_<T> centroidPoints(const std::vector<Point3_<T>>& points)
{
    if (points.empty()) return Point3_<T> {0, 0, 0};
    return centroidPoints(&points[0], points.size());
}
}  // namespace rulermvs
#endif