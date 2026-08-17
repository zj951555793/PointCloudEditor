#ifndef _RULERMVS_CORE_INTERSECT_HPP_
#define _RULERMVS_CORE_INTERSECT_HPP_
#include "rulermvs/point.hpp"
namespace rulermvs
{
template <typename T> static inline bool isIntersect(const Point2_<T>& ptO,
    const Point2_<T>& rayO, const Point2_<T>& ptA, const Point2_<T>& ptB)
{
    return rayO.dot(ptB - ptO) * rayO.dot(ptA - ptO) <= 0;
}
template <typename T> static inline T calcIntersectLambda(const Point2_<T>& ptO,
    const Point2_<T>& rayO, const Point2_<T>& ptA, const Point2_<T>& ptB)
{
    auto OA = ptA - ptO;
    auto AB = ptB - ptA;
    return AB.cross(OA) / AB.cross(rayO);
}
template <typename T>
static inline Point2_<T> calcIntersectPoint(const Point2_<T>& ptO,
    const Point2_<T>& rayO, const Point2_<T>& ptA, const Point2_<T>& ptB)
{
    return ptO + calcIntersectLambda(ptO, rayO, ptA, ptB) * rayO;
}
template <typename T> static inline Point2_<T> findNearestIntersectPoint(
    const std::vector<Point2_<T>>& contours, const Point2_<T>& center,
    const Point2_<T>& ray)
{
    Point2_<T> find_pt   = center;
    T          ray_norm2 = ray.norm2();
    T          min_dist  = std::numeric_limits<T>::max();
    for (size_t i = 0; i < contours.size() - 1; ++i) {
        auto& A = contours[i];
        auto& B = contours[i + 1];
        if (isIntersect(center, ray, A, B)) {
            T lambda = calcIntersectLambda(center, ray, A, B);
            if (lambda > 0) {
                T distance = lambda * ray_norm2;
                if (distance < min_dist) {
                    min_dist = distance;
                    find_pt  = lambda * ray + center;
                }
            }
        }
    }
    return find_pt;
}
}  // namespace rulermvs
#endif