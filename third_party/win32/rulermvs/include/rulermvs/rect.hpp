#ifndef _RULERMVS_CORE_RECT_HPP_
#define _RULERMVS_CORE_RECT_HPP_
#include "rulermvs/size.hpp"
#include "rulermvs/point.hpp"
#include "rulermvs/scalar.hpp"
#include "rulermvs/intersect.hpp"
namespace rulermvs
{
/// @brief 矩阵结构体
/// @tparam T 数据类型
template <typename T> struct Rect_: Point2_<T>, Size_<T> {
    typedef Size_<T>   RSize;
    typedef Point2_<T> RPoint;
    Rect_() : RPoint(0, 0), RSize(0, 0) {}
    Rect_(const Rect_& rect) : RPoint(rect), RSize(rect) {}
    Rect_(RPoint pt, RSize sz) : RPoint(pt), RSize(sz) {}
    Rect_(RSize sz) : RPoint(0, 0), RSize(sz) {}
    Rect_(T w, T h) : RPoint(0, 0), RSize(w, h) {}
    Rect_(T x, T y, T w, T h) : RPoint(x, y), RSize(w, h) {}
    inline std::vector<Point2_<T>> contour(T step = 1) const
    {
        T l1 = RSize::width;
        T l2 = l1 + RSize::height;
        T l3 = l1 * 2 + RSize::height;
        T l4 = l1 * 2 + RSize::height * 2;

        std::vector<Point2_<T>> vec;
        for (T i : rangeVec<T>(0, l4, step)) {
            if (i < l1) vec.emplace_back(i, 0);
            else if (i < l2)
                vec.emplace_back(RSize::width, i - l1);
            else if (i < l3)
                vec.emplace_back(l3 - i, RSize::height);
            else
                vec.emplace_back(0, l4 - i);
        }
        return vec;
    }
};
using Rect = Rect_<float>;
template <typename T> struct CoordTraits<Rect_<T>> {
    typedef T type;
};
template <typename T>
static inline std::ostream& operator<<(std::ostream& out, const Rect_<T>& rect)
{
    return out << "[Rect: " << static_cast<Point2_<T>>(rect) << " "
               << static_cast<Size_<T>>(rect) << "]";
}
template <typename T>
Rect_<T> findMinOuterRect(const std::vector<Point2_<T>>& points)
{
    auto minx = std::numeric_limits<T>::max();
    auto miny = std::numeric_limits<T>::max();
    auto maxx = std::numeric_limits<T>::min();
    auto maxy = std::numeric_limits<T>::min();
    for (size_t i = 0; i < points.size(); ++i) {
        auto& point = points[i];
        if (point.x > maxx) maxx = point.x;
        if (point.x < minx) minx = point.x;
        if (point.y > maxy) maxy = point.y;
        if (point.y < miny) miny = point.y;
    }
    return {minx, miny, maxx - minx, maxy - miny};
}
template <typename T>
Rect_<T> findMaxInnerRect(const std::vector<Point2_<T>>& points)
{
    auto rect   = findMinOuterRect(points);
    auto pt_lb  = static_cast<Point2_<T>>(rect);
    auto pt_rb  = Point2_<T>(rect.x + rect.width, rect.y);
    auto pt_lt  = Point2_<T>(rect.x, rect.y + rect.height);
    auto pt_rt  = Point2_<T>(rect.x + rect.width, rect.y + rect.height);
    auto pt_o   = Point2_<T>(rect.x + rect.width / 2, rect.y + rect.height / 2);
    auto lt_new = findNearestIntersectPoint(points, pt_o, pt_lt - pt_o);
    auto rt_new = findNearestIntersectPoint(points, pt_o, pt_rt - pt_o);
    auto lb_new = findNearestIntersectPoint(points, pt_o, pt_lb - pt_o);
    auto rb_new = findNearestIntersectPoint(points, pt_o, pt_rb - pt_o);

    auto minx = std::max(lt_new.x, lb_new.x);
    auto miny = std::max(lb_new.y, rb_new.y);
    auto maxx = std::min(rt_new.x, rb_new.x);
    auto maxy = std::min(lt_new.y, rt_new.y);
    return {minx, miny, maxx - minx, maxy - miny};
}
template <typename T>
Rect_<T> findMaxInnerRect2(const std::vector<Point2_<T>>& points)
{
    T                max_area = 0;
    Scalar2_<size_t> record   = {0, 0};
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = i + 1; j < points.size(); ++j) {
            auto pt   = points[j] - points[i];
            T    area = std::abs(pt.x * pt.y);
            if (max_area < area) {
                max_area = area;
                record   = {i, j};
            }
        }
    }
    if (max_area <= 0) return Rect_<T>();
    auto minx = std::min(points[record[0]].x, points[record[1]].x);
    auto miny = std::min(points[record[0]].y, points[record[1]].y);
    auto maxx = std::max(points[record[0]].x, points[record[1]].x);
    auto maxy = std::max(points[record[0]].y, points[record[1]].y);
    return {minx, miny, maxx - minx, maxy - miny};
}
}  // namespace rulermvs
#endif