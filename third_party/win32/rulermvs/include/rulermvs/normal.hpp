#ifndef _RULERMVS_CORE_NORMAL_HPP_
#define _RULERMVS_CORE_NORMAL_HPP_
#include "rulermvs/math.hpp"
#include "rulermvs/point.hpp"
#include "rulermvs/kdtree.hpp"
#include "rulermvs/centroid.hpp"
namespace rulermvs
{
template <typename T> static inline Point2_<T> normalize(const Point2_<T>& pt)
{
    T norm = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    return norm ? Point2_<T> {pt.x / norm, pt.y / norm} : Point2_<T> {0, 0};
}
template <typename T> static inline Point3_<T> normalize(const Point3_<T>& pt)
{
    T norm = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    return norm ? Point3_<T> {pt.x / norm, pt.y / norm, pt.z / norm} :
                  Point3_<T> {0, 0, 0};
}
template <typename T>
static inline std::vector<Point2_<T>> normalize(const std::vector<Point2_<T>>& normals)
{
    std::vector<Point2_<T>> normal_vec;
    if (!normals.empty()) {
        normal_vec.resize(normals.size());
        for (size_t i = 0; i < normals.size(); ++i)
            normal_vec[i] = normalize<T>(normals[i]);
    }
    return normal_vec;
}
template <typename T>
static inline Point3Vec<T> normalize(const Point3Vec<T>& normals)
{
    Point3Vec<T> normal_vec;
    if (!normals.empty()) {
        normal_vec.resize(normals.size());
        for (size_t i = 0; i < normals.size(); ++i)
            normal_vec[i] = normalize<T>(normals[i]);
    }
    return normal_vec;
}

template <typename Float> void computeNormalsInKnn(
    CloudKDTree<Float>& index, Point3_<Float>* normals, size_t k)
{
    auto                ptnum           = index.dataset.point_num;
    auto*               points          = index.dataset.points;
    double              covariance[9]   = {0}, X[3];
    const size_t        covraiance_byte = 9 * sizeof(double);
    Point3_<Float>      pt_vec;
    std::vector<Float>  out_dist(k);
    std::vector<size_t> ret_index(k);
    KNNResultSet<Float> result_set(k);
    for (size_t i = 0; i < ptnum; ++i) {
        const auto& query_pt = points[i];
        result_set.init(&ret_index[0], &out_dist[0]);
        index.findNeighbors(result_set, &query_pt.x, SearchParams(10));
        ::memset(&covariance[0], 0, covraiance_byte);
        for (size_t j = 0; j < k; ++j) {
            pt_vec = points[ret_index[j]] - query_pt;
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
        normals[i] = normalize(Point3_<Float>(X[0], X[1], X[2]));
    }
}

template <typename Float> void recomputeNormalsInKnn(
    CloudKDTree<Float>& index, Point3_<Float>* normals, size_t k)
{
    auto                ptnum           = index.dataset.point_num;
    auto*               points          = index.dataset.points;
    double              covariance[9]   = {0}, X[3];
    const size_t        covraiance_byte = 9 * sizeof(double);
    Point3_<Float>      pt_vec;
    std::vector<Float>  out_dist(k);
    std::vector<size_t> ret_index(k);
    KNNResultSet<Float> result_set(k);
    for (size_t i = 0; i < ptnum; ++i) {
        const auto& query_pt = points[i];
        result_set.init(&ret_index[0], &out_dist[0]);
        index.findNeighbors(result_set, &query_pt.x, SearchParams(10));
        ::memset(&covariance[0], 0, covraiance_byte);
        for (size_t j = 0; j < k; ++j) {
            pt_vec = points[ret_index[j]] - query_pt;
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
        Point3_<Float> nl = normalize(Point3_<Float>(X[0], X[1], X[2]));
        normals[i]        = normals[i].dot(nl) >= 0 ? nl : -nl;
    }
}

/// @brief 拟合平面
/// @param points 三维点数组
/// @param center 中心点
/// @param normal 平面法向朝向
template <typename T> void computePlane(
    const Point3Vec<T>& points, Point3_<T>& center, Point3_<T>& normal)
{
    double cov[9] = {0}, X[3];
    assert(points.size() >= 3);
    center = centroidPoints(points);
    for (size_t i = 0; i < points.size(); ++i) {
        auto pt_vec = points[i] - center;
        cov[0] += pt_vec.x * pt_vec.x;
        cov[1] += pt_vec.x * pt_vec.y;
        cov[2] += pt_vec.x * pt_vec.z;
        cov[4] += pt_vec.y * pt_vec.y;
        cov[5] += pt_vec.y * pt_vec.z;
        cov[8] += pt_vec.z * pt_vec.z;
    }
    cov[3] = cov[1];
    cov[6] = cov[2];
    cov[7] = cov[5];
    solveZ3x3(cov, X);
    normal = normalize(Point3_<T>(X[0], X[1], X[2]));
}

/// @brief 判断三维点是否在平面前方
/// @tparam T 类型
/// @param points 三维点数组
/// @param center 中心点
/// @param normal 平面法向朝向
/// @param inliers 记录是否在平面前方
template <typename T> void isInFrontOfPlane(const Point3Vec<T>& points,
    const Point3_<T>& center, const Point3_<T>& normal,
    std::vector<bool>& inliers)
{
    if (inliers.size() != points.size()) inliers.resize(points.size());
    auto pt_nl = normalize(normal);
    for (size_t i = 0; i < points.size(); ++i) {
        auto pt_v  = normalize(points[i] - center);
        inliers[i] = std::acos(pt_v.dot(pt_nl)) < MVS_PI_2;
    }
}

}  // namespace rulermvs

#endif