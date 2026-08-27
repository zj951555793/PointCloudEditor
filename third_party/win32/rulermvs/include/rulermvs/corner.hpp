#ifndef _RULERMVS_MARKEREXTRACTOR_CORNER_HPP_
#define _RULERMVS_MARKEREXTRACTOR_CORNER_HPP_
//#include "rulermvs/pose.hpp"
//#include "rulermvs/rgbd.hpp"
#include "rulermvs/eigen.hpp"
#include "rulermvs/cv.hpp"
namespace rulermvs
{
// 椭圆拟合结果结构体
struct EllipseResult {
    cv::Point2f center;  // 中心点
    cv::Size2f axes;     // 长短轴 (width >= height)
    float angle;        // 旋转角度（度，范围[0, 180)）
    double error;        // 拟合误差
    bool valid;          // 是否有效

    double a, b, c, d, e, f;  // 椭圆一般方程参数
    Eigen::MatrixXd infos;

    EllipseResult() : center(0, 0), axes(0, 0), angle(0), error(0), valid(false)
    {}
    EllipseResult(cv::Point2f center_, cv::Size2f axes_, float ang,
        double err = 0,
        const std::vector<cv::Point2f>& points = std::vector<cv::Point2f>())
        : center(center_), axes(axes_), angle(ang), error(err), valid(true)
    {
        const float u0 = center_.x;
        const float v0 = center_.y;
        const float angle_deg = ang;
        const float w2 = axes_.width * 0.5;
        const float h2 = axes_.height * 0.5;

        // 角度转换为弧度
        const float theta = angle_deg * MVS_PI / 180.0;

        // 三角函数值
        const float sin_theta = std::sin(theta);
        const float cos_theta = std::cos(theta);

        // 计算二次项系数
        a = cos_theta * cos_theta / (w2 * w2) +
            sin_theta * sin_theta / (h2 * h2);
        b = 2.0 * sin_theta * cos_theta * (1.0 / (w2 * w2) - 1.0 / (h2 * h2));
        c = sin_theta * sin_theta / (w2 * w2) +
            cos_theta * cos_theta / (h2 * h2);

        // 计算一次项系数
        d = -2.0 * a * u0 - b * v0;
        e = -b * u0 - 2.0 * c * v0;

        // 计算常数项
        f = a * u0 * u0 + b * u0 * v0 + c * v0 * v0 - 1.0;

        if (!points.empty()) {
            double total_error = 0.0;
            Eigen::MatrixXd A(points.size(), 6);
            int count = 0;
            for (const auto& p : points) {
                const float x = p.x, y = p.y;
                const double error =
                    a * x * x + b * x * y + c * y * y + d * x + e * y + f;
                total_error += error * error;
                A(count, 0) = x * x;
                A(count, 1) = x * y;
                A(count, 2) = y * y;
                A(count, 3) = x;
                A(count, 4) = y;
                A(count++, 5) = 1.0;
            }
            error = sqrt(total_error / points.size());
            infos = A.transpose() * A;
        }
    }

    // 计算椭圆面积
    double getArea() const
    {
        return axes.width * axes.height;
    }
};

struct Corner 
{
    Corner() {}

    Corner(double x, double y)
    {
        x_ = x;
        y_ = y;

        // char char_area[100];
        // char char_ratio[100];
        // char char_roundnes[100];
        // char char_fiterror[100];
        // char char_convexity[100];
        // char char_eccentricity[100];
        sprintf_s(char_area, sizeof(char_area), "%d", 0);
        sprintf_s(char_ratio, sizeof(char_ratio), "%d", 0);
        sprintf_s(char_roundnes, sizeof(char_roundnes), "%d", 0);
        sprintf_s(char_fiterror, sizeof(char_fiterror), "%d", 0);
        sprintf_s(char_convexity, sizeof(char_convexity), "%d", 0);
        sprintf_s(char_eccentricity, sizeof(char_eccentricity), "%d", 0);
    }

    int id_ = 0;

    double x_;
    double y_;
    EllipseResult ellipse_;
    double radius_;

    Vec3d normal_c_ = Vec3d(0.0, 0.0, 0.0);

    Vec3d point3d_c_ = Vec3d(0.0, 0.0, 0.0);

    std::vector<cv::Point2f> contour_uv;
    std::vector<cv::Point3f> contour_3d;
    std::vector<cv::Point3f> contour_normal;

    int match_mappoint_id_ = -1;

    bool is_new_match = false;  // if the corner is new expand or not

    bool has_depth = false;

    bool is_onplan = false;  //是否是平面上的点

    bool is_outlier = false;

    double err_ = 0.0;

    double pixel_error_ = 0.0;

    double area_ = 0.0;

    double ratio = 0.0;

    double fit_error_ = 0.0;

    double eccentricity_ = 0.0;

    double convexity_ = 0.0;

    double roundnes_ = 0.0;

    char char_area[100];
    char char_ratio[100];
    char char_roundnes[100];
    char char_fiterror[100];
    char char_convexity[100];
    char char_eccentricity[100];
    // bool has_matched_=false;
};

typedef std::pair<rulermvs::Corner, rulermvs::Corner> CornerPair;
typedef std::vector<CornerPair> CornerPairList;

// int detectCircleMarker(const Image8u& gray, const Imagef& depth,
//     const CameraP& cam, Point3fVec& pts, Point3fVec& nls)
// {
//     return 0;
// }
}  // namespace rulermvs

#endif