#ifndef _SLAM_TRACKER_CAMERATRACKER_HPP_
#define _SLAM_TRACKER_CAMERATRACKER_HPP_
#include "rulermvs/eigen.hpp"
#include "rulermvs/cv.hpp"

namespace slam
{
class MVS_EXPORT Camera
{
public:
    Camera() {}

    ~Camera() {}

    cv::Point3f Project2dto3d(double u, double v, double d);

    rulermvs::Vec2d Project3dto2d(rulermvs::Vec3d& point3d, const rulermvs::Mat44& pose_w2c);

    rulermvs::Vec2d Project3dto2d(rulermvs::Vec3d& point3d);

    double fx_;
    double fy_;
    double cx_;
    double cy_;

    cv::Mat K_;
    cv::Mat D_;

    int width_;
    int height_;

    cv::Mat pose_R_ = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat pose_t_ = cv::Mat::zeros(3, 1, CV_64F);
};
}  // namespace slam
#endif 