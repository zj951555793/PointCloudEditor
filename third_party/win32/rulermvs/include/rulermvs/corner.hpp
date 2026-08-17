#ifndef _RULERMVS_MARKEREXTRACTOR_CORNER_HPP_
#define _RULERMVS_MARKEREXTRACTOR_CORNER_HPP_
//#include "rulermvs/pose.hpp"
//#include "rulermvs/rgbd.hpp"
#include "rulermvs/eigen.hpp"
#include "rulermvs/cv.hpp"
namespace rulermvs
{
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

    Vec3d normal_c_;

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