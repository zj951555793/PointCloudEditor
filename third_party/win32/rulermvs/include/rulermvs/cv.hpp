#ifndef _RULERMVS_CORE_CV_HPP_
#define _RULERMVS_CORE_CV_HPP_
#include "rulermvs/pose.hpp"
#include "rulermvs/image.hpp"
#include "rulermvs/camera.hpp"
#include <opencv2/opencv.hpp>
// #include <opencv2/ximgproc.hpp>
namespace rulermvs
{
struct RGBDData 
{
    cv::Mat rgb;         // 纹理图
    cv::Mat depth;       // 深度图
    cv::Mat gray;        // 灰度图
    cv::Mat gradient_x;  // 灰度图的x方向梯度
    cv::Mat gradient_y;  // 灰度图的y方向梯度
    cv::Mat vmap;        // 点云
    cv::Mat nmap;        // 法线
    cv::Mat mask;        // 标记无效值
};

template <> struct Converter_<Point3_<double>, cv::Mat> {
    static inline void to(const Point3_<double>& pt, cv::Mat& O)
    {
        O = (cv::Mat_<double>(3, 1) << pt.x, pt.y, pt.z);
    }
};
template <> struct Converter_<Rotation_<double>, cv::Mat> {
    static inline void to(const Rotation_<double>& R, cv::Mat& mat)
    {
        mat = (cv::Mat_<double>(3, 3) << R.a1, R.a2, R.a3, R.b1, R.b2, R.b3,
            R.c1, R.c2, R.c3);
    }
};
template <> struct Converter_<Pose_<double>, cv::Mat> {
    static inline void to(const Pose_<double>& RT, cv::Mat& rtmat)
    {
        rtmat = cv::Mat::eye(4, 4, CV_64F);
        RT.toMatrix(rtmat.ptr<double>(0));
    }
    static inline void to(const Pose_<double>& RT, cv::Mat& R, cv::Mat& T)
    {
        Converter_<Rotation_<double>, cv::Mat>::to(RT.getRotation(), R);
        Converter_<Point3_<double>, cv::Mat>::to(-RT.rotate(RT.center()), T);
    }
};

template <> struct Converter_<cv::Mat, Pose_<double>> {
    static inline void to(const cv::Mat& rtmat, Pose_<double>& RT)
    {
        assert(rtmat.cols == 4 && (rtmat.rows == 3 || rtmat.rows == 4) &&
               rtmat.type() == CV_64F);
        RT = Pose_<double>(rtmat.ptr<double>(0));
    }
};

template <> struct Converter_<CameraPB, cv::Mat> {
    static inline void to(const CameraPB& cam, cv::Mat& K, cv::Mat& D)
    {
        K = (cv::Mat_<double>(3, 3) << cam.fx, 0, cam.cx, 0, cam.fy, cam.cy, 0,
            0, 1);
        D = (cv::Mat_<double>(1, 5) << cam.k1, cam.k2, cam.p1, cam.p2, cam.k3);
    }
};

template <> struct Converter_<CameraSkewPB, cv::Mat> {
    static inline void to(const CameraSkewPB& cam, cv::Mat& K, cv::Mat& D)
    {
        K = (cv::Mat_<double>(3, 3) << cam.fx, cam.sk, cam.cx, 0, cam.fy,
            cam.cy, 0, 0, 1);
        D = (cv::Mat_<double>(1, 5) << cam.k1, cam.k2, cam.p1, cam.p2, cam.k3);
    }
};

template <> struct Converter_<GRAYImage, cv::Mat> {
    static inline void to(const GRAYImage& src, cv::Mat& dst, bool copy = false)
    {
        dst = cv::Mat(src.height, src.width, CV_8U, src.data, copy);
    }
};
template <> struct Converter_<RGBImage, cv::Mat> {
    static inline void to(const RGBImage& src, cv::Mat& dst, bool copy = false)
    {
        dst = cv::Mat(src.height, src.width, CV_8UC3, src.data, copy);
    }
};
template <> struct Converter_<Imagef, cv::Mat> {
    static inline void to(const Imagef& src, cv::Mat& dst, bool copy = false)
    {
        dst = cv::Mat(src.height, src.width, CV_32F, src.data, copy);
    }
};
template <> struct Converter_<Imagei, cv::Mat> {
    static inline void to(const Imagei& src, cv::Mat& dst, bool copy = false)
    {
        dst = cv::Mat(src.height, src.width, CV_32S, src.data, copy);
    }
};
template <> struct Converter_<Image16u, cv::Mat> {
    static inline void to(const Image16u& src, cv::Mat& dst, bool copy = false)
    {
        dst = cv::Mat(src.height, src.width, CV_16UC1, src.data, copy);
    }
};
template <> struct Converter_<Image_<short>, cv::Mat> {
    static inline void to(const Image_<short>& src, cv::Mat& dst, bool copy = false)
    {
        dst = cv::Mat(src.height, src.width, CV_16SC1, src.data, copy);
    }
};
template <> struct Converter_<cv::Mat, GRAYImage> {
    static inline void to(const cv::Mat& src, GRAYImage& dst)
    {
        dst = Image_<uchar>(src.cols, src.rows, src.data);
    }
};
template <> struct Converter_<cv::Mat, RGBImage> {
    static inline void to(const cv::Mat& src, RGBImage& dst)
    {
        dst = Image_<RGBPixel>(src.cols, src.rows, (RGBPixel*)src.data);
    }
};
template <> struct Converter_<cv::Mat, Imagef> {
    static inline void to(const cv::Mat& src, Imagef& dst)
    {
        dst = Image_<float>(src.cols, src.rows, (float*)src.data);
    }
};
template <> struct Converter_<cv::Mat, Imagei> {
    static inline void to(const cv::Mat& src, Imagei& dst)
    {
        dst = Image_<int>(src.cols, src.rows, (int*)src.data);
    }
};
template <> struct Converter_<cv::Mat, Image16u> {
    static inline void to(const cv::Mat& src, Image16u& dst)
    {
        dst = Image_<ushort>(src.cols, src.rows, (ushort*)src.data);
    }
};
template <> struct Converter_<cv::Mat, Image_<short>> {
    static inline void to(const cv::Mat& src, Image_<short>& dst)
    {
        dst = Image_<short>(src.cols, src.rows, (short*)src.data);
    }
};

}  // namespace rulermvs
#endif  //_RULERMVS_CORE_CV_HPP_
