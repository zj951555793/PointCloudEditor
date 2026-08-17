#ifndef _RULERMVS_MARKEREXTRACTOR_MARKEREXTRACTOR_HPP_
#define _RULERMVS_MARKEREXTRACTOR_MARKEREXTRACTOR_HPP_
#include "rulermvs/rgbd.hpp"
#include "rulermvs/image.hpp"
#include "rulermvs/point.hpp"
#include "rulermvs/cv.hpp"
#include "rulermvs/corner.hpp"

namespace rulermvs
{
struct CicrleConfigs {
    bool   filterByArea = true;
    double minArea = 150 /*50*/ /*20*/ /*150*/ /*50*/;  // 轮廓最小面积
    double maxArea = 4000;                              // 轮廓最大面积

    bool   filterByCircularity = true;
    double minCircularity      = 0.8;  // 最小圆度限制
    double maxCircularity      = 1.2;  // 最大圆度限制

    bool   filterByConvexity    = true;
    double maxConvexity         = 1.0;          // 最大凸度限制
    double minConvexity         = 0.9 /*0.5*/;  // 最小凸度限制
    bool   filterByEccentricity = false;
    double maxEccentricity      = 1.0 /*0.9*/ /*0.7*/;  // 最大离心率限制

    double maxFitRMSE = 1.0;  // 1.50/*0.5*//*1.0*/;//轮廓椭圆拟合RMSE限制

    double maxContourError = 1.50 /*1.5*/;  // 轮廓椭圆拟合局部最大偏差限制

    std::string output_dir;

    double r1r2_ratio_    = 1.75; // 圆环外圈、内圈比值
    //double r1_            = 3.5;  // 圆环外圈半径
    //double r2_            = 2;    // 圆环内圈半径
    int lowThreshold_  = 30;   // 120.0;//200
    int highThreshold_ = 50;
    double ratio_         = 2.5;  // 3.0; 2.5
    int    kernel_size_   = 7;    // 高斯核大小
    double alpha_ = 0.6;  // 高斯模糊处理函数中的高斯核的标准差
    int    min_contour_size = 30;     // 圆环内圈轮廓最小周长
    bool   dnn_state        = false;  // 默认不包含DNN模块
    cv::dnn::Net net_;                // DNN神经网络
    int          scale_ = 2;//图片缩放系数，可取1、2、3、4
    //double       radius1_ = 1;//标志点1的内圈半径
    //double       radius2_ = 2.5;//标志点2的内圈半径
    double       LRmaxdist_ = 1.5;//左右目同名点极线校正欧氏距离阈值
    std::vector<double> radiusVec {1, 1.5, 2};//根据实际情况在demo中修改、增减
    bool has_distorted = false;//默认输入图片未经过畸变矫正
    bool has_rectified = false;//默认输入图片未经过极线校正
    bool output_rectifiedcoordinate = false;//默认输出原始左目坐标系下三维点
    bool multi_thread_ = true;//默认提点开启多线程(竖块分割)
    bool sparse = true;//默认标志点是稀疏分布(暂时废弃)
    double min_depth_ = 100;//最小深度
    double max_depth_ = 500;//最大深度

    bool long_distance_ = false;//默认是近距离扫描，远距离扫描提点精度变差，设定的误差阈值随之改变
    double tolerance_ = 0.15;//半径的RMSE的阈值，近距离下建议设置为0.12，同时是左右目轮廓面积的比值差阈值
    int tolerance_num_ = 5;//容错性翻倍倍数，远距离状态下(inputscaled_为true时会被屏蔽)
    bool Shining_ = false;//是否为反光标志点(inputscaled_为true时会被屏蔽)
    bool inputscaled_ = true;//输入灰度图是否为缩放后的，若为true则三维前交时关闭部分约束
    double distance_ratio = 0.15;  //可调整，左右目同名点对之间的二维距离比值差异阈值
    double max_angle = MVS_PI_8;
};

class MVS_EXPORT CircleMarkerExtractor {
public:
    CircleMarkerExtractor() {}
    ~CircleMarkerExtractor() {}

    // 提取圆形点
    bool CircleExtractSimple(/*const*/ cv::Mat& img,
        std::vector<Corner>& corners, int idx,
        const CicrleConfigs& configs = CicrleConfigs());

    bool CircleExtractSimpleForHand(/*const*/ cv::Mat& img, std::vector<Corner>& corners, int idx, const CicrleConfigs& configs = CicrleConfigs());

    void DetectFeaturePointsAndInterpolate_binocular(
        std::vector<Corner>& corners, cv::Mat& depth, const cv::Mat& K);
    void CalcFoundationMat(cv::Mat& R, cv::Mat& T, cv::Mat& camMatL,
        cv::Mat& camMatR, cv::Mat& FoundationMat);
    bool computeStereoMatches(const std::vector<Corner>& corners_left,
        const std::vector<Corner>&                       corners_right,
        std::map<Corner, Corner>&                        markMathcedMap);
    bool extractMarkPointsFast(const cv::Mat& srcImg,
        std::vector<Corner>& corners, int idx, const CicrleConfigs& configs);
    void UndistortCorners(
        std::vector<Corner>& corners, const cv::Mat& K, const cv::Mat& D);
    void StereoRectifyCorners(
        std::vector<Corner>& corners, std::vector<Corner>& rectified_corners,
        const cv::Mat& K, const cv::Mat& R, const cv::Mat& P);
    // 修正偏心误差（Ahn方法）
    void correctedEccentricityAhn(cv::Mat& img, cv::Mat& XYZ,
        cv::RotatedRect& box, double& epsilon_u, double& epsilon_v,
        double& dis);

    void transform_rulermvsData2cvMat(const std::vector<Pose>& camPoseVec, const std::vector<CameraPB>& cameraPBVec, int width, int height, int rectify_flags = cv::CALIB_ZERO_DISPARITY);
    std::string debug_dir;

    std::string current_debug_name;

    cv::Mat R_, T_, _F, K_left, K_right, D_left, D_right, P1_, P2_, R1_, R2_, Q_;
    std::vector<cv::RotatedRect> ellipses_;  // 圆形标志点最内圈轮廓对应的椭圆参数

    bool SetStereorectifyParam(const std::vector<cv::Mat> matrixes) 
    {
        if (matrixes.size() <= 3) {
            MVS_WLOG << "Warning: input matrixes num error!";
            return false;
        }

        //默认输入矩阵的顺序是P1、P2、R1、R2(可选)，至少3个
        if (matrixes[0].size() == cv::Size(4, 3))
        {
            this->P1_ = matrixes[0].clone();
        } else 
        {
            MVS_WLOG << "Warning: the first matrix size error!";
            return false;
        }
        if (matrixes[1].size() == cv::Size(4, 3))
        {
            this->P2_ = matrixes[1].clone();
        } else 
        {
            MVS_WLOG << "Warning: the second matrix size error!";
            return false;
        }
        if (matrixes[2].size() == cv::Size(3, 3))
        {
            this->R1_ = matrixes[2].clone();
        } else 
        {
            MVS_WLOG << "Warning: the third matrix size error!";
            return false;
        }
        if (matrixes.size() > 3)
        {
            this->R2_ = matrixes[3].clone();
        }
        return true;
    }
};

/// @brief 单帧图片提取圆形标志点
/// @param img 灰度图或3通道彩色图
/// @param corners 畸变矫正后的标志点中心corner数组
/// @param index 相机编号，index=0为左目，否则为右目
MVS_EXPORT bool ExtractCircleMarker(cv::Mat& img,
    CircleMarkerExtractor& corner_extractor, std::vector<Corner>& corners,
    int index, const CicrleConfigs& configs = CicrleConfigs());

/// @brief 根据双目提取的标志点二维中心corners，得到标志点三维坐标，并返回左右目标志点匹配索引
/// @param corners_left、corners_right 左目、右目标志点二维中心corners
/// @param K_left、K_right 双目相机内参矩阵
/// @param Fundamental 双目相机基础矩阵
MVS_EXPORT std::vector<std::pair<int, int>> getStereoMarker3dPoints(
    std::vector<Corner>& corners_left, std::vector<Corner>& corners_right,
    const cv::Mat& RT, const cv::Mat& K_left, const cv::Mat& K_right,
    const cv::Mat& Fundamental, Point3dVec& points);

MVS_EXPORT std::vector<std::pair<int, int>> getStereoRectifiedMarker3dPoints(
    std::vector<Corner>& corners_left, std::vector<Corner>& corners_right, const std::vector<double> radiusVec_temp,
    Point3dVec& points, cv::Mat& R1, cv::Mat& P1, cv::Mat& P2, double maxdist, bool sparse = true, double min_depth = 100.0, 
    double max_depth = 500.0, bool long_distance = false, bool Shining = false, double tolerance = 0.15, int tolerance_num = 5, bool scaled = false, double distance_ratio = 0.15, double max_angle = MVS_PI_8);

static inline std::vector<std::pair<Corner, Corner>> getStereoMarker3dPoints(cv::Mat& img_L, cv::Mat& img_R,
    CircleMarkerExtractor& corner_extractor, Point3dVec& points,
    std::vector<cv::RotatedRect>& ellipses_L,
    std::vector<cv::RotatedRect>& ellipses_R,
    const CicrleConfigs&          configs = CicrleConfigs())
{
    std::vector<std::pair<Corner, Corner>> corner_matchs;
    if (img_L.empty() || img_R.empty() || corner_extractor.K_left.empty() ||
        corner_extractor.K_right.empty() || corner_extractor._F.empty())
        return corner_matchs;
    std::vector<Corner> corners_left, corners_right;
    //lImg、rImg为rulermvs数据
    //cv::Mat cvimg_L(lImg.getHeight(), lImg.getWidth(), CV_8UC1, (void*)lImg.getData(), lImg.getStride());
    //cv::Mat cvimg_R(rImg.getHeight(), rImg.getWidth(), CV_8UC1, (void*)rImg.getData(), rImg.getStride());
    //cv::cvtColor(cvimg_L, cvimg_L, cv::COLOR_BGR2GRAY);
    //cv::cvtColor(cvimg_R, cvimg_R, cv::COLOR_BGR2GRAY);
    bool state1 =
        ExtractCircleMarker(img_L, corner_extractor, corners_left, 0, configs);
    ellipses_L = corner_extractor.ellipses_;
    bool state2 =
        ExtractCircleMarker(img_R, corner_extractor, corners_right, 1, configs);
    ellipses_R = corner_extractor.ellipses_;
    if (state1 && state2) {
        std::vector<Corner> rectified_corners_left, rectified_corners_right;
        if (!configs.has_rectified)
        {        
            corner_extractor.StereoRectifyCorners(corners_left, rectified_corners_left, corner_extractor.K_left, corner_extractor.R1_, corner_extractor.P1_);
            corner_extractor.StereoRectifyCorners(corners_right, rectified_corners_right, corner_extractor.K_right, corner_extractor.R2_, corner_extractor.P2_);
        } else 
        {
            rectified_corners_left = corners_left,
            rectified_corners_right = corners_right;
        }
        //cv::Mat gray_L1 = cv::Mat::zeros(img_L.rows, img_L.cols/* * 3*/, CV_8UC3);
        //for (int i = 0; i < rectified_corners_left.size(); ++i) 
        //{
        //    for (int j = 0; j < rectified_corners_left[i].contour_uv.size(); ++j) 
        //    {
        //        circle(gray_L1, cv::Point2d(rectified_corners_left[i].contour_uv[j].x/* + img_L.cols*/, rectified_corners_left[i].contour_uv[j].y),
        //            0.1, cv::Scalar(0, 255, 0), -1, 8);
        //    }
        //}
        //for (int i = 0; i < rectified_corners_right.size(); ++i) {
        //    for (int j = 0; j < rectified_corners_right[i].contour_uv.size(); ++j) {
        //        circle(gray_L1, cv::Point2d(rectified_corners_right[i].contour_uv[j].x/* + img_L.cols*/, rectified_corners_right[i].contour_uv[j].y),
        //            0.1, cv::Scalar(0, 0, 255), -1, 8);
        //    }
        //}
        std::vector<std::pair<int, int>> index_matchs1;
        if (!configs.output_rectifiedcoordinate)
        {
            index_matchs1 = getStereoRectifiedMarker3dPoints(rectified_corners_left, rectified_corners_right, configs.radiusVec,
                points, corner_extractor.R1_, corner_extractor.P1_, corner_extractor.P2_, configs.LRmaxdist_, configs.sparse, configs.min_depth_, configs.max_depth_, configs.long_distance_, configs.Shining_, configs.tolerance_, configs.tolerance_num_, configs.inputscaled_, configs.distance_ratio, configs.max_angle);
        } else
        {
            cv::Mat R_temp = cv::Mat::eye(3, 3, CV_64F);
            index_matchs1 = getStereoRectifiedMarker3dPoints(rectified_corners_left, rectified_corners_right, configs.radiusVec,
                points, R_temp, corner_extractor.P1_, corner_extractor.P2_, configs.LRmaxdist_, configs.sparse, configs.min_depth_, configs.max_depth_, configs.long_distance_, configs.Shining_, configs.tolerance_, configs.tolerance_num_, configs.inputscaled_, configs.distance_ratio, configs.max_angle);
        }
        //std::srand(std::time(0));
        //for (int i = 0; i < index_matchs1.size(); i++) 
        //{
        //    std::pair<int, int> index_LR = index_matchs1[i];
        //    cv::Scalar color(rand() % 256, rand() % 256, rand() % 256);
        //    cv::line(gray_L1,
        //        cv::Point((int)rectified_corners_left[index_LR.first].x_, (int)rectified_corners_left[index_LR.first].y_),
        //        cv::Point((int)rectified_corners_right[index_LR.second].x_, (int)rectified_corners_right[index_LR.second].y_), color, 1);
        //}
        for (int i = 0; i < index_matchs1.size(); i++) 
        {
            // 左相机坐标系下三维坐标
            Vec3d point_temp = Vec3d(points[i].x, points[i].y, points[i].z);
            corners_left[index_matchs1[i].first].has_depth   = true;
            corners_right[index_matchs1[i].second].has_depth = true;
            corners_left[index_matchs1[i].first].point3d_c_  = point_temp;
            corners_right[index_matchs1[i].second].point3d_c_ = point_temp;

            corner_matchs.emplace_back(
                std::make_pair(corners_left[index_matchs1[i].first],
                    corners_right[index_matchs1[i].second]));
            corner_matchs.back().first.id_ = i;
            corner_matchs.back().second.id_ = i;
        }

        //cv::Mat T_temp = cv::Mat(3, 4, CV_64F);
        //corner_extractor.R_.copyTo(T_temp.rowRange(0, 3).colRange(0, 3));
        //corner_extractor.T_.copyTo(T_temp.rowRange(0, 3).col(3));
        //auto index_matchs2 = getStereoMarker3dPoints(corners_left, corners_right,
        //    T_temp, corner_extractor.K_left, corner_extractor.K_right,
        //    corner_extractor._F, points);
        if (points.empty()) {
            MVS_WLOG << "StereoMarkerMatcher: Please check the left eye camera "
                        "or right eye camera, failed to match Marker Points.";
        }
        return corner_matchs;
    } else {
        MVS_WLOG << "StereoMarkerMatcher: Please check the left eye camera or "
                    "right eye camera, there are not any Marker Points in one "
                    "of the frames.";
        return corner_matchs;
    }
}

static inline void getEllipsePixel(const cv::Size img_sz,
    const std::vector<cv::RotatedRect>&           ellipses,
    std::vector<cv::Point>& nonZeroLocations, double scale_e = 1.0)
{
    cv::Mat image(img_sz, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < ellipses.size(); i++) {
        cv::RotatedRect cur_ellipse = ellipses[i];
        cur_ellipse.size.height     = cur_ellipse.size.height * scale_e;
        cur_ellipse.size.width      = cur_ellipse.size.width * scale_e;
        cv::ellipse(image, cur_ellipse, cv::Scalar(255), -1);
    }
    cv::findNonZero(image, nonZeroLocations);
}

/// @brief 读取DNN神经网络模块
/// @param path 文件路径
/// @param net 已训练的神经网络模块
MVS_EXPORT bool load_dnn_from_binary(const std::string& path, cv::dnn::Net& net);

////二维图像提取标志点及轮廓
//    MVS_EXPORT bool  ExtractPointAndEllipse(GRAYImage img,
//     std::vector<cv::RotatedRect>& ellipses, float scale_e,
//     std::string dnn_path);
//    MVS_EXPORT bool DeleteMarkpointArea(const Image8u& src, Imagef& dst, int color, float scale_e, std::string dnn_path);
}  // namespace rulermvs
#endif