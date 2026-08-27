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
    //二维提点参数（椭圆外形判断参数）
    bool   filterByArea = true;
    double minArea = 150 /*50*/ /*20*/ /*150*/;  // 轮廓最小面积
    double maxArea = 4000;                       // 轮廓最大面积
    int min_contour_size = 30;  // 圆环内圈轮廓最小周长

    bool   filterByCircularity = true;
    double minCircularity      = 0.8;  // 最小圆度限制
    double maxCircularity      = 1.2;  // 最大圆度限制

    bool   filterByConvexity    = true;
    double maxConvexity         = 1.0;          // 最大凸度限制
    double minConvexity         = 0.9 /*0.5*/;  // 最小凸度限制
    bool   filterByEccentricity = false;
    double maxEccentricity      = 1.0 /*0.9*/ /*0.7*/;  // 最大离心率限制

    double maxFitRMSE = 1.0;  // 轮廓椭圆拟合RMSE限制
    double maxContourError = 1.50;  // 轮廓椭圆拟合局部最大偏差限制

    std::string output_dir;

    //二维提点参数（图像处理参数）
    int lowThreshold_  = 30;
    int highThreshold_ = 50;
    int kernel_size_ = 7;  // 高斯核大小
    double alpha_ = 0.6;  // 高斯模糊处理函数中的高斯核的标准差

    //外部控制参数
    bool dnn_state = false;  // 默认不包含DNN模块
#ifdef USE_OPENCV_DNN
    cv::dnn::Net net_;  // DNN神经网络
#endif
    int scale_ = 1;  // 图片缩放系数，可取1、2、3、4
    std::vector<double> radiusVec {1.5, 3};  // 根据实际情况在demo中修改、增减
    bool has_distorted = false;  // 默认输入图片未经过畸变矫正
    bool has_rectified = false;  // 默认输入图片未经过极线校正
    bool output_rectifiedcoordinate = false;  // 默认输出原始左目坐标系下三维点
    bool multi_thread_ = false;  // 默认提点不开启多线程(竖块分割)
    //bool sparse = true;  // 默认标志点是稀疏分布(暂时废弃)
    bool long_distance_ = false;  // 默认是近距离扫描，远距离扫描提点精度变差，设定的误差阈值随之改变
    bool Shining_ = false;  // 是否为反光标志点(inputscaled_为true时会被屏蔽)
    bool inputscaled_ = true;  // 输入灰度图是否为缩放后的，若为true则三维前交时关闭部分约束（非常重要，强制为true）
    bool complex_Extractor = false;  // 使用新版提点算法

    //左右目标志点匹配及前交参数
    double LRmaxdist_ = 1.5;  // 左右目同名点极线校正欧氏距离阈值
    double min_depth_ = 100;  // 最小深度
    double max_depth_ = 500;  // 最大深度
    double tolerance_ = 0.15;  // 半径的RMSE的阈值，近距离下建议设置为0.12，同时是左右目轮廓面积的比值差阈值（很重要）
    int tolerance_num_ = 5;  // 容错性翻倍倍数，远距离状态下(inputscaled_为true时会被屏蔽)
    double distance_ratio = 0.15;  // 可调整，左右目同名点对之间的二维距离比值差异阈值
    double max_angle = MVS_PI_8;
    float LR_radius_ratio_th_ = 0.8f;  // 左右目解算半径比值阈值

    //新版标志点及过渡版二维参数
    bool strong_white_light = false;  // 强白光
    int adaptive_C = -8;  // 自适应阈值
    bool ori_adaptive = false;  // 是否在原图上重新adaptiveThreshold
    float ratio_valid = 0.95f;  // 轮廓像素有效点比率
    int DIF_threshold = 48;  // 内外圈灰度差值阈值
    float max_residual = 0.08f;  // 椭圆轮廓拟合最大残差
    float min_axis_length = 3.0f;
    float max_axis_length = 100.0f;
    float min_axis_ratio = 0.3f;
    float nms_Threshold = 0.9f;
    float non_edge_ratio = 0.8f;
};

class MVS_EXPORT CircleMarkerExtractor {
public:
    CircleMarkerExtractor() {}
    ~CircleMarkerExtractor() {}

    CircleMarkerExtractor(const CircleMarkerExtractor&) = default;

    // 提取圆形点
    bool CircleExtractSimple(const cv::Mat& img, std::vector<Corner>& corners,
        int idx, const CicrleConfigs& configs = CicrleConfigs());

    bool CircleExtractComplex(const cv::Mat& img, std::vector<Corner>& corners,
        int idx, const CicrleConfigs& configs = CicrleConfigs());

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
    void StereoRectifyCorners(const std::vector<Corner>& corners,
        std::vector<Corner>& rectified_corners, const cv::Mat& K,
        const cv::Mat& D, const cv::Mat& R, const cv::Mat& P, const cv::Mat& H);
    // 修正偏心误差（Ahn方法）
    void correctedEccentricityAhn(cv::Mat& img, cv::Mat& XYZ,
        cv::RotatedRect& box, double& epsilon_u, double& epsilon_v,
        double& dis);

    void transform_rulermvsData2cvMat(const std::vector<Pose>& camPoseVec, const std::vector<CameraPB>& cameraPBVec, int width, int height, int rectify_flags = cv::CALIB_ZERO_DISPARITY);
    std::string debug_dir;

    std::string current_debug_name;

    cv::Mat R_, T_, _F, K_left, K_right, D_left, D_right, P1_, P2_, R1_, R2_,
        Q_, RT_, RT_new_, map1x_, map1y_, map2x_, map2y_, H1_, H2_;
    Eigen::MatrixXd KL_eigen_, KR_eigen_, ML_eigen_, MR_eigen_;
    std::vector<cv::RotatedRect> ellipses_;  // 圆形标志点最内圈轮廓对应的椭圆参数
    int width_, height_;

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
            this->K_left = matrixes[0](cv::Rect(0, 0, 3, 3)).clone();
        } else 
        {
            MVS_WLOG << "Warning: the first matrix size error!";
            return false;
        }
        if (matrixes[1].size() == cv::Size(4, 3))
        {
            this->P2_ = matrixes[1].clone();
            this->K_right = matrixes[1](cv::Rect(0, 0, 3, 3)).clone();
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

    void SetTriangulationIntrinsicMatrix(
        const cv::Mat& K_L, const cv::Mat& K_R);  // 与左右目输入照片保持统一

protected:
    std::vector<uint8_t> blur_, flag1_, flag2_;
    //std::vector<short> dx_, dy_;
    std::vector<int> labels_, stack1_, stack2_;
    std::vector<cv::Point2f> unitVecs1_, unitVecs2_;
    std::vector<ushort> blur16U_;
    std::vector<float> img_grad_, img_dx_, img_dy_;
};

/// @brief 单帧图片提取圆形标志点
/// @param img 灰度图或3通道彩色图
/// @param corners 畸变矫正后的标志点中心corner数组
/// @param index 相机编号，index=0为左目，否则为右目
MVS_EXPORT bool ExtractCircleMarker(const cv::Mat& img,
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
    Point3dVec& points, cv::Mat& R1, cv::Mat& P1, cv::Mat& P2, double maxdist, double min_depth = 100.0, 
    double max_depth = 500.0, bool long_distance = false, bool Shining = false, double tolerance = 0.15, int tolerance_num = 5, bool scaled = false, double distance_ratio = 0.15, double max_angle = MVS_PI_8);

MVS_EXPORT std::vector<std::pair<Corner, Corner>> getStereoMarker3dPoints(
    const cv::Mat& img_L, const cv::Mat& img_R,
    CircleMarkerExtractor& corner_extractor, Point3dVec& points,
    std::vector<cv::RotatedRect>& ellipses_L,
    std::vector<cv::RotatedRect>& ellipses_R,
    const CicrleConfigs& configs = CicrleConfigs());

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

#ifdef USE_OPENCV_DNN
/// @brief 读取DNN神经网络模块
/// @param path 文件路径
/// @param net 已训练的神经网络模块
MVS_EXPORT bool load_dnn_from_binary(const std::string& path, cv::dnn::Net& net);
#endif

////二维图像提取标志点及轮廓
//    MVS_EXPORT bool  ExtractPointAndEllipse(GRAYImage img,
//     std::vector<cv::RotatedRect>& ellipses, float scale_e,
//     std::string dnn_path);
//    MVS_EXPORT bool DeleteMarkpointArea(const Image8u& src, Imagef& dst, int color, float scale_e, std::string dnn_path);
}  // namespace rulermvs
#endif