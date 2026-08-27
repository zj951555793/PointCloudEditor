#ifndef _RULERMVS_MULTILINES_MULTILINES_HPP_
#define _RULERMVS_MULTILINES_MULTILINES_HPP_
#include "rulermvs/eigen.hpp"
#include "rulermvs/cv.hpp"
#include "rulermvs/point.hpp"
#include "rulermvs/rgbd.hpp"

namespace rulermvs
{
enum LineAngleMode {
    Forty_five = 0,    /*/*/
    Triple_forty_five, /*\*/
    Multiple_Vertical, /*|||*/
    Single_Vertical,   /*|*/
    White_pure,
    None
};

struct MVS_EXPORT QuadraticSurface {
    int idx = -1;
    //double a, b, c, d, e, f;  // z = a*x^2 + b*y^2 + c*x*y + d*x + e*y + f
    double A, B, C, D, E, F, G, H, I, J;
    //A*x^2 + B*y^2 + C*z^2 + D*x*y + E*x*z + F*y*z + G*x + H*y + I*z + J = 0

    QuadraticSurface()
        : A(0.0), B(0.0), C(0.0), D(0.0), E(0.0), F(0.0), G(0.0), H(0.0), I(0.0), J(0.0)
    {}
    QuadraticSurface(
        double a_, double b_, double c_, double d_, double e_, double f_)
        : A(a_), B(b_), D(c_), G(d_), H(e_), J(f_), C(0.0), E(0.0), F(0.0), I(-1.0)
    {}
    QuadraticSurface(double A_, double B_, double C_, double D_, double E_,
        double F_, double G_, double H_, double I_, double J_)
        : A(A_), B(B_), C(C_), D(D_), E(E_), F(F_) , G(G_) , H(H_) , I(I_) , J(J_)
    {}
    QuadraticSurface(const QuadraticSurface& other)
    {
        A = other.A;
        B = other.B;
        C = other.C;
        D = other.D;
        E = other.E;
        F = other.F;
        G = other.G;
        H = other.H;
        I = other.I;
        J = other.J;
        idx = other.idx;
    }

    void toMatrix3d(
        Eigen::Matrix3d& Q, Eigen::Vector3d& L, double& constant) const
    {
        Q.setZero();
        //L.setZero();
        Q(0, 0) = -A / I, Q(1, 1) = -B / I, Q(0, 1) = Q(1, 0) = -D / (2.0 * I);
        L = Eigen::Vector3d(-G / I, -H / I, -1.0);
        constant = -J / I;
        // PT * Q * P + LT * P + f = 0
    }
    static QuadraticSurface fromMatrix3d(const Eigen::Matrix3d& Q,
        const Eigen::Vector3d& L, const double constant, int index = -1)
    {
        QuadraticSurface surface;
        surface.A = -Q(0, 0) / L(2);
        surface.B = -Q(1, 1) / L(2);
        surface.D = -2.0 * Q(0, 1) / L(2);
        surface.G = -L(0) / L(2);
        surface.H = -L(1) / L(2);
        surface.J = -constant / L(2);
        surface.C = surface.E = surface.F = 0.0;
        surface.I = -1.0;
        surface.idx = index;
        return surface;
    }

    void toMatrix4d(Eigen::Matrix4d& Q) const
    {
        Q.setZero();
        Q(0, 0) = A, Q(1, 1) = B, Q(2, 2) = C, Q(3, 3) = J,
             Q(0, 1) = Q(1, 0) = D / 2.0, Q(0, 2) = Q(2, 0) = E / 2.0,
             Q(1, 2) = Q(2, 1) = F / 2.0, Q(0, 3) = Q(3, 0) = G / 2.0,
             Q(1, 3) = Q(3, 1) = H / 2.0, Q(2, 3) = Q(3, 2) = I / 2.0;
        //X =[x, y, z, 1]T
        //XT * Q * X = 0
    }
    static QuadraticSurface fromMatrix4d(
        const Eigen::Matrix4d& Q, int index = -1)
    {
        QuadraticSurface surface;
        surface.A = Q(0, 0);
        surface.B = Q(1, 1);
        surface.C = Q(2, 2);
        surface.J = Q(3, 3);
        surface.D = 2.0 * Q(0, 1);
        surface.E = 2.0 * Q(0, 2);
        surface.F = 2.0 * Q(1, 2);
        surface.G = 2.0 * Q(0, 3);
        surface.H = 2.0 * Q(1, 3);
        surface.I = 2.0 * Q(2, 3);
        surface.idx = index;
        return surface;
    }

    static QuadraticSurface transformQuadraticSurface(
        const QuadraticSurface& original,
        const Eigen::Matrix3d& rotation_matrix);
    static QuadraticSurface transformQuadraticSurface(
        const QuadraticSurface& original,
        const Eigen::Matrix4d& transform_matrix);
};

struct MVS_EXPORT LightPlane {
    int idx = -1;
    int imgY = -1;
    bool state = false;
    double A, B, C, D;

    LightPlane() : idx(-1), imgY(-1), state(false) {}
    LightPlane(double a_, double b_, double c_, double d_)
        : A(a_), B(b_), C(c_), D(d_), state(true) {}
    LightPlane(double a_, double b_, double c_, double d_, int idx_, int imgY_)
        : A(a_), B(b_), C(c_), D(d_), idx(idx_), imgY(imgY_), state(true)
    {}
    LightPlane(const LightPlane& other)
    {
        A = other.A;
        B = other.B;
        C = other.C;
        D = other.D;
        idx = other.idx;
        imgY = other.imgY;
        state = other.state;
    }
    LightPlane normalizePlane() const
    {
        LightPlane normalized_plane = *this;
        double norm = std::sqrt(normalized_plane.A * normalized_plane.A +
                                normalized_plane.B * normalized_plane.B +
                                normalized_plane.C * normalized_plane.C);
        normalized_plane.A /= norm;
        normalized_plane.B /= norm;
        normalized_plane.C /= norm;
        normalized_plane.D /= norm;
        return normalized_plane;
    }
    void toMatrix4d(Eigen::Matrix4d& Q) const
    {
        Q.setZero();
        Q(0, 3) = Q(3, 0) = A / 2.0, Q(1, 3) = Q(3, 1) = B / 2.0,
             Q(2, 3) = Q(3, 2) = C / 2.0, Q(3, 3) = D;
        //X =[x, y, z, 1]T
        //XT * Q * X = 0
    }
    static LightPlane fromMatrix4d(const Eigen::Matrix4d& Q, int idx = -1,
        int imgY = -1, bool state = false)
    {
        LightPlane plane;
        plane.A = 2.0 * Q(0, 3);
        plane.B = 2.0 * Q(1, 3);
        plane.C = 2.0 * Q(2, 3);
        plane.D = Q(3, 3);
        plane.idx = idx;
        plane.imgY = imgY;
        plane.state = state;
        return plane;
    }
    static LightPlane transformLightPlane(
        const LightPlane& original, const Eigen::Matrix4d& transform_matrix);
};

struct LaserNode {
    int x, y;
    int groupId = -1;
    int next_idx = -1;  // 链表指向下一个节点的索引（该索引可用于查找）
    int pind = -1; // 当前节点在Nodes中的索引（可用于查找）
    cv::Point2f subPixel;
    cv::Vec2f normal;

    LaserNode(int _x, int _y) : x(_x), y(_y) {}
    LaserNode(int _x, int _y, int _index) : x(_x), y(_y), pind(_index) {}
};

struct LaserNodeMatch {
    int left_pind;
    int right_pind;
    int plane_label;
    float dist;
    LaserNodeMatch(
        int _left_pind, int _right_pind, int _plane_label, float _dist)
        : left_pind(_left_pind)
        , right_pind(_right_pind)
        , plane_label(_plane_label)
        , dist(_dist)
    {}
};

struct LineConfigs {
    //二维提线参数
    int minArea = 15;  // 连通区域最小面积（新版解码中表示激光线的最小二维像素点数量）
    int Estimate_points_per_row = 34;  // 每一行预估最大激光点数量，设置为光平面数量的3倍
    float min_val = 0.15f;  // 最小强度（大于该值）
    float lambda_threshold_ = -0.001f;  // 特征值阈值（小于该值）
    float response_threshold_ = 0.3f;  // 小大特征值比值绝对值阈值（小于该值）
    float t_threshold_ = 0.6f;  // 偏移量绝对值阈值（小于该值）
    float normal_threshold_ = 0.985f;  // 法向绝对值阈值（小于该值）

    int kernel_size_ = 7;  // 高斯核大小
    double alpha_ = 3.0;  // 高斯模糊处理函数中的高斯核的标准差

    //外部控制参数
    bool has_distorted = false;  // 默认输入图片未经过畸变矫正
    bool has_rectified = false;  // 默认输入图片未经过极线校正
    bool output_rectifiedcoordinate = false;  // 默认输出原始左目坐标系下三维点
    //LineAngleMode line_mode = LineAngleMode::Forty_five;
    int interpolation = cv::INTER_LINEAR;  // remap插值方式

    //左右目激光线匹配及前交参数
    double min_depth_ = 100;  // 最小深度
    double max_depth_ = 500;  // 最大深度
    double max_dist_ = 0.5;  // 三维点距离光平面方程最大距离
    float max_theta_ = CV_PI / 6;
    float smooth_ = 0.15f;
    bool new_decode = true;  // 使用新版的解码方法
};

class MVS_EXPORT LineExtractor {
public:
    LineExtractor() {}
    ~LineExtractor() {}

    LineExtractor(const LineExtractor&) = default;

    // 提取激光线
    bool LineExtractSimple(const cv::Mat& img, cv::Mat& subimgX,
        cv::Mat& subimgY, std::vector<std::vector<cv::Point2f>>& pointsRows,
        const LineConfigs& configs = LineConfigs());
    // SSE加速、九宫格NMS法
    bool LineExtractSimple_Fast(const cv::Mat& img, cv::Mat& subimgX,
        cv::Mat& subimgY, std::vector<std::vector<cv::Point2f>>& pointsRows,
        const LineConfigs& configs = LineConfigs());

    bool LineExtractSimple(const cv::Mat& img, std::vector<LaserNode>& nodes,
        std::vector<std::vector<int>>& groups,
        const LineConfigs& configs = LineConfigs());

    void OrganizePointsByRow(std::vector<LaserNode>& nodes,
        std::vector<std::vector<LaserNode>>& rowLaserNodes, const int img_w,
        const int img_h, const int points_per_row);

    void getStereoRectifiedLine3dPoints(
        const std::vector<std::vector<cv::Point2f>>& pointsRows_L,
        const std::vector<std::vector<cv::Point2f>>& pointsRows_R,
        std::vector<cv::Point3d>& points, int index, double max_dist,
        double min_depth, double max_depth);

    void getStereoRectifiedLine3dPoints(std::vector<cv::Point3f>& points,
        int index, double max_dist, double min_depth, double max_depth,
        float max_theta, float smooth);

    void getStereoRectifiedLine3dPoints(const cv::Mat& subimgX_L,
        const cv::Mat& subimgY_L,
        const std::vector<std::vector<cv::Point2f>>& pointsRows_L,
        const std::vector<std::vector<cv::Point2f>>& pointsRows_R,
        /*cv::Mat rectified_img_L,*/ std::vector<cv::Point3f>& points, int index,
        double max_dist);

    void init(const std::vector<Pose>& camPoseVec,
        const std::vector<CameraPB>& cameraPBVec, int width, int height,
        const std::vector<std::vector<LightPlane>>& original_rows_surfaces,
        int rectify_flags = cv::CALIB_ZERO_DISPARITY, int MapType = CV_16SC2,
        bool has_rectified = false, const int coeff_num = 4);

    void init(const std::vector<Pose>& camPoseVec,
        const std::vector<CameraPB>& cameraPBVec, int width, int height,
        const std::vector<std::vector<QuadraticSurface>> planes,
        int rectify_flags = cv::CALIB_ZERO_DISPARITY,
        bool has_rectified = false, const int coeff_num = 7);

    //void transform_rulermvsData2cvMat(const std::vector<Pose>& camPoseVec, const std::vector<CameraPB>& cameraPBVec, int width, int height, int rectify_flags = cv::CALIB_ZERO_DISPARITY);

    void computeQ_By_ProjectionMatrix() {
        this->Q_ = cv::Mat::eye(4, 4, CV_64F);
        double focal_length = this->P1_.at<double>(0, 0);  //焦距
        double T_norm =
            this->P2_.at<double>(0, 3) / focal_length;  //平移向量模长（有正负性）
        this->Q_.at<double>(0, 3) = -this->P1_.at<double>(0, 2);
        this->Q_.at<double>(1, 3) = -this->P1_.at<double>(1, 2);
        this->Q_.at<double>(2, 3) = focal_length;
        this->Q_.at<double>(3, 3) =
            (this->P1_.at<double>(0, 2) - this->P2_.at<double>(0, 2)) / T_norm;
        this->Q_.at<double>(3, 2) = -1 / T_norm;
        this->Q_.at<double>(2, 2) = 0.0;
    }

    cv::Mat R_, T_, K_left, K_right, D_left, D_right, P1_, P2_, R1_, R2_, Q_,
        RT_new_, map1x_, map1y_, map2x_, map2y_;
    std::vector<LaserNode> nodesL_, nodesR_;
    std::vector<std::vector<int>> groupsL, groupsR;
    std::vector<std::vector<LaserNode>> rowLaserNodesL_, rowLaserNodesR_;
    std::vector<uint8_t> rectified_L_, rectified_R_;

protected:
    std::vector<float> abs_ts_, blur_;
    std::vector<uint8_t> flag_;
    std::vector<cv::Point2f> sub_pixels_;
    std::vector<cv::Vec2f> normals_;
    std::vector<ushort> blur16U_;
    std::vector<int> inds_;
    //std::vector<int*> inds_ptrs_;
    std::vector<std::vector<int>> pointMatchL_;
    std::vector<std::vector<int>> pointMatchR_;
    std::vector<LaserNodeMatch> matchs_;
    int width_, height_;

    std::vector<int> light_plane_nums; //光平面数量
    std::vector<std::vector<std::vector<LightPlane>>> rows_planes_;  // 绝对光平面参数
    std::vector<std::vector<QuadraticSurface>> surfaces_;
    std::vector<std::shared_ptr<Eigen::MatrixXd>> surfaces_coeff_matrix_;
    std::vector<std::shared_ptr<Eigen::Matrix<double, Eigen::Dynamic, 3>>>
        IntersectPlaneMatrix_L;
    std::vector<std::shared_ptr<Eigen::Matrix<double, Eigen::Dynamic, 3>>>
        IntersectPlaneMatrix_R;
    //std::vector<std::vector<std::shared_ptr<Eigen::MatrixXd>>>
    //    surfaces_coeff_rows_matrix_;

    void transform_rulermvsData2cvMat(const std::vector<Pose>& camPoseVec,
        const std::vector<CameraPB>& cameraPBVec, int width, int height,
        int rectify_flags = cv::CALIB_ZERO_DISPARITY, int MapType = CV_16SC2);
    // coeff_num为4、6、7或10
    void SetLightplaneParam(
        const std::vector<std::vector<QuadraticSurface>> planes,
        bool has_rectified = false,
        const int coeff_num = 7);  // 默认输入光平面方程未经过极线校正
    // coeff_num必须为4
    void SetLightplaneParam(const std::vector<std::vector<LightPlane>> planes,
        const int height,
        bool has_rectified = false);  // 默认输入光平面方程未经过极线校正
};

/// @brief 读取多相机标定文件，默认双目相机且无极线校正，第三个相机为纹理相机
/// @param path 标定文件路径，格式需统一
/// @param camPoseVec (输出)各个相机的(原始)外参，按顺序排列
/// @param cameraPBVec (输出)各个相机的(原始)内参，按顺序排列
/// @param camera_count 相机个数，与标定文件内容保持一致
/// @param output_rectify 是否求解双目相机极线校正(仅针对第一第二个相机)
/// @param rectify_flags 极线校正参数
/// @return 每个相机相对于第一个相机的相对位姿，每个相机的内参，第一个相机(左目)极线校正时的旋转矩阵，均包括极线校正和原始相机参数两种情况
MVS_EXPORT std::tuple<std::vector<Pose>, std::vector<CameraPB>, Pose>
load_multicamera_param_from_ascii_lines(const std::string& path,
    std::vector<Pose>& camPoseVec, std::vector<CameraPB>& cameraPBVec,
    int camera_count, int width, int height, bool output_rectify = false,
    int rectify_flags = cv::CALIB_SAME_FOCAL_LENGTH);

MVS_EXPORT std::tuple<std::vector<Pose>, std::vector<CameraPB>, Pose>
load_multicamera_lightplane_param_from_ascii_lines(const std::string& path,
    std::vector<Pose>& camPoseVec, std::vector<CameraPB>& cameraPBVec,
    std::vector<std::vector<LightPlane>>& rows_planes, int camera_count,
    int width, int height, bool output_rectify = false,
    int rectify_flags = cv::CALIB_SAME_FOCAL_LENGTH);

/// @param coeff_num 二次曲面系数，7（退化二次曲面）或4（平面）或6（退化二次曲面）
MVS_EXPORT std::vector<std::vector<QuadraticSurface>>
load_lightplane_param_from_ascii(
    const std::string& path, const int coeff_num = 7);

MVS_EXPORT std::vector<std::vector<LightPlane>>
load_rows_lightplane_param_from_ascii(
    const std::string& path, const int coeff_num = 4);

/// @brief 单帧图片提取激光线
/// @param img 灰度图或3通道彩色图
MVS_EXPORT bool ExtractLine(const cv::Mat& img, LineExtractor& line_extractor,
    cv::Mat& subimgX, cv::Mat& subimgY,
    std::vector<std::vector<cv::Point2f>>& pointsRows, int index,
    const LineConfigs& configs = LineConfigs());

static inline void getStereoLine3dPoints(const cv::Mat& img_L,
    const cv::Mat& img_R, LineExtractor& line_extractor, Point3fVec& points,
    const LineAngleMode& line_mode, const LineConfigs& configs = LineConfigs())
{
    if (img_L.empty() || img_R.empty() || line_extractor.P1_.empty() ||
        line_extractor.P2_.empty() || line_extractor.R1_.empty() ||
        line_extractor.Q_.empty() || line_extractor.map1x_.empty() ||
        line_extractor.map1y_.empty() || line_extractor.map2x_.empty() ||
        line_extractor.map2y_.empty())
        return;
    cv::Mat rectified_img_L, rectified_img_R;
    if (!configs.has_rectified) {
        if (!line_extractor.rectified_L_.empty() &&
            !line_extractor.rectified_R_.empty())
        {
            rectified_img_L = cv::Mat(img_L.rows, img_L.cols, CV_8UC1,
                line_extractor.rectified_L_.data());
            rectified_img_R = cv::Mat(img_R.rows, img_R.cols, CV_8UC1,
                line_extractor.rectified_R_.data());
        }
        cv::remap(img_L, rectified_img_L, line_extractor.map1x_,
            line_extractor.map1y_, configs.interpolation, cv::BORDER_CONSTANT,
            cv::Scalar(0));
        cv::remap(img_R, rectified_img_R, line_extractor.map2x_,
            line_extractor.map2y_, configs.interpolation, cv::BORDER_CONSTANT,
            cv::Scalar(0));
    }
    else {
        rectified_img_L = img_L, rectified_img_R = img_R;
    }
    cv::Mat subimgX_L, subimgY_L, subimgX_R, subimgY_R;
    std::vector<std::vector<cv::Point2f>> pointsRows_L, pointsRows_R;
    bool state1 = ExtractLine(rectified_img_L, line_extractor, subimgX_L,
        subimgY_L, pointsRows_L, 0, configs);
    bool state2 = ExtractLine(rectified_img_R, line_extractor, subimgX_R,
        subimgY_R, pointsRows_R, 1, configs);
    if (state1 && state2)
    {
        std::vector<cv::Point3f> points_temp;
        if (configs.new_decode) {
            line_extractor.getStereoRectifiedLine3dPoints(points_temp,
                /*configs.*/ line_mode, configs.max_dist_, configs.min_depth_,
                configs.max_depth_, configs.max_theta_, configs.smooth_);
        } else {
            line_extractor.getStereoRectifiedLine3dPoints(subimgX_L, subimgY_L,
                pointsRows_L, pointsRows_R, /*rectified_img_L, */ points_temp,
                /*configs.*/ line_mode, configs.max_dist_);
        }
        if (!configs.output_rectifiedcoordinate && !points_temp.empty())
        {
            cv::transform(points_temp, points_temp, line_extractor.R1_.inv());
        }
        points.resize(points_temp.size());
        std::memcpy(points.data(), points_temp.data(),
            points_temp.size() * sizeof(cv::Point3f));
        if (points.empty()) {
            MVS_WLOG << "StereoLineMatcher: Please check the left eye camera "
                        "or right eye camera, failed to match Line.";
        }
        return;
    }
    else {
        MVS_WLOG << "StereoLineMatcher: Please check the left eye camera or "
                    "right eye camera, there are not any Line in one "
                    "of the frames.";
        return;
    }
}

}  // namespace rulermvs
#endif