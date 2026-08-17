#ifndef _RULERMVS_RGBD_MARKERFUSIONMATCH_RGBD_MARKERFUSIONMATCH_HPP_
#define _RULERMVS_RGBD_MARKERFUSIONMATCH_RGBD_MARKERFUSIONMATCH_HPP_
#include "rulermvs/rgbd.hpp"
#include "rulermvs/pointcloud.hpp"
#include "rulermvs/MarkerExtractor.hpp"
#include "rulermvs/Tracker.hpp"
#include <map>
namespace rulermvs
{
MVS_EXPORT void setRGBD_MarkerFusionThreadNum(int num);

struct IRGBD_MarkerFusionPara {
    rulermvs::String verbose_dir = "";  ///< 输出中间数据
    // 纹理特征参数
    int   key_max_num    = -1;               ///< 特征点最大数量
    int   key_max_iter   = 10000;            ///< 特征匹配最大迭代次数
    float key_max_dist   = 3.0f;             ///< 最大点间距
    float key_max_angle  = (float)MVS_PI_4;  ///< 最大点间法向角度
    int   key_min_match  = 7;                ///< 最小匹配数量
    float key_edge_ratio = .9f;              ///< 边长比值
    // 几何特征点参数
    int   geo_nms        = 3;      ///< 几何特征点极大值抑制范围
    int   geo_pfh        = 7;      ///< 几何特征点PFH范围
    int   geo_fpfh       = 3;      ///< 几何特征点FPFH范围
    float geo_gamma      = 0.1f;   ///< 检测特征点的各向最小比值
    int   geo_radius     = 3;      ///< 特征点提取范围
    int   geo_max_iter   = 10000;  ///< 特征点匹配最大迭代次数
    float geo_max_dist   = 3.0f;   ///< 最大点间距
    float geo_max_angle  = (float)MVS_PI_4;  ///< 最大点间法向角度
    int   geo_min_match  = 7;                ///< 最小匹配数量
    float geo_edge_ratio = .9f;              ///< 边长比值
    int geo_maxnum = -1;           ///< 特征点截取数量，-1表示全部保留
    bool bCrossCheck = true;  ///< 暴力匹配时是否进行交叉验证
    // ICP参数
    int   icp_max_iter    = 3;
    float icp_max_dist    = 3.0f;
    float icp_max_angle   = (float)MVS_PI_4;
    float icp_min_overlap = 0.5f;
    // 里程计对应参数
    MatchMode odometry_mode     = MatchMode::Both;
    int       odometry_max_step = 15;
    int       global_max_step   = 15;
    int       odometry_key_step = 7;
    // 回环检测
    int  loop_option        = 2;
    float loop_edge_ratio    = 0.99f;
    float loop_min_overlap   = 0.5f;
    int   loop_geo_min_match = 10;
    int   loop_key_min_match = 10;

    // 重定位对应参数
    int   relocate_option        = 1;
    float relocate_edge_ratio    = 0.95f;
    float relocate_min_overlap   = 0.5f;
    int   relocate_geo_min_match = 7;
    int   relocate_key_min_match = 7;
    // 优化项对应参数
    int   posegraph_max_iter         = 1000;
    float posegraph_max_dist         = 3.0f;
    float posegraph_loop_closure     = 1.0f;
    float posegraph_loop_max_dist    = 10.0f;
    float optimize_voxel_size        = 6.0f;
    int   optimize_rgbd_max_group    = 15;
    int   optimize_rgbd_max_iter     = 50;
    float optimize_rgbd_min_overlap  = 0.001f;
    float optimize_rgbd_max_dist     = 3.0f;
    float optimize_rgbd_theta        = 0.000001f;
    float optimize_rgbd_max_angle    = (float)MVS_PI_4;
    int   optimize_cloud_max_iter    = 50;
    float optimize_cloud_theta       = 0.000001f;
    float optimize_cloud_min_overlap = 0.001f;
    float optimize_cloud_max_dist    = 10.0f;
    float optimize_cloud_max_angle   = (float)MVS_PI_4;

    bool consecutive = true;  //是否连续扫描模式，该模式下特征对齐frames大小是1且当前帧与前一帧对齐失败时会将第一帧替换掉并重新初始化
    //标志点对齐模式下，连续若干帧对齐失败时，会清空全局地图并重新初始化
    //bool calculate_pose = true;  //是否计算位姿，若为false当前帧既无法初始化成功也不参与对齐，不存储帧的内存(已废弃)
    bool use_marker_track = false;
    bool input_rectified = false;  //默认输入图片为原始图片
    bool input_rectifiefcamera = true;  //是否输入极线校正后相机参数，true表示内部调用stereorectify函数后再读取输入的P1、P2等
    int width = -1;  //照片宽度
    int height = -1;  //照片高度
    int rectify_flags = cv::CALIB_SAME_FOCAL_LENGTH;
    CicrleConfigs configs;
    int min_match_num = 4;  //最小同名点数量
    int min_mappoint_num = 7;  //最小地图点数量，小于此值无法初始化全局地图
    double MAX_PTBIAS_ERROR = 0.18;  //同名点间的均方根差，相邻帧的该值对于下一帧的误匹配筛选具有重要意义
    int match_adjacentframe_nums = 1;  //与相邻帧对齐时，默认只选取一帧，只与全局地图对齐时该参数没有意义
    bool reproject_KeyFrame = false;  //是否将全局地图点重投影到关键帧生成新的帧
    double site_diff_thres = 1.0;  //两对同名点之间的线段距离差阈值
    double p3d_match_thres = 1.5;  //同名点匹配对在RT作用后两者之间的距离阈值
    double p3d_nearest_thres = 3.0;  //求解RT后合并单帧到global_map时，未参与匹配的同名点在同名点在RT作用后两者间的距离阈值，大于p3d_match_thres
    int iter_Num = 1;  //BA优化迭代次数
    //bool insert_newframe = true;  //对齐成功或初始化成功是否将当前帧添加到global_map中，false表示否定(calculate_pose为false时会被屏蔽)(已废弃)
    slam::FrameMatchMode match_mode = slam::FrameMatchMode::match_adjacentFrame_only;  //默认为只与相邻帧对齐
    bool savePointer = false;  //不开启共享指针(避免内存爆炸)

    int max_failedframes_num = 5;  //连续失败帧数量，超过该值说明全局地图globalmap存在问题，非标志点模式下，超过该值进行重定位
    int min_stable_globalmapframes = 10;  //全局地图内置稳定帧数量，低于此值表明该地图尚不稳定
    int BA_min_added_mappoints = 7;  //BA优化要求的最小新增地图点数量
    int BA_min_added_frames = 6;  //BA优化要求的最小新增Frame数量
};

/// @brief 获取拼接的结果数据
struct IRGBD_MarkerFusionResult {
    /// @brief 匹配状态标识
    enum : int { Failed = 0, Init, Succeed, Key, Relocation,Reset };
    virtual ~IRGBD_MarkerFusionResult() {}
    virtual int           getID() const            = 0;
    virtual const void* getUserData() const = 0;
    virtual Pose getPose() const = 0;
    virtual int           getState() const         = 0;
    virtual Image3f getVmap() const = 0;
    virtual Image3f getNmap() const = 0;
    virtual Image8u getMask() const = 0;
    virtual PointCloud getPointCloud() const = 0;
    virtual RGBPointCloud getRGBPointCloud() const = 0;
    virtual Point3dVec getMarker3Dpoints() const = 0;
    virtual Point3dVec getGlobalmap3Dpoints() const = 0;
};

/// @brief 在线拼接类,多帧RGBD在线配准和姿态优化
struct IRGBD_MarkerFusion {
    using Result    = IRGBD_MarkerFusionResult;
    using ResultSet = std::vector<const Result*>;
    using Parameter = IRGBD_MarkerFusionPara;
    using RetFunc   = std::function<void(const Result&)>;
    using InputFunc = std::function<void(RGBImage&, Imagef&)>;

    typedef std::shared_ptr<IRGBD_MarkerFusion> Ptr;
    /// @brief 创建实例
    static MVS_EXPORT Ptr create(const CameraP&, const IRGBD_MarkerFusionPara&,
        bool is_vio = false, int threadNum = -1);
    static MVS_EXPORT Ptr create(const std::vector<Pose>& camPoseVec, const std::vector<CameraPB>& cameraPBVec, const IRGBD_MarkerFusionPara&,
        bool is_vio = false, int threadNum = -1, const std::vector<cv::Mat>& input_matrixes = std::vector<cv::Mat>());
    /// @brief 虚析构函数，占位
    virtual inline ~IRGBD_MarkerFusion() {}
    /// @brief 初始化
    /// @param RGBD相机内参
    /// @param para 拼接参数
    /// @return 返回是否初始化成功
    //virtual bool init(const CameraP&, const IRGBD_MarkerFusionPara&, bool is_vio = false, int threadNum = -1) = 0;
    /// @brief 暂停
    virtual void stop() = 0;
    /// @brief 重置
    virtual void reset(bool is_vio, bool binocular = false) = 0;
    /// @brief 清空内存
    virtual void clear() = 0;
    /// @brief 等待已有任务运行完成;
    virtual void waiting() const = 0;
    /// @brief 返回相机参数引用
    virtual CameraP& camera() = 0;
    /// @brief 返回里程计参数引用
    virtual Parameter& param() = 0;
    /// @brief 设置里程计参数
    virtual void setparam(const Parameter& para) = 0;
    /// @brief 返回匹配结果
    /// @return 结果数据集
    virtual ResultSet results() const = 0;
    /// @brief 设置标志点全局地图参数
    virtual void SetGlobalMap(const Parameter& para) = 0;
    /// @brief 回调添加深度图和纹理图,用于结构光解码数据导入
    virtual void proc(InputFunc in, RetFunc ret) = 0;
    /// @brief 增量处理RGBD数据帧
    virtual void proc(const RGBDImage& frame, RetFunc ret = 0, bool calculate_pose = true, bool insert_newframe = true) = 0;
    virtual void proc(const RGBDImage& frame, RetFunc ret, void* userData, bool calculate_pose = true, bool insert_newframe = true) = 0;
    /// @brief 增量处理左右目灰度图gray_pair数据帧
    virtual void proc(
        const std::pair<Image8u, Image8u>& gray_pair, RetFunc ret = 0, bool calculate_pose = true, bool insert_newframe = true) = 0;
    virtual void proc(const std::pair<Image8u, Image8u>& gray_pair, RetFunc ret,
        void* userData, bool calculate_pose = true, bool insert_newframe = true) = 0;
    /// @brief 增量处理左右目灰度图gray_pair、深度图depth数据帧
    virtual void proc(
        const std::pair<Image8u, Image8u>& gray_pair, const Imagef& depth, RetFunc ret = 0, bool calculate_pose = true, bool insert_newframe = true) = 0;
    virtual void proc(const std::pair<Image8u, Image8u>& gray_pair, const Imagef& depth, RetFunc ret,
        void* userData, bool calculate_pose = true, bool insert_newframe = true) = 0;
    /// @brief 增量处理RGBD数据帧,等待该帧匹配完成并返回匹配结果;
    int proc(const rulermvs::RGBImage& rgb, const rulermvs::Imagef& depth, rulermvs::Pose& pose)
    {
        int ret;
        proc(rulermvs::RGBDImage(rgb, depth), [&ret, &pose](const Result& result) {
            ret = result.getState(), pose = result.getPose();
        });
        waiting();
        return ret;
    }
    virtual void load(const rulermvs::RGBImage& rgb, const rulermvs::Imagef& depth, Pose& pose,int index) = 0;
    virtual void load(int fuse_step) = 0;
    /// @brief 合并地图点
    virtual PointCloud fuseMap() const = 0;
    virtual RGBPointCloud fuseRGBMap() const = 0;
    /// @brief 所有对齐帧标志点(未合并同名点)
    virtual Point3fVec fuseAllFramesMarker3Dpoints() const = 0;
    /// @brief 获得标志点全局地图
    virtual slam::GlobalMap* getMarkerGlobalMap() const = 0;
    /// @brief 删除地图最后一帧(标志点)
    virtual void deleteLastFrame() = 0;
    /// @brief 姿态优化
    /// @param progress 进度条
    virtual std::pair<bool, Pose> rescan(const RGBImage& rgb, const Imagef& depth, Pose& pose, int min_num,int flag) = 0;
    
    virtual void optimize(rulermvs::ProgressBar progress = 0) = 0;  //标志点模式下不可调用
    virtual void optimize_point_clouds(rulermvs::ProgressBar progress = 0) = 0;  //标志点模式下可调用

    virtual void equalizeSpeed() = 0;

    virtual void removeLast(int count) = 0;

    virtual void removeAt(int start, int count) = 0;

    virtual void remove(const std::set<int>& ids) = 0;

    virtual std::pair<int, std::vector<Pose>> getOptRes() const = 0;

    virtual void rescan_set() = 0;
};

/// @brief 读取多相机标定文件，默认双目相机且无极线校正，第三个相机为纹理相机
/// @param path 标定文件路径，格式需统一
/// @param camPoseVec (输出)各个相机的(原始)外参，按顺序排列
/// @param cameraPBVec (输出)各个相机的(原始)内参，按顺序排列
/// @param camera_count 相机个数，与标定文件内容保持一致
/// @param output_rectify 是否求解双目相机极线校正(仅针对第一第二个相机)
/// @param rectify_flags 极线校正参数
/// @return 每个相机相对于第一个相机的相对位姿，每个相机的内参，第一个相机(左目)极线校正时的旋转矩阵，均包括极线校正和原始相机参数两种情况
MVS_EXPORT std::tuple<std::vector<Pose>, std::vector<CameraPB>, Pose> load_multicamera_param_from_ascii(const std::string& path,
    std::vector<Pose>& camPoseVec, std::vector<CameraPB>& cameraPBVec,
    int camera_count, int width, int height, bool output_rectify = false,
    int rectify_flags = cv::CALIB_SAME_FOCAL_LENGTH);

/// @brief 读取厂家提供的极线校正后相机部分参数文件，默认双目相机
/// @param path 标定文件路径，格式需统一
/// @param matrixes (输出)4个矩阵，分别为P1、P2、R1、R2(顺序不可乱，方便后续使用)
/// @return 左右目极线校正后内参，第一个相机(左目)极线校正时的旋转矩阵，极线校正后右目的相对位姿
static inline std::tuple<std::vector<CameraPB>, Pose, Pose> load_stereorectify_param_from_ascii(const std::string& path, std::vector<cv::Mat>& matrixes, int width = -1, int height = -1)
{
    std::vector<CameraPB> RectifycameraPBVec(2);//只有左右目
    FILE* f = nullptr;
    fopen_s(&f, path.c_str(), "r");
    if (!f) return std::make_tuple(RectifycameraPBVec, Pose(), Pose());

    char buf[1024];
    for (int i = 0; i < 1; ++i) fgets(buf, 1024, f);
    int id;
    matrixes.resize(4);
    if (fgets(buf, 1024, f)) {
        matrixes[2] = cv::Mat(4, 4, CV_64FC1);
        auto* p = matrixes[2].ptr<double>();
        sscanf_s(buf,
            "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
            &id, &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6], &p[7], &p[8],
            &p[9], &p[10], &p[11], &p[12], &p[13], &p[14], &p[15]);
        //rt1.setTranslation(tx, ty, tz);
    }
    if (fgets(buf, 1024, f)) {
        matrixes[3] = cv::Mat(4, 4, CV_64FC1);
        auto* p = matrixes[3].ptr<double>();
        sscanf_s(buf,
            "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
            &id, &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6], &p[7], &p[8],
            &p[9], &p[10], &p[11], &p[12], &p[13], &p[14], &p[15]);
        //rt1.setTranslation(tx, ty, tz);
    }
    fgets(buf, 1024, f);
    if (fgets(buf, 1024, f)) {
        matrixes[0] = cv::Mat(3, 3, CV_64FC1);
        auto* p = matrixes[0].ptr<double>();
        sscanf_s(buf, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf",
            &id, &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6], &p[7], &p[8]);
    }
    if (fgets(buf, 1024, f)) {
        matrixes[1] = cv::Mat(3, 3, CV_64FC1);
        auto* p = matrixes[1].ptr<double>();
        sscanf_s(buf, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf",
            &id, &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6], &p[7], &p[8]);
    }
    fclose(f);
    cv::Mat T = (cv::Mat_<double>(3, 1) << matrixes[3].at<double>(0, 3), matrixes[3].at<double>(1, 3), matrixes[3].at<double>(2, 3));
    double T_norm = cv::norm(T);//平移向量模长
    double focal_length = matrixes[0].at<double>(0, 0);//焦距
    cv::Mat newCol = (cv::Mat_<double>(3, 1) << 0, 0, 0);//投影矩阵最后一列扩充
    cv::hconcat(matrixes[0], newCol, matrixes[0]);
    cv::hconcat(matrixes[1], newCol, matrixes[1]);
    matrixes[2] = (matrixes[2](cv::Rect(0, 0, 3, 3))).t();//逆矩阵等于转置矩阵
    matrixes[3] = matrixes[3](cv::Rect(0, 0, 3, 3)).t();
    double P2_last = T_norm * focal_length;
    Pose camR_Pose_relative;
    //判断水平校正或竖直校正及左右目位置关系(正负)
    if (std::abs(T.at<double>(0, 0)) > std::abs(T.at<double>(1, 0)))
    {
        matrixes[1].at<double>(0, 3) = T.at<double>(0, 0) > 0 ? P2_last : -P2_last;
        camR_Pose_relative.x = T.at<double>(0, 0) > 0 ? -T_norm : T_norm;
    }
    else
    {
        matrixes[1].at<double>(1, 3) = T.at<double>(1, 0) > 0 ? P2_last : -P2_last;
        camR_Pose_relative.y = T.at<double>(1, 0) > 0 ? -T_norm : T_norm;
    }
    RectifycameraPBVec[0].fx = matrixes[0].at<double>(0, 0);
    RectifycameraPBVec[0].fy = matrixes[0].at<double>(1, 1);
    RectifycameraPBVec[0].cx = matrixes[0].at<double>(0, 2);
    RectifycameraPBVec[0].cy = matrixes[0].at<double>(1, 2);
    RectifycameraPBVec[1].fx = matrixes[1].at<double>(0, 0);
    RectifycameraPBVec[1].fy = matrixes[1].at<double>(1, 1);
    RectifycameraPBVec[1].cx = matrixes[1].at<double>(0, 2);
    RectifycameraPBVec[1].cy = matrixes[1].at<double>(1, 2);
    RectifycameraPBVec[0].width = RectifycameraPBVec[1].width = width;
    RectifycameraPBVec[0].height = RectifycameraPBVec[1].height = height;

    Rotation leftcamRectify_leftcam_rotation(matrixes[2].ptr<double>(0));
    Point3_<double> leftcamRectify_leftcam_T(0.0, 0.0, 0.0);
    Pose leftcamRectify_leftcam_pose(leftcamRectify_leftcam_rotation, leftcamRectify_leftcam_T);//左目相机极线校正的旋转矩阵

    return std::make_tuple(RectifycameraPBVec, leftcamRectify_leftcam_pose, camR_Pose_relative);
}
}  // namespace rulermvs
#endif