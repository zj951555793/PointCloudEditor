#ifndef _RULERMVS_LINES_MARKERFUSION_LINES_MARKERFUSION_HPP_
#define _RULERMVS_LINES_MARKERFUSION_LINES_MARKERFUSION_HPP_
#include "rulermvs/rgbd.hpp"
#include "rulermvs/pointcloud.hpp"
#include "rulermvs/MarkerExtractor.hpp"
#include "rulermvs/Tracker.hpp"
#include "rulermvs/multilines.hpp"
#include <map>
namespace rulermvs
{
MVS_EXPORT void setLines_MarkerFusionThreadNum(int num);

struct ILines_MarkerFusionPara {
    String verbose_dir = "";  ///< 输出中间数据
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

    // 标志点提点参数
    CicrleConfigs configs;
    // 标志点粗对齐参数
    int min_match_num = 4;  //最小同名点数量
    int min_mappoint_num = 7;  //最小地图点数量，小于此值无法初始化全局地图
    double MAX_PTBIAS_ERROR = 0.18;  //同名点间的均方根差，相邻帧的该值对于下一帧的误匹配筛选具有重要意义
    double site_diff_thres = 1.0;  //两对同名点之间的线段距离差阈值
    double p3d_match_thres = 1.5;  //同名点匹配对在RT作用后两者之间的距离阈值
    double p3d_nearest_thres = 3.0;  //求解RT后合并单帧到global_map时，未参与匹配的同名点在同名点在RT作用后两者间的距离阈值，大于p3d_match_thres
    int max_triMatched_counter = 2;  //三角形匹配成功次数阈值
    bool use_mintree = false;
    slam::FrameMatchMode match_mode =
        slam::FrameMatchMode::match_adjacentFrame_only;  //默认为只与相邻帧对齐
    int match_adjacentframe_nums = 1;  //与相邻帧对齐时，默认只选取一帧，只与全局地图对齐时该参数没有意义
    bool reproject_KeyFrame = false;  //是否将全局地图点重投影到关键帧生成新的帧
    bool savePointer = false;  //不开启共享指针(避免内存爆炸)

    // 标志点精对齐参数
    int iter_Num = 1;  // BA优化迭代次数
    int max_failedframes_num = 5;  //连续失败帧数量，超过该值说明全局地图globalmap存在问题，非标志点模式下，超过该值进行重定位
    int min_stable_globalmapframes = 10;  //全局地图内置稳定帧数量，低于此值表明该地图尚不稳定
    //int BA_min_added_mappoints = 7;  //BA优化要求的最小新增地图点数量
    int BA_min_added_frames = 6;  //BA优化要求的最小新增Frame数量

    //外部控制参数（公共）
    bool consecutive = true;  
    //是否连续扫描模式，该模式下"特征对齐"frames大小是1且当前帧与前一帧对齐失败时会将第一帧替换掉并重新初始化，拍照模式下则不会重复初始化
    //特征对齐下，连续模式时对齐失败若干帧才能重定位，拍照模式时对齐失败立刻重定位
    //标志点对齐模式下，连续若干帧对齐失败时，会清空全局地图并重新初始化（连续模式下），拍照模式下初始化成功后即使多帧连续失败也不会重复初始化

    // 多线激光解码参数
    LineConfigs line_configs;

    bool use_marker_track = true;
    bool input_rectified = false;  //默认输入图片为原始图片
    int width = 1200;  //照片宽度
    int height = 2048;  //照片高度
    int rectify_flags = cv::CALIB_SAME_FOCAL_LENGTH;
    float valid_data_ratio = 0.125;  //深度图中有效数据占比（除以图片尺寸的乘积）最低阈值，低于该值为无效数据
};

/// @brief 获取拼接的结果数据
struct ILines_MarkerFusionResult {
    /// @brief 匹配状态标识
    enum : int {
        Failed = 0, /*参与对齐但返回失败*/
        Init,       /*未参与对齐求解*/
        Succeed,    /*参与对齐且返回成功*/
        Key,        /*关键帧（前提条件当前帧必须要加入）*/
        Relocation, /*重定位成功帧*/
        Reset /*初始化成功（前提条件当前帧必须要加入）*/
    };
    virtual ~ILines_MarkerFusionResult() {}
    virtual int getID() const = 0;
    virtual const void* getUserData() const = 0;
    virtual Pose getPose() const = 0;
    virtual int getState() const = 0;
    virtual Image3f getVmap() const = 0;
    virtual Image3f getNmap() const = 0;
    virtual Image8u getMask() const = 0;
    virtual PointCloud getPointCloud() const = 0;
    virtual RGBPointCloud getRGBPointCloud() const = 0;
    virtual Point3dVec getMarker3Dpoints() const = 0;
    virtual Point3dVec getGlobalmap3Dpoints() const = 0;
    virtual Point3fVec getLine3Dpoints() const = 0;
    virtual std::vector<Point3fVec> getLine3DpointsVec() const = 0;
    virtual std::vector<Pose> getPoseVec() const = 0;
    virtual std::vector<cv::Point3d> getMarkerNormals() const = 0;
    virtual std::vector<double> getMarkerRadiusVec() const = 0;
};

/// @brief 在线拼接类，多帧RGBD在线配准和姿态优化
struct ILines_MarkerFusion {
    using Result    = ILines_MarkerFusionResult;
    using ResultSet = std::vector<const Result*>;
    using Parameter = ILines_MarkerFusionPara;
    using RetFunc   = std::function<void(const Result&)>;
    using InputFunc = std::function<void(RGBImage&, Imagef&)>;

    typedef std::shared_ptr<ILines_MarkerFusion> Ptr;
    /// @brief 创建实例
    static MVS_EXPORT Ptr create(const std::vector<Pose>& camPoseVec,
        const std::vector<CameraPB>& cameraPBVec,
        const std::vector<std::vector<LightPlane>>& rows_planes,
        const ILines_MarkerFusionPara& para, bool is_vio = false,
        int threadNum = -1);
    /// @brief 虚析构函数，占位
    virtual inline ~ILines_MarkerFusion() {}
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
    virtual void proc(const std::vector<cv::Mat>& grayL_vec,
        const std::vector<cv::Mat>& grayR_vec, const std::string& code,
        RetFunc ret, void* userData,
        const std::vector<double>& timestamp_vec = std::vector<double>()) = 0;
    virtual void proc(const std::vector<cv::Mat>& grayL_vec,
        const std::vector<cv::Mat>& grayR_vec, const std::string& code,
        RetFunc ret = 0,
        const std::vector<double>& timestamp_vec = std::vector<double>()) = 0;
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
    /// @brief 合并多线激光点云
    virtual PointCloud fuseLine3DMap() const = 0;
    /// @brief 球面插值后合并多线激光点云
    virtual PointCloud fuseLine3DMapAfterInterpolation() const = 0;
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
    virtual std::pair<bool, Pose> rescan(const RGBImage& rgb,
        const Imagef& depth, Pose& pose, int min_num, int flag) = 0;
    
    virtual void optimize(rulermvs::ProgressBar progress = 0) = 0;  //标志点模式下不可调用
    virtual void optimize_point_clouds(rulermvs::ProgressBar progress = 0) = 0;  //标志点模式下可调用

    virtual void BundleAdjustmentWithLoopClosure(int min_match_num = 4, int iter_Num = 1, rulermvs::ProgressBar progress = 0) = 0;  //标志点模式下专用，用于回环校正及后续的BA优化

    virtual void InterpolatePoseSlerp() = 0;  // 位姿球面插值
    virtual void InterpolatePoseSlerpWithoutTimestamp(
        double FrameInterval, double GroupInterval) = 0;

    virtual void equalizeSpeed() = 0;

    virtual void removeLast(int count) = 0;

    virtual void removeAt(int start, int count) = 0;

    virtual void remove(const std::set<int>& ids) = 0;

    virtual std::pair<int, std::vector<Pose>> getOptRes() const = 0;

    virtual void rescan_set() = 0;
};
}  // namespace rulermvs
#endif