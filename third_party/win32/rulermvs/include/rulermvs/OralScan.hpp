//#pragma once
#ifndef _RULERMVS_MATCH_OralScan_HPP_
#define _RULERMVS_MATCH_OralScan_HPP_
#include "rulermvs/rgbd.hpp"
//#include "rulermvs/plugin.hpp"
#include "rulermvs/pointcloud.hpp"
#include "DBoW3/DBoW3.h"
#include <map>

namespace rulermvs
{
MVS_EXPORT void setThreadNum(int num);

struct IOralScanPara {
    String verbose_dir = "";  ///< 输出中间数据
    
    // 纹理特征参数
    int   key_max_num    = -1;               ///< 特征点最大数量
    int   key_max_iter   = 10000;            ///< 特征匹配最大迭代次数
    float key_max_dist   = 3.0f;             ///< 最大点间距
    float key_max_angle  = (float)MVS_PI_4;  ///< 最大点间法向角度
    int   key_min_match  = 7;                ///< 最小匹配数量
    float key_edge_ratio = .9f;              ///< 边长比值
    bool is_use_rgbsift = false;
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
    // ICP参数
    int   icp_max_iter    = 3;
    float icp_max_dist    = 3.0f;
    float icp_max_angle   = (float)MVS_PI_4;
    float icp_min_overlap = 0.5f;
    // 里程计对应参数
    MatchMode odometry_mode     = MatchMode::Texture;
    int       odometry_max_step = 15;
    int       global_max_step = 15;
    int       odometry_key_step = 7;
    // 回环检测
    int  loop_option        = 2;
    float loop_edge_ratio    = 0.99f;
    float loop_min_overlap   = 0.5f;
    int   loop_geo_min_match = 10;
    int   loop_key_min_match = 10;

    // 重定位对应参数
    int   relocate_option        = 1;
    float relocate_edge_ratio    = 0.95f;//越小越容易匹配上，建议不小于0.9
    float relocate_min_overlap   = 0.5f;//越小越容易匹配上，建议不小于0.3
    int   relocate_geo_min_match = 7;//越小越容易匹配上，建议不小于3
    int   relocate_key_min_match = 7;//越小越容易匹配上，建议不小于3
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
};

/// @brief 获取拼接的结果数据
struct IOralScanResult {
    /// @brief 匹配状态标识
    enum : int { Failed = 0, Init, Succeed, Key, Relocation,Reset };
    virtual ~IOralScanResult() {}
    virtual int           getID() const            = 0;
    virtual Pose          getPose() const          = 0;
    virtual int           getState() const         = 0;
    virtual Image3f       getVmap() const          = 0;
    virtual Image3f       getNmap() const          = 0;
    virtual Image8u       getMask() const          = 0;
    virtual PointCloud    getPointCloud() const    = 0;
    virtual RGBPointCloud getRGBPointCloud() const = 0;
    
};


/// @brief 在线拼接类,多帧RGBD在线配准和姿态优化
struct IOralScan {
    using Frame     = RGBDImage;
    using Result    = IOralScanResult;
    using ResultSet = std::vector<const Result*>;
    using Parameter = IOralScanPara;
    using RetFunc   = std::function<void(const Result&)>;
    using InputFunc = std::function<void(RGBImage&, Imagef&)>;
    using FuseMapCallback = std::function<void(int, PointCloud&)>;

    typedef std::shared_ptr<IOralScan> Ptr;
    /// @brief 创建实例
    static inline Ptr create();
    
   
    /// @brief 虚析构函数，占位
    virtual inline ~IOralScan() {}
    /// @brief 初始化
    /// @param RGBD相机内参
    /// @param para 拼接参数
    /// @return 返回是否初始化成功
    virtual bool init(const CameraP&, const IOralScanPara& para,
        const DBoW3::Vocabulary& dbow,
        const DBoW3::Database& db, bool is_vio) = 0;
    /// @brief 暂停
    virtual void stop() = 0;
    virtual bool is_finish() = 0;
    virtual std::pair<int, std::vector<Pose>> getOptRes() const = 0;
    /// @brief 重置
    virtual void reset(bool is_vio) = 0;
    virtual void rescan_set()       = 0;
    /// @brief 等待已有任务运行完成;
    virtual void waiting() const = 0;
    /// @brief 返回相机参数引用
    virtual CameraP& camera() = 0;
    /// @brief 返回里程计参数引用
    virtual Parameter& param() = 0;
    /// @brief 返回匹配结果
    /// @return 结果数据集
    virtual ResultSet results() const = 0;
    /// @brief 回调添加深度图和纹理图,用于结构光解码数据导入
    virtual void proc(InputFunc in, RetFunc ret) = 0;
    /// @brief 增量处理RGBD数据帧
    virtual void proc(const Frame& frame, RetFunc ret = 0) = 0;
    /// @brief 增量处理RGBD数据帧,等待该帧匹配完成并返回匹配结果;
    int proc(const RGBImage& rgb, const Imagef& depth, Pose& pose)
    {
        int ret;
        proc(RGBDImage(rgb, depth), [&ret, &pose](const Result& result) {
            ret = result.getState(), pose = result.getPose();
        });
        waiting();
        return ret;
    }
    virtual void load(const RGBImage& rgb, const Imagef& depth, Pose& pose,int index) = 0;
    virtual void load(int fuse_step)                                     = 0;
    /// @brief 合并地图点
    virtual PointCloud fuseMap() const = 0;
    virtual PointCloud fuseMap(const FuseMapCallback& callback) const = 0;
    virtual RGBPointCloud fuseRGBMap() const = 0;
    /// @brief 姿态优化
    /// @param progress 进度条
    virtual std::pair<bool, Pose> rescan(const RGBImage& rgb, const Imagef& depth, Pose& pose, int min_num,int flag) = 0;
    
    virtual void optimize(ProgressBar progress = 0) = 0;
    virtual void optimize_point_clouds(ProgressBar progress = 0) = 0;
    /*virtual void compute_feature_map(std::vector<Point2fVec>& feature_map_uvs,
        std::vector<Point3fVec>& feature_map_points,
        std::vector<Point3fVec>&  feature_map_normals,
        std::vector<Desc128fVec>& feature_map_desc,
        std::vector<Pose>& key_poses, std::vector<RGBImage>& key_images,
        std::vector<Image3f>& key_vmaps, std::vector<Image3f>& key_nmaps) = 0;*/

    virtual void equalizeSpeed() = 0;

    virtual void removeLast(int count) = 0;

    virtual void removeAt(int start, int count) = 0;

    virtual void remove(const std::set<int>& ids) = 0;

    virtual void updateFrame(int id, const Image8u& mask) = 0;
};

/// @brief 创建RGBD拼接方法类
MVS_EXPORT IOralScan* createRGBDFusion();
template <> struct ObjectCreator_<IOralScan> {
    static inline decltype(auto) create() { return createRGBDFusion(); }
};
//template <> struct PluginProperty_<IRGBDFusion> {
//    static inline std::string name() { return "RGBDFusion"; }
//};

typename IOralScan::Ptr IOralScan::create()
{
    /*auto obj = PluginCreate<IRGBDFusion>();
    if (obj != nullptr) {
        MVS_ILOG << "Create RGBDFusion Instance From Plugin DLL.";
        return obj;
    }
    MVS_ILOG << "Create RGBDFusion Instance From Inside Module.";*/
    return createObject<IOralScan>();
}

MVS_EXPORT void oralscan_merge_points_to_cloud(std::vector<GRAYPointCloud>& _clouds,
    std::vector<Pose>& _poses, int _num, RGBPointCloud& _cloud);

MVS_EXPORT void oralscan_merge_points_to_cloud(
    std::vector<RGBPointCloud*>& _clouds,
    std::vector<Pose>& _poses, int _num, RGBPointCloud& _cloud);

MVS_EXPORT void oralscan_merge_frame_to_cloud(std::vector<Imagef>& depths,
    std::vector<RGBImage>& rgbs,   CameraP& cam,
    std::vector<Pose>& poses, RGBPointCloud& cloud);

MVS_EXPORT void oralscan_optimizeMultiGroupClouds(
     std::vector<RGBPointCloud*>& _multi_clouds,
     std::vector<Pose>& _multi_poses,
    int _cloud_num, ICPMethod _method = ICPMethod::Color,
    int _max_iter = 100, double _min_overlap = 0.001,
    double _max_distance = 1.0,
    double _min_cos_angle = std::cos(3.14159265358979 / 12),
    double _theta = 0.000001f,
    int _group_number = 80, float voxel_leaf = 1.0f);

MVS_EXPORT void oralscan_optimizeMultiGroupRGBDFrames(
    std::vector<Imagef>& _multi_depths,
    std::vector<RGBImage>& _multi_rgbs,
    std::vector<Pose>& _multi_poses,  CameraP cam,
    ICPMethod _method = ICPMethod::Color, int _max_iter = 100,
    double _min_overlap = 0.001, double _max_distance = 1.0,
    double _min_cos_angle = std::cos(3.14159265358979 / 12),
    double _theta = 0.000001f, int _group_number = 80,
    float voxel_leaf = 1.0f);

}  // namespace rulermvs
#endif  // _RULERMVS_MATCH_RGBDFUSION_H_