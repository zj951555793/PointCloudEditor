#ifndef _RULERMVS_MATCH_MATCH_HPP_
#define _RULERMVS_MATCH_MATCH_HPP_
#include <random>
#include <numeric>
#include "rulermvs/rgbd.hpp"
#include "rulermvs/matchinfo.hpp"
#include "rulermvs/posegraph.hpp"
#include "rulermvs/pointcloud.hpp"
namespace rulermvs
{
template <typename Float> struct IRGBDImage_;
template <typename Float> struct IPointCloud_;

/// @brief 检测RGBD帧的SIFT特征点
/// @param gray 灰度图
/// @param depth 深度图
/// @param mask 掩模
/// @param camera 相机内参
/// @param keypts 特征点三维坐标
/// @param keynls 特征点法向
/// @param desc 特征描述子
/// @param nfeatures 特征点最大数量
/// @return 返回检测特征点数量.
MVS_EXPORT int detectAndComputeSIFT(const Image8u& gray, const Imagef& depth,
    const Image8u& mask, const CameraP& camera, Point3fVec& keypts,
    Point3fVec& keynls, Desc128fVec& desc, int nfeatures = 0,
    bool is_use_rgbsift = false);

MVS_EXPORT int detectAndComputeSIFTShow(const Image8u& gray, const Imagef& depth,
    const Image8u& mask, const CameraP& camera, Point3fVec& keypts, Point3fVec& keynls, Desc128fVec& desc,
    Desc128fVec& desc_all, Point2fVec& keyuvs,
    int nfeatures = 0,bool is_use_rgbsift =false);

/// @brief 检测RGBD帧的SIFT特征点
/// @tparam Pixel 像素类型
/// @tparam Float 三维点的浮点类型
/// @param rgbd IRGBD接口的实例数据
/// @param camera 相机内参
/// @param keypts 特征点三维坐标
/// @param keynls 特征点法向
/// @param keydesc 特征描述子
/// @param nfeatures 特征点最大数量
/// @return 返回检测特征点数量.
template <typename Float> static inline int detectAndComputeSIFT(
    const IRGBDImage& rgbd, const CameraP& camera, Point3Vec<Float>& keypts,
    Point3Vec<Float>& keynls, Desc128fVec& keydesc, int nfeatures = 0)
{
    Image8u gray(rgbd.colorImage());
    Imagef  depth(rgbd.rangeImage());
    return detectAndComputeSIFT(gray, depth, depth2Mask(depth), camera, keypts,
        keynls, keydesc, nfeatures);
}

template <typename Float> static inline int detectAndComputeSIFTShow(
    const IRGBDImage& rgbd, const CameraP& camera, Point3Vec<Float>& keypts,
    Point3Vec<Float>& keynls, 
    Desc128fVec& keydesc, Desc128fVec& keydesc_all,
    Point2Vec<Float>& keyuvs, int nfeatures = 0)
{
    Image8u gray(rgbd.colorImage());
    Imagef  depth(rgbd.rangeImage());
    return detectAndComputeSIFTShow(gray, depth, depth2Mask(depth), camera,
        keypts, keynls, keydesc, keydesc_all, keyuvs, nfeatures);
}
/// @brief 检测FPFH几何特征点
/// @param index kdtree
/// @param keypts 特征点三维坐标
/// @param keynls 特征点空间法向
/// @param desc 特征点特征描述子
/// @param radius 关键点提取半径
/// @param gamma 关键点筛选阈值
/// @param nms 极大值抑制范围
/// @param fpfh fpfh搜索半径
/// @param pfh pfh搜索半径
MVS_EXPORT void detectAndComputeFPFH(const CloudKDTree<float>& index,
    Point3fVec& keypts, Point3fVec& keynls, Desc33fVec& desc,
    float radius = .0f, float gamma = .1f, float nms = .0f, float fpfh = .0f,
    float pfh = .0f);

/// @brief 提取有序三维点的FPFH几何特征
/// @param vmap 有序三维点云
/// @param nmap 有序三维法向
/// @param mask 掩模版
/// @param keypts 特征点三维坐标
/// @param keynls 特征点三维法向
/// @param desc 特征点对应的特征描述子
/// @param k 特征点搜索范围
/// @param gamma 特征点判定阈值,一般为0.1
/// @param nms 极大值抑制范围
/// @param fpfh FPFH计算范围
/// @param pfh PFH特征计算范围,通过测试一般来说PFH计算范围要大于FPFH统计范围。
/// @param maxnum 特征点截取数量，-1表示所有特征点全部保留。
MVS_EXPORT void detectAndComputeFPFH(const Image3f& vmap, const Image3f& nmap,
    const Image8u& mask, Point3fVec& keypts, Point3fVec& keynls,
    Desc33fVec& desc, int k = 0, double gamma = 1e-1, int nms = 0, int fpfh = 0,
    int pfh = 0, int maxnum = -1);

/// @brief 提取点云接口类的FPFH几何特征
/// @param cloud 点云接口类
/// @param keypts 特征点三维坐标
/// @param keynls 特征点三维法向
/// @param desc 特征点对应的特征描述子
/// @param radius 提取特征点的搜索范围
/// @param gamma 特征点判定阈值,一般为0.1
/// @param nms 极大值抑制的搜索范围
/// @param fpfh FPFH特征计算范围
/// @param pfh PFH特征计算范围
static inline void detectAndComputeFPFH(const IPointCloud& cloud,
    Point3fVec& keypts, Point3fVec& keynls, Desc33fVec& desc,
    float radius = -0.0f, float gamma = 0.1f, float nms = -0.0f,
    float fpfh = -0.0f, float pfh = -0.0f)
{
    assert(cloud.getPointNum() && cloud.getPointData());
    const auto  ptnum   = cloud.getPointNum();
    const auto* points  = cloud.getPointData();
    const auto* normals = cloud.getNormalData();
    Point3fVec  normal_vec;
    if (normals == nullptr) {
        normal_vec.resize(ptnum);
        normals = &normal_vec[0];
    }
    const CloudAdaptor_<float> pc2kd(points, normals, ptnum);
    CloudKDTree<float> index(3, pc2kd, KDTreeSingleIndexAdaptorParams(10));
    if (!normal_vec.empty()) computeNormalsInKnn(index, &normal_vec[0], 16);
    detectAndComputeFPFH(
        index, keypts, keynls, desc, radius, gamma, nms, fpfh, pfh);
}

/// @brief 提取深度图的FPFH几何特征
/// @param depth 深度图
/// @param camera 相机内参
/// @param keypts 特征点三维坐标
/// @param keynls 特征点三维法向
/// @param desc 特征点对应的特征描述子
/// @param k 特征点搜索范围
/// @param gamma 特征点判定阈值,一般为0.1
/// @param nms 极大值抑制范围
/// @param fpfh FPFH计算范围
/// @param pfh PFH特征计算范围,通过测试一般来说PFH计算范围要大于FPFH统计范围。
/// @param maxnum 特征点截取数量，-1表示所有特征点全部保留。
static inline void detectAndComputeFPFH(const Imagef& depth,
    const CameraP& camera, Point3fVec& keypts, Point3fVec& keynls,
    Desc33fVec& desc, int k = 5, double gamma = 1e-1, int nms = -1,
    int fpfh = -1, int pfh = -1, int maxnum = -1)
{
    Image3f vmap, nmap;
    depth2VmapAndNmap(depth, camera, vmap, nmap);
    Image8u mask;
    vmapAndNmapToMask(vmap, nmap, mask);
    detectAndComputeFPFH(
        vmap, nmap, mask, keypts, keynls, desc, k, gamma, nms, fpfh, pfh, maxnum);
}

/// @brief 绝对定向,{s*(R*[x,y,z]'+T)}
/// @tparam Float 浮点类型
/// @param src 源三维点
/// @param dst 目标三维点
/// @param pt_num 三维点数量
/// @param rt 输出RT
/// @param scale 是否计算尺度,如果为False则返回值为1.0.
/// @param mode 0表示使用四元素，1表示使用SVD分解求解.
/// @return 返回尺度信息>0，如果匹配失败,如三点共线的情况会返回零值.
MVS_EXPORT double absoluteOrientation(const Point3f* src, const Point3f* dst,
    size_t pt_num, Pose& rt, bool scale = false, int mode = 0);

/// @brief 基于采样一致性的匹配搜索
/// @param src_pts 源三维点
/// @param src_nls 源法线
/// @param dst_pts 目标三维点
/// @param dst_nls 目标法线
/// @param pt_num 三维点数量
/// @param rt 输出RT
/// @param max_iter 最大迭代次数
/// @param edge_ratio 三角形边长阈值
/// @param max_dist 匹配点的最大距离
/// @param max_angle 匹配点的最大法线夹角
/// @return 返回匹配点的数量.
MVS_EXPORT int matchPointsRansac(const Point3f* src_pts, const Point3f* src_nls,
    const Point3f* dst_pts, const Point3f* dst_nls, size_t pt_num, Pose& rt,
    int max_iter, double edge_ratio, double max_dist,
    double max_angle = MVS_PI_4);


MVS_EXPORT int matchPointsRansacShow(const Point3f* src_pts, const Point3f* src_nls,
    const Point3f* dst_pts, const Point3f* dst_nls, std::vector<uchar>& flags,
    size_t pt_num, Pose& rt,
    int max_iter, double edge_ratio, double max_dist,
    double max_angle = MVS_PI_4);
/// @brief 基于采样一致性的匹配搜索
/// @param src_pts 源三维点
/// @param src_nls 源法线
/// @param dst_pts 目标三维点
/// @param dst_nls 目标法线
/// @param rt 输出RT
/// @param max_iter 最大迭代次数
/// @param edge_ratio 三角形边长阈值
/// @param max_dist 匹配点的最大距离
/// @param max_angle 匹配点的最大法线夹角
/// @return 返回匹配点的数量.
static inline int matchPointsRansac(const Point3fVec& src_pts,
    const Point3fVec& src_nls, const Point3fVec& dst_pts,
    const Point3fVec& dst_nls, Pose& rt, int max_iter, double edge_ratio,
    double max_dist, double max_angle = MVS_PI_4)
{
    assert(!src_pts.empty() && src_nls.size() == src_pts.size() &&
           dst_pts.size() == src_pts.size() &&
           dst_nls.size() == src_pts.size());
    return matchPointsRansac(src_pts.data(), src_nls.data(), dst_pts.data(),
        dst_nls.data(), src_pts.size(), rt, max_iter, edge_ratio, max_dist,
        max_angle);
}

static inline int matchPointsRansacShow(const Point3fVec& src_pts,
    const Point3fVec& src_nls, const Point3fVec& dst_pts,
    const Point3fVec& dst_nls, std::vector<uchar>&flags, Pose& rt, int max_iter, double edge_ratio,
    double max_dist, double max_angle = MVS_PI_4)
{
    assert(!src_pts.empty() && src_nls.size() == src_pts.size() &&
           dst_pts.size() == src_pts.size() &&
           dst_nls.size() == src_pts.size());
    return matchPointsRansacShow(src_pts.data(), src_nls.data(), dst_pts.data(),
        dst_nls.data(), flags, src_pts.size(), rt, max_iter, edge_ratio, max_dist, max_angle);
}
/// @brief 基于采样一致性的匹配搜索,无法线约束.
/// @param src 源三维点
/// @param dst 目标三维点
/// @param rt 输出RT
/// @param max_iter 最大迭代次数
/// @param edge_ratio 三角形边长阈值
/// @param max_dist 匹配点的最大距离
/// @return 返回匹配点对的数量.
static inline int matchPointsRansac(const Point3fVec& src,
    const Point3fVec& dst, Pose& rt, int iter_num, double edge_ratio,
    double max_dist)
{
    assert(!src.empty() && src.size() == dst.size());
    return matchPointsRansac(src.data(), nullptr, dst.data(), nullptr,
        src.size(), rt, iter_num, edge_ratio, max_dist, 0);
}

/// @brief 匹配描述子
/// @param src_desc 源特征向量起始指针
/// @param src_num 源特征向量数量
/// @param dst_desc 目标特征向量起始指针
/// @param dst_num 目标特征向量数量
/// @param desc_dim 特征向量维度
/// @param matchs 匹配结果数组
/// @return 返回匹配数量
extern "C" MVS_EXPORT int matchDescriptors(const float* src_desc,
    size_t src_num, const float* dst_desc, size_t dst_num, size_t desc_dim,
    std::vector<DMatch>& matchs);

/// @brief 匹配特征向量,对上述接口的封装.
/// @tparam N 特征向量维度
/// @param src_desc 源特征向量
/// @param dst_desc 目标特征向量
/// @param matchs 匹配结果数组
/// @return 返回匹配数量
template <int N = 33> static inline int matchDescriptors(
    const std::vector<Scalar_<float, N>>& src_desc,
    const std::vector<Scalar_<float, N>>& dst_desc, std::vector<DMatch>& matchs)
{
    if (src_desc.empty() || dst_desc.empty()) return -1;
    return matchDescriptors(src_desc[0].s, src_desc.size(), dst_desc[0].s,
        dst_desc.size(), N, matchs);
}

/// @brief 基于3d特征匹配
/// @param src_pts 源三维特征点坐标起始地址
/// @param src_nls 源三维特征点法向起始地址
/// @param src_desc 源特征向量起始地址
/// @param src_num 源三维特征点数量
/// @param dst_pts 目标特征点三维坐标起始地址
/// @param dst_nls 目标特征点三维法向起始地址
/// @param dst_desc 目标特征向量起始地址
/// @param dst_num 目标三维特征点数量
/// @param desc_dims 特征向量维度
/// @param rt 相对姿态
/// @param imax_iter 最大迭代次数
/// @param edge_ratio 三角形边长的筛选阈值
/// @param max_dist 对应点的最大欧式距离
/// @param max_angle 对应点法向之间的最大夹角
/// @param bCrossCheck 暴力匹配时是否进行交叉验证
/// @param bMultiThread 暴力匹配时是否开启多线程
/// @return 返回匹配点数量
extern "C" MVS_EXPORT int matchFeaturePoints(const Point3f* src_pts,
    const Point3f* src_nls, const float* src_desc, size_t src_num,
    const Point3f* dst_pts, const Point3f* dst_nls, const float* dst_desc,
    size_t dst_num, size_t desc_dims, Pose& rt, int max_iter = 100000,
    double edge_ratio = 0.95, double max_dist = 3.0,
    double max_angle = MVS_PI_4, bool bCrossCheck = true, bool bMultiThread = false);

extern "C" MVS_EXPORT int matchFeaturePointsShow(const Point3f* src_pts,
    const Point3f* src_nls, const float* src_desc, size_t src_num,
    const Point3f* dst_pts, const Point3f* dst_nls, const float* dst_desc,
    std::vector<DMatch>& matchs, std::vector<uchar>& flags,
    size_t dst_num, size_t desc_dims, Pose& rt, int max_iter = 100000,
    double edge_ratio = 0.95, double max_dist = 3.0,
    double max_angle = MVS_PI_4, bool bCrossCheck = true, bool bMultiThread = false);

/// @brief 基于3d特征匹配,对上面接口的封装.
/// @tparam N 特征向量维度
/// @param src_pts 源三维特征点坐标起始地址
/// @param src_nls 源三维特征点法向起始地址
/// @param src_desc 源特征向量起始地址
/// @param dst_pts 目标特征点三维坐标起始地址
/// @param dst_nls 目标特征点三维法向起始地址
/// @param dst_desc 目标特征向量起始地址
/// @param desc_dims 特征向量维度
/// @param rt 相对姿态
/// @param imax_iter 最大迭代次数
/// @param edge_ratio 三角形边长的筛选阈值
/// @param max_dist 对应点的最大欧式距离
/// @param max_angle 对应点法向之间的最大夹角
/// @param bCrossCheck 暴力匹配时是否进行交叉验证
/// @param bMultiThread 暴力匹配时是否开启多线程
/// @return 返回匹配点数量
template <int N = 33> static inline int matchFeaturePoints(
    const Point3fVec& src_pts, const Point3fVec& src_nls,
    const std::vector<Scalar_<float, N>>& src_desc, const Point3fVec& dst_pts,
    const Point3fVec& dst_nls, const std::vector<Scalar_<float, N>>& dst_desc,
    Pose& rt, int max_iter = 100000, double edge_ratio = 0.95,
    double max_dist = 3.0, double max_angle = MVS_PI_4, bool bCrossCheck = true, bool bMultiThread = false)
{
    if (src_pts.empty() || src_pts.size() != src_desc.size() ||
        dst_pts.empty() || dst_desc.size() != dst_pts.size())
        return -1;
    return matchFeaturePoints(&src_pts[0],
        src_pts.size() == src_nls.size() ? &src_nls[0] : nullptr, src_desc[0].s,
        src_pts.size(), &dst_pts[0],
        dst_pts.size() == dst_nls.size() ? &dst_nls[0] : nullptr, dst_desc[0].s,
        dst_pts.size(), N, rt, max_iter, edge_ratio, max_dist, max_angle, bCrossCheck, bMultiThread);
}
template <int N = 33> static inline int matchFeaturePointsShow(
    const Point3fVec& src_pts, const Point3fVec& src_nls,
    const std::vector<Scalar_<float, N>>& src_desc, const Point3fVec& dst_pts,
    const Point3fVec& dst_nls, const std::vector<Scalar_<float, N>>& dst_desc,
    std::vector<DMatch>& matchs, std::vector<uchar>& flags,
    Pose& rt, int max_iter = 100000, double edge_ratio = 0.95,
    double max_dist = 3.0, double max_angle = MVS_PI_4, bool bCrossCheck = true, bool bMultiThread = false)
{
    if (src_pts.empty() || src_pts.size() != src_desc.size() ||
        dst_pts.empty() || dst_desc.size() != dst_pts.size())
        return -1;
    return matchFeaturePointsShow(&src_pts[0],
        src_pts.size() == src_nls.size() ? &src_nls[0] : nullptr, src_desc[0].s,
        src_pts.size(), &dst_pts[0],
        dst_pts.size() == dst_nls.size() ? &dst_nls[0] : nullptr, dst_desc[0].s,
        matchs, flags, dst_pts.size(), N, rt, max_iter, edge_ratio, max_dist, max_angle, bCrossCheck, bMultiThread);
}

/// @brief 基于FPFH特征匹配点云数据
/// @param src 源点云
/// @param dst 目标点云
/// @param rt 外参,源->目标.
/// @param max_iter 最大迭代次数
/// @param max_dist 最大点间距离 checkDist(cloud,9)
/// @return 返回匹配点对数量
MVS_EXPORT int matchFPFH(const IPointCloud& src, const IPointCloud& dst,
    Pose& rt, int max_iter = 100000, double max_dist = 1e1,
    double edge_ratio = 0.95);

/// @brief RGBD数据帧间配准参数
struct IRGBDMatchPara {
    IRGBDMatchPara() {}
    IRGBDMatchPara(Size sz) : depth_size(sz), image_size(sz) {}
    IRGBDMatchPara(Size sz, float max_dist, float max_angle)
        : depth_size(sz)
        , image_size(sz)
        , icp_max_dist(max_dist)
        , icp_max_angle(max_angle)
    {}
    MatchMode match_mode = MatchMode::Both;  ///< 匹配类型
    Size depth_size = {160, 128};            ///< 深度图尺寸
    Size image_size = {320, 256};            ///< 纹理图像尺寸
    int icp_max_iter = 10;                   ///< 帧间ICP最大迭代次数
    double icp_max_dist = 3.0f;              ///< 帧间ICP对应的最大距离
    double icp_max_angle = MVS_PI_4;  ///< 帧间ICP对应点法向之间的最大夹角
    double icp_rgb_theta = 0.000001f;  ///< 帧间ICP的纹理权重
    double icp_min_overlap = 0.3f;     ///< 帧间ICP的最大重叠率
    int geo_max_iter = 10000;       ///< 几何特征匹配的最大迭代次数
    double geo_edge_ratio = 0.99f;  ///< 几何特征匹配的边长阈值
    double geo_max_dist = 3.0f;  ///< 几何特征匹配的最大对应点之间的距离
    double geo_max_angle = MVS_PI_4;  ///< 几何特征匹配时的最大法向夹角
    int geo_min_match = 10;           ///< 最小有效匹配特征点
    int key_max_iter = 10000;       ///< 纹理特征匹配的最大迭代次数
    int key_feature_num = 0;        ///< 纹理特征点的最大数量
    double key_edge_ratio = 0.99f;  ///< 纹理特征匹配时的边长阈值
    double key_max_dist = 3.0f;  ///< 纹理特征匹配时的最大对应点之间的距离
    double key_max_angle = MVS_PI_4;  ///< 纹理特征匹配时的最大对应点之间的夹角
    int key_min_match = 10;  ///< 纹理特征匹配时的最小匹配数量
    int optimize_max_iter = 100;       ///< 稠密优化的最大迭代次数
    int posegraph_max_iter = 1000;     ///< 姿态图优化的最大迭代次数
    double posegraph_max_dist = 3.0f;  ///< 姿态图优化的距离阈值

    // 几何配准参数
    int geo_nms = 3;          ///< 几何特征点极大值抑制范围
    int geo_pfh = 9;          ///< 几何特征点PFH范围
    int geo_fpfh = 7;         ///< 几何特征点FPFH范围
    float geo_gamma = 0.02f;  ///< 检测特征点的各向最小比值
    int geo_diam = 5;         ///< 特征点提取直径
    int geo_maxnum = -1;      ///< 特征点截取数量，-1表示全部保留
};

/// @brief 基于FPFH特征点匹配RGBD帧
/// @param src 源RGBD数据
/// @param dst 目标RGBD数据
/// @param cam 相机内参
/// @param rt 相机外参
/// @param max_iter 最大迭代次数
/// @param bCrossCheck 暴力匹配时是否进行交叉验证
/// @param bMultiThread 暴力匹配时是否开启多线程
/// @return 返回匹配点对数量
MVS_EXPORT int matchFPFH(const IRGBDImage& src, const IRGBDImage& dst,
    const CameraP& cam, Pose& rt, const IRGBDMatchPara& param,
    int max_iter = 100000, double max_dist = 1e1,
    double edge_ratio = 0.95, bool bCrossCheck = true, bool bMultiThread = false);

/// @brief 基于特征点的几何配准,首先尝试用纹理特征,如果没有则用几何特征.
/// @param src 源RGBD数据
/// @param dst 目标RGBD数据
/// @param cam 相机内参
/// @param rt 相机外参
/// @param max_iter 最大迭代次数
/// @param max_dist 最小点间距
/// @param edge_ratio 匹配时边长比阈值
/// @param nfeatures 特征点最大数量
/// @param bCrossCheck 暴力匹配时是否进行交叉验证
/// @return 返回匹配数量,如果未匹配上则返回-1
MVS_EXPORT int matchSIFT(const IRGBDImage& src, const IRGBDImage& dst,
    const CameraP& cam, Pose& rt, int max_iter = 100000, double max_dist = 1e1,
    double edge_ratio = 0.95, int nfeatures = 0, bool bCrossCheck = true);

/// @brief 点到点ICP
/// @tparam Float 浮点类型
/// @param src_vmap 有序点云
/// @param dst_vmap 有序点云
/// @param camera 相机内参
/// @param rt 初始姿态,迭代更新.
/// @param max_iter 最大迭代次数
/// @param min_overlap 最小重叠率
/// @param max_dist 邻近点的最大距离
/// @param optimizer 优化方法,0为LM,其它为GaussNewton.
/// @return 返回是否成功迭代
MVS_EXPORT bool matchICP(const Image3f& src_vmap, const Image3f& dst_vmap,
    const CameraP& cam, Pose& rt, int max_iter = 10, double min_overlap = 1e-1,
    double max_dist = 1e1, int optimizer = 0);

/// @brief 点到面ICP
/// @tparam Float 浮点类型
/// @param src_vmap 有序点云
/// @param src_nmap 有序点云法向
/// @param dst_vmap 有序点云
/// @param dst_nmap 有序点云法向
/// @param camera 相机内参
/// @param rt 初始姿态,迭代更新.
/// @param max_iter 最大迭代次数
/// @param min_overlap 最小重叠率
/// @param max_dist 邻近点的最大距离
/// @param max_angle 邻近点最大向量夹角
/// @param optimizer 优化方法,0为LM,其它为GaussNewton.
/// @return 返回是否成功迭代
MVS_EXPORT bool matchICP(const Image3f& src_vmap, const Image3f& src_nmap,
    const Image3f& dst_vmap, const Image3f& dst_nmap, const CameraP& cam,
    Pose& rt, int max_iter = 10, double min_overlap = 1e-1,
    double max_dist = 1e1, double max_angle = MVS_PI_4, int optimizer = 0);

/// @brief 结合纹理的ICP配准
/// @param src_vmap 源有序三维点
/// @param src_nmap 源有序三维法向
/// @param src_pix 源纹理图像
/// @param dst_vmap 目标有序三维点
/// @param dst_nmap 目标有序三维法向
/// @param dst_pix 目标纹理图像
/// @param dst_dx 目标纹理的X方向梯度
/// @param dst_dy 目标纹理的Y方向梯度
/// @param camera 相机内参，无畸变
/// @param rt 相对姿态
/// @param max_iter 高斯-牛顿优化,最大迭代次数
/// @param min_overlap 最小重叠率
/// @param max_dist 点对的最大距离
/// @param max_angle 点对的最大法向角度
/// @param theta 纹理残差的权重
/// @param optimizer 优化方法,0为LM,其它为GaussNewton.
/// @return 返回是否成功迭代
MVS_EXPORT bool matchICP(const Image3f& src_vmap, const Image3f& src_nmap,
    const Image8u& src_pix, const Image3f& dst_vmap, const Image3f& dst_nmap,
    const Image8u& dst_pix, const Imagef& dst_dx, const Imagef& dst_dy,
    const CameraP& camera, Pose& rt, int max_iter = 10,
    double min_overlap = 1e-1, double max_dist = 1e1,
    double max_angle = MVS_PI_2, double theta = 1e-6, int optimizer = 0);

/// @brief 深度图帧间ICP配准
/// @param src_depth 源深度图
/// @param dst_depth 目标深度图
/// @param camera 相机内参
/// @param rt 相对姿态
/// @param max_iter 最大迭代次数
/// @param min_overlap 最小重叠率【一般大于0.3】
/// @param max_dist 最大对应点的欧式距离
/// @param max_angle 最大对应点法向直接的夹角
/// @return 返回是否成功迭代
MVS_EXPORT bool matchICP(const Imagef& src, const Imagef& dst,
    const CameraP& cam, Pose& rt, int max_iter, double min_overlap = 0.1f,
    double max_dist = 1e1, double max_angle = MVS_PI_4, int optimizer = 0);

/// @brief RGBD帧间ICP配准
/// @param src 源RGBD帧接口类
/// @param dst 目标RGBD帧接口类
/// @param camera 相机内参
/// @param rt 相对位姿,src->dst.
/// @param method
/// ICP类型，其中Point-to-Point被禁止，原因是此种数据的对应点搜索不相匹配.
/// @param max_iter 最大迭代次数
/// @param min_overlap 最小重叠率
/// @param max_dist 对应点之间的最大欧式距离
/// @param max_angle 对应点之间的最大法向夹角
/// @param theta 纹理残差权重，只有当Point-to-Normal-With-Color时才有用.
/// @param optimizer 优化方法,0为LM,其它为GaussNewton.
/// @return 返回是否成功迭代
MVS_EXPORT bool matchICP(const IRGBDImage& src, const IRGBDImage& dst,
    const CameraP& camera, Pose& rt, ICPMethod method = ICPMethod::Normal,
    int max_iter = 10, double min_overlap = 1e-1, double max_dist = 1e1,
    double max_angle = MVS_PI_4, double theta = 1e-6, int optimizer = 0);

/// @brief 多帧深度图ICP优化
/// @param vmaps 有序点云数组
/// @param nmaps 有序点云法向数组
/// @param masks 深度图掩模数组,若为NULL,则以深度值大于零为约束.
/// @param grays 有序点云亮度数组,若为NULL,则不采用纹理约束.
/// @param dxs 有序点云x方向梯度数组,若为NULL,则不采用纹理约束.
/// @param dys 有序点云y方向梯度数组,若为NULL,则不采用纹理约束.
/// @param cam 相机内参
/// @param rts 相机外参数组
/// @param sz 有序点云数量
/// @param max_iter 最大迭代次数
/// @param max_dist 最大对应点间距
/// @param max_angle 最大对应点法向夹角
/// @param min_overlap 帧间最小重叠率
/// @param theta 纹理权重
/// @param optimizer 优化方法,0为LM,其它为GaussNewton.
/// @return 返回是否成功迭代
MVS_EXPORT bool matchICP(const Image3f* vmaps, const Image3f* nmaps,
    const Image8u* grays, const Imagef* dxs, const Imagef* dys, Pose* rts,
    size_t vmap_num, const CameraP& cam, int max_iter = 100,
    double max_dist = 1e1, double max_angle = MVS_PI_4,
    double min_overlap = 1e-1, double theta = 1e-6, int optimizer = 0);

/// @brief 多帧深度图配准
/// @param depths 深度图数组
/// @param grays 纹理图数组
/// @param masks 掩模数组
/// @param rts 姿态数组,从当前帧旋转到空间坐标系中.
/// @param depth_num 深度图数量
/// @param cam 相机内参
/// @param max_iter 最大迭代次数
/// @param max_dist 最大对应点之间的距离
/// @param max_angle 最大对应点之间的法向夹角
/// @param min_overlap 最小重叠率
/// @param theta 纹理权重
/// @param optimizer 优化方法,0为LM,其它为GaussNewton.
/// @return 返回是否正常迭代
MVS_EXPORT bool matchICP(const Imagef* depths, const Image8u* grays,
    const Image8u* masks, Pose* rts, size_t depth_num, const CameraP& cam,
    int max_iter = 100, double max_dist = 1e1, double max_angle = MVS_PI_4,
    double min_overlap = 1e-1, double theta = 1e-6, int optimizer = 0);

/// @brief 多帧RGBD类型数据配准
/// @tparam RGBDFrame RGBD帧
/// @param rgbds RGBD数组
/// @param rts 姿态数组
/// @param rgbd_num RGBD数量
/// @param cam 相机内参
/// @param max_iter 最大迭代次数
/// @param max_dist 最大对应点之间的距离
/// @param max_angle 最大对应点之间的法向夹角
/// @param min_overlap 最小重叠率
/// @param theta 纹理权重
/// @param optimizer 优化方法,0为LM,其它为GaussNewton.
/// @return 返回是否成功迭代
template <typename RGBDFrame, class = typename std::enable_if<std::is_base_of<IRGBDImage, RGBDFrame>::value>::type>
static inline bool matchICP(const RGBDFrame* rgbds, Pose* rts, size_t rgbd_num,
    const CameraP& cam, int max_iter = 100, double min_overlap = 1e-1,
    double max_dist = 1e1, double max_angle = MVS_PI_4, double theta = 1e-9, int optimizer = 0)
{
    assert(rgbds && rts && rgbd_num > 1 && cam.size().valid());
    Image8uVec gray_vec(rgbd_num);
    ImagefVec  depth_vec(rgbd_num);
    for (size_t i = 0; i < rgbd_num; ++i) {
        depth_vec[i] = rgbds[i].rangeImage();
        gray_vec[i]  = Image8u(rgbds[i].colorImage());
    }
    return matchICP(&depth_vec[0], &gray_vec[0], nullptr, rts, rgbd_num, cam,
        max_iter, max_dist, max_angle, min_overlap, theta, optimizer);
}

/// @brief 点云之间的ICP配准
/// @param src 源点云接口类实例
/// @param dst 目标点云接口类实例
/// @param rt 相对姿态, src->dst.
/// @param method ICP迭代方法
/// @param max_iter 最大迭代次数
/// @param min_overlap 最小重叠率
/// @param max_dist 对应点之间的最大欧式距离
/// @param max_angle 对应点之间的最大法向夹角
/// @param theta 纹理残差权重，只有当Point-to-Normal-With-Color时才有用.
/// @return 返回是否成功迭代
MVS_EXPORT bool matchICP(const IPointCloud_<float>& src,
    const IPointCloud_<float>& dst, Pose& rt,
    ICPMethod method = ICPMethod::Normal, int max_iter = 10,
    float min_overlap = 1e-1f, float max_dist = 0.0f,
    float max_angle = static_cast<float>(MVS_PI_4), float theta = 0.3f,
    int optimizer = 0);

/// @brief 多帧点云ICP配准,会利用所有的输入信息完成迭代.
/// @param cloud_kdtree 点云数据对应的KDTree列表
/// @param pixels 纹理数据数组
/// @param gradients 梯度数据数组
/// @param rts 姿态数组
/// @param cloud_num 点云数量
/// @param max_iter 最大迭代次数
/// @param max_dist 最大对应点间距,可通过checkDist(cloud,9)计算.
/// @param max_angle 最大对应点法向夹角
/// @param min_overlap 最小重叠率
/// @param theta 纹理权重
/// @return 返回是否迭代成功
MVS_EXPORT bool matchICP(const CloudKDTree<float>** clouds,
    const uchar** pixels, const Point3f** gradients, Pose* rts,
    size_t cloud_num, int max_iter = 100, double max_dist = 1e1,
    double max_angle = MVS_PI_4, double min_overlap = 1e-1, double theta = 3e-1,
    int optimizer = 0);

/// @brief 多帧点云配准,会尝试自动转换纹理和梯度;
/// @param clouds 接口类点云数组
/// @param rts 姿态数组
/// @param cloud_num 点云数量
/// @param max_iter 最大迭代次数
/// @param max_dist 最大对应点间距,可通过checkDist(cloud,9)计算.
/// @param max_angle 最大对应点法向夹角
/// @param min_overlap 最小重叠率
/// @param theta 纹理权重
/// @return 返回是否迭代成功
MVS_EXPORT bool matchICP(const IPointCloud** clouds, Pose* rts,
    size_t cloud_num, ICPMethod method = ICPMethod::Normal, int max_iter = 100,
    double max_dist = 1e1, double max_angle = MVS_PI_4,
    double min_overlap = 1e-1, double theta = 3e-1, int optimizer = 0);

template <typename CloudFrame, class = typename std::enable_if<std::is_base_of<IPointCloud, CloudFrame>::value>::type>
static inline bool matchICP(const CloudFrame* clouds, Pose* rts,
    size_t cloud_num, ICPMethod method = ICPMethod::Normal, int max_iter = 100,
    double max_dist = 1e1, double max_angle = MVS_PI_4,
    double min_overlap = 1e-1, double theta = 3e-1, int optimizer = 0)
{
    if (!clouds || !rts || !cloud_num) return false;
    std::vector<const IPointCloud*> cloud_ptrs(cloud_num);
    for (size_t i = 0; i < cloud_num; ++i)
        cloud_ptrs[i] = reinterpret_cast<const IPointCloud*>(&clouds[i]);
    return matchICP(cloud_ptrs.data(), rts, cloud_num, method, max_iter,
        max_dist, max_angle, min_overlap, theta, optimizer);
}

/// @brief 计算帧间匹配的信息矩阵,PoseGraph的配套函数;
/// @param src_vmap 有序点云
/// @param src_nmap 有序点云法向
/// @param dst_vmap 有序点云
/// @param dst_nmap 有序点云法向
/// @param camera 相机内参
/// @param rt 帧间相对位姿
/// @param info 信息矩阵
/// @param min_overlap 最小重叠率
/// @param max_dist 邻近点最大距离
/// @param max_angle 邻近点最大夹角
/// @return 返回是否满足条件
MVS_EXPORT bool infoICP(const Image3f& src_vmap, const Image3f& src_nmap,
    const Image3f& dst_vmap, const Image3f& dst_nmap, const CameraP& camera,
    const Pose& rt, double info[36], double min_overlap = 1e-1,
    double max_dist = 1e1, double max_angle = MVS_PI_2);

/// @brief 计算点云帧间的匹配信息矩阵
/// @param src 源点云接口类实例
/// @param dst 目标点云接口类实例
/// @param rt 相对姿态
/// @param info 信息矩阵
/// @param min_overlap 最小重叠率
/// @param max_dist 对应点最大欧式距离
/// @param max_angle 对应点法向之间的最大夹角
/// @return 返回是否满足条件
MVS_EXPORT bool infoICP(const IPointCloud_<float>& src,
    const IPointCloud_<float>& dst, const Pose& rt, double info[36],
    double min_overlap = 1e-1, double max_dist = 1e1,
    double max_angle = MVS_PI_2);

/// @brief RGBD数据帧间配准接口类
/// @tparam Pixel 像素类型
/// @tparam Float 浮点类型[float or double]
template <typename Pixel, typename Float> struct IRGBDMatcher_ {
    typedef IRGBDMatchPara         MatchPara;
    typedef Image_<Point3_<Float>> Image3F;
    typedef std::vector<Image3F>   Image3FVec;
    /// @brief 虚析构函数
    virtual ~IRGBDMatcher_() {}
    /// @brief 创建实例
    /// @param cam 相机内参
    /// @param para 拼接参数
    /// @param detect_texture 是否检测纹理特征
    /// @return 如果创建失败,返回nullptr.
    static std::shared_ptr<IRGBDMatcher_> create(
        const CameraP& cam, const IRGBDMatchPara& para, bool keymatch);
    /// @brief 输入数据和参数
    virtual bool input(const Image_<Float>* depths, const Image_<Pixel>* colors,
        size_t depth_num) = 0;
    /// @brief 输入深度图和纹理
    /// @param depths 深度图
    /// @param colors 纹理图
    /// @return 返回数据是否成功导入
    virtual bool input(
        const ImageVec_<Float>& depths, const ImageVec_<Pixel>& colors)
    {
        if (depths.size() <= 1) return false;
        return input(&depths[0],
            colors.size() != depths.size() ? nullptr : &colors[0],
            depths.size());
    }
    /// @brief 仅输入深度图数据
    /// @param depths 深度图
    /// @return 返回是否成功导入
    virtual bool input(const std::vector<Image_<Float>>& depths)
    {
        if (depths.size() <= 1) return false;
        return input(&depths[0], nullptr, depths.size());
    }
    /// @brief 输入数据和参数
    virtual bool input(const IRGBDImage_<Float>** rgbds, size_t rgbd_num)
    {
        if (!rgbds || rgbd_num <= 1) return false;
        std::vector<Image_<Pixel>> color_vec(rgbd_num);
        std::vector<Image_<Float>> depth_vec(rgbd_num);
        for (size_t i = 0; i < rgbd_num; ++i) {
            color_vec[i] = Image_<Pixel>(rgbds[i]->colorImage());
            depth_vec[i] = Image_<Float>(rgbds[i]->rangeImage());
        }
        return input(&depth_vec[0], &color_vec[0], depth_vec.size());
    }
    /// @brief 输入RGBD数据帧
    /// @param rgbds RGBD数据
    /// @param rgbd_num 帧数量
    /// @return 返回数据是否成功导入
    template <typename RGBDFrame, class = typename std::enable_if<std::is_base_of<IRGBDImage_<Float>, RGBDFrame>::value>::type>
    bool input(const RGBDFrame* rgbds, size_t rgbd_num)
    {
        if (!rgbds || rgbd_num <= 1) return false;
        std::vector<const IRGBDImage_<Float>*> rgbd_vec(rgbd_num);
        for (size_t i = 0; i < rgbd_num; ++i) rgbd_vec[i] = &rgbds[i];
        return input(&rgbd_vec[0], rgbd_num);
    }
    /// @brief 输入RGBD数据帧
    /// @param rgbds RGBD数据
    /// @return 返回数据是否成功导入
    template <typename RGBDFrame, class = typename std::enable_if<std::is_base_of<IRGBDImage_<Float>, RGBDFrame>::value>::type>
    bool input(const std::vector<RGBDFrame>& rgbds)
    {
        return input(rgbds.data(), rgbds.size());
    }
    /// @brief 此处参数输入形式大部分只是用来进行稠密优化；
    virtual bool input(const Image3_<Float>* vmaps, const Image3_<Float>* nmaps,
        const Image_<Pixel>* colors, const Image_<Float>* dxs,
        const Image_<Float>* dys, size_t vmap_num) = 0;
    /// @brief 过程数据导入接口
    /// @param vmaps 有序点云
    /// @param nmaps 有序法向
    /// @param colors 纹理图
    /// @param dxs X方向梯度图
    /// @param dys Y方向梯度图
    /// @return 返回是否成功导入数据.
    virtual bool input(const Image3FVec& vmaps, const Image3FVec& nmaps,
        const ImageVec_<Pixel>& colors, const ImageVec_<Float>& dxs,
        const ImageVec_<Float>& dys)
    {
        if (vmaps.size() <= 1) return false;
        return input(&vmaps[0],
            nmaps.size() == vmaps.size() ? &nmaps[0] : nullptr,
            colors.size() == vmaps.size() ? &colors[0] : nullptr,
            dxs.size() == vmaps.size() ? &dxs[0] : nullptr,
            dys.size() == vmaps.size() ? &dys[0] : nullptr, vmaps.size());
    }
    /// @brief 过程数据导入接口
    /// @param vmaps 有序点云
    /// @param nmaps 有序法向
    /// @return 返回是否成功导入数据.
    virtual bool input(const Image3fVec& vmaps, const Image3fVec& nmaps)
    {
        if (vmaps.size() <= 1) return false;
        return input(&vmaps[0],
            nmaps.size() == vmaps.size() ? &nmaps[0] : nullptr, nullptr,
            nullptr, nullptr, vmaps.size());
    }
    /// @brief 辅助配置参数
    /// @return 返回参数的引用类型
    virtual MatchPara& param() = 0;
    /// @brief 帧间匹配并优化姿态图
    virtual bool match(PoseVec& rts, ProgressBar progress = 0) const = 0;
    /// @brief 组间匹配并优化姿态图
    virtual bool matchInterGroup(
        const IntVec& groups, PoseVec& rts, ProgressBar progress = 0) const = 0;
    /// @brief 帧间稠密优化姿态,需要输入初始姿态；
    virtual bool optimize(PoseVec& rts, ProgressBar progress = 0) const = 0;
    /// @brief 组内分组稠密优化
    virtual bool optimizeIntraGroup(
        const IntVec& groups, PoseVec& rts, ProgressBar progress = 0) const = 0;
};
using IRGBDMatcher = IRGBDMatcher_<uchar, float>;

/// @brief 创建RGBD拼接方法类
extern "C" MVS_EXPORT IRGBDMatcher* createRGBDMatcher(
    const CameraP&, const IRGBDMatchPara&, bool);

template <> struct ObjectCreator_<IRGBDMatcher> {
    static inline std::shared_ptr<IRGBDMatcher> create(const CameraP& camera,
        const IRGBDMatcher::MatchPara& para, bool keymatch = true)
    {
        return {createRGBDMatcher(camera, para, keymatch), [](IRGBDMatcher* p) {
                    if (p) delete p;
                }};
    }
};

template <typename Pixel, typename Float>
std::shared_ptr<IRGBDMatcher_<Pixel, Float>>
IRGBDMatcher_<Pixel, Float>::create(
    const CameraP& camera, const IRGBDMatchPara& para, bool keymatch)
{
    return createObject<IRGBDMatcher_<Pixel, Float>>(camera, para, keymatch);
}

/// @brief 匹配RGBD帧数据,并进行姿态图优化.
/// @tparam ...Args 输入参数模板
/// @param camera 相机内参
/// @param para 匹配参数
/// @param rts 各帧对应的姿态
/// @param ...args 输入参数
/// @return 返回是否成功匹配
template <typename... Args> static inline bool matchRGBD(const CameraP& camera,
    const IRGBDMatchPara& para, std::vector<Pose>& poses, Args&&... args)
{
    auto matcher = createObject<IRGBDMatcher>(camera, para, true);
    return matcher && matcher->input(std::forward<Args>(args)...) &&
           matcher->match(poses);
}

/// @brief 对RGBD帧数据的匹配姿态进行优化
/// @tparam ...Args 输入参数模板
/// @param camera 相机内参
/// @param para 匹配参数
/// @param rts 各帧对应的姿态
/// @param ...args 输入参数
/// @return 返回是否成功匹配
template <typename... Args>
static inline bool optimizeRGBD(const CameraP& camera,
    const IRGBDMatchPara& para, std::vector<Pose>& poses, Args&&... args)
{
    auto matcher = createObject<IRGBDMatcher>(camera, para, false);
    return matcher && matcher->input(std::forward<Args>(args)...) &&
           matcher->optimize(poses);
}

template <typename... Args> static inline bool matchRGBDInGroup(
    const CameraP& camera, const IRGBDMatchPara& para, const IntVec& groups,
    PoseVec& poses, Args&&... args)
{
    auto matcher = createObject<IRGBDMatcher>(camera, para, true);
    return matcher && matcher->input(std::forward<Args>(args)...) &&
           matcher->matchInterGroup(groups, poses);
}

/// @brief 对RGBD帧数据进行匹配和稠密优化.
/// @tparam ...Args 输入参数模板
/// @param camera 相机内参
/// @param para 匹配参数
/// @param rts 各帧对应的姿态
/// @param ...args 输入参数
/// @return 返回是否成功匹配
template <typename... Args>
static inline bool matchAndOptimizeRGBD(const CameraP& cam,
    const IRGBDMatchPara& para, std::vector<Pose>& rts, Args&&... args)
{
    auto matcher = createObject<IRGBDMatcher>(cam, para, true);
    return matcher && matcher->input(std::forward<Args>(args)...) &&
           matcher->match(rts) && matcher->optimize(rts);
}
template <typename... Args> static inline bool matchAndOptimizeRGBDInGroup(
    const CameraP& cam, const IRGBDMatchPara& para, const IntVec& groups,
    PoseVec& rts, Args&&... args)
{
    auto matcher = createObject<IRGBDMatcher>(cam, para, true);
    return matcher && matcher->input(std::forward<Args>(args)...) &&
           matcher->matchInterGroup(groups, rts) && matcher->optimize(rts);
}

void optmizeRGBDInGroup(const Image3f* vmaps, const Image3f* nmaps,
    const Image8u* grays, const Imagef* dxs, const Imagef* dys, Pose* rts,
    int* groups, size_t vmap_num, const CameraP& cam, int max_iter = 100,
    double max_dist = 1e1, double max_angle = MVS_PI_4,
    double min_overlap = 1e-1, double theta = 1e-6, int optimizer = 0);

/// @brief 点云数据配准参数类
struct ICloudMatchPara {
    ICPMethod icp_method = ICPMethod::Color;  ///< 匹配时使用的ICP方法
    float     key_radius = .0f;  ///< 提取几何特征点的搜索范围
    float key_nms_radius = .0f;  ///< 提取特征点的极大值抑制的搜索范围
    float key_pfh_radius  = .0f;    ///< 提取FPFH特征的PFH计算范围
    float key_fpfh_radius = .0f;    ///< 提取FPFH特征的统计范围
    int   key_max_iter    = 10000;  ///< 特征点匹配的最大迭代次数
    float key_max_dist = .0f;  ///< 特征点匹配时对应点的最大距离
    float key_max_angle =
        (float)MVS_PI_2;  ///< 特征点匹配时对应点法向之间的最大夹角
    float key_edge_ratio = 0.95f;  ///< 特征点匹配是边长之间的比例阈值
    int key_min_match = 7;  ///< 特征点匹配时的最小有效特征点对数量
    int icp_max_iter = 10;  ///< 帧间ICP匹配的迭代次数
    float icp_max_dist = 3.0f;  ///< 帧间ICP匹配时的最大点对之间的欧式距离
    float icp_max_angle =
        (float)MVS_PI_2;  ///< 帧间ICP匹配时的点对法向之间的最大夹角
    float icp_rgb_theta     = 0.001f;  ///< 帧间结合纹理ICP的纹理权重
    float icp_min_overlap   = 0.1f;    ///< 帧间ICP匹配时的最小重叠率
    int   optimize_max_iter = 100;     ///< 稠密优化时的最大迭代次数
    float optimize_min_overlap = 0.001f;  ///< 稠密优化时帧间的最小重叠率
    int posegraph_max_iter = 1000;  ///< 姿态图优化时的最大迭代次数
    float posegraph_max_dist = 3.0f;  ///< 姿态凸优化时的距离阈值
};

/// @brief 点云数据配准接口类
/// @tparam Pixel 像素类型
/// @tparam Float 浮点类型【float or double】
template <typename Pixel, typename Float> struct ICloudMatcher_ {
    typedef std::vector<const Float*>       FloatArrVec;
    typedef std::vector<const Pixel*>       PixelArrVec;
    typedef Point3_<Float>                  Point3;
    typedef std::vector<const Point3*>      Point3ArrVec;
    typedef PointCloud_<Float>              Cloud;
    typedef ICloudMatchPara                 MatchPara;
    typedef std::shared_ptr<ICloudMatcher_> Ptr;
    virtual ~ICloudMatcher_() {}
    static Ptr create(const ICloudMatchPara&, bool);
    /// @brief 初始化
    /// @param pts_vec 输入三维点数组
    /// @param nls_vec 输入三维法向数组
    /// @param pixs_vec 输入像素点数组
    /// @param ptnum_vec 输入帧的数量
    /// @return 返回是否成功预处理
    virtual bool input(const std::vector<const Point3_<Float>*>& pts_vec,
        const std::vector<const Point3_<Float>*>&                nls_vec,
        const std::vector<const Pixel*>&                         pixs_vec,
        const std::vector<size_t>&                               ptnum_vec) = 0;

    /// @brief 初始化并预处理,对上一接口的封装.
    /// @tparam T 集成IPointCloud接口类
    /// @param clouds 点云数组
    /// @return 返回是否成功预处理
    template <typename CloudFrame, class = typename std::enable_if<
        std::is_base_of<IPointCloud_<Float>, CloudFrame>::value>::type>
    bool input(const CloudFrame* clouds, size_t cloud_num)
    {
        if (cloud_num <= 1) return false;
        for (size_t i = 0; i < cloud_num; ++i) {
            if (!clouds[i].getPointNum())
                throw std::runtime_error("Error: Empty point cloud exists.");
        }
        PixelArrVec                     pixs_vec(cloud_num, 0);
        std::vector<std::vector<Pixel>> pixs_tmp(cloud_num);
        std::vector<size_t>             ptnum_vec(cloud_num, 0);
        Point3ArrVec pts_vec(cloud_num, 0), nls_vec(cloud_num, 0);
        for (size_t i = 0; i < cloud_num; ++i) {
            ptnum_vec[i] = clouds[i].getPointNum();
            pts_vec[i]   = clouds[i].getPointData();
            nls_vec[i]   = clouds[i].getNormalData();
            if (clouds[i].getPixelData() &&
                clouds[i].getPixelType() == PixelType::UINT8)
                pixs_vec[i] = clouds[i].getPixelData();
            else {
                // 尝试转换像素到灰度
                pixs_tmp[i] = PixelGetter_<uchar, CloudFrame>::get(clouds[i]);
            }
            if (!pixs_vec[i] && !pixs_tmp[i].empty())
                pixs_vec[i] = &pixs_tmp[i][0];
        }
        keepPixelData(pixs_tmp);
        return input(pts_vec, nls_vec, pixs_vec, ptnum_vec);
    }

    template <typename CloudFrame, class = typename std::enable_if<
        std::is_base_of<IPointCloud_<Float>, CloudFrame>::value>::type>
    bool input(const std::vector<CloudFrame>& clouds)
    {
        if (clouds.size() <= 1) return false;
        return input(&clouds[0], clouds.size());
    }

    /// @brief 初始配准
    /// @param rts 各帧对应姿态数据
    /// @param progress 进度条
    /// @return 返回是否成功匹配
    virtual bool match(PoseVec& rts, ProgressBar progress = 0) = 0;

    /// @brief 姿态优化
    /// @param rts 各帧对应姿态数据
    /// @return 返回是否成功优化
    virtual bool optimize(PoseVec& rts) = 0;

protected:
    /// @brief 辅助函数，用与保持临时转换的像素数据
    virtual void keepPixelData(std::vector<std::vector<Pixel>>&) = 0;
};
using ICloudMatcher = ICloudMatcher_<uchar, float>;
extern "C" MVS_EXPORT ICloudMatcher* createCloudMatcher(
    const ICloudMatchPara&, bool);
template <> struct ObjectCreator_<ICloudMatcher> {
    static inline std::shared_ptr<ICloudMatcher> create(
        const ICloudMatchPara& para, bool detect_key = true)
    {
        return {createCloudMatcher(para, detect_key), [](ICloudMatcher* p) {
                    if (p) delete p;
                }};
    }
};
template <typename Pixel, typename Float>
typename ICloudMatcher_<Pixel, Float>::Ptr ICloudMatcher_<Pixel, Float>::create(
    const ICloudMatchPara& para, bool detect_key)
{
    return createObject<ICloudMatcher_<Pixel, Float>>(para, detect_key);
}

/// @brief 匹配点云数据
/// @tparam ...Args 参数列表模板
/// @param para 匹配参数
/// @param rts 各帧对应的姿态
/// @param ...args 参数列表
/// @return 返回是否成功匹配
template <typename... Args> static inline bool matchCloud(
    const ICloudMatchPara& para, PoseVec& rts, Args&&... args)
{
    auto matcher = createObject<ICloudMatcher>(para, true);
    return matcher && matcher->input(std::forward<Args>(args)...) &&
           matcher->match(rts);
}

/// @brief 优化点云姿态
/// @tparam ...Args 参数列表模板
/// @param para 匹配参数
/// @param rts 各帧对应的姿态
/// @param ...args 参数列表
/// @return 返回是否成功迭代
template <typename... Args> static inline bool optimizeCloud(
    const ICloudMatchPara& para, PoseVec& rts, Args&&... args)
{
    auto matcher = createObject<ICloudMatcher>(para, false);
    return matcher && matcher->input(std::forward<Args>(args)...) &&
           matcher->optimize(rts);
}

/// @brief 匹配并优化各帧点云对应的姿态
/// @tparam ...Args 参数列表模板
/// @param para 匹配参数
/// @param rts 各帧对应的姿态
/// @param ...args 参数列表
/// @return 返回是否成功迭代
template <typename... Args> static inline bool matchAndOptimizeCloud(
    const ICloudMatchPara& para, PoseVec& rts, Args&&... args)
{
    auto matcher = createObject<ICloudMatcher>(para, true);
    return matcher && matcher->input(std::forward<Args>(args)...) &&
           matcher->match(rts) && matcher->optimize(rts);
}
}  // namespace rulermvs
#endif  // _RULERMVS_CORE_MATCH3D_HPP_