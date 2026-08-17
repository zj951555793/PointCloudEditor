#ifndef _RULERMVS_MULTIFRAMEFILTER_MULTIFRAMEFILTER_HPP_
#define _RULERMVS_MULTIFRAMEFILTER_MULTIFRAMEFILTER_HPP_
#include "rulermvs/rgbd.hpp"
#include "rulermvs/image.hpp"
#include "rulermvs/point.hpp"
#include "rulermvs/pointcloud.hpp"
#include "rulermvs/cv.hpp"
namespace rulermvs
{
///@brief ICP match method，based on the distance of measure function.
enum class DeduplicateMethod { Point = 0, Normal };

using Point3fvoteCounts = std::pair<Scalar3_<int>, Point3f>;

/// @brief
/// 腐蚀深度图，获得边缘在深度图中的序列索引及点云中的序列索引、深度图中的掩模版;
/// @param img 输入深度图
/// @param erode_size 边缘腐蚀尺寸
/// @param vec1 记录所有point在整个阵列图片中索引，大小为所有point的num
/// @param mask 深度图掩模版
/// @param edgemask 边缘point掩模版
/// @param vec2 记录边缘point在点云中索引，大小为点云num
template <typename Tp> static inline int cvtImgToEdgeVec(const Image_<Tp>& img,
    uchar erode_size, std::vector<int>& vec1, Image8u& mask, Image8u& edgemask,
    std::vector<bool>& vec2)
{
    mask     = depth2Mask(img);
    auto num = countNonZero(mask);
    vec1.resize(num);
    vec2.resize(num, false);  // true表示当前点是边缘点
    Image8u mask2(mask.size());
    edgemask.create(mask.size());
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(erode_size, erode_size));
    cv::erode(
        mask.to<cv::Mat>(), mask2.to<cv::Mat>(), kernel, cv::Point(-1, -1), 1);
    int count = 0;  // 边缘point数量计数器
    int index = 0;  // 每个point在点云中的序号
    for (int j = 0; j < mask.size().area(); j++) {
        if (mask.data[j]) {
            if (!mask2.data[j]) {
                vec2[index]      = true;
                edgemask.data[j] = (uchar)-1;
                count++;
            }
            vec1[index] = j;
            index++;
        }
    }
    return count;
}

/// @param cam 去畸变后的相机参数
MVS_EXPORT void multidepth_filter_edgenoise(const Imagef* depths,
    const Pose* rts, size_t depth_num, const CameraP& cam,
    std::vector<Imagef>& filtereddepths, std::vector<Image3f>& vmaps,
    std::vector<Image3f>& nmaps, uchar erode_size = 7, uchar searchmode = 0,
    const int minview = 3, double max_dist1 = 3e-1,
    double max_angle1 = MVS_PI_6, double max_dist2 = 1e-1,
    double max_angle2 = MVS_PI_12);

/// @brief 在多个帧的点云数据中去除冗余点，提高数据的质量和处理效率;
/// @param clouds 输入点云起始地址
/// @param rts 点云旋转矩阵起始地址
/// @param cloud_num 输入点云数量cloud_num,cloud_num>=2.
/// @param max_dist1 判断同名点距离阈值
/// @param max_angle1 判断同名点法向夹角阈值
/// @param max_dist2 同名点评分距离阈值
/// @param max_angle2 同名点评分法向夹角阈值
MVS_EXPORT bool multiframe_deduplicate(const IPointCloud** clouds,
    size_t cloud_num, PointCloud& cloud_out,
    DeduplicateMethod method = DeduplicateMethod::Normal,
    double max_dist1 = 3e-1, double max_angle1 = MVS_PI_6,
    double max_dist2 = 1e-1, double max_angle2 = MVS_PI_12);

MVS_EXPORT bool multiframe_deduplicate(const CloudKDTree<float>** cloud_kdtree,
    size_t cloud_num, PointCloud& cloud_out, double max_dist1 = 3e-1,
    double max_angle1 = MVS_PI_6, double max_dist2 = 1e-1,
    double max_angle2 = MVS_PI_12);

MVS_EXPORT bool multiframe_deduplicate(const Image3f* vmaps,
    const Image3f* nmaps, const Pose* rts, size_t depth_num, const CameraP& cam,
    PointCloud& cloud_out, double max_dist1 = 3e-1,
    double max_angle1 = MVS_PI_6, double max_dist2 = 1e-1,
    double max_angle2 = MVS_PI_12);

/// @brief 在多个帧的点云数据中去除冗余点，提高数据的质量和处理效率;
/// @param clouds 输入点云起始地址
/// @param rts 点云旋转矩阵起始地址
/// @param cloud_num 输入点云数量cloud_num,cloud_num>=2.
/// @param max_dist1 判断同名点距离阈值
/// @param max_angle1 判断同名点法向夹角阈值
/// @param max_dist2 同名点评分距离阈值
/// @param max_angle2 同名点评分法向夹角阈值
template <typename CloudFrame, class = typename std::enable_if<std::is_base_of<
                                   IPointCloud, CloudFrame>::value>::type>
static inline bool multiframe_deduplicate(const CloudFrame* clouds,
    size_t cloud_num, CloudFrame& cloud_out,
    DeduplicateMethod method = DeduplicateMethod::Normal,
    double max_dist1 = 3e-1, double max_angle1 = MVS_PI_6,
    double max_dist2 = 1e-1, double max_angle2 = MVS_PI_12)
{
    if (!clouds || /*!rts ||*/ !cloud_num) return false;
    std::vector<const IPointCloud*> cloud_ptrs(cloud_num);
    for (size_t i = 0; i < cloud_num; ++i)
        cloud_ptrs[i] = reinterpret_cast<const IPointCloud*>(&clouds[i]);
    return multiframe_deduplicate(cloud_ptrs.data(), /* rts,*/ cloud_num,
        cloud_out, method, max_dist1, max_angle1, max_dist2, max_angle2);
}

/// @brief
/// 在多个帧(未变换到同一个视角)的点云数据中去除冗余点，提高数据的质量和处理效率;
static inline bool multiframe_deduplicate(const PointCloud* clouds, Pose* rts,
    size_t cloud_num, PointCloud& cloud_out,
    DeduplicateMethod method = DeduplicateMethod::Normal,
    double max_dist1 = 3e-1, double max_angle1 = MVS_PI_6,
    double max_dist2 = 1e-1, double max_angle2 = MVS_PI_12)
{
    if (!clouds || !rts || !cloud_num) return false;
    std::vector<PointCloud> trans_clouds(cloud_num);
    for (size_t i = 0; i < cloud_num; ++i) {
        trans_clouds[i] = clouds[i];
        trans_clouds[i].transform(rts[i]);
        // cloud_out.points.insert(cloud_out.points.end(),
        //     trans_clouds[i].points.begin(), trans_clouds[i].points.end());
        // cloud_out.normals.insert(cloud_out.normals.end(),
        //     trans_clouds[i].normals.begin(), trans_clouds[i].normals.end());
    }
    multiframe_deduplicate(trans_clouds.data(), cloud_num, cloud_out, method,
        max_dist1, max_angle1, max_dist2, max_angle2);
}

static inline bool multiframe_deduplicate(const Imagef* depths, Pose* rts,
    size_t depth_num, const CameraP& cam, PointCloud& cloud_out,
    DeduplicateMethod method = DeduplicateMethod::Normal,
    double max_dist1 = 3e-1, double max_angle1 = MVS_PI_6,
    double max_dist2 = 1e-1, double max_angle2 = MVS_PI_12)
{
    if (!depths || !rts || !depth_num) return false;
    std::vector<Image3f> vmaps(depth_num), nmaps(depth_num);
    MVS_OMP_PARALLEL_FOR
    for (int i = 0; i < depth_num; i++) {
        depth2VmapAndNmap(depths[i], cam, vmaps[i], nmaps[i]);
    }
    multiframe_deduplicate(vmaps.data(), nmaps.data(), rts, depth_num, cam,
        cloud_out, max_dist1, max_angle1, max_dist2, max_angle2);
}
}  // namespace rulermvs
#endif