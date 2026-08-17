/***********************************************************************
* Software License Agreement (Ruler License)
*
* Copyright 2008-2011  Li YunQiang (liyunqiang@91ruler.com). All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
* THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************/
#ifndef _SLAM_TRACKER_POINTCLOUDTRACKER_HPP_
#define _SLAM_TRACKER_POINTCLOUDTRACKER_HPP_
#include "rulermvs/cv.hpp"
#include "rulermvs/eigen.hpp"
namespace slam
{

using uchar = unsigned char;
enum class VoxelFilterMethod
{
    VoxelFilterMethod_Centorid,
    VoxelFilterMethod_Approximate
};
enum class ICPMethod {
    ICPMethod_PointToPoint,
    ICPMethod_PointToNormal,
    ICPMethod_PointToNormalWithTexture
};

class TriMesh;
typedef std::shared_ptr<TriMesh> TriMeshPtr;

struct  MatchInfo
{
//public:
//	MatchInfo() = default;
//	~MatchInfo() = default;
//
//	MatchInfo(const MatchInfo& minfo)
//	{
//		this->source_ind = minfo.source_ind;
//		this->target_ind = minfo.target_ind;
//		this->T_measure = minfo.T_measure.clone();
//		this->info = minfo.info;
//	}
//
//	MatchInfo(int sind, int tind)
//	{
//		this->source_ind = sind;
//		this->target_ind = tind;
//		this->T_measure = cv::Mat();
//	}
//
//	MatchInfo(int sind, int tind, cv::Mat tm, Eigen::MatrixXd i)
//	{
//		this->source_ind = sind;
//		this->target_ind = tind;
//		this->T_measure = tm.clone();
//		this->info = i;
//	}

//public:
    int source_ind;
    int target_ind;

    cv::Mat T_measure;     // 测量值
    Eigen::MatrixXd info;  // 信息矩阵
};

struct RGBA
{
public:
    RGBA() : r(0), g(0), b(0), a(0) {}

    template<typename T>
    RGBA(T intensity) : r((uchar)intensity), g((uchar)intensity), b((uchar)intensity), a(255) {}

    template<typename T>
    RGBA(T r, T g, T b) : r((uchar)r), g((uchar)g), b((uchar)b), a(255) {}

    template<typename T>
    RGBA(T r, T g, T b, T a) : r((uchar)r), g((uchar)g), b((uchar)b), a(a) {}

public:
    union
    {
        struct
        {
            uchar r, g, b, a;
        };
        uchar data[4];
    };
};

class MVS_EXPORT PointCloud {
public:
    PointCloud() = default;
    ~PointCloud() = default;
    PointCloud(const PointCloud&) = default;

    void clear();

    // 深拷贝
    PointCloud clone() const;

    PointCloud* clone_and_rtn_ptr() const;

    // 将点云和法向对应坐标进行变换
    PointCloud& transfrom(const cv::Mat& P, bool use_omp = true);

    PointCloud& voxel_filter_flag(float voxel_leaf /*= 3.0f*/, VoxelFilterMethod method = VoxelFilterMethod::VoxelFilterMethod_Approximate);
    // 体素滤波, if voxel_leaf <= 0 do nothing
    PointCloud& voxel_filter(float voxel_leaf = 3.0f, VoxelFilterMethod method = VoxelFilterMethod::VoxelFilterMethod_Approximate);

    // 移除所有没有对应色彩的点云
    PointCloud& remove_invalid_color_point();

    PointCloud operator +(const PointCloud& cloud);
    PointCloud& operator +=(const PointCloud& cloud);

    // 保存文件
    PointCloud& load_asc(const char* asc_path);
    PointCloud& save_asc(const char* asc_path, const cv::Mat& P = cv::Mat());

    // 计算FPFH特征
    void compute_fpfh();
    void compute_normals();

    // 计算彩色点云梯度
    void compute_gradients(float max_distance);

    // inline size_t kdtree_get_point_count() const;
    // inline float kdtree_get_pt(const size_t idx, const size_t dim) const;
    // Must return the number of data points
    size_t kdtree_get_point_count() const { return points.size(); }
    
    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    float kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0) return points[idx].x;
        else if (dim == 1) return points[idx].y;
        else return points[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

public:
    std::vector<RGBA> rgbs;
    std::vector<bool> flags;
    // 点云和其对应法向量
    std::vector<cv::Point3f> points, normals;

    // icp for color points
    std::vector<float> intensities;
    std::vector<cv::Point3f> gradients;
};

// 会进行深拷贝
MVS_EXPORT PointCloud mergePointClouds(const std::vector<PointCloud>& clouds, const std::vector<cv::Mat>& poses,
                                              const std::vector<bool>& inliers = std::vector<bool>{});

MVS_EXPORT PointCloud* mergePointClouds(const std::vector<PointCloud*>& clouds, const std::vector<cv::Mat>& poses,
                                               const std::vector<bool>& inliers = std::vector<bool>{});

MVS_EXPORT PointCloud* mergePointClouds(const std::vector<const PointCloud*>& clouds, const std::vector<cv::Mat>& poses,
                                               const std::vector<bool>& inliers = std::vector<bool>{});

}  // namespace slam
#endif