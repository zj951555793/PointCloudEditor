/**
 ********************************************************************************
 * Copyright (c) 2023, Li Yunqiang, walkfish8@hotmail.com.
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the organization nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ''AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTOR BE
 * LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************
 */
#pragma once

#ifndef _RGBDSLAM_RGBDSLAM_H_
#define _RGBDSLAM_RGBDSLAM_H_

#define RGBDSLAM_VERSION_MAJOR 1
#define RGBDSLAM_VERSION_MINOR 0
#define RGBDSLAM_VERSION_PATCH 1

// undef export setting
#ifdef RULERMVS_EXPORTS
#undef RULERMVS_EXPORT
#endif

#ifdef RULERMVS_EXPORTS
#if defined(WIN32) && defined(_MSC_VER)
#define RULERMVS_EXPORT __declspec(dllexport)
#elif defined(WIN32) && defined(__GNUC__)
#define RULERMVS_EXPORT __attribute__((visibility("default")))
#else
#define RULERMVS_EXPORT
#endif
#else
#if defined(WIN32) && defined(_MSC_VER) && defined(RGBDSLAM_SHARED_LIBS)
#define RULERMVS_EXPORT __declspec(dllimport)
#elif defined(WIN32) && defined(__GNUC__)
#define RULERMVS_EXPORT
#else
#define RULERMVS_EXPORT
#endif
#endif
#include "DBoW3/DBoW3.h"
#include <opencv2/opencv.hpp>
namespace rgbdslam
{
struct RULERMVS_EXPORT IRGBDResult {
    enum : uchar { Failed = 0, Succeed = 1, Key, Relocate, Reset };
    virtual uchar getFlag() const = 0;
    virtual cv::Mat getRT() const = 0;
    virtual int64_t getTime() const = 0;
    virtual cv::Mat getMask() const = 0;
    virtual cv::Mat getDepth() const = 0;
    virtual cv::Mat getColor() const = 0;
    virtual const uchar* getMaskPtr() const = 0;
    virtual const float* getDepthPtr() const = 0;
    virtual const cv::Vec3b* getColorPtr() const = 0;
    virtual int64_t getUserID() const = 0;
    virtual int64_t getFrameID() const = 0;
    virtual int64_t getElapsedTime() const = 0;
    virtual int getKeyMatchNum() const = 0;
    virtual void getVmapAndNmap(cv::Mat& vmap, cv::Mat& nmap) const = 0;
    virtual void toCloud(std::vector<cv::Point3f>& pts,
        std::vector<cv::Point3f>& nls, std::vector<cv::Vec3b>& rgbs) const = 0;
};

class RGBDFusionImpl;
class RULERMVS_EXPORT RGBDFusion {
public:
    /** 在线拼接参数 */
    struct Para {
        bool is_use_dbow;
        int localMode; /* 0:None, 1:Pair, 2:Dense */
        int localMaxIter /*=10*/;
        int globalMode; /* 0:None, 1:Fast, 2:Full */
        double maxAngle;
        double minOverlap;
        double colorTheta;
        int minMatchNum;
        int maxFeatureNum;
        int minRelocMatchNum;
        double minEdgeRatio;
        double maxFeatureDist;
    };
    using ProgressFunc = std::function<void(int /*nPercent*/, bool& /*bStop*/)>;
    using AddFrameCallback = std::function<void(int& userID, int64_t& nTime,
        cv::Mat& depth, cv::Mat& color, cv::Mat& mask, cv::Mat& gray)>;

    /*RGBDFusion(const cv::Mat& K, const DBoW3::Vocabulary& dbow,
        const DBoW3::Database& db, int width, int height, double* maxDist,
        int* maxIter, int nLayer, int nThread = 2, int nGroup = 15,
        bool bColorICP = true, bool bWithCuda = false);*/

    /////* 20260610, 新增两个参数，方便一体机测试 */
    RGBDFusion(const cv::Mat& K, const DBoW3::Vocabulary& dbow,
        const DBoW3::Database& db, int width, int height, double* maxDist,
        int* maxIter, int nLayer, int nThread = 2, int nGroup = 15,
        bool bColorICP = true, bool bWithCuda = false, bool use_rgbsift = true, bool EnableExpandImage = false);
    /*RGBDFusion(const cv::Mat& K, int width, int height, double* maxDist,
        int* maxIter, int nLayer, int nThread = 2, int nGroup = 15,
        bool bColorICP = true, bool bWithCuda = false);*/

    /** 析构函数 */
    ~RGBDFusion();

    /** 配置拼接参数*/
    Para& para();

    /** 停止线程并忽略后续加入的数据 */
    void stop();

    /** 启动线程池和各优化线程 */
    void start();

    /** 强制停止,可能引起数据异常 */
    void forceStop();

    /** 添加深度图和纹理等数据 */
    void addFrame(const cv::Mat& inDepth, const cv::Mat& inColor,
        const cv::Mat& inMask, int userID = -1, int64_t nTime = -1);

    /** 回调添加帧数据 */
    void addFrameInCallBack(AddFrameCallback func);

    /** 添加里程计匹配回调函数 */
    void setTraceCallBack(std::function<void(const IRGBDResult&)> func);

    /** 遍历访问帧数据以获得姿态和点云数据 */
    void getResults(std::function<void(const IRGBDResult&)> func) const;

    /*合并点云*/
    void fusePoints(std::vector<cv::Point3f>& pts,
        std::vector<cv::Point3f>& nls, std::vector<cv::Vec3b>& rgbs) const;

    /** 基于姿态图优化的方式 */
    void optimizePoseGraph(bool bFastMethod = false);

    /** 基于稠密点云迭代的方式 */
    void optimizePointMap(
        double voxelUnit, int maxIter = 50, ProgressFunc progress = nullptr);

    /** 保存所有有效帧的点云到目录中 */
    void savePointsPerFrame(std::string& outputDir);

    /** 获取版本号*/
    static const char* version();

    /** 获取当前系统时间 */
    int64_t getCurrentTime() const;

    /** 点云抽稀 */
    static void voxelFilter(double voxelUnit, std::vector<cv::Point3f>& pts,
        std::vector<cv::Point3f>& nls, std::vector<cv::Vec3b>& rgbs);

    /** 将点云数据保存到文件中 */
    static bool writePoints(const std::string& path,
        std::vector<cv::Point3f>& pts, std::vector<cv::Point3f>& nls,
        std::vector<cv::Vec3b>& rgbs);

protected:
    RGBDFusionImpl* const impl;
};

class OneShotDecoder;
class RULERMVS_EXPORT OneShotFusion: RGBDFusion {
public:
    /** @brief 解码参数 */
    typedef struct {
        int minGrayValue;       /** 黑色敏感度 */
        int linkInterval;       /** 搜索连接范围阈值 */
        int smoothXEnergy;      /** 动态优化的X方向平滑项 */
        int smoothYEnergy;      /** 动态优化的Y方向平滑项 */
        int minGroupCount;      /** 最小聚集数量 */
        double lineThreshold;   /** 中心线判定 */
        double gaussianSigma;   /** 高斯核对应的方差 */
        double minTriEdgeRatio; /** 三角形边长之比 */
    } DecodePara;
    OneShotFusion(const std::string& calibFile, double* maxDist, int* maxIter,
        int nLayer, int nThread = 2, int nGroup = 15, bool bColorICP = true,
        bool bWithCuda = false);
    void addFrame(const cv::Mat& code, const cv::Mat& color,
        const cv::Mat& mask, int64_t userID = -1, int64_t nTime = -1);

protected:
    OneShotDecoder* const decoder;
};

}  // namespace rgbdslam

#endif  //_RGBDSLAM_RGBDSLAM_H_