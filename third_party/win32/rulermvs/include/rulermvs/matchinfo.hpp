#ifndef _RULERMVS_CORE_MATCH_INFO_HPP_
#define _RULERMVS_CORE_MATCH_INFO_HPP_
#include "rulermvs/pose.hpp"
namespace rulermvs
{
/// @brief 帧间匹配信息，用于图优化
struct MatchInfo {
    MatchInfo() : info {0}, src_id(0), dst_id(0), quality(255) {}
    MatchInfo(const MatchInfo& match_info)
        : src_id(match_info.src_id), dst_id(match_info.dst_id), quality(255)
    {
        measure = match_info.measure;
        memcpy(this->info, match_info.info, 36 * sizeof(double));
    }
    MatchInfo(int src_id, int dst_id, const Pose& RT, const double info[36])
        : measure(RT), src_id(src_id), dst_id(dst_id), quality(255)
    {
        memcpy(this->info, info, 36 * sizeof(double));
    }
    MatchInfo(int src_id, int dst_id, const Pose& RT, const double info[36],
        uchar quality)
        : measure(RT), src_id(src_id), dst_id(dst_id), quality(quality)
    {
        memcpy(this->info, info, 36 * sizeof(double));
    }
    virtual ~MatchInfo() {}

    Pose   measure;   ///< 相对姿态
    double info[36];  ///< 信息矩阵
    int    src_id;    ///< 源索引
    int    dst_id;    ///< 目标索引
    uchar  quality;   ///< 匹配质量
};

RULERMVS_JSON_IO_EXPORT(MatchInfo);
RULERMVS_JSON_IO_EXPORT(std::vector<MatchInfo>);
}  // namespace rulermvs
#endif