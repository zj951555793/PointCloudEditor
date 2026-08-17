#ifndef _RULERMVS_MATCH_POSEGRAPH_HPP_
#define _RULERMVS_MATCH_POSEGRAPH_HPP_
#include "rulermvs/core.hpp"
#include "rulermvs/pose.hpp"
#include "rulermvs/matchinfo.hpp"
namespace rulermvs
{
/// @brief 类G2O利用信息矩阵优化姿态
/// @param infos 匹配信息
/// @param weights 动态权重,为空则不输出;
/// @param info_num 信息数量
/// @param rts 初始姿态矩阵
/// @param rt_num 姿态数量
/// @param max_iter 最大迭代次数
/// @param loop_closure 回环系数
/// @param max_dist 最大距离
/// @param fix_mode 优化方法,0代表不固定, 1代码固定邻近帧 2 代码固定所有.
/// @return 返回是否成功迭代
MVS_EXPORT bool optimizePoseGraph(const MatchInfo* infos, double* weights,
    size_t info_num, Pose* rts, size_t rt_num, int max_iter,
    double loop_closure, double max_dist, int fix_mode = 0,
    bool verbose = false);

/// @brief 优化PoseGraph,并抑制误匹配
/// @param infos 信息矩阵
/// @param rts 姿态矩阵
/// @param max_iter 最大迭代次数
/// @param loop_closure 回环系统
/// @param max_dist 最大距离
/// @param pruned_ratio 低于该权重的匹配信息将删除
/// @return 返回是否成功迭代
MVS_EXPORT bool optimizePoseGraphAndPruned(const std::vector<MatchInfo>& infos,
    std::vector<Pose>& poses, int max_iter, double loop_closure,
    double max_dist, double pruned_ratio);

/// @brief 组间优化PoseGraph
/// @param infos 信息矩阵
/// @param weights 信息矩阵返回权重
/// @param info_num 信息矩阵数量
/// @param groups 分组信息
/// @param rts 姿态矩阵
/// @param rt_num 姿态矩阵数量
/// @param max_iter 最大迭代次数
/// @param loop_closure 闭环系数
/// @param max_dist 最大距离
/// @param fix_mode 优化方法,0代表不固定, 1代码固定邻近帧 2 代码固定所有.
/// @return 返回是否成功迭代
MVS_EXPORT bool optimizePoseGraphInterGroup(const MatchInfo* infos,
    size_t info_num, const int* groups, Pose* rts, size_t rt_num, int max_iter,
    double loop_closure, double max_dist, int fix_mode = 0,
    bool verbose = false);

/// @brief 组内优化PoseGraph
/// @param infos 信息矩阵
/// @param groups 分组信息
/// @param rts 姿态矩阵
/// @param max_iter 最大迭代次数
/// @param loop_closure 闭环系数
/// @param max_dist 最大距离
/// @return
MVS_EXPORT bool optimizePoseGraphIntraGroup(const MatchInfo* infos,
    size_t info_num, const int* groups, Pose* rts, size_t rt_num, int max_iter,
    double loop_closure, double max_dist, int fix_mode = 0);

static inline bool optimizePoseGraph(const std::vector<MatchInfo>& infos,
    std::vector<Pose>& rts, int max_iter, double loop_closure, double max_dist,
    int optimizer = 0, bool verbose = false)
{
    if (infos.empty() || rts.size() <= 1) return false;
    return optimizePoseGraph(infos.data(), nullptr, infos.size(), rts.data(),
        rts.size(), max_iter, loop_closure, max_dist, optimizer, verbose);
}

static inline bool optimizePoseGraph(const std::vector<MatchInfo>& infos,
    std::vector<Pose>& rts, std::vector<double>& weights, int max_iter,
    double loop_closure, double max_dist, int optimizer = 0,
    bool verbose = false)
{
    if (infos.empty() || rts.size() <= 1) return false;
    if (weights.size() != infos.size()) weights.resize(infos.size(), 0);
    return optimizePoseGraph(infos.data(), weights.data(), infos.size(),
        rts.data(), rts.size(), max_iter, loop_closure, max_dist, optimizer,
        verbose);
}
static inline bool optimizePoseGraphInterGroup(
    const std::vector<MatchInfo>& infos, const IntVec& groups, PoseVec& poses,
    int max_iter, double loop_closure, double max_dist, int fix_mode = 0,
    bool verbose = false)
{
    if (poses.empty() || poses.size() != groups.size()) return false;
    return optimizePoseGraphInterGroup(infos.data(), infos.size(),
        groups.data(), poses.data(), poses.size(), max_iter, loop_closure,
        max_dist, fix_mode, verbose);
}
static inline bool optimizePoseGraphIntraGroup(
    const std::vector<MatchInfo>& infos, std::vector<int>& groups,
    std::vector<Pose>& rts, int max_iter, double loop_closure, double max_dist,
    int fix_mode = 0)
{
    if (rts.size() <= 1 || groups.size() != rts.size() || infos.empty())
        return false;
    return optimizePoseGraphIntraGroup(infos.data(), infos.size(),
        groups.data(), rts.data(), rts.size(), max_iter, loop_closure, max_dist,
        fix_mode);
}

}  // namespace rulermvs

#endif  // _RULERMVS_MATCH_POSEGRAPH_HPP_