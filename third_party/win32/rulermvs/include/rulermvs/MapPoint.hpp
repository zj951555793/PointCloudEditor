#ifndef _SLAM_TRACKER_MAPPOINT_H_
#define _SLAM_TRACKER_MAPPOINT_H_
#include "rulermvs/eigen.hpp"
#include "rulermvs/cv.hpp"

namespace slam
{
class GlobalMap;
struct MapPointObservation
{
    MapPointObservation(int frame_id, int corner_id)
    {
        frame_id_ = frame_id;
        corner_id_ = corner_id;
    }
    int frame_id_;
    int corner_id_;
};

class MVS_EXPORT MapPoint
{
public:
    MapPoint() {}
    MapPoint(rulermvs::Vec3d& point3d, rulermvs::Vec3d& normal3d) :point3d_world_(point3d), normal_world_(normal3d) {}
    ~MapPoint() {}

    int id_;
    bool            is_bad_ = false;//是否为外点
    //bool is_reliable_ = false;  //地图点是否稳定，默认不稳定
    bool is_unmerged_ = false;  //误差阈值过大导致未合并同名点
    int father_id_ = -1;  //未合并同名点的父点id
    std::vector<int> sons_id_;  //未合并同名点的子点id集合
    bool is_checked_ = false;//是否被检验过，默认未检验
    rulermvs::Vec3d point3d_world_;
    rulermvs::Vec3d normal_world_;

    std::vector<MapPointObservation> observations_;
    //std::map<int ,int> observations_;

    //添加帧号-角点号对应观测
    void AddObservation(int frame_id, int corner_id);

    //删除观测
    bool DeleteObservation(int frame_id);

    //获取观测数量
    int GetKeyFrameObservationCount(GlobalMap* global_map_ptr, int min_key_frame_idx);

    //更新点的法线-未使用
    void UpdateNormal(rulermvs::Vec3d& normal_cam, const rulermvs::Mat44& pose_c2w);

    //更新点的坐标
    void UpdatePoint3d(rulermvs::Vec3d& point3d_cam, const rulermvs::Mat44& pose_c2w);
};

}  // namespace slam
#endif