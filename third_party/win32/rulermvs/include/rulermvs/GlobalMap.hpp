#ifndef _SLAM_TRACKER_GLOBALMAP_HPP_
#define _SLAM_TRACKER_GLOBALMAP_HPP_
#include "rulermvs/MapPoint.hpp"
#include "rulermvs/Frame.hpp"

namespace slam
{
enum FrameMatchMode : int
{
    match_globalMap_direct,
    match_adjacentFrame_priority,
    match_adjacentFrame_only
};//匹配模式:直接与全局地图对齐，优先与相邻帧对齐(失败则考虑与全局对齐)，仅仅与相邻帧对齐(失败则退出)

class MVS_EXPORT GlobalMap
    {
    public:
        GlobalMap() {}
        ~GlobalMap() {}

        void MultiForwardIntersectionSingleCam(
            cv::Mat& camMat, cv::Mat& distortMat, std::vector<cv::Mat>& vPos,
            std::vector<cv::Point2f>& imgPoints,
            cv::Point3f& objectPoint);
        //insert one frame to the global map
        void InsertFrame(Frame& frame, bool is_keyframe = false, bool is_MatchByMap = false);

        /**
         * 针对未加入全局地图且相对位姿已知的帧，根据旧的RT计算同名点、未合并同名点，再根据所有合并同名点重新计算新的RT，并根据新的RT将该帧加入全局地图
         * @param frame 相对位姿已知的帧，建议不包含观测、匹配地图点、位姿等信息，内部将重置
         * @param rt frame的旧位姿
         * @param min_match_points_num 最小同名点数量
         * @param updatepointandnormal 是否使用当前帧的三维点更新全局地图中的地图点的坐标和法向
         */
        void InsertFrameByRecalculateObservationsAndRT(Frame& frame,
            const cv::Mat& rt, int min_match_points_num = 3,
            bool updatepointandnormal = true);

        //insert one map point to the global map
        void InsertMapPoint(rulermvs::Vec3d& point3d, rulermvs::Vec3d& normal);

        //if we can find a map point that the point' id is equal to the input map_point_id ,return true
        bool FindFrameMatchPoint(int map_point_id, Frame& frame);

        /**
         * 针对未加入全局地图且相对位姿已知的帧，根据RT计算同名点、未合并同名点，并改变frame中观测信息、匹配地图点信息
         * @param frame 相对位姿已知的帧，建议不包含观测、匹配地图点、位姿等信息，内部将重置
         * @param rt frame相对全局地图的位姿
         * @param unmerged_observations 未合并同名点（欧氏距离大于最大阈值但不超过2倍）的观测列表，帧内corner_id与未合并同名地图点id构成pair的集合
         * @param merged_father_son_IDs 其size与frame的corner数量相等，当且仅当该corner成功匹配上全局地图点时，其对应的集合不为空，其集合包括所有阈值符合要求的地图点和未合并同名点
         */
        void FindFrameMatchGlobalMapPointWithUmergedPoint(Frame& frame,
            const cv::Mat& rt,
            std::vector<std::pair<int, int>>& unmerged_observations,
            std::vector<std::vector<int>>& merged_father_son_IDs);

        //insert all the point into the global map if the point is not exist int the map 
        void ExpandMapPoints(Frame& key_frame, bool is_MatchByMap = false);

        //set the tag of the map point false if the map point is bad
        void MapPointCutting();

        void KeyframeCutting();

        void FindNoDepthCornerMatchMappoint();       

        std::vector<MapPoint>map_points_;

        std::vector<Frame>frames_;

        std::vector<int>key_frames_idxs_;

		std::vector<int>key_global_idxs_;

        std::vector<int>relocation_frames_idxs_;

        void Setparameter(double site_diff_thres, double p3d_match_thres);

        double site_diff_thres_;

        double p3d_match_thres_;// P3D_MATCH_THRES

        bool UpdatePointerAndDistMap(std::vector<int>& new_mappoint_id, double maxP3dDist = 500.0, bool clear = false);//更新GlobalMap内的point3d的指针以及点对间的dist

        void DeleteFrame(const std::vector<int> indices);//删除global_map中的帧，indices表示帧的索引集合

        void DeleteMapPoints(std::vector<int> delete_mappoint_indices);//删除global_map中的地图点，delete_mappoint_indices表示地图点的索引集合

        void Clear();//清除地图信息，地图点和帧信息及储存的变量，其余设置仍然保留

        /**
         * 实现滑动窗口检验单帧三维点是否为误提点
         * @param current_frame_id 当前帧ID
         * @param front_frames 全局地图中头部靠前的帧数不适用此方法
         * @param back_frames 全局地图中尾部靠后的帧数不适用此方法
         * @param window_size 滑动窗口区间大小
         * @param count 地图点在滑动窗口内的计数器阈值
         */
        void VerifyMapPointInSlidingWindow(int frontback_frames = 3, int window_size = 10, int count = 3);  //检验当前地图中地图点是否为可靠的点

        /**
         * 实现已存在的帧位姿改变后对地图点新坐标的求解
         * @param frame_id 选定帧ID
         * @param rt_ori 选定帧的旧位姿
         * @param rt_new 选定帧的新位姿
         * @param map_points_temp 临时的地图点坐标的集合，size大小与全局地图中地图点一致
         * @param map_points_delta 地图点坐标值改变量，size大小与全局地图中地图点一致
         */
        void UpdateMapPointByExistentFrame(int frame_id,
            const rulermvs::Mat44& rt_ori, const rulermvs::Mat44& rt_new,
            std::vector<MapPoint>& map_points_temp,
            std::vector<rulermvs::Point3d>& map_points_delta);

        /**
         * 遍历地图点，将欧氏距离符合阈值要求的未合并同名点合并到其真实的同名点，并更新合并后同名点的坐标值、地图点观测信息、帧观测信息、corner匹配地图点信息更和删减子点索引集合，删除掉被合并的同名点
         */
        void UpdateObservationWithMapPoint();

        /**
         * 重新计算所有地图点的子点（存在父点的地图点不参与是否为子点的计算），只更改father_id_和sons_id_，不包括地图点合并后的删减
         */
        void RecalculateSonsIDForAll();

        /**
         * 根据帧内观测的地图点信息重新计算帧的位姿
         * @param frame 必须包含观测信息
         * @param rt 新的位姿（输出）
         * @param min_match_num 与全局地图的最小同名点数量
         */
        bool RecalculatePoseForFrame(
            Frame& frame, rulermvs::Mat44& rt, int min_match_num = 3);

        /**
         * 子点更新父点（递归函数，子点的子点----），包括坐标值、地图点观测信息、帧观测信息、corner匹配地图点信息更新和子点索引集合删减
         * @param map_id 地图点ID
         * @param delete_mappoint_indices 删减地图点的索引，由于其对应的观测信息已被合并，其对应的地图点观测信息、父子点索引不会被更新
         */
        bool UpdateMappointBySons(
            const int& map_id, std::vector<int>& delete_mappoint_indices);

        std::vector<int> new_mappoint_id_;//GlobalMap更新后新增的MapPoint的id
        std::vector<std::shared_ptr<T1>> points3d_sharedPointer_;//存储共享指针，与非零3d点数量相等，使用unique_ptr会报错
        //T1List points3d_Pointer_;//点集，存储指针，与非零3d点数量相等
        std::map<int, int> index_map_;//索引对应关系，非零point3d索引————corner角点索引
        std::vector<cv::Point3d> points3d_;//存储3d点，与corner数量相等，包括三零point3d
        T2DistMapind siteLenMap_;//帧内三维点的距离特征（元素分别为：两点距离，两点索引对，两点相对于帧内第一个点的地址对）
        bool savePointer = true;//是否保存上述指针及共享指针

        double MAX_PTBIAS_ERROR = 0.3;//同名点间的均方根差，相邻帧的该值对于下一帧的误匹配筛选具有重要意义

        FrameMatchMode match_mode_ = FrameMatchMode::match_globalMap_direct;//默认为直接与全局地图对齐
        int match_adjacentframe_nums_ = 1;//与相邻帧对齐时，默认只选取一帧，只与全局地图对齐时该参数没有意义
        bool reproject_KeyFrame_ = false;//是否将全局地图点重投影到关键帧生成新的帧

        std::vector<int> accumulated_mappont_id_;//自最后一次BA优化后新增的地图点id
        std::vector<int> accumulated_frame_id_;//自最后一次BA优化后新增的帧id
        int BA_min_added_mappoints_ = 0;//BA优化要求的最小新增地图点数量
        int BA_min_added_frames_ = 1;//BA优化要求的最小新增Frame数量
    };

}  // namespace slam
#endif