#ifndef _SLAM_TRACKER_FRAME_HPP_
#define _SLAM_TRACKER_FRAME_HPP_
#include "rulermvs/corner.hpp"
#include "rulermvs/CameraTracker.hpp"

namespace slam
{
struct FrameObservation
    {
        FrameObservation(int map_point_id, int corner_id)
        {
            map_point_id_ = map_point_id;
            corner_id_ = corner_id;
        }
        int map_point_id_;
        int corner_id_;
    };

typedef cv::Point3d T1;
typedef cv::Point3d T2;

typedef std::vector<T1*>         T1List;
typedef std::pair<T1*, T1*>      T1Pair;
typedef std::vector<T1Pair>      T1PairList;
typedef std::map<double, T1Pair> T1DistMap;

typedef std::vector<T2*>    T2List;
typedef std::pair<T2*, T2*> T2Pair;
typedef std::vector<T2Pair> T2PairList;

typedef std::pair<T1*, T2*>   T1T2Pair;
typedef std::vector<T1T2Pair> T1T2PairList;

typedef std::map<double, T2Pair> T2DistMap;
typedef std::vector<std::tuple<double, T2Pair, std::pair<int, int>>> T2DistMapind;
typedef std::tuple<double, T2Pair, std::pair<int, int>> T2DistMapind_tuple;
// typedef std::map<T1*, std::pair<KeyPoint*, KeyPoint*>> T1KeyPairMap;
// typedef std::map<T2*, std::pair<KeyPoint*, KeyPoint*>> T2KeyPairMap;

class MVS_EXPORT Frame
{
public:
    Frame() {}

    //Frame(int idx, Camera* camera, rulermvs::RGBDData rgbd_data,
    //    cv::Mat& rgb, cv::Mat& depth,
    //    cv::Mat& vmap, cv::Mat& nmap, cv::Mat& mask,
    //    double depth_sacle_, double resolution_scale_, double interpolat_thres, std::string output_dir);
    Frame(int idx, Camera* camera, rulermvs::RGBDData rgbd_data_small, rulermvs::RGBDData rgbd_data,
        cv::Mat& rgb, cv::Mat& depth,
        cv::Mat& vmap, cv::Mat& nmap, cv::Mat& mask,
        double depth_sacle_, double resolution_scale_, double interpolat_thres, std::string output_dir);

    Frame(int idx, Camera* camera, std::string rgb_file, std::string depth_file, std::string output_dir, double depth_sacle_ = 32.0f, double resolution_scale_ = 2.0f);

    virtual ~Frame() {}

    //设置对应的点位外点
    void SetOutliers(std::vector<int>& corner_indexes);

    //添加观测的3d-2d的对应
    void AddObservation(int map_point_id, int corner_id);

    //删除观测
    bool DeleteObservation(int map_point_id);

    //设置位姿
    void SetPose(rulermvs::Mat44& pose_w2c);

    void unprojectStereo(cv::Mat& _Kl, cv::Mat& _Kr, cv::Mat& _R, cv::Mat& _t, cv::Mat& x2DL, cv::Mat& x2DR, cv::Point3d* p3d);

    //重置当前帧
    void ResetFrame();

    std::set<int> GetConvisibleKeyFrameIdxes();//共视图未使用

    void UpdateConvisibleGraph();//共视图未使用

    void UpdatePointerAndDistMap(double maxP3dDist = 500.0);//更新帧内的point3d的指针以及点对间的dist
public:
    int input_id_;

    int id_;
    
    //rulermvs::Mat44 pose_w2c_ = rulermvs::Mat44::Identity();// the pose matrix:world to camera
    //rulermvs::Mat44 pose_c2w_ = rulermvs::Mat44::Identity();// the pose matrix:camera to world

    cv::Mat pose_w2c_ = cv::Mat::eye(4, 4, CV_64F);// the pose matrix:world to camera
    cv::Mat pose_c2w_ = cv::Mat::eye(4, 4, CV_64F);// the pose matrix:camera to world

    std::vector<rulermvs::Corner> corners_;       // the key point corner
    std::vector<rulermvs::CornerPair> cornerpairs_;        // the key point corner pair

    rulermvs::Vec3d camera_center_w_;

	rulermvs::Vec3d plan_o = rulermvs::Vec3d(0.0, 0.0, 0.0);//平面中心点

	rulermvs::Vec3d plan_n = rulermvs::Vec3d(0.0, 0.0, 0.0);//平面法线

    bool is_detect_plan = false;

    double plan_distance;

    int /*matched_mappoint_count*/ MatchedMappointCount();      //the count of corner that has matched map point

    int HasDepthCornerCount();      //the count of corner that has matched map point

    std::vector<FrameObservation> observations_;    // the vector of the corner and map point pair that match

    //std::map<int,int> observations_;    // the vector of the corner and map point pair that match

    Camera* camera_;  // the camera pointer 
    Camera* camera_R;  // the right camera pointer 

    bool is_keyframe_ = false;            //the tag that if the current frame is a key frame or not

    cv::Mat image_;                     //the rgb image for debug 

    //cv::Mat depth_;                     //the depth image for debug 

    //cv::Mat nmap_;                      //the nmap image for debug 

    bool is_bad_ = false;              //if the frame is deleted or not 

    bool err_too_big_ = false;

    bool is_relocation = false;

    std::map<int, int> convisible_frame_idx_count_map_;

    double optimize_error = 0;

    std::string output_dir_;

    rulermvs::RGBDData rgbd_data;
    rulermvs::RGBDData rgbd_data_small;
    //GlobalMap* global_map_ptr_;

    std::vector<std::shared_ptr<T1>> points3d_sharedPointer_;//存储共享指针，与非零3d点数量相等，使用unique_ptr会报错
    //T1List points3d_Pointer_;  //点集，存储指针，与非零3d点数量相等
    std::map<int, int> index_map_;//索引对应关系，非零point3d索引————corner角点索引
    std::vector<cv::Point3d> points3d_;//存储3d点，与corner数量相等，包括三零point3d
    T2DistMapind siteLenMap_;//帧内三维点的距离特征（元素分别为：两点距离，两点索引对，两点相对于帧内第一个点的地址对）
    //bool joined_globalmap_ = false;//是否已加入到全局地图，默认false

private:

    void UndistortCorners();
};

}  // namespace rulermvs
#endif