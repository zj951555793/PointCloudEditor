#ifndef _SLAM_TRACKER_TRACKER_HPP_
#define _SLAM_TRACKER_TRACKER_HPP_
#include "rulermvs/corner.hpp"
#include "rulermvs/Frame.hpp"
#include "rulermvs/GlobalMap.hpp"

namespace slam
{
#define KEYFRAME_INTERVAL 15
    enum TrackState
    {
        INIT,
        TRACKING,
        TRACKING_BY_REF_FRAME_SUCCESS,
        TRACKING_BY_KEY_FRAME_SUCCESS,
        TRACKING_BY_RELOC_SUCCESS,
        TRACKING_FAILED
    };

    class MVS_EXPORT Tracker
    {
    public:
        Tracker() : global_map_ptr_(nullptr) {}
        ~Tracker() {}

        //跟踪初始化
        bool Init(Frame& frame);

        //跟踪一帧
        bool Track(Frame& frame, cv::Mat& K, bool is_relocation);

        //两帧的匹配（手持）
        std::pair<bool, float> MatchByFrame(Frame& reference_frame, Frame& current_frame, Eigen::Matrix4d& reference_to_current_rt, cv::Mat& K, double site_diff_thres, double p3d_match_thres, int min_match_num, bool modify_frame_state = true, bool save_match_img = true);

        //两帧的匹配
		bool MatchByFrame(GlobalMap& grobal_map, Frame& reference_frame, Frame& current_frame, rulermvs::Mat44& reference_to_current_rt, cv::Mat& K, double site_diff_thres, double p3d_match_thres, int min_match_num, bool modify_frame_state = true, bool save_match_img = true);
        //帧与全局地图点匹配
        bool MatchByMap(GlobalMap& grobal_map, Frame& current_frame, rulermvs::Mat44& reference_to_current_rt, cv::Mat& K, double site_diff_thres, double p3d_match_thres, int min_match_num, bool modify_frame_state = true, bool save_match_img = true);
        //两个地图的匹配
        bool MatchMapAndMap(GlobalMap& grobal_mapA, GlobalMap& grobal_mapB, rulermvs::Mat44& reference_to_current_rt, double site_diff_thres, double p3d_match_thres, int min_match_num, bool modify_frame_state = true, bool save_match_img = true);

        int  reference_frame_idx_ = -1;

        int reference_key_frame_idx_ = -1;

        TrackState track_state_ = TrackState::INIT;
        TrackState last_track_state = TrackState::INIT;

        GlobalMap* global_map_ptr_ = nullptr;
		
        std::string output_dir_;

        int max_triMatched_counter_ = 6; //参考帧与当前帧计算RT时，会根据路标point构造三角形，当三角形匹配成功次数计数器超过该值时停止三角形匹配

        bool mintree_ = false;

        //设置参数
		void Setparameter(double site_diff_thres, double p3d_match_thres, double p3d_match_thres_online, double p3d_match_thres_relocation);

    private:
        //void ExpandMapPoints(Frame& key_frame);

        //找到更多匹配地图点
        int FindMoreMatchedMapPoint(GlobalMap& globalmap, Frame& frame, rulermvs::Mat44& pose4x4, double distance_threashold);

        bool TrackTwoFrames(Frame& reference_frame, Frame& current_frame, rulermvs::Mat44& relate_rt);

        //重定位
		bool Relocalization(Frame& frame, int& ref_key_frame_idx, cv::Mat& K, double site_diff_thres, double p3d_match_thres, int min_match_num, bool is_relocation);

        //判断是否需要插入新的关键帧
        bool NeedKeyFrame(Frame& frame);

		bool NeedglobalKeyFrame(Frame& frame);

		bool odometry(Frame& current_frame, cv::Mat& K, int min_match_num);

        int CaulateSameMatchedCount(Frame& reference_frame, Frame& current_frame);

        double site_diff_thres_;

        double p3d_match_thres_;// P3D_MATCH_THRES

		double p3d_match_thres_online_;// P3D_MATCH_THRES
		double p3d_match_thres_relocation_;// P3D_MATCH_THRES
    };

    static inline void BuildFrameFrom3dPoints(const std::vector<rulermvs::Point3d>& points, Frame& frame, const rulermvs::Pose& pose, Camera* camera = nullptr, cv::Mat image = cv::Mat()) 
    {
        frame.corners_.resize(points.size());
        std::vector<rulermvs::Corner>& corners = frame.corners_;
        for (int i = 0; i < corners.size(); i++) 
        {
            corners[i].has_depth = true;
            corners[i].point3d_c_ = rulermvs::Vec3d(points[i].x, points[i].y, points[i].z);
            corners[i].id_ = i;
        }
        cv::Mat mat_temp = cv::Mat::eye(4, 4, CV_64F);
        rulermvs::Mat44 relate_rt = rulermvs::Mat44::Identity();
        mat_temp.at<double>(0, 0) = pose.a1;
        mat_temp.at<double>(0, 1) = pose.a2;
        mat_temp.at<double>(0, 2) = pose.a3;
        mat_temp.at<double>(1, 0) = pose.b1;
        mat_temp.at<double>(1, 1) = pose.b2;
        mat_temp.at<double>(1, 2) = pose.b3;
        mat_temp.at<double>(2, 0) = pose.c1;
        mat_temp.at<double>(2, 1) = pose.c2;
        mat_temp.at<double>(2, 2) = pose.c3;
        mat_temp.at<double>(3, 0) = pose.x;
        mat_temp.at<double>(3, 1) = pose.y;
        mat_temp.at<double>(3, 2) = pose.z;
        std::memcpy(relate_rt.data(), mat_temp.data, relate_rt.size() * sizeof(double));  // Eigen是列存储，不同于cv中的Mat
        frame.SetPose(relate_rt);
        if (camera != nullptr)
        {
            frame.camera_ = camera;
        }
        if (!image.empty()) 
        { 
            frame.image_ = image.clone(); 
        }
    }

    static inline void BuildFrameFromCorners(const std::vector<rulermvs::Corner>& corners, Frame& frame, const rulermvs::Pose& pose, Camera* camera = nullptr, cv::Mat image = cv::Mat()) 
    {
        frame.corners_ = corners;
        for (int i = 0; i < frame.corners_.size(); i++) {
            frame.corners_[i].has_depth = true;
            frame.corners_[i].id_ = i;
        }
        cv::Mat mat_temp = cv::Mat::eye(4, 4, CV_64F);
        rulermvs::Mat44 relate_rt = rulermvs::Mat44::Identity();
        mat_temp.at<double>(0, 0) = pose.a1;
        mat_temp.at<double>(0, 1) = pose.a2;
        mat_temp.at<double>(0, 2) = pose.a3;
        mat_temp.at<double>(1, 0) = pose.b1;
        mat_temp.at<double>(1, 1) = pose.b2;
        mat_temp.at<double>(1, 2) = pose.b3;
        mat_temp.at<double>(2, 0) = pose.c1;
        mat_temp.at<double>(2, 1) = pose.c2;
        mat_temp.at<double>(2, 2) = pose.c3;
        mat_temp.at<double>(3, 0) = pose.x;
        mat_temp.at<double>(3, 1) = pose.y;
        mat_temp.at<double>(3, 2) = pose.z;
        std::memcpy(relate_rt.data(), mat_temp.data, relate_rt.size() * sizeof(double));  // Eigen是列存储，不同于cv中的Mat
        frame.SetPose(relate_rt);
        if (camera != nullptr)
        {
            frame.camera_ = camera;
        }
        if (!image.empty()) 
        { 
            frame.image_ = image.clone(); 
        }
    }

    static inline void BuildFrameFromCornerPairs(const std::vector<rulermvs::CornerPair>& cornerpairs, Frame& frame, const rulermvs::Pose& pose, Camera* camera1 = nullptr, Camera* camera2 = nullptr, cv::Mat image = cv::Mat()) 
    {
        frame.corners_.resize(cornerpairs.size());
        frame.cornerpairs_ = cornerpairs;
        std::vector<rulermvs::Corner>& corners = frame.corners_;
        for (int i = 0; i < corners.size(); i++) 
        {
            corners[i] = cornerpairs[i].first;
            corners[i].has_depth = true;
            corners[i].id_ = i;
        }
        cv::Mat mat_temp = cv::Mat::eye(4, 4, CV_64F);
        rulermvs::Mat44 relate_rt = rulermvs::Mat44::Identity();
        mat_temp.at<double>(0, 0) = pose.a1;
        mat_temp.at<double>(0, 1) = pose.a2;
        mat_temp.at<double>(0, 2) = pose.a3;
        mat_temp.at<double>(1, 0) = pose.b1;
        mat_temp.at<double>(1, 1) = pose.b2;
        mat_temp.at<double>(1, 2) = pose.b3;
        mat_temp.at<double>(2, 0) = pose.c1;
        mat_temp.at<double>(2, 1) = pose.c2;
        mat_temp.at<double>(2, 2) = pose.c3;
        mat_temp.at<double>(3, 0) = pose.x;
        mat_temp.at<double>(3, 1) = pose.y;
        mat_temp.at<double>(3, 2) = pose.z;
        std::memcpy(relate_rt.data(), mat_temp.data, relate_rt.size() * sizeof(double));  // Eigen是列存储，不同于cv中的Mat
        frame.SetPose(relate_rt);
        if (camera1 != nullptr)
        {
            frame.camera_ = camera1;
        }
        if (camera2 != nullptr)
        {
            frame.camera_R = camera2;
        }
        if (!image.empty()) 
        { 
            frame.image_ = image.clone(); 
        }
    }

    /// @brief 跟踪新加入的当前帧(单帧)，输出RT(相对于tracker中的global_map)
    /// @param tracker 跟踪器，tracker中的global_map_ptr_指针使用前必须初始化(tracker.global_map_ptr_ = new slam::GlobalMap()或简单的赋值)
    /// @param current_frame 当前帧，包含corner、point3d、RT(输入时未用到)等信息，输出时的成员变量RT与下方参数rt表达形式不一样实质相同
    /// @param rt 当前帧跟踪global_map成功后得到的RT，当前帧的帧内三维点相对于global_map中的map_points
    /// @param min_match_num 最小同名点数量
    /// @param site_diff_thres 两对同名点之间的线段距离差阈值
    /// @param p3d_match_thres 同名点匹配对在RT作用后两者之间的距离阈值
    /// @param p3d_nearest_thres 求解RT后合并单帧到global_map时，未参与匹配的同名点在同名点在RT作用后两者间的距离阈值，大于p3d_match_thres
    /// @param match_adjacentFrame 当前帧是否与相邻帧(global_map的最后一帧)对齐，false表示否定
    /// @param match_globalmap 当前帧是否与global_map全局地图对齐，false表示否定
    /// @param insert_newframe 对齐成功或初始化成功是否将当前帧添加到global_map中，false表示否定
    static inline int getCurrentFrameLocation(Frame& current_frame,
        Tracker& tracker, rulermvs::Pose& rt, int min_match_num = 3, double site_diff_thres = 1.0,
        double p3d_match_thres = 1.5, double p3d_nearest_thres = 3.0, bool match_adjacentFrame = true, bool match_globalmap = true, bool insert_newframe = true)
    {
        int depth_num = 0;
        for (int i = 0; i < current_frame.corners_.size(); i++)
        {
            if (current_frame.corners_[i].has_depth)
            { 
                depth_num++;
            }
        }
        //第一帧初始化,添加地图点
        if (tracker.track_state_ == slam::TrackState::INIT)
        {
            if (depth_num >= min_match_num)
            {
                tracker.global_map_ptr_->p3d_match_thres_ = p3d_nearest_thres;
                if (tracker.global_map_ptr_->savePointer) {
                    current_frame.UpdatePointerAndDistMap(500.0);
                }
                if (insert_newframe)
                {
                    tracker.global_map_ptr_->InsertFrame(current_frame, true);
                }
                tracker.global_map_ptr_->UpdatePointerAndDistMap(tracker.global_map_ptr_->new_mappoint_id_, 500.0, false);
                std::cout << "init success. " << "\t";
                tracker.track_state_ = slam::TrackState::TRACKING;
            }
            else
            {
                std::cout << "init failed. " << "\t";
            }
            return tracker.track_state_;
        }
        if (depth_num < min_match_num) 
        {
#ifndef NDEBUG
            std::cout << "ref failed. " << "\t";
            std::cout << "There are not enough Marker Points in this frame. " << "\t";
#endif
            return slam::TrackState::TRACKING_FAILED;
        }

        //里程计
        rulermvs::Mat44 relate_rt = rulermvs::Mat44::Identity();
        bool matched = false;
        if (tracker.global_map_ptr_->savePointer) {
            current_frame.UpdatePointerAndDistMap(500.0);
        }
        //帧与帧
        if (match_adjacentFrame && tracker.MatchByFrame(*(tracker.global_map_ptr_), tracker.global_map_ptr_->frames_.back(), current_frame, relate_rt, current_frame.camera_->K_, site_diff_thres, p3d_match_thres, min_match_num)) // match last reference frame
        {
            matched = true;
            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> pose_w2c(tracker.global_map_ptr_->frames_.back().pose_w2c_.ptr<double>(0), 4, 4);
            rulermvs::Mat44 rt_temp = relate_rt * pose_w2c;
            relate_rt = rt_temp;
            if (insert_newframe)
            {
                tracker.global_map_ptr_->p3d_match_thres_ = p3d_nearest_thres * 0.8;
                if (tracker.global_map_ptr_->frames_.size() > 1)
                {
                    tracker.global_map_ptr_->InsertFrame(current_frame, true, false);
                } else {
                    tracker.global_map_ptr_->InsertFrame(current_frame, true, true);
                }
            }
        }
        //帧与全局地图点匹配 
        else if(match_globalmap && (!match_adjacentFrame || tracker.global_map_ptr_->frames_.size() > 1) && tracker.MatchByMap(*(tracker.global_map_ptr_), current_frame, relate_rt, current_frame.camera_->K_, site_diff_thres, p3d_match_thres, min_match_num)) 
        {
            matched = true;
            if (insert_newframe) {
                tracker.global_map_ptr_->p3d_match_thres_ = p3d_nearest_thres;            
                tracker.global_map_ptr_->InsertFrame(current_frame, true, false);
            }
        }
        if (matched)
        {
            tracker.global_map_ptr_->UpdatePointerAndDistMap(tracker.global_map_ptr_->new_mappoint_id_, 500.0, false);
#ifndef NDEBUG
            std::cout << "ref success. " << "\t";
#endif
            tracker.track_state_ = slam::TrackState::TRACKING_BY_REF_FRAME_SUCCESS;
            cv::Mat mat_temp = cv::Mat::eye(4, 4, CV_64F);
            //cv::eigen2cv(relate_rt, mat_temp);
            std::memcpy(mat_temp.data, relate_rt.data(), relate_rt.size() * sizeof(double));//Eigen是列存储，不同于cv中的Mat
            rt.a1 = mat_temp.at<double>(0, 0);
            rt.a2 = mat_temp.at<double>(0, 1);
            rt.a3 = mat_temp.at<double>(0, 2);
            rt.b1 = mat_temp.at<double>(1, 0);
            rt.b2 = mat_temp.at<double>(1, 1);
            rt.b3 = mat_temp.at<double>(1, 2);
            rt.c1 = mat_temp.at<double>(2, 0);
            rt.c2 = mat_temp.at<double>(2, 1);
            rt.c3 = mat_temp.at<double>(2, 2);
            rt.x = mat_temp.at<double>(3, 0);
            rt.y = mat_temp.at<double>(3, 1);
            rt.z = mat_temp.at<double>(3, 2);
        } 
        else 
        {
#ifndef NDEBUG
            std::cout << "ref failed. " << "\t";
#endif
            tracker.track_state_ = slam::TrackState::TRACKING_FAILED;
        }
        return tracker.track_state_;
    }

    static inline int getCurrentFrameLocation(std::vector<rulermvs::Corner>& corners, Tracker& tracker, rulermvs::Pose& rt, 
        Camera* camera, cv::Mat image = cv::Mat(), int min_match_num = 3,
        double site_diff_thres = 1.0, double p3d_match_thres = 1.5, double p3d_nearest_thres = 3.0, bool match_adjacentFrame = true)
    {
        if (corners.size() < min_match_num)
        {
            if (tracker.track_state_ == slam::TrackState::INIT)
            {
                std::cout << "init failed. " << "\t";
                return tracker.track_state_;
            }
            std::cout << "ref failed. " << "\t";
            return slam::TrackState::TRACKING_FAILED;
        }
        Frame frameCur;
        frameCur.corners_ = corners;
        rulermvs::Mat44 mat_temp = rulermvs::Mat44::Identity();
        frameCur.SetPose(mat_temp);
        frameCur.image_ = image.clone();
        frameCur.camera_ = camera;
        return getCurrentFrameLocation(frameCur, tracker, rt, min_match_num, site_diff_thres, p3d_match_thres, p3d_nearest_thres, match_adjacentFrame);
    }

    /// @brief 跟踪新加入的当前帧(多帧)，输出RT(相对于global_map)
    /// @param global_map_ 全局地图，包含MapPoint和Frame，若传入为空，函数内部根据参考帧(自带RT)重新构建global_map，若不为空，默认为global_map已存在，不再构造直接沿用
    /// @param rt 当前帧跟踪global_map成功后得到的RT，当前帧的三维地图点相对于global_map中的map_points
    /// @param reference_frames 参考帧集合，包含corner、point3d、RT等信息，若global_map_不为空，该参数不会被使用，但禁止传入空的集合，且size与rts1的size保持一致
    /// @param current_frames 当前帧集合，包含corner、point3d、RT等信息，单帧直接求解RT然后合并到global_map，多帧内部构造新的global_map，再求解RT后合并多帧到旧的global_map
    /// @param rts1 参考帧(多帧)的RT(局部同时是全局)，定值
    /// @param rts2 当前帧(多帧)的RT(仅仅是局部)，定值
    /// @param min_match_num 最小同名点数量
    /// @param site_diff_thres 两对同名点之间的线段距离差阈值
    /// @param p3d_match_thres 同名点匹配对在RT作用后两者之间的距离阈值
    /// @param p3d_nearest_thres 求解RT后合并单帧到global_map时，未参与匹配的同名点在同名点在RT作用后两者间的距离阈值，大于p3d_match_thres
    /// @param match_adjacentFrame 当前帧是否先尝试与相邻帧(global_map的最后一帧)对齐，false表示直接与global_map对齐(仅对当前帧是单帧起作用)
    static inline bool matchMarkerPoint(GlobalMap& global_map_, rulermvs::Pose& rt, const std::vector<std::vector<rulermvs::Point3d>>& reference, const std::vector<rulermvs::Pose>& rts1, 
        const std::vector<std::vector<rulermvs::Point3d>>& current, const std::vector<rulermvs::Pose>& rts2,
        int min_match_num = 3, double site_diff_thres = 1.0, double p3d_match_thres = 1.5, double p3d_nearest_thres = 3.0, bool match_adjacentFrame = true)
    {
        assert(reference.size() == rts1.size() && current.size() == rts2.size());
        std::vector<Frame> reference_frames(reference.size()), current_frames(current.size());
        std::vector<int> new_mappoint_id;
        //判断全局globalmap是否存在，存在不用重新构建globalmap（包含了去除同名点过程）
        if (global_map_.map_points_.empty())
        {
            global_map_.p3d_match_thres_ = p3d_nearest_thres;
            for (int i = 0; i < reference.size(); i++) 
            {
                BuildFrameFrom3dPoints(reference[i], reference_frames[i], rts1[i]);
                //reference_frames[i].UpdatePointerAndDistMap(500.0);
                global_map_.InsertFrame(reference_frames[i], true);
                new_mappoint_id.insert(new_mappoint_id.end(), global_map_.new_mappoint_id_.begin(), global_map_.new_mappoint_id_.end());
            }
            global_map_.UpdatePointerAndDistMap(new_mappoint_id, 500.0, true);
        }
        //else//否则直接沿用已有的globalmap 
        //{
        //}
        Tracker tracker;
        tracker.global_map_ptr_ = &global_map_;  // tracker中的global_map_ptr_指针使用前必须初始化
        if (tracker.global_map_ptr_->map_points_.size() >= min_match_num)//地图点大于一定数量
        {
            tracker.track_state_ = slam::TrackState::TRACKING;
        }
        new_mappoint_id.clear();
        for (int i = 0; i < current.size(); i++) 
        {
            BuildFrameFrom3dPoints(current[i], current_frames[i], rts2[i]);
        }
        if (current_frames.size() == 1)
        { 
            int track_state = getCurrentFrameLocation(current_frames[0], tracker, rt, min_match_num, site_diff_thres, p3d_match_thres, p3d_nearest_thres, match_adjacentFrame);
            if (track_state == 2)
            {
                return true;
            } else
            {
                return false;
            }
        } 
        else
        {
            GlobalMap current_globalmap;
            current_globalmap.savePointer = global_map_.savePointer;
            current_globalmap.p3d_match_thres_ = p3d_nearest_thres;
            for (int i = 0; i < current_frames.size(); i++) 
            {
                if (current_globalmap.savePointer) {
                    current_frames[i].UpdatePointerAndDistMap(500.0);
                }
                current_globalmap.InsertFrame(current_frames[i], true);
                new_mappoint_id.insert(new_mappoint_id.end(), current_globalmap.new_mappoint_id_.begin(), current_globalmap.new_mappoint_id_.end());
            }
            current_globalmap.UpdatePointerAndDistMap(new_mappoint_id, 500.0, true);
            rulermvs::Mat44 relate_rt = rulermvs::Mat44::Identity();
            if (tracker.MatchMapAndMap(global_map_, current_globalmap, relate_rt, site_diff_thres, p3d_match_thres, min_match_num)) 
            {
                for (int k = 0; k < current_globalmap.frames_.size(); k++) 
                {
                    global_map_.InsertFrame(current_globalmap.frames_[k], true, false);
                    global_map_.UpdatePointerAndDistMap(global_map_.new_mappoint_id_, 500.0, false);
                }
                cv::Mat mat_temp = cv::Mat::eye(4, 4, CV_64F);
                std::memcpy(mat_temp.data, relate_rt.data(), relate_rt.size() * sizeof(double));//Eigen是列存储，不同于cv中的Mat
                rt.a1 = mat_temp.at<double>(0, 0);
                rt.a2 = mat_temp.at<double>(0, 1);
                rt.a3 = mat_temp.at<double>(0, 2);
                rt.b1 = mat_temp.at<double>(1, 0);
                rt.b2 = mat_temp.at<double>(1, 1);
                rt.b3 = mat_temp.at<double>(1, 2);
                rt.c1 = mat_temp.at<double>(2, 0);
                rt.c2 = mat_temp.at<double>(2, 1);
                rt.c3 = mat_temp.at<double>(2, 2);
                rt.x = mat_temp.at<double>(3, 0);
                rt.y = mat_temp.at<double>(3, 1);
                rt.z = mat_temp.at<double>(3, 2);
                return true;
            }
            return false;
        }
    }

    /// @brief 对选取的global_map_中的关键帧进行BA优化，减少error和pixel_error，同步优化RT和MapPoint
    /// @param global_map_ 全局地图，包含MapPoint和Frame
    /// @param frame_index 待优化帧在global_map_中的索引
    /// @param iter_Num BA优化迭代次数
    /// @param stereo_vision 是否开启双目BA优化，默认为单目优化
    /// @param singleMapPoint_optimize 是否将只有一个Observation的MapPoint加入到BA优化中
    /// @param with_roundness BA优化时是否将corner的属性roundness大小作为影响residual的因素
    /// @param recompute_mappoint BA优化之前是否根据待优化帧重新加权计算mappoint
    /// @param acc_before BA优化前的误差计算及保存路径
    /// @param acc_after BA优化后的误差计算及保存路径
    MVS_EXPORT void BundleAdjustment(GlobalMap& global_map_, std::vector<int>& frame_index, int iter_Num = 10, bool stereo_vision = false, bool singleMapPoint_optimize = true, bool with_roundness = false, bool recompute_mappoint = true, const std::string acc_before = "", const std::string acc_after = "");

    /// @param frame_indexes_for_optimize 待优化帧在global_map_中的索引
    /// @param valid_index track成功的帧在所有原始帧中索引
    /// @param rts 所有原始帧(track成功及失败)中的RT
    static inline void BundleAdjustment(GlobalMap* global_map_ptr, std::vector<int>& frame_indexes_for_optimize,
        std::vector<rulermvs::Pose>& rts, const std::vector<int> valid_index, int iter_Num = 10, bool stereo_vision = false, bool singleMapPoint_optimize = true, bool with_roundness = false, bool recompute_mappoint = true, const std::string acc_before = "", const std::string acc_after = "") 
    {
        assert(global_map_ptr->frames_.size() == valid_index.size());
        BundleAdjustment(*global_map_ptr, frame_indexes_for_optimize, iter_Num, stereo_vision, singleMapPoint_optimize, with_roundness, recompute_mappoint, acc_before, acc_after);
        for (int i = 0; i < valid_index.size(); i++)
        { 
            rulermvs::Pose& RT = rts[valid_index[i]];
            auto& relate_rt = global_map_ptr->frames_[i].pose_w2c_;
            //cv::Mat mat_temp = cv::Mat::eye(4, 4, CV_64F);
            //std::memcpy(mat_temp.data, relate_rt.data(), relate_rt.size() * sizeof(double));//Eigen是列存储，不同于cv中的Mat
            //RT.a1 = mat_temp.at<double>(0, 0);
            //RT.a2 = mat_temp.at<double>(0, 1);
            //RT.a3 = mat_temp.at<double>(0, 2);
            //RT.b1 = mat_temp.at<double>(1, 0);
            //RT.b2 = mat_temp.at<double>(1, 1);
            //RT.b3 = mat_temp.at<double>(1, 2);
            //RT.c1 = mat_temp.at<double>(2, 0);
            //RT.c2 = mat_temp.at<double>(2, 1);
            //RT.c3 = mat_temp.at<double>(2, 2);
            //RT.x  = mat_temp.at<double>(3, 0);
            //RT.y  = mat_temp.at<double>(3, 1);
            //RT.z  = mat_temp.at<double>(3, 2);
            RT.a1 = relate_rt.at<double>(0, 0);
            RT.a2 = relate_rt.at<double>(1, 0);
            RT.a3 = relate_rt.at<double>(2, 0);
            RT.b1 = relate_rt.at<double>(0, 1);
            RT.b2 = relate_rt.at<double>(1, 1);
            RT.b3 = relate_rt.at<double>(2, 1);
            RT.c1 = relate_rt.at<double>(0, 2);
            RT.c2 = relate_rt.at<double>(1, 2);
            RT.c3 = relate_rt.at<double>(2, 2);
            RT.x  = relate_rt.at<double>(0, 3);
            RT.y  = relate_rt.at<double>(1, 3);
            RT.z  = relate_rt.at<double>(2, 3);
        }
    }

    /// @brief 跟踪新加入的当前帧(多帧)，输出RT，同时对加入新帧的global_map进行BA优化，缩小同名点偏差
    /// @param global_map_ 全局地图，包含MapPoint和Frame，若传入为空，函数内部根据参考帧(自带RT)重新构建global_map并BA优化，若不为空，默认为是经过BA优化的global_map，不再构造直接沿用
    /// @param rt 当前帧跟踪global_map成功后得到的RT，当前帧的三维地图点相对于global_map中的map_points
    /// @param reference_frames 参考帧集合，包含corner、point3d、RT等信息，若global_map_不为空，该参数不会被使用
    /// @param current_frames 当前帧集合，包含corner、point3d、RT等信息，单帧直接求解RT然后合并后BA优化，多帧内部构造global_map然后BA优化，再求解RT后合并多帧后再BA优化
    /// @param min_match_num 最小同名点数量
    /// @param site_diff_thres 两对同名点之间的线段距离差阈值
    /// @param p3d_match_thres 同名点匹配对在RT作用后两者之间的距离阈值
    /// @param p3d_nearest_thres 求解RT后合并单帧到global_map时，未参与匹配的同名点在同名点在RT作用后两者间的距离阈值，大于p3d_match_thres
    /// @param iter_Num BA优化迭代次数
    /// @param match_adjacentFrame 当前帧是否先尝试与相邻帧(global_map的最后一帧)对齐，false表示直接与global_map对齐(仅对当前帧是单帧起作用)(已废弃)
    /// @param insert_newframe 对齐成功或初始化成功是否将当前帧添加到global_map中，false表示否定
    MVS_EXPORT bool matchMarkerPointAndOptimizeMultiGroupFrames(
        GlobalMap& global_map_, rulermvs::Pose& rt,
        std::vector<Frame>& reference_frames,
        std::vector<Frame>& current_frames, int min_match_num = 3,
        double site_diff_thres = 1.0, double p3d_match_thres = 1.5,
        double p3d_nearest_thres = 3.0, int iter_Num = 10,
        bool insert_newframe = true, int max_triMatched_counter = 6,
        bool mintree = false);

    /// @brief 新加入的当前帧，与所有关键帧匹配，检验是否由于累计误差导致回环拼接错误，适用于在线检验，速度较慢且不完善
    MVS_EXPORT void InsertFrameByRecalculateObservationsAndMatchKeyFrames(
        GlobalMap& global_map_, Frame& frame, const cv::Mat& rt,
        std::vector<int>& key_inds, int min_match_points_num = 4,
        int min_match_frames_num = 3, double site_diff_thres = 1.0,
        double p3d_match_thres = 1.5);

    /// @brief 离线回环校验及校正
    MVS_EXPORT void RectifyLoopClosure(Tracker& tracker,
        const std::vector<int>& key_inds, int min_match_points_num = 4,
        /*int min_match_frames_num = 3,*/ double site_diff_thres = 1.0,
        double p3d_match_thres = 1.5, int iter_Num = 1,
        rulermvs::ProgressBar progress = 0);

    /// @brief 跟踪新加入的当前帧(多帧)，输出RT，同时对加入新帧的global_map进行BA优化，缩小同名点偏差
    /// @param global_map_ 全局地图，包含MapPoint和Frame，若传入为空，函数内部根据参考帧reference和对应的rts1重新构建global_map并BA优化，若不为空，默认为是经过BA优化的global_map，不再构造直接沿用
    /// @param rt 当前帧跟踪global_map成功后得到的RT，当前帧的三维地图点相对于global_map中的map_points
    /// @param reference 参考帧集合，包含point3d信息，若global_map_不为空，该参数不会被使用
    /// @param rts1 参考帧(多帧)的RT(局部同时是全局)，会随着BA优化一直更新，非定值
    /// @param current 当前帧集合，包含point3d信息
    /// @param rts2 当前帧(多帧)的RT(仅仅是局部)，若与参考帧对齐成功，则会求出帧间RT后更新一次然后全局BA优化后重新更新，非定值
    /// @param camera1 左目相机参数
    /// @param camera2 右目相机参数
    /// @param min_match_num 最小同名点数量
    /// @param site_diff_thres 两对同名点之间的线段距离差阈值
    /// @param p3d_match_thres 同名点匹配对在RT作用后两者之间的距离阈值
    /// @param p3d_nearest_thres 求解RT后合并单帧到global_map时，未参与匹配的同名点在同名点在RT作用后两者间的距离阈值，大于p3d_match_thres
    /// @param iter_Num BA优化迭代次数
    /// @param match_adjacentFrame 当前帧是否先尝试与相邻帧(global_map的最后一帧)对齐，false表示直接与global_map对齐(仅对当前帧是单帧起作用)(已废弃)
    /// @param insert_newframe 对齐成功或初始化成功是否将当前帧添加到global_map中，false表示否定
    static inline bool matchMarkerPointAndOptimizeMultiGroupFrames(GlobalMap& global_map_, rulermvs::Pose& rt, std::vector<rulermvs::CornerPairList>& reference, std::vector<rulermvs::Pose>& rts1,
        std::vector<rulermvs::CornerPairList>& current, std::vector<rulermvs::Pose>& rts2, Camera* camera1, Camera* camera2,
        int min_match_num = 3, double site_diff_thres = 1.0, double p3d_match_thres = 1.5, double p3d_nearest_thres = 3.0, int iter_Num = 10, bool insert_newframe = true)
    {
        assert(reference.size() == rts1.size() && current.size() == rts2.size());
        std::vector<Frame> reference_frames(reference.size()), current_frames(current.size());
        std::vector<int> new_mappoint_id;
        //判断全局globalmap是否存在，存在不用重新构建参考帧
        int count = 0;  //有效帧数，帧内点大于等于一定值
        if (global_map_.map_points_.empty())
        {
            for (int i = 0; i < reference.size(); i++) 
            {
                if (reference[i].size() < min_match_num)
                {
                    continue;
                }
                BuildFrameFromCornerPairs(reference[i], reference_frames[count++], rts1[i], camera1, camera2);
            }
            reference_frames.resize(count);
        } 
        else {
            count = 0;  //有效帧数，帧内点大于等于一定值
            for (int i = 0; i < reference.size(); i++) 
            {
                if (reference[i].size() < min_match_num) { continue; }
                //reference_frames[count++].joined_globalmap_ = true;
            }
            reference_frames.resize(count);
        }
        count = 0;
        for (int i = 0; i < current.size(); i++) 
        {
            if (current[i].size() < min_match_num) { continue; }
            BuildFrameFromCornerPairs(current[i], current_frames[count++], rts2[i], camera1, camera2);
        }
        current_frames.resize(count);
        count = 0;
        bool match_state = matchMarkerPointAndOptimizeMultiGroupFrames(global_map_, rt,
            reference_frames, current_frames, min_match_num, site_diff_thres,
            p3d_match_thres, p3d_nearest_thres, iter_Num/*, match_adjacentFrame*/, insert_newframe);
        if (match_state)
        {
            for (int i = 0; i < rts1.size(); i++)
            {
                if (reference[i].size() < min_match_num) { continue; }
                rulermvs::Pose& RT = rts1[i];
                auto& relate_rt = global_map_.frames_[count++].pose_w2c_;
                RT.a1 = relate_rt.at<double>(0, 0);
                RT.a2 = relate_rt.at<double>(1, 0);
                RT.a3 = relate_rt.at<double>(2, 0);
                RT.b1 = relate_rt.at<double>(0, 1);
                RT.b2 = relate_rt.at<double>(1, 1);
                RT.b3 = relate_rt.at<double>(2, 1);
                RT.c1 = relate_rt.at<double>(0, 2);
                RT.c2 = relate_rt.at<double>(1, 2);
                RT.c3 = relate_rt.at<double>(2, 2);
                RT.x  = relate_rt.at<double>(0, 3);
                RT.y  = relate_rt.at<double>(1, 3);
                RT.z  = relate_rt.at<double>(2, 3);
            }
            count = 0;
            if (insert_newframe)
            {           
                for (int i = 0; i < rts2.size(); i++)
                {
                    if (current[i].size() < min_match_num) { continue; }
                    rulermvs::Pose& RT = rts2[i];
                    auto& relate_rt = global_map_.frames_[count + reference_frames.size()].pose_w2c_;
                    count++;
                    RT.a1 = relate_rt.at<double>(0, 0);
                    RT.a2 = relate_rt.at<double>(1, 0);
                    RT.a3 = relate_rt.at<double>(2, 0);
                    RT.b1 = relate_rt.at<double>(0, 1);
                    RT.b2 = relate_rt.at<double>(1, 1);
                    RT.b3 = relate_rt.at<double>(2, 1);
                    RT.c1 = relate_rt.at<double>(0, 2);
                    RT.c2 = relate_rt.at<double>(1, 2);
                    RT.c3 = relate_rt.at<double>(2, 2);
                    RT.x  = relate_rt.at<double>(0, 3);
                    RT.y  = relate_rt.at<double>(1, 3);
                    RT.z  = relate_rt.at<double>(2, 3);
                }
            }
            else
            {
                if (rts2.size() == 1)
                { 
                    rts2[0] = rt;
                } else {
                    for (int i = 0; i < rts2.size(); i++)
                    {
                        rulermvs::Pose& RT = rts2[i];
                        RT = RT.inv() * rt;
                    }
                }
            }
        }
        //else
        //{
        //    reference.resize(reference_frames.size());//如果全局地图初始化失败，参考帧会清空
        //    rts1.resize(reference_frames.size());
        //}
        return match_state;
    }

    /// @brief 混乱无序多帧随机匹配
    /// @param multiFrames 输入多帧Frame
    /// @param min_match_num 任意两帧间标志点最小匹配数量，小于该值则无法匹配
    /// @param p3d_nearest_thres 最近的两个标志点间三位距离大于该值
    MVS_EXPORT void matchMultiUnorderedFrames(std::vector<Frame>& multiFrames, std::vector<rulermvs::Pose>& rts, int min_match_num = 3, double site_diff_thres = 1.0, double p3d_match_thres = 1.5, double p3d_nearest_thres = 3.0);

    static inline void matchMultiUnorderedFrames(std::vector<std::vector<rulermvs::Corner>>& cornerVecs, std::vector<rulermvs::Pose>& rts, Camera* camera,
        /*cv::Mat image = cv::Mat(),*/ int min_match_num = 3, double site_diff_thres = 1.0, double p3d_match_thres = 1.5, double p3d_nearest_thres = 3.0)
    {
        std::vector<Frame> multiFrames;
        std::vector<int>   valid_index;
        for (int i = 0; i < cornerVecs.size(); i++) 
        { 
            auto& corners = cornerVecs[i];
            if (corners.size() < min_match_num) 
            { 
                continue;
            }
            Frame frameCur;
            frameCur.corners_  = corners;
            rulermvs::Mat44 relate_rt = rulermvs::Mat44::Identity();
            frameCur.SetPose(relate_rt);
            //frameCur.image_  = image.clone();
            frameCur.camera_ = camera;
            multiFrames.emplace_back(frameCur);
            valid_index.push_back(i);
        }
        std::vector<rulermvs::Pose> rts_temp(multiFrames.size());
        matchMultiUnorderedFrames(multiFrames, rts_temp, min_match_num,
            site_diff_thres, p3d_match_thres, p3d_nearest_thres);
    }

}  // namespace slam
#endif