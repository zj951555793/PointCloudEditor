#ifndef _RULERMVS_CODEDLIGHT_PHASESHIFT_HPP_
#define _RULERMVS_CODEDLIGHT_PHASESHIFT_HPP_
#include "rulermvs/rgbd.hpp"
#include "rulermvs/image.hpp"
#include "rulermvs/calib.hpp"
#include "rulermvs/camera.hpp"
// #include "rulermvs/plugin.hpp"
#include "rulermvs/trimesh.hpp"
#include "rulermvs/cv.hpp"
namespace rulermvs
{
/// @brief 相移相位展开, 支持任意N步相移编码;
/// @param imgs 输入图像数组起始地址
/// @param sz 输入图像数量sz,sz>=3.
/// @param phi 相位图
MVS_EXPORT void unwrapPhase(const Image8u* imgs, size_t sz, Imagef& phi);

/// @brief 解格雷编码值
/// @param imgs 编码图像起始地址
/// @param sz 编码图像数量[1-8]
/// @param white 全亮影像
/// @param black 全暗影像
/// @param code 输出解码值
/// @param quality 将均方差小于此值的码值置为-1
MVS_EXPORT void unwrapBinary(const Image8u* imgs, size_t img_num,
    const Image8u& white, const Image8u& black, Image8u& code,
    uchar quality = 10u, uchar background_lowthreshold = 0u, uchar background_highthreshold = 255u);

/// @brief 基于动态优化扩展相位值
/// @param phi 输入相位影像，相位分布值[0.0-1.0]
/// @param code 编码值[0-2^N]
/// @param phase 解包裹后的最终相位值
/// @param ncode 码元最大值[0-255]
/// @param smooth 平滑能量项,uchar类型用来限定能量的大小【0-255】分布.
MVS_EXPORT void expandPhase(const Imagef& phi, const Image8u& code,
    Imagef& phase, uchar ncode = 128u, uchar smooth = 30u);

/// @brief 解格雷码和相位并扩展
/// @param imgs 输入编码影像[顺序为黑-白-二值编码-相移]
/// @param Nc 二值编码影像数量
/// @param Np 相移影像数量
/// @param phi 扩展相位
/// @param has_wb 前两张影像是否为黑白
/// @param quality 均方差阈值，将不显著区域排除
/// @param max_code 最大解码值
/// @param smooth 平滑项
MVS_EXPORT void unwrapBinaryAndPhase(const Image8u* imgs, int Nc, int Np,
    Imagef& phi, Image8u& white, bool has_wb = true, uchar quality = 10u, int min_connect = 50,
    uchar max_code = 128u, uchar smooth = 30u, uchar background_lowthreshold = 0u, uchar background_highthreshold = 255u);

/// @brief 沿行匹配相位并计算视差图
/// @param p0 相机0对应的相位图
/// @param p1 相机1对应的相位图
/// @param disparity 视差图
/// @param ncode 最大解码值;当ncode为0时，采用KDTree；否则使用动态规划.
MVS_EXPORT void matchPhaseRectify(
    const Imagef& p0, const Imagef& p1, Imagef& disparity, uchar ncode = 0);
MVS_EXPORT void matchPhaseRectify2(const Imagef& p0, const Imagef& p1,
    const Image8u& white_rectify1, const Image8u& white_rectify2,
    Imagef& disparity, uchar ncode = 0);

/// @brief 相移设备参数
struct IPhaseShiftDevicePara {
    double farZ;  ///< 近端Z平面，近似最近扫描距离，标定输出；
    double nearZ;  ///< 远端Z平面，近似最远扫描距离，标定输出；
    String   version;             ///< 版本号;
    Pose     rt_1;                ///< 1相机外参
    Pose     rt_2;                ///< 2相机外参
    CameraPB cam_1;               ///< 1相机内参
    CameraPB cam_2;               ///< 2相机内参
    int      wave_len    = 10;    ///< 相移波长
    uchar    max_code    = 128;   ///< 最大解码值
    int      phase_num   = 6;     ///< 相移图像数量，仅支持3.4.6步
    int      binary_num  = 9;     ///< 格雷码图像数量
    bool     white_black = true;  ///< 前两张是否为黑白
    IntVec   code_dict;           ///< 编码序列,Optional.
};

/// @brief 从旧的标定文件(*.rt)中读取设备参数
/// @param path 文件路径
/// @param cam1 相机1内参
/// @param rt1 相机1外参
/// @param cam2 相机2内参
/// @param rt2 相机2外参
/// @return 返回状态值
MVS_EXPORT bool load_phase_calib_param_from_ascii(const std::string& path,
    CameraPB& cam1, Pose& rt1, CameraPB& cam2, Pose& rt2);

/// @brief 求极线校正后的虚拟左目与原始左目之间的RT
MVS_EXPORT Pose getStereoRectifiedParam(const std::vector<Pose>& camPoseVec,
    const std::vector<CameraPB>& cameraPBVec, int width, int height, int rectify_flags, 
    std::vector<Pose>& RectifycamPoseVec, std::vector<CameraPB>& RectifycameraPBVec);

MVS_EXPORT void get_DenseDepth_from_SparsePoints(const Point3fVec& src_pts,
    const CameraP& cam, const Imagef& depth_scale, Imagef& depth, bool consider_edge = true, int min_connect = 20);

/// @brief 根据缩放后的深度图去除原始深度图中的噪声
/// @param depth_scale 缩放后深度图
/// @param depth_ori 原始深度图
/// @param depth_new 新的深度图(输出)
/// @param radius 深度图导向滤波窗口半径大小
/// @param eps 深度图导向滤波模糊程度
/// @param gamma 深度图去噪强度，0.4~0.05，越靠近区间右端越强
/// @param min_connect 最小连接数量
/// @param erode_state 是否腐蚀深度图
MVS_EXPORT void get_DenseDepth_from_SparseDepth(const Imagef& depth_scale,
    const Imagef& depth_ori, Imagef& depth_new, int radius = 1,
    double eps = 0.5, float gamma = 0.15, int min_connect = 20,
    bool erode_state = false);

/// @brief 相移解码参数
struct IPhaseShiftDecodePara {
    uchar quality = 1;  ///< 解码参考,0为所有像素，值越大有效点越少.[0-10]
    uchar match_mode = 1;  ///< 0表示kdtree,other表示DP.
    uchar erode_size = 0;  ///< 腐蚀参数>3且为奇数,会将点云模糊.
    float min_dist   = 100.0f;  ///< 最小扫描距离
    float max_dist   = 500.0f;  ///< 最大扫描距离
    float max_angle = 1.4f;  ///< 三角面对应法向与到相机中心的最大夹角;
    float point_dist = 30.0f;  ///< 最大点间距离
    float edge_ratio = 0.1f;  ///< 网格的边长之比的阈值，用于抑制狭长三角面;
    int   min_group = 50;  ///< 三角面的最小聚集数量
    float proj_filter =
        0;  ///< 投影仪约束的反投影误差阈值,小于等于0时表示不采用此种滤波方式;
    uchar background_lowThreshold = 0;//背景阈值，像素灰度小于该值时不参与解码及构网
    uchar background_highThreshold = 255;//背景阈值，像素灰度大于该值时不参与解码及构网
    float scale_depth = 1.0f;  // 深度图缩放系数，默认不缩放
    uchar mesh_erode_times = 3;     // 网格腐蚀的次数
    uchar point_faces = 3;  // point连接的面片数量最小阈值，point最多会连接6个面片，小于或等于此值时该点会被删除，勿轻易改动，2、3、4，代表网格腐蚀程度一般、标准、严重

    int    radius = 1;    // 深度图导向滤波窗口半径大小，或2
    double eps    = 0.5;  // 深度图导向滤波模糊程度
    float gamma = 0.20f;  // 深度图去噪强度，0.4~0.05，越靠近区间右端越强

    std::vector<cv::Point> ZeroPoints_L;//左目中不解码的像素坐标集合
    std::vector<cv::Point> ZeroPoints_R;//左目中不解码的像素坐标集合
#ifdef _DEBUG
    std::string verbose_dir = "./";
#endif
};

/// @brief 相移结构光接口类
template <> struct ICodedLight_<LightType::PhaseShift> {
    typedef IPhaseShiftDecodePara DecodePara;
    typedef IPhaseShiftDevicePara DevicePara;

    /// @brief 枚举类型，返回值状态
    enum : int { Success = 0, Failed, WrongPath, WrongPara, WrongInput };

    typedef std::shared_ptr<ICodedLight_<LightType::PhaseShift>> Ptr;

    /// @brief 静态函数创建实例
    static inline Ptr create();

    /// @brief 析构函数
    virtual inline ~ICodedLight_() {}

    /// @brief 获取RGBD相机内参;
    virtual CameraPB camera() const = 0;

    /// @brief 解码初始化,配置相移设备参数
    /// @param device 设备参数
    /// @return 返回错误码
    virtual int init(const DevicePara& device) = 0;

    /// @brief 解码初始化;
    /// @param device_path 设备参数文件路径
    /// @return 返回错误码
    virtual int init(const std::string& device_path)
    {
        DevicePara device;
        if (device_path.substr(device_path.rfind('.')) == ".rt" &&
            load_phase_calib_param_from_ascii(device_path, device.cam_1,
                device.rt_1, device.cam_2, device.rt_2)) {
            device.cam_1.width = device.cam_2.width = 3072;
            device.cam_1.height = device.cam_2.height = 2048;
            return this->init(device);
        }
        return WrongPath;
    }

    /// @brief 解码并构建三角网格
    /// @param imgs_l 0相机影像列表
    /// @param imgs_r 1相机影像列表
    /// @param img_num 影像数量
    /// @param mesh 三角网格
    /// @param param 解码参数
    /// @return 返回错误码
    virtual int decode(const Image8u* imgs_l, const Image8u* imgs_r,
        size_t img_num, SimpleTriMesh& mesh, const DecodePara& param) const = 0;

    // 加入深度图、mesh缩放因子scale_factor
    /*virtual int decode(const Image8u* imgs_l, const Image8u* imgs_r,
        size_t img_num, SimpleTriMesh& mesh, float& scale_factor,
        const DecodePara& param) const = 0;*/

    virtual int decode(const Image8uVec& imgs_l, const Image8uVec& imgs_r,
        SimpleTriMesh& mesh, const DecodePara& param) const
    {
        if (imgs_l.empty() || imgs_l.size() != imgs_r.size()) return WrongInput;
        return decode(&imgs_l[0], &imgs_r[0], imgs_l.size(), mesh, param);
    }

    /// @brief 解码并构建带纹理三角网格
    /// @param imgs_l 0相机影像列表
    /// @param imgs_r 1相机影像列表
    /// @param img_num 影像数量
    /// @param mesh 三角网格
    /// @param param 解码参数
    /// @return 返回错误码
    virtual int decode(const Image8u* imgs_l, const Image8u* imgs_r,
        size_t img_num, GRAYTriMesh& mesh, const DecodePara& param) const = 0;

    // 加入深度图、mesh缩放因子scale_factor
    /*virtual int decode(const Image8u* imgs_l, const Image8u* imgs_r,
        size_t img_num, GRAYTriMesh& mesh, float& scale_factor,
        const DecodePara& param) const = 0;*/

    virtual int decode(const Image8uVec& imgs_l, const Image8uVec& imgs_r,
        GRAYTriMesh& mesh, const DecodePara& param) const
    {
        assert(!imgs_l.empty() && imgs_l.size() == imgs_r.size());
        return decode(&imgs_l[0], &imgs_r[0], imgs_l.size(), mesh, param);
    }

    /// @brief 解码并输出深度图
    /// @param imgs_l 1相机影像列表
    /// @param imgs_r 2相机影像列表
    /// @param img_num 影像数量
    /// @param depth 深度图
    /// @param param 解码参数
    /// @return 返回错误码
    virtual int decode(const Image8u* imgs_l, const Image8u* imgs_r,
        size_t img_num, Imagef& depth, const DecodePara& param) const
    {
        SimpleTriMesh mesh;
        if (!imgs_l || !imgs_r || img_num <= 0) return WrongInput;
        int ret = decode(imgs_l, imgs_r, img_num, mesh, param);
        if (ret == Success) rasterDepth(mesh, camera().nodistor(), depth);
        return ret;
    }

    /// @brief 解码并输出深度图
    /// @param imgs_l 1相机影像列表
    /// @param imgs_r 2相机影像列表
    /// @param img_num 影像数量
    /// @param depth 深度图
    /// @param param 解码参数
    /// @return 返回错误码
    virtual int decode(const Image8uVec& imgs_l, const Image8uVec& imgs_r,
        Imagef& depth, const DecodePara& param) const
    {
        if (imgs_l.empty() || imgs_l.size() != imgs_r.size()) return WrongInput;
        return decode(&imgs_l[0], &imgs_r[0], imgs_l.size(), depth, param);
    }

    /// @brief 解码并输出深度图
    /// @param imgs_l 左相机影像列表
    /// @param imgs_r 右相机影像列表
    /// @param rgbd RGBD数据
    virtual int decode(const Image8u* imgs_l, const Image8u* imgs_r,
        size_t img_num, GRAYDImage& rgbd, const DecodePara& param) const
    {
        GRAYTriMesh mesh;
        if (!imgs_l || !imgs_r || img_num <= 0) return WrongInput;
        int ret = decode(imgs_l, imgs_r, img_num, mesh, param);
        if (ret == Success) {
            undistorImage(mesh.image, camera(), rgbd.color);
            rasterDepth(mesh, camera().nodistor(), rgbd.depth);
        }
        return ret;
    }

    /// @brief 解码并输出深度图
    /// @param imgs_l 左相机影像列表
    /// @param imgs_r 右相机影像列表
    /// @param rgbd RGBD数据
    /// @param param 解码参数
    virtual int decode(const Image8uVec& imgs_l, const Image8uVec& imgs_r,
        GRAYDImage& rgbd, const DecodePara& param) const
    {
        if (imgs_l.empty() || imgs_l.size() != imgs_r.size()) return WrongInput;
        return decode(&imgs_l[0], &imgs_r[0], imgs_l.size(), rgbd, param);
    }
};
typedef ICodedLight_<LightType::PhaseShift> IPhaseShift;

/// @brief 创建相移解码实例
/// @return 返回实例指针,需要用户释放
MVS_EXPORT IPhaseShift* createPhaseShift();
template <> struct ObjectCreator_<IPhaseShift> {
    static inline IPhaseShift::Ptr create()
    {
        return {createPhaseShift(), [](IPhaseShift* p) {
                    if (p) delete p;
                }};
    }
};
/// @brief 相移插件动态加载
// template <> struct PluginProperty_<IPhaseShift> {
//     static inline std::string name() { return "PhaseShift"; }
// };
// template <> struct CodedLightWapper_<LightType::PhaseShift> {
//     static inline std::shared_ptr<IPhaseShift> create()
//     {
//         return PluginCreate<IPhaseShift>();
//     }
// };
typename IPhaseShift::Ptr IPhaseShift::create()
{
    // 优先调用外部插件模块，不成功则返回内置模块.
    // auto obj = PluginCreate<IPhaseShift>();
    // if (obj != nullptr) {
    //     MVS_ILOG << "Create PhaseShift Instance From Plugin Dll.";
    //     return obj;
    // }
    return createObject<IPhaseShift>();
}
}  // namespace rulermvs
#endif  //_RULERMVS_CODEDLIGHT_PHASESHIFT_HPP_