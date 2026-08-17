#ifndef _RULERMVS_CODEDLIGHT_ONESHOT_HPP_
#define _RULERMVS_CODEDLIGHT_ONESHOT_HPP_
#include "rulermvs/image.hpp"
#include "rulermvs/camera.hpp"
#include "rulermvs/logger.hpp"
#include "rulermvs/trimesh.hpp"
namespace rulermvs
{
/// 编码线结构光的解码接口类
template <> struct ICodedLight_<LightType::CodedLine> {
    // 解码参数
    typedef struct IOneShotDecodePara {
        bool bAttachPoint = false;
        bool bFuzzyDecode = false;
        bool bSparseMesh = false;
        float sigma = 2.0f;       ///< 高斯核对应的方差,建议值1.3f;
        float darkness = 1.f;   ///< 黑色敏感度,建议值3.0f;
        int smoothX = 5;          ///< 动态优化的X方向平滑项
        int smoothY = 5;          ///< 动态优化的Y方向平滑项
        float maxAngle = 1.4f;    ///< 三角面朝向的最大角度
        int minGroup = 50;        ///< 用于去除孤立项
        float pointDist = 10.0f;  ///< 三维点之间的最大点间距
        float edgeRatio = 0.1f;   ///< 构造三角网时距离比值阈值
        int linkInterval = 20;    ///< 搜索连接范围阈值
        float lineThreshold = 0.75f;  ///< 中心线判定
    } DecodePara;

    // 设备参数
    typedef struct IOneShotDevicePara {
        int mode = 0;
        double farZ = 1000;  ///< 远端Z值,近似最远扫描距离[标定输出]
        double nearZ = 0;  ///< 近端Z值,近似最近扫描距离[标定输出]
        String version;  ///< 版本识别号,用于标识标定时状态及设备类型
        Pose colorRT;  ///< 以解码相机光心为原点，对应纹理相机姿态
        CameraSkewPB colorCam;  ///< 纹理相机内参和畸变
        Pose decodeRT;  ///< 以解码相机光心为原点，对应相机姿态,默认单帧矩阵.
        CameraSkewPB decodeCam;  ///< 解码相机对应的内参和畸变
        Pose projectorRT;  ///< 以解码相机光心为原点，对应投影仪姿态
        CameraSkewPB projectorCam;  ///< 投影仪对应的内参和畸变,[143,6]
        String projectorCodeStr;  ///< 编码字典,字符串【0~A对应16进制】
        Rect_<int> projectorValid;  ///< 投影码元有效区域
    } DevicePara;

    /// 匿名枚举，单帧解码的返回状态。
    enum : int { Succeed = 0, Failed, EmptyMesh, WrongPara, EmptyImage };

    typedef std::shared_ptr<ICodedLight_<LightType::CodedLine>> Ptr;

    /// @brief 析构函数,占位.
    virtual inline ~ICodedLight_() {}

    /// @brief 静态函数创建实例
    static inline Ptr create();
    
    /// @brief 解码初始化;
    virtual int init(DevicePara& device_para) = 0;

    /// @brief 解码初始化,输入文件.
    virtual int init(const std::string& device_path) = 0;

    virtual int setScanRange(float nearZ, float farZ) = 0;

    virtual int getScanRange(float& nearZ, float& farZ) const = 0;

    /// @brief 解码，输出三角网格;
    /// @param gray 相机接受结构光影像
    /// @param mesh 输出三角网格
    /// @param para 解码参数
    /// @return 返回错误码
    virtual int decode(const Image8u& gray, SimpleTriMesh& mesh,
        const DecodePara& para) const = 0;

    /// @brief 获取纹理相机内参;
    virtual int getColorCamera(CameraSkewPB& camera) const = 0;

    /** 这里的畸变矫正用于快速查看结果，获取高性能畸变矫正请采用Remap的形式;
        具体请参考Camera的createUndistorRectifyMap函数; */
    template <typename Tp>
    inline void undistor(const Image_<Tp>& src, Image_<Tp>& dst) const
    {
        assert(!src.empty());
        CameraSkewPB cam;
        if (getColorCamera(cam) != Succeed) return;
        cam.resize(src.size());
        // auto cam = resizeCamera(camera(), src.size());
        undistorImage(src, cam, Rotation(), cam.nodistor().noskew(), dst);
    }

    /// @brief 解码,并输出深度图;
    /// @param gray 相机接收结构光影像
    /// @param depth 深度图
    /// @param para 解码参数
    /// @return 返回错误码
    virtual int decode(const Image8u& gray, Imagef& depth,
        const DecodePara& decode_para) const
    {
        SimpleTriMesh mesh;
        assert(!gray.empty());
        int ret = decode(gray, mesh, decode_para);
        if (ret != Succeed) return ret;
        CameraSkewPB cam;
        ret = getColorCamera(cam);
        if (ret != Succeed) return ret;
        rasterDepth(mesh, cam.nodistor().noskew() / 4, depth);
        return Succeed;
    }

    /// @brief 解码并对纹理进行畸变校正，输出深度图和校正后的纹理.
    /// @tparam Pixel 纹理图像类型
    /// @param gray 相机接受结构光影像
    /// @param color 纹理影像
    /// @param rgbd 输出RGBD数据
    /// @param decode_para 解码参数
    /// @return 返回错误码
    template <typename Pixel> int decode(const Image8u& gray,
        const Image_<Pixel>& color, RGBDImage_<Pixel, float>& rgbd,
        const DecodePara& decode_para) const
    {
        SimpleTriMesh mesh;
        assert(!gray.empty() && !color.empty());
        int ret = decode(gray, mesh, decode_para);
        if (ret != Succeed) return ret;
        undistor(color, rgbd.color);
        CameraSkewPB cam;
        ret = getColorCamera(cam);
        if (ret != Succeed) return ret;
        rasterDepth(mesh, cam.nodistor().noskew() / 4, rgbd.depth);
        return Succeed;
    }

    /// @brief 解码结构光并校正纹理影像，并输出三角网格;
    /// @tparam pixType 三角网格的纹理影像类型
    /// @param gray 结构光影像
    /// @param color 纹理影像
    /// @param mesh 带纹理三角网格
    /// @param decode_para 解码参数
    /// @return 返回错误码
    template <PixelType pixType> int decode(const Image8u& gray,
        const Image_<pixel_traits_t<pixType>>& color,
        TriMesh_<float, size_t, pixType>& mesh, const DecodePara& decode_para) const
    {
        assert(!gray.empty() && !color.empty());
        int ret = decode(gray, mesh, decode_para);
        if (ret != Succeed) return ret;
        CameraSkewPB cam;
        ret = getColorCamera(cam);
        if (ret != Succeed) return ret;
        cam.resize(color.size());
        undistorImage(color, cam, Rotation(), cam.nodistor().noskew(), mesh.image);
        cam.nodistor().noskew().reproject(mesh.points, mesh.texcoords);
        for (size_t i = 0; i < mesh.texcoords.size(); ++i) {
            auto& uv = mesh.texcoords[i];
            uv.x /= (float)cam.width;
            uv.y = (1.0f - uv.y / (float)cam.height);
        }
        mesh.trinl_inds.assign(mesh.tript_inds.begin(), mesh.tript_inds.end());
        mesh.triuv_inds.assign(mesh.tript_inds.begin(), mesh.tript_inds.end());
        return Succeed;
    }

    /// @brief 从文本文件中读取编码线结构光的设备参数;
    static MVS_EXPORT bool loadDeviceFile(
        const std::string& path, DevicePara& para);

    /// @brief 将标定的结果数据写出到文本文中，note 后续可能会改进线方程的表达
    static MVS_EXPORT bool saveDeviceFile(
        const std::string& path, const DevicePara& para);
};
using IOneShot = ICodedLight_<LightType::CodedLine>;

/// @brief 内置创建单帧解码实例
extern "C" MVS_EXPORT IOneShot* createOneShot();
template <> struct ObjectCreator_<IOneShot> {
    static inline decltype(auto) create() { return createOneShot(); }
};
typename ICodedLight_<LightType::CodedLine>::Ptr
ICodedLight_<LightType::CodedLine>::create()
{
    return createObject<IOneShot>();
}
}  // namespace rulermvs
#endif