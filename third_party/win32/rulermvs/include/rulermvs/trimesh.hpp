#ifndef _RULERMVS_CORE_TRIMESH_HPP_
#define _RULERMVS_CORE_TRIMESH_HPP_
#include "rulermvs/rect.hpp"
#include "rulermvs/rgbd.hpp"
#include "rulermvs/camera.hpp"
#include "rulermvs/raster.hpp"
#include "rulermvs/pointcloud.hpp"
namespace rulermvs
{
///@brief 简单实现的三角面结构体
template <typename Float, typename Index = size_t> struct SimpleTriMesh_
    : ITriMesh_<Float, Index> {
    typedef std::vector<Index>    IndexVec;
    typedef std::vector<IndexVec> IndexVecs;
    SimpleTriMesh_() {}
    SimpleTriMesh_(const Image_<Float>& depth, const CameraP& cam);
    SimpleTriMesh_(const std::string& mesh_path) { load(mesh_path); }
    SimpleTriMesh_(SimpleTriMesh_&& mesh) noexcept
    {
        std::swap(points, mesh.points);
        std::swap(normals, mesh.normals);
        std::swap(tript_inds, mesh.tript_inds);
    }
    SimpleTriMesh_(const SimpleTriMesh_& mesh)
    {
        points.assign(mesh.points.begin(), mesh.points.end());
        normals.assign(mesh.normals.begin(), mesh.normals.end());
        tript_inds.assign(mesh.tript_inds.begin(), mesh.tript_inds.end());
    }
    SimpleTriMesh_& operator=(SimpleTriMesh_&& mesh)
    {
        std::swap(points, mesh.points);
        std::swap(normals, mesh.normals);
        std::swap(tript_inds, mesh.tript_inds);
        return *this;
    }
    SimpleTriMesh_& operator=(const SimpleTriMesh_& mesh)
    {
        points.assign(mesh.points.begin(), mesh.points.end());
        normals.assign(mesh.normals.begin(), mesh.normals.end());
        tript_inds.assign(mesh.tript_inds.begin(), mesh.tript_inds.end());
        return *this;
    }
    virtual ~SimpleTriMesh_() {}
    virtual bool load(const std::string& mesh_path);
    virtual bool save(const std::string& mesh_path) const;

    /// @brief 三角面的数量
    virtual size_t size() const { return tript_inds.size(); }
    /// @brief 是否为空
    bool empty() const { return points.empty() || tript_inds.empty(); }
    /// @brief 是否存在三维点
    virtual bool hasPoint() const { return !points.empty(); }
    /// @brief 是否存在法向
    virtual bool hasNormal() const
    {
        return !points.empty() && points.size() == normals.size();
    }
    /// @brief 是否存在三角面
    virtual bool hasTriPoint() const { return !tript_inds.empty(); }
    /// @brief 计算法向
    SimpleTriMesh_& computeNormals();
    /// @brief 检查未使用的点云，并标识
    /// @return 返回三维点索引数组
    inline std::vector<Index> checkUnusedPoints() const;
    /// @brief 检查无效点，如nan或inf点
    /// @return 返回三维点索引数组
    inline std::vector<Index> checkUnvalidPoints() const;
    /// @brief 检查无效三角网
    /// @return 返回无效面索引
    inline std::vector<Index> checkUnvalidTripts() const;
    /// @brief 移除给定bool状态指示的三维点及面
    /// @param inliers 状态数组，inliers.size() == getTriFaceNum().
    SimpleTriMesh_& removePoints(const std::vector<bool>& inliers);
    /// @brief 移除给定索引的三维点及面
    /// @param pt_inds 三维点
    SimpleTriMesh_& removePoints(const std::vector<Index>& pt_inds);
    /// @brief 移除给定索引的三角面
    /// @param tript_inds 三角面索引
    SimpleTriMesh_& removeTripts(const std::vector<Index>& tri_inds);
    /// @brief 移除未使用的三维点
    SimpleTriMesh_& removeUnusedPoints()
    {
        return removePoints(checkUnusedPoints());
    }
    /// @brief 移除无效的三角面片
    SimpleTriMesh_& removeUnvalidTripts()
    {
        return removeTripts(checkUnvalidTripts());
    }
    /// @brief 移除离群点
    SimpleTriMesh_& removeIsolatedTripts(Index min_group = 20);
	/// @brief 腐蚀网格边缘（仅适用于深度图生成的mesh）
    SimpleTriMesh_& erodeEdgeTripts(Index mesh_erode_times = 3, Index point_faces = 3);
    /// @brief 返回点所归属的面索引
    inline IndexVecs incidentTripts() const;
    /// @brief 临近面数组
    inline IndexVecs adjacencyTripts() const;
    /// @brief 按照连接分组
    /// @param groups 面分组索引
    /// @return 返回分组数量
    inline IndexVec groupComponents(IndexVec& groups) const;
    /// @brief 返回三维点数量
    virtual size_t getPointNum() const { return points.size(); }
    /// @brief 返回像素数据实际类型
    virtual PixelType getPixelType() const { return PixelType::RGB; }
    /// @brief 返回像素内存起始地址
    virtual const uchar* getPixelData() const { return nullptr; }
    /// @brief 返回三维点内存起始地址
    virtual const Point3_<Float>* getPointData() const
    {
        return hasPoint() ? &points[0] : nullptr;
    }
    /// @brief 返回法线内存起始地址
    virtual const Point3_<Float>* getNormalData() const
    {
        return hasNormal() ? &normals[0] : nullptr;
    }
    /// @brief 返回纹理坐标内存起始地址
    virtual const Point2_<Float>* getTexcoordData() const { return nullptr; }
    /// @brief 返回纹理引用
    virtual const IImage& getTexture() const { return RGBImage::Empty; }
    /// @brief 返回面数量
    virtual size_t getTriFaceNum() const { return tript_inds.size(); }
    /// @brief 返回三角面纹理索引数组首地址
    virtual const Scalar3_<Index>* getTriUVData() const { return nullptr; }
    /// @brief 返回三角面数组首地址
    virtual const Scalar3_<Index>* getTriPointData() const
    {
        return tript_inds.empty() ? nullptr : &tript_inds[0];
    }
    /// @brief 返回三角面法向索引数组首地址
    virtual const Scalar3_<Index>* getTriNormalData() const
    {
        // SimpleTrimesh中法向跟点共用一套索引.
        return getTriPointData();
    }
    /// @brief 以SE3进行位姿变换
    /// @param rt 姿态类;
    /// @return 返回点云的引用本身;
    SimpleTriMesh_& transform(const Pose& rt)
    {
        if (hasNormal()) rt.rotate(normals, normals);
        rt.transform(points, points);
        return *this;
    }

    std::vector<Point3_<Float>> points;       ///< 坐标
    std::vector<Point3_<Float>> normals;      ///< 法向
    std::vector<Scalar3_<Index>> tript_inds;  ///< 三角面对应的点云索引
};
using SimpleTriMesh = SimpleTriMesh_<float, size_t>;

template <typename Float, typename Index = size_t,
    PixelType pixType = PixelType::RGB>
struct TriMesh_: SimpleTriMesh_<Float, Index> {
    typedef pixel_traits_t<pixType>      Pixel;
    typedef SimpleTriMesh_<Float, Index> MeshBase;
    TriMesh_() {}
    TriMesh_(const std::string& path) { load(path); }
    TriMesh_(const RGBDImage_<Pixel, Float>& rgbd, const CameraP& cam)
        : MeshBase(rgbd.depth, cam)
    {
        image = rgbd.color;
        cam.reproject(MeshBase::points, texcoords);
        for (size_t i = 0; i < texcoords.size(); ++i) {
            auto& uv = texcoords[i];
            uv.x /= (float)cam.width;
            uv.y = (1.0f - uv.y / (float)cam.height);
        }
        trinl_inds.assign(
            MeshBase::tript_inds.begin(), MeshBase::tript_inds.end());
        triuv_inds.assign(
            MeshBase::tript_inds.begin(), MeshBase::tript_inds.end());
    }
    TriMesh_(const MeshBase& mesh)
        : MeshBase(mesh)
        , trinl_inds(MeshBase::tript_inds.begin(), MeshBase::tript_inds.end())
    {}
    TriMesh_(MeshBase&& mesh) noexcept
        : MeshBase(mesh)
        , trinl_inds(MeshBase::tript_inds.begin(), MeshBase::tript_inds.end())
    {}
    TriMesh_(TriMesh_&& mesh) noexcept : MeshBase(mesh)
    {
        std::swap(image, mesh.image);
        std::swap(texcoords, mesh.texcoords);
        std::swap(trinl_inds, mesh.trinl_inds);
        std::swap(triuv_inds, mesh.triuv_inds);
    }
    TriMesh_(const TriMesh_& mesh) : MeshBase(mesh)
    {
        image = mesh.image.clone();
        texcoords.assign(mesh.texcoords.begin(), mesh.texcoords.end());
        trinl_inds.assign(mesh.trinl_inds.begin(), mesh.trinl_inds.end());
        triuv_inds.assign(mesh.triuv_inds.begin(), mesh.triuv_inds.end());
    }
    TriMesh_& operator=(TriMesh_&& mesh) noexcept
    {
        std::swap(image, mesh.image);
        std::swap(texcoords, mesh.texcoords);
        std::swap(trinl_inds, mesh.trinl_inds);
        std::swap(triuv_inds, mesh.triuv_inds);
        std::swap(MeshBase::points, mesh.points);
        std::swap(MeshBase::normals, mesh.normals);
        std::swap(MeshBase::tript_inds, mesh.tript_inds);
        return *this;
    }
    TriMesh_& operator=(const TriMesh_& mesh)
    {
        image = mesh.image.clone();
        texcoords.assign(mesh.texcoords.begin(), mesh.texcoords.end());
        trinl_inds.assign(mesh.trinl_inds.begin(), mesh.trinl_inds.end());
        triuv_inds.assign(mesh.triuv_inds.begin(), mesh.triuv_inds.end());
        MeshBase::tript_inds.assign(
            mesh.tript_inds.begin(), mesh.tript_inds.end());
        MeshBase::points.assign(mesh.points.begin(), mesh.points.end());
        MeshBase::normals.assign(mesh.normals.begin(), mesh.normals.end());
        return *this;
    }
    virtual bool load(const std::string& path);
    virtual bool save(const std::string& path) const;
    /// @brief 是否存在纹理坐标
    virtual bool hasTexcoord() const
    {
        return !texcoords.empty() &&
               MeshBase::points.size() == texcoords.size();
    }
    /// @brief 是否存在三角面纹理索引
    virtual bool hasTriUV() const
    {
        return !MeshBase::hasTriPoint() &&
               MeshBase::tript_inds.size() == triuv_inds.size();
    }
    /// @brief 是否存在三角面法向索引
    virtual bool hasTriNormal() const
    {
        return !MeshBase::hasTriPoint() &&
               MeshBase::tript_inds.size() == trinl_inds.size();
    }
    /// @brief 计算三维点对应的像素值
    void computePointPixel();
    /// @brief 通过三维点反投计算对应的像素值
    /// @param cam 相机内参
    /// @param rt 相机外参
    /// @param img 外部纹理图像
    void computePointPixel(
        const CameraP& cam, const Pose& rt, const Image_<Pixel>& img);
    /// @brief 返回三维点的数量
    virtual size_t getPointNum() const { return MeshBase::points.size(); }
    /// @brief 返回像素数据实际类型
    virtual PixelType getPixelType() const { return pixType; }
    /// @brief 返回像素内存起始地址
    virtual const uchar* getPixelData() const
    {
        return (MeshBase::points.empty() ||
                   pixels.size() != MeshBase::points.size()) ?
                   nullptr :
                   (const uchar*)&pixels[0];
    }
    /// @brief 返回三维点内存起始地址
    virtual const Point3_<Float>* getPointData() const
    {
        return MeshBase::points.empty() ? nullptr : &MeshBase::points[0];
    }
    /// @brief 返回法线内存起始地址
    virtual const Point3_<Float>* getNormalData() const
    {
        // 接口类中要求法向跟点的数量一致.
        return !MeshBase::points.empty() &&
                       MeshBase::normals.size() == MeshBase::points.size() ?
                   &MeshBase::normals[0] :
                   nullptr;
    }
    /// @brief 返回纹理坐标内存起始地址
    virtual const Point2_<Float>* getTexcoordData() const
    {
        return !MeshBase::points.empty() &&
                       texcoords.size() == MeshBase::points.size() ?
                   &texcoords[0] :
                   nullptr;
    }
    /// @brief 返回纹理影像对应图像接口类的引用
    virtual const IImage& getTexture() const { return image; }
    /// @brief 返回三角面的数量
    virtual size_t getTriFaceNum() const { return MeshBase::tript_inds.size(); }
    /// @brief 返回三角面对应纹理索引数据的内存地址
    virtual const Scalar3_<Index>* getTriUVData() const
    {
        return hasTriUV() ? &triuv_inds[0] : nullptr;
    }
    /// @brief 返回三维点数据的内存地址
    virtual const Scalar3_<Index>* getTriPointData() const
    {
        return MeshBase::hasTriPoint() ? &MeshBase::tript_inds[0] : nullptr;
    }
    /// @brief 返回三维点法向数据的内存地址
    virtual const Scalar3_<Index>* getTriNormalData() const
    {
        return hasTriNormal() ? &trinl_inds[0] : nullptr;
    }

    Image_<Pixel> image;  ///< 纹理图像，只支持单张纹理
    std::vector<Point2_<Float>> texcoords;  ///< 点云对应的二维纹理坐标
    std::vector<Scalar3_<Index>> trinl_inds;  ///< 三角面对应的法线索引
    std::vector<Scalar3_<Index>> triuv_inds;  ///< 三角面对应的纹理索引
protected:
    std::vector<Pixel> pixels;  ///< 辅助成员变量，用于保存对应三维点的像素值
};
using RGBTriMesh  = TriMesh_<float, size_t, PixelType::RGB>;
using RGBATriMesh = TriMesh_<float, size_t, PixelType::RGBA>;
using GRAYTriMesh = TriMesh_<float, size_t, PixelType::UINT8>;
template <typename Tp, typename Float, typename Index, PixelType pixType>
struct PixelGetter_<Tp, TriMesh_<Float, Index, pixType>> {
    static inline std::vector<Tp> get(
        const TriMesh_<Float, Index, pixType>& mesh)
    {
        std::vector<Tp> pixels;
        if (!mesh.image.empty() && !mesh.points.empty() &&
            mesh.texcoords.size() == mesh.points.size()) {
            Image_<Tp> piximg;
            cvtColor(mesh.image, piximg);
            pixels.resize(mesh.points.size());
            for (size_t i = 0; i < mesh.texcoords.size(); ++i) {
                Float x = mesh.texcoords[i].x * (Float)mesh.image.width;
                Float y =
                    (1.f - mesh.texcoords[i].y) * (Float)mesh.image.height;
                interpolate(piximg, x, y, pixels[i]);
            }
        }
        return pixels;
    }
};
template <typename Float, typename Index, PixelType pixType>
void TriMesh_<Float, Index, pixType>::computePointPixel()
{
    if (image.empty() || MeshBase::points.empty() ||
        texcoords.size() != MeshBase::points.size())
        return;
    pixels.resize(MeshBase::points.size());
    for (size_t i = 0; i < texcoords.size(); ++i) {
        Float x = texcoords[i].x * (Float)image.width;
        Float y = (1.f - texcoords[i].y) * (Float)image.height;
        interpolate(image, x, y, pixels[i]);
    }
}

template <typename Float, typename Index, PixelType pixType>
void TriMesh_<Float, Index, pixType>::computePointPixel(
    const CameraP& cam, const Pose& rt, const Image_<Pixel>& img)
{
    if (img.empty() || MeshBase::points.empty()) return;
    pixels.resize(MeshBase::points.size());
    Point2fVec uvs(MeshBase::points.size());
    for (size_t i = 0; i < uvs.size(); ++i)
        uvs[i] = cam.reproject(rt.transform(MeshBase::points[i]));
    // reprojectPoints(cam, rt, MeshBase::points, uvs);
    for (size_t i = 0; i < uvs.size(); ++i)
        interpolate(img, uvs[i].x, uvs[i].y, pixels[i]);
}

template <typename Float, typename Index = size_t,
    PixelType pixType = PixelType::RGB>
struct RectangleMesh_: TriMesh_<Float, Index, pixType> {
    typedef TriMesh_<Float, Index, pixType> MeshBase;
    RectangleMesh_() {}
    RectangleMesh_(const Rect_<Float>& _rect) {}
    RectangleMesh_(const RectangleMesh_& _mesh) : MeshBase(_mesh) {}
};

template <> struct Converter_<RGBTriMesh, RGBPointCloud> {
    static inline void to(const RGBTriMesh& mesh, RGBPointCloud& cloud)
    {
        if (mesh.points.empty()) return;
        grabPointPixels(mesh, cloud.pixels);
        cloud.points.assign(mesh.points.begin(), mesh.points.end());
        cloud.normals.assign(mesh.normals.begin(), mesh.normals.end());
    }
};

template <> struct Converter_<GRAYTriMesh, GRAYPointCloud> {
    static inline void to(const GRAYTriMesh& mesh, GRAYPointCloud& cloud)
    {
        if (mesh.points.empty()) return;
        grabPointPixels(mesh, cloud.pixels);
        cloud.points.assign(mesh.points.begin(), mesh.points.end());
        cloud.normals.assign(mesh.normals.begin(), mesh.normals.end());
    }
};

template <> struct Converter_<SimpleTriMesh, PointCloud> {
    static inline void to(const SimpleTriMesh& mesh, PointCloud& cloud)
    {
        if (mesh.points.empty()) return;
        cloud.points.assign(mesh.points.begin(), mesh.points.end());
        cloud.normals.assign(mesh.normals.begin(), mesh.normals.end());
    }
};

template <typename Float, typename Index = size_t>
void computeTriNormals(const Scalar3_<Index>* _tript_inds, size_t _tript_num,
    const Point3_<Float>* _points, Point3_<Float>* _normals, size_t _point_num)
{
    // assert(_normals == nullptr);
    if (!_normals) return;
    if (_tript_num <= 0 || _point_num <= 0) return;
    memset((void*)_normals, 0,
        sizeof(Point3_<Float>) * static_cast<size_t>(_point_num));
    for (size_t i = 0; i < _tript_num; ++i) {
        const auto& tri_ind = _tript_inds[i];
        // 获取三角形各点
        const auto& pt0 = _points[tri_ind.s[0]];
        const auto& pt1 = _points[tri_ind.s[1]];
        const auto& pt2 = _points[tri_ind.s[2]];
        // 计算法向，默认三角形索引为逆时针排列
        auto nl = (pt1 - pt0).cross(pt2 - pt0);
        // 各点都累加法向，并在最后进行归一化
        _normals[tri_ind.s[0]] += nl;
        _normals[tri_ind.s[1]] += nl;
        _normals[tri_ind.s[2]] += nl;
    }
    // 法向归一化
    for (size_t i = 0; i < _point_num; ++i)
        _normals[i] = normalize(_normals[i]);
}
template <typename Float, typename Index = size_t>
void computeTriNormals(const std::vector<Scalar3_<Index>>& _tript_inds,
    const std::vector<Point3_<Float>>&                     _points,
    std::vector<Point3_<Float>>&                           _normals)
{
    if (_points.empty() || _tript_inds.empty()) return;
    _normals.resize(_points.size());
    computeTriNormals<Float, Index>(&_tript_inds[0], _tript_inds.size(),
        &_points[0], &_normals[0], _points.size());
}
template <typename Float, typename Index = size_t>
void computeTriNormals(const SimpleTriMesh_<Float, Index>& mesh)
{
    if (mesh.empty()) return;
    computeTriNormals<Float, Index>(mesh.tript_inds, mesh.points, mesh.normals);
}
template <typename Float, typename Index>
SimpleTriMesh_<Float, Index>& SimpleTriMesh_<Float, Index>::computeNormals()
{
    computeTriNormals<Float, Index>(tript_inds, points, normals);
    return *this;
}

template <typename Float = float, typename Index = size_t>
void rasterDepth(const SimpleTriMesh_<Float, Index>& mesh,
    const CameraP& camera, Image_<Float>& depth)
{
    rasterDepth<Float, Index>(mesh.tript_inds, mesh.points, camera, depth);
}

template <typename Float = float, typename Index = size_t>
Image_<Float> rasterDepth(
    const SimpleTriMesh_<Float, Index>& mesh, const CameraP& camera)
{
    Image_<Float> depth;
    rasterDepth<Float, Index>(mesh.tript_inds, mesh.points, camera, depth);
    return depth;
}
template <typename Float = float, typename Index = size_t>
void rasterDepth(const SimpleTriMesh_<Float, Index>& mesh,
    const CameraP& camera, const Pose& pose, Image_<Float>& depth)
{
    rasterDepth<Float, Index>(
        mesh.tript_inds, mesh.points, camera, pose, depth);
}
template <typename Float = float, typename Index = size_t>
Image_<Float> rasterDepth(const SimpleTriMesh_<Float, Index>& mesh,
    const CameraP& camera, const Pose& pose)
{
    Image_<Float> depth;
    rasterDepth<Float, Index>(
        mesh.tript_inds, mesh.points, camera, pose, depth);
    return depth;
}

template <typename Float, typename Index>
static inline void depth2Mesh(const Image_<Float>& depth, const CameraP& cam,
    SimpleTriMesh_<Float, Index>& mesh, double max_dist = 1e2,
    double edge_ratio = 0.1)
{
    if (depth.empty() || cam.width <= 0 || cam.height <= 0) return;
    Imagei inds(depth.size());
    inds.memsetNegOne();
    int cnt = 0;
    for (int i = 0; i < depth.height; i++) {
        auto inds_ptr  = inds.ptr(i);
        auto depth_ptr = depth.ptr(i);
        for (int j = 0; j < depth.width; ++j)
            if (depth_ptr[j] > 0) inds_ptr[j] = cnt++;
    }
    mesh.points.resize(cnt);
    auto cam_r = resizeCamera(cam, depth.size());
    for (int y = 0; y < depth.height; y++) {
        auto inds_ptr  = inds.ptr(y);
        auto depth_ptr = depth.ptr(y);
        for (int x = 0; x < depth.width; ++x) {
            if (depth_ptr[x] > 0)
                mesh.points[inds_ptr[x]] =
                    cam_r.uv2point(Point2_<Float> {x, y}) * depth_ptr[x];
        }
    }
    // 搜索三角面
    int tri_cnt = 0;
    mesh.tript_inds.resize(mesh.points.size() << 1);
    for (int y = 0; y < inds.height - 1; ++y) {
        auto inds_ptr = inds.ptr(y);
        for (int x = 0; x < inds.width - 1; ++x) {  // 右下
            auto& pt0 = inds_ptr[x];
            auto& pt1 = (&pt0)[1];
            auto& pt2 = (&pt0)[inds.width];
            if (pt0 >= 0 && pt1 >= 0 && pt2 >= 0) {
                auto v01    = mesh.points[pt1] - mesh.points[pt0];
                auto v02    = mesh.points[pt2] - mesh.points[pt0];
                auto v12    = mesh.points[pt2] - mesh.points[pt1];
                auto dist01 = v01.norm2();
                auto dist02 = v02.norm2();
                auto dist12 = v12.norm2();

                if (dist01 < max_dist && dist02 < max_dist &&
                    dist12 < max_dist && (dist01 > edge_ratio * dist02) &&
                    (dist01 * edge_ratio < dist02) &&
                    (dist01 * edge_ratio < dist12) &&
                    (dist12 * edge_ratio < dist01) &&
                    (dist02 * edge_ratio < dist12) &&
                    (dist12 * edge_ratio < dist02))
                    mesh.tript_inds[tri_cnt++] = {pt0, pt2, pt1};
            }
        }
        for (int x = 1; x < inds.width; ++x) {  // 下左
            auto& pt0 = inds_ptr[x];
            auto& pt1 = (&pt0)[inds.width];
            auto& pt2 = (&pt1)[-1];
            if (pt0 >= 0 && pt1 >= 0 && pt2 >= 0) {
                auto v01    = mesh.points[pt1] - mesh.points[pt0];
                auto v02    = mesh.points[pt2] - mesh.points[pt0];
                auto v12    = mesh.points[pt2] - mesh.points[pt1];
                auto dist01 = v01.norm2();
                auto dist02 = v02.norm2();
                auto dist12 = v12.norm2();

                if (dist01 < max_dist && dist02 < max_dist &&
                    dist12 < max_dist && (edge_ratio * dist02 < dist01) &&
                    (dist01 * edge_ratio < dist02) &&
                    (dist01 * edge_ratio < dist12) &&
                    (dist12 * edge_ratio < dist01) &&
                    (dist02 * edge_ratio < dist12) &&
                    (dist12 * edge_ratio < dist02))
                    mesh.tript_inds[tri_cnt++] = {pt0, pt2, pt1};
            }
        }
    }
    mesh.tript_inds.resize(tri_cnt);
    computeTriNormals(mesh.tript_inds, mesh.points, mesh.normals);
}

template <typename Float, typename Index>
SimpleTriMesh_<Float, Index>::SimpleTriMesh_(
    const Image_<Float>& depth, const CameraP& cam)
{
    depth2Mesh<Float, Index>(depth, cam, *this);
}

/// @brief 网格帧
struct MeshFrame: IRGBDImage {
    MeshFrame()                 = delete;
    MeshFrame(const MeshFrame&) = delete;
    MeshFrame(RGBTriMesh& mesh) : mesh(mesh) {}
    virtual ~MeshFrame() {}
    virtual RGBImage color() const { return mesh.image; }
    virtual Imagef   depth() const
    {
        /** 这里的实现，只针对单帧解码的网格数据;
            其对应的反投影坐标在原点位置的相机上,
            且去除了畸变.  */
        Imagef depth32f(320, 256);
        if (!mesh.empty() && mesh.points.size() == mesh.texcoords.size()) {
            std::vector<Point2f> uvs(mesh.texcoords.size());
            for (size_t i = 0; i < uvs.size(); ++i) {
                uvs[i].x = mesh.texcoords[i].x * (float)depth32f.width;
                uvs[i].y = (1 - mesh.texcoords[i].y) * (float)depth32f.height;
            }
            rasterDepth(mesh.trinl_inds, mesh.points, uvs, depth32f);
        } else {
            std::cout << "Points and UVs should be same nubmer." << std::endl;
        }
        return depth32f;
    }

    RGBTriMesh& mesh;
};
// 读写TriMesh,支持OBJ等格式
MVS_EXPORT bool readTriMesh(ConstStr& file_name, RGBTriMesh& mesh);
MVS_EXPORT bool readTriMesh(ConstStr& file_name, RGBATriMesh& mesh);
MVS_EXPORT bool readTriMesh(ConstStr& file_name, GRAYTriMesh& mesh);
MVS_EXPORT bool readTriMesh(ConstStr& file_name, SimpleTriMesh& mesh);
MVS_EXPORT bool writeTriMesh(ConstStr& file_name, const RGBTriMesh& mesh);
MVS_EXPORT bool writeTriMesh(ConstStr& file_name, const RGBATriMesh& mesh);
MVS_EXPORT bool writeTriMesh(ConstStr& file_name, const GRAYTriMesh& mesh);
MVS_EXPORT bool writeTriMesh(ConstStr& file_name, const SimpleTriMesh& mesh);
/// @brief 判断是否存在对应的图像加载函数
template <typename Mesh> struct has_trimesh_reader {
    typedef long No;
    typedef char Yes;
    template <typename Tp> struct helper {
        typedef bool (*func_ptr)(ConstStr&, Mesh&);
    };
    template <typename Tp, Tp> struct TypeCheck;
    template <typename Tp> static Yes has_reader(
        TypeCheck<typename helper<Tp>::func_ptr, &readTriMesh>*);
    template <typename Tp> static No has_reader(...);
    enum { value = (sizeof(has_reader<Mesh>(0)) == sizeof(Yes)) };
};
/// @brief 判断是否存在对应的图像保存函数
template <typename Mesh> struct has_trimesh_writer {
    typedef long No;
    typedef char Yes;
    template <typename Tp> struct helper {
        typedef bool (*func_ptr)(ConstStr&, const Mesh&);
    };
    template <typename Tp, Tp> struct TypeCheck;
    template <typename Tp> static Yes has_writer(
        TypeCheck<typename helper<Tp>::func_ptr, &writeTriMesh>*);
    template <typename Tp> static No has_writer(...);
    enum { value = (sizeof(has_writer<Mesh>(0)) == sizeof(Yes)) };
};
/// @brief 文件读取
template <typename Mesh, typename = void> struct TriMeshReader_ {
    static inline bool read(ConstStr&, Mesh&) { return false; }
};
template <typename Mesh, typename = void> struct TriMeshWriter_ {
    static inline bool write(ConstStr&, const Mesh&) { return false; }
};
template <typename Float, typename Index>
bool SimpleTriMesh_<Float, Index>::load(const std::string& mesh_path)
{
    return TriMeshReader_<SimpleTriMesh_<Float, Index>>::read(mesh_path, *this);
}
template <typename Float, typename Index>
bool SimpleTriMesh_<Float, Index>::save(const std::string& mesh_path) const
{
    return TriMeshWriter_<SimpleTriMesh_<Float, Index>>::write(
        mesh_path, *this);
}
template <typename Float, typename Index, PixelType pixType>
bool TriMesh_<Float, Index, pixType>::load(const std::string& mesh_path)
{
    return TriMeshReader_<TriMesh_<Float, Index, pixType>>::read(
        mesh_path, *this);
}
template <typename Float, typename Index, PixelType pixType>
bool TriMesh_<Float, Index, pixType>::save(const std::string& mesh_path) const
{
    return TriMeshWriter_<TriMesh_<Float, Index, pixType>>::write(
        mesh_path, *this);
}
template <typename Mesh> struct TriMeshReader_<Mesh,
    typename std::enable_if<has_trimesh_reader<Mesh>::value>::type> {
    static inline bool read(ConstStr& mesh_path, Mesh& mesh)
    {
        return readTriMesh(mesh_path, mesh);
    }
};
template <typename Mesh> struct TriMeshWriter_<Mesh,
    typename std::enable_if<has_trimesh_writer<Mesh>::value>::type> {
    static inline bool write(ConstStr& mesh_path, const Mesh& mesh)
    {
        return writeTriMesh(mesh_path, mesh);
    }
};

}  // namespace rulermvs
#include "rulermvs/trimesh_impl.hpp"
#endif
