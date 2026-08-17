#ifndef _RULERMVS_CORE_POINTCLOUD_HPP_
#define _RULERMVS_CORE_POINTCLOUD_HPP_
#include "rulermvs/core.hpp"
#include "rulermvs/kdtree.hpp"
#include "rulermvs/logger.hpp"
#include "rulermvs/math.hpp"
#include "rulermvs/rgbd.hpp"
#include "rulermvs/voxel.hpp"
#include "rulermvs/normal.hpp"
namespace rulermvs
{
/// @brief 获取点云类的三维点对应的像素
/// @tparam Cloud 点云类型 PointCloud,Mesh等
/// @tparam Tp 返回像素类型
template <typename Tp, typename Cloud, typename = void> struct PixelGetter_ {
    static inline std::vector<Tp> get(const Cloud&)
    {
        return std::vector<Tp>();
    }
};
template <typename Cloud, typename Tp>
static inline void grabPointPixels(const Cloud& cloud, std::vector<Tp>& pixs)
{
    pixs = PixelGetter_<Tp, Cloud>::get(cloud);
}
/// @brief 点云读写接口类
/// @tparam Cloud 点云类型
template <typename Cloud, class = void> struct CloudReader_ {
    static inline bool read(ConstStr&, Cloud&) { return false; }
};
template <typename Cloud, class = void> struct CloudWriter_ {
    static inline bool write(ConstStr&, const Cloud&) { return false; }
};
template <typename Cloud, typename... Args>
bool readPointCloud(ConstStr& path, Cloud& cloud, Args&&... args)
{
    return CloudReader_<Cloud>::read(path, cloud, std::forward<Args>(args)...);
}
template <typename Cloud, typename... Args>
bool writePointCloud(ConstStr& path, const Cloud& cloud, Args&&... args)
{
    return CloudWriter_<Cloud>::write(path, cloud, std::forward<Args>(args)...);
}
/**@brief 点云结构体
 * @details 点云结构体的简单实现，这里主要用来配合在线拼接和数据存储。*/
template <typename Float> struct PointCloud_: public IPointCloud_<Float> {
    static_assert(std::is_floating_point<Float>::value,
        "class PointCloud can only be instantiated with float point type.");
    PointCloud_() : points(0), normals(0) {}
    PointCloud_(int npoints) : points(npoints), normals(npoints) {}
    PointCloud_(ConstStr& path) { load(path); }
    PointCloud_(PointCloud_&& cloud) noexcept
    {
        std::swap(points, cloud.points);
        std::swap(normals, cloud.normals);
    }
    PointCloud_(const PointCloud_& cloud)
        : points(cloud.points.begin(), cloud.points.end())
        , normals(cloud.normals.begin(), cloud.normals.end())
    {}
    PointCloud_(const Image_<Float>& depth, const CameraP& camera)
    {
        Image_<Point3_<Float>> vmap, nmap;
        depth2Vmap(depth, camera, vmap);
        vmap2Nmap(vmap, nmap);
        Image8u mask;
        vmapAndNmapToMask<Float>(vmap, nmap, mask);
        cvtImgToVec(vmap, mask, points);
        cvtImgToVec(nmap, mask, normals);
    }
    virtual ~PointCloud_() {}
    PointCloud_& operator=(PointCloud_&& cloud) noexcept
    {
        std::swap(points, cloud.points);
        std::swap(normals, cloud.normals);
        return *this;
    }
    PointCloud_& operator=(const PointCloud_& cloud)
    {
        points.assign(cloud.points.begin(), cloud.points.end());
        normals.assign(cloud.normals.begin(), cloud.normals.end());
        return *this;
    }
    virtual bool load(ConstStr& path) { return readPointCloud(path, *this); }
    virtual bool save(ConstStr& path) const
    {
        return writePointCloud(path, *this);
    }
    /// @brief 返回点的数量
    virtual size_t size() const { return points.size(); }
    /// @brief 将点云和法向量重置为指定大小;
    virtual void resize(size_t sz) { points.resize(sz), normals.resize(sz); }
    /// @brief 清空点云
    virtual void clear() { points.clear(), normals.clear(); }
    /// @brief 判断点云是否为空
    virtual bool empty() const { return points.empty(); }
    /// @brief 判断是否存在向量
    virtual bool hasNormal() const
    {
        return !empty() && size() == normals.size();
    }
    /// @brief 以SE3进行位姿变换
    /// @param rt 姿态类;
    /// @return 返回点云的引用本身;
    PointCloud_& transform(const Pose& rt)
    {
        rt.transform(points, points);
        if (hasNormal()) rt.rotate(normals, normals);
        return *this;
    }
    /// @brief 检测指定数量最近点的近似距离；
    /// @param k 用于搜索的邻近点个数;
    /// @return 返回k点范围的近似距离;
    Float checkDist(int k) const;
    /// @brief 计算点云所有point到指定点的平均距离；
    /// @return 返回到指定点的平均距离;
    Float computeMeanDistToView(
        const Point3_<Float>& view = Point3_<Float> {0, 0, 0}) const;
    /// @brief 法向归一化
    void normalize()
    {
        for (size_t i = 0; i < normals.size(); ++i)
            normals[i] = normalize(normals[i]);
    }
    /// @brief 翻转反向
    PointCloud_& reverseNormal()
    {
        for (size_t i = 0; i < normals.size(); ++i) normals[i] = -normals[i];
        return *this;
    }
    /// @brief 在视点的指导下修复法向
    PointCloud_& reverseNormalTowardView(
        const Point3_<Float>& view = Point3_<Float> {0, 0, 0})
    {
        for (size_t i = 0; i < this->size(); ++i)
            normals[i] =
                normals[i].dot(points[i] - view) < 0 ? normals[i] : -normals[i];
        return *this;
    }
    /// @brief 计算各点的法向量
    /// @param k 用于计算法向量的邻近点个数;
    PointCloud_& computeNormalsInKnn(size_t k);
    /// @brief 计算各点的法向量
    /// @param k 用于计算法向量的邻近点个数;
    PointCloud_& recomputeNormalsInKnn(size_t k);
    /// @brief 计算各点的法向量
    /// @param radius 用于计算法向量的邻近点距离范围;
    PointCloud_& computeNormalsInRadius(Float radius);
    /// @brief 计算各点的法向量
    PointCloud_& computeNormals() { return computeNormalsInKnn(16); }
    /// @brief 返回点云中最近点索引.
    /// @return 返回最近点索引列表;
    std::vector<size_t> nearestIndexs() const;
    /// @brief 获取点云中各个点的邻近点索引
    /// @param k 搜索邻近点的个数;
    /// @return 返回邻近点的索引列表;
    std::vector<std::vector<size_t>> neighborIndexs(int k) const;
    /// @brief 体素滤波
    /// @param voxel_leaf 最小体素的长度;
    /// @return 返回自身的引用;
    virtual PointCloud_& voxelFilter(Float voxel_leaf, bool meancenter = false);
    /// @brief 检测关键点的快速调用
    /// @param radius 检测关键点的邻近点范围;
    /// @return 返回关键点序列;
    // Point3Vec<Float>              detectKeyPoints(Float radius = .0f) const;
    virtual size_t                getPointNum() const { return points.size(); }
    virtual const Point3_<Float>* getPointData() const { return points.data(); }
    virtual const Point3_<Float>* getNormalData() const
    {
        return hasNormal() ? normals.data() : nullptr;
    }
    virtual const uchar* getPixelData() const { return nullptr; }
    virtual PixelType    getPixelType() const { return PixelType::UINT8; }

    Point3Vec<Float> points;   ///< 坐标
    Point3Vec<Float> normals;  ///< 法向
};
using PointCloud = PointCloud_<float>;
template <typename Float> struct CoordTraits<PointCloud_<Float>> {
    typedef Float type;
};
/// @brief 彩色点云
/// @details 与PointCloud类相比仅仅增加一个成员变量pixels,用于储存像素值.
/// @tparam 像素类型，常用的如RGB,RGBA,FLOAT等.
/// @tparam Float 三维点的基本类型,仅支持float或double.
template <typename Float, PixelType pixType = PixelType::UINT8>
struct PixelPointCloud_: public PointCloud_<Float> {
    typedef pixel_traits_t<pixType> Pixel;
    typedef PointCloud_<Float>      CloudBase;
    PixelPointCloud_() : CloudBase() {}
    PixelPointCloud_(ConstStr& path) { load(path); }
    explicit PixelPointCloud_(const CloudBase& cloud) : CloudBase(cloud) {}
    PixelPointCloud_(PixelPointCloud_&& cloud) : CloudBase(cloud)
    {
        std::swap(pixels, cloud.pixels);
    }
    PixelPointCloud_(const PixelPointCloud_& cloud)
        : CloudBase(cloud), pixels(cloud.pixels.begin(), cloud.pixels.end())
    {}
    PixelPointCloud_(const IRGBDImage& rgbd, const CameraP& camera)
    {
        Image3_<Float> vmap, nmap;
        depth2Vmap(Image_<Float>(rgbd.rangeImage()), camera, vmap);
        vmap2Nmap<Float>(vmap, nmap);
        Image8u mask;
        vmapAndNmapToMask<Float>(vmap, nmap, mask);
        cvtImgToVec(vmap, mask, CloudBase::points);
        cvtImgToVec(nmap, mask, CloudBase::normals);
        auto color = Image_<Pixel>(rgbd.colorImage());
        color.resize(mask.size());
        cvtImgToVec<Pixel>(color, mask, pixels);
    }
    virtual ~PixelPointCloud_() {}
    PixelPointCloud_& operator=(PixelPointCloud_&& cloud)
    {
        std::swap(pixels, cloud.pixels);
        std::swap(CloudBase::points, cloud.points);
        std::swap(CloudBase::normals, cloud.normals);
        return *this;
    }
    PixelPointCloud_& operator=(const PixelPointCloud_& cloud)
    {
        pixels.assign(cloud.pixels.begin(), cloud.pixels.end());
        CloudBase::points.assign(cloud.points.begin(), cloud.points.end());
        CloudBase::normals.assign(cloud.normals.begin(), cloud.normals.end());
        return *this;
    }
    virtual bool load(ConstStr& path) { return readPointCloud(path, *this); }
    virtual bool save(ConstStr& path) const
    {
        return writePointCloud(path, *this);
    }
    /// @brief 是否存在像素
    virtual bool hasPixel() const
    {
        return !CloudBase::empty() && CloudBase::size() == pixels.size();
    }
    /// @brief 重置彩色点云为指定大小
    virtual void resize(size_t sz) { CloudBase::resize(sz), pixels.resize(sz); }
    /// @brief 体素滤波
    /// @param voxel_leaf 体素最小单元的长度;
    /// @return 返回点云类的自身引用;
    virtual PixelPointCloud_& voxelFilter(
        Float voxel_leaf, bool meancenter = false);
    /// @brief 返回灰度像素的内存指针
    /// @return 仅当像素为uchar类型时返回;
    virtual const uchar* getPixelData() const
    {
        return (std::is_same<Pixel, uchar>::value && hasPixel()) ?
                   reinterpret_cast<const uchar*>(pixels.data()) :
                   nullptr;
    }
    /// @brief 返回像素类型
    virtual PixelType getPixelType() const { return pixType; }
    template <typename Tp = uchar> std::vector<Tp> getPointPixels() const
    {
        return PixelGetter_<typename std::decay<decltype(*this)>::type,
            Tp>::get(*this);
    }

    std::vector<Pixel> pixels;  ///< 像素
};
using RGBPointCloud  = PixelPointCloud_<float, PixelType::RGB>;
using RGBAPointCloud = PixelPointCloud_<float, PixelType::RGBA>;
using GRAYPointCloud = PixelPointCloud_<float, PixelType::UINT8>;
template <typename Float, PixelType Type>
struct CoordTraits<PixelPointCloud_<Float, Type>> {
    typedef Float type;
};

template <typename Tp, typename Float, PixelType pixType>
struct PixelGetter_<Tp, PixelPointCloud_<Float, pixType>> {
    static inline std::vector<Tp> get(
        const PixelPointCloud_<Float, pixType>& cloud)
    {
        std::vector<Tp> pixs;
        if (cloud.hasPixel()) {
            pixs.resize(cloud.pixels.size());
            CvtPixel_<pixel_traits_t<pixType>, Tp>::convert(
                cloud.pixels.data(), pixs.data(), pixs.size());
        }
        return pixs;
    }
};

// 极大值抑制
template <typename Float, typename Tp>
std::vector<bool> suppressNonMaximum(const Point3Vec<Float>& points,
    const std::vector<Tp>& values, double radius)
{
    std::vector<bool> flags;
    if (points.empty() || points.size() != values.size()) return flags;
    flags.resize(points.size(), true);
    const CloudAdaptor_<Float> pc2kd(&points[0], points.size());
    CloudKDTree<Float> index(3, pc2kd, KDTreeSingleIndexAdaptorParams(10));
    std::vector<std::pair<size_t, Float>> indices_dists;
    RadiusResultSet<Float, size_t>        result(
        (Float)(radius * radius), indices_dists);
    for (size_t i = 0; i < points.size(); ++i) {
        indices_dists.clear();
        const auto& query_val = values[i];
        index.findNeighbors(result, &points[i].x, SearchParams());
        for (size_t j = 0; j < indices_dists.size(); ++j) {
            auto& ind = indices_dists[j].first;
            if (flags[ind] && values[ind] < query_val) flags[ind] = false;
        }
    }
    return flags;
}

template <typename Float> std::vector<std::vector<size_t>> findNeighbors(
    const CloudKDTree<Float>& index, int k = 10)
{
    std::vector<std::vector<size_t>> neighbors;
    if (k > 0) {
        auto& cloud = index.dataset.derived();
        neighbors.resize(cloud.size(), std::vector<size_t>(k));
        std::vector<Float>  dists(k);
        KNNResultSet<Float> result(k);
        for (size_t i = 0; i < cloud.size(); ++i) {
            result.init(&neighbors[i][0], &dists);
            index.findNeighbors(result, &cloud.points[i].x, SearchParams(10));
        }
    }
    return neighbors;
}

template <typename Float> std::vector<std::vector<size_t>> findNeighbors(
    const IPointCloud_<Float>& cloud, int k = 10)
{
    const CloudAdaptor_<Float> pc2kd(cloud);
    CloudKDTree<Float> index(3, pc2kd, KDTreeSingleIndexAdaptorParams(10));
    return findNeighbors<Float>(index, k);
}

template <typename Float>
std::vector<std::vector<size_t>> PointCloud_<Float>::neighborIndexs(int k) const
{
    return findNeighbors(*this, k);
}

template <typename Float>
std::vector<size_t> PointCloud_<Float>::nearestIndexs() const
{
    std::vector<size_t> nearest_inds;
    if (!this->empty()) {
        nearest_inds.resize(this->size());
        std::vector<size_t>        inds(2);
        std::vector<Float>         dists(2);
        KNNResultSet<Float>        result(2);
        const CloudAdaptor_<Float> pc2kd(*this);
        CloudKDTree<Float> index(3, pc2kd, KDTreeSingleIndexAdaptorParams(10));
        for (size_t i = 0; i < this->size(); ++i) {
            result.init(&inds[0], &dists[0]);
            index.findNeighbors(result, &points[i].x, SearchParams(10));
            nearest_inds[i] = i == inds[0] ? inds[1] : inds[0];
        }
    }
    return nearest_inds;
}

template <typename Float>
static inline Float checkDist(const CloudKDTree<Float>& index, int k)
{
    if (k <= 0) return (Float)0;
    Float        dist      = 0;
    const auto*  points    = index.dataset.points;
    const size_t point_num = index.dataset.point_num;
    if (k <= 1 || !point_num) return Float(0);
    std::vector<size_t>         inds(k);
    std::vector<Float>          dists(k);
    KNNResultSet<Float, size_t> result(k);
    for (size_t i = 0; i < point_num; ++i) {
        result.init(&inds[0], &dists[0]);
        index.findNeighbors(result, &points[i].x, SearchParams(10));
        dist += dists.back();
    }
    return std::sqrt(dist / (Float)point_num);
}
template <typename Float>
static inline Float checkDist(const IPointCloud_<Float>& cloud, int k)
{
    const CloudAdaptor_<Float> pc2kd(cloud);
    CloudKDTree<Float> index(3, pc2kd, KDTreeSingleIndexAdaptorParams(10));
    return checkDist(index, k);
}
template <typename Float> Float PointCloud_<Float>::checkDist(int k) const
{
    return rulermvs::checkDist(*this, k);
}

template <typename Float>
static inline Float computeMeanDistToView(const PointCloud_<Float>& cloud,
    const Point3_<Float>& view = Point3_<Float> {0, 0, 0})
{
    Float        dist      = 0;
    const size_t point_num = cloud.points.size();
    for (size_t i = 0; i < point_num; ++i) {
        auto tem = (cloud.points[i] - view).norm();
        dist += static_cast<Float>(tem);
    }
    return dist / (Float)point_num;
}
template <typename Float> Float PointCloud_<Float>::computeMeanDistToView(
    const Point3_<Float>& view) const
{
    return rulermvs::computeMeanDistToView(*this, view);
}

template <typename Float>
PointCloud_<Float>& PointCloud_<Float>::computeNormalsInKnn(size_t k)
{
    if (this->points.empty()) return *this;
    const CloudAdaptor_<Float> pc2kd(*this);
    CloudKDTree<Float> index(3, pc2kd, KDTreeSingleIndexAdaptorParams(10));
    if (!hasNormal()) normals.resize(this->size());
    rulermvs::computeNormalsInKnn(index, &normals[0], k);
    return *this;
}
template <typename Float>
PointCloud_<Float>& PointCloud_<Float>::recomputeNormalsInKnn(size_t k)
{
    if (this->points.empty()) return *this;
    const CloudAdaptor_<Float> pc2kd(*this);
    CloudKDTree<Float> index(3, pc2kd, KDTreeSingleIndexAdaptorParams(10));
    if (!hasNormal()) normals.resize(this->size());
    rulermvs::recomputeNormalsInKnn(index, &normals[0], k);
    return *this;
}
template <typename Float>
PointCloud_<Float>& PointCloud_<Float>::computeNormalsInRadius(Float radius)
{
    const CloudAdaptor_<Float> pc2kd(*this);
    CloudKDTree<Float> index(3, pc2kd, KDTreeSingleIndexAdaptorParams(10));
    size_t             covraiance_byte = 9 * sizeof(double);
    double             covariance[9]   = {0}, X[3];
    Point3_<Float>     pt_vec;
    std::vector<std::pair<size_t, Float>> indices_dists;
    RadiusResultSet<Float, size_t> result(radius * radius, indices_dists);
    for (size_t i = 0; i < size(); ++i) {
        indices_dists.clear();
        const auto& query_pt = points[i];
        index.findNeighbors(result, &points[i].x, SearchParams());
        memset(&covariance[0], 0, covraiance_byte);
        for (size_t j = 0; j < indices_dists.size(); ++j) {
            pt_vec = points[indices_dists[j].first] - query_pt;
            covariance[0] += pt_vec.x * pt_vec.x;
            covariance[1] += pt_vec.x * pt_vec.y;
            covariance[2] += pt_vec.x * pt_vec.z;
            covariance[4] += pt_vec.y * pt_vec.y;
            covariance[5] += pt_vec.y * pt_vec.z;
            covariance[8] += pt_vec.z * pt_vec.z;
        }
        covariance[3] = covariance[1];
        covariance[6] = covariance[2];
        covariance[7] = covariance[5];
        solveZ3x3(covariance, X);
        normals[i] = rulermvs::normalize(Point3_<Float>(X[0], X[1], X[2]));
        if (normals[i].dot(query_pt) < 0) normals[i] = -normals[i];
    }
    return *this;
}
template <typename Float> void voxelFilter(const PointCloud_<Float>& src,
    coord_traits_t<Float> sz, PointCloud_<Float>& dst)
{
    if (src.empty()) return;
    std::vector<size_t> pt_inds;
    voxelFilter<Float>(&src.points[0], src.size(), sz, pt_inds);
    if (src.hasNormal()) {
        dst.resize(pt_inds.size());
        for (size_t i = 0; i < pt_inds.size(); ++i) {
            dst.points[i]  = src.points[pt_inds[i]];
            dst.normals[i] = src.normals[pt_inds[i]];
        }
    } else {
        dst.points.resize(pt_inds.size());
        for (size_t i = 0; i < pt_inds.size(); ++i)
            dst.points[i] = src.points[pt_inds[i]];
    }
}
template <typename Float> void voxelGridFilter(
    PointCloud_<Float>& src, coord_traits_t<Float> sz, PointCloud_<Float>& dst)
{
    if (src.empty()) return;
    std::vector<size_t> pt_inds;
    if (src.hasNormal()) {
        voxelGridFilter<Float>(
            &src.points[0], &src.normals[0], src.size(), sz, pt_inds);
        dst.resize(pt_inds.size());
        for (size_t i = 0; i < pt_inds.size(); ++i) {
            dst.points[i]  = src.points[pt_inds[i]];
            dst.normals[i] = src.normals[pt_inds[i]];
        }
    } else {
        Point3Vec<Float> normals(src.size());
        voxelGridFilter<Float>(
            &src.points[0], &normals[0], src.size(), sz, pt_inds);
        dst.points.resize(pt_inds.size());
        for (size_t i = 0; i < pt_inds.size(); ++i)
            dst.points[i] = src.points[pt_inds[i]];
    }
}
template <typename Float>
PointCloud_<Float>& PointCloud_<Float>::voxelFilter(Float sz, bool meancenter)
{
    PointCloud_<Float> dst;
    if (!meancenter) {
        rulermvs::voxelFilter<Float>(*this, sz, dst);
    } else {
        rulermvs::voxelGridFilter<Float>(*this, sz, dst);
    }
    return *this = std::move(dst);
}
template <typename Float, PixelType pixType>
void voxelFilter(const PixelPointCloud_<Float, pixType>& src,
    coord_traits_t<Float> sz, PixelPointCloud_<Float, pixType>& dst)
{
    if (src.empty()) return;
    if (src.hasPixel()) {
        std::vector<size_t> pt_inds;
        voxelFilter<Float>(&src.points[0], src.size(), sz, pt_inds);
        if (src.hasNormal()) {
            dst.resize(pt_inds.size());
            for (size_t i = 0; i < pt_inds.size(); ++i) {
                dst.pixels[i]  = src.pixels[pt_inds[i]];
                dst.points[i]  = src.points[pt_inds[i]];
                dst.normals[i] = src.normals[pt_inds[i]];
            }
        } else {
            dst.pixels.resize(pt_inds.size());
            dst.points.resize(pt_inds.size());
            for (size_t i = 0; i < pt_inds.size(); ++i) {
                dst.pixels[i] = src.pixels[pt_inds[i]];
                dst.points[i] = src.points[pt_inds[i]];
            }
        }
        return;
    }
    voxelFilter<Float>(
        *(const PointCloud_<Float>*)&src, sz, *(PointCloud_<Float>*)&dst);
}
template <typename Float, PixelType pixType>
void voxelGridFilter(PixelPointCloud_<Float, pixType>& src,
    coord_traits_t<Float> sz, PixelPointCloud_<Float, pixType>& dst)
{
    if (src.empty()) return;
    if (src.hasPixel()) {
        std::vector<size_t> pt_inds;
        if (src.hasNormal()) {
            voxelGridFilter<Float>(
                &src.points[0], &src.normals[0], src.size(), sz, pt_inds);
            dst.resize(pt_inds.size());
            for (size_t i = 0; i < pt_inds.size(); ++i) {
                dst.pixels[i]  = src.pixels[pt_inds[i]];
                dst.points[i]  = src.points[pt_inds[i]];
                dst.normals[i] = src.normals[pt_inds[i]];
            }
        } else {
            Point3Vec<Float> normals(src.size());
            voxelGridFilter<Float>(
                &src.points[0], &normals[0], src.size(), sz, pt_inds);
            dst.pixels.resize(pt_inds.size());
            dst.points.resize(pt_inds.size());
            for (size_t i = 0; i < pt_inds.size(); ++i) {
                dst.pixels[i] = src.pixels[pt_inds[i]];
                dst.points[i] = src.points[pt_inds[i]];
            }
        }
        return;
    }
    voxelGridFilter<Float>(*static_cast<PointCloud_<Float>*>(&src), sz,
        *static_cast<PointCloud_<Float>*>(&dst));
}
template <typename Float, PixelType pixType>
PixelPointCloud_<Float, pixType>& PixelPointCloud_<Float, pixType>::voxelFilter(
    Float sz, bool meancenter /* = false*/)
{
    PixelPointCloud_<Float, pixType> dst;
    if (!meancenter) {
        rulermvs::voxelFilter<Float, pixType>(*this, sz, dst);
    } else {
        rulermvs::voxelGridFilter<Float, pixType>(*this, sz, dst);
    }
    return *this = std::move(dst);
}

/// @brief 寻找点云之间的匹配点
/// @tparam Float 浮点类型
/// @param query  查询点云;
/// @param train  目标点云;
/// @param matchs 匹配结果;
/// @param max_dist 点对之间的最大距离;
/// @param max_angle 如果最大角大于0且点云存在法向量才利用此阈值;
/// @return 返回匹配点占比
template <typename Float> static inline double findPointCorres(
    const CloudKDTree<Float>& index, const Point3_<Float>* query_pts,
    const Point3_<coord_traits_t<Float>>* query_nls, size_t query_num,
    DMatchVec& matchs, double max_dist, double max_angle = MVS_PI_4)
{
    if (!query_pts || !query_num) return 0;
    KNNResultSet<Float, size_t> result_set(1);
    matchs.clear(), matchs.reserve(query_num);
    typename decltype(result_set)::IndexType    ind;
    typename decltype(result_set)::DistanceType dist;
    // const auto* train_pts   = index.dataset.points;
    const auto* train_nls   = index.dataset.normals;
    const auto  SQUARE_DIST = (Float)(max_dist * max_dist);
    // 如果存在法向量且给出角度约束值，则利用此约束进行计算;
    if (max_angle > 0 && query_nls && train_nls) {
        const auto MINCOS_ANGLE = (Float)std::cos(max_angle);
        for (size_t i = 0; i < query_num; ++i) {
            result_set.init(&ind, &dist);
            index.findNeighbors(result_set, &query_pts[i].x, SearchParams());
            if (dist < SQUARE_DIST &&
                query_nls[i].dot(train_nls[ind]) > MINCOS_ANGLE)
                matchs.emplace_back((int)i, (int)ind);
        }
    } else {
        for (size_t i = 0; i < query_num; ++i) {
            result_set.init(&ind, &dist);
            index.findNeighbors(result_set, &query_pts[i].x, SearchParams());
            if (dist < SQUARE_DIST) matchs.emplace_back((int)i, (int)ind);
        }
    }
    return (double)matchs.size() / (double)query_num;
}
template <typename Float>
static inline double findPointCorres(const Point3_<Float>* query_pts,
    size_t query_num, const Point3_<Float>* train_pts, size_t train_num,
    const Pose& rt, DMatchVec& matchs, double max_dist)
{
    const CloudAdaptor_<Float> pc2kd(train_pts, train_num);
    CloudKDTree<Float> index(3, pc2kd, KDTreeSingleIndexAdaptorParams(10));
    Point3Vec<Float>   trans_pts(query_num);
    rt.transform(query_pts, &trans_pts[0], query_num);
    return findPointCorres<Float>(
        index, &trans_pts[0], nullptr, query_num, matchs, max_dist);
}
template <typename Float> static inline double findPointCorres(
    const Point3_<Float>* query_pts, const Point3_<Float>* query_nls,
    size_t query_num, const Point3_<Float>* train_pts,
    const Point3_<Float>* train_nls, size_t train_num, const Pose& rt,
    DMatchVec& matchs, double max_dist, double max_angle = MVS_PI_4)
{
    // 构建KDTree
    const CloudAdaptor_<Float> pc2kd(train_pts, train_nls, train_num);
    CloudKDTree<Float> index(3, pc2kd, KDTreeSingleIndexAdaptorParams(10));
    // 查询对应点
    Point3Vec<Float> trans_pts(query_num), trans_nls(query_num);
    rt.rotate(query_nls, &trans_nls[0], query_num);
    rt.transform(query_pts, &trans_pts[0], query_num);
    return findPointCorres<Float>(index, &trans_pts[0], &trans_nls[0],
        query_num, matchs, max_dist, max_angle);
}
template <typename Float>
static inline double findPointCorres(const IPointCloud_<Float>& query,
    const IPointCloud_<Float>& train, const Pose& rt, DMatchVec& matchs,
    double max_dist, double max_angle = MVS_PI_4)
{
    const CloudAdaptor_<Float> pc2kd(train);
    CloudKDTree<Float> index(3, pc2kd, KDTreeSingleIndexAdaptorParams(10));
    return findPointCorres(index, query.getPointData(), query.getNormalData(),
        query.getPointNum(), rt, matchs, max_dist, max_angle);
}

/// @brief 点云读取的辅助函数
/// @param path 文件路径[ASC;PLY]
/// @param pts 点云数组, 为空则返回FALSE.
/// @param nls 法线数组, 可以为空.
/// @param pix 像素素组, 可以为空.
/// @return 返回是否成功读取
extern "C" MVS_EXPORT bool read_points_and_normals(
    const char* path, Point3fVec& pts, Point3fVec& nls, RGBPixelVec& pix);

/// @brief 点云保存的辅助函数
/// @param path 文件路径[ASC;PLY]
/// @param pts 点云数组
/// @param nls 法线数组, 不存在输入nullptr
/// @param pix 像素数组, 不存在输入nullptr
/// @param sz 点云数量
/// @return 返回是否成功保存
extern "C" MVS_EXPORT bool write_points_and_normals(const char* path,
    const Point3f* pts, const Point3f* nls, const RGBPixel* pix, size_t num);

template <> struct CloudReader_<PointCloud> {
    static inline bool read(ConstStr& path, PointCloud& cloud)
    {
        RGBPixelVec rgbs;
        return read_points_and_normals(
            path.c_str(), cloud.points, cloud.normals, rgbs);
    }
};
template <> struct CloudReader_<RGBPointCloud> {
    static inline bool read(ConstStr& path, RGBPointCloud& cloud)
    {
        return read_points_and_normals(
            path.c_str(), cloud.points, cloud.normals, cloud.pixels);
    }
};
template <> struct CloudWriter_<PointCloud> {
    static inline bool write(ConstStr& path, const PointCloud& cloud)
    {
        return write_points_and_normals(path.c_str(), cloud.points.data(),
            cloud.normals.data(), nullptr, cloud.size());
    }
};
template <> struct CloudWriter_<RGBPointCloud> {
    static inline bool write(ConstStr& path, const RGBPointCloud& cloud)
    {
        return write_points_and_normals(path.c_str(), cloud.points.data(),
            cloud.normals.data(), cloud.pixels.data(), cloud.size());
    }
};
template <typename Cloud> struct CloudWriter_<Cloud,
    typename std::enable_if<
        std::is_base_of<IPointCloud, Cloud>::value>::type> {
    static inline bool write(ConstStr& path, const Cloud& cloud)
    {
        if (cloud.getPixelType() == PixelType::RGB && cloud.getPixelData())
            return write_points_and_normals(path.c_str(), cloud.getPointData(),
                cloud.getNormalData(), (const RGBPixel*)cloud.getPixelData(),
                cloud.getPointNum());
        // 尝试转换像素
        auto pixels = PixelGetter_<RGBPixel, Cloud>::get(cloud);
        return write_points_and_normals(path.c_str(), cloud.points.data(),
            cloud.normals.data(), pixels.data(), cloud.getPointNum());
    }
};
}  // namespace rulermvs
// RULERMVS_READ_AND_WRITE_FUNC_WITH_NAME(RGBPointCloud, Cloud)
#endif  // _RULERMVS_CORE_POINTCLOUD_HPP_