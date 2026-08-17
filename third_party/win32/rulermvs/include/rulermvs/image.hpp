#ifndef _RULERMVS_CORE_IMAGE_HPP_
#define _RULERMVS_CORE_IMAGE_HPP_
#include "rulermvs/core.hpp"
#include "rulermvs/size.hpp"
#include "rulermvs/math.hpp"
#include "rulermvs/pixel.hpp"
#include "rulermvs/point.hpp"
namespace rulermvs
{
// 前置声明
enum class ImageOpType;
template <typename T> struct AutoDeleter_;
template <typename _BinaryOp, ImageOpType opType> struct ImageUnaryOp_;
template <typename ImageT, typename Tp, ImageOpType opType>
struct ImageBinaryOp_;

/// @brief 图像类型定义
template <typename T> struct Image_: IImage, AutoDeleter_<T> {
    typedef T Type;
    static const Image_ Empty;
    Image_() : data(nullptr), width(0), height(0), stride(-1) {}
    template <typename BinaryOp, ImageOpType OpType>
    Image_(const ImageUnaryOp_<BinaryOp, OpType>& op)
    {
        op >> *this;
    }
    template <typename ImageT, typename Tp, ImageOpType opType>
    Image_(const ImageBinaryOp_<ImageT, Tp, opType>& op)
    {
        op >> *this;
    }
    Image_(const std::string& path) : Image_() { load(path); }
    Image_(Size sz) : width(sz.width), height(sz.height)
    {
        if (width <= 0 || height <= 0) width = height = 0;
        stride = width * (int)sizeof(T);
        AutoDeleter_<T>::try_create_memory(data, stride * height);
    }
    Image_(int w, int h) : Image_(Size {w, h}) {}
    Image_(Size sz, void* in, int in_stride = -1, bool copy = false)
    {
        if (copy) {
            create(sz, in, in_stride);
        } else {
            data = (T*)in, width = sz.width, height = sz.height;
            stride = in_stride <= 0 ? sz.width * (int)sizeof(T) : in_stride;
        }
    }
    Image_(int w, int h, void* in, int in_stride = -1, bool copy = false)
        : Image_(Size {w, h}, in, in_stride, copy)
    {}
    // Image_(Image_&& img) noexcept
    //     : AutoDeleter_<T>(img)
    //     , data(img.data)
    //     , width(img.width)
    //     , height(img.height)
    //     , stride(img.stride)
    // {
    //     // 将右值对象地址置空, 防止析构时删除内存.
    //     img = Empty;
    // }
    Image_(const Image_& img)
        : AutoDeleter_<T>(img)
        , data(img.data)
        , width(img.width)
        , height(img.height)
        , stride(img.stride)
    {}
    /// @brief 构造函数
    /// @param covert 是否自动转换
    explicit Image_(const IImage& img, bool convert = false);
    // Image_& operator=(Image_&& img) noexcept
    // {
    //     std::swap(data, img.data);
    //     std::swap(width, img.width);
    //     std::swap(height, img.height);
    //     std::swap(stride, img.stride);
    //     std::swap(AutoDeleter_<T>::mem_, img.mem_);
    //     // 这里修复右值传值错误.
    //     img = Empty;
    //     return *this;
    // }
    Image_& operator=(const Image_& img)
    {
        AutoDeleter_<T>::mem_ = img.mem_;
        data = img.data, stride = img.stride;
        width = img.width, height = img.height;
        return *this;
    }
    virtual ~Image_()
    {
        data = nullptr;
        width = height = stride = 0;
        AutoDeleter_<T>::mem_ = nullptr;
    }
    RULERMVS_SMART_CONVERT_MEMBER_FUNC(Image_<T>)

    bool load(const std::string& path);
    bool save(const std::string& path) const;
    void resize(Size sz);
    void resize(int w, int h) { return resize(Size {w, h}); }
    /// 清除数据.
    void clear() { *this = Empty; }
    /// @brief 获取图像的尺寸信息.
    Size size() const { return {width, height}; }
    /// @brief 判断实例是否为空.
    bool empty() const { return width <= 0 || height <= 0 || data == nullptr; }
    /// @brief 是否为整块内存
    bool unbiased() const
    {
        return stride <= 0 || stride == width * static_cast<int>(sizeof(T));
    }
    /// @brief 将内存中的值赋值为0.
    void memsetZero()
    {
        if (this->empty() || stride <= 0) return;
        const int WByte = width * (int)sizeof(T);
        if (stride == WByte) {
            memset(data, 0, (size_t)height * WByte);
        } else {
            for (int i = 0; i < height; ++i) memset(ptr(i), 0, WByte);
        }
    }
    /// @brief 将内存中的值赋值为-1.
    void memsetNegOne()
    {
        if (empty() || stride <= 0) return;
        const int WSize = width * (int)sizeof(T);
        if (stride == WSize) {
            memset(data, -1, height * WSize);
            return;
        }
        for (int i = 0; i < height; ++i) memset(ptr(i), -1, WSize);
    }
    /// @brief 获取对应行数据的起始指针.
    template <typename Tp = T> Tp* ptr(int row_ind = 0) const
    {
        return (Tp*)&((uchar*)data)[row_ind * stride];
    }
    /// @brief 保持数据
    Image_ keep() const
    {
        return AutoDeleter_<T>::mem_ &&
                       (T*)AutoDeleter_<T>::mem_.get() == data ?
                   Image_(*this) :
                   clone();
    }
    /// @brief 深拷贝内存
    Image_ clone() const { return Image_ {width, height, data, stride, true}; }
    ///@brief 创建图像空间,并拷贝内存数据
    void create(Size sz, void* in = nullptr, int in_stride = -1)
    {
        if (sz.width <= 0 || sz.height <= 0) return;
        if (width != sz.width || height != sz.height) {
            stride = sz.width * (int)sizeof(T);
            width = sz.width, height = sz.height;
            AutoDeleter_<T>::try_create_memory(data, height * stride);
        }
        if (in == nullptr) return;
        if (stride == width * (int)sizeof(T) && in_stride == stride) {
            memcpy(data, in,
                static_cast<size_t>(height) * static_cast<size_t>(stride));
        } else {
            in_stride = in_stride <= 0 ? width * (int)sizeof(T) : in_stride;
            for (int i = 0; i < height; ++i) {
                memcpy(ptr(i),
                    &((uchar*)in)[static_cast<size_t>(i) *
                                  static_cast<size_t>(in_stride)],
                    (size_t)width * sizeof(T));
            }
        }
    }
    void create(int w, int h, void* in = nullptr, int in_stride = -1)
    {
        create(Size(w, h), in, in_stride);
    }
    /// @brief 返回掩模指示的像素数组.
    std::vector<T> operator[](const Image_<uchar>& m) const;
    /// @brief 返回图像宽
    virtual int getWidth() const { return width; }
    /// @brief 返回图像高
    virtual int getHeight() const { return height; }
    /// @brief 返回内存偏移量
    virtual int getStride() const { return stride; }
    /// @brief 返回像素类型
    virtual PixelType getType() const { return cvtPixelType<T>(); }
    /// @brief 返回图像内存起始地址
    virtual const uchar* getData() const { return (uchar*)data; }

    T* data;     ///< 内存指针
    int width;   ///< 图像宽
    int height;  ///< 图像高
    int stride;  ///< 迭代行宽
};
template <typename T> const Image_<T> Image_<T>::Empty = Image_<T>();
template <typename T> struct CoordTraits<Image_<T>>
    : CoordType<coord_traits_t<T>> {};
template <typename T> using Image2_ = Image_<Point2_<T>>;
template <typename T> using Image3_ = Image_<Point3_<T>>;
template <typename T> using ImageVec_ = std::vector<Image_<T>>;

using Imagei = Image_<int>;               ///< 索引图
using Imagef = Image_<float>;             ///< 浮点图像
using Imaged = Image_<double>;             ///< 浮点图像
using Image8u = Image_<uchar>;            ///< 灰度图
using Image3f = Image_<Point3f>;          ///< 高动态、vmap、nmap等
using Image3d = Image_<Point3d>;          ///< 高动态、vmap、nmap等
using Image16u = Image_<ushort>;          ///< 深度图【16位】
using GRAYImage = Image_<uchar>;          ///< 灰度图
using RGBImage = Image_<RGBPixel>;        ///< 彩色图像
using RGBAImage = Image_<RGBAPixel>;      ///< 彩色图像【透明通道】
using ImagefVec = ImageVec_<float>;       ///< 浮点图像数组
using Image8uVec = ImageVec_<uchar>;      ///< 灰度图像数组
using Image3fVec = ImageVec_<Point3f>;    ///< 有序三维点数组
using RGBImageVec = ImageVec_<RGBPixel>;  ///< 彩色图像数组

/// @brief 管理内存自动释放
template <typename T> struct AutoDeleter_ {
    AutoDeleter_() : mem_(nullptr) {}
    AutoDeleter_(const AutoDeleter_<T>& deleter) : mem_(deleter.mem_) {}
    // AutoDeleter_(AutoDeleter_<T>&& deleter) { std::swap(mem_, deleter.mem_);
    // }
    virtual ~AutoDeleter_() { mem_ = nullptr; }

protected:
    inline void try_create_memory(T*& ptr, int len)
    {
        mem_ = nullptr;
#if defined(RULERMVS_USE_SSE)
        if (len > 0) mem_ = {new uchar[len + 16], [](uchar* p) { delete p; }};
        ptr = mem_ ? (T*)(((size_t)mem_.get() + 15) & ~15) : nullptr;
#else
        if (len > 0) mem_ = {new uchar[len], [](uchar* p) { delete p; }};
        ptr = mem_ ? (T*)mem_.get() : nullptr;
#endif
    }

    std::shared_ptr<uchar> mem_;
};

enum class SamplerType : int { MVS_NEAR, MVS_BILINEAR, MVS_CUBIC };
template <typename Tp, typename Float = float,
    SamplerType mode = SamplerType::MVS_NEAR>
struct ImageSampler_ {
    static inline void interpolate(
        const Image_<Tp>& img, Float u, Float v, Tp& val)
    {
        int x = (int)u, y = (int)v;
        if (x >= 0 && x < img.width && y >= 0 && y < img.height)
            val = img.ptr(y)[x];
    }
};
template <typename Tp, typename Float>
struct ImageSampler_<Tp, Float, SamplerType::MVS_BILINEAR> {
    static inline void interpolate(
        const Image_<Tp>& img, Float x, Float y, Tp& val)
    {
        int xi = (int)std::floor(x), yi = (int)std::floor(y);
        int xc = xi + 1, yc = yi + 1;
        if (xc > img.width || xc < 0 || yc > img.height || yc < 0) return;
        if (xc == 0) {
            if (yc == 0) {
                val = img.ptr(yc)[xc];
            } else if (yc == img.height) {
                val = img.ptr(yi)[xc];
            } else {
                Float a = (Float)yc - y, b = y - (Float)yi;
                val = (Tp)(img.ptr(yi)[xc] * a + img.ptr(yc)[xc] * b);
            }
        } else if (xc == img.width) {
            if (yc == 0) {
                val = img.ptr(yc)[xi];
            } else if (yc == img.height) {
                val = img.ptr(yi)[xi];
            } else {
                Float a = (Float)yc - y, b = y - (Float)yi;
                val = (Tp)(img.ptr(yi)[xi] * a + img.ptr(yc)[xi] * b);
            }
        } else {
            if (yc == 0) {
                Float a = (Float)xc - x, b = x - (Float)xi;
                val = (Tp)(img.ptr(yc)[xi] * a + img.ptr(yc)[xc] * b);
            } else if (yc == img.height) {
                Float a = (Float)xc - x, b = x - (Float)xi;
                val = (Tp)(img.ptr(yi)[xi] * a + img.ptr(yi)[xc] * b);
            } else {
                Float a0 = (Float)xc - x, b0 = x - (Float)xi;
                Float a1 = (Float)yc - y, b1 = y - (Float)yi;
                val = (Tp)((img.ptr(yi)[xi] * a0 + img.ptr(yi)[xc] * b0) * a1 +
                           (img.ptr(yc)[xi] * a0 + img.ptr(yc)[xc] * b0) * b1);
            }
        }
    }
};
template <typename Tp, typename Float = float,
    SamplerType mode = SamplerType::MVS_NEAR>
static inline void interpolate(const Image_<Tp>& img, Float x, Float y, Tp& val)
{
    ImageSampler_<Tp, Float, mode>::interpolate(img, x, y, val);
}

template <typename T1, typename T2, typename = void> struct CvtColor_ {
    static inline void convert(const Image_<T1>& src, Image_<T2>& dst)
    {
        if (src.empty()) return;
        dst.create(src.size());
        for (int i = 0; i < src.height; ++i)
            cvtPixel(src.ptr(i), dst.ptr(i), src.width);
    }
};
template <typename T> struct CvtColor_<T, T> {
    static inline void convert(const Image_<T>& src, Image_<T>& dst)
    {
        // 这里判断dst和src是否指向相同的内存
        if (src.size() == dst.size() && src.data == dst.data &&
            src.stride == dst.stride)
            return;
        dst.create(src.size(), src.data, src.stride);
    }
};
template <typename T1, typename T2, typename = void>
static inline void cvtColor(const Image_<T1>& src, Image_<T2>& dst)
{
    CvtColor_<T1, T2>::convert(src, dst);
}

// clang-format off
static inline void RGB2GRAY(const RGBImage& src, GRAYImage& dst) { cvtColor(src, dst); }
static inline void GRAY2RGB(const GRAYImage& src, RGBImage& dst) { cvtColor(src, dst); }
static inline void RGB2RGBA(const RGBImage& src, RGBAImage& dst) { cvtColor(src, dst); }
static inline void RGBA2RGB(const RGBAImage& src, RGBImage& dst) { cvtColor(src, dst); }
static inline void GRAY2RGBA(const GRAYImage& src, RGBAImage& dst) { cvtColor(src, dst); }
static inline void RGBA2GRAY(const RGBAImage& src, GRAYImage& dst) { cvtColor(src, dst); }
// clang-format on

static inline void RGB2BGR(const RGBImage& src, RGBImage& dst)
{
    if (src.empty()) return;
    dst.create(src.size());
    for (int i = 0; i < src.height; ++i) {
        auto* src_ptr = src.ptr(i);
        auto* dst_ptr = dst.ptr(i);
        for (int j = 0; j < src.width; ++j)
            dst_ptr[j] = {src_ptr[j].b, src_ptr[j].g, src_ptr[j].r};
    }
}
static inline void RGBA2BGRA(const RGBAImage& src, RGBAImage& dst)
{
    if (src.empty()) return;
    dst.create(src.size());
    for (int i = 0; i < src.height; ++i) {
        auto* src_ptr = src.ptr(i);
        auto* dst_ptr = dst.ptr(i);
        for (int j = 0; j < src.width; ++j) {
            dst_ptr[j] = {
                src_ptr[j].b, src_ptr[j].g, src_ptr[j].r, src_ptr[j].a};
        }
    }
}

static inline void setImageValue(const RGBImage& src, Imagef& dst)
{
    if (src.empty()) return;
    // dst.create(src.size());
     for (int i = 0; i < src.height; ++i) {
        auto* src_ptr = src.ptr(i);
        auto* dst_ptr = dst.ptr(i);
        for (int j = 0; j < src.width; ++j) {
            if (src_ptr[j].b == 0 && src_ptr[j].g == 0 && src_ptr[j].r == 0)
                dst_ptr[j] = 0.0;
        }
    }
}

// 图像类型转换
template <typename T1, typename T2> struct Converter_<Image_<T1>, Image_<T2>,
    typename std::enable_if<std::is_arithmetic<T1>::value &&
                            std::is_arithmetic<T2>::value>::type> {
    static inline void to(
        const Image_<T1>& src, Image_<T2>& dst, double a = 1, double b = 0)
    {
        if (src.empty()) return;
        dst.create(src.size());
        for (int i = 0; i < src.height; ++i) {
            auto* src_ptr = src.ptr(i);
            auto* dst_ptr = dst.ptr(i);
            for (int j = 0; j < src.width; ++j)
                dst_ptr[j] = (T2)(src_ptr[j] * a + b);
        }
    }
};
template <> struct Converter_<Imagef, Image16u> {
    static inline void to(
        const Imagef& src, Image16u& dst, double a = 1, double b = 0)
    {
        if (src.empty()) return;
        dst.create(src.size());
        for (int i = 0; i < src.height; ++i) {
            auto* src_ptr = src.ptr(i);
            auto* dst_ptr = dst.ptr(i);
            for (int j = 0; j < src.width; ++j)
                dst_ptr[j] = (ushort)std::round(
                    clamp<double>(src_ptr[j] * a + b, 0, 65535));
        }
    }
};
template <typename T> struct Converter_<IImage, Image_<T>> {
    static inline void to(const IImage& src, Image_<T>& dst)
    {
        auto data = const_cast<uchar*>(src.getData());
        auto width = src.getWidth();
        auto height = src.getHeight();
        auto stride = src.getStride();
        switch (src.getType()) {
            case PixelType::RGB:
                cvtColor(RGBImage(width, height, data, stride), dst);
                break;
            case PixelType::RGBA:
                cvtColor(RGBAImage(width, height, data, stride), dst);
                break;
            case PixelType::INT8:
                cvtColor(Image_<char>(width, height, data, stride), dst);
                break;
            case PixelType::INT16:
                cvtColor(Image_<short>(width, height, data, stride), dst);
                break;
            case PixelType::INT32:
                cvtColor(Image_<int>(width, height, data, stride), dst);
                break;
            case PixelType::UINT8:
                cvtColor(GRAYImage(width, height, data, stride), dst);
                break;
            case PixelType::UINT16:
                cvtColor(Image_<ushort>(width, height, data, stride), dst);
                break;
            case PixelType::UINT32:
                cvtColor(Image_<uint>(width, height, data, stride), dst);
                break;
            case PixelType::FLOAT32:
                cvtColor(Image_<float>(width, height, data, stride), dst);
                break;
            case PixelType::FLOAT64:
                cvtColor(Image_<double>(width, height, data, stride), dst);
                break;
            case PixelType::UNKNOWN:
            default:
                throw std::runtime_error("Input IImage With Unkonwn Type.");
                break;
        }
    }
};
template <typename T> Image_<T>::Image_(const IImage& img, bool convert)
{
    if (!convert && img.getType() == cvtPixelType<T>()) {
        *this = Image_<T>(img.getWidth(), img.getHeight(),
            const_cast<uchar*>(img.getData()), img.getStride());
    } else {
        convertTo(img, *this);
    }
}

template <typename Tp, SamplerType mode = SamplerType::MVS_BILINEAR>
struct ImageResizer_ {
    static inline void resize(const Image_<Tp>& src, Size sz, Image_<Tp>& dst)
    {
        // 只有当dst所对应内存跟src不一致，且size为输入尺寸时，直接引用dst内存.
        Image_<Tp> img =
            dst.data != src.data && dst.size() == sz ? dst : Image_<Tp>(sz);
        if (!src.empty() && !img.empty()) {
            float ppx = (float)src.width / (float)sz.width;
            float ppy = (float)src.height / (float)sz.height;
            for (int i = 0; i < img.height; ++i) {
                Tp* dst_ptr = img.ptr(i);
                for (int j = 0; j < img.width; ++j) {
                    interpolate<Tp, float, mode>(
                        src, (float)j * ppx, (float)i * ppy, dst_ptr[j]);
                }
            }
        }
        dst = img;
    }
};
template <typename Tp, SamplerType mode = SamplerType::MVS_BILINEAR>
void resizeImage(const Image_<Tp>& src, Size sz, Image_<Tp>& dst)
{
    ImageResizer_<Tp, mode>::resize(src, sz, dst);
}
template <typename Tp, SamplerType mode = SamplerType::MVS_BILINEAR>
Image_<Tp> resizeImage(const Image_<Tp>& src, Size sz)
{
    Image_<Tp> dst;
    ImageResizer_<Tp, mode>::resize(src, sz, dst);
    return dst;
}
template <typename T> void Image_<T>::resize(Size sz)
{
    if (size() == sz) return;
    resizeImage<T, SamplerType::MVS_BILINEAR>(*this, sz, *this);
}

template <typename Tp, typename Float,
    SamplerType mode = SamplerType::MVS_BILINEAR>
struct ImageRemapper_ {
    static inline void remap(const Image_<Tp>& src, const Image_<Float>& mapx,
        const Image_<Float>& mapy, Image_<Tp>& dst)
    {
        if (mapx.empty() || mapy.empty() || (mapx.size() != mapy.size()))
            return;
        Image_<Tp> img(mapx.size());
        for (int i = 0; i < img.height; ++i) {
            Tp* dst_ptr = img.ptr(i);
            Float *mapx_ptr = mapx.ptr(i), *mapy_ptr = mapy.ptr(i);
            for (int j = 0; j < img.width; ++j)
                interpolate<Tp, Float, mode>(
                    src, mapx_ptr[j], mapy_ptr[j], dst_ptr[j]);
        }
        dst = img;
    }
};
template <typename Tp, typename Float,
    SamplerType mode = SamplerType::MVS_BILINEAR>
static inline void remapImage(const Image_<Tp>& src, const Image_<Float>& mapx,
    const Image_<Float>& mapy, Image_<Tp>& dst)
{
    ImageRemapper_<Tp, Float, mode>::remap(src, mapx, mapy, dst);
}
template <typename Tp, typename Float,
    SamplerType mode = SamplerType::MVS_BILINEAR>
static inline Image_<Tp> remapImage(
    const Image_<Tp>& src, const Image_<Float>& mapx, const Image_<Float>& mapy)
{
    Image_<Tp> dst;
    ImageRemapper_<Tp, Float, mode>::remap(src, mapx, mapy, dst);
    return dst;
}

/// @brief 统计影像中的非零个数.
template <typename Tp> static inline int countNonZero(const Image_<Tp>& img)
{
    if (img.empty()) return 0;
    int cnt = 0;
    // #if defined(RULERMVS_USE_OPENMP) && defined(_OPENMP)
    // #pragma omp parallel for reduction(+ : cnt)
    //     for (int i = 0; i < img.height; ++i)
    // #else
    for (int i = 0; i < img.height; ++i)
        cnt += countNonZero(img.ptr(i), static_cast<size_t>(img.width));
    return cnt;
}

/// @brief 图像阵列转到数组
template <typename Tp> static inline int cvtImgToVec(
    const Image_<Tp>& img, const Image8u& mask, std::vector<Tp>& vec)
{
    assert(!img.empty() && img.size() == mask.size());
    // 事先计算数量，避免push_back.
    auto num = countNonZero(mask);
    if (num <= 0) return 0;
    vec.resize(num), num = 0;
    for (int i = 0; i < img.height; ++i) {
        auto img_ptr = img.ptr(i);
        auto mask_ptr = mask.ptr(i);
        for (int j = 0; j < img.width; ++j)
            if (mask_ptr[j]) vec[num++] = img_ptr[j];
    }
    return num;
}
template <typename T>
std::vector<T> Image_<T>::operator[](const Image_<uchar>& m) const
{
    std::vector<T> vec;
    cvtImgToVec(*this, m, vec);
    return vec;
}

/// @brief 计算均值影像
template <typename T> static inline void meanImage(const Image_<T>& src0,
    const Image_<T>& src1, Image_<T>& dst, double alpha = 0.5f)
{
    if (src0.empty() || src0.size() != src1.size()) return;
    if (dst.size() != src0.size()) dst.create(src0.size());
    double beta = 1 - alpha;
    // MVS_OMP_PARALLEL_FOR
    for (int i = 0; i < dst.height; ++i) {
        auto dst_ptr = dst.ptr(i);
        auto src0ptr = src0.ptr(i);
        auto src1ptr = src1.ptr(i);
        for (int j = 0; j < dst.width; ++j)
            dst_ptr[j] = (T)(alpha * src0ptr[j] + beta * src1ptr[j]);
    }
}
/// @brief 计算最小值影像
template <typename T> static inline void minImage(
    const Image_<T>& src0, const Image_<T>& src1, Image_<T>& dst)
{
    assert(!src0.empty() && src0.size() == src1.size());
    if (dst.size() != src0.size()) dst.create(src0.size());
    // MVS_OMP_PARALLEL_FOR
    for (int i = 0; i < dst.height; ++i) {
        auto dst_ptr = dst.ptr(i);
        auto src0ptr = src0.ptr(i);
        auto src1ptr = src1.ptr(i);
        for (int j = 0; j < dst.width; ++j)
            dst_ptr[j] = std::min<T>(src0ptr[j], src1ptr[j]);
    }
}
/// @brief 计算最大值影像
template <typename T> static inline void maxImage(
    const Image_<T>& src0, const Image_<T>& src1, Image_<T>& dst)
{
    assert(!src0.empty() && src0.size() == src1.size());
    if (dst.size() != src0.size()) dst.create(src0.size());
    // MVS_OMP_PARALLEL_FOR
    for (int i = 0; i < dst.height; ++i) {
        auto dst_ptr = dst.ptr(i);
        auto src0ptr = src0.ptr(i);
        auto src1ptr = src1.ptr(i);
        for (int j = 0; j < dst.width; ++j)
            dst_ptr[j] = std::max<T>(src0ptr[j], src1ptr[j]);
    }
}
/// @brief 极大值抑制
/// @param k should bigger than 3 and be odd number.
template <typename Tp> static inline void suppressNonMaximum(
    const Image_<Tp>& img, int k, Image8u& mask)
{
    if (img.empty() || k < 3) return;
    const int half = k >> 1;
    const int width_2 = img.width - half;
    mask.create(img.size()), mask.memsetZero();
    for (int i = half; i < img.height - half; ++i) {
        auto* img_ptr = img.ptr(i);
        auto* mask_ptr = mask.ptr(i);
        for (int j = half; j < width_2; ++j) {
            if (!img_ptr[j]) continue;
            mask_ptr[j] = 255;
            for (int m = -half; m <= half; ++m) {
                auto img_ptr2 = &img.ptr(i + m)[j];
                for (int n = -half; n <= half; ++n) {
                    if (img_ptr2[n] && img_ptr2[n] > img_ptr[j]) {
                        mask_ptr[j] = 0;
                        break;
                    }
                }
                if (!mask_ptr[j]) break;
            }
        }
    }
}
template <typename Tp>
static inline Image8u suppressNonMaximum(const Image_<Tp>& img, int k)
{
    Image8u mask;
    suppressNonMaximum<Tp>(img, k, mask);
    return mask;
}

/// @brief 卷积类
/// @tparam Tp 像素类型
/// @tparam Float 卷积核类型
/// @tparam Type 求导类型
template <typename Tp, typename Float, DerivType Type = DerivType::C>
struct Convolver_ {
    static inline void convolve(const Image_<Tp>& src,
        const std::vector<Float>& ker, Image_<Float>& dst)
    {
        std::vector<Tp> tmp(src.width + (ker.size() >> 1) * 2);
        memset(&tmp[0], 0, tmp.size() * sizeof(Tp));
        dst.create(src.size());
        for (int i = 0; i < src.height; ++i) {
            Float* dst_ptr = dst.ptr(i);
            memcpy(&tmp[ker.size() >> 1], src.ptr(i), src.width * sizeof(Tp));
            for (int j = 0; j < src.width; ++j) {
                Float sum(0);
                for (int k = 0; k < (int)ker.size(); ++k)
                    sum += tmp[j + k] * ker[k];
                dst_ptr[j] = sum;
            }
        }
    }
};

template <typename Tp, typename Float>
struct Convolver_<Tp, Float, DerivType::R> {
    static inline void convolve(const Image_<Tp>& src,
        const std::vector<Float>& ker, Image_<Float>& dst)
    {
        std::vector<Tp> tmp(src.height + (ker.size() >> 1) * 2);
        memset(&tmp[0], 0, tmp.size() * sizeof(Tp));
        dst.create(src.size());
        for (int i = 0; i < src.width; ++i) {
            Tp* tmp_ptr = &tmp[ker.size() >> 1];
            for (int j = 0; j < src.height; ++j) tmp_ptr[j] = src.ptr(j)[i];
            for (int j = 0; j < src.height; ++j) {
                Float sum(0);
                for (int k = 0; k < (int)ker.size(); ++k)
                    sum += tmp[j + k] * ker[k];
                dst.ptr(j)[i] = sum;
            }
        }
    }
};

template <typename Tp, typename Float> static inline void convolveRow(
    const Image_<Tp>& src, const std::vector<Float>& kernel, Image_<Float>& dst)
{
    Convolver_<Tp, Float, DerivType::R>::convolve(src, kernel, dst);
}

template <typename Tp, typename Float> static inline void convolveCol(
    const Image_<Tp>& src, const std::vector<Float>& kernel, Image_<Float>& dst)
{
    Convolver_<Tp, Float, DerivType::C>::convolve(src, kernel, dst);
}

template <typename Tp, typename Float> void convolveGauss(const Image_<Tp>& src,
    coord_traits_t<Float> sigma, DerivType type, Image_<Float>& dst)
{
    switch (type) {
        case DerivType::C:
            convolveRow<Tp, Float>(src, gaussianKernel(sigma), dst);
            convolveCol<Float, Float>(dst, gaussianDerivedKernel(sigma), dst);
            break;
        case DerivType::RR:
            convolveRow<Tp, Float>(src, gaussianDerived2ndKernel(sigma), dst);
            convolveCol<Float, Float>(dst, gaussianKernel(sigma), dst);
            break;
        case DerivType::RC:
            convolveRow<Tp, Float>(src, gaussianDerivedKernel(sigma), dst);
            convolveCol<Float, Float>(dst, gaussianDerivedKernel(sigma), dst);
            break;
        case DerivType::CC:
            convolveRow<Tp, Float>(src, gaussianKernel(sigma), dst);
            convolveCol<Float, Float>(
                dst, gaussianDerived2ndKernel(sigma), dst);
            break;
        case DerivType::R:
            convolveRow<Tp, Float>(src, gaussianDerivedKernel(sigma), dst);
            convolveCol<Float, Float>(dst, gaussianKernel(sigma), dst);
        default:
            break;
    }
}

#ifdef RULERMVS_USE_SSE

extern "C" MVS_EXPORT void convolve_col_sse(int w, int h, const float* in,
    int in_stride, float* out, int out_stride, const float* kernel,
    int kernel_len);

extern "C" MVS_EXPORT void convolve_row_sse(int w, int h, const uchar* in,
    int in_stride, float* out, int out_stride, const float* kernel,
    int kernel_len);

template <> struct Convolver_<uchar, float, DerivType::R> {
    static inline void convolve(
        const Image8u& src, const FloatVec& kernel, Imagef& dst)
    {
        if (kernel.empty() || src.empty()) return;
        dst.create(src.size());
        convolve_row_sse(src.width, src.height, src.ptr(0), src.stride,
            dst.ptr(0), dst.stride >> 2, kernel.data(), (int)kernel.size());
    }
};
template <> struct Convolver_<float, float, DerivType::C> {
    static inline void convolve(
        const Imagef& src, const FloatVec& kernel, Imagef& dst)
    {
        if (kernel.empty() || src.empty()) return;
        dst.create(src.size());
        convolve_col_sse(src.width, src.height, src.ptr(0), src.stride >> 2,
            dst.ptr(0), dst.stride >> 2, kernel.data(), (int)kernel.size());
    }
};
#endif

// template <typename Tp> struct ImageIter {
//     static inline void proc() {}
// };
// // void iterProc(Image_<Tp>& out, std::function<void(const Tp& s, Tp& d)>
// func)
// // {}

// template <typename Tp>
// static inline Image8u operator>(const Image_<Tp>& img, Tp val)
// {
//     return Image8u();
// }

// 读取图像的函数接口
MVS_EXPORT bool readImage(ConstStr&, Image8u&);
MVS_EXPORT bool readImage(ConstStr&, Image16u&);
MVS_EXPORT bool readImage(ConstStr&, RGBImage&);
MVS_EXPORT bool readImage(ConstStr&, RGBAImage&);

// // 保存图像的函数接口
MVS_EXPORT bool writeImage(ConstStr&, const Image8u&);
MVS_EXPORT bool writeImage(ConstStr&, const Image16u&);
MVS_EXPORT bool writeImage(ConstStr&, const RGBImage&);
MVS_EXPORT bool writeImage(ConstStr&, const RGBAImage&);

/// @brief 判断是否存在对应的图像加载函数
template <typename T> struct has_image_reader {
    typedef long No;
    typedef char Yes;
    template <typename Tp> struct helper {
        typedef bool (*func_ptr)(ConstStr&, Image_<T>&);
    };
    template <typename Tp, Tp> struct TypeCheck;
    template <typename Tp> static Yes has_reader(
        TypeCheck<typename helper<Tp>::func_ptr, &readImage>*);
    template <typename Tp> static No has_reader(...);
    enum { value = (sizeof(has_reader<T>(0)) == sizeof(Yes)) };
};

/// @brief 判断是否存在对应的图像保存函数
template <typename T> struct has_image_writer {
    typedef long No;
    typedef char Yes;
    template <typename Tp> struct helper {
        typedef bool (*func_ptr)(ConstStr&, const Image_<T>&);
    };
    template <typename Tp, Tp> struct TypeCheck;
    template <typename Tp> static Yes has_writer(
        TypeCheck<typename helper<Tp>::func_ptr, &writeImage>*);
    template <typename Tp> static No has_writer(...);
    enum { value = (sizeof(has_writer<T>(0)) == sizeof(Yes)) };
};

/// @brief 文件读取
template <typename Tp, typename = void> struct ImageReader_ {
    static inline bool read(ConstStr&, Image_<Tp>&) { return false; }
};
template <typename Tp, typename = void> struct ImageWriter_ {
    static inline bool write(ConstStr&, const Image_<Tp>&) { return false; }
};
template <typename T> bool Image_<T>::load(ConstStr& path)
{
    return ImageReader_<T>::read(path, *this);
}
template <typename T> bool Image_<T>::save(ConstStr& path) const
{
    return ImageWriter_<T>::write(path, *this);
}
template <typename T> struct ImageReader_<T,
    typename std::enable_if<has_image_reader<T>::value>::type> {
    static inline bool read(ConstStr& path, Image_<T>& image)
    {
        return readImage(path, image);
    }
};
template <typename T> struct ImageWriter_<T,
    typename std::enable_if<has_image_writer<T>::value>::type> {
    static inline bool write(ConstStr& path, const Image_<T>& image)
    {
        return writeImage(path, image);
    }
};

}  // namespace rulermvs

#include "rulermvs/image_impl.hpp"

#endif  // _RULERMVS_CORE_IMAGE_HPP_
