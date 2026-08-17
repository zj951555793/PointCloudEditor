#ifndef _RULERMVS_CORE_PIXEL_HPP_
#define _RULERMVS_CORE_PIXEL_HPP_
#include "rulermvs/core.hpp"
namespace rulermvs
{
struct RGBAPixel;
/**@brief 彩色点云像素的简单结构体。
 * @note 注意此处的像素排列顺序 */
struct RGBPixel {
    RGBPixel() = default;
    RGBPixel(uchar c) : r(c), g(c), b(c) {}
    RGBPixel(const RGBPixel& p) = default;
    template <typename Tp,
        class = typename std::enable_if<sizeof(Tp) != 1>::type>
    explicit inline RGBPixel(Tp c) : RGBPixel((uchar)clamp<Tp>(c, 0, 255))
    {}
    RGBPixel(uchar r, uchar g, uchar b) : r(r), g(g), b(b) {}
    template <typename Tp,
        class = typename std::enable_if<sizeof(Tp) != 1>::type>
    explicit inline RGBPixel(Tp r, Tp g, Tp b)
        : r((uchar)clamp<Tp>(r, 0, 255))
        , g((uchar)clamp<Tp>(g, 0, 255))
        , b((uchar)clamp<Tp>(b, 0, 255))
    {}
    inline RGBPixel(const RGBAPixel& p);
    friend inline std::ostream& operator<<(std::ostream& O, const RGBPixel& pix)
    {
        return O << "RGBPixel(" << (int)pix.r << "," << (int)pix.g << ","
                 << (int)pix.b << ")";
    };

    uchar r;  ///< red
    uchar g;  ///< green
    uchar b;  ///< blue
};
using RGBPixelVec = std::vector<RGBPixel>;
template <> struct CoordTraits<RGBPixel> {
    typedef uchar type;
};
static inline RGBPixel operator+(const RGBPixel& p1, const RGBPixel& p2)
{
    return RGBPixel {p1.r + p2.r, p1.g + p2.g, p1.b + p2.b};
}
static inline RGBPixel operator-(const RGBPixel& p1, const RGBPixel& p2)
{
    return RGBPixel {p1.r - p2.r, p1.g - p2.g, p1.b - p2.b};
}
static inline RGBPixel operator*(const RGBPixel& val, float s)
{
    return RGBPixel {val.r * s, val.g * s, val.b * s};
}
static inline RGBPixel operator/(const RGBPixel& val, float s)
{
    return RGBPixel {val.r / s, val.g / s, val.b / s};
}
static inline RGBPixel operator*(const RGBPixel& val, double s)
{
    return RGBPixel {val.r * s, val.g * s, val.b * s};
}
static inline RGBPixel operator/(const RGBPixel& val, double s)
{
    return RGBPixel {val.r / s, val.g / s, val.b / s};
}

/// @brief RGBA带透明通道像素结构体
struct RGBAPixel {
    // RGBAPixel() : r(0), g(0), b(0), a(255) {}
    RGBAPixel() = default;
    RGBAPixel(uchar c) : r(c), g(c), b(c), a(255) {}
    RGBAPixel(const RGBAPixel& p) = default;
    template <typename Tp,
        class = typename std::enable_if<sizeof(Tp) != 1>::type>
    explicit inline RGBAPixel(Tp c) : RGBAPixel((uchar)clamp<Tp>(c, 0, 255))
    {}
    RGBAPixel(const RGBPixel& p) : r(p.r), g(p.g), b(p.b), a(255) {}
    RGBAPixel(uchar r, uchar g, uchar b, uchar a = 255) : r(r), g(g), b(b), a(a)
    {}
    template <typename Tp,
        class = typename std::enable_if<sizeof(Tp) != 1>::type>
    explicit inline RGBAPixel(Tp r, Tp g, Tp b, Tp a = 255)
        : r((uchar)clamp<Tp>(r, 0, 255))
        , g((uchar)clamp<Tp>(g, 0, 255))
        , b((uchar)clamp<Tp>(b, 0, 255))
        , a((uchar)clamp<Tp>(a, 0, 255))
    {}
    friend inline std::ostream& operator<<(
        std::ostream& O, const RGBAPixel& pix)
    {
        return O << "RGBPixel(" << (int)pix.r << "," << (int)pix.g << ","
                 << (int)pix.b << "," << (int)pix.a << ")";
    };

    uchar r;  ///< red
    uchar g;  ///< green
    uchar b;  ///< blue
    uchar a;  ///< alpha
};
template <> struct CoordTraits<RGBAPixel> {
    typedef uchar type;
};
RGBPixel::RGBPixel(const RGBAPixel& pix)
{
    auto a = pix.a / 255.0f;
    *this  = RGBPixel {a * pix.r, a * pix.g, a * pix.b};
}

/// @brief 将RGB像素转到整形.
/// @tparam Tp 整形
template <typename Tp> struct Converter_<RGBPixel, Tp,
    typename std::enable_if<std::is_arithmetic<Tp>::value>::type> {
    static inline void to(const RGBPixel& src, Tp& dst)
    {
        dst = (Tp)clamp<float>(
            0.299f * src.r + 0.587f * src.g + 0.114f * src.b, 0.f, 255.f);
    }
};
// /// @brief 将RGB像素转到浮点类型时进行归一化.
// /// @tparam Tp 浮点类型
// template <typename Tp> struct Converter_<RGBPixel, Tp,
//     std::enable_if_t<std::is_floating_point<Tp>::value>> {
//     static inline void to(const RGBPixel& src, Tp& dst)
//     {
//         dst = (Tp)clamp<float>(
//             (0.299f * src.r + 0.587f * src.g + 0.114f * src.b) / 255.f, 0.f,
//             1.f);
//     }
// };
template <typename Tp> struct Converter_<Tp, RGBPixel,
    typename std::enable_if<!std::is_same<RGBPixel, Tp>::value>::type> {
    static inline void to(const Tp& src, RGBPixel& dst) { dst = RGBPixel(src); }
};
template <typename Tp> struct Converter_<RGBAPixel, Tp> {
    static inline void to(const RGBAPixel& pix, Tp& val)
    {
        auto v = 0.299f * pix.r + 0.587f * pix.g + 0.114f * pix.b;
        val    = (Tp)clamp<float>((pix.a / 255.f) * v, 0.f, 255.f);
    }
};
template <typename Tp> struct Converter_<Tp, RGBAPixel> {
    static inline void to(const Tp& src, RGBAPixel& dst) { dst = src; }
};
template <> struct Converter_<RGBAPixel, RGBPixel> {
    static inline void to(const RGBAPixel& src, RGBPixel& dst) { dst = src; }
};

/// @brief 像素类型
enum class PixelType {
    RGB,
    RGBA,
    INT8,
    INT16,
    INT32,
    UINT8,
    UINT16,
    UINT32,
    FLOAT32,
    FLOAT64,
    UNKNOWN
};
/// @brief 像素类型查询
/// @tparam pixType
template <PixelType pixType = PixelType::UNKNOWN> struct PixelTraits {
    typedef void type;
};
template <> struct PixelTraits<PixelType::RGB> {
    typedef RGBPixel type;
};
template <> struct PixelTraits<PixelType::RGBA> {
    typedef RGBAPixel type;
};
template <> struct PixelTraits<PixelType::INT8> {
    typedef char type;
};
template <> struct PixelTraits<PixelType::INT16> {
    typedef short type;
};
template <> struct PixelTraits<PixelType::INT32> {
    typedef int type;
};
template <> struct PixelTraits<PixelType::UINT8> {
    typedef uchar type;
};
template <> struct PixelTraits<PixelType::UINT16> {
    typedef ushort type;
};
template <> struct PixelTraits<PixelType::UINT32> {
    typedef unsigned int type;
};
template <> struct PixelTraits<PixelType::FLOAT32> {
    typedef float type;
};
template <> struct PixelTraits<PixelType::FLOAT64> {
    typedef double type;
};
template <PixelType pixType> using pixel_traits_t =
    typename PixelTraits<pixType>::type;

// clang-format off
template <typename T> inline PixelType cvtPixelType() { return PixelType::UNKNOWN; }
template <> inline PixelType cvtPixelType<int>() { return PixelType::INT32; }
template <> inline PixelType cvtPixelType<char>() { return PixelType::INT8; }
template <> inline PixelType cvtPixelType<short>() { return PixelType::INT16; }
template <> inline PixelType cvtPixelType<uint>() { return PixelType::UINT32; }
template <> inline PixelType cvtPixelType<uchar>() { return PixelType::UINT8; }
template <> inline PixelType cvtPixelType<ushort>() { return PixelType::UINT16; }
template <> inline PixelType cvtPixelType<float>() { return PixelType::FLOAT32; }
template <> inline PixelType cvtPixelType<double>() { return PixelType::FLOAT64; }
template <> inline PixelType cvtPixelType<RGBPixel>() { return PixelType::RGB; }
template <> inline PixelType cvtPixelType<RGBAPixel>() { return PixelType::RGBA; }
// clang-format on

/// @brief 像素转换
/// @tparam Tp 目标像素
/// @tparam pixType 输入数据的像素类型
template <typename T1, typename T2, typename = void> struct CvtPixel_ {
    static inline void convert(const T1* src, T2* dst, size_t sz)
    {
        assert(src && sz && dst);
        for (size_t i = 0; i < sz; ++i) dst[i] = (T2)src[i];
    }
};
template <typename Tp> struct CvtPixel_<Tp, Tp> {
    static inline void convert(const Tp* src, Tp* dst, size_t sz)
    {
        assert(src && sz && dst);
        if (sz && src != dst) memcpy(dst, src, sz * sizeof(Tp));
    }
};
template <typename Tp> struct CvtPixel_<RGBPixel, Tp,
    typename std::enable_if<std::is_arithmetic<Tp>::value>::type> {
    static inline void convert(const RGBPixel* src, Tp* dst, size_t sz)
    {
        assert(src && sz && dst);
        for (size_t i = 0; i < sz; ++i) {
            auto& pix = src[i];
            dst[i]    = (Tp)clamp<float>(
                0.299f * pix.r + 0.587f * pix.g + 0.114f * pix.b, 0.f, 255.f);
        }
    }
};
template <typename Tp> struct CvtPixel_<RGBAPixel, Tp,
    typename std::enable_if<std::is_arithmetic<Tp>::value>::type> {
    static inline void convert(const RGBAPixel* src, Tp* dst, size_t sz)
    {
        assert(src && sz && dst);
        for (size_t i = 0; i < sz; ++i) {
            auto& pix = src[i];
            auto  val = 0.299f * pix.r + 0.587f * pix.g + 0.114f * pix.b;
            dst[i]    = (Tp)clamp<float>((pix.a / 255.f) * val, 0.f, 255.f);
        }
    }
};
template <typename Tp> struct CvtPixel_<Tp, RGBPixel,
    typename std::enable_if<std::is_arithmetic<Tp>::value ||
                            std::is_same<Tp, RGBAPixel>::value>::type> {
    static inline void convert(const Tp* src, RGBPixel* dst, size_t sz)
    {
        assert(src && sz && dst);
        for (size_t i = 0; i < sz; ++i) dst[i] = RGBPixel(src[i]);
    }
};
template <typename Tp> struct CvtPixel_<Tp, RGBAPixel,
    typename std::enable_if<std::is_arithmetic<Tp>::value ||
                            std::is_same<Tp, RGBPixel>::value>::type> {
    static inline void convert(const Tp* src, RGBAPixel* dst, size_t sz)
    {
        assert(src && sz && dst);
        for (size_t i = 0; i < sz; ++i) dst[i] = RGBAPixel(src[i]);
    }
};

template <typename T1, typename T2>
void cvtPixel(const T1* src, T2* dst, size_t sz)
{
    assert(src && sz && dst);
    CvtPixel_<T1, T2>::convert(src, dst, sz);
}

template <typename Tp> static inline bool convertPixelTo(
    const void* src, size_t sz, PixelType pixType, Tp* dst)
{
    if (!src || !sz || !dst) return false;
    switch (pixType) {
        case PixelType::RGB:
            cvtPixel(reinterpret_cast<const RGBPixel*>(src), dst, sz);
            break;
        case PixelType::RGBA:
            cvtPixel(reinterpret_cast<const RGBAPixel*>(src), dst, sz);
            break;
        case PixelType::INT8:
            cvtPixel(reinterpret_cast<const char*>(src), dst, sz);
            break;
        case PixelType::INT16:
            cvtPixel(reinterpret_cast<const short*>(src), dst, sz);
            break;
        case PixelType::INT32:
            cvtPixel(reinterpret_cast<const int*>(src), dst, sz);
            break;
        case PixelType::UINT8:
            cvtPixel(reinterpret_cast<const uchar*>(src), dst, sz);
            break;
        case PixelType::UINT16:
            cvtPixel(reinterpret_cast<const ushort*>(src), dst, sz);
            break;
        case PixelType::UINT32:
            cvtPixel(reinterpret_cast<const uint*>(src), dst, sz);
            break;
        case PixelType::FLOAT32:
            cvtPixel(reinterpret_cast<const float*>(src), dst, sz);
            break;
        case PixelType::FLOAT64:
            cvtPixel(reinterpret_cast<const double*>(src), dst, sz);
            break;
        case PixelType::UNKNOWN:
        default:
            return false;
    }
    return true;
}

template <typename Tp> static inline std::vector<Tp> convertPixelTo(
    const void* src, size_t sz, PixelType pixType)
{
    std::vector<Tp> dst;
    if (src && sz && pixType != PixelType::UNKNOWN) {
        dst.resize(sz);
        if (!convertPixelTo<Tp>(src, sz, pixType, dst.data())) dst.clear();
    }
    return dst;
}

#if defined(RULERMVS_USE_SSE)
template <> struct CvtPixel_<RGBPixel, uchar> {
    static inline void convert(const RGBPixel* src, uchar* dst, size_t sz)
    {
        assert(src && dst && sz);
        if (!checkHardwareSupport(SIMDMode::MVS_SSE3)) {
            for (size_t i = 0; i < sz; ++i) {
                auto& pix = src[i];
                dst[i] =
                    (uchar)(0.299f * pix.r + 0.587f * pix.g + 0.114f * pix.b);
            }
            return;
        }
        uchar*       out_ptr = dst;
        const uchar* in_ptr  = reinterpret_cast<const uchar*>(src);
        const int    nLoop   = ((int)sz - 4) / 12;

        const static short BW = (short)std::ceil(0.114 * 256);
        const static short GW = (short)std::ceil(0.587 * 256);
        const static short RW = 256 - BW - GW;
        for (int i = 0; i < nLoop; ++i, in_ptr += 36, out_ptr += 12) {
            __m128i r1 = _mm_mullo_epi16(
                _mm_cvtepu8_epi16(_mm_loadu_si128((const __m128i*)&in_ptr[0])),
                _mm_setr_epi16(RW, GW, BW, RW, GW, BW, RW, GW));
            __m128i g1 = _mm_mullo_epi16(
                _mm_cvtepu8_epi16(_mm_loadu_si128((const __m128i*)&in_ptr[1])),
                _mm_setr_epi16(GW, BW, RW, GW, BW, RW, GW, BW));
            __m128i b1 = _mm_mullo_epi16(
                _mm_cvtepu8_epi16(_mm_loadu_si128((const __m128i*)&in_ptr[2])),
                _mm_setr_epi16(BW, RW, GW, BW, RW, GW, BW, RW));
            __m128i res1 = _mm_shuffle_epi8(
                _mm_srli_epi16(_mm_add_epi16(r1, _mm_add_epi16(g1, b1)), 8),
                _mm_setr_epi8(0, 6, 12, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                    -1, -1, -1));
            __m128i r2 = _mm_mullo_epi16(
                _mm_cvtepu8_epi16(_mm_loadu_si128((const __m128i*)&in_ptr[9])),
                _mm_setr_epi16(RW, GW, BW, RW, GW, BW, RW, GW));
            __m128i g2 = _mm_mullo_epi16(
                _mm_cvtepu8_epi16(_mm_loadu_si128((const __m128i*)&in_ptr[10])),
                _mm_setr_epi16(GW, BW, RW, GW, BW, RW, GW, BW));
            __m128i b2 = _mm_mullo_epi16(
                _mm_cvtepu8_epi16(_mm_loadu_si128((const __m128i*)&in_ptr[11])),
                _mm_setr_epi16(BW, RW, GW, BW, RW, GW, BW, RW));
            __m128i res2 = _mm_shuffle_epi8(
                _mm_srli_epi16(_mm_add_epi16(r2, _mm_add_epi16(g2, b2)), 8),
                _mm_setr_epi8(-1, -1, -1, 0, 6, 12, -1, -1, -1, -1, -1, -1, -1,
                    -1, -1, -1));
            __m128i r3 = _mm_mullo_epi16(
                _mm_cvtepu8_epi16(_mm_loadu_si128((const __m128i*)&in_ptr[18])),
                _mm_setr_epi16(RW, GW, BW, RW, GW, BW, RW, GW));
            __m128i g3 = _mm_mullo_epi16(
                _mm_cvtepu8_epi16(_mm_loadu_si128((const __m128i*)&in_ptr[19])),
                _mm_setr_epi16(GW, BW, RW, GW, BW, RW, GW, BW));
            __m128i b3 = _mm_mullo_epi16(
                _mm_cvtepu8_epi16(_mm_loadu_si128((const __m128i*)&in_ptr[20])),
                _mm_setr_epi16(BW, RW, GW, BW, RW, GW, BW, RW));
            __m128i res3 = _mm_shuffle_epi8(
                _mm_srli_epi16(_mm_add_epi16(r3, _mm_add_epi16(g3, b3)), 8),
                _mm_setr_epi8(-1, -1, -1, -1, -1, -1, 0, 6, 12, -1, -1, -1, -1,
                    -1, -1, -1));
            __m128i r4 = _mm_mullo_epi16(
                _mm_cvtepu8_epi16(_mm_loadu_si128((const __m128i*)&in_ptr[26])),
                _mm_setr_epi16(BW, RW, GW, BW, RW, GW, BW, RW));
            __m128i g4 = _mm_mullo_epi16(
                _mm_cvtepu8_epi16(_mm_loadu_si128((const __m128i*)&in_ptr[27])),
                _mm_setr_epi16(RW, GW, BW, RW, GW, BW, RW, GW));
            __m128i b4 = _mm_mullo_epi16(
                _mm_cvtepu8_epi16(_mm_loadu_si128((const __m128i*)&in_ptr[28])),
                _mm_setr_epi16(GW, BW, RW, GW, BW, RW, GW, BW));
            __m128i res4 = _mm_shuffle_epi8(
                _mm_srli_epi16(_mm_add_epi16(r4, _mm_add_epi16(g4, b4)), 8),
                _mm_setr_epi8(-1, -1, -1, -1, -1, -1, -1, -1, -1, 18, 24, 30,
                    -1, -1, -1, -1));
            __m128i res = _mm_or_si128(
                _mm_or_si128(res1, res2), _mm_or_si128(res3, res4));
            _mm_storeu_si128((__m128i*)out_ptr, res);
        }
        for (int i = nLoop * 12; i < (int)sz; ++i) {
            auto& pix = src[i];
            dst[i] = (uchar)(0.299f * pix.r + 0.587f * pix.g + 0.114f * pix.b);
        }
    }
};
#endif

}  // namespace rulermvs
#endif