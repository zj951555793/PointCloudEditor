#ifndef _RULERMVS_CORE_IMAGE_IMPL_HPP_
#define _RULERMVS_CORE_IMAGE_IMPL_HPP_
// #include <type_traits>
namespace rulermvs
{
/// @brief 图像操作类型
enum class ImageOpType : int {
    Log,
    Sqrt,
    Ceil,
    Plus,
    Minus,
    Floor,
    Divide,
    Multiply,
    Absolute
};

template <typename T, ImageOpType opType, typename = void> struct ImageOp_ {
    template <typename... Args> static inline void proc(Args&&... args)
    {
        // static_assert(false, "Function Not Implemented.");
    }
};
// template <typename T> struct ImageOp_<T, ImageOpType::Max> {
//     template <int N>
//     static inline void proc(const Image_<T> imgs[N], Image_<T>& dst)
//     {
//         proc<N - 1>(imgs, dst);
//         auto& src = imgs[N - 1];
//         if (!src.empty() && dst.size() == src.size()) {
//             for (int i = 0; i < src.height; ++i) {
//                 auto dst_ptr = dst.ptr(i);
//                 auto src_ptr = src.ptr(i);
//                 for (int j = 0; j < src.width; ++j)
//                     dst_ptr[j] = std::max(src_ptr[j], dst_ptr[j]);
//             }
//         }
//     }
//     template <>
//     static inline void proc<1>(const Image_<T> imgs[1], Image_<T>& dst)
//     {
//         dst = imgs[0].clone();
//     }
// };
// template <int N, typename T>
// inline void maxImages(const Image_<T> imgs[N], Image_<T>& dst)
// {
//     ImageOp_<T, ImageOpType::Max>::proc<N>(imgs, dst);
// }
// template <typename T> struct ImageOp_<T, ImageOpType::Min> {
//     template <int N>
//     static inline void proc(const Image_<T> imgs[N], Image_<T>& dst)
//     {
//         proc<N - 1>(imgs, dst);
//         auto& src = imgs[N - 1];
//         if (!src.empty() && dst.size() == src.size()) {
//             for (int i = 0; i < src.height; ++i) {
//                 auto dst_ptr = dst.ptr(i);
//                 auto src_ptr = src.ptr(i);
//                 for (int j = 0; j < src.width; ++j)
//                     dst_ptr[j] = std::min(src_ptr[j], dst_ptr[j]);
//             }
//         }
//     }
//     template <>
//     static inline void proc<1>(const Image_<T> imgs[1], Image_<T>& dst)
//     {
//         dst = imgs[0].clone();
//     }
// };
// template <int N, typename T>
// inline void minImages(const Image_<T> imgs[N], Image_<T>& dst)
// {
//     ImageOp_<T, ImageOpType::Min>::proc<N>(imgs, dst);
// }

template <typename ImageT, typename Tp, ImageOpType opType>
struct ImageBinaryOpImpl {};
template <typename Tp>
struct ImageBinaryOpImpl<Image_<Tp>, Tp, ImageOpType::Plus> {
    static inline void proc(
        const Image_<Tp>& _src, Tp _scalar, Image_<Tp>& _dst)
    {
        if (_src.empty()) return;
        _dst.create(_src.size());
        for (int i = 0; i < _src.height; ++i) {
            Tp *srcptr = _src.ptr(i), *dstptr = _dst.ptr(i);
            for (int j = 0; j < _src.width; ++j)
                dstptr[j] = srcptr[j] + _scalar;
        }
    }
};
template <typename Tp>
struct ImageBinaryOpImpl<Tp, Image_<Tp>, ImageOpType::Plus> {
    static inline void proc(
        Tp _scalar, const Image_<Tp>& _src, Image_<Tp>& _dst)
    {
        ImageBinaryOpImpl<Image_<Tp>, Tp, ImageOpType::Plus>::proc(
            _src, _scalar, _dst);
    }
};
template <typename Tp>
struct ImageBinaryOpImpl<Image_<Tp>, Image_<Tp>, ImageOpType::Plus> {
    static inline void proc(
        const Image_<Tp>& _srcA, const Image_<Tp>& _srcB, Image_<Tp>& _dst)
    {
        if (_srcA.size() != _srcB.size()) return;
        _dst.create(_srcA.size());
        for (int i = 0; i < _srcA.height; ++i) {
            Tp *ptrA = _srcA.ptr(i), *ptrB = _srcB.ptr(i),
               *dstptr = _dst.ptr(i);
            for (int j = 0; j < _srcA.width; ++j) dstptr[j] = ptrA[j] + ptrB[j];
        }
    }
};
template <typename Tp>
struct ImageBinaryOpImpl<Image_<Tp>, Tp, ImageOpType::Minus> {
    static inline void proc(
        const Image_<Tp>& _src, Tp _scalar, Image_<Tp>& _dst)
    {
        if (_src.empty()) return;
        _dst.create(_src.size());
        for (int i = 0; i < _src.height; ++i) {
            Tp *srcptr = _src.ptr(i), *dstptr = _dst.ptr(i);
            for (int j = 0; j < _src.width; ++j)
                dstptr[j] = srcptr[j] - _scalar;
        }
    }
};
template <typename Tp>
struct ImageBinaryOpImpl<Image_<Tp>, Image_<Tp>, ImageOpType::Minus> {
    static inline void proc(
        const Image_<Tp>& _srcA, const Image_<Tp>& _srcB, Image_<Tp>& _dst)
    {
        if (_srcA.size() != _srcB.size()) return;
        _dst.create(_srcA.size());
        for (int i = 0; i < _srcA.height; ++i) {
            Tp *ptrA = _srcA.ptr(i), *ptrB = _srcB.ptr(i),
               *dstptr = _dst.ptr(i);
            for (int j = 0; j < _srcA.width; ++j) dstptr[j] = ptrA[j] - ptrB[j];
        }
    }
};
template <typename Tp>
struct ImageBinaryOpImpl<Image_<Tp>, Tp, ImageOpType::Multiply> {
    static inline void proc(
        const Image_<Tp>& _src, Tp _scalar, Image_<Tp>& _dst)
    {
        if (_src.empty()) return;
        _dst.create(_src.size());
        for (int i = 0; i < _src.height; ++i) {
            Tp *srcptr = _src.ptr(i), *dstptr = _dst.ptr(i);
            for (int j = 0; j < _src.width; ++j)
                dstptr[j] = srcptr[j] * _scalar;
        }
    }
};
template <typename Tp>
struct ImageBinaryOpImpl<Image_<Tp>, Image_<Tp>, ImageOpType::Multiply> {
    static inline void proc(
        const Image_<Tp>& _A, const Image_<Tp>& _B, Image_<Tp>& _C)
    {
        if (_A.size() != _B.size()) return;
        _C.create(_A.size());
        for (int i = 0; i < _A.height; ++i) {
            Tp *ptrA = _A.ptr(i), *ptrB = _B.ptr(i), *ptrC = _C.ptr(i);
            for (int j = 0; j < _A.width; ++j) ptrC[j] = ptrA[j] * ptrB[j];
        }
    }
};
template <typename Tp>
struct ImageBinaryOpImpl<Tp, Image_<Tp>, ImageOpType::Divide> {
    static inline void proc(
        Tp _scalar, const Image_<Tp>& _src, Image_<Tp>& _dst)
    {
        if (_src.empty()) return;
        _dst.create(_src.size());
        for (int i = 0; i < _src.height; ++i) {
            Tp *srcptr = _src.ptr(i), *dstptr = _dst.ptr(i);
            for (int j = 0; j < _src.width; ++j)
                dstptr[j] = _scalar / srcptr[j];
        }
    }
};
template <typename Tp>
struct ImageBinaryOpImpl<Image_<Tp>, Tp, ImageOpType::Divide> {
    static inline void proc(
        const Image_<float>& _src, Tp _scalar, Image_<float>& _dst)
    {
        if (_src.empty()) return;
        _dst.create(_src.size());
        for (int i = 0; i < _src.height; ++i) {
            Tp *srcptr = _src.ptr(i), *dstptr = _dst.ptr(i);
            for (int j = 0; j < _src.width; ++j)
                dstptr[j] = srcptr[j] / _scalar;
        }
    }
};
template <typename Tp>
struct ImageBinaryOpImpl<Image_<Tp>, Image_<Tp>, ImageOpType::Divide> {
    static inline void proc(
        const Image_<Tp>& _srcA, const Image_<Tp>& _srcB, Image_<Tp>& _dst)
    {
        if (_srcA.size() != _srcB.size()) return;
        _dst.create(_srcA.size());
        for (int i = 0; i < _srcA.height; ++i) {
            Tp *ptrA = _srcA.ptr(i), *ptrB = _srcB.ptr(i),
               *dstptr = _dst.ptr(i);
            for (int j = 0; j < _srcA.width; ++j) dstptr[j] = ptrA[j] / ptrB[j];
        }
    }
};
template <typename Tp, ImageOpType opType> struct ImageUnaryOpImpl {};
template <typename Tp> struct ImageUnaryOpImpl<Tp, ImageOpType::Minus> {
    static inline void proc(const Image_<Tp>& _src, Image_<Tp>& _dst)
    {
        if (_src.empty()) return;
        _dst.create(_src.size());
        for (int i = 0; i < _src.height; ++i) {
            Tp *srcptr = _src.ptr(i), *dstptr = _dst.ptr(i);
            for (int j = 0; j < _src.width; ++j) dstptr[j] = -srcptr[j];
        }
    }
};
template <typename Tp> struct ImageUnaryOpImpl<Tp, ImageOpType::Sqrt> {
    static inline void proc(const Image_<Tp>& _src, Image_<Tp>& _dst)
    {
        if (_src.empty()) return;
        _dst.create(_src.size());
        for (int i = 0; i < _src.height; ++i) {
            Tp *srcptr = _src.ptr(i), *dstptr = _dst.ptr(i);
            for (int j = 0; j < _src.width; ++j)
                dstptr[j] = std::sqrt(srcptr[j]);
        }
    }
};
template <typename Tp> struct ImageUnaryOpImpl<Tp, ImageOpType::Absolute> {
    static inline void proc(const Image_<Tp>& _src, Image_<Tp>& _dst)
    {
        if (_src.empty()) return;
        _dst.create(_src.size());
        for (int i = 0; i < _src.height; ++i) {
            Tp *srcptr = _src.ptr(i), *dstptr = _dst.ptr(i);
            for (int j = 0; j < _src.width; ++j)
                dstptr[j] = std::abs(srcptr[j]);
        }
    }
};

template <class _UnaryOp, ImageOpType opType> struct ImageUnaryOp_ {
    typedef typename _UnaryOp::DataType DataType;
    ImageUnaryOp_() = delete;
    ImageUnaryOp_(const ImageUnaryOp_& op) : input_(op.input_) {}
    ImageUnaryOp_(const _UnaryOp& input) : input_(&input) {}
    void operator>>(Image_<DataType>& _output) const
    {
        *input_ >> _output;
        ImageUnaryOpImpl<DataType, opType>::proc(_output, _output);
    }

    const _UnaryOp* input_;
};
template <typename Tp, ImageOpType opType>
struct ImageUnaryOp_<Image_<Tp>, opType> {
    typedef Tp DataType;
    ImageUnaryOp_() = delete;
    ImageUnaryOp_(const ImageUnaryOp_& op) : input_(op.input_) {}
    ImageUnaryOp_(const Image_<Tp>& input) : input_(&input) {}
    void operator>>(Image_<DataType>& output) const
    {
        ImageUnaryOpImpl<Tp, opType>::proc(*input_, output);
    }

    const Image_<Tp>* input_;
};
template <typename Tp>
static inline ImageUnaryOp_<Image_<Tp>, ImageOpType::Minus> operator-(
    const Image_<Tp>& _src)
{
    return _src;
}
template <typename _UnaryOp, ImageOpType opType>
static inline ImageUnaryOp_<ImageUnaryOp_<_UnaryOp, opType>, ImageOpType::Minus>
operator-(const ImageUnaryOp_<_UnaryOp, opType>& op)
{
    return ImageUnaryOp_<ImageUnaryOp_<_UnaryOp, opType>, ImageOpType::Minus>(
        op);
}
template <typename Tp>
static inline ImageUnaryOp_<Image_<Tp>, ImageOpType::Sqrt> Sqrt(
    const Image_<Tp>& src)
{
    return ImageUnaryOp_<Image_<Tp>, ImageOpType::Sqrt>(src);
}
template <typename _UnaryOp, ImageOpType opType>
static inline ImageUnaryOp_<ImageUnaryOp_<_UnaryOp, opType>, ImageOpType::Sqrt>
Sqrt(const ImageUnaryOp_<_UnaryOp, opType>& op)
{
    return ImageUnaryOp_<ImageUnaryOp_<_UnaryOp, opType>, ImageOpType::Sqrt>(
        op);
}
template <typename Tp>
static inline ImageUnaryOp_<Image_<Tp>, ImageOpType::Absolute> Abs(
    const Image_<Tp>& _src)
{
    return ImageUnaryOp_<Image_<Tp>, ImageOpType::Absolute>(_src);
}
template <typename _UnaryOp, ImageOpType opType>
static inline ImageUnaryOp_<ImageUnaryOp_<_UnaryOp, opType>,
    ImageOpType::Absolute>
Abs(const ImageUnaryOp_<_UnaryOp, opType>& _op)
{
    return ImageUnaryOp_<ImageUnaryOp_<_UnaryOp, opType>,
        ImageOpType::Absolute>(_op);
}
template <typename _BinaryOp, typename _UnaryOp, ImageOpType opType>
struct ImageBinaryOp_ {
    typedef typename _BinaryOp::DataType DataType;
    ImageBinaryOp_() = delete;
    ImageBinaryOp_(const ImageBinaryOp_& _op) : op1_(_op.op1_), op2_(_op.op2_)
    {}
    ImageBinaryOp_(const _BinaryOp& _op1, const _UnaryOp& _op2)
        : op1_(&_op1), op2_(&_op2)
    {}
    void operator>>(Image_<DataType>& _output) const
    {
        _output               = *op1_;
        Image_<DataType> temp = *op2_;
        ImageBinaryOpImpl<Image_<DataType>, Image_<DataType>, opType>::proc(
            _output, temp, _output);
    }

    const _BinaryOp* op1_;
    const _UnaryOp*  op2_;
};
template <typename _BinaryOp, ImageOpType opType>
struct ImageBinaryOp_<_BinaryOp, typename _BinaryOp::DataType, opType> {
    typedef typename _BinaryOp::DataType DataType;
    ImageBinaryOp_() = delete;
    ImageBinaryOp_(const ImageBinaryOp_& _op) : op_(_op.op_), input_(_op.input_)
    {}
    ImageBinaryOp_(const _BinaryOp& _op, DataType _scalar)
        : op_(&_op), input_(_scalar)
    {}
    void operator>>(Image_<DataType>& _output) const
    {
        *op_ >> _output;
        ImageBinaryOpImpl<Image_<DataType>, DataType, opType>::proc(
            _output, input_, _output);
    }

    const _BinaryOp* op_;
    const DataType   input_;
};
template <typename _BinaryOp, ImageOpType opType>
struct ImageBinaryOp_<typename _BinaryOp::DataType, _BinaryOp, opType> {
    typedef typename _BinaryOp::DataType DataType;
    ImageBinaryOp_() = delete;
    ImageBinaryOp_(const ImageBinaryOp_& _op) : op_(_op.op_), input_(_op.input_)
    {}
    ImageBinaryOp_(DataType _scalar, const _BinaryOp& _op)
        : op_(&_op), input_(_scalar)
    {}
    void operator>>(Image_<DataType>& _output) const
    {
        *op_ >> _output;
        ImageBinaryOpImpl<DataType, Image_<DataType>, opType>::proc(
            input_, _output, _output);
    }

    const _BinaryOp* op_;
    const DataType   input_;
};
template <typename _BinaryOp, ImageOpType opType>
struct ImageBinaryOp_<_BinaryOp, Image_<typename _BinaryOp::DataType>, opType> {
    typedef typename _BinaryOp::DataType DataType;
    ImageBinaryOp_() = delete;
    ImageBinaryOp_(const ImageBinaryOp_& _op) : op_(_op.op_), input_(_op.input_)
    {}
    ImageBinaryOp_(const _BinaryOp& _op, const Image_<DataType>& _src)
        : op_(&_op), input_(&_src)
    {}
    void operator>>(Image_<DataType>& _output) const
    {
        *op_ >> _output;
        ImageBinaryOpImpl<Image_<DataType>, Image_<DataType>, opType>::proc(
            _output, *input_, _output);
    }

    const _BinaryOp*        op_;
    const Image_<DataType>* input_;
};
template <typename _BinaryOp, ImageOpType opType>
struct ImageBinaryOp_<Image_<typename _BinaryOp::DataType>, _BinaryOp, opType> {
    typedef typename _BinaryOp::DataType DataType;
    ImageBinaryOp_() = delete;
    ImageBinaryOp_(const ImageBinaryOp_& _op) : op_(_op.op_), input_(_op.input_)
    {}
    ImageBinaryOp_(const _BinaryOp& _op, const Image_<DataType>& _src)
        : op_(&_op), input_(&_src)
    {}
    void operator>>(Image_<DataType>& _output) const
    {
        *op_ >> _output;
        ImageBinaryOpImpl<Image_<DataType>, Image_<DataType>, opType>::proc(
            *input_, _output, _output);
    }

    const _BinaryOp*       op_;
    const Image_<DataType> input_;
};
template <typename Tp, ImageOpType opType>
struct ImageBinaryOp_<Image_<Tp>, Image_<Tp>, opType> {
    typedef Tp DataType;
    ImageBinaryOp_() = delete;
    ImageBinaryOp_(const ImageBinaryOp_& _op)
        : input1_(_op.input1_), input2_(_op.input2_)
    {}
    ImageBinaryOp_(const Image_<Tp>& _srcA, const Image_<Tp>& _srcB)
        : input1_(&_srcA), input2_(&_srcB)
    {}
    void operator>>(Image_<Tp>& _output) const
    {
        ImageBinaryOpImpl<Image_<Tp>, Image_<Tp>, opType>::proc(
            *input1_, *input2_, _output);
    }

    const Image_<Tp>* input1_;
    const Image_<Tp>* input2_;
};
template <typename Tp, ImageOpType opType>
struct ImageBinaryOp_<Image_<Tp>, Tp, opType> {
    typedef Tp DataType;
    ImageBinaryOp_() = delete;
    ImageBinaryOp_(const ImageBinaryOp_& _op)
        : input1_(_op.input1_), input2_(_op.input2_)
    {}
    ImageBinaryOp_(const Image_<Tp>& _src, Tp _scalar)
        : input1_(&_src), input2_(_scalar)
    {}
    void operator>>(Image_<Tp>& _output) const
    {
        ImageBinaryOpImpl<Image_<Tp>, Tp, opType>::proc(
            *input1_, input2_, _output);
    }

    const Image_<Tp>* input1_;
    const Tp          input2_;
};
template <typename Tp, ImageOpType opType>
struct ImageBinaryOp_<Tp, Image_<Tp>, opType> {
    typedef Tp DataType;
    ImageBinaryOp_() = delete;
    ImageBinaryOp_(const ImageBinaryOp_& _op)
        : input1_(_op.input1_), input2_(_op.input2_)
    {}
    ImageBinaryOp_(Tp _scalar, const Image_<Tp>& _src)
        : input1_(_scalar), input2_(&_src)
    {}
    void operator>>(Image_<Tp>& _output) const
    {
        ImageBinaryOpImpl<Tp, Image_<Tp>, opType>::proc(
            input1_, *input2_, _output);
    }

    const Tp          input1_;
    const Image_<Tp>* input2_;
};
template <typename Tp>
static inline ImageBinaryOp_<Image_<Tp>, Tp, ImageOpType::Plus> operator+(
    coord_traits_t<Tp> _scalar, const Image_<Tp>& _src)
{
    return ImageBinaryOp_<Image_<Tp>, Tp, ImageOpType::Plus>(_src, _scalar);
}
template <typename Tp>
static inline ImageBinaryOp_<Image_<Tp>, Tp, ImageOpType::Plus> operator+(
    const Image_<Tp>& _src, coord_traits_t<Tp> _scalar)
{
    return ImageBinaryOp_<Image_<Tp>, Tp, ImageOpType::Plus>(_src, _scalar);
}
template <typename Tp>
static inline ImageBinaryOp_<Image_<Tp>, Image_<Tp>, ImageOpType::Plus>
operator+(const Image_<Tp>& _src, const Image_<Tp>& _dst)
{
    return ImageBinaryOp_<Image_<Tp>, Image_<Tp>, ImageOpType::Plus>(
        _src, _dst);
}
template <typename _ImageOp> static inline ImageBinaryOp_<_ImageOp,
    typename _ImageOp::DataType, ImageOpType::Plus>
operator+(const _ImageOp& _op, typename _ImageOp::DataType _scalar)
{
    return ImageBinaryOp_<_ImageOp, typename _ImageOp::DataType,
        ImageOpType::Plus>(_op, _scalar);
}
template <typename _ImageOp>
static inline ImageBinaryOp_<typename _ImageOp::DataType, _ImageOp,
    ImageOpType::Plus>
operator+(typename _ImageOp::DataType _scalar, const _ImageOp& _op)
{
    return ImageBinaryOp_<typename _ImageOp::DataType, _ImageOp,
        ImageOpType::Plus>(_scalar, _op);
}
template <typename _ImageOp> static inline ImageBinaryOp_<_ImageOp,
    Image_<typename _ImageOp::DataType>, ImageOpType::Plus>
operator+(const _ImageOp& _op, Image_<typename _ImageOp::DataType>& _img)
{
    return ImageBinaryOp_<_ImageOp, Image_<typename _ImageOp::DataType>,
        ImageOpType::Plus>(_op, _img);
}
template <typename _ImageOp> static inline ImageBinaryOp_<_ImageOp,
    Image_<typename _ImageOp::DataType>, ImageOpType::Plus>
operator+(Image_<typename _ImageOp::DataType>& _img, const _ImageOp& _op)
{
    return ImageBinaryOp_<_ImageOp, Image_<typename _ImageOp::DataType>,
        ImageOpType::Plus>(_op, _img);
}
template <typename _ImageOpA, typename _ImageOpB,
    class = typename std::enable_if<std::is_same<typename _ImageOpA::DataType,
        typename _ImageOpB::DataType>::value>::type>
static inline ImageBinaryOp_<_ImageOpA, _ImageOpB, ImageOpType::Plus> operator+(
    const _ImageOpA& _op1, const _ImageOpB& _op2)
{
    return ImageBinaryOp_<_ImageOpA, _ImageOpB, ImageOpType::Plus>(_op1, _op2);
}
template <typename Tp, typename _Tp>
static inline ImageBinaryOp_<_Tp, Image_<_Tp>, ImageOpType::Minus> operator-(
    Tp _scalar, const Image_<_Tp>& _src)
{
    return ImageBinaryOp_<_Tp, Image_<_Tp>, ImageOpType::Minus>(
        (_Tp)_scalar, _src);
}
template <typename _Tp, typename Tp>
static inline ImageBinaryOp_<Image_<_Tp>, _Tp, ImageOpType::Minus> operator-(
    const Image_<_Tp>& _src, Tp _scalar)
{
    return ImageBinaryOp_<Image_<_Tp>, _Tp, ImageOpType::Minus>(
        _src, (_Tp)_scalar);
}
template <typename _Tp>
static inline ImageBinaryOp_<Image_<_Tp>, Image_<_Tp>, ImageOpType::Minus>
operator-(const Image_<_Tp>& _srcA, const Image_<_Tp>& _srcB)
{
    return ImageBinaryOp_<Image_<_Tp>, Image_<_Tp>, ImageOpType::Minus>(
        _srcA, _srcB);
}
template <typename _ImageOp> static inline ImageBinaryOp_<_ImageOp,
    typename _ImageOp::DataType, ImageOpType::Minus>
operator-(const _ImageOp& _op, typename _ImageOp::DataType _scalar)
{
    return ImageBinaryOp_<_ImageOp, typename _ImageOp::DataType,
        ImageOpType::Minus>(_op, _scalar);
}
template <typename _ImageOp>
static inline ImageBinaryOp_<typename _ImageOp::DataType, _ImageOp,
    ImageOpType::Minus>
operator-(typename _ImageOp::DataType _scalar, const _ImageOp& _op)
{
    return ImageBinaryOp_<typename _ImageOp::DataType, _ImageOp,
        ImageOpType::Minus>(_scalar, _op);
}
template <typename _ImageOp> static inline ImageBinaryOp_<_ImageOp,
    Image_<typename _ImageOp::DataType>, ImageOpType::Minus>
operator-(const _ImageOp& _op, Image_<typename _ImageOp::DataType>& _img)
{
    return ImageBinaryOp_<_ImageOp, Image_<typename _ImageOp::DataType>,
        ImageOpType::Minus>(_op, _img);
}
template <typename _ImageOp>
static inline ImageBinaryOp_<Image_<typename _ImageOp::DataType>, _ImageOp,
    ImageOpType::Minus>
operator-(Image_<typename _ImageOp::DataType>& _img, const _ImageOp& _op)
{
    return ImageBinaryOp_<Image_<typename _ImageOp::DataType>, _ImageOp,
        ImageOpType::Minus>(_img, _op);
}
template <typename _ImageOpA, typename _ImageOpB,
    class = typename std::enable_if<std::is_same<typename _ImageOpA::DataType,
        typename _ImageOpB::DataType>::value>::type>
static inline ImageBinaryOp_<_ImageOpA, _ImageOpB, ImageOpType::Minus>
operator-(const _ImageOpA& _op1, const _ImageOpB& _op2)
{
    return ImageBinaryOp_<_ImageOpA, _ImageOpB, ImageOpType::Minus>(_op1, _op2);
}
template <typename Tp, typename _Tp>
static inline ImageBinaryOp_<_Tp, Image_<_Tp>, ImageOpType::Divide> operator/(
    Tp _scalar, const Image_<_Tp>& _src)
{
    return ImageBinaryOp_<_Tp, Image_<_Tp>, ImageOpType::Divide>(
        (_Tp)_scalar, _src);
}
template <typename _Tp, typename Tp>
static inline ImageBinaryOp_<Image_<_Tp>, _Tp, ImageOpType::Divide> operator/(
    const Image_<_Tp>& _src, Tp _scalar)
{
    return ImageBinaryOp_<Image_<_Tp>, _Tp, ImageOpType::Divide>(
        _src, (_Tp)_scalar);
}
template <typename _Tp>
static inline ImageBinaryOp_<Image_<_Tp>, Image_<_Tp>, ImageOpType::Divide>
operator/(const Image_<_Tp>& _srcA, const Image_<_Tp>& _srcB)
{
    return ImageBinaryOp_<Image_<_Tp>, Image_<_Tp>, ImageOpType::Divide>(
        _srcA, _srcB);
}
template <typename _ImageOp> static inline ImageBinaryOp_<_ImageOp,
    typename _ImageOp::DataType, ImageOpType::Divide>
operator/(const _ImageOp& _op, typename _ImageOp::DataType _scalar)
{
    return ImageBinaryOp_<_ImageOp, typename _ImageOp::DataType,
        ImageOpType::Divide>(_op, _scalar);
}
template <typename _ImageOp>
static inline ImageBinaryOp_<typename _ImageOp::DataType, _ImageOp,
    ImageOpType::Divide>
operator/(typename _ImageOp::DataType _scalar, const _ImageOp& _op)
{
    return ImageBinaryOp_<typename _ImageOp::DataType, _ImageOp,
        ImageOpType::Divide>(_scalar, _op);
}
template <typename _ImageOp> static inline ImageBinaryOp_<_ImageOp,
    Image_<typename _ImageOp::DataType>, ImageOpType::Divide>
operator/(const _ImageOp& _op, Image_<typename _ImageOp::DataType>& _img)
{
    return ImageBinaryOp_<_ImageOp, Image_<typename _ImageOp::DataType>,
        ImageOpType::Divide>(_op, _img);
}
template <typename _ImageOp>
static inline ImageBinaryOp_<Image_<typename _ImageOp::DataType>, _ImageOp,
    ImageOpType::Divide>
operator/(Image_<typename _ImageOp::DataType>& _img, const _ImageOp& _op)
{
    return ImageBinaryOp_<Image_<typename _ImageOp::DataType>, _ImageOp,
        ImageOpType::Divide>(_img, _op);
}
template <typename _ImageOpA, typename _ImageOpB,
    class = typename std::enable_if<std::is_same<typename _ImageOpA::DataType,
                                        typename _ImageOpB::DataType>::value,
        typename _ImageOpA::DataType>::type>
static inline ImageBinaryOp_<_ImageOpA, _ImageOpB, ImageOpType::Divide>
operator/(const _ImageOpA& _op1, const _ImageOpB& _op2)
{
    return ImageBinaryOp_<_ImageOpA, _ImageOpB, ImageOpType::Divide>(
        _op1, _op2);
}
template <typename Tp, typename _Tp>
static inline ImageBinaryOp_<_Tp, Image_<_Tp>, ImageOpType::Multiply> operator*(
    Tp _scalar, const Image_<_Tp>& _src)
{
    return ImageBinaryOp_<_Tp, Image_<_Tp>, ImageOpType::Multiply>(
        (_Tp)_scalar, _src);
}
template <typename _Tp, typename Tp>
static inline ImageBinaryOp_<Image_<_Tp>, _Tp, ImageOpType::Multiply> operator*(
    const Image_<_Tp>& _src, Tp _scalar)
{
    return ImageBinaryOp_<Image_<_Tp>, _Tp, ImageOpType::Multiply>(
        _src, (_Tp)_scalar);
}
template <typename _Tp>
static inline ImageBinaryOp_<Image_<_Tp>, Image_<_Tp>, ImageOpType::Multiply>
operator*(const Image_<_Tp>& _srcA, const Image_<_Tp>& _srcB)
{
    return ImageBinaryOp_<Image_<_Tp>, Image_<_Tp>, ImageOpType::Multiply>(
        _srcA, _srcB);
}
template <typename _ImageOp> static inline ImageBinaryOp_<_ImageOp,
    typename _ImageOp::DataType, ImageOpType::Multiply>
operator*(const _ImageOp& _op, typename _ImageOp::DataType _scalar)
{
    return ImageBinaryOp_<_ImageOp, typename _ImageOp::DataType,
        ImageOpType::Multiply>(_op, _scalar);
}
template <typename _ImageOp>
static inline ImageBinaryOp_<typename _ImageOp::DataType, _ImageOp,
    ImageOpType::Multiply>
operator*(typename _ImageOp::DataType _scalar, const _ImageOp& _op)
{
    return ImageBinaryOp_<typename _ImageOp::DataType, _ImageOp,
        ImageOpType::Multiply>(_scalar, _op);
}
template <typename _ImageOp> static inline ImageBinaryOp_<_ImageOp,
    Image_<typename _ImageOp::DataType>, ImageOpType::Multiply>
operator*(const _ImageOp& _op, Image_<typename _ImageOp::DataType>& _img)
{
    return ImageBinaryOp_<_ImageOp, Image_<typename _ImageOp::DataType>,
        ImageOpType::Multiply>(_op, _img);
}
template <typename _ImageOp>
static inline ImageBinaryOp_<Image_<typename _ImageOp::DataType>, _ImageOp,
    ImageOpType::Multiply>
operator*(Image_<typename _ImageOp::DataType>& _img, const _ImageOp& _op)
{
    return ImageBinaryOp_<Image_<typename _ImageOp::DataType>, _ImageOp,
        ImageOpType::Multiply>(_img, _op);
}
template <typename _ImageOpA, typename _ImageOpB,
    class = typename std::enable_if<std::is_same<typename _ImageOpA::DataType,
        typename _ImageOpB::DataType>::value>::type>
static inline ImageBinaryOp_<_ImageOpA, _ImageOpB, ImageOpType::Multiply>
operator*(const _ImageOpA& _op1, const _ImageOpB& _op2)
{
    return ImageBinaryOp_<_ImageOpA, _ImageOpB, ImageOpType::Multiply>(
        _op1, _op2);
}
// template <typename _Tp> static inline Image_<_Tp>& operator+=(Image_<_Tp>&
// _img, _Tp _scalar)
//{
//    ImageBinaryOpImpl<Image_<_Tp>, _Tp, ImageOpType::Plus>::proc(_img,
//    _scalar, _img); return _img;
//}
template <typename Tp> static inline Image_<Tp>& operator+=(
    Image_<Tp>& _img, typename Image_<Tp>::Type _scalar)
{
    ImageBinaryOpImpl<Image_<Tp>, Tp, ImageOpType::Plus>::proc(
        _img, _scalar, _img);
    return _img;
}
template <typename Tp>
static inline Image_<Tp>& operator+=(Image_<Tp>& _src, const Image_<Tp>& _dst)
{
    ImageBinaryOpImpl<Image_<Tp>, Image_<Tp>, ImageOpType::Plus>::proc(
        _src, _dst, _src);
    return _src;
}
template <typename _ImageOp, typename Tp,
    class = typename std::enable_if<
        std::is_same<typename _ImageOp::DataType, Tp>::value>::type>
static inline Image_<Tp>& operator+=(Image_<Tp>& _img, const _ImageOp& _op)
{
    ImageBinaryOpImpl<Image_<Tp>, Image_<Tp>, ImageOpType::Plus>::proc(
        _img, _op, _img);
    return _img;
}
// template <typename _ImageT, typename _ScalarT, ImageOpType _OpType>
// static inline Image_<typename ImageBinaryOp_<_ImageT, _ScalarT,
// _OpType>::DataType>& operator+=(
//     Image_<typename ImageBinaryOp_<_ImageT, _ScalarT, _OpType>::DataType>&
//     _img, const ImageBinaryOp_<_ImageT, _ScalarT, _OpType>& _op)
// {
//     using DataType = typename ImageBinaryOp_<_ImageT, _ScalarT,
//     _OpType>::DataType;
//     // Image_<DataType> temp = _op;
//     ImageBinaryOpImpl<Image_<DataType>, Image_<DataType>,
//     ImageOpType::Plus>::proc(_img, _op, _img); return _img;
// }
template <typename Tp> static inline Image_<Tp>& operator-=(
    Image_<Tp>& _img, typename Image_<Tp>::Type _scalar)
{
    ImageBinaryOpImpl<Image_<Tp>, Tp, ImageOpType::Minus>::proc(
        _img, _scalar, _img);
    return _img;
}
template <typename Tp>
static inline Image_<Tp>& operator-=(Image_<Tp>& _src, const Image_<Tp>& _dst)
{
    ImageBinaryOpImpl<Image_<Tp>, Image_<Tp>, ImageOpType::Minus>::proc(
        _src, _dst, _src);
    return _src;
}
// template <typename _ImageT, typename _ScalarT, ImageOpType _OpType>
// static inline Image_<typename ImageBinaryOp_<_ImageT, _ScalarT,
// _OpType>::DataType>& operator-=(
//     Image_<typename ImageBinaryOp_<_ImageT, _ScalarT, _OpType>::DataType>&
//     _img, const ImageBinaryOp_<_ImageT, _ScalarT, _OpType>& _op)
// {
//     using DataType = typename ImageBinaryOp_<_ImageT, _ScalarT,
//     _OpType>::DataType;
//     // Image_<DataType> temp = _op;
//     ImageBinaryOpImpl<Image_<DataType>, Image_<DataType>,
//     ImageOpType::Minus>::proc(_img, _op, _img); return _img;
// }
template <typename _ImageOp, typename Tp,
    class = typename std::enable_if<
        std::is_same<typename _ImageOp::DataType, Tp>::value>::type>
static inline Image_<Tp>& operator-=(Image_<Tp>& _img, const _ImageOp& _op)
{
    ImageBinaryOpImpl<Image_<Tp>, Image_<Tp>, ImageOpType::Minus>::proc(
        _img, _op, _img);
    return _img;
}
template <typename Tp> static inline Image_<Tp>& operator*=(
    Image_<Tp>& _img, typename Image_<Tp>::Type _scalar)
{
    ImageBinaryOpImpl<Image_<Tp>, Tp, ImageOpType::Multiply>::proc(
        _img, _scalar, _img);
    return _img;
}
template <typename Tp>
static inline Image_<Tp>& operator*=(Image_<Tp>& _src, const Image_<Tp>& _dst)
{
    ImageBinaryOpImpl<Image_<Tp>, Image_<Tp>, ImageOpType::Multiply>::proc(
        _src, _dst, _src);
    return _src;
}
template <typename _ImageOp, typename Tp,
    class = typename std::enable_if<
        std::is_same<typename _ImageOp::DataType, Tp>::value>::type>
static inline Image_<Tp>& operator*=(Image_<Tp>& _img, const _ImageOp& _op)
{
    ImageBinaryOpImpl<Image_<Tp>, Image_<Tp>, ImageOpType::Multiply>::proc(
        _img, _op, _img);
    return _img;
}
template <typename Tp> static inline Image_<Tp>& operator/=(
    Image_<Tp>& _img, typename Image_<Tp>::Type _scalar)
{
    ImageBinaryOpImpl<Image_<Tp>, Tp, ImageOpType::Divide>::proc(
        _img, _scalar, _img);
    return _img;
}
template <typename Tp>
static inline Image_<Tp>& operator/=(Image_<Tp>& _src, const Image_<Tp>& _dst)
{
    ImageBinaryOpImpl<Image_<Tp>, Image_<Tp>, ImageOpType::Divide>::proc(
        _src, _dst, _src);
    return _src;
}
template <typename _ImageOp, typename Tp,
    class = typename std::enable_if<
        std::is_same<typename _ImageOp::DataType, Tp>::value>::type>
static inline Image_<Tp>& operator/=(Image_<Tp>& _img, const _ImageOp& _op)
{
    ImageBinaryOpImpl<Image_<Tp>, Image_<Tp>, ImageOpType::Divide>::proc(
        _img, _op, _img);
    return _img;
}

#if defined(RULERMVS_USE_SSE)
template <> struct ImageRemapper_<uchar, float, SamplerType::MVS_BILINEAR> {
    static inline void remap(const Image8u& src, const Imagef& mapx,
        const Imagef& mapy, Image8u& dst)
    {
        assert(!mapx.empty() && mapy.size() == mapx.size());
        if (src.empty()) return;
        if (dst.size() != mapx.size()) dst.create(mapx.size());
        if (!checkHardwareSupport(SIMDMode::MVS_SSE3)) {
            for (int i = 0; i < mapx.height; ++i) {
                uchar*       dst_ptr  = dst.ptr(i);
                const float *mapx_ptr = mapx.ptr(i), *mapy_ptr = mapy.ptr(i);
                for (int j = 0; j < mapx.width; ++j)
                    interpolate<uchar, float, SamplerType::MVS_BILINEAR>(
                        src, mapx_ptr[j], mapy_ptr[j], dst_ptr[j]);
            }
            return;
        }
        const __m128i       one    = _mm_set1_epi32(1);
        const __m128i       zero   = _mm_setzero_si128();
        const __m128i       w_1    = _mm_set1_epi32(src.width - 1);
        const __m128i       h_1    = _mm_set1_epi32(src.height - 1);
        const __m128i       stride = _mm_set1_epi32(src.stride);
        alignas(16) __m128i a0[4], a1[4], b0[4], b1[4];
        alignas(16) int     ind0[16], ind1[16], ind2[16], ind3[16];
        const int           loop = mapx.width >> 4;
        for (int i = 0; i < mapx.height; ++i) {
            uchar*       dst_ptr  = dst.ptr(i);
            const float *mapx_ptr = mapx.ptr(i), *mapy_ptr = mapy.ptr(i);
            for (int j = 0; j < loop; j++) {
                const float* ptrmapx = &mapx_ptr[j << 4];
                const float* ptrmapy = &mapy_ptr[j << 4];
                for (int k = 0; k < 4; ++k) {
                    __m128  mx  = _mm_load_ps(&ptrmapx[k << 2]);
                    __m128  my  = _mm_load_ps(&ptrmapy[k << 2]);
                    __m128  mxf = _mm_round_ps(mx, 0x01 | 0x00);
                    __m128  myf = _mm_round_ps(my, 0x01 | 0x00);
                    __m128i xi  = _mm_cvtps_epi32(mxf);
                    __m128i yi  = _mm_cvtps_epi32(myf);
                    __m128i mi =
                        _mm_and_si128(_mm_and_si128(_mm_cmpgt_epi32(xi, zero),
                                          _mm_cmpgt_epi32(yi, zero)),
                            _mm_and_si128(_mm_cmpgt_epi32(w_1, xi),
                                _mm_cmpgt_epi32(h_1, yi)));
                    xi         = _mm_blendv_epi8(zero, xi, mi);
                    yi         = _mm_blendv_epi8(zero, yi, mi);
                    __m128i i0 = _mm_add_epi32(_mm_mullo_epi32(yi, stride), xi);
                    __m128i i1 = _mm_add_epi32(i0, one);
                    __m128i i2 = _mm_add_epi32(i0, stride);
                    __m128i i3 = _mm_add_epi32(i2, one);

                    _mm_store_si128((__m128i*)&ind0[k << 2], i0);
                    _mm_store_si128((__m128i*)&ind1[k << 2], i1);
                    _mm_store_si128((__m128i*)&ind2[k << 2], i2);
                    _mm_store_si128((__m128i*)&ind3[k << 2], i3);
                    // 这里转成整数进行乘法运算，会损失一定的精度.
                    __m128i ma = _mm_cvtps_epi32(
                        _mm_mul_ps(_mm_sub_ps(mx, mxf), _mm_set1_ps(256.0f)));
                    __m128i mb = _mm_cvtps_epi32(
                        _mm_mul_ps(_mm_sub_ps(my, myf), _mm_set1_ps(256.0f)));
                    a1[k] = _mm_blendv_epi8(zero, ma, mi);
                    b1[k] = _mm_blendv_epi8(zero, mb, mi);
                    a0[k] = _mm_blendv_epi8(
                        zero, _mm_sub_epi32(_mm_set1_epi32(256), ma), mi);
                    b0[k] = _mm_blendv_epi8(
                        zero, _mm_sub_epi32(_mm_set1_epi32(256), mb), mi);
                }

                __m128i v0 = _mm_setr_epi16(src.data[ind0[0]],
                    src.data[ind0[1]], src.data[ind0[2]], src.data[ind0[3]],
                    src.data[ind0[4]], src.data[ind0[5]], src.data[ind0[6]],
                    src.data[ind0[7]]);
                __m128i v1 = _mm_setr_epi16(src.data[ind1[0]],
                    src.data[ind1[1]], src.data[ind1[2]], src.data[ind1[3]],
                    src.data[ind1[4]], src.data[ind1[5]], src.data[ind1[6]],
                    src.data[ind1[7]]);
                __m128i v2 = _mm_setr_epi16(src.data[ind2[0]],
                    src.data[ind2[1]], src.data[ind2[2]], src.data[ind2[3]],
                    src.data[ind2[4]], src.data[ind2[5]], src.data[ind2[6]],
                    src.data[ind2[7]]);
                __m128i v3 = _mm_setr_epi16(src.data[ind3[0]],
                    src.data[ind3[1]], src.data[ind3[2]], src.data[ind3[3]],
                    src.data[ind3[4]], src.data[ind3[5]], src.data[ind3[6]],
                    src.data[ind3[7]]);

                __m128i ma0 = _mm_packs_epi32(a0[0], a0[1]);
                __m128i ma1 = _mm_packs_epi32(a1[0], a1[1]);
                __m128i mb0 = _mm_packs_epi32(b0[0], b0[1]);
                __m128i mb1 = _mm_packs_epi32(b1[0], b1[1]);
                __m128i v01 = _mm_add_epi16(
                    _mm_mullo_epi16(v0, ma0), _mm_mullo_epi16(v1, ma1));
                __m128i v23 = _mm_add_epi16(
                    _mm_mullo_epi16(v2, ma0), _mm_mullo_epi16(v3, ma1));
                __m128i v0123 = _mm_srli_epi16(
                    _mm_add_epi16(_mm_mullo_epi16(_mm_srli_epi16(v01, 8), mb0),
                        _mm_mullo_epi16(_mm_srli_epi16(v23, 8), mb1)),
                    8);

                __m128i v4 = _mm_setr_epi16(src.data[ind0[8]],
                    src.data[ind0[9]], src.data[ind0[10]], src.data[ind0[11]],
                    src.data[ind0[12]], src.data[ind0[13]], src.data[ind0[14]],
                    src.data[ind0[15]]);
                __m128i v5 = _mm_setr_epi16(src.data[ind1[8]],
                    src.data[ind1[9]], src.data[ind1[10]], src.data[ind1[11]],
                    src.data[ind1[12]], src.data[ind1[13]], src.data[ind1[14]],
                    src.data[ind1[15]]);
                __m128i v6 = _mm_setr_epi16(src.data[ind2[8]],
                    src.data[ind2[9]], src.data[ind2[10]], src.data[ind2[11]],
                    src.data[ind2[12]], src.data[ind2[13]], src.data[ind2[14]],
                    src.data[ind2[15]]);
                __m128i v7 = _mm_setr_epi16(src.data[ind3[8]],
                    src.data[ind3[9]], src.data[ind3[10]], src.data[ind3[11]],
                    src.data[ind3[12]], src.data[ind3[13]], src.data[ind3[14]],
                    src.data[ind3[15]]);

                __m128i ma2 = _mm_packs_epi32(a0[2], a0[3]);
                __m128i ma3 = _mm_packs_epi32(a1[2], a1[3]);
                __m128i mb2 = _mm_packs_epi32(b0[2], b0[3]);
                __m128i mb3 = _mm_packs_epi32(b1[2], b1[3]);
                __m128i v45 = _mm_add_epi16(
                    _mm_mullo_epi16(v4, ma2), _mm_mullo_epi16(v5, ma3));
                __m128i v67 = _mm_add_epi16(
                    _mm_mullo_epi16(v6, ma2), _mm_mullo_epi16(v7, ma3));
                __m128i v4567 = _mm_srli_epi16(
                    _mm_add_epi16(_mm_mullo_epi16(_mm_srli_epi16(v45, 8), mb2),
                        _mm_mullo_epi16(_mm_srli_epi16(v67, 8), mb3)),
                    8);

                _mm_storeu_si128(
                    (__m128i*)&dst_ptr[j << 4], _mm_packus_epi16(v0123, v4567));
            }
            for (int j = loop << 4; j < mapx.width; ++j)
                interpolate<uchar, float, SamplerType::MVS_BILINEAR>(
                    src, mapx_ptr[j], mapy_ptr[j], dst_ptr[j]);
        }
    }
};
#endif

}  // namespace rulermvs
#endif  // _RULERMVS_CORE_IMAGE_IMPL_HPP_