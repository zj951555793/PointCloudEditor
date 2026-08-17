#pragma once

#include "ColorPicking24.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace JMEngine {

// RGB24 Block Picking 的平台无关分块描述。
//
// 为什么需要分块：
// RGB24 单次只有 24 bit 可用于对象 ID，且 0 必须保留给背景，因此一个 Picking Pass
// 最多只能编码 16,777,215 个对象。通过把模型拆成多个 PickBlock，每个 Pass 只编码
// localId，CPU 再通过 firstObject 还原 globalId，可以让总点数/三角形数量不受 RGB24
// 总 ID 数限制。
//
// Desktop OpenGL 2.1 与 OpenGL ES 3.1 共用完全相同的规则。
struct PickBlock24 {
    std::uint64_t firstObject{0};
    std::uint32_t objectCount{0};

    constexpr std::uint64_t endObject() const noexcept {
        return firstObject + static_cast<std::uint64_t>(objectCount);
    }
};

// 工程默认每个 Picking Block 为 100 万对象。
// 该值远小于 RGB24 极限，既方便千万级模型分块，也避免一次 Pass 处理过大的对象范围。
constexpr std::uint32_t kDefaultPickBlockSize = 1'000'000u;

inline constexpr std::uint32_t sanitizePickBlockSize(std::uint32_t requested) noexcept {
    if (requested == 0u)
        return kDefaultPickBlockSize;
    const std::uint32_t maxCount = kColorPicking24MaxObjectId + 1u;
    return requested > maxCount ? maxCount : requested;
}

inline constexpr std::uint64_t pickBlockCount(std::uint64_t totalObjects,
                                              std::uint32_t blockSize = kDefaultPickBlockSize) noexcept {
    const std::uint64_t size = sanitizePickBlockSize(blockSize);
    return totalObjects == 0u ? 0u : (totalObjects + size - 1u) / size;
}

inline constexpr PickBlock24 pickBlockAt(std::uint64_t totalObjects, std::uint64_t blockIndex,
                                         std::uint32_t blockSize = kDefaultPickBlockSize) noexcept {
    const std::uint64_t size = sanitizePickBlockSize(blockSize);
    const std::uint64_t first = blockIndex * size;
    if (first >= totalObjects)
        return {};

    const std::uint64_t remaining = totalObjects - first;
    return {first, static_cast<std::uint32_t>(remaining < size ? remaining : size)};
}

// GPU 中只编码 block 内 localId。
inline constexpr ColorId24 encodeBlockLocalId24(std::uint32_t localId) noexcept {
    return encodeColorId24(localId);
}

// 将 RGB24 解出的 localId 恢复成整个模型内的 globalId。
inline constexpr bool decodeBlockGlobalId24(const PickBlock24& block, std::uint8_t r, std::uint8_t g, std::uint8_t b,
                                            std::uint64_t& globalId) noexcept {
    std::uint32_t localId = 0u;
    if (!decodeColorId24(r, g, b, localId) || localId >= block.objectCount) {
        globalId = 0u;
        return false;
    }
    globalId = block.firstObject + static_cast<std::uint64_t>(localId);
    return true;
}

} // namespace JMEngine
