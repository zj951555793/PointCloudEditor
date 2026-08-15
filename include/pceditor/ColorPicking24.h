#pragma once

#include <cstdint>

namespace pceditor {

// OpenGL 2.1 兼容的 24-bit Color Picking 编解码工具。
//
// 设计约定：
// - RGB=(0,0,0) 保留给“背景/无命中”；
// - 实际对象 id 在写入 GPU 前编码为 id+1；
// - 因此单次可唯一编码的对象数量为 16,777,215（对象 ID 0 ~ 16,777,214）。
//
// 该头文件完全不依赖 OpenGL/Qt，可由 Win32/WGL、老显卡后端或单元测试直接复用。
struct ColorId24 {
    std::uint8_t r{0};
    std::uint8_t g{0};
    std::uint8_t b{0};
};

constexpr std::uint32_t kColorPicking24Background = 0u;
constexpr std::uint32_t kColorPicking24MaxEncoded = 0x00FFFFFFu;
constexpr std::uint32_t kColorPicking24MaxObjectId = kColorPicking24MaxEncoded - 1u;

inline constexpr ColorId24 encodeColorId24(std::uint32_t objectId) noexcept {
    const std::uint32_t encoded = objectId + 1u; // 0 留给背景
    return {static_cast<std::uint8_t>(encoded & 0xFFu), static_cast<std::uint8_t>((encoded >> 8) & 0xFFu),
            static_cast<std::uint8_t>((encoded >> 16) & 0xFFu)};
}

// 返回 true 表示命中了有效对象；false 表示背景。
inline constexpr bool decodeColorId24(std::uint8_t r, std::uint8_t g, std::uint8_t b,
                                      std::uint32_t& objectId) noexcept {
    const std::uint32_t encoded =
        static_cast<std::uint32_t>(r) | (static_cast<std::uint32_t>(g) << 8) | (static_cast<std::uint32_t>(b) << 16);

    if (encoded == kColorPicking24Background) {
        objectId = 0u;
        return false;
    }

    objectId = encoded - 1u;
    return true;
}

} // namespace pceditor
