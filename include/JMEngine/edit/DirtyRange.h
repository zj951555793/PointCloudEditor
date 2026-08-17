#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace JMEngine {

// 连续脏区间。Renderer 可直接把它转换成 glBufferSubData 的 offset/count。
struct DirtyRange {
    std::uint32_t first{0};
    std::uint32_t count{0};
};

// 把有序/无序 ID 压缩成连续 range，避免 UI/Renderer 重复实现相同逻辑。
std::vector<DirtyRange> makeDirtyRanges(std::vector<std::uint32_t> ids);

} // namespace JMEngine
