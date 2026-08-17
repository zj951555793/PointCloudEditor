#pragma once

#include "DirtyRange.h"
#include <cstdint>
#include <vector>

namespace JMEngine {

// Core 编辑操作返回给 Renderer 的统一结果。
// Core 不知道 OpenGL，只描述“哪些逻辑数据发生了变化”。
struct EditResult {
    bool changed{false};
    bool topologyChanged{false};
    bool geometryChanged{false};
    bool selectionChanged{false};
    bool fullRebuild{false};

    std::vector<DirtyRange> pointFlagRanges;
    std::vector<DirtyRange> pointPositionRanges;
    std::vector<DirtyRange> triangleFlagRanges;

    void clear() {
        changed = topologyChanged = geometryChanged = selectionChanged = fullRebuild = false;
        pointFlagRanges.clear();
        pointPositionRanges.clear();
        triangleFlagRanges.clear();
    }
};

} // namespace JMEngine
