#include <pceditor/edit/DirtyRange.h>

#include <algorithm>

namespace pceditor {

std::vector<DirtyRange> makeDirtyRanges(std::vector<std::uint32_t> ids) {
    if (ids.empty())
        return {};
    if (!std::is_sorted(ids.begin(), ids.end()))
        std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());

    std::vector<DirtyRange> out;
    out.reserve(ids.size() / 8u + 1u);
    std::uint32_t first = ids.front();
    std::uint32_t prev = first;
    for (std::size_t i = 1; i < ids.size(); ++i) {
        if (ids[i] == prev + 1u) {
            prev = ids[i];
            continue;
        }
        out.push_back({first, prev - first + 1u});
        first = prev = ids[i];
    }
    out.push_back({first, prev - first + 1u});
    return out;
}

} // namespace pceditor
