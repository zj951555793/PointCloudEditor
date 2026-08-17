#pragma once

#include "CpuSelector.h"
#include "IGpuPicker.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

namespace JMEngine {

// GPU ID Buffer 的通用同步适配器。
//
// 本类自身不包含 OpenGL/GLES 头文件，而是通过回调读取一块 uint32 像素区域，
// 因此 Qt OpenGL、Win32 WGL、EGL/GLES 都能复用同一套 Picking 逻辑。
//
// 重要：glReadPixels 本身可能让 GPU/CPU 同步，因此千万点量产 UI 建议使用 PBO 异步读回。
// Qt 示例已经演示异步 PBO；本类主要用于简单程序、测试和小区域读取。
class PixelIdPicker final : public IGpuPicker {
  public:
    // 回调参数依次是 x/y/宽/高/输出缓冲区。
    // 返回 true 表示读取成功，false 表示图形 API 读取失败。
    using ReadPixelsFn = std::function<bool(int x, int y, int width, int height, std::uint32_t* dst)>;

    PixelIdPicker(int framebufferWidth, int framebufferHeight, ReadPixelsFn reader, bool inputOriginTopLeft = true)
        : width_(framebufferWidth), height_(framebufferHeight), reader_(std::move(reader)),
          topLeft_(inputOriginTopLeft) {}

    // FBO 尺寸变化后必须同步更新。
    void resize(int w, int h) noexcept {
        width_ = w;
        height_ = h;
    }

    PointId pickPoint(int x, int y) override {
        if (!validPoint(x, y) || !reader_)
            return kInvalidPointId;

        std::uint32_t id = kInvalidPointId;
        const int readY = convertY(y, 1);
        if (!reader_(x, readY, 1, 1, &id))
            return kInvalidPointId;
        return id;
    }

    std::vector<PointId> pickRectangle(int x1, int y1, int x2, int y2) override {
        const Bounds bounds = normalizeBounds(x1, y1, x2, y2);
        if (!bounds.valid)
            return {};
        if (!readBounds(bounds))
            return {};

        std::vector<PointId> out;
        out.reserve(scratch_.size() / 4 + 1);
        for (const auto id : scratch_) {
            if (id != kInvalidPointId)
                out.push_back(id);
        }
        normalizeIds(out);
        return out;
    }

    // 圆形选择。圆心/半径都使用输入坐标系（默认左上角 UI 坐标）。
    std::vector<PointId> pickCircle(int cx, int cy, int radiusPixels) {
        const int r = std::max(0, radiusPixels);
        const Bounds bounds = normalizeBounds(cx - r, cy - r, cx + r, cy + r);
        if (!bounds.valid || !readBounds(bounds))
            return {};

        const float rr = static_cast<float>(r * r);
        std::vector<PointId> out;
        forEachScratchPixel(bounds, [&](int x, int y, PointId id) {
            if (id == kInvalidPointId)
                return;
            const float dx = static_cast<float>(x - cx);
            const float dy = static_cast<float>(y - cy);
            if (dx * dx + dy * dy <= rr)
                out.push_back(id);
        });
        normalizeIds(out);
        return out;
    }

    // 套索选择。polygon 至少需要 3 个屏幕点。
    std::vector<PointId> pickLasso(const std::vector<Point2i>& polygon) {
        if (polygon.size() < 3)
            return {};

        int minX = polygon.front().x;
        int maxX = minX;
        int minY = polygon.front().y;
        int maxY = minY;
        for (const auto& p : polygon) {
            minX = std::min(minX, p.x);
            maxX = std::max(maxX, p.x);
            minY = std::min(minY, p.y);
            maxY = std::max(maxY, p.y);
        }

        const Bounds bounds = normalizeBounds(minX, minY, maxX, maxY);
        if (!bounds.valid || !readBounds(bounds))
            return {};

        std::vector<PointId> out;
        forEachScratchPixel(bounds, [&](int x, int y, PointId id) {
            if (id != kInvalidPointId && pointInPolygon(x, y, polygon))
                out.push_back(id);
        });
        normalizeIds(out);
        return out;
    }

    // 画刷路径选择。一个像素距离任意路径线段不超过 radiusPixels 即命中。
    std::vector<PointId> pickBrushStroke(const std::vector<Point2i>& path, int radiusPixels) {
        if (path.empty())
            return {};
        const int r = std::max(0, radiusPixels);

        int minX = path.front().x;
        int maxX = minX;
        int minY = path.front().y;
        int maxY = minY;
        for (const auto& p : path) {
            minX = std::min(minX, p.x);
            maxX = std::max(maxX, p.x);
            minY = std::min(minY, p.y);
            maxY = std::max(maxY, p.y);
        }

        const Bounds bounds = normalizeBounds(minX - r, minY - r, maxX + r, maxY + r);
        if (!bounds.valid || !readBounds(bounds))
            return {};

        const float rr = static_cast<float>(r * r);
        std::vector<PointId> out;
        forEachScratchPixel(bounds, [&](int x, int y, PointId id) {
            if (id == kInvalidPointId)
                return;

            bool hit = false;
            if (path.size() == 1) {
                const float dx = static_cast<float>(x - path.front().x);
                const float dy = static_cast<float>(y - path.front().y);
                hit = dx * dx + dy * dy <= rr;
            } else {
                for (std::size_t i = 1; i < path.size(); ++i) {
                    if (distanceSquaredToSegment(static_cast<float>(x), static_cast<float>(y),
                                                 static_cast<float>(path[i - 1].x), static_cast<float>(path[i - 1].y),
                                                 static_cast<float>(path[i].x), static_cast<float>(path[i].y)) <= rr) {
                        hit = true;
                        break;
                    }
                }
            }
            if (hit)
                out.push_back(id);
        });
        normalizeIds(out);
        return out;
    }

  private:
    struct Bounds {
        int left{0};
        int right{-1};
        int top{0};
        int bottom{-1};
        int width{0};
        int height{0};
        bool valid{false};
    };

    bool validPoint(int x, int y) const noexcept {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    Bounds normalizeBounds(int x1, int y1, int x2, int y2) const noexcept {
        Bounds b;
        if (!reader_ || width_ <= 0 || height_ <= 0)
            return b;

        b.left = std::clamp(std::min(x1, x2), 0, width_ - 1);
        b.right = std::clamp(std::max(x1, x2), 0, width_ - 1);
        b.top = std::clamp(std::min(y1, y2), 0, height_ - 1);
        b.bottom = std::clamp(std::max(y1, y2), 0, height_ - 1);
        b.width = b.right - b.left + 1;
        b.height = b.bottom - b.top + 1;
        b.valid = b.width > 0 && b.height > 0;
        return b;
    }

    bool readBounds(const Bounds& b) {
        scratch_.resize(static_cast<std::size_t>(b.width) * b.height);
        const int readY = convertY(b.top, b.height);
        return reader_(b.left, readY, b.width, b.height, scratch_.data());
    }

    // 遍历读回像素并恢复到“输入坐标系”。
    // glReadPixels 的第 0 行对应读取区域底部；UI 输入原点在左上时必须把行反过来。
    template <typename Fn> void forEachScratchPixel(const Bounds& b, Fn&& fn) const {
        for (int row = 0; row < b.height; ++row) {
            const int inputY = topLeft_ ? (b.bottom - row) : (b.top + row);
            for (int col = 0; col < b.width; ++col) {
                const auto index = static_cast<std::size_t>(row) * b.width + col;
                fn(b.left + col, inputY, scratch_[index]);
            }
        }
    }

    static bool pointInPolygon(int x, int y, const std::vector<Point2i>& polygon) {
        bool inside = false;
        const float px = static_cast<float>(x);
        const float py = static_cast<float>(y);
        for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
            const float xi = static_cast<float>(polygon[i].x);
            const float yi = static_cast<float>(polygon[i].y);
            const float xj = static_cast<float>(polygon[j].x);
            const float yj = static_cast<float>(polygon[j].y);
            const bool crosses = ((yi > py) != (yj > py)) &&
                                 (px < (xj - xi) * (py - yi) / ((yj - yi) == 0.0f ? 1.0e-20f : (yj - yi)) + xi);
            if (crosses)
                inside = !inside;
        }
        return inside;
    }

    static float distanceSquaredToSegment(float px, float py, float ax, float ay, float bx, float by) {
        const float abx = bx - ax;
        const float aby = by - ay;
        const float apx = px - ax;
        const float apy = py - ay;
        const float denom = abx * abx + aby * aby;
        float t = 0.0f;
        if (denom > 1.0e-12f) {
            t = std::clamp((apx * abx + apy * aby) / denom, 0.0f, 1.0f);
        }
        const float dx = px - (ax + t * abx);
        const float dy = py - (ay + t * aby);
        return dx * dx + dy * dy;
    }

    static void normalizeIds(std::vector<PointId>& ids) {
        std::sort(ids.begin(), ids.end());
        ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    }

    // Qt/Windows 鼠标坐标通常左上角为原点，而 glReadPixels 使用左下角原点。
    int convertY(int topY, int rectHeight) const noexcept {
        if (!topLeft_)
            return topY;
        return height_ - topY - rectHeight;
    }

    int width_{0};
    int height_{0};
    ReadPixelsFn reader_;
    bool topLeft_{true};
    std::vector<std::uint32_t> scratch_;
};

} // namespace JMEngine
