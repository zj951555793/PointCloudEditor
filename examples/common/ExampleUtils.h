#pragma once

#include <pceditor/PointCloudIO.h>
#include <pceditor/Types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace pceditor::example {

// Windows 控制台默认经常不是 UTF-8，直接 std::cout 中文会乱码。
// 示例程序启动时调用一次即可；核心库本身不修改宿主程序控制台设置。
inline void enableUtf8Console() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
}

// 以 UTF-8 字符串作为输入输出到控制台。
// Windows 下不依赖当前代码页，直接转换成 UTF-16 后 WriteConsoleW，
// 可以解决 VS/系统为 GBK 时 std::cout 中文出现乱码的问题。
inline void consolePrintUtf8(const std::string& text, bool error = false) {
#ifdef _WIN32
    const int chars = MultiByteToWideChar(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), nullptr, 0);
    if (chars > 0) {
        std::wstring wide(static_cast<std::size_t>(chars), L'\0');
        MultiByteToWideChar(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), wide.data(), chars);
        HANDLE h = GetStdHandle(error ? STD_ERROR_HANDLE : STD_OUTPUT_HANDLE);
        DWORD mode = 0;
        if (h != INVALID_HANDLE_VALUE && h != nullptr && GetConsoleMode(h, &mode)) {
            DWORD written = 0;
            WriteConsoleW(h, wide.data(), static_cast<DWORD>(wide.size()), &written, nullptr);
            return;
        }
        // GUI 子系统或没有控制台时，调试器仍能在 VS Output 中看到 Unicode。
        OutputDebugStringW(wide.c_str());
        return;
    }
#endif
    (error ? std::cerr : std::cout) << text;
}

// 示例统一的文件加载入口。
//
// 1.5.1 起不再创建“内置测试模型”：
// - fileName 为空时返回一个空 PointCloud；
// - 只有用户明确给出 OBJ/PLY 路径时才加载文件。
// 这样 Qt/Windows/RK3588 三套示例的启动行为一致，不会把测试几何误认为真实模型。
inline std::shared_ptr<PointCloud> loadCloudOrEmpty(const std::string& fileName) {
    if (fileName.empty()) {
        consolePrintUtf8("未指定 OBJ/PLY，保持空场景。\n");
        return std::make_shared<PointCloud>();
    }

    std::string error;
    auto cloud = PointCloudIO::load(fileName, &error);
    if (!cloud) {
        consolePrintUtf8("加载点云失败: " + error + "\n", true);
        return nullptr;
    }

    consolePrintUtf8("已加载点云: " + fileName + "，点数=" + std::to_string(cloud->size()) + "\n");
    return cloud;
}

inline Vec3f add(const Vec3f& a, const Vec3f& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline Vec3f sub(const Vec3f& a, const Vec3f& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline Vec3f mul(const Vec3f& v, float s) {
    return {v.x * s, v.y * s, v.z * s};
}
inline float dot(const Vec3f& a, const Vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vec3f cross(const Vec3f& a, const Vec3f& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float length(const Vec3f& v) {
    return std::sqrt(dot(v, v));
}
inline Vec3f normalize(const Vec3f& v) {
    const float len = length(v);
    if (len <= 1.0e-12f)
        return {0.0f, 0.0f, 0.0f};
    return mul(v, 1.0f / len);
}

// OpenGL 列主序 4x4 矩阵相乘：结果 = a * b。
inline Mat4f multiply(const Mat4f& a, const Mat4f& b) {
    Mat4f out{};
    out.m.fill(0.0f);
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            for (int k = 0; k < 4; ++k) {
                out.m[static_cast<std::size_t>(col) * 4 + row] +=
                    a.m[static_cast<std::size_t>(k) * 4 + row] * b.m[static_cast<std::size_t>(col) * 4 + k];
            }
        }
    }
    return out;
}

inline Mat4f perspective(float fovYRadians, float aspect, float nearPlane, float farPlane) {
    Mat4f m{};
    m.m.fill(0.0f);
    const float f = 1.0f / std::tan(fovYRadians * 0.5f);
    m.m[0] = f / std::max(aspect, 1.0e-6f);
    m.m[5] = f;
    m.m[10] = (farPlane + nearPlane) / (nearPlane - farPlane);
    m.m[11] = -1.0f;
    m.m[14] = (2.0f * farPlane * nearPlane) / (nearPlane - farPlane);
    return m;
}

inline Mat4f lookAt(const Vec3f& eye, const Vec3f& center, const Vec3f& upHint) {
    const Vec3f f = normalize(sub(center, eye));
    const Vec3f s = normalize(cross(f, upHint));
    const Vec3f u = cross(s, f);

    Mat4f m = Mat4f::identity();
    m.m[0] = s.x;
    m.m[1] = u.x;
    m.m[2] = -f.x;
    m.m[4] = s.y;
    m.m[5] = u.y;
    m.m[6] = -f.y;
    m.m[8] = s.z;
    m.m[9] = u.z;
    m.m[10] = -f.z;
    m.m[12] = -dot(s, eye);
    m.m[13] = -dot(u, eye);
    m.m[14] = dot(f, eye);
    return m;
}

struct CloudBounds {
    Vec3f min{};
    Vec3f max{};
    Vec3f center{};
    float radius{1.0f};
    bool valid{false};
};

inline CloudBounds calculateBounds(const PointCloud& cloud) {
    CloudBounds out;
    bool first = true;
    for (const auto& p : cloud.points()) {
        if ((p.flags & PointDeleted) != 0)
            continue;
        if (first) {
            out.min = out.max = p.position;
            first = false;
        } else {
            out.min.x = std::min(out.min.x, p.position.x);
            out.min.y = std::min(out.min.y, p.position.y);
            out.min.z = std::min(out.min.z, p.position.z);
            out.max.x = std::max(out.max.x, p.position.x);
            out.max.y = std::max(out.max.y, p.position.y);
            out.max.z = std::max(out.max.z, p.position.z);
        }
    }
    if (first)
        return out;

    out.valid = true;
    out.center = mul(add(out.min, out.max), 0.5f);
    out.radius = std::max(length(sub(out.max, out.min)) * 0.5f, 1.0e-4f);
    return out;
}

// 轻量 Orbit 相机。
//
// 与很多 CAD/CloudCompare 类软件一致：
// - Orbit 永远围绕模型包围盒中心 target 旋转；
// - 平移不修改 target，而是使用投影空间 panNdcX/panNdcY 做画面平移；
// - 因此用户先平移再旋转时，旋转轴仍然是模型中心轴，不会出现“绕屏幕某一点飘着转”。
//
// 浏览模式：左键旋转、右键/中键平移、滚轮缩放。
struct OrbitCamera {
    // 旋转中心始终是模型包围盒中心。Quaternion 只描述相机相对 target 的姿态，
    // 因此不会因为 pitch 到 90 度附近发生万向节锁，也不需要限制俯仰角。
    Vec3f target{};
    float distance{3.0f};
    float sceneRadius{1.0f};
    float fovYRadians{45.0f * 3.14159265358979323846f / 180.0f};

    // 四元数采用 (w, x, y, z)。初始姿态对应相机位于 target 的 +Z 方向。
    struct Quat {
        float w{1.0f}, x{0.0f}, y{0.0f}, z{0.0f};
    } orientation{};

    // 只做屏幕空间平移，不改变真正的 Orbit target。
    float panNdcX{0.0f};
    float panNdcY{0.0f};

    static Quat normalizeQuat(const Quat& q) {
        const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        if (n <= 1.0e-12f)
            return {};
        return {q.w / n, q.x / n, q.y / n, q.z / n};
    }

    static Quat multiplyQuat(const Quat& a, const Quat& b) {
        return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
    }

    static Quat axisAngle(const Vec3f& axisIn, float angle) {
        const Vec3f axis = normalize(axisIn);
        const float h = angle * 0.5f;
        const float s = std::sin(h);
        return normalizeQuat({std::cos(h), axis.x * s, axis.y * s, axis.z * s});
    }

    static Vec3f rotate(const Quat& qIn, const Vec3f& v) {
        const Quat q = normalizeQuat(qIn);
        const Quat p{0.0f, v.x, v.y, v.z};
        const Quat qi{q.w, -q.x, -q.y, -q.z};
        const Quat r = multiplyQuat(multiplyQuat(q, p), qi);
        return {r.x, r.y, r.z};
    }

    void fit(const PointCloud& cloud) {
        const auto b = calculateBounds(cloud);
        if (!b.valid)
            return;
        target = b.center;
        sceneRadius = b.radius;
        distance = std::max(sceneRadius / std::tan(fovYRadians * 0.5f) * 1.25f, sceneRadius * 1.5f);
        orientation = {};
        panNdcX = 0.0f;
        panNdcY = 0.0f;
    }

    Vec3f eye() const {
        // 相机初始位于 +Z，之后只围绕 target 旋转该 offset。
        return add(target, rotate(orientation, {0.0f, 0.0f, distance}));
    }

    Vec3f up() const {
        // up 也随四元数旋转，所以越过顶部/底部后仍能继续旋转，不会锁死。
        return normalize(rotate(orientation, {0.0f, 1.0f, 0.0f}));
    }

    Vec3f forward() const {
        return normalize(sub(target, eye()));
    }
    Vec3f right() const {
        return normalize(rotate(orientation, {1.0f, 0.0f, 0.0f}));
    }

    Mat4f mvp(int width, int height) const {
        const float aspect = static_cast<float>(std::max(1, width)) / static_cast<float>(std::max(1, height));
        // Live scanning can move the orbit target with the physical camera while a long
        // history remains behind it.  A tight far plane based only on the current target
        // clips valid history when the user rotates the view.  Keep near reasonably small
        // and far deliberately conservative; this is still far tighter than an arbitrary
        // infinite projection and avoids the rotation-time slicing seen in long scans.
        const float safeRadius = std::max(sceneRadius, 1.0f);
        const float nearPlane = std::max(0.01f, std::min(safeRadius * 0.001f, distance * 0.01f));
        const float farPlane = std::max({nearPlane + safeRadius * 50.0f,
                                         distance + safeRadius * 25.0f,
                                         distance * 20.0f});
        const Mat4f pv = multiply(perspective(fovYRadians, aspect, nearPlane, farPlane), lookAt(eye(), target, up()));

        Mat4f pan = Mat4f::identity();
        pan.m[12] = panNdcX;
        pan.m[13] = panNdcY;
        return multiply(pan, pv);
    }

    void orbit(float dxPixels, float dyPixels, int viewportWidth, int viewportHeight) {
        // 1.8.1：真正的“围绕屏幕中心/模型中心”自由 Orbit。
        //
        // 旧实现把左右旋转轴固定为世界 Y={0,1,0}。模型一旦已经倾斜，继续左右拖动时
        // 旋转轴仍然停留在世界 Y，操作手感会突然变得很别扭，尤其难以继续观察模型侧面。
        //
        // 新实现使用“当前相机 Up / Right”作为两条轨道轴：
        // - 水平拖动：绕当前屏幕竖直方向（camera Up）旋转；
        // - 垂直拖动：绕当前屏幕水平方向（camera Right）旋转；
        // - 两条轴都穿过 target，因此旋转中心始终是活动模型包围盒中心；
        // - Quaternion 不做 pitch clamp，可连续越过顶部/底部，不会在某个角度锁死。
        const float w = static_cast<float>(std::max(1, viewportWidth));
        const float h = static_cast<float>(std::max(1, viewportHeight));
        constexpr float pi = 3.14159265358979323846f;

        // 统一为“拖动模型”的视觉习惯：鼠标向右，模型视觉上向右转；鼠标向下，模型向下翻。
        const float yawDelta = -dxPixels * (2.0f * pi / w);
        const float pitchDelta = -dyPixels * (1.5f * pi / h);

        // 注意：必须在应用 yaw 之前取得当前相机 Up。这里绝不能再写死世界 Y。
        const Vec3f currentUp = up();
        const Quat yawQ = axisAngle(currentUp, yawDelta);
        const Quat yawed = normalizeQuat(multiplyQuat(yawQ, orientation));

        // yaw 后重新计算屏幕 Right，再围绕它做 pitch。
        const Vec3f currentRight = normalize(rotate(yawed, {1.0f, 0.0f, 0.0f}));
        const Quat pitchQ = axisAngle(currentRight, pitchDelta);
        orientation = normalizeQuat(multiplyQuat(pitchQ, yawed));
    }

    void orbit(float dxPixels, float dyPixels) {
        orbit(dxPixels, dyPixels, 1000, 1000);
    }

    void pan(float dxPixels, float dyPixels, int viewportWidth, int viewportHeight) {
        panNdcX += 2.0f * dxPixels / static_cast<float>(std::max(1, viewportWidth));
        panNdcY -= 2.0f * dyPixels / static_cast<float>(std::max(1, viewportHeight));
    }

    void pan(float dxPixels, float dyPixels, int viewportHeight) {
        pan(dxPixels, dyPixels, viewportHeight, viewportHeight);
    }

    void zoom(float wheelSteps) {
        distance *= std::pow(0.85f, wheelSteps);
        distance = std::clamp(distance, sceneRadius * 0.02f, sceneRadius * 200.0f);
    }
};

// 保留旧测试所需的正交适配矩阵接口。
inline Mat4f makeFitMvp(const PointCloud& cloud) {
    Mat4f m = Mat4f::identity();
    const auto b = calculateBounds(cloud);
    if (!b.valid)
        return m;

    const Vec3f extent = sub(b.max, b.min);
    const float maxExtent = std::max({extent.x, extent.y, extent.z, 1.0e-6f});
    const float scale = 1.8f / maxExtent;
    m.m = {
        scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, scale, 0, -b.center.x * scale, -b.center.y * scale, -b.center.z * scale,
        1};
    return m;
}

} // namespace pceditor::example
