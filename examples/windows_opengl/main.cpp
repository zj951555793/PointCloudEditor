#ifdef _WIN32

// Windows 原生 Win32 + WGL 测试。
// 本示例故意只依赖系统 opengl32.lib，不依赖 GLFW/GLEW/GLAD。
// 因为 Windows 自带 OpenGL 头只直接暴露到 1.1，所以显示使用兼容模式立即绘制，
// 1.8.1：框选改为 OpenGL 2.1 兼容 RGB24 Color Picking。
// 不使用 R32UI / GLSL 3.3 / FBO；直接在后缓冲执行隐藏 ID Pass，再用 glReadPixels 读回。

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <windowsx.h>
#include <GL/gl.h>

#include "../common/ExampleUtils.h"

#include <pceditor/ColorPicking24.h>
#include <pceditor/PointCloudEditor.h>
#include <pceditor/PointCloudIO.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

HWND g_hwnd = nullptr;
HDC g_dc = nullptr;
HGLRC g_glrc = nullptr;

std::shared_ptr<pceditor::PointCloud> g_cloud;
std::unique_ptr<pceditor::PointCloudEditor> g_editor;
pceditor::Mat4f g_mvp = pceditor::Mat4f::identity();

int g_width = 1000;
int g_height = 700;
bool g_dragging = false;
POINT g_dragBegin{0, 0};
POINT g_dragEnd{0, 0};

// Windows OpenGL 2.1 示例的高亮选择。
// 每个点 1 byte；1000 万点约 10 MB，可接受，且高亮判断 O(1)。
std::vector<std::uint8_t> g_selectedMask;
std::vector<pceditor::PointId> g_selectedIds;

// 从打包的 RGBA 中解出一个颜色通道。
float colorByte(std::uint32_t rgba, unsigned shift) {
    return static_cast<float>((rgba >> shift) & 0xffu) / 255.0f;
}

bool createWglContext(HWND hwnd) {
    g_dc = GetDC(hwnd);
    if (!g_dc)
        return false;

    PIXELFORMATDESCRIPTOR pfd{};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    pfd.iLayerType = PFD_MAIN_PLANE;

    const int format = ChoosePixelFormat(g_dc, &pfd);
    if (format == 0 || !SetPixelFormat(g_dc, format, &pfd))
        return false;

    g_glrc = wglCreateContext(g_dc);
    if (!g_glrc || !wglMakeCurrent(g_dc, g_glrc))
        return false;

    glEnable(GL_DEPTH_TEST);
    glPointSize(3.0f);
    const auto* version = reinterpret_cast<const char*>(glGetString(GL_VERSION));
    std::cout << "OpenGL Version: " << (version ? version : "unknown") << '\n'
              << "Picking Backend: OpenGL 2.1 compatible RGB24 Color Picking\n";
    return true;
}

void destroyWglContext() {
    if (g_glrc) {
        wglMakeCurrent(nullptr, nullptr);
        wglDeleteContext(g_glrc);
        g_glrc = nullptr;
    }
    if (g_dc && g_hwnd) {
        ReleaseDC(g_hwnd, g_dc);
        g_dc = nullptr;
    }
}

void drawSelectionRectangle() {
    if (!g_dragging || g_width <= 0 || g_height <= 0)
        return;

    auto toNdcX = [](LONG x) { return static_cast<float>(x) / static_cast<float>(g_width) * 2.0f - 1.0f; };
    auto toNdcY = [](LONG y) { return 1.0f - static_cast<float>(y) / static_cast<float>(g_height) * 2.0f; };

    // 临时关闭深度测试，用屏幕空间画黄色框选矩形。
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glColor3f(1.0f, 1.0f, 0.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(toNdcX(g_dragBegin.x), toNdcY(g_dragBegin.y));
    glVertex2f(toNdcX(g_dragEnd.x), toNdcY(g_dragBegin.y));
    glVertex2f(toNdcX(g_dragEnd.x), toNdcY(g_dragEnd.y));
    glVertex2f(toNdcX(g_dragBegin.x), toNdcY(g_dragEnd.y));
    glEnd();

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glEnable(GL_DEPTH_TEST);
}

void render() {
    if (!g_dc)
        return;

    glViewport(0, 0, g_width, g_height);
    glClearColor(0.06f, 0.07f, 0.09f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(g_mvp.m.data());

    if (g_cloud) {
        glBegin(GL_POINTS);
        for (const auto& p : g_cloud->points()) {
            if ((p.flags & pceditor::PointDeleted) != 0)
                continue;
            const std::size_t id = static_cast<std::size_t>(&p - g_cloud->points().data());
            if (id < g_selectedMask.size() && g_selectedMask[id]) {
                glColor3f(1.0f, 0.82f, 0.08f);
            } else {
                glColor4f(colorByte(p.rgba, 0), colorByte(p.rgba, 8), colorByte(p.rgba, 16), colorByte(p.rgba, 24));
            }
            glVertex3f(p.position.x, p.position.y, p.position.z);
        }
        glEnd();
    }

    drawSelectionRectangle();
    SwapBuffers(g_dc);
}

// 执行一次 OpenGL 2.1 兼容 Color Picking。
//
// 注意：这里故意不依赖 FBO。OpenGL 2.1 本身不保证核心 FBO API，而 Windows 系统 gl.h
// 只直接声明到 OpenGL 1.1。为了让示例在老驱动/兼容上下文中也能开箱验证 GPU Picking，
// 我们把 ID 颜色画到“当前后缓冲”，读取完成后下一次正常 render() 会立刻覆盖它，用户看不到 ID 图。
std::vector<pceditor::PointId> gpuPickRectangleOpenGL21() {
    std::vector<pceditor::PointId> out;
    if (!g_cloud || !g_dc || g_width <= 0 || g_height <= 0)
        return out;

    const std::size_t pointCount = g_cloud->size();
    if (pointCount == 0)
        return out;

    // RGB24 中 0 保留为背景，所以有效对象 id 最大为 0xFFFFFE。
    if (pointCount - 1u > static_cast<std::size_t>(pceditor::kColorPicking24MaxObjectId)) {
        std::cerr << "OpenGL 2.1 RGB24 Picking: 当前点数超过单 Pass 24-bit ID 上限，"
                     "请使用 Block Picking。pointCount="
                  << pointCount << '\n';
        return out;
    }

    const LONG x0 = std::max<LONG>(0, std::min(g_dragBegin.x, g_dragEnd.x));
    const LONG x1 = std::min<LONG>(g_width - 1, std::max(g_dragBegin.x, g_dragEnd.x));
    const LONG y0Top = std::max<LONG>(0, std::min(g_dragBegin.y, g_dragEnd.y));
    const LONG y1Top = std::min<LONG>(g_height - 1, std::max(g_dragBegin.y, g_dragEnd.y));
    const int readW = static_cast<int>(x1 - x0 + 1);
    const int readH = static_cast<int>(y1Top - y0Top + 1);
    if (readW <= 0 || readH <= 0)
        return out;

    // Windows 鼠标坐标原点在左上，OpenGL glReadPixels 原点在左下。
    const int readX = static_cast<int>(x0);
    const int readY = g_height - 1 - static_cast<int>(y1Top);

    // 保存并关闭可能改变 ID 颜色的固定管线状态。
    const GLboolean depthWas = glIsEnabled(GL_DEPTH_TEST);
    const GLboolean blendWas = glIsEnabled(GL_BLEND);
    const GLboolean ditherWas = glIsEnabled(GL_DITHER);
    const GLboolean textureWas = glIsEnabled(GL_TEXTURE_2D);
    const GLboolean lightingWas = glIsEnabled(GL_LIGHTING);

    glEnable(GL_DEPTH_TEST); // 表面选择：只保留当前视角最前面的点。
    glDisable(GL_BLEND);
    glDisable(GL_DITHER); // Color Picking 必须关抖动，否则 RGB 可能被改写。
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);

    glViewport(0, 0, g_width, g_height);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // RGB=0 表示背景。
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(g_mvp.m.data());

    glPointSize(3.0f);
    glBegin(GL_POINTS);
    const auto& points = g_cloud->points();
    for (std::uint32_t id = 0; id < static_cast<std::uint32_t>(points.size()); ++id) {
        const auto& p = points[id];
        if ((p.flags & pceditor::PointDeleted) != 0)
            continue;
        const auto c = pceditor::encodeColorId24(id);
        glColor3ub(c.r, c.g, c.b);
        glVertex3f(p.position.x, p.position.y, p.position.z);
    }
    glEnd();
    glFlush();

    // 只读框选区域，而不是整屏，降低 CPU/GPU 同步和内存带宽。
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(readW) * static_cast<std::size_t>(readH) * 3u);
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(readX, readY, readW, readH, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    // 恢复状态。
    if (depthWas)
        glEnable(GL_DEPTH_TEST);
    else
        glDisable(GL_DEPTH_TEST);
    if (blendWas)
        glEnable(GL_BLEND);
    else
        glDisable(GL_BLEND);
    if (ditherWas)
        glEnable(GL_DITHER);
    else
        glDisable(GL_DITHER);
    if (textureWas)
        glEnable(GL_TEXTURE_2D);
    else
        glDisable(GL_TEXTURE_2D);
    if (lightingWas)
        glEnable(GL_LIGHTING);
    else
        glDisable(GL_LIGHTING);

    // 用 byte mask 去重，比 unordered_set 更省内存、更稳定。
    std::vector<std::uint8_t> hit(pointCount, 0);
    for (std::size_t i = 0; i + 2 < pixels.size(); i += 3) {
        std::uint32_t id = 0;
        if (!pceditor::decodeColorId24(pixels[i], pixels[i + 1], pixels[i + 2], id))
            continue;
        if (id < pointCount)
            hit[id] = 1;
    }

    for (std::uint32_t id = 0; id < static_cast<std::uint32_t>(pointCount); ++id) {
        if (hit[id])
            out.push_back(id);
    }
    return out;
}

void selectRectangleGpu() {
    if (!g_cloud || !g_editor)
        return;
    auto ids = gpuPickRectangleOpenGL21();
    g_editor->select(ids);
    g_selectedIds = std::move(ids);
    g_selectedMask.assign(g_cloud->size(), 0);
    for (const auto id : g_selectedIds) {
        if (id < g_selectedMask.size())
            g_selectedMask[id] = 1;
    }
    std::cout << "OpenGL 2.1 RGB24 GPU 选择完成，选中=" << g_selectedIds.size() << '\n';
}

LRESULT CALLBACK windowProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_SIZE:
        // LOWORD/HIWORD 返回 WORD(unsigned short)，直接与 int 传给 std::max 会导致 MSVC 模板推导失败。
        g_width = std::max(1, static_cast<int>(LOWORD(lParam)));
        g_height = std::max(1, static_cast<int>(HIWORD(lParam)));
        return 0;

    case WM_LBUTTONDOWN:
        g_dragging = true;
        g_dragBegin = {GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam)};
        g_dragEnd = g_dragBegin;
        SetCapture(hwnd);
        InvalidateRect(hwnd, nullptr, FALSE);
        return 0;

    case WM_MOUSEMOVE:
        if (g_dragging) {
            g_dragEnd = {GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam)};
            InvalidateRect(hwnd, nullptr, FALSE);
        }
        return 0;

    case WM_LBUTTONUP:
        if (g_dragging) {
            g_dragEnd = {GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam)};
            g_dragging = false;
            ReleaseCapture();
            selectRectangleGpu();
            InvalidateRect(hwnd, nullptr, FALSE);
        }
        return 0;

    case WM_KEYDOWN:
        if (!g_editor || !g_cloud)
            return 0;
        if (wParam == VK_DELETE) {
            if (!g_selectedIds.empty()) {
                g_editor->select(g_selectedIds);
                g_editor->deleteSelection();
                g_selectedIds.clear();
                g_selectedMask.assign(g_cloud->size(), 0);
            }
        } else if (wParam == 'Z') {
            g_editor->undo();
        } else if (wParam == 'Y') {
            g_editor->redo();
        } else if (wParam == 'S') {
            std::string error;
            if (!pceditor::PointCloudIO::savePly(*g_cloud, "edited_output.ply", &error))
                std::cerr << "保存失败: " << error << '\n';
            else
                std::cout << "已保存 edited_output.ply\n";
        }
        InvalidateRect(hwnd, nullptr, FALSE);
        return 0;

    case WM_PAINT: {
        PAINTSTRUCT ps{};
        BeginPaint(hwnd, &ps);
        render();
        EndPaint(hwnd, &ps);
        return 0;
    }

    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProc(hwnd, msg, wParam, lParam);
}

} // namespace

int main(int argc, char* argv[]) {
    // 使用 Console 子系统而不是 WIN32 子系统：
    // 1. VS/CMake 链接时入口就是标准 main，不再受 WinMain/wWinMain 配置影响；
    // 2. 原生 OpenGL 示例可以直接看到测试日志；
    // 3. enableUtf8Console + consolePrintUtf8 负责 Windows 中文输出。
    pceditor::example::enableUtf8Console();
    const std::string fileName = argc > 1 ? std::string(argv[1]) : std::string{};

    g_cloud = pceditor::example::loadCloudOrEmpty(fileName);
    if (!g_cloud)
        return 2;
    g_editor = std::make_unique<pceditor::PointCloudEditor>(g_cloud);
    g_selectedMask.assign(g_cloud->size(), 0);
    g_mvp = pceditor::example::makeFitMvp(*g_cloud);

    HINSTANCE instance = GetModuleHandleW(nullptr);
    WNDCLASSW wc{};
    wc.style = CS_OWNDC;
    wc.lpfnWndProc = windowProc;
    wc.hInstance = instance;
    wc.lpszClassName = L"PointCloudEditorWglTest";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    if (!RegisterClassW(&wc) && GetLastError() != ERROR_CLASS_ALREADY_EXISTS)
        return 3;

    g_hwnd = CreateWindowExW(0, wc.lpszClassName, L"PointCloudEditor - Windows OpenGL Test",
                             WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT, CW_USEDEFAULT, g_width, g_height, nullptr,
                             nullptr, instance, nullptr);
    if (!g_hwnd)
        return 4;

    if (!createWglContext(g_hwnd)) {
        MessageBoxW(g_hwnd, L"Create WGL Context failed", L"PointCloudEditor", MB_ICONERROR);
        return 5;
    }

    ShowWindow(g_hwnd, SW_SHOWDEFAULT);
    UpdateWindow(g_hwnd);

    pceditor::example::consolePrintUtf8("Windows WGL 示例已启动：OpenGL 2.1 兼容 RGB24 GPU "
                                        "Picking。左键框选只高亮，Delete 删除，Z/Y 撤销重做，S 保存。\n");

    MSG msg{};
    while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }

    destroyWglContext();
    return static_cast<int>(msg.wParam);
}

#endif // _WIN32
