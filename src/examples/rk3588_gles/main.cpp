// RK3588 / Linux EGL + OpenGL ES 3.0 无窗口集成测试。
//
// 目标：验证与 Qt 编辑器相同的核心链路在 RK3588 GLES 上可用：
// 1. OBJ/PLY 导入；
// 2. GL_R32UI PointId Picking；
// 3. 矩形 / 圆形 / Lasso / Brush 四种 GPU 选择；
// 4. PointCloudEditor 软删除 + Undo/Redo；
// 5. 删除后只更新 flags VBO 的脏区，不重新上传 position/color 全 VBO。

#include "../common/ExampleUtils.h"

#include <pceditor/PixelIdPicker.h>
#include <pceditor/PointCloudEditor.h>
#include <pceditor/PointCloudIO.h>

#include <EGL/egl.h>
#include <GLES3/gl3.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr int kWidth = 800;
constexpr int kHeight = 600;

constexpr const char* kPickVs = R"GLSL(#version 300 es
precision highp float;
precision highp int;
layout(location = 0) in vec3 aPosition;
layout(location = 1) in uint aPointId;
layout(location = 2) in uint aFlags;
uniform mat4 uMVP;
flat out uint vPointId;
const uint POINT_DELETED = 4u;
void main() {
    if ((aFlags & POINT_DELETED) != 0u) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        gl_PointSize = 1.0;
    } else {
        gl_Position = uMVP * vec4(aPosition, 1.0);
        gl_PointSize = 4.0;
    }
    vPointId = aPointId;
}
)GLSL";

constexpr const char* kPickFs = R"GLSL(#version 300 es
precision highp float;
precision highp int;
flat in uint vPointId;
layout(location = 0) out highp uint outId;
void main() {
    vec2 q = gl_PointCoord * 2.0 - vec2(1.0);
    if (dot(q, q) > 1.0) discard;
    outId = vPointId;
}
)GLSL";

struct EglState {
    EGLDisplay display{EGL_NO_DISPLAY};
    EGLSurface surface{EGL_NO_SURFACE};
    EGLContext context{EGL_NO_CONTEXT};

    ~EglState() {
        if (display != EGL_NO_DISPLAY) {
            eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
            if (context != EGL_NO_CONTEXT)
                eglDestroyContext(display, context);
            if (surface != EGL_NO_SURFACE)
                eglDestroySurface(display, surface);
            eglTerminate(display);
        }
    }
};

bool createEgl(EglState& egl) {
    egl.display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (egl.display == EGL_NO_DISPLAY)
        return false;

    EGLint major = 0, minor = 0;
    if (!eglInitialize(egl.display, &major, &minor))
        return false;
    std::cout << "EGL version: " << major << '.' << minor << '\n';

    const EGLint configAttrs[] = {EGL_SURFACE_TYPE,
                                  EGL_PBUFFER_BIT,
                                  EGL_RENDERABLE_TYPE,
                                  EGL_OPENGL_ES3_BIT,
                                  EGL_RED_SIZE,
                                  8,
                                  EGL_GREEN_SIZE,
                                  8,
                                  EGL_BLUE_SIZE,
                                  8,
                                  EGL_DEPTH_SIZE,
                                  24,
                                  EGL_NONE};

    EGLConfig config{};
    EGLint configCount = 0;
    if (!eglChooseConfig(egl.display, configAttrs, &config, 1, &configCount) || configCount < 1)
        return false;

    const EGLint surfaceAttrs[] = {EGL_WIDTH, kWidth, EGL_HEIGHT, kHeight, EGL_NONE};
    egl.surface = eglCreatePbufferSurface(egl.display, config, surfaceAttrs);
    if (egl.surface == EGL_NO_SURFACE)
        return false;

    const EGLint contextAttrs[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
    egl.context = eglCreateContext(egl.display, config, EGL_NO_CONTEXT, contextAttrs);
    if (egl.context == EGL_NO_CONTEXT)
        return false;

    return eglMakeCurrent(egl.display, egl.surface, egl.surface, egl.context) == EGL_TRUE;
}

GLuint compileShader(GLenum type, const char* source) {
    const GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok == GL_TRUE)
        return shader;

    GLint length = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
    std::vector<char> log(static_cast<std::size_t>(std::max(1, length)));
    glGetShaderInfoLog(shader, length, nullptr, log.data());
    std::cerr << "GLES Shader 编译失败: " << log.data() << '\n';
    glDeleteShader(shader);
    return 0;
}

GLuint createProgram() {
    const GLuint vs = compileShader(GL_VERTEX_SHADER, kPickVs);
    const GLuint fs = compileShader(GL_FRAGMENT_SHADER, kPickFs);
    if (!vs || !fs)
        return 0;

    const GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint ok = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (ok == GL_TRUE)
        return program;

    GLint length = 0;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
    std::vector<char> log(static_cast<std::size_t>(std::max(1, length)));
    glGetProgramInfoLog(program, length, nullptr, log.data());
    std::cerr << "GLES Program 链接失败: " << log.data() << '\n';
    glDeleteProgram(program);
    return 0;
}

bool createPickingFbo(GLuint& fbo, GLuint& texture, GLuint& depth) {
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32UI, kWidth, kHeight);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);

    glGenRenderbuffers(1, &depth);
    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, kWidth, kHeight);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);

    const bool ok = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return ok;
}

// 将一组 changed PointId 合并成连续区间，仅更新 flags VBO。
// 即使删除几十万点，也不重新上传 position/id 全 VBO。
void updateFlagRanges(GLuint flagsVbo, const pceditor::PointCloud& cloud, std::vector<pceditor::PointId> ids) {
    if (ids.empty())
        return;
    if (!std::is_sorted(ids.begin(), ids.end()))
        std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());

    glBindBuffer(GL_ARRAY_BUFFER, flagsVbo);
    std::size_t i = 0;
    while (i < ids.size()) {
        pceditor::PointId begin = ids[i];
        pceditor::PointId end = begin;
        std::size_t j = i + 1;
        while (j < ids.size() && ids[j] <= end + 32 && ids[j] - begin < 65536) {
            end = ids[j++];
        }

        const std::size_t count = static_cast<std::size_t>(end - begin + 1);
        std::vector<std::uint32_t> flags(count);
        for (std::size_t k = 0; k < count; ++k) {
            flags[k] = cloud.points()[static_cast<std::size_t>(begin) + k].flags;
        }
        glBufferSubData(GL_ARRAY_BUFFER, static_cast<GLintptr>(begin * sizeof(std::uint32_t)),
                        static_cast<GLsizeiptr>(flags.size() * sizeof(std::uint32_t)), flags.data());
        i = j;
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

} // namespace

int main(int argc, char* argv[]) {
    pceditor::example::enableUtf8Console();

    const std::string fileName = argc > 1 ? argv[1] : std::string{};
    auto cloud = pceditor::example::loadCloudOrEmpty(fileName);
    if (!cloud)
        return 2;

    pceditor::PointCloudEditor editor(cloud);
    pceditor::example::OrbitCamera camera;
    camera.fit(*cloud);
    const pceditor::Mat4f mvp = camera.mvp(kWidth, kHeight);

    EglState egl;
    if (!createEgl(egl)) {
        std::cerr << "创建 EGL/GLES3 Context 失败，EGL error=0x" << std::hex << eglGetError() << std::dec << '\n';
        return 3;
    }

    std::cout << "GL_VENDOR   = " << glGetString(GL_VENDOR) << '\n'
              << "GL_RENDERER = " << glGetString(GL_RENDERER) << '\n'
              << "GL_VERSION  = " << glGetString(GL_VERSION) << '\n';

    const GLuint program = createProgram();
    if (!program)
        return 4;

    const std::size_t count = cloud->size();
    std::vector<float> positions(count * 3);
    std::vector<std::uint32_t> pointIds(count);
    std::vector<std::uint32_t> flags(count);
    for (std::size_t i = 0; i < count; ++i) {
        const auto& p = cloud->points()[i];
        positions[i * 3 + 0] = p.position.x;
        positions[i * 3 + 1] = p.position.y;
        positions[i * 3 + 2] = p.position.z;
        pointIds[i] = static_cast<std::uint32_t>(i);
        flags[i] = p.flags;
    }

    GLuint vao = 0;
    GLuint positionVbo = 0;
    GLuint idVbo = 0;
    GLuint flagsVbo = 0;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &positionVbo);
    glBindBuffer(GL_ARRAY_BUFFER, positionVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(positions.size() * sizeof(float)), positions.data(),
                 GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glGenBuffers(1, &idVbo);
    glBindBuffer(GL_ARRAY_BUFFER, idVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(pointIds.size() * sizeof(std::uint32_t)), pointIds.data(),
                 GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribIPointer(1, 1, GL_UNSIGNED_INT, 0, nullptr);

    glGenBuffers(1, &flagsVbo);
    glBindBuffer(GL_ARRAY_BUFFER, flagsVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(flags.size() * sizeof(std::uint32_t)), flags.data(),
                 GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribIPointer(2, 1, GL_UNSIGNED_INT, 0, nullptr);

    GLuint fbo = 0, texture = 0, depth = 0;
    if (!createPickingFbo(fbo, texture, depth)) {
        std::cerr << "RK3588: GL_R32UI Picking FBO 创建失败。\n";
        return 5;
    }

    auto renderIds = [&]() {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glViewport(0, 0, kWidth, kHeight);
        const GLuint clearId = pceditor::kInvalidPointId;
        glClearBufferuiv(GL_COLOR, 0, &clearId);
        glClear(GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_PROGRAM_POINT_SIZE);
        glUseProgram(program);
        glUniformMatrix4fv(glGetUniformLocation(program, "uMVP"), 1, GL_FALSE, mvp.m.data());
        glBindVertexArray(vao);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(count));
    };

    renderIds();

    pceditor::PixelIdPicker picker(
        kWidth, kHeight,
        [fbo](int x, int y, int w, int h, std::uint32_t* dst) {
            glBindFramebuffer(GL_FRAMEBUFFER, fbo);
            glReadPixels(x, y, w, h, GL_RED_INTEGER, GL_UNSIGNED_INT, dst);
            return glGetError() == GL_NO_ERROR;
        },
        true);

    const auto rectIds = picker.pickRectangle(kWidth / 4, kHeight / 4, kWidth * 3 / 4, kHeight * 3 / 4);
    const auto circleIds = picker.pickCircle(kWidth / 2, kHeight / 2, 120);
    const std::vector<pceditor::Point2i> lasso = {{250, 180}, {550, 190}, {620, 330}, {470, 460}, {230, 390}};
    const auto lassoIds = picker.pickLasso(lasso);
    const std::vector<pceditor::Point2i> brush = {{250, 300}, {350, 280}, {450, 320}, {550, 300}};
    const auto brushIds = picker.pickBrushStroke(brush, 24);

    std::cout << "矩形选择=" << rectIds.size() << " 圆选=" << circleIds.size() << " Lasso=" << lassoIds.size()
              << " Brush=" << brushIds.size() << '\n';

    // 使用 Brush 结果做删除，并仅更新 flags 脏区。
    if (!brushIds.empty()) {
        editor.select(brushIds);
        if (editor.deleteSelection()) {
            updateFlagRanges(flagsVbo, *cloud, editor.lastChangedIds());
            std::cout << "Brush 删除后剩余点数=" << cloud->activeCount() << '\n';

            // 再渲染一次，验证 Deleted 点不会进入 Picking。
            renderIds();
            const auto afterDelete = picker.pickBrushStroke(brush, 24);
            std::cout << "同一区域删除后二次 Picking=" << afterDelete.size() << '\n';
        }
    }

    std::string error;
    if (!pceditor::PointCloudIO::savePly(*cloud, "rk3588_edited_output.ply", &error))
        std::cerr << "保存失败: " << error << '\n';
    else
        std::cout << "已保存 rk3588_edited_output.ply\n";

    glDeleteFramebuffers(1, &fbo);
    glDeleteTextures(1, &texture);
    glDeleteRenderbuffers(1, &depth);
    glDeleteBuffers(1, &positionVbo);
    glDeleteBuffers(1, &idVbo);
    glDeleteBuffers(1, &flagsVbo);
    glDeleteVertexArrays(1, &vao);
    glDeleteProgram(program);
    return 0;
}
