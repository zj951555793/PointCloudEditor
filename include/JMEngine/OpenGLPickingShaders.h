#pragma once

namespace JMEngine::shaders {

// 这里仅保存最小 GPU Picking Shader 文本，不直接包含任何 OpenGL/GLES 头文件。
// 因此 JMEngine Core 本身仍然与图形 API 解耦。
//
// 顶点属性约定：
// location 0 -> vec3 position
// location 2 -> uint PointId
//
// 渲染目标必须是整数颜色附件，例如 GL_R32UI。
// 背景建议清成 0xffffffff(kInvalidPointId)，之后 PixelIdPicker 会自动过滤。

// 桌面 OpenGL 3.3 Core 顶点 Shader。
inline constexpr const char* kDesktopVertex330 = R"GLSL(#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 2) in uint aPointId;
uniform mat4 uMVP;
flat out uint vPointId;
void main() {
    gl_Position = uMVP * vec4(aPosition, 1.0);
    gl_PointSize = 4.0;
    vPointId = aPointId;
}
)GLSL";

// 桌面 OpenGL 3.3 Core 整数颜色输出 Shader。
inline constexpr const char* kDesktopFragment330 = R"GLSL(#version 330 core
flat in uint vPointId;
layout(location = 0) out uint outId;
void main() {
    outId = vPointId;
}
)GLSL";

// OpenGL ES 3.0 顶点 Shader，可直接用于 RK3588 GLES3。
inline constexpr const char* kGlesVertex300 = R"GLSL(#version 300 es
precision highp float;
precision highp int;
layout(location = 0) in vec3 aPosition;
layout(location = 2) in uint aPointId;
uniform mat4 uMVP;
flat out highp uint vPointId;
void main() {
    gl_Position = uMVP * vec4(aPosition, 1.0);
    gl_PointSize = 4.0;
    vPointId = aPointId;
}
)GLSL";

// OpenGL ES 3.0 整数颜色输出 Shader。
inline constexpr const char* kGlesFragment300 = R"GLSL(#version 300 es
precision highp float;
precision highp int;
flat in highp uint vPointId;
layout(location = 0) out highp uint outId;
void main() {
    outId = vPointId;
}
)GLSL";

} // namespace JMEngine::shaders
