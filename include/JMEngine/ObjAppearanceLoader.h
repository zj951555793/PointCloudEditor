#pragma once

#include "PointCloud.h"

#include <cstdint>
#include <string>
#include <vector>

namespace JMEngine {

// OBJ 外观信息。
// Core 只负责 OBJ/MTL 文件解析，不负责 PNG/JPG 图片解码，因此库保持零 Qt 依赖。
// 上层可以使用 Qt/QImage、stb_image、WIC 等任意图片解码器读取 diffuseTexturePath，
// 再根据 vertexUv 将纹理颜色烘焙到 PointCloud::rgba。
struct ObjAppearanceData {
    std::vector<Vec2f> vertexUv;
    std::vector<std::uint8_t> hasUv;
    std::vector<Vec3f> vertexNormals;
    std::string diffuseTexturePath;

    bool hasTextureCoordinates() const noexcept;
    bool hasNormals() const noexcept;
};

// OBJ 的 UV / 法向 / MTL(map_Kd) 加载器。
// 不使用 Qt。OpenMP 可用于大模型法向归一化等 CPU 阶段。
class ObjAppearanceLoader {
  public:
    static bool load(const std::string& objFile, std::size_t vertexCount, ObjAppearanceData& out,
                     std::string* message = nullptr);

    // 把解析得到的顶点法向写回点云。
    // 顶点数量一致时可直接调用；启用 OpenMP 后并行处理。
    static void applyNormals(PointCloud& cloud, const ObjAppearanceData& appearance);
};

} // namespace JMEngine
