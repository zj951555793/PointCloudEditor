#pragma once

#include "ObjAppearanceLoader.h"
#include "ObjMeshLoader.h"
#include "PointCloud.h"

#include <memory>
#include <string>

namespace JMEngine {

// 一次性 OBJ 加载结果。
// Qt/OpenGL 示例应该优先调用本类，而不是分别读三次 OBJ 文件。
struct ObjModelData {
    std::shared_ptr<PointCloud> cloud;
    ObjMeshData mesh;
    ObjAppearanceData appearance;
};

// 高性能 OBJ 模型加载器。
// - OBJ 文件只读入内存一次；
// - 顶点、UV、法向、Face 数字解析使用快速 C 数值解析；
// - OpenMP 可并行顶点/UV/法向/Face 解析；
// - 输出点云 + 三角网格 + 外观信息；
// - 完全不依赖 Qt。
class ObjModelLoader {
  public:
    static bool load(const std::string& objFile, ObjModelData& out, std::string* message = nullptr);
};

} // namespace JMEngine
