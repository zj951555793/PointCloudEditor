#pragma once

#include "PointCloud.h"

#include <memory>
#include <string>

namespace pceditor {

// 点云文件读写工具。
//
// 当前版本刻意保持轻量，不依赖 PCL / VTK / Assimp：
// 1. OBJ：读取所有 "v x y z [r g b]" 顶点行，忽略面、纹理和法线。
// 2. PLY：支持 ASCII 与 binary_little_endian PLY 的 vertex 元素，至少要求 x/y/z；可选 red/green/blue/alpha。
//
// 这足以用于点云编辑器的导入测试。如果后续需要 binary_big_endian PLY、带面 OBJ、法线等，
// 建议继续扩展本类，而不要把第三方格式库耦合到 PointCloudEditor 核心编辑逻辑。
class PointCloudIO {
  public:
    // 根据文件扩展名自动选择 OBJ / PLY 解析器。
    // 失败时返回 nullptr；errorMessage 非空时会写入可读错误原因。
    static std::shared_ptr<PointCloud> load(const std::string& fileName, std::string* errorMessage = nullptr);

    // 读取 OBJ 顶点。
    static std::shared_ptr<PointCloud> loadObj(const std::string& fileName, std::string* errorMessage = nullptr);

    // 读取 ASCII / binary_little_endian PLY 顶点。
    static std::shared_ptr<PointCloud> loadPly(const std::string& fileName, std::string* errorMessage = nullptr);

    // 保存为 ASCII PLY。
    // 仅输出未删除点，便于直接验证编辑结果。
    static bool savePly(const PointCloud& cloud, const std::string& fileName, std::string* errorMessage = nullptr);
};

} // namespace pceditor
