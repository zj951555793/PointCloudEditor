#pragma once

#include "PointCloud.h"
#include "TriangleMesh.h"

#include <memory>
#include <string>

namespace JMEngine {

class ModelIO {
  public:
    // TXT / ASC 支持空格、Tab、逗号、分号分隔：XYZ、XYZI、XYZRGB、XYZRGBNormal。
    // 4 列 XYZI 会忽略 intensity（PointCloud 当前无 intensity 字段），不会误当颜色。
    static std::shared_ptr<PointCloud> loadTxt(const std::string& fileName, std::string* errorMessage = nullptr);
    static std::shared_ptr<PointCloud> loadAsc(const std::string& fileName, std::string* errorMessage = nullptr);

    // 根据扩展名导出。PLY/ASC 可导出点云；OBJ/STL 需要 mesh。
    static bool save(const PointCloud& cloud, const TriangleMesh* mesh, const std::string& fileName,
                     std::string* errorMessage = nullptr);
    static bool saveObj(const PointCloud& cloud, const TriangleMesh& mesh, const std::string& fileName,
                        std::string* errorMessage = nullptr);
    static bool saveStl(const PointCloud& cloud, const TriangleMesh& mesh, const std::string& fileName,
                        std::string* errorMessage = nullptr);
    static bool savePly(const PointCloud& cloud, const TriangleMesh* mesh, const std::string& fileName,
                        std::string* errorMessage = nullptr);
    // ASC 固定输出 XYZRGB（ASCII，空格分隔），仅输出未删除且坐标有限的点。
    static bool saveAsc(const PointCloud& cloud, const std::string& fileName, std::string* errorMessage = nullptr);
};

} // namespace JMEngine
