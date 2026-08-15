#include "pceditor/ObjAppearanceLoader.h"
#include "pceditor/ObjModelLoader.h"
#include <pceditor/processing/Parallel.h>

#include <algorithm>

namespace pceditor {

bool ObjAppearanceLoader::load(const std::string& objFile, std::size_t vertexCount, ObjAppearanceData& out,
                               std::string* message) {
    ObjModelData model;
    std::string local;
    if (!ObjModelLoader::load(objFile, model, &local)) {
        out = {};
        if (message)
            *message = local;
        return false;
    }
    if (model.cloud && model.cloud->size() != vertexCount) {
        out = {};
        if (message)
            *message = "OBJ 顶点数量与调用方 PointCloud 不一致";
        return false;
    }
    out = std::move(model.appearance);
    if (message)
        *message = std::move(local);
    return out.hasNormals() || out.hasTextureCoordinates() || !out.diffuseTexturePath.empty();
}

void ObjAppearanceLoader::applyNormals(PointCloud& cloud, const ObjAppearanceData& appearance) {
    const std::size_t count = std::min(cloud.size(), appearance.vertexNormals.size());
#ifdef PCEDITOR_USE_OPENMP
#pragma omp parallel for schedule(static) num_threads(processing::processingThreadCount())
#endif
    for (std::int64_t ii = 0; ii < static_cast<std::int64_t>(count); ++ii) {
        const std::size_t i = static_cast<std::size_t>(ii);
        cloud.points()[i].normal = appearance.vertexNormals[i];
    }
}

} // namespace pceditor
