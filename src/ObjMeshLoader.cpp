#include "pceditor/ObjMeshLoader.h"
#include "pceditor/ObjModelLoader.h"

namespace pceditor {

bool ObjMeshLoader::load(const std::string& objFile, std::size_t vertexCount, ObjMeshData& out, std::string* message) {
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
    out = std::move(model.mesh);
    if (message)
        *message = std::move(local);
    return !out.empty();
}

} // namespace pceditor
