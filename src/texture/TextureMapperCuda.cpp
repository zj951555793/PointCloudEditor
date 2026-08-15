#include "TextureMapperCuda.h"
#include "TextureMapperCudaApi.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <vector>

namespace pceditor::texture::detail {
#ifdef PCEDITOR_TEXTURE_HAS_CUDA
namespace {

std::string apiError(const char* text) {
    return text && *text ? std::string(text) : std::string("unknown CUDA error");
}

} // namespace
#endif

bool cudaRuntimeAvailable(std::string* reason) noexcept {
#ifdef PCEDITOR_TEXTURE_HAS_CUDA
    std::array<char, 1024> error{};
    const bool ok = cudaApiRuntimeAvailable(error.data(), error.size());
    if (reason) {
        if (ok) reason->clear();
        else *reason = apiError(error.data());
    }
    return ok;
#else
    if (reason) *reason = "CUDA backend was not compiled";
    return false;
#endif
}

bool selectBestCamerasCuda(const TriangleMesh& mesh,
                           const std::vector<CameraFrame>& cameras,
                           const Config& config,
                           std::vector<float>& packedDepth,
                           int depthWidth,
                           int depthHeight,
                           std::vector<int>& bestCamera,
                           std::vector<float>& bestScore,
                           std::string& error) {
#ifdef PCEDITOR_TEXTURE_HAS_CUDA
    const auto cloud = mesh.vertices();
    if (!cloud) {
        error = "mesh has no vertices";
        return false;
    }
    if (mesh.indices().size() % 3u != 0u) {
        error = "mesh index count is not divisible by 3";
        return false;
    }
    if (bestCamera.size() != mesh.triangleCount() || bestScore.size() != mesh.triangleCount()) {
        error = "CUDA result buffers have invalid size";
        return false;
    }

    std::vector<CudaPointPod> points;
    points.reserve(cloud->size());
    for (const auto& point : cloud->points()) {
        points.push_back({point.position.x, point.position.y, point.position.z});
    }

    std::vector<CudaCameraPod> cameraPods(cameras.size());
    for (std::size_t i = 0; i < cameras.size(); ++i) {
        const auto& source = cameras[i];
        auto& target = cameraPods[i];
        target.fx = source.fx;
        target.fy = source.fy;
        target.cx = source.cx;
        target.cy = source.cy;
        target.imageWidth = source.image.width;
        target.imageHeight = source.image.height;
        std::copy(source.worldToCamera.m.begin(), source.worldToCamera.m.end(), target.worldToCamera);
    }

    CudaConfigPod cudaConfig{};
    cudaConfig.visibilityWidth = depthWidth;
    cudaConfig.visibilityHeight = depthHeight;
    cudaConfig.maxViewAngleDeg = config.maxViewAngleDeg;
    cudaConfig.visibilityTolerance = config.visibilityTolerance;
    cudaConfig.borderMarginRatio = config.borderMarginRatio;
    cudaConfig.buildVisibilityDepth = config.buildVisibilityDepth ? 1 : 0;

    std::array<char, 2048> apiErrorText{};
    const std::size_t depthCount = config.buildVisibilityDepth
        ? cameraPods.size() * static_cast<std::size_t>(depthWidth) * static_cast<std::size_t>(depthHeight) : 0u;
    if (depthCount && packedDepth.empty()) packedDepth.resize(depthCount);
    const bool ok = cudaApiSelectBestCameras(
        points.data(), points.size(), mesh.indices().data(), mesh.indices().size(),
        cameraPods.data(), static_cast<int>(cameraPods.size()), cudaConfig,
        nullptr, 0u,
        packedDepth.empty() ? nullptr : packedDepth.data(), packedDepth.size(),
        bestCamera.data(), bestScore.data(), mesh.triangleCount(),
        apiErrorText.data(), apiErrorText.size());
    if (!ok) error = apiError(apiErrorText.data());
    else error.clear();
    return ok;
#else
    (void)mesh;
    (void)cameras;
    (void)config;
    (void)packedDepth;
    (void)depthWidth;
    (void)depthHeight;
    (void)bestCamera;
    (void)bestScore;
    error = "CUDA backend was not compiled";
    return false;
#endif
}

} // namespace pceditor::texture::detail
