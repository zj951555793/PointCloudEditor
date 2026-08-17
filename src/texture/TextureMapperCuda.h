#pragma once
#include <JMEngine/texture/TextureMapper.h>

namespace JMEngine::texture::detail {
bool cudaRuntimeAvailable(std::string* reason) noexcept;
bool selectBestCamerasCuda(const TriangleMesh& mesh,
                           const std::vector<CameraFrame>& cameras,
                           const Config& config,
                           std::vector<float>& packedDepth,
                           int depthWidth,
                           int depthHeight,
                           std::vector<int>& bestCamera,
                           std::vector<float>& bestScore,
                           std::string& error);
}
