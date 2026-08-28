#pragma once

#include <JMEngine/texture/TextureMapper.h>

namespace JMEngine::texture::detail {

bool mapWithOpenMVS(const TriangleMesh& mesh, const std::vector<CameraFrame>& inputCameras,
                    const Config& config, Result& out, std::string& error);

} // namespace JMEngine::texture::detail
