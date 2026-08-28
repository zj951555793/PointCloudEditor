#include <JMEngine/texture/TextureMapper.h>

#include "TextureMapperOpenMVS.h"

#include <fstream>

namespace JMEngine::texture {

bool TextureMapper::cudaCompiled() noexcept {
    return false;
}

bool TextureMapper::cudaAvailable(std::string* reason) noexcept {
    if (reason) {
        *reason = "CUDA texture mapping backend was removed; OpenMVS library backend is used";
    }
    return false;
}

Result TextureMapper::map(const TriangleMesh& mesh, const std::vector<CameraFrame>& inputCameras,
                          const Config& config) const {
    Result out;
    std::string error;
    if (!detail::mapWithOpenMVS(mesh, inputCameras, config, out, error)) {
        out.ok = false;
        out.message = error.empty() ? "OpenMVS texture mapping failed" : error;
    }
    return out;
}

bool saveObj(const Result& result, const std::string& path, std::string* message,
             const TriangleMesh* activeTopology) {
    if (!result.ok || !result.vertices || result.indices.empty() ||
        result.texcoords.size() != result.vertices->size() || !result.atlas.valid()) {
        if (message) *message = "invalid texture result";
        return false;
    }
    auto dot = path.find_last_of('.');
    std::string stem = (dot == std::string::npos) ? path : path.substr(0, dot);
    std::string obj = stem + ".obj", mtl = stem + ".mtl", tex = stem + "_texture.tga";
    auto slash = stem.find_last_of("/\\");
    std::string base = (slash == std::string::npos) ? stem : stem.substr(slash + 1);
    std::ofstream tf(tex, std::ios::binary);
    if (!tf) {
        if (message) *message = "cannot open texture output";
        return false;
    }
    unsigned char header[18]{};
    header[2] = 2;
    header[12] = static_cast<unsigned char>(result.atlas.width & 0xff);
    header[13] = static_cast<unsigned char>((result.atlas.width >> 8) & 0xff);
    header[14] = static_cast<unsigned char>(result.atlas.height & 0xff);
    header[15] = static_cast<unsigned char>((result.atlas.height >> 8) & 0xff);
    header[16] = 24;
    header[17] = 0x00;
    tf.write(reinterpret_cast<const char*>(header), sizeof(header));
    for (std::size_t i = 0; i < result.atlas.pixels.size(); i += 3u) {
        const unsigned char bgr[3]{result.atlas.pixels[i + 2u], result.atlas.pixels[i + 1u],
                                   result.atlas.pixels[i]};
        tf.write(reinterpret_cast<const char*>(bgr), 3);
    }
    std::ofstream mf(mtl);
    if (!mf) {
        if (message) *message = "cannot open mtl output";
        return false;
    }
    mf << "newmtl material0\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd " << base
       << "_texture.tga\n";
    std::ofstream of(obj);
    if (!of) {
        if (message) *message = "cannot open obj output";
        return false;
    }
    of << "mtllib " << base << ".mtl\nusemtl material0\n";
    for (const auto& p : result.vertices->points()) {
        of << "v " << p.position.x << " " << p.position.y << " " << p.position.z << "\n";
    }
    for (const auto& uv : result.texcoords) {
        of << "vt " << uv.x << " " << uv.y << "\n";
    }
    if (activeTopology && activeTopology->triangleCount() != result.indices.size() / 3u) {
        if (message) *message = "textured mesh topology changed; remap texture before export";
        return false;
    }
    for (std::size_t i = 0; i + 2 < result.indices.size(); i += 3) {
        const std::size_t ti = i / 3u;
        if (activeTopology && !activeTopology->triangleActive(static_cast<TriangleId>(ti))) continue;
        auto a = result.indices[i] + 1, b = result.indices[i + 1] + 1, c = result.indices[i + 2] + 1;
        of << "f " << a << "/" << a << " " << b << "/" << b << " " << c << "/" << c << "\n";
    }
    if (message) *message = "saved textured OBJ/MTL/TGA";
    return true;
}

} // namespace JMEngine::texture
