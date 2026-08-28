#include "TextureMapperOpenMVS.h"

#if JMENGINE_TEXTURE_OPENMVS_USE_OPENCV_HEADERS
#include <OpenMVS/MVS.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <sstream>
#include <vector>

namespace JMEngine::texture::detail {
namespace {

MVS::Mesh::Vertex toOpenMvsVertex(const JMEngine::Vec3f& p) {
    return MVS::Mesh::Vertex(p.x, p.y, p.z);
}

SEACAVE::KMatrix toOpenMvsK(const CameraFrame& frame) {
    SEACAVE::KMatrix k(SEACAVE::KMatrix::IDENTITY);
    k(0, 0) = frame.fx;
    k(1, 1) = frame.fy;
    k(0, 2) = frame.cx;
    k(1, 2) = frame.cy;
    return k;
}

SEACAVE::RMatrix toOpenMvsR(const JMEngine::Mat4f& worldToCamera) {
    SEACAVE::RMatrix r(SEACAVE::RMatrix::IDENTITY);
    r(0, 0) = worldToCamera.m[0];
    r(0, 1) = worldToCamera.m[4];
    r(0, 2) = worldToCamera.m[8];
    r(1, 0) = worldToCamera.m[1];
    r(1, 1) = worldToCamera.m[5];
    r(1, 2) = worldToCamera.m[9];
    r(2, 0) = worldToCamera.m[2];
    r(2, 1) = worldToCamera.m[6];
    r(2, 2) = worldToCamera.m[10];
    return r;
}

SEACAVE::CMatrix toOpenMvsC(const JMEngine::Mat4f& worldToCamera) {
    const float tx = worldToCamera.m[12];
    const float ty = worldToCamera.m[13];
    const float tz = worldToCamera.m[14];
    return SEACAVE::CMatrix(
        -(worldToCamera.m[0] * tx + worldToCamera.m[1] * ty + worldToCamera.m[2] * tz),
        -(worldToCamera.m[4] * tx + worldToCamera.m[5] * ty + worldToCamera.m[6] * tz),
        -(worldToCamera.m[8] * tx + worldToCamera.m[9] * ty + worldToCamera.m[10] * tz));
}

SEACAVE::Image8U3 toOpenMvsImage(const ImageRGB8& image) {
    SEACAVE::Image8U3 out(image.height, image.width);
    for (int y = 0; y < image.height; ++y) {
        auto* row = out.ptr<SEACAVE::Pixel8U>(y);
        for (int x = 0; x < image.width; ++x) {
            const std::size_t src =
                (static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width) +
                 static_cast<std::size_t>(x)) *
                3u;
            row[x] = SEACAVE::Pixel8U(image.pixels[src], image.pixels[src + 1u],
                                      image.pixels[src + 2u]);
        }
    }
    return out;
}

ImageRGB8 fromOpenMvsImage(const SEACAVE::Image8U3& image) {
    ImageRGB8 out;
    out.width = image.width();
    out.height = image.height();
    out.pixels.resize(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height) *
                      3u);
    for (int y = 0; y < out.height; ++y) {
        const auto* row = image.ptr<SEACAVE::Pixel8U>(y);
        for (int x = 0; x < out.width; ++x) {
            const std::size_t dst =
                (static_cast<std::size_t>(y) * static_cast<std::size_t>(out.width) +
                 static_cast<std::size_t>(x)) *
                3u;
            out.pixels[dst] = row[x].r;
            out.pixels[dst + 1u] = row[x].g;
            out.pixels[dst + 2u] = row[x].b;
        }
    }
    return out;
}

bool appendOpenMvsTextures(const MVS::Mesh& mesh, ImageRGB8& atlas, std::vector<int>& offsetsX) {
    if (mesh.texturesDiffuse.empty()) return false;

    int atlasW = 0;
    int atlasH = 0;
    offsetsX.clear();
    offsetsX.reserve(mesh.texturesDiffuse.size());
    for (const auto& texture : mesh.texturesDiffuse) {
        if (texture.empty()) continue;
        offsetsX.push_back(atlasW);
        atlasW += texture.width();
        atlasH = std::max(atlasH, texture.height());
    }
    if (atlasW <= 0 || atlasH <= 0) return false;

    atlas.width = atlasW;
    atlas.height = atlasH;
    atlas.pixels.assign(static_cast<std::size_t>(atlasW) * static_cast<std::size_t>(atlasH) * 3u,
                        0u);

    for (std::size_t ti = 0; ti < mesh.texturesDiffuse.size(); ++ti) {
        const auto& texture = mesh.texturesDiffuse[static_cast<MVS::Mesh::TexIndex>(ti)];
        if (texture.empty()) continue;
        const int ox = offsetsX[ti];
        for (int y = 0; y < texture.height(); ++y) {
            const auto* row = texture.ptr<SEACAVE::Pixel8U>(y);
            for (int x = 0; x < texture.width(); ++x) {
                const std::size_t dst =
                    (static_cast<std::size_t>(y) * static_cast<std::size_t>(atlasW) +
                     static_cast<std::size_t>(ox + x)) *
                    3u;
                atlas.pixels[dst] = row[x].r;
                atlas.pixels[dst + 1u] = row[x].g;
                atlas.pixels[dst + 2u] = row[x].b;
            }
        }
    }
    return true;
}

std::uint32_t packRgb(std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    return std::uint32_t(r) | (std::uint32_t(g) << 8u) | (std::uint32_t(b) << 16u) |
           0xff000000u;
}

} // namespace

bool mapWithOpenMVS(const TriangleMesh& mesh, const std::vector<CameraFrame>& inputCameras,
                    const Config& config, Result& out, std::string& error) {
    const auto started = std::chrono::steady_clock::now();
    out = Result{};
    error.clear();

    const auto vertices = mesh.vertices();
    if (mesh.empty() || !vertices || vertices->empty()) {
        error = out.message = "mesh is empty";
        return false;
    }

    std::vector<const CameraFrame*> cameras;
    cameras.reserve(std::min<std::size_t>(inputCameras.size(), std::max(0, config.maxKeyframes)));
    out.inputCameraCount = inputCameras.size();
    for (const auto& camera : inputCameras) {
        if (static_cast<int>(cameras.size()) >= config.maxKeyframes) break;
        if (!camera.image.valid() || camera.fx <= 0.0f || camera.fy <= 0.0f) continue;
        cameras.push_back(&camera);
    }
    out.acceptedCameraCount = cameras.size();
    if (cameras.empty()) {
        error = out.message = "no valid texture camera frames";
        return false;
    }

    MVS::Scene scene;
    scene.mesh.vertices.reserve(vertices->size());
    for (const auto& point : vertices->points()) {
        scene.mesh.vertices.push_back(toOpenMvsVertex(point.position));
    }

    scene.mesh.faces.reserve(mesh.triangleCount());
    const auto& indices = mesh.indices();
    for (std::size_t ti = 0; ti < mesh.triangleCount(); ++ti) {
        scene.mesh.faces.push_back(MVS::Mesh::Face(indices[ti * 3u], indices[ti * 3u + 1u],
                                                   indices[ti * 3u + 2u]));
    }

    scene.platforms.reserve(cameras.size());
    scene.images.reserve(cameras.size());
    for (std::size_t i = 0; i < cameras.size(); ++i) {
        const CameraFrame& source = *cameras[i];
        const SEACAVE::KMatrix k = toOpenMvsK(source);
        const SEACAVE::RMatrix r = toOpenMvsR(source.worldToCamera);
        const SEACAVE::CMatrix c = toOpenMvsC(source.worldToCamera);

        MVS::Platform platform;
        platform.name = SEACAVE::String::FormatString("camera_%u", static_cast<unsigned>(i));
        platform.cameras.push_back(MVS::Platform::Camera(k, r, c));
        platform.poses.push_back(
            MVS::Platform::Pose{SEACAVE::RMatrix::IDENTITY, SEACAVE::CMatrix::ZERO});
        scene.platforms.push_back(platform);

        MVS::Image image;
        image.platformID = static_cast<std::uint32_t>(i);
        image.cameraID = 0;
        image.poseID = 0;
        image.ID = static_cast<std::uint32_t>(i);
        image.name = SEACAVE::String::FormatString("frame_%u",
                                                   static_cast<unsigned>(source.frameId));
        image.width = static_cast<std::uint32_t>(source.image.width);
        image.height = static_cast<std::uint32_t>(source.image.height);
        image.image = toOpenMvsImage(source.image);
        image.camera = MVS::Camera(k, r, c);
        image.scale = 1.0f;
        scene.images.push_back(image);
    }
    scene.nCalibratedImages = static_cast<unsigned>(scene.images.size());

    const unsigned resolutionLevel = 0;
    const unsigned minResolution =
        static_cast<unsigned>(std::max(0, std::min(config.visibilityWidth, config.visibilityHeight)));
    const unsigned minCommonCameras = 0;
    const float outlierThreshold = 0.0f;
    const float ratioDataSmoothness = std::clamp(config.patchSmoothness, 0.0f, 1.0f);
    const unsigned textureSizeMultiple = 0;
    const SEACAVE::Pixel8U emptyColor(190, 190, 190);
    const float sharpnessWeight = config.rejectBlurredFrames ? 0.5f : 0.0f;
    const int ignoreMaskLabel = -1;
    const int maxTextureSize = std::max(256, config.maxAtlasSize);

    if (!scene.TextureMesh(resolutionLevel, minResolution, minCommonCameras, outlierThreshold,
                           ratioDataSmoothness, config.exposureCompensation,
                           config.exposureCompensation, textureSizeMultiple, emptyColor,
                           sharpnessWeight, ignoreMaskLabel, maxTextureSize)) {
        error = out.message = "OpenMVS TextureMesh failed";
        return false;
    }

    std::vector<int> textureOffsets;
    if (!appendOpenMvsTextures(scene.mesh, out.atlas, textureOffsets)) {
        error = out.message = "OpenMVS produced no diffuse texture";
        return false;
    }

    PointCloud::Container outVertices;
    outVertices.reserve(scene.mesh.faces.size() * 3u);
    out.indices.reserve(scene.mesh.faces.size() * 3u);
    out.texcoords.reserve(scene.mesh.faces.size() * 3u);
    out.triangleCameraIds.assign(mesh.triangleCount(), -1);

    std::size_t mappedTriangleCount = 0;
    for (std::size_t ti = 0; ti < scene.mesh.faces.size(); ++ti) {
        const auto& face = scene.mesh.faces[static_cast<MVS::Mesh::FIndex>(ti)];
        const auto texIndex = scene.mesh.GetFaceTextureIndex(static_cast<MVS::Mesh::FIndex>(ti));
        const bool hasTexture =
            !scene.mesh.faceTexcoords.empty() && texIndex < textureOffsets.size() &&
            texIndex < scene.mesh.texturesDiffuse.size() &&
            !scene.mesh.texturesDiffuse[texIndex].empty();
        const auto& texture = hasTexture ? scene.mesh.texturesDiffuse[texIndex]
                                         : scene.mesh.texturesDiffuse[0];
        const int textureOffset = hasTexture ? textureOffsets[texIndex] : textureOffsets[0];
        const std::uint32_t base = static_cast<std::uint32_t>(outVertices.size());

        for (int corner = 0; corner < 3; ++corner) {
            const auto vertex = scene.mesh.vertices[face[corner]];
            Point point;
            point.position = {vertex.x, vertex.y, vertex.z};
            point.rgba = packRgb(255, 255, 255);
            outVertices.push_back(point);
            out.indices.push_back(base + static_cast<std::uint32_t>(corner));

            Vec2f uv{0.5f, 0.5f};
            if (hasTexture) {
                const std::size_t texCoordIndex = scene.mesh.HasTextureCoordinatesPerVertex()
                                                      ? static_cast<std::size_t>(face[corner])
                                                      : ti * 3u + static_cast<std::size_t>(corner);
                if (texCoordIndex < scene.mesh.faceTexcoords.size()) {
                    const auto& mvsUv =
                        scene.mesh.faceTexcoords[static_cast<MVS::Mesh::FIndex>(texCoordIndex)];
                    uv.x = (static_cast<float>(textureOffset) + mvsUv.x * texture.width()) /
                           static_cast<float>(out.atlas.width);
                    uv.y = mvsUv.y * static_cast<float>(texture.height()) /
                           static_cast<float>(out.atlas.height);
                }
            }
            out.texcoords.push_back(uv);
        }

        if (hasTexture) {
            ++mappedTriangleCount;
            if (ti < out.triangleCameraIds.size()) out.triangleCameraIds[ti] = 0;
        }
    }

    if (outVertices.empty()) {
        error = out.message = "OpenMVS produced no textured triangles";
        return false;
    }

    out.vertices = std::make_shared<PointCloud>(std::move(outVertices));
    out.ok = true;
    out.backendUsed = Backend::Cpu;
    out.usedCameraCount = cameras.size();
    out.mappedTriangleCount = mappedTriangleCount;
    out.unmappedTriangleCount = scene.mesh.faces.size() - mappedTriangleCount;
    out.texturePatchCount = scene.mesh.texturesDiffuse.size();
    out.elapsedMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
            .count();

    std::ostringstream message;
    message << "OpenMVS textured " << out.mappedTriangleCount << "/" << scene.mesh.faces.size()
            << " triangles with " << out.usedCameraCount << " cameras, textures="
            << out.texturePatchCount;
    out.message = message.str();
    return true;
}

} // namespace JMEngine::texture::detail
#else

#include <string>
#include <vector>

namespace JMEngine::texture::detail {

bool mapWithOpenMVS(const TriangleMesh&, const std::vector<CameraFrame>&, const Config&,
                    Result& out, std::string& error) {
    out = Result{};
    error =
        "OpenMVS texture mapping is not available: the bundled static OpenMVS SDK still exposes "
        "OpenCV types in its public MVS headers and libraries. Rebuild/vendor OpenMVS with a "
        "no-OpenCV public ABI, or enable JMENGINE_OPENMVS_WITH_OPENCV_ABI to compile against that "
        "SDK.";
    out.message = error;
    return false;
}

} // namespace JMEngine::texture::detail
#endif
