#include <pceditor/PointCloud.h>
#include <pceditor/TriangleMesh.h>
#include <pceditor/texture/TextureMapper.h>

#include <cassert>
#include <iostream>

int main() {
    using namespace pceditor;
    using namespace pceditor::texture;

    PointCloud::Container pts(3);
    pts[0].position = {-10.0f, -10.0f, 100.0f};
    pts[1].position = { 10.0f, -10.0f, 100.0f};
    pts[2].position = {  0.0f,  10.0f, 100.0f};
    auto cloud = std::make_shared<PointCloud>(std::move(pts));
    TriangleMesh mesh(cloud, {0, 1, 2});

    CameraFrame cam;
    cam.frameId = 7;
    cam.fx = 100.0f; cam.fy = 100.0f; cam.cx = 32.0f; cam.cy = 32.0f;
    cam.image.width = 64; cam.image.height = 64;
    cam.image.pixels.resize(64u * 64u * 3u);
    for (int y = 0; y < 64; ++y) {
        for (int x = 0; x < 64; ++x) {
            const auto i = (static_cast<std::size_t>(y) * 64u + static_cast<std::size_t>(x)) * 3u;
            cam.image.pixels[i + 0] = static_cast<std::uint8_t>(x * 4);
            cam.image.pixels[i + 1] = static_cast<std::uint8_t>(y * 4);
            cam.image.pixels[i + 2] = 128;
        }
    }

    Config cfg;
    cfg.backend = Backend::Auto;
    cfg.maxAtlasSize = 256;
    cfg.visibilityWidth = 64;
    cfg.visibilityHeight = 64;
    cfg.visibilityTolerance = 2.0f;

    TextureMapper mapper;
    Result result = mapper.map(mesh, {cam}, cfg);
    assert(result.ok);
    assert(result.backendUsed == Backend::Cpu || result.backendUsed == Backend::Cuda);
    assert(result.vertices && result.vertices->size() == 3u);
    assert(result.indices.size() == 3u);
    assert(result.texcoords.size() == 3u);
    assert(result.triangleCameraIds.size() == 1u && result.triangleCameraIds[0] == 7);
    assert(result.atlas.valid());
    assert(result.inputCameraCount == 1u);
    assert(result.acceptedCameraCount == 1u);
    assert(result.usedCameraCount == 1u);
    assert(result.mappedTriangleCount == 1u);
    assert(result.unmappedTriangleCount == 0u);
    assert(result.texturePatchCount == 1u);
    // Industrial atlas mode reserves an 8 px gutter by default; UVs must stay inside it.
    for (const auto& uv : result.texcoords) {
        assert(uv.x > 8.0f / float(result.atlas.width));
        assert(uv.x < 1.0f - 8.0f / float(result.atlas.width));
        assert(uv.y > 8.0f / float(result.atlas.height));
        assert(uv.y < 1.0f - 8.0f / float(result.atlas.height));
    }


    // OpenMVS-style mode auto-detects global winding and uses a connected patch atlas.
    Config openCfg = cfg;
    openCfg.backend = Backend::Cpu;
    openCfg.quality = Quality::OpenMVS;
    openCfg.patchAtlas = true;
    Result openResult = mapper.map(mesh, {cam}, openCfg);
    assert(openResult.ok);
    assert(openResult.mappedTriangleCount == 1u);
    assert(openResult.texturePatchCount == 1u);
    assert(openResult.triangleCameraIds[0] == 7);

    // Severe blur rejection must discard an obvious flat frame without collapsing the sequence.
    std::vector<CameraFrame> qualityCams;
    for (int ci=0; ci<4; ++ci) {
        CameraFrame q = cam;
        q.frameId = 100 + ci;
        for (int y=0; y<64; ++y) for (int x=0; x<64; ++x) {
            const auto i=(static_cast<std::size_t>(y)*64u+static_cast<std::size_t>(x))*3u;
            const std::uint8_t v = (ci == 3) ? 128u : (((x/2 + y/2) & 1) ? 230u : 25u);
            q.image.pixels[i]=v; q.image.pixels[i+1u]=v; q.image.pixels[i+2u]=v;
        }
        qualityCams.push_back(std::move(q));
    }
    Config qualityCfg = cfg;
    qualityCfg.backend = Backend::Cpu;
    qualityCfg.rejectBlurredFrames = true;
    qualityCfg.minRelativeSharpness = 0.18f;
    Result qualityResult = mapper.map(mesh, qualityCams, qualityCfg);
    assert(qualityResult.ok);
    assert(qualityResult.inputCameraCount == 4u);
    assert(qualityResult.acceptedCameraCount == 3u);

    std::cout << "texture CPU test passed: " << result.message << "\n";
    return 0;
}
