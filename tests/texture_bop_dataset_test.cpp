#include <JMEngine/PointCloud.h>
#include <JMEngine/TriangleMesh.h>
#include <JMEngine/texture/TextureMapper.h>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace JMEngine;
using namespace JMEngine::texture;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

ImageRGB8 loadPpm(const fs::path& path) {
    std::ifstream f(path, std::ios::binary);
    require(bool(f), "cannot open image: " + path.string());
    std::string magic;
    int w = 0, h = 0, maxv = 0;
    f >> magic >> w >> h >> maxv;
    require(magic == "P6" && w > 0 && h > 0 && maxv == 255, "invalid PPM: " + path.string());
    f.get();
    ImageRGB8 out;
    out.width = w;
    out.height = h;
    out.pixels.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3u);
    f.read(reinterpret_cast<char*>(out.pixels.data()), static_cast<std::streamsize>(out.pixels.size()));
    require(f.gcount() == static_cast<std::streamsize>(out.pixels.size()), "truncated PPM: " + path.string());
    return out;
}

TriangleMesh loadMesh(const fs::path& path) {
    std::ifstream f(path);
    require(bool(f), "cannot open mesh case: " + path.string());
    std::string token;
    int version = 0;
    f >> token >> version;
    require(token == "JMENGINE_TEXTURE_MESH" && version == 1, "invalid mesh case header");
    std::size_t n = 0;
    f >> token >> n;
    require(token == "vertices" && n > 0, "mesh has no vertices");
    PointCloud::Container pts(n);
    for (auto& p : pts) {
        f >> p.position.x >> p.position.y >> p.position.z;
        require(bool(f), "invalid mesh vertex");
    }
    std::size_t triCount = 0;
    f >> token >> triCount;
    require(token == "triangles" && triCount > 0, "mesh has no triangles");
    std::vector<std::uint32_t> idx(triCount * 3u);
    for (auto& i : idx) {
        f >> i;
        require(bool(f) && i < n, "invalid mesh triangle index");
    }
    return TriangleMesh(std::make_shared<PointCloud>(std::move(pts)), std::move(idx));
}

std::vector<CameraFrame> loadCameras(const fs::path& caseDir) {
    std::ifstream f(caseDir / "cameras.txt");
    require(bool(f), "cannot open cameras.txt");
    std::string token;
    int version = 0;
    f >> token >> version;
    require(token == "JMENGINE_TEXTURE_CAMERAS" && version == 1, "invalid camera case header");
    std::size_t n = 0;
    f >> token >> n;
    require(token == "cameras" && n >= 2, "not enough cameras");
    std::vector<CameraFrame> cameras;
    cameras.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        CameraFrame c;
        std::string imageName;
        int expectedW = 0, expectedH = 0;
        f >> c.frameId >> imageName >> expectedW >> expectedH >> c.fx >> c.fy >> c.cx >> c.cy;
        for (float& v : c.worldToCamera.m) f >> v;
        require(bool(f), "invalid camera record");
        c.image = loadPpm(caseDir / "images" / imageName);
        require(c.image.width == expectedW && c.image.height == expectedH, "camera/image dimensions differ");
        cameras.push_back(std::move(c));
    }
    return cameras;
}

std::size_t assignedCount(const Result& r) {
    return static_cast<std::size_t>(std::count_if(r.triangleCameraIds.begin(), r.triangleCameraIds.end(), [](int v) { return v >= 0; }));
}

void validateResult(const char* label, const TriangleMesh& input, const Result& r) {
    require(r.ok, std::string(label) + " failed: " + r.message);
    require(r.vertices && !r.vertices->empty(), std::string(label) + " returned no vertices");
    require(r.indices.size() % 3u == 0u && !r.indices.empty(), std::string(label) + " returned no triangles");
    require(r.texcoords.size() == r.vertices->size(), std::string(label) + " UV/vertex count mismatch");
    require(r.atlas.valid(), std::string(label) + " atlas invalid");
    require(r.triangleCameraIds.size() == input.triangleCount(), std::string(label) + " camera assignment count mismatch");
    for (const auto& uv : r.texcoords) {
        require(std::isfinite(uv.x) && std::isfinite(uv.y), std::string(label) + " UV contains NaN/Inf");
        require(uv.x >= -1e-4f && uv.x <= 1.0001f && uv.y >= -1e-4f && uv.y <= 1.0001f,
                std::string(label) + " UV outside [0,1]");
    }
    const auto assigned = assignedCount(r);
    const double ratio = r.triangleCameraIds.empty() ? 0.0 : double(assigned) / double(r.triangleCameraIds.size());
    require(ratio >= 0.20, std::string(label) + " assigned too few triangles: " + std::to_string(ratio));
    std::cout << "[" << label << "] backend=" << int(r.backendUsed) << " time=" << r.elapsedMs
              << "ms assigned=" << assigned << "/" << r.triangleCameraIds.size()
              << " (" << ratio * 100.0 << "%) atlas=" << r.atlas.width << "x" << r.atlas.height << "\n";
}

} // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 2) throw std::runtime_error("usage: JMEngine_texture_bop_dataset_test <prepared-case-dir> [output-dir]");
        const fs::path caseDir = fs::absolute(argv[1]);
        const fs::path outputDir = argc >= 3 ? fs::absolute(argv[2]) : caseDir / "output";
        require(fs::is_regular_file(caseDir / "READY.txt"), "dataset case is not prepared: " + caseDir.string());
        fs::create_directories(outputDir);

        const TriangleMesh mesh = loadMesh(caseDir / "mesh.txt");
        const auto cameras = loadCameras(caseDir);
        std::cout << "BOP LM texture case: vertices=" << mesh.vertices()->size() << " triangles=" << mesh.triangleCount()
                  << " cameras=" << cameras.size() << "\n";

        TextureMapper mapper;
        Config cfg;
        cfg.maxKeyframes = static_cast<int>(cameras.size());
        cfg.maxAtlasSize = 2048;
        cfg.visibilityWidth = 240;
        cfg.visibilityHeight = 180;
        cfg.visibilityTolerance = 5.0f; // BOP LINEMOD geometry/poses are millimetres.
        cfg.maxViewAngleDeg = 75.0f;
        cfg.exposureCompensation = true;

        cfg.backend = Backend::Cpu;
        Result cpu = mapper.map(mesh, cameras, cfg);
        validateResult("CPU", mesh, cpu);
        std::string msg;
        require(saveObj(cpu, (outputDir / "bop_lm_cpu.obj").string(), &msg), "CPU OBJ export failed: " + msg);

        cfg.backend = Backend::Auto;
        Result automatic = mapper.map(mesh, cameras, cfg);
        validateResult("AUTO", mesh, automatic);
        require(saveObj(automatic, (outputDir / "bop_lm_auto.obj").string(), &msg), "AUTO OBJ export failed: " + msg);

        std::string cudaReason;
        if (TextureMapper::cudaCompiled() && TextureMapper::cudaAvailable(&cudaReason)) {
            cfg.backend = Backend::Cuda;
            Result cuda = mapper.map(mesh, cameras, cfg);
            validateResult("CUDA", mesh, cuda);
            require(cuda.backendUsed == Backend::Cuda, "explicit CUDA test did not use CUDA backend");
            require(saveObj(cuda, (outputDir / "bop_lm_cuda.obj").string(), &msg), "CUDA OBJ export failed: " + msg);

            std::size_t comparable = 0, same = 0;
            for (std::size_t i = 0; i < cpu.triangleCameraIds.size(); ++i) {
                if (cpu.triangleCameraIds[i] >= 0 && cuda.triangleCameraIds[i] >= 0) {
                    ++comparable;
                    if (cpu.triangleCameraIds[i] == cuda.triangleCameraIds[i]) ++same;
                }
            }
            require(comparable > 0, "CPU/CUDA have no commonly textured triangles");
            const double agreement = double(same) / double(comparable);
            std::cout << "[CPU/CUDA] camera agreement=" << agreement * 100.0 << "% (" << same << "/" << comparable << ")\n";
            require(agreement >= 0.85, "CPU/CUDA camera assignment agreement below 85%");
        } else {
            std::cout << "[CUDA] skipped: " << (cudaReason.empty() ? "CUDA backend/device unavailable" : cudaReason) << "\n";
        }

        std::cout << "BOP LINEMOD Mesh->Texture integration test PASSED. Output: " << outputDir << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "BOP LINEMOD texture test FAILED: " << e.what() << "\n";
        return 1;
    }
}
