#include <JMEngine/PointCloud.h>
#include <JMEngine/TriangleMesh.h>
#include <JMEngine/texture/TextureMapper.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace JMEngine;
using namespace JMEngine::texture;

namespace {

bool readToken(std::istream& in, std::string& token) {
    token.clear();
    while (in) {
        const int c = in.peek();
        if (c == '#') {
            std::string dummy;
            std::getline(in, dummy);
            continue;
        }
        if (c != EOF && std::isspace(static_cast<unsigned char>(c))) {
            in.get();
            continue;
        }
        break;
    }
    return static_cast<bool>(in >> token);
}

bool loadPpm(const fs::path& path, ImageRGB8& image, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "cannot open image: " + path.string();
        return false;
    }
    std::string tok;
    if (!readToken(in, tok) || tok != "P6") {
        error = "expected binary PPM P6: " + path.string();
        return false;
    }
    if (!readToken(in, tok)) return false;
    image.width = std::stoi(tok);
    if (!readToken(in, tok)) return false;
    image.height = std::stoi(tok);
    if (!readToken(in, tok)) return false;
    const int maxv = std::stoi(tok);
    if (image.width <= 0 || image.height <= 0 || maxv != 255) {
        error = "unsupported PPM header: " + path.string();
        return false;
    }
    in.get(); // consume one whitespace byte before pixels
    image.pixels.resize(static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height) * 3u);
    in.read(reinterpret_cast<char*>(image.pixels.data()), static_cast<std::streamsize>(image.pixels.size()));
    if (in.gcount() != static_cast<std::streamsize>(image.pixels.size())) {
        error = "truncated PPM: " + path.string();
        return false;
    }
    return true;
}

bool loadMesh(const fs::path& path, TriangleMesh& mesh, std::string& error) {
    std::ifstream in(path);
    if (!in) {
        error = "cannot open mesh: " + path.string();
        return false;
    }
    std::string magic;
    std::size_t nv = 0, nt = 0;
    in >> magic >> nv >> nt;
    if (!in || magic != "JMENGINE_ETH3D_MESH_V1" || nv == 0 || nt == 0) {
        error = "invalid mesh header: " + path.string();
        return false;
    }
    PointCloud::Container pts(nv);
    for (std::size_t i = 0; i < nv; ++i) {
        in >> pts[i].position.x >> pts[i].position.y >> pts[i].position.z;
        if (!in) {
            error = "invalid vertex data";
            return false;
        }
    }
    std::vector<std::uint32_t> idx(nt * 3u);
    for (std::size_t i = 0; i < idx.size(); ++i) {
        in >> idx[i];
        if (!in || idx[i] >= nv) {
            error = "invalid triangle index";
            return false;
        }
    }
    mesh = TriangleMesh(std::make_shared<PointCloud>(std::move(pts)), std::move(idx));
    return true;
}

bool loadCameras(const fs::path& root, std::vector<CameraFrame>& cameras, std::string& error) {
    std::ifstream in(root / "cameras.txt");
    if (!in) {
        error = "cannot open cameras.txt";
        return false;
    }
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        CameraFrame c;
        std::string imageName;
        ss >> c.frameId >> c.fx >> c.fy >> c.cx >> c.cy >> imageName;
        for (int i = 0; i < 16; ++i) ss >> c.worldToCamera.m[i];
        if (!ss) {
            error = "invalid camera line: " + line;
            return false;
        }
        if (!loadPpm(root / imageName, c.image, error)) return false;
        cameras.push_back(std::move(c));
    }
    if (cameras.empty()) {
        error = "no cameras in cameras.txt";
        return false;
    }
    return true;
}

bool uvValid(const Result& r) {
    if (!r.vertices || r.texcoords.size() != r.vertices->size()) return false;
    for (const auto& uv : r.texcoords) {
        if (!(uv.x >= -1e-5f && uv.x <= 1.00001f && uv.y >= -1e-5f && uv.y <= 1.00001f)) return false;
    }
    return true;
}

double assignedFraction(const Result& r) {
    if (r.triangleCameraIds.empty()) return 0.0;
    const auto n = std::count_if(r.triangleCameraIds.begin(), r.triangleCameraIds.end(), [](int id) { return id >= 0; });
    return static_cast<double>(n) / static_cast<double>(r.triangleCameraIds.size());
}

double assignmentAgreement(const Result& a, const Result& b) {
    const std::size_t n = std::min(a.triangleCameraIds.size(), b.triangleCameraIds.size());
    if (n == 0) return 0.0;
    std::size_t valid = 0, same = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (a.triangleCameraIds[i] < 0 && b.triangleCameraIds[i] < 0) continue;
        ++valid;
        if (a.triangleCameraIds[i] == b.triangleCameraIds[i]) ++same;
    }
    return valid ? static_cast<double>(same) / static_cast<double>(valid) : 1.0;
}

int fail(const std::string& s) {
    std::cerr << "[ETH3D texture test] FAIL: " << s << '\n';
    return 1;
}

} // namespace

int main(int argc, char** argv) {
    fs::path root = argc > 1 ? fs::path(argv[1]) : fs::path("tests/data/eth3d_rgbd_texture");
    if (!fs::exists(root / "case.txt")) {
        std::cout << "[ETH3D texture test] SKIP: prepared dataset not found at " << root << "\n"
                  << "Run tools/prepare_eth3d_texture_case.py first.\n";
        return 77;
    }

    TriangleMesh mesh;
    std::vector<CameraFrame> cameras;
    std::string error;
    if (!loadMesh(root / "mesh.txt", mesh, error)) return fail(error);
    if (!loadCameras(root, cameras, error)) return fail(error);

    Config cfg;
    cfg.backend = Backend::Cpu;
    cfg.maxKeyframes = static_cast<int>(cameras.size());
    cfg.maxAtlasSize = 2048;
    cfg.visibilityWidth = 320;
    cfg.visibilityHeight = 240;
    cfg.visibilityTolerance = 0.015f; // ETH3D RGB-D mesh units are metres.
    cfg.maxViewAngleDeg = 75.0f;
    cfg.exposureCompensation = true;

    TextureMapper mapper;
    const Result cpu = mapper.map(mesh, cameras, cfg);
    if (!cpu.ok) return fail("CPU backend: " + cpu.message);
    if (!cpu.atlas.valid()) return fail("CPU atlas invalid");
    if (!uvValid(cpu)) return fail("CPU UVs outside [0,1]");
    const double cpuAssigned = assignedFraction(cpu);
    if (cpuAssigned < 0.25) {
        return fail("CPU assigned triangle fraction too low: " + std::to_string(cpuAssigned));
    }

    std::cout << "[ETH3D texture test] CPU PASS: triangles=" << mesh.triangleCount()
              << " cameras=" << cameras.size()
              << " assigned=" << cpuAssigned * 100.0 << "% elapsed=" << cpu.elapsedMs << " ms\n";

    Config autoCfg = cfg;
    autoCfg.backend = Backend::Auto;
    const Result automatic = mapper.map(mesh, cameras, autoCfg);
    if (!automatic.ok) return fail("Auto backend: " + automatic.message);
    if (!uvValid(automatic) || !automatic.atlas.valid()) return fail("Auto output invalid");
    std::cout << "[ETH3D texture test] AUTO PASS: backend="
              << (automatic.backendUsed == Backend::Cuda ? "CUDA" : "CPU")
              << " assigned=" << assignedFraction(automatic) * 100.0
              << "% elapsed=" << automatic.elapsedMs << " ms\n";

    std::string cudaReason;
    if (TextureMapper::cudaAvailable(&cudaReason)) {
        Config cudaCfg = cfg;
        cudaCfg.backend = Backend::Cuda;
        const Result cuda = mapper.map(mesh, cameras, cudaCfg);
        if (!cuda.ok) return fail("CUDA backend: " + cuda.message);
        if (!uvValid(cuda) || !cuda.atlas.valid()) return fail("CUDA output invalid");
        const double agree = assignmentAgreement(cpu, cuda);
        if (agree < 0.90) {
            return fail("CPU/CUDA camera assignment agreement too low: " + std::to_string(agree));
        }
        std::cout << "[ETH3D texture test] CUDA PASS: assigned=" << assignedFraction(cuda) * 100.0
                  << "% agreementWithCPU=" << agree * 100.0 << "% elapsed=" << cuda.elapsedMs << " ms\n";
    } else {
        std::cout << "[ETH3D texture test] CUDA SKIP: " << cudaReason << "\n";
    }

    std::string exportMessage;
    const fs::path out = root / "output_cpu.obj";
    if (!saveObj(cpu, out.string(), &exportMessage)) return fail("OBJ export: " + exportMessage);
    std::cout << "[ETH3D texture test] output: " << out << "\n";
    return 0;
}
