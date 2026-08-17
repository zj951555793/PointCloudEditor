#include <JMEngine/processing/Operations.h>
#include <JMEngine/MeshUtils.h>
#include <JMEngine/processing/Parallel.h>
#include <JMEngine/processing/Diagnostics.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <iomanip>
#include <memory>
#include <numeric>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#endif

#ifdef JMENGINE_HAS_POISSONRECON
#include "MultiThreading.h"
#include "PreProcessor.h"
#include "Reconstructors.h"
#endif

namespace JMEngine::processing {
namespace {

float normalLength(const Vec3f& n) {
    return std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
}

Vec3f normalized(Vec3f n) {
    const float l = normalLength(n);
    if (l > 1e-12f) {
        n.x /= l;
        n.y /= l;
        n.z /= l;
    }
    return n;
}

struct ColorGridKey {
    int x{}, y{}, z{};
    bool operator==(const ColorGridKey& rhs) const noexcept {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }
};

struct ColorGridHash {
    std::size_t operator()(const ColorGridKey& k) const noexcept {
        std::size_t h = 1469598103934665603ull;
        auto mix = [&](int v) {
            h ^= static_cast<unsigned int>(v);
            h *= 1099511628211ull;
        };
        mix(k.x);
        mix(k.y);
        mix(k.z);
        return h;
    }
};

ColorGridKey colorGridKey(const Vec3f& p, float invCell) {
    return {static_cast<int>(std::floor(p.x * invCell)), static_cast<int>(std::floor(p.y * invCell)),
            static_cast<int>(std::floor(p.z * invCell))};
}

float distanceSquared(const Vec3f& a, const Vec3f& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

// Poisson 输出顶点与输入点不一一对应。这里使用空间哈希在源点云中寻找最近采样点，
// 把 RGB 传递到最终 Mesh 顶点。它是 O(N) 级构建 + 局部邻域查询，不引入额外 KDTree 依赖。
[[maybe_unused]] bool transferInputColors(const PointCloud& source, TriangleMesh& mesh, const CancelToken& cancel) {
    auto vertices = mesh.vertices();
    if (!vertices || vertices->empty() || source.empty())
        return false;

    Vec3f mn{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    Vec3f mx{-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
             -std::numeric_limits<float>::max()};
    std::size_t active = 0;
    for (const auto& p : source.points()) {
        if (p.flags & PointDeleted)
            continue;
        mn.x = std::min(mn.x, p.position.x);
        mn.y = std::min(mn.y, p.position.y);
        mn.z = std::min(mn.z, p.position.z);
        mx.x = std::max(mx.x, p.position.x);
        mx.y = std::max(mx.y, p.position.y);
        mx.z = std::max(mx.z, p.position.z);
        ++active;
    }
    if (!active)
        return false;

    const double dx = std::max(0.0, double(mx.x) - double(mn.x));
    const double dy = std::max(0.0, double(mx.y) - double(mn.y));
    const double dz = std::max(0.0, double(mx.z) - double(mn.z));
    const double volume = dx * dy * dz;
    const double diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double countScale = std::cbrt(double(active));
    const double spacingEstimate =
        volume > 1e-18 ? std::cbrt(volume / double(active)) : diagonal / std::max(1.0, countScale);
    const float spacing = static_cast<float>(std::max(1e-7, spacingEstimate));
    const float cell = std::max(1e-7f, spacing * 2.0f);
    const float invCell = 1.0f / cell;

    std::unordered_map<ColorGridKey, std::vector<std::uint32_t>, ColorGridHash> grid;
    grid.reserve(active / 4 + 1);
    for (std::uint32_t i = 0; i < source.size(); ++i) {
        const auto& p = source.points()[i];
        if (p.flags & PointDeleted)
            continue;
        grid[colorGridKey(p.position, invCell)].push_back(i);
    }

    const int nt = processingThreadCount();
#ifdef JMENGINE_USE_OPENMP
#pragma omp parallel for schedule(static) num_threads(nt)
#endif
    for (long long vi = 0; vi < static_cast<long long>(vertices->size()); ++vi) {
        if (cancel.cancelled())
            continue;
        auto& outPoint = vertices->points()[static_cast<std::size_t>(vi)];
        const auto base = colorGridKey(outPoint.position, invCell);
        float best = std::numeric_limits<float>::max();
        std::uint32_t bestColor = outPoint.rgba;
        bool found = false;

        // 两层邻域足以覆盖 Poisson 等值面顶点与原始采样之间的常见偏移。
        for (int radius = 1; radius <= 2 && !found; ++radius) {
            for (int dz0 = -radius; dz0 <= radius; ++dz0)
                for (int dy0 = -radius; dy0 <= radius; ++dy0)
                    for (int dx0 = -radius; dx0 <= radius; ++dx0) {
                        const auto it = grid.find({base.x + dx0, base.y + dy0, base.z + dz0});
                        if (it == grid.end())
                            continue;
                        for (const auto id : it->second) {
                            const auto& srcPoint = source.points()[id];
                            const float d2 = distanceSquared(outPoint.position, srcPoint.position);
                            if (d2 < best) {
                                best = d2;
                                bestColor = srcPoint.rgba;
                                found = true;
                            }
                        }
                    }
        }
        if (found)
            outPoint.rgba = bestColor;
    }
    return !cancel.cancelled();
}


double pointCloudDiagonal(const PointCloud& source) {
    bool valid = false;
    Vec3f lo{}, hi{};
    for (const auto& p : source.points()) {
        if ((p.flags & PointDeleted) || !std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
            !std::isfinite(p.position.z))
            continue;
        if (!valid) { lo = hi = p.position; valid = true; }
        else {
            lo.x = std::min(lo.x, p.position.x); lo.y = std::min(lo.y, p.position.y); lo.z = std::min(lo.z, p.position.z);
            hi.x = std::max(hi.x, p.position.x); hi.y = std::max(hi.y, p.position.y); hi.z = std::max(hi.z, p.position.z);
        }
    }
    if (!valid) return 0.0;
    const double x = static_cast<double>(hi.x) - lo.x;
    const double y = static_cast<double>(hi.y) - lo.y;
    const double z = static_cast<double>(hi.z) - lo.z;
    return std::sqrt(x*x + y*y + z*z);
}

// 注意：本模块不再修改输入法线方向。
// Poisson 要求调用方提供已经定向好的法线；如果法线缺失，仅通过 KNN/PCA 补齐方向轴。
// 这里刻意不做“质心朝外”、BFS/MST 翻转等二次一致化，避免非凸/开放扫描被错误翻面。

std::shared_ptr<PointCloud> ensurePoissonNormals(const ProcessInput& input, const ParameterMap& params,
                                                 const ProgressCallback& progress, const CancelToken& cancel,
                                                 std::string& error) {
    if (!input.cloud) {
        error = "泊松重建需要点云";
        return {};
    }

    std::shared_ptr<PointCloud> cloud = input.cloud;

    if (boolParam(params, "preclean", false)) {
        // MeshLab 的 Pre-Clean 默认关闭；开启时才复制并清理无效点/空法线。
        auto cleaned = std::make_shared<PointCloud>(cloud->points());
        const int nt = processingThreadCount();
#ifdef JMENGINE_USE_OPENMP
#pragma omp parallel for schedule(static) num_threads(nt)
#endif
        for (long long i = 0; i < static_cast<long long>(cleaned->size()); ++i) {
            auto& pt = cleaned->points()[static_cast<std::size_t>(i)];
            const bool badPos =
                !std::isfinite(pt.position.x) || !std::isfinite(pt.position.y) || !std::isfinite(pt.position.z);
            const bool badNormal =
                !std::isfinite(pt.normal.x) || !std::isfinite(pt.normal.y) || !std::isfinite(pt.normal.z) ||
                normalLength(pt.normal) < 0.5f;
            if (badPos || badNormal)
                pt.flags |= PointDeleted;
        }
        cloud = std::move(cleaned);
    }

    // 全量法向覆盖率确认放在 worker 内，并行 reduction。
    unsigned long long validNormals64 = 0;
    unsigned long long active64 = 0;
    const int validationThreads = processingThreadCount();
#ifdef JMENGINE_USE_OPENMP
#pragma omp parallel for reduction(+ : validNormals64, active64) schedule(static) num_threads(validationThreads)
#endif
    for (long long i = 0; i < static_cast<long long>(cloud->size()); ++i) {
        const auto& p = cloud->points()[static_cast<std::size_t>(i)];
        if (p.flags & PointDeleted)
            continue;
        ++active64;
        if (normalLength(p.normal) > 0.5f)
            ++validNormals64;
    }
    const std::size_t validNormals = static_cast<std::size_t>(validNormals64);
    const std::size_t active = static_cast<std::size_t>(active64);

    if (active < 16) {
        error = "泊松重建至少需要 16 个有效点";
        return {};
    }

    const bool estimateMissing = boolParam(params, "estimate_normals", true);
    if (validNormals < active * 9 / 10) {
        if (!estimateMissing) {
            error = "输入点云缺少稳定法向，请启用自动法向估计";
            return {};
        }
        NormalEstimationOperation normalOp;
        ProcessInput normalInput;
        normalInput.cloud = cloud;
        const auto normalDesc = estimateOperationDescriptor(normalOp, normalInput);
        ParameterMap normalParams;
        for (const auto& spec : normalDesc.parameters)
            normalParams[spec.key] = spec.kind == ParameterKind::Integer
                                         ? ParameterValue{static_cast<std::int64_t>(std::llround(spec.defaultValue))}
                                         : ParameterValue{spec.defaultValue};
        // Poisson 面板中的线程数同时控制法线阶段，避免大点云仍落到保守默认线程数。
        normalParams["threads"] = ParameterValue{static_cast<std::int64_t>(std::max<std::int64_t>(1, intParam(params, "threads", processingThreadCount())))};

        auto normalResult = normalOp.run(
            normalInput, normalParams,
            [&](const ProgressInfo& info) {
                if (progress)
                    progress({info.progress * 0.12f, "1/6 PCA 法向估计"});
            },
            cancel);
        if (normalResult.cancelled)
            return {};
        if (!normalResult.success || !normalResult.cloud) {
            error = "自动法向估计失败";
            return {};
        }
        cloud = normalResult.cloud;
    } else {
        cloud = std::make_shared<PointCloud>(cloud->points());
    }

    if (cancel.cancelled())
        return {};

    return cloud;
}

#ifdef JMENGINE_HAS_POISSONRECON

using PoissonReal = float;
constexpr unsigned int kPoissonDim = 3;

class CloudOrientedStream final
    : public PoissonRecon::Reconstructor::InputOrientedSampleStream<PoissonReal, kPoissonDim> {
  public:
    explicit CloudOrientedStream(const PointCloud& cloud) : cloud_(cloud) {}

    void reset() override {
        cursor_ = 0;
    }

    bool read(PoissonRecon::Point<PoissonReal, kPoissonDim>& p,
              PoissonRecon::Point<PoissonReal, kPoissonDim>& n) override {
        while (cursor_ < cloud_.size()) {
            const auto& src = cloud_.points()[cursor_++];
            if (src.flags & PointDeleted)
                continue;
            const Vec3f nn = normalized(src.normal);
            if (normalLength(nn) < 0.5f)
                continue;
            p[0] = src.position.x;
            p[1] = src.position.y;
            p[2] = src.position.z;
            n[0] = nn.x;
            n[1] = nn.y;
            n[2] = nn.z;
            return true;
        }
        return false;
    }

  private:
    const PointCloud& cloud_;
    std::size_t cursor_{0};
};

class MeshVertexStream final
    : public PoissonRecon::Reconstructor::OutputLevelSetVertexStream<PoissonReal, kPoissonDim> {
  public:
    std::size_t size() const override {
        return points.size();
    }

    std::size_t write(const PoissonRecon::Point<PoissonReal, kPoissonDim>& p,
                      const PoissonRecon::Point<PoissonReal, kPoissonDim>& g, const PoissonReal& density) override {
        Point out;
        out.position = {p[0], p[1], p[2]};
        out.normal = normalized({g[0], g[1], g[2]});
        points.push_back(out);
        densities.push_back(density);
        return points.size() - 1;
    }

    PointCloud::Container points;
    std::vector<float> densities;
};

class TriangleFaceStream final : public PoissonRecon::Reconstructor::OutputFaceStream<2> {
  public:
    std::size_t size() const override {
        return polygons.size();
    }

    std::size_t write(const std::vector<PoissonRecon::node_index_type>& polygon) override {
        polygons.push_back(polygon);
        return polygons.size() - 1;
    }

    std::vector<std::vector<PoissonRecon::node_index_type>> polygons;
};


std::string quoteCommandArg(const std::string& value) {
    std::string out;
    out.reserve(value.size() + 2);
    out.push_back('"');
    for (char c : value) {
        if (c == '"') out += "\\\"";
        else out.push_back(c);
    }
    out.push_back('"');
    return out;
}

bool runHiddenCommand(const std::string& command) {
#ifdef _WIN32
    std::vector<char> mutableCommand(command.begin(), command.end());
    mutableCommand.push_back('\0');
    STARTUPINFOA si{};
    si.cb = sizeof(si);
    PROCESS_INFORMATION pi{};
    const BOOL ok = CreateProcessA(nullptr, mutableCommand.data(), nullptr, nullptr, FALSE,
                                   CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi);
    if (!ok) return false;
    WaitForSingleObject(pi.hProcess, INFINITE);
    DWORD exitCode = 1;
    GetExitCodeProcess(pi.hProcess, &exitCode);
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    return exitCode == 0;
#else
    return std::system(command.c_str()) == 0;
#endif
}

bool writeOfficialSurfaceTrimmerInput(const MeshVertexStream& vertices, const TriangleFaceStream& faces,
                                      const std::filesystem::path& fileName) {
    if (vertices.points.empty() || vertices.densities.size() != vertices.points.size() || faces.polygons.empty())
        return false;
    std::ofstream out(fileName, std::ios::binary);
    if (!out) return false;
    out << "ply\nformat ascii 1.0\n";
    out << "comment JMEngine official PoissonRecon SurfaceTrimmer input\n";
    out << "element vertex " << vertices.points.size() << "\n";
    out << "property float x\nproperty float y\nproperty float z\nproperty float value\n";
    out << "element face " << faces.polygons.size() << "\n";
    out << "property list uchar int vertex_indices\nend_header\n";
    out << std::setprecision(9);
    for (std::size_t i = 0; i < vertices.points.size(); ++i) {
        const auto& p = vertices.points[i].position;
        out << p.x << ' ' << p.y << ' ' << p.z << ' ' << vertices.densities[i] << '\n';
    }
    for (const auto& poly : faces.polygons) {
        out << poly.size();
        for (auto id : poly) out << ' ' << static_cast<unsigned long long>(id);
        out << '\n';
    }
    return static_cast<bool>(out);
}

std::shared_ptr<TriangleMesh> readOfficialSurfaceTrimmerOutput(const std::filesystem::path& fileName) {
    std::ifstream in(fileName, std::ios::binary);
    if (!in) return {};
    std::string line;
    if (!std::getline(in, line) || line != "ply") return {};
    bool ascii = false, inVertex = false;
    std::size_t vertexCount = 0, faceCount = 0;
    std::vector<std::string> vertexProperties;
    while (std::getline(in, line)) {
        if (line.rfind("format ", 0) == 0) ascii = line.find("ascii") != std::string::npos;
        else if (line.rfind("element vertex ", 0) == 0) {
            vertexCount = static_cast<std::size_t>(std::stoull(line.substr(15)));
            inVertex = true;
        } else if (line.rfind("element face ", 0) == 0) {
            faceCount = static_cast<std::size_t>(std::stoull(line.substr(13)));
            inVertex = false;
        } else if (inVertex && line.rfind("property ", 0) == 0 && line.find(" list ") == std::string::npos) {
            const auto pos = line.find_last_of(' ');
            if (pos != std::string::npos) vertexProperties.push_back(line.substr(pos + 1));
        } else if (line == "end_header") break;
    }
    if (!ascii || !vertexCount || !faceCount) return {};
    int ix=-1, iy=-1, iz=-1;
    for (int i=0; i<static_cast<int>(vertexProperties.size()); ++i) {
        if (vertexProperties[i] == "x") ix=i;
        else if (vertexProperties[i] == "y") iy=i;
        else if (vertexProperties[i] == "z") iz=i;
    }
    if (ix < 0 || iy < 0 || iz < 0) return {};

    PointCloud::Container points(vertexCount);
    for (std::size_t i=0; i<vertexCount; ++i) {
        if (!std::getline(in, line)) return {};
        std::istringstream ss(line);
        std::vector<double> values(vertexProperties.size(), 0.0);
        for (auto& v : values) if (!(ss >> v)) return {};
        points[i].position = {static_cast<float>(values[ix]), static_cast<float>(values[iy]), static_cast<float>(values[iz])};
        points[i].rgba = 0xffb8b8b8u;
        points[i].flags = PointValid;
    }
    std::vector<std::uint32_t> indices;
    indices.reserve(faceCount * 3u);
    for (std::size_t i=0; i<faceCount; ++i) {
        if (!std::getline(in, line)) return {};
        std::istringstream ss(line);
        std::size_t n=0;
        if (!(ss >> n) || n < 3) continue;
        std::vector<std::uint64_t> poly(n);
        for (auto& id : poly) if (!(ss >> id) || id >= vertexCount) return {};
        for (std::size_t k=1; k+1<n; ++k) {
            indices.push_back(static_cast<std::uint32_t>(poly[0]));
            indices.push_back(static_cast<std::uint32_t>(poly[k]));
            indices.push_back(static_cast<std::uint32_t>(poly[k+1]));
        }
    }
    if (indices.empty()) return {};
    auto cloud = std::make_shared<PointCloud>(std::move(points));
    return std::make_shared<TriangleMesh>(std::move(cloud), std::move(indices));
}

std::filesystem::path officialSurfaceTrimmerExecutable() {
#ifdef JMENGINE_OFFICIAL_SURFACE_TRIMMER_EXE
    std::filesystem::path configured(JMENGINE_OFFICIAL_SURFACE_TRIMMER_EXE);
    std::error_code ec;
    if (std::filesystem::exists(configured, ec)) return configured;
    const auto fileName = configured.filename();
#else
#ifdef _WIN32
    const std::filesystem::path fileName("JMEngine_surface_trimmer_official.exe");
#else
    const std::filesystem::path fileName("JMEngine_surface_trimmer_official");
#endif
#endif
#ifdef _WIN32
    std::array<char, 32768> buffer{};
    const DWORD n = GetModuleFileNameA(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
    if (n > 0 && n < buffer.size()) {
        const auto adjacent = std::filesystem::path(std::string(buffer.data(), n)).parent_path() / fileName;
        std::error_code ec;
        if (std::filesystem::exists(adjacent, ec)) return adjacent;
    }
#elif defined(__linux__)
    std::array<char, 4096> buffer{};
    const auto n = ::readlink("/proc/self/exe", buffer.data(), buffer.size()-1);
    if (n > 0) {
        const auto adjacent = std::filesystem::path(std::string(buffer.data(), static_cast<std::size_t>(n))).parent_path() / fileName;
        std::error_code ec;
        if (std::filesystem::exists(adjacent, ec)) return adjacent;
    }
#endif
    return {};
}

std::shared_ptr<TriangleMesh> runOfficialSurfaceTrimmer(const MeshVertexStream& vertexStream,
                                                        const TriangleFaceStream& faceStream,
                                                        double trimValue, double islandAreaRatio,
                                                        bool removeIslands) {
    if (!(trimValue > 0.0)) {
        // trim=0 means bypass SurfaceTrimmer but still convert the official Poisson output.
        std::vector<std::uint32_t> indices;
        indices.reserve(faceStream.polygons.size()*3u);
        for (const auto& poly : faceStream.polygons) {
            if (poly.size() < 3) continue;
            for (std::size_t k=1; k+1<poly.size(); ++k) {
                const auto a=static_cast<std::uint64_t>(poly[0]), b=static_cast<std::uint64_t>(poly[k]), c=static_cast<std::uint64_t>(poly[k+1]);
                if (a>=vertexStream.points.size() || b>=vertexStream.points.size() || c>=vertexStream.points.size()) continue;
                indices.push_back(static_cast<std::uint32_t>(a));
                indices.push_back(static_cast<std::uint32_t>(b));
                indices.push_back(static_cast<std::uint32_t>(c));
            }
        }
        if (indices.empty()) return {};
        auto cloud = std::make_shared<PointCloud>(vertexStream.points);
        return std::make_shared<TriangleMesh>(std::move(cloud), std::move(indices));
    }
    const auto trimmerExe = officialSurfaceTrimmerExecutable();
    if (trimmerExe.empty()) return {};
    std::error_code ec;
    const auto stamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    const auto tid = std::hash<std::thread::id>{}(std::this_thread::get_id());
    const auto dir = std::filesystem::temp_directory_path(ec) /
                     ("JMEngine_surface_trimmer_" + std::to_string(stamp) + "_" + std::to_string(tid));
    if (ec || !std::filesystem::create_directories(dir, ec)) return {};
    struct Cleanup { std::filesystem::path p; ~Cleanup(){ std::error_code e; std::filesystem::remove_all(p,e); } } cleanup{dir};
    const auto input = dir / "poisson_density.ply";
    const auto output = dir / "poisson_trimmed.ply";
    if (!writeOfficialSurfaceTrimmerInput(vertexStream, faceStream, input)) return {};

    std::ostringstream cmd;
    cmd << quoteCommandArg(trimmerExe.string())
        << " --in " << quoteCommandArg(input.string())
        << " --out " << quoteCommandArg(output.string())
        << " --trim " << std::setprecision(9) << trimValue
        << " --aRatio " << std::setprecision(9) << islandAreaRatio
        << " --ascii";
    if (removeIslands) cmd << " --removeIslands";
    if (!runHiddenCommand(cmd.str())) return {};
    return readOfficialSurfaceTrimmerOutput(output);
}

#endif // JMENGINE_HAS_POISSONRECON

// Surface trimming is delegated to the unmodified upstream SurfaceTrimmer executable.

double poissonMaxExtent(const PointCloud& cloud) {
    bool have = false;
    Vec3f mn{}, mx{};
    for (const auto& p : cloud.points()) {
        if (p.flags & PointDeleted)
            continue;
        if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(p.position.z))
            continue;
        if (!have) {
            mn = mx = p.position;
            have = true;
        } else {
            mn.x = std::min(mn.x, p.position.x);
            mn.y = std::min(mn.y, p.position.y);
            mn.z = std::min(mn.z, p.position.z);
            mx.x = std::max(mx.x, p.position.x);
            mx.y = std::max(mx.y, p.position.y);
            mx.z = std::max(mx.z, p.position.z);
        }
    }
    if (!have)
        return 0.0;
    return std::max({double(mx.x - mn.x), double(mx.y - mn.y), double(mx.z - mn.z)});
}

int poissonDepthFromPrecision(const PointCloud& cloud, double precisionMm) {
    // PointCloud 坐标单位为 m，UI 精度单位为 mm。
    const double precisionM = std::max(0.00001, precisionMm * 0.001);
    const double extent = std::max(precisionM, poissonMaxExtent(cloud));
    // 一个叶节点尺寸约为 extent / 2^depth。
    const double cells = std::max(1.0, extent / precisionM);
    const int depth = static_cast<int>(std::ceil(std::log2(cells)));
    // 12 已经足以覆盖约 4k 个单轴体素；更深层级对内存非常敏感。
    return std::clamp(depth, 6, 12);
}


struct Bounds3d {
    Vec3f min{};
    Vec3f max{};
    bool valid{false};
};

Bounds3d pointCloudBounds(const PointCloud& cloud) {
    Bounds3d b;
    for (const auto& p : cloud.points()) {
        if (p.flags & PointDeleted)
            continue;
        if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(p.position.z))
            continue;
        if (!b.valid) {
            b.min = b.max = p.position;
            b.valid = true;
        } else {
            b.min.x = std::min(b.min.x, p.position.x);
            b.min.y = std::min(b.min.y, p.position.y);
            b.min.z = std::min(b.min.z, p.position.z);
            b.max.x = std::max(b.max.x, p.position.x);
            b.max.y = std::max(b.max.y, p.position.y);
            b.max.z = std::max(b.max.z, p.position.z);
        }
    }
    return b;
}

Bounds3d meshBounds(const TriangleMesh& mesh) {
    Bounds3d b;
    const auto cloud = mesh.vertices();
    if (!cloud)
        return b;
    for (const auto& p : cloud->points()) {
        if (p.flags & PointDeleted)
            continue;
        if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(p.position.z))
            continue;
        if (!b.valid) {
            b.min = b.max = p.position;
            b.valid = true;
        } else {
            b.min.x = std::min(b.min.x, p.position.x);
            b.min.y = std::min(b.min.y, p.position.y);
            b.min.z = std::min(b.min.z, p.position.z);
            b.max.x = std::max(b.max.x, p.position.x);
            b.max.y = std::max(b.max.y, p.position.y);
            b.max.z = std::max(b.max.z, p.position.z);
        }
    }
    return b;
}

double axisExtent(float lo, float hi) {
    return std::max(0.0, static_cast<double>(hi) - static_cast<double>(lo));
}

bool preservePoissonInputScale(TriangleMesh& mesh, const PointCloud& input, double maxAllowedRelativeError = 0.005) {
    const Bounds3d inB = pointCloudBounds(input);
    const Bounds3d outB = meshBounds(mesh);
    if (!inB.valid || !outB.valid)
        return false;

    const double inX = axisExtent(inB.min.x, inB.max.x);
    const double inY = axisExtent(inB.min.y, inB.max.y);
    const double inZ = axisExtent(inB.min.z, inB.max.z);
    const double outX = axisExtent(outB.min.x, outB.max.x);
    const double outY = axisExtent(outB.min.y, outB.max.y);
    const double outZ = axisExtent(outB.min.z, outB.max.z);

    std::vector<double> ratios;
    ratios.reserve(3);
    if (inX > 1e-9 && outX > 1e-9) ratios.push_back(inX / outX);
    if (inY > 1e-9 && outY > 1e-9) ratios.push_back(inY / outY);
    if (inZ > 1e-9 && outZ > 1e-9) ratios.push_back(inZ / outZ);
    if (ratios.empty())
        return false;

    std::sort(ratios.begin(), ratios.end());
    const double scale = ratios[ratios.size() / 2]; // median is more robust than mean

    // If size is already close enough, do not touch geometry.
    if (std::abs(scale - 1.0) <= maxAllowedRelativeError)
        return false;

    // Only correct near-uniform global scaling. If the axes disagree too much,
    // the issue is geometric distortion / bad registration, not a global Poisson scale.
    double maxDev = 0.0;
    for (double r : ratios)
        maxDev = std::max(maxDev, std::abs(r - scale));
    if (maxDev > 0.02)
        return false;

    const Vec3f inC{
        0.5f * (inB.min.x + inB.max.x),
        0.5f * (inB.min.y + inB.max.y),
        0.5f * (inB.min.z + inB.max.z)};
    const Vec3f outC{
        0.5f * (outB.min.x + outB.max.x),
        0.5f * (outB.min.y + outB.max.y),
        0.5f * (outB.min.z + outB.max.z)};

    auto cloud = mesh.vertices();
    if (!cloud)
        return false;

    const float sf = static_cast<float>(scale);
    for (auto& p : cloud->points()) {
        if (p.flags & PointDeleted)
            continue;
        p.position.x = inC.x + (p.position.x - outC.x) * sf;
        p.position.y = inC.y + (p.position.y - outC.y) * sf;
        p.position.z = inC.z + (p.position.z - outC.z) * sf;
    }
    return true;
}

} // namespace

OperationDescriptor OctreePoissonOperation::descriptor() const {
    return {
        "poisson_octree",
        "工业泊松重建",
        "网格/重建",
        ModelKind::PointCloud,
        {
         // meshRecon 风格：用户控制物理分辨率，Depth/Scale 在 worker 内按输入 BBox 自动计算。
         {"resolution_mm", "目标重建精度", ParameterKind::Real, 5.0, 0.1, 100.0, 0.1, "mm"},
         {"max_depth", "最大八叉树深度", ParameterKind::Integer, 11, 7, 12, 1, ""},
         {"full_depth", "自适应八叉树深度", ParameterKind::Integer, 5, 3, 8, 1, ""},
         {"samples_per_node", "每节点最少样本", ParameterKind::Real, 0.5, 0.1, 20.0, 0.1, ""},
         {"point_weight", "插值权重", ParameterKind::Real, 4.0, 0.0, 20.0, 0.25, ""},
         {"iterations", "每层松弛迭代", ParameterKind::Integer, 8, 1, 32, 1, ""},
         {"cg_depth", "CG 深度", ParameterKind::Integer, 0, 0, 8, 1, ""},
         {"threads", "Poisson 线程数", ParameterKind::Integer, 16, 1, 64, 1, ""},
         {"preclean", "预清理无效点/空法线", ParameterKind::Boolean, 0, 0, 1, 1, ""},
         // 项目额外能力：不属于 MeshLab Poisson 参数。
         {"estimate_normals", "缺失时估算法向", ParameterKind::Boolean, 1, 0, 1, 1, ""},
         {"use_input_color", "使用输入颜色", ParameterKind::Boolean, 1, 0, 1, 1, ""},
         {"trim_value", "SurfaceTrimmer Trim", ParameterKind::Real, 7.0, 0.0, 12.0, 0.25, ""},
         {"island_area_ratio", "SurfaceTrimmer 岛面积比", ParameterKind::Real, 0.001, 0.0, 0.05, 0.0001, ""},
         {"surface_trim_remove_islands", "SurfaceTrimmer 删除小岛", ParameterKind::Boolean, 0, 0, 1, 1, ""},
         {"linear_fit", "线性顶点拟合", ParameterKind::Boolean, 1, 0, 1, 1, ""},
         {"cleanup", "重建后基础清理", ParameterKind::Boolean, 1, 0, 1, 1, ""},
         {"remove_small_components", "删除小连通网格", ParameterKind::Boolean, 0, 0, 1, 1, ""},
         {"preserve_input_scale", "兼容旧版尺寸校正", ParameterKind::Boolean, 0, 0, 1, 1, ""}},
        OutputPolicy::AddModelOnKindChange};
}

ProcessResult OctreePoissonOperation::run(const ProcessInput& input, const ParameterMap& params,
                                          const ProgressCallback& progress, const CancelToken& cancel) const {
    ProcessResult result;
    result.inputPoints = input.cloud ? input.cloud->activeCount() : 0;

    std::string error;
    auto cloud = ensurePoissonNormals(input, params, progress, cancel, error);
    if (cancel.cancelled()) {
        result.cancelled = true;
        return result;
    }
    if (!cloud) {
        result.message = error.empty() ? "泊松输入预处理失败" : error;
        return result;
    }

#ifndef JMENGINE_HAS_POISSONRECON
    result.message = "工业泊松后端未配置。请先运行 tools/vendor_poissonrecon.py，将官方 PoissonRecon 18.76 放入 "
                     "third_party/PoissonRecon，然后重新 Configure。";
    return result;
#else
    if (progress)
        progress({0.14f, "2/6 初始化 PoissonRecon"});

    // 官方 ThreadPool 默认使用全部 hardware_concurrency。构建系统会给官方头文件加入 SetNumThreads，
    // 这里统一使用整个工程的 CPU-1 策略。
    PoissonRecon::ThreadPool::SetNumThreads(static_cast<unsigned int>(processingThreadCount()));
#ifdef _OPENMP
    PoissonRecon::ThreadPool::ParallelizationType = PoissonRecon::ThreadPool::ParallelType::OPEN_MP;
#else
    PoissonRecon::ThreadPool::ParallelizationType = PoissonRecon::ThreadPool::ParallelType::ASYNC;
#endif

    using Recon = PoissonRecon::Reconstructor::Poisson;
    constexpr unsigned int femSig =
        PoissonRecon::FEMDegreeAndBType<Recon::DefaultFEMDegree, Recon::DefaultFEMBoundary>::Signature;
    using FemSigs = PoissonRecon::IsotropicUIntPack<kPoissonDim, femSig>;
    using Solver = Recon::Solver<PoissonReal, kPoissonDim, FemSigs>;
    using Implicit = PoissonRecon::Reconstructor::Implicit<PoissonReal, kPoissonDim, FemSigs>;

    typename Recon::SolutionParameters<PoissonReal> solverParams;

    // meshRecon 风格：目标物理分辨率 -> 自动 Depth + 最小必要 Scale。
    // Scale 只负责给 BBox 留约一个叶节点的边界，不再固定放大 5%/10%，
    // 从源头减少开放扫描外围形成“大鼓包/包络壳”的空间。
    const double extent = poissonMaxExtent(*cloud);
    if (!(extent > 0.0) || !std::isfinite(extent)) {
        result.message = "Poisson 输入包围盒无效";
        return result;
    }
    const double unitPerMm = extent >= 50.0 ? 1.0 : 0.001; // 兼容 mm / m 坐标工程
    const double resolutionMm = std::max(0.1, realParam(params, "resolution_mm", 5.0));
    const double resolutionModel = std::max(extent * 1e-7, resolutionMm * unitPerMm);
    const int maxDepth = static_cast<int>(std::clamp<std::int64_t>(intParam(params, "max_depth", 11), 7, 12));

    // 留一个目标分辨率单元的 padding；大型模型通常得到约 1.01，而不是固定 1.05/1.10。
    const double autoScale = std::clamp(1.0 + 2.0 * resolutionModel / extent, 1.005, 1.08);
    const double cells = std::max(1.0, extent * autoScale / resolutionModel);
    const int requestedDepth = std::clamp(static_cast<int>(std::ceil(std::log2(cells))), 6, maxDepth);
    const int requestedFullDepth =
        static_cast<int>(std::clamp<std::int64_t>(intParam(params, "full_depth", 5), 3, 8));

    solverParams.depth = static_cast<unsigned int>(requestedDepth);
    solverParams.fullDepth = static_cast<unsigned int>(std::min(requestedDepth, requestedFullDepth));
    solverParams.samplesPerNode = static_cast<PoissonReal>(realParam(params, "samples_per_node", 0.5));
    solverParams.pointWeight = static_cast<PoissonReal>(realParam(params, "point_weight", 4.0));
    solverParams.scale = static_cast<PoissonReal>(autoScale);
    solverParams.iters =
        static_cast<unsigned int>(std::clamp<std::int64_t>(intParam(params, "iterations", 8), 1, 32));
 
    solverParams.verbose = false;
    const int poissonThreads =
        static_cast<int>(std::clamp<std::int64_t>(intParam(params, "threads", 16), 1, 64));
    (void)poissonThreads; // PoissonRecon build may use its configured OpenMP/thread pool internally.

    CloudOrientedStream sampleStream(*cloud);
    if (progress)
        progress({0.2f, "3/6 自适应八叉树 FEM 求解"});

    std::unique_ptr<Implicit> implicit;
    try {
        implicit.reset(Solver::Solve(sampleStream, solverParams));
    } catch (const std::exception& e) {
        result.message = std::string("PoissonRecon 求解失败: ") + e.what();
        return result;
    } catch (...) {
        result.message = "PoissonRecon 求解失败: 未知异常";
        return result;
    }

    if (cancel.cancelled()) {
        result.cancelled = true;
        return result;
    }
    if (!implicit) {
        result.message = "PoissonRecon 未生成隐式场";
        return result;
    }

    if (progress)
        progress({0.86f, "4/6 提取等值面与密度"});
    MeshVertexStream vertexStream;
    TriangleFaceStream faceStream;
    PoissonRecon::Reconstructor::LevelSetExtractionParameters extractionParams;
    extractionParams.outputDensity = true;
    extractionParams.forceManifold = true;
    extractionParams.polygonMesh = false;
    extractionParams.linearFit = boolParam(params, "linear_fit", true);
    extractionParams.verbose = false;

    try {
        implicit->extractLevelSet(vertexStream, faceStream, extractionParams);
    } catch (const std::exception& e) {
        result.message = std::string("PoissonRecon 等值面提取失败: ") + e.what();
        return result;
    } catch (...) {
        result.message = "PoissonRecon 等值面提取失败: 未知异常";
        return result;
    }

    if (cancel.cancelled()) {
        result.cancelled = true;
        return result;
    }

    const double trimValue = std::max(0.0, realParam(params, "trim_value", 7.0));
    const double islandAreaRatio = std::max(0.0, realParam(params, "island_area_ratio", 0.001));
    const bool surfaceTrimRemoveIslands = boolParam(params, "surface_trim_remove_islands", false);
    if (progress)
        progress({0.89f, "5/7 官方 SurfaceTrimmer"});
    auto mesh = runOfficialSurfaceTrimmer(vertexStream, faceStream, trimValue, islandAreaRatio,
                                          surfaceTrimRemoveIslands);
    if (!mesh || mesh->empty()) {
        result.message = "泊松重建输出为空，请检查法向或降低 SurfaceTrimmer Trim 值";
        return result;
    }

    // V4: 不再对 Poisson 成品网格做“到原始点云距离 -> 删面”的后裁剪。
    // 该策略在稀疏/高曲率/大尺度扫描上会产生锯齿边界、碎片和误删主体。
    // 开放区域通过 Poisson 输出的 density + SurfaceTrimmer 连续等值切割处理。
    // 坐标/尺寸校正仍保留，但它不删除任何三角形。
    bool scaleCorrected = false;
    if (boolParam(params, "preserve_input_scale", false)) {
        if (progress)
            progress({0.92f, "5/7 校正到输入点云坐标/尺寸"});
        scaleCorrected = preservePoissonInputScale(*mesh, *input.cloud);
    }

    if (boolParam(params, "cleanup", true)) {
        if (progress)
            progress({0.95f, "6/7 网格清理"});
        MeshCleanupOperation cleanup;
        ProcessInput ci;
        ci.mesh = mesh;
        ParameterMap cp;
        cp["remove_degenerate"] = true;
        cp["remove_duplicate"] = true;
        cp["merge_vertices"] = false;
        cp["remove_unreferenced"] = true;
        // V4 默认不删除小连通域。复杂扫描中的薄结构/雕花经常本来就是小连通片，
        // 自动删除会造成“去网格很乱”。需要时由用户明确开启。
        cp["remove_small_components"] = boolParam(params, "remove_small_components", false);
        cp["min_triangles"] = static_cast<std::int64_t>(
            std::clamp<std::size_t>(mesh->activeTriangleCount() / 2000u, 30u, 1000u));
        auto cr = cleanup.run(ci, cp, {}, cancel);
        if (cr.success && cr.mesh)
            mesh = cr.mesh;
    }

    // Poisson 输入法线只用于隐式场求解。最终等值面经过 Density Trim / Cleanup 后，
    // 顶点集合与输入点云已经不是一一对应，因此必须在最终拓扑上重新计算渲染/后处理法线。
    if (progress)
        progress({0.975f, "6/7 重建最终网格法线"});
    if (!JMEngine::recomputeVertexNormals(*mesh)) {
        result.message = "泊松网格已生成，但最终顶点法线重建失败";
        return result;
    }

    if (boolParam(params, "use_input_color", true)) {
        if (progress)
            progress({0.988f, "6/7 传递输入点云颜色"});
        if (!transferInputColors(*cloud, *mesh, cancel)) {
            if (cancel.cancelled()) {
                result.cancelled = true;
                return result;
            }
            result.message = "泊松网格已生成，但输入颜色传递失败";
            return result;
        }
    }

    result.success = true;
    result.geometryChanged = true;
    result.topologyChanged = true;
    result.mesh = mesh;
    result.outputPoints = mesh->vertices() ? mesh->vertices()->activeCount() : 0;
    result.outputTriangles = mesh->activeTriangleCount();
    {
        std::ostringstream oss;
        oss << "Poisson完成: resolution=" << std::fixed << std::setprecision(3) << resolutionMm
            << " mm, autoDepth=" << requestedDepth
            << ", autoScale=" << std::setprecision(5) << autoScale
            << ", trim=" << realParam(params, "trim_value", 7.0)
            << ", islandRatio=" << realParam(params, "island_area_ratio", 0.001);
        result.message = oss.str();
    }

    // 输出健康度检查放在 worker 线程内完成，避免 UI 线程扫描百万三角形。
    ProcessInput diagnosticInput;
    diagnosticInput.cloud = mesh->vertices();
    diagnosticInput.mesh = mesh;
    const auto diagnostics = analyzeModel(diagnosticInput);
    result.degenerateTriangles = diagnostics.degenerateTriangles;
    result.boundaryEdges = diagnostics.boundaryEdges;
    result.nonManifoldEdges = diagnostics.nonManifoldEdges;
    result.connectedComponents = diagnostics.connectedComponents;
    {
        std::ostringstream msg;
        msg << "工业泊松重建完成，Depth=" << requestedDepth << "，FullDepth=" << requestedFullDepth;
        msg << "，SurfaceTrimmer Trim=" << std::fixed << std::setprecision(2) << trimValue;
        if (scaleCorrected)
            msg << "，已校正到输入点云尺寸";
        if (diagnostics.nonManifoldEdges != 0)
            msg << "（检测到非流形边）";
        result.message = msg.str();
    }
    if (progress)
        progress({1.0f, "7/7 完成"});
    return result;
#endif
}

bool industrialPoissonAvailable() noexcept {
#ifdef JMENGINE_HAS_POISSONRECON
    return true;
#else
    return false;
#endif
}

} // namespace JMEngine::processing
