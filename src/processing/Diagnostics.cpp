#include <JMEngine/processing/Diagnostics.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <iomanip>
#include <unordered_map>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace JMEngine::processing {
namespace {

bool finite(const Vec3f& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

double lengthSquared(const Vec3f& v) {
    return double(v.x) * v.x + double(v.y) * v.y + double(v.z) * v.z;
}

std::uint64_t edgeKey(std::uint32_t a, std::uint32_t b) {
    if (a > b)
        std::swap(a, b);
    return (std::uint64_t(a) << 32u) | std::uint64_t(b);
}

std::size_t availablePhysicalMemory() {
#ifdef _WIN32
    MEMORYSTATUSEX state{};
    state.dwLength = sizeof(state);
    return GlobalMemoryStatusEx(&state) ? static_cast<std::size_t>(state.ullAvailPhys) : 0u;
#else
    const long pages = sysconf(_SC_AVPHYS_PAGES);
    const long pageSize = sysconf(_SC_PAGESIZE);
    if (pages <= 0 || pageSize <= 0)
        return 0u;
    return static_cast<std::size_t>(pages) * static_cast<std::size_t>(pageSize);
#endif
}

std::string bytesText(std::size_t bytes) {
    std::ostringstream out;
    const double gib = static_cast<double>(bytes) / (1024.0 * 1024.0 * 1024.0);
    if (gib >= 1.0)
        out << std::fixed << std::setprecision(2) << gib << " GiB";
    else
        out << std::fixed << std::setprecision(1) << (static_cast<double>(bytes) / (1024.0 * 1024.0)) << " MiB";
    return out.str();
}

std::size_t estimatePoissonBytes(std::size_t points, int depth, double samplesPerNode) {
    if (!points)
        return 0u;

    // PoissonRecon 是自适应稀疏八叉树，不是完整 8^depth 体素网格。
    // 旧估算按很高的 nodeMultiplier 计算，在扫描点云上会严重高估并错误阻止任务启动。
    // 这里仅作为 UI 提示值：按输入点 + 随 depth 增长的稀疏节点工作集估算。
    samplesPerNode = std::max(1.0, samplesPerNode);
    const double depthExtra = static_cast<double>(std::max(0, depth - 8));
    const double bytesPerPoint = (360.0 + depthExtra * 96.0) / std::sqrt(samplesPerNode);
    const double fixedOverhead = 128.0 * 1024.0 * 1024.0;
    const double bytes = static_cast<double>(points) * bytesPerPoint + fixedOverhead;
    return static_cast<std::size_t>(
        std::min(bytes, static_cast<double>(std::numeric_limits<std::size_t>::max())));
}

ModelDiagnostics analyzePoissonInputFast(const ProcessInput& input) {
    ModelDiagnostics d;
    d.kind = ModelKind::PointCloud;
    const auto cloud = input.cloud ? input.cloud : (input.mesh ? input.mesh->vertices() : nullptr);
    if (!cloud || cloud->empty())
        return d;

    // 预检运行在“提交后台”之前，因此必须是有界成本。这里只均匀抽样，
    // 真正完整的数据检查/法向处理留到 worker 线程。
    const auto& pts = cloud->points();
    d.points = pts.size();

    constexpr std::size_t kMaxSamples = 8192;
    const std::size_t step = std::max<std::size_t>(1, pts.size() / kMaxSamples);
    std::size_t sampled = 0, valid = 0, normals = 0;
    bool haveBounds = false;
    Vec3f mn{}, mx{};

    for (std::size_t i = 0; i < pts.size() && sampled < kMaxSamples; i += step, ++sampled) {
        const auto& p = pts[i];
        if (p.flags & PointDeleted)
            continue;
        if (!finite(p.position))
            continue;
        ++valid;
        if (finite(p.normal) && lengthSquared(p.normal) > 0.25)
            ++normals;
        if (!haveBounds) {
            mn = mx = p.position;
            haveBounds = true;
        } else {
            mn.x = std::min(mn.x, p.position.x);
            mn.y = std::min(mn.y, p.position.y);
            mn.z = std::min(mn.z, p.position.z);
            mx.x = std::max(mx.x, p.position.x);
            mx.y = std::max(mx.y, p.position.y);
            mx.z = std::max(mx.z, p.position.z);
        }
    }

    d.validNormals = normals;
    d.normalCoverage = valid ? static_cast<double>(normals) / static_cast<double>(valid) : 0.0;
    if (sampled > valid)
        d.invalidPoints = sampled - valid; // sampled count only; used as a warning signal

    if (haveBounds) {
        const double dx = double(mx.x) - mn.x;
        const double dy = double(mx.y) - mn.y;
        const double dz = double(mx.z) - mn.z;
        d.bboxDiagonal = std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return d;
}

int estimatedDepthFromPrecision(const ModelDiagnostics& d, const ParameterMap& params) {
    const double precisionMm = std::clamp(realParam(params, "precision_mm", 0.5), 0.05, 10.0);
    const double precisionM = precisionMm * 0.001;
    const double extent = std::max(precisionM, d.bboxDiagonal);
    return std::clamp(static_cast<int>(std::ceil(std::log2(std::max(1.0, extent / precisionM)))), 6, 12);
}

} // namespace

ModelDiagnostics analyzeModel(const ProcessInput& input) {
    ModelDiagnostics d;
    d.kind = input.mesh ? ModelKind::TriangleMesh : ModelKind::PointCloud;
    const auto cloud = input.mesh ? input.mesh->vertices() : input.cloud;
    if (!cloud)
        return d;

    d.deletedPoints = cloud->deletedCount();
    bool haveBounds = false;
    Vec3f mn{}, mx{};
    for (const auto& p : cloud->points()) {
        if (p.flags & PointDeleted)
            continue;
        ++d.points;
        if (!finite(p.position)) {
            ++d.invalidPoints;
            continue;
        }
        if (finite(p.normal) && lengthSquared(p.normal) > 0.25)
            ++d.validNormals;
        if (!haveBounds) {
            mn = mx = p.position;
            haveBounds = true;
        } else {
            mn.x = std::min(mn.x, p.position.x);
            mn.y = std::min(mn.y, p.position.y);
            mn.z = std::min(mn.z, p.position.z);
            mx.x = std::max(mx.x, p.position.x);
            mx.y = std::max(mx.y, p.position.y);
            mx.z = std::max(mx.z, p.position.z);
        }
    }
    d.normalCoverage = d.points ? static_cast<double>(d.validNormals) / static_cast<double>(d.points) : 0.0;
    if (haveBounds) {
        const double dx = double(mx.x) - mn.x;
        const double dy = double(mx.y) - mn.y;
        const double dz = double(mx.z) - mn.z;
        d.bboxDiagonal = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double volume = std::max(1e-18, dx * dy * dz);
        d.estimatedSpacing = d.points ? std::cbrt(volume / static_cast<double>(d.points)) : 0.0;
    }

    if (!input.mesh)
        return d;

    d.triangles = input.mesh->activeTriangleCount();
    d.deletedTriangles = input.mesh->deletedTriangleCount();
    const auto& indices = input.mesh->indices();
    const auto& points = cloud->points();
    std::unordered_map<std::uint64_t, std::uint32_t> edgeUse;
    edgeUse.reserve(d.triangles * 3u / 2u + 1u);

    std::vector<std::uint32_t> parent(points.size());
    for (std::size_t i = 0; i < parent.size(); ++i)
        parent[i] = static_cast<std::uint32_t>(i);
    auto findRoot = [&](std::uint32_t x) {
        std::uint32_t r = x;
        while (parent[r] != r)
            r = parent[r];
        while (parent[x] != x) {
            const auto n = parent[x];
            parent[x] = r;
            x = n;
        }
        return r;
    };
    auto unite = [&](std::uint32_t a, std::uint32_t b) {
        a = findRoot(a);
        b = findRoot(b);
        if (a != b)
            parent[b] = a;
    };

    std::vector<std::uint8_t> used(points.size(), 0u);
    const std::size_t triCount = indices.size() / 3u;
    for (std::size_t t = 0; t < triCount; ++t) {
        if (!input.mesh->triangleActive(static_cast<TriangleId>(t)))
            continue;
        const std::uint32_t a = indices[t * 3u + 0u];
        const std::uint32_t b = indices[t * 3u + 1u];
        const std::uint32_t c = indices[t * 3u + 2u];
        if (a >= points.size() || b >= points.size() || c >= points.size()) {
            ++d.degenerateTriangles;
            continue;
        }
        const auto& pa = points[a].position;
        const auto& pb = points[b].position;
        const auto& pc = points[c].position;
        const Vec3f ab{pb.x - pa.x, pb.y - pa.y, pb.z - pa.z};
        const Vec3f ac{pc.x - pa.x, pc.y - pa.y, pc.z - pa.z};
        const Vec3f cross{ab.y * ac.z - ab.z * ac.y, ab.z * ac.x - ab.x * ac.z, ab.x * ac.y - ab.y * ac.x};
        if (a == b || b == c || a == c || !finite(pa) || !finite(pb) || !finite(pc) || lengthSquared(cross) < 1e-20)
            ++d.degenerateTriangles;
        ++edgeUse[edgeKey(a, b)];
        ++edgeUse[edgeKey(b, c)];
        ++edgeUse[edgeKey(c, a)];
        used[a] = used[b] = used[c] = 1u;
        unite(a, b);
        unite(b, c);
    }
    for (const auto& kv : edgeUse) {
        if (kv.second == 1u)
            ++d.boundaryEdges;
        else if (kv.second > 2u)
            ++d.nonManifoldEdges;
    }
    std::unordered_map<std::uint32_t, std::uint8_t> roots;
    for (std::size_t i = 0; i < used.size(); ++i)
        if (used[i])
            roots[findRoot(static_cast<std::uint32_t>(i))] = 1u;
    d.connectedComponents = roots.size();
    return d;
}

ProcessingPreflight preflightOperation(const IProcessingOperation& operation, const ProcessInput& input,
                                       const ParameterMap& params) {
    ProcessingPreflight p;
    const auto desc = operation.descriptor();
    ProcessInput diagnosticInput = input;
    if (desc.id == "poisson_octree") {
        // Poisson 的确认/提交路径保持 O(1)：不扫描点云、不确认法线、不计算包围盒。
        // 完整检查统一移动到 worker，避免千万点云在 UI 线程卡住。
        p.diagnostics.kind = ModelKind::PointCloud;
        const auto cloud = input.cloud ? input.cloud : (input.mesh ? input.mesh->vertices() : nullptr);
        p.diagnostics.points = cloud ? cloud->size() : 0u;
    } else {
        p.diagnostics = analyzeModel(diagnosticInput);
    }
    p.availableMemoryBytes = availablePhysicalMemory();

    if (p.diagnostics.invalidPoints > 0)
        p.warnings.push_back("模型包含 NaN/Inf 坐标，建议先清理无效点");

    if (desc.id == "poisson_octree") {
        // Resolution -> 实际 Depth 需要真实包围盒，放到 worker 内计算。
        // 预检按用户允许的 max_depth 做保守内存上限估算，避免低估 OOM 风险。
        p.requestedDepth = static_cast<int>(std::clamp<std::int64_t>(intParam(params, "max_depth", 11), 7, 12));
        p.recommendedDepth = p.requestedDepth;
        const double samples = realParam(params, "samples_per_node", 1.5);

        // 前置阶段只按点数给参考值，不扫描法线/包围盒，也不阻止启动。
        p.estimatedWorkingSetBytes = estimatePoissonBytes(p.diagnostics.points, p.requestedDepth, samples);
        if (p.diagnostics.points < 1000)
            p.warnings.push_back("点数较少，Poisson 重建质量可能不稳定");
        p.allowed = true;
    }
    return p;
}

std::string diagnosticsSummary(const ModelDiagnostics& d) {
    std::ostringstream out;
    out << "有效点: " << d.points << "\n"
        << "已删除点: " << d.deletedPoints << "\n"
        << "无效坐标: " << d.invalidPoints << "\n"
        << "有效法向: " << d.validNormals << " (" << std::fixed << std::setprecision(1) << d.normalCoverage * 100.0
        << "%)\n"
        << "BBox 对角线: " << std::setprecision(6) << d.bboxDiagonal << "\n"
        << "估计点间距: " << d.estimatedSpacing;
    if (d.kind == ModelKind::TriangleMesh) {
        out << "\n有效三角形: " << d.triangles << "\n已删除三角形: " << d.deletedTriangles
            << "\n退化三角形: " << d.degenerateTriangles << "\n边界边: " << d.boundaryEdges
            << "\n非流形边: " << d.nonManifoldEdges << "\n连通域: " << d.connectedComponents;
    }
    return out.str();
}

std::string preflightSummary(const ProcessingPreflight& p) {
    std::ostringstream out;
    out << diagnosticsSummary(p.diagnostics);
    if (p.estimatedWorkingSetBytes)
        out << "\n预计处理工作集: " << bytesText(p.estimatedWorkingSetBytes);
    if (p.availableMemoryBytes)
        out << "\n当前可用物理内存: " << bytesText(p.availableMemoryBytes);
    if (p.requestedDepth)
        out << "\n自动 Depth: " << p.requestedDepth;
    if (!p.warnings.empty()) {
        out << "\n\n警告:";
        for (const auto& w : p.warnings)
            out << "\n- " << w;
    }
    return out.str();
}

} // namespace JMEngine::processing
