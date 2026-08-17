#include <JMEngine/processing/Processing.h>
#include <JMEngine/processing/Operations.h>
#include <cmath>
namespace JMEngine::processing {
std::unique_ptr<IProcessingOperation> createOperation(const std::string& id) {
    if (id == "voxel")
        return std::make_unique<VoxelDownsampleOperation>();
    if (id == "radius_outlier")
        return std::make_unique<RadiusOutlierOperation>();
    if (id == "statistical_outlier")
        return std::make_unique<StatisticalOutlierOperation>();
    if (id == "small_cluster")
        return std::make_unique<SmallClusterOperation>();
    if (id == "normal_estimation")
        return std::make_unique<NormalEstimationOperation>();
    if (id == "mesh_cleanup")
        return std::make_unique<MeshCleanupOperation>();
    if (id == "laplacian")
        return std::make_unique<LaplacianSmoothOperation>();
    if (id == "taubin")
        return std::make_unique<TaubinSmoothOperation>();
    if (id == "qem_decimate")
        return std::make_unique<QemDecimateOperation>();
    if (id == "hole_fill")
        return std::make_unique<HoleFillOperation>();
    if (id == "poisson_octree")
        return std::make_unique<OctreePoissonOperation>();
    return {};
}
std::vector<OperationDescriptor> builtinOperations() {
    const char* ids[] = {
        "voxel",     "radius_outlier", "statistical_outlier", "small_cluster", "normal_estimation", "mesh_cleanup",
        "laplacian", "taubin",         "qem_decimate",        "hole_fill",     "poisson_octree"};
    std::vector<OperationDescriptor> out;
    for (auto* id : ids) {
        auto op = createOperation(id);
        if (op)
            out.push_back(op->descriptor());
    }
    return out;
}
} // namespace JMEngine::processing

namespace JMEngine::processing {
namespace {
struct ModelStatistics {
    double diagonal{1.0};
    double spacing{0.001};
    double averageEdge{0.001};
    std::size_t points{0};
    std::size_t triangles{0};
};

ModelStatistics computeStatistics(const ProcessInput& input) {
    ModelStatistics s;
    const auto cloud = input.mesh ? input.mesh->vertices() : input.cloud;
    if (!cloud || cloud->empty())
        return s;

    bool initialized = false;
    Vec3f mn{}, mx{};
    std::size_t active = 0;
    for (const auto& p : cloud->points()) {
        if (p.flags & PointDeleted)
            continue;
        if (!initialized) {
            mn = mx = p.position;
            initialized = true;
        } else {
            mn.x = std::min(mn.x, p.position.x);
            mn.y = std::min(mn.y, p.position.y);
            mn.z = std::min(mn.z, p.position.z);
            mx.x = std::max(mx.x, p.position.x);
            mx.y = std::max(mx.y, p.position.y);
            mx.z = std::max(mx.z, p.position.z);
        }
        ++active;
    }
    if (!initialized)
        return s;

    const double dx = double(mx.x) - mn.x;
    const double dy = double(mx.y) - mn.y;
    const double dz = double(mx.z) - mn.z;
    s.diagonal = std::max(1e-9, std::sqrt(dx * dx + dy * dy + dz * dz));
    s.points = active;
    // 点云是 2D 表面采样，不应使用 cbrt(volume/N) 的体采样公式。
    // 用包围盒表面积/N 的平方根得到更合理的邻域尺度，尤其适合大件扫描。
    const double area = std::max(1e-18, 2.0 * (dx * dy + dx * dz + dy * dz));
    s.spacing = std::max(s.diagonal * 1e-7, std::sqrt(area / double(std::max<std::size_t>(1, active))));

    if (input.mesh && input.mesh->triangleCount() > 0) {
        s.triangles = input.mesh->activeTriangleCount();
        const auto& ix = input.mesh->indices();
        const auto& pts = cloud->points();
        const std::size_t triCount = ix.size() / 3;
        const std::size_t sampleCount = std::min<std::size_t>(triCount, 20000);
        const std::size_t step = std::max<std::size_t>(1, triCount / std::max<std::size_t>(1, sampleCount));
        double sum = 0.0;
        std::size_t count = 0;
        auto edge = [&](std::uint32_t a, std::uint32_t b) {
            if (a >= pts.size() || b >= pts.size())
                return;
            const auto& pa = pts[a].position;
            const auto& pb = pts[b].position;
            const double ex = double(pa.x) - pb.x;
            const double ey = double(pa.y) - pb.y;
            const double ez = double(pa.z) - pb.z;
            sum += std::sqrt(ex * ex + ey * ey + ez * ez);
            ++count;
        };
        for (std::size_t t = 0; t < triCount; t += step) {
            if (!input.mesh->triangleActive(static_cast<TriangleId>(t)))
                continue;
            const std::size_t b = t * 3;
            edge(ix[b], ix[b + 1]);
            edge(ix[b + 1], ix[b + 2]);
            edge(ix[b + 2], ix[b]);
        }
        if (count)
            s.averageEdge = sum / double(count);
        else
            s.averageEdge = s.spacing;
        s.spacing = std::min(std::max(s.spacing, s.averageEdge * 0.25), s.averageEdge * 2.0);
    } else {
        s.averageEdge = s.spacing;
    }
    return s;
}

void setDefault(OperationDescriptor& d, const char* key, double value, double minValue = -1.0, double maxValue = -1.0,
                double step = -1.0) {
    for (auto& p : d.parameters) {
        if (p.key != key)
            continue;
        if (minValue >= 0.0)
            p.minValue = minValue;
        if (maxValue >= 0.0)
            p.maxValue = maxValue;
        if (step >= 0.0)
            p.step = step;
        p.defaultValue = std::clamp(value, p.minValue, p.maxValue);
        return;
    }
}
} // namespace

OperationDescriptor estimateOperationDescriptor(const IProcessingOperation& operation, const ProcessInput& input) {
    auto d = operation.descriptor();

    // Poisson 的主要参数现在是用户直接输入的“目标精度(mm)”。
    // 打开参数窗口时不再为了猜 Depth 在 UI 线程遍历整份百万/千万点云。
    if (d.id == "poisson_octree") {
        // Poisson 直接采用 MeshLab 风格参数，不在打开对话框时遍历点云自动猜 Depth。
        setDefault(d, "resolution_mm", 5.0);
        setDefault(d, "max_depth", 11);
        setDefault(d, "full_depth", 5);
        setDefault(d, "samples_per_node", 0.5);
        setDefault(d, "point_weight", 4.0);
        setDefault(d, "iterations", 8);
        setDefault(d, "threads", 16);
        return d;
    }

    const auto s = computeStatistics(input);
    // 工程兼容 m 坐标与 mm 坐标。大型扫描（对角线 >= 50）直接认为模型单位就是 mm。
    const double unitToMm = s.diagonal >= 50.0 ? 1.0 : 1000.0;
    const double spacingMm = std::max(1e-5, s.spacing * unitToMm);
    const double edgeMm = std::max(1e-5, s.averageEdge * unitToMm);

    if (d.id == "voxel") {
        setDefault(d, "voxel_mm", spacingMm * 1.5, spacingMm * 0.1, spacingMm * 20.0, spacingMm * 0.1);
    } else if (d.id == "radius_outlier") {
        setDefault(d, "radius_mm", spacingMm * 2.5, spacingMm * 0.5, spacingMm * 30.0, spacingMm * 0.25);
        setDefault(d, "min_neighbors", s.points > 1000000 ? 8 : 5);
    } else if (d.id == "statistical_outlier") {
        const double k = std::clamp(12.0 + std::log10(double(std::max<std::size_t>(10, s.points))) * 2.0, 12.0, 40.0);
        setDefault(d, "k", k);
        setDefault(d, "stddev", 1.5);
    } else if (d.id == "small_cluster") {
        setDefault(d, "radius_mm", spacingMm * 2.5, spacingMm * 0.5, spacingMm * 30.0, spacingMm * 0.25);
        setDefault(d, "min_points", std::clamp(double(s.points) * 0.0005, 20.0, 5000.0));
    } else if (d.id == "normal_estimation") {
        // KNN 对非均匀扫描密度更稳定，不再根据 bbox/点数推导固定半径。
        setDefault(d, "knn", s.points > 3000000 ? 20.0 : 24.0, 6.0, 64.0, 1.0);
        setDefault(d, "max_distance_mm", 0.0, 0.0, std::max(1000.0, spacingMm * 50.0), std::max(0.5, spacingMm * 0.5));
    } else if (d.id == "mesh_cleanup") {
        setDefault(d, "merge_tolerance_mm", edgeMm * 0.02, edgeMm * 0.0001, edgeMm * 0.25, edgeMm * 0.005);
        setDefault(d, "min_triangles", std::clamp(double(s.triangles) * 0.0001, 10.0, 5000.0));
    } else if (d.id == "laplacian" || d.id == "taubin") {
        setDefault(d, "iterations", s.triangles > 3000000 ? 5 : 10);
    } else if (d.id == "qem_decimate") {
        setDefault(d, "ratio", s.triangles > 3000000 ? 0.35 : 0.5);
    } else if (d.id == "hole_fill") {
        setDefault(d, "max_edges",
                   std::clamp(std::sqrt(double(std::max<std::size_t>(1, s.triangles))) * 0.5, 32.0, 2000.0));
    }
    return d;
}
} // namespace JMEngine::processing
