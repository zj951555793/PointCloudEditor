#include <pceditor/processing/Operations.h>
#include <pceditor/processing/Parallel.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <vector>

#ifdef PCEDITOR_USE_OPENMP
#include <omp.h>
#endif

namespace pceditor::processing {
namespace {

Vec3f addP(const Vec3f& a, const Vec3f& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
Vec3f subP(const Vec3f& a, const Vec3f& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
Vec3f mulP(const Vec3f& a, float s) {
    return {a.x * s, a.y * s, a.z * s};
}
float dotP(const Vec3f& a, const Vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
float lenP(const Vec3f& v) {
    return std::sqrt(std::max(0.0f, dotP(v, v)));
}
Vec3f normP(const Vec3f& v) {
    const float l = lenP(v);
    return l > 1e-20f ? mulP(v, 1.0f / l) : Vec3f{};
}

struct OctreeNode {
    Vec3f center{};
    float half{0.0f};
    std::uint32_t begin{0};
    std::uint32_t count{0};
    std::array<std::int32_t, 8> child{{-1, -1, -1, -1, -1, -1, -1, -1}};
    std::uint8_t level{0};
    bool leaf{true};

    Vec3f pointCenter{};
    Vec3f normal{};
    double phi{0.0};
};

class AdaptiveOctree {
  public:
    AdaptiveOctree(const PointCloud& cloud, int maxDepth, int minDepth, int samplesPerNode, float scale)
        : cloud_(cloud), maxDepth_(std::max(2, maxDepth)), minDepth_(std::max(1, std::min(minDepth, maxDepth_))),
          samplesPerNode_(std::max(1, samplesPerNode)) {
        bool init = false;
        Vec3f mn{}, mx{};
        for (std::uint32_t i = 0; i < cloud.size(); ++i) {
            const auto& p = cloud.points()[i];
            if (p.flags & PointDeleted)
                continue;
            order_.push_back(i);
            if (!init) {
                mn = mx = p.position;
                init = true;
            } else {
                mn.x = std::min(mn.x, p.position.x);
                mn.y = std::min(mn.y, p.position.y);
                mn.z = std::min(mn.z, p.position.z);
                mx.x = std::max(mx.x, p.position.x);
                mx.y = std::max(mx.y, p.position.y);
                mx.z = std::max(mx.z, p.position.z);
            }
        }
        if (!init)
            return;
        Vec3f c{(mn.x + mx.x) * 0.5f, (mn.y + mx.y) * 0.5f, (mn.z + mx.z) * 0.5f};
        float span = std::max({mx.x - mn.x, mx.y - mn.y, mx.z - mn.z, 1e-6f});
        float half = span * 0.5f * std::max(1.001f, scale);
        nodes_.reserve(std::min<std::size_t>(order_.size() * 2 + 1, 4000000));
        scratch_.resize(order_.size());
        root_ = build(c, half, 0, 0, static_cast<std::uint32_t>(order_.size()));
        collectLeaves();
    }

    bool empty() const noexcept {
        return root_ < 0;
    }
    const std::vector<OctreeNode>& nodes() const noexcept {
        return nodes_;
    }
    std::vector<OctreeNode>& nodes() noexcept {
        return nodes_;
    }
    const std::vector<std::int32_t>& leaves() const noexcept {
        return leaves_;
    }
    const std::vector<std::uint32_t>& order() const noexcept {
        return order_;
    }

    std::int32_t findLeaf(Vec3f p) const {
        if (root_ < 0)
            return -1;
        std::int32_t idx = root_;
        while (idx >= 0) {
            const auto& n = nodes_[static_cast<std::size_t>(idx)];
            if (n.leaf)
                return idx;
            const int oct = (p.x >= n.center.x ? 1 : 0) | (p.y >= n.center.y ? 2 : 0) | (p.z >= n.center.z ? 4 : 0);
            const auto next = n.child[static_cast<std::size_t>(oct)];
            if (next < 0)
                return idx;
            idx = next;
        }
        return -1;
    }

  private:
    std::int32_t build(const Vec3f& center, float half, int level, std::uint32_t begin, std::uint32_t count) {
        const std::int32_t index = static_cast<std::int32_t>(nodes_.size());
        nodes_.push_back({});
        auto& node = nodes_.back();
        node.center = center;
        node.half = half;
        node.begin = begin;
        node.count = count;
        node.level = static_cast<std::uint8_t>(level);

        const bool split =
            level < maxDepth_ && (level < minDepth_ || count > static_cast<std::uint32_t>(samplesPerNode_));
        if (!split || count <= 1)
            return index;

        std::array<std::uint32_t, 8> counts{};
        for (std::uint32_t k = 0; k < count; ++k) {
            const auto& p = cloud_.points()[order_[begin + k]].position;
            const int oct = (p.x >= center.x ? 1 : 0) | (p.y >= center.y ? 2 : 0) | (p.z >= center.z ? 4 : 0);
            ++counts[static_cast<std::size_t>(oct)];
        }
        int nonEmpty = 0;
        for (auto c : counts)
            nonEmpty += c != 0;
        if (nonEmpty <= 1 && level >= minDepth_)
            return index;

        std::array<std::uint32_t, 8> offsets{};
        offsets[0] = begin;
        for (int i = 1; i < 8; ++i)
            offsets[static_cast<std::size_t>(i)] =
                offsets[static_cast<std::size_t>(i - 1)] + counts[static_cast<std::size_t>(i - 1)];
        auto cursor = offsets;
        for (std::uint32_t k = 0; k < count; ++k) {
            const auto id = order_[begin + k];
            const auto& p = cloud_.points()[id].position;
            const int oct = (p.x >= center.x ? 1 : 0) | (p.y >= center.y ? 2 : 0) | (p.z >= center.z ? 4 : 0);
            scratch_[cursor[static_cast<std::size_t>(oct)]++] = id;
        }
        std::copy(scratch_.begin() + begin, scratch_.begin() + begin + count, order_.begin() + begin);

        nodes_[static_cast<std::size_t>(index)].leaf = false;
        const float childHalf = half * 0.5f;
        for (int oct = 0; oct < 8; ++oct) {
            const auto cnt = counts[static_cast<std::size_t>(oct)];
            if (!cnt)
                continue;
            Vec3f cc = center;
            cc.x += (oct & 1) ? childHalf : -childHalf;
            cc.y += (oct & 2) ? childHalf : -childHalf;
            cc.z += (oct & 4) ? childHalf : -childHalf;
            const auto childIndex = build(cc, childHalf, level + 1, offsets[static_cast<std::size_t>(oct)], cnt);
            nodes_[static_cast<std::size_t>(index)].child[static_cast<std::size_t>(oct)] = childIndex;
        }
        return index;
    }

    void collectLeaves() {
        leaves_.clear();
        for (std::size_t i = 0; i < nodes_.size(); ++i)
            if (nodes_[i].leaf && nodes_[i].count > 0)
                leaves_.push_back(static_cast<std::int32_t>(i));
    }

    const PointCloud& cloud_;
    int maxDepth_{9};
    int minDepth_{5};
    int samplesPerNode_{2};
    std::int32_t root_{-1};
    std::vector<OctreeNode> nodes_;
    std::vector<std::uint32_t> order_;
    std::vector<std::uint32_t> scratch_;
    std::vector<std::int32_t> leaves_;
};

Vec3f interpolate(const Vec3f& a, const Vec3f& b, double va, double vb) {
    const double denom = va - vb;
    const double t = std::abs(denom) > 1e-20 ? std::clamp(va / denom, 0.0, 1.0) : 0.5;
    return {static_cast<float>(a.x + (b.x - a.x) * t), static_cast<float>(a.y + (b.y - a.y) * t),
            static_cast<float>(a.z + (b.z - a.z) * t)};
}

void emitTetra(const std::array<Vec3f, 4>& p, const std::array<double, 4>& v, PointCloud::Container& pts,
               std::vector<std::uint32_t>& indices) {
    std::array<int, 4> inside{};
    int nInside = 0;
    for (int i = 0; i < 4; ++i)
        if (v[static_cast<std::size_t>(i)] <= 0.0)
            inside[static_cast<std::size_t>(nInside++)] = i;
    if (nInside == 0 || nInside == 4)
        return;

    auto addVertex = [&](int a, int b) {
        Point q;
        q.position = interpolate(p[static_cast<std::size_t>(a)], p[static_cast<std::size_t>(b)],
                                 v[static_cast<std::size_t>(a)], v[static_cast<std::size_t>(b)]);
        const auto id = static_cast<std::uint32_t>(pts.size());
        pts.push_back(q);
        return id;
    };

    if (nInside == 1 || nInside == 3) {
        const bool invert = nInside == 3;
        int a = invert ? -1 : inside[0];
        if (invert) {
            for (int i = 0; i < 4; ++i) {
                bool found = false;
                for (int k = 0; k < 3; ++k)
                    found = found || inside[static_cast<std::size_t>(k)] == i;
                if (!found)
                    a = i;
            }
        }
        std::array<int, 3> other{};
        int oi = 0;
        for (int i = 0; i < 4; ++i)
            if (i != a)
                other[static_cast<std::size_t>(oi++)] = i;
        const auto i0 = addVertex(a, other[0]);
        const auto i1 = addVertex(a, other[1]);
        const auto i2 = addVertex(a, other[2]);
        if (invert)
            indices.insert(indices.end(), {i0, i2, i1});
        else
            indices.insert(indices.end(), {i0, i1, i2});
        return;
    }

    int a = inside[0], b = inside[1];
    std::array<int, 2> out{};
    int oi = 0;
    for (int i = 0; i < 4; ++i)
        if (i != a && i != b)
            out[static_cast<std::size_t>(oi++)] = i;
    const auto i0 = addVertex(a, out[0]);
    const auto i1 = addVertex(a, out[1]);
    const auto i2 = addVertex(b, out[0]);
    const auto i3 = addVertex(b, out[1]);
    indices.insert(indices.end(), {i0, i1, i2, i2, i1, i3});
}

} // namespace

OperationDescriptor OctreePoissonOperation::descriptor() const {
    return {"poisson_octree",
            "八叉树泊松重建",
            "网格/重建",
            ModelKind::PointCloud,
            {{"depth", "八叉树深度", ParameterKind::Integer, 9, 6, 12, 1, ""},
             {"min_depth", "最小深度", ParameterKind::Integer, 5, 3, 10, 1, ""},
             {"samples_per_node", "每节点样本", ParameterKind::Real, 1.5, 1.0, 8.0, 0.25, ""},
             {"point_weight", "点约束权重", ParameterKind::Real, 4.0, 0.1, 20.0, 0.25, ""},
             {"scale", "包围盒扩展", ParameterKind::Real, 1.1, 1.01, 1.5, 0.01, ""},
             {"iterations", "求解迭代", ParameterKind::Integer, 80, 10, 500, 10, ""},
             {"estimate_normals", "缺失时估算法向", ParameterKind::Boolean, 1, 0, 1, 1, ""},
             {"orient_normals", "法向一致朝外", ParameterKind::Boolean, 1, 0, 1, 1, ""}}};
}

ProcessResult OctreePoissonOperation::run(const ProcessInput& input, const ParameterMap& params,
                                          const ProgressCallback& progress, const CancelToken& cancel) const {
    ProcessResult result;
    if (!input.cloud || input.cloud->activeCount() < 16) {
        result.message = "泊松重建需要有效点云";
        return result;
    }

    std::shared_ptr<PointCloud> cloud = input.cloud;
    bool hasNormals = false;
    for (const auto& p : cloud->points()) {
        if ((p.flags & PointDeleted) == 0 && lenP(p.normal) > 0.5f) {
            hasNormals = true;
            break;
        }
    }
    if (!hasNormals && boolParam(params, "estimate_normals", true)) {
        auto normalOp = NormalEstimationOperation{};
        ProcessInput ni;
        ni.cloud = cloud;
        ParameterMap np;
        const auto desc = estimateOperationDescriptor(normalOp, ni);
        for (const auto& spec : desc.parameters)
            np[spec.key] = spec.kind == ParameterKind::Integer
                               ? ParameterValue{static_cast<std::int64_t>(std::llround(spec.defaultValue))}
                               : ParameterValue{spec.defaultValue};
        auto nr = normalOp.run(
            ni, np,
            [&](const ProgressInfo& info) {
                if (progress)
                    progress({info.progress * 0.18f, "估算法向"});
            },
            cancel);
        if (nr.cancelled) {
            result.cancelled = true;
            return result;
        }
        if (!nr.success || !nr.cloud) {
            result.message = "法向估计失败";
            return result;
        }
        cloud = nr.cloud;
    }

    if (cancel.cancelled()) {
        result.cancelled = true;
        return result;
    }

    Vec3f centroid{};
    std::size_t active = 0;
    for (const auto& p : cloud->points()) {
        if (p.flags & PointDeleted)
            continue;
        centroid = addP(centroid, p.position);
        ++active;
    }
    centroid = active ? mulP(centroid, 1.0f / static_cast<float>(active)) : centroid;
    if (boolParam(params, "orient_normals", true)) {
        auto oriented = std::make_shared<PointCloud>(cloud->points());
        const int nt = processingThreadCount();
#ifdef PCEDITOR_USE_OPENMP
#pragma omp parallel for schedule(static) num_threads(nt)
#endif
        for (long long i = 0; i < static_cast<long long>(oriented->size()); ++i) {
            auto& p = oriented->points()[static_cast<std::size_t>(i)];
            if (p.flags & PointDeleted)
                continue;
            if (dotP(p.normal, subP(p.position, centroid)) < 0.0f)
                p.normal = mulP(p.normal, -1.0f);
        }
        cloud = oriented;
    }

    const int depth = static_cast<int>(intParam(params, "depth", 9));
    const int minDepth = static_cast<int>(intParam(params, "min_depth", 5));
    const int samples = std::max(1, static_cast<int>(std::ceil(realParam(params, "samples_per_node", 1.5))));
    const float scale = static_cast<float>(realParam(params, "scale", 1.1));
    const double pointWeight = realParam(params, "point_weight", 4.0);
    const int iterations = static_cast<int>(intParam(params, "iterations", 80));

    if (progress)
        progress({0.2f, "构建自适应八叉树"});
    AdaptiveOctree tree(*cloud, depth, minDepth, samples, scale);
    if (tree.empty() || tree.leaves().empty()) {
        result.message = "八叉树构建失败";
        return result;
    }

    auto& nodes = tree.nodes();
    const auto& order = tree.order();
    const auto& leaves = tree.leaves();
    const int nt = processingThreadCount();
#ifdef PCEDITOR_USE_OPENMP
#pragma omp parallel for schedule(dynamic, 64) num_threads(nt)
#endif
    for (long long li = 0; li < static_cast<long long>(leaves.size()); ++li) {
        auto& leaf = nodes[static_cast<std::size_t>(leaves[static_cast<std::size_t>(li)])];
        Vec3f pc{}, n{};
        std::size_t count = 0;
        for (std::uint32_t k = 0; k < leaf.count; ++k) {
            const auto& p = cloud->points()[order[leaf.begin + k]];
            if (p.flags & PointDeleted)
                continue;
            pc = addP(pc, p.position);
            n = addP(n, p.normal);
            ++count;
        }
        if (count) {
            pc = mulP(pc, 1.0f / static_cast<float>(count));
            n = normP(n);
        } else {
            pc = leaf.center;
        }
        leaf.pointCenter = pc;
        leaf.normal = n;
        leaf.phi = dotP(n, subP(leaf.center, pc));
    }

    if (progress)
        progress({0.38f, "组装稀疏泊松方程"});
    std::vector<std::array<std::int32_t, 6>> neighbors(leaves.size());
    std::vector<std::array<double, 6>> weights(leaves.size());
    std::vector<double> rhs(leaves.size(), 0.0);
    for (auto& n : neighbors)
        n.fill(-1);
    const Vec3f axis[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

    std::vector<std::int32_t> nodeToLeaf(nodes.size(), -1);
    for (std::size_t i = 0; i < leaves.size(); ++i)
        nodeToLeaf[static_cast<std::size_t>(leaves[i])] = static_cast<std::int32_t>(i);

#ifdef PCEDITOR_USE_OPENMP
#pragma omp parallel for schedule(dynamic, 64) num_threads(nt)
#endif
    for (long long li = 0; li < static_cast<long long>(leaves.size()); ++li) {
        const auto nodeIndex = leaves[static_cast<std::size_t>(li)];
        const auto& leaf = nodes[static_cast<std::size_t>(nodeIndex)];
        double div = 0.0;
        for (int a = 0; a < 6; ++a) {
            const Vec3f q = addP(leaf.center, mulP(axis[a], leaf.half * 1.02f));
            const auto neighborNode = tree.findLeaf(q);
            if (neighborNode < 0 || neighborNode == nodeIndex)
                continue;
            const auto nli = nodeToLeaf[static_cast<std::size_t>(neighborNode)];
            if (nli < 0)
                continue;
            neighbors[static_cast<std::size_t>(li)][static_cast<std::size_t>(a)] = nli;
            const auto& nb = nodes[static_cast<std::size_t>(neighborNode)];
            const double distance = std::max(1e-9f, lenP(subP(nb.center, leaf.center)));
            const double w = 1.0 / (distance * distance);
            weights[static_cast<std::size_t>(li)][static_cast<std::size_t>(a)] = w;
            const Vec3f dv = subP(nb.normal, leaf.normal);
            div += dotP(dv, axis[a]) / distance;
        }
        rhs[static_cast<std::size_t>(li)] = div;
    }

    std::vector<double> phi(leaves.size(), 0.0), next(phi.size(), 0.0);
    for (std::size_t i = 0; i < leaves.size(); ++i)
        phi[i] = nodes[static_cast<std::size_t>(leaves[i])].phi;

    for (int iter = 0; iter < iterations; ++iter) {
        if (cancel.cancelled()) {
            result.cancelled = true;
            return result;
        }
#ifdef PCEDITOR_USE_OPENMP
#pragma omp parallel for schedule(static) num_threads(nt)
#endif
        for (long long li = 0; li < static_cast<long long>(leaves.size()); ++li) {
            const auto& leaf = nodes[static_cast<std::size_t>(leaves[static_cast<std::size_t>(li)])];
            double sumW = pointWeight;
            double sum = pointWeight * dotP(leaf.normal, subP(leaf.center, leaf.pointCenter)) -
                         rhs[static_cast<std::size_t>(li)];
            for (int a = 0; a < 6; ++a) {
                const auto j = neighbors[static_cast<std::size_t>(li)][static_cast<std::size_t>(a)];
                if (j < 0)
                    continue;
                const double w = weights[static_cast<std::size_t>(li)][static_cast<std::size_t>(a)];
                sumW += w;
                sum += w * phi[static_cast<std::size_t>(j)];
            }
            next[static_cast<std::size_t>(li)] = sumW > 0.0 ? sum / sumW : phi[static_cast<std::size_t>(li)];
        }
        phi.swap(next);
        if (progress && (iter % 8 == 0 || iter + 1 == iterations))
            progress({0.42f + 0.34f * float(iter + 1) / float(std::max(1, iterations)), "求解八叉树泊松方程"});
    }
    for (std::size_t i = 0; i < leaves.size(); ++i)
        nodes[static_cast<std::size_t>(leaves[i])].phi = phi[i];

    auto fieldAt = [&](const Vec3f& x) -> double {
        const auto ni = tree.findLeaf(x);
        if (ni < 0)
            return 1.0;
        const auto& leaf = nodes[static_cast<std::size_t>(ni)];
        const double plane = dotP(leaf.normal, subP(x, leaf.pointCenter));
        return 0.7 * plane + 0.3 * leaf.phi;
    };

    if (progress)
        progress({0.78f, "提取等值面"});
    PointCloud::Container vertices;
    std::vector<std::uint32_t> indices;
    vertices.reserve(leaves.size() * 2);
    indices.reserve(leaves.size() * 6);
    constexpr int tetra[6][4] = {{0, 5, 1, 6}, {0, 1, 2, 6}, {0, 2, 3, 6}, {0, 3, 7, 6}, {0, 7, 4, 6}, {0, 4, 5, 6}};
    constexpr int cornerSign[8][3] = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
                                      {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1}};

    for (std::size_t li = 0; li < leaves.size(); ++li) {
        if (cancel.cancelled()) {
            result.cancelled = true;
            return result;
        }
        const auto& leaf = nodes[static_cast<std::size_t>(leaves[li])];
        std::array<Vec3f, 8> cp{};
        std::array<double, 8> cv{};
        bool negative = false, positive = false;
        for (int c = 0; c < 8; ++c) {
            cp[static_cast<std::size_t>(c)] = {
                leaf.center.x + cornerSign[c][0] * leaf.half,
                leaf.center.y + cornerSign[c][1] * leaf.half,
                leaf.center.z + cornerSign[c][2] * leaf.half,
            };
            cv[static_cast<std::size_t>(c)] = fieldAt(cp[static_cast<std::size_t>(c)]);
            negative = negative || cv[static_cast<std::size_t>(c)] <= 0.0;
            positive = positive || cv[static_cast<std::size_t>(c)] > 0.0;
        }
        if (!negative || !positive)
            continue;
        for (const auto& t : tetra) {
            std::array<Vec3f, 4> tp = {cp[t[0]], cp[t[1]], cp[t[2]], cp[t[3]]};
            std::array<double, 4> tv = {cv[t[0]], cv[t[1]], cv[t[2]], cv[t[3]]};
            emitTetra(tp, tv, vertices, indices);
        }
    }

    if (indices.empty()) {
        result.message = "泊松求解完成，但未提取到有效等值面；请提高 Depth 或检查法向";
        return result;
    }

    auto vertexCloud = std::make_shared<PointCloud>(std::move(vertices));
    auto mesh = std::make_shared<TriangleMesh>(vertexCloud, std::move(indices));
    result.inputPoints = input.cloud->activeCount();
    result.outputPoints = vertexCloud->activeCount();
    result.outputTriangles = mesh->triangleCount();
    result.mesh = mesh;
    result.cloud = vertexCloud;
    result.success = true;
    result.geometryChanged = true;
    result.topologyChanged = true;
    result.message = "八叉树泊松重建完成";
    if (progress)
        progress({1.0f, "完成"});
    return result;
}

} // namespace pceditor::processing
