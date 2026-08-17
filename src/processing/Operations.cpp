#include <JMEngine/processing/Operations.h>
#include <JMEngine/MeshUtils.h>
#include <JMEngine/processing/Parallel.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#ifdef JMENGINE_USE_OPENMP
#include <omp.h>
#endif

namespace JMEngine::processing {
namespace {
struct Key {
    int x, y, z;
    bool operator==(const Key& o) const noexcept {
        return x == o.x && y == o.y && z == o.z;
    }
};
struct KeyHash {
    size_t operator()(const Key& k) const noexcept {
        size_t h = 1469598103934665603ull;
        auto f = [&](int v) {
            h ^= static_cast<unsigned>(v);
            h *= 1099511628211ull;
        };
        f(k.x);
        f(k.y);
        f(k.z);
        return h;
    }
};
inline Key keyOf(const Vec3f& p, float s) {
    return {static_cast<int>(std::floor(p.x / s)), static_cast<int>(std::floor(p.y / s)),
            static_cast<int>(std::floor(p.z / s))};
}
inline float dist2(const Vec3f& a, const Vec3f& b) {
    float x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
    return x * x + y * y + z * z;
}
inline Vec3f add(Vec3f a, Vec3f b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline Vec3f sub(Vec3f a, Vec3f b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline Vec3f mul(Vec3f a, float s) {
    return {a.x * s, a.y * s, a.z * s};
}
inline float dot(Vec3f a, Vec3f b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vec3f cross(Vec3f a, Vec3f b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Vec3f norm(Vec3f v) {
    float l = std::sqrt(std::max(0.0f, dot(v, v)));
    return l > 1e-20f ? mul(v, 1.0f / l) : Vec3f{};
}

// 工程历史数据同时存在“坐标单位=m”和“坐标单位=mm”。
// 大型扫描模型（包围盒对角线 >= 50）按 mm 坐标处理，避免 UI 的 mm 参数被错误缩小 1000 倍。
double cloudDiagonal(const PointCloud& c) {
    bool valid = false;
    Vec3f lo{}, hi{};
    for (const auto& p : c.points()) {
        if (p.flags & PointDeleted)
            continue;
        if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(p.position.z))
            continue;
        if (!valid) { lo = hi = p.position; valid = true; }
        else {
            lo.x = std::min(lo.x, p.position.x); lo.y = std::min(lo.y, p.position.y); lo.z = std::min(lo.z, p.position.z);
            hi.x = std::max(hi.x, p.position.x); hi.y = std::max(hi.y, p.position.y); hi.z = std::max(hi.z, p.position.z);
        }
    }
    if (!valid) return 0.0;
    const double x = double(hi.x) - lo.x, y = double(hi.y) - lo.y, z = double(hi.z) - lo.z;
    return std::sqrt(x*x + y*y + z*z);
}

float millimetersToCloudUnits(const PointCloud& c, double mm) {
    return static_cast<float>(cloudDiagonal(c) >= 50.0 ? mm : mm * 0.001);
}
inline void prog(const ProgressCallback& cb, float p, const char* s) {
    if (cb)
        cb({p, s});
}

std::unordered_map<Key, std::vector<std::uint32_t>, KeyHash> grid(const PointCloud& c, float cell) {
    std::unordered_map<Key, std::vector<std::uint32_t>, KeyHash> g;
    g.reserve(c.size() / 4 + 1);
    for (std::uint32_t i = 0; i < c.size(); ++i) {
        const auto& p = c.points()[i];
        if (p.flags & PointDeleted)
            continue;
        g[keyOf(p.position, cell)].push_back(i);
    }
    return g;
}

std::unordered_map<Key, std::vector<std::uint32_t>, KeyHash> gridParallel(const PointCloud& c, float cell) {
#ifndef JMENGINE_USE_OPENMP
    return grid(c, cell);
#else
    const int nt = std::max(1, processingThreadCount());
    if (nt <= 1 || c.size() < 100000)
        return grid(c, cell);

    using Grid = std::unordered_map<Key, std::vector<std::uint32_t>, KeyHash>;
    std::vector<Grid> locals(static_cast<std::size_t>(nt));

#pragma omp parallel num_threads(nt)
    {
        const int tid = omp_get_thread_num();
        auto& local = locals[static_cast<std::size_t>(tid)];
        local.reserve(c.size() / static_cast<std::size_t>(nt * 4) + 1);

#pragma omp for schedule(static)
        for (long long ii = 0; ii < static_cast<long long>(c.size()); ++ii) {
            const auto i = static_cast<std::uint32_t>(ii);
            const auto& p = c.points()[i];
            if (p.flags & PointDeleted)
                continue;
            local[keyOf(p.position, cell)].push_back(i);
        }
    }

    Grid g;
    std::size_t cellCount = 0;
    for (const auto& local : locals)
        cellCount += local.size();
    g.reserve(cellCount + 1);

    for (auto& local : locals) {
        for (auto& kv : local) {
            auto [it, inserted] = g.try_emplace(kv.first);
            auto& dst = it->second;
            if (inserted) {
                dst = std::move(kv.second);
            } else {
                dst.insert(dst.end(),
                           std::make_move_iterator(kv.second.begin()),
                           std::make_move_iterator(kv.second.end()));
            }
        }
    }
    return g;
#endif
}


class PointKdTree {
  public:
    explicit PointKdTree(const PointCloud& cloud) : cloud_(cloud) {
        indices_.reserve(cloud.size());
        for (std::uint32_t i = 0; i < cloud.size(); ++i) {
            const auto& p = cloud.points()[i];
            if ((p.flags & PointDeleted) == 0 && std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
                std::isfinite(p.position.z))
                indices_.push_back(i);
        }
        nodes_.reserve(indices_.size());
        root_ = build(0, indices_.size(), 0);
    }

    template <std::size_t MaxK>
    std::size_t knn(std::uint32_t queryId, std::size_t k, float maxDistance,
                    std::array<std::uint32_t, MaxK>& outIds, std::array<float, MaxK>& outD2) const {
        if (root_ < 0 || k == 0)
            return 0;
        k = std::min<std::size_t>(k, MaxK);
        const float maxD2 = maxDistance > 0.0f ? maxDistance * maxDistance : std::numeric_limits<float>::infinity();
        std::size_t count = 0;
        query(root_, queryId, cloud_.points()[queryId].position, k, maxD2, outIds, outD2, count);
        return count;
    }

  private:
    struct Node {
        std::uint32_t point = 0;
        int left = -1;
        int right = -1;
        std::uint8_t axis = 0;
    };

    static float coord(const Vec3f& p, int axis) {
        return axis == 0 ? p.x : (axis == 1 ? p.y : p.z);
    }

    int build(std::size_t begin, std::size_t end, int depth) {
        if (begin >= end)
            return -1;
        const int axis = depth % 3;
        const std::size_t mid = begin + (end - begin) / 2;
        std::nth_element(indices_.begin() + static_cast<std::ptrdiff_t>(begin),
                         indices_.begin() + static_cast<std::ptrdiff_t>(mid),
                         indices_.begin() + static_cast<std::ptrdiff_t>(end),
                         [&](std::uint32_t a, std::uint32_t b) {
                             return coord(cloud_.points()[a].position, axis) < coord(cloud_.points()[b].position, axis);
                         });
        const int nodeId = static_cast<int>(nodes_.size());
        nodes_.push_back({indices_[mid], -1, -1, static_cast<std::uint8_t>(axis)});
        const int left = build(begin, mid, depth + 1);
        const int right = build(mid + 1, end, depth + 1);
        nodes_[static_cast<std::size_t>(nodeId)].left = left;
        nodes_[static_cast<std::size_t>(nodeId)].right = right;
        return nodeId;
    }

    template <std::size_t MaxK>
    void query(int nodeId, std::uint32_t queryId, const Vec3f& target, std::size_t k, float maxD2,
               std::array<std::uint32_t, MaxK>& ids, std::array<float, MaxK>& d2s, std::size_t& count) const {
        if (nodeId < 0)
            return;
        const Node& node = nodes_[static_cast<std::size_t>(nodeId)];
        const auto& q = cloud_.points()[node.point].position;
        const float delta = coord(target, node.axis) - coord(q, node.axis);
        const int nearChild = delta <= 0.0f ? node.left : node.right;
        const int farChild = delta <= 0.0f ? node.right : node.left;

        query(nearChild, queryId, target, k, maxD2, ids, d2s, count);

        if (node.point != queryId) {
            const float d2 = dist2(target, q);
            if (d2 <= maxD2) {
                if (count < k) {
                    ids[count] = node.point;
                    d2s[count] = d2;
                    ++count;
                } else {
                    std::size_t worst = 0;
                    for (std::size_t i = 1; i < count; ++i)
                        if (d2s[i] > d2s[worst])
                            worst = i;
                    if (d2 < d2s[worst]) {
                        ids[worst] = node.point;
                        d2s[worst] = d2;
                    }
                }
            }
        }

        float limit = maxD2;
        if (count >= k) {
            float worst = d2s[0];
            for (std::size_t i = 1; i < count; ++i)
                worst = std::max(worst, d2s[i]);
            limit = std::min(limit, worst);
        }
        if (delta * delta <= limit)
            query(farChild, queryId, target, k, maxD2, ids, d2s, count);
    }

    const PointCloud& cloud_;
    std::vector<std::uint32_t> indices_;
    std::vector<Node> nodes_;
    int root_ = -1;
};

template <class Fn>
void forNeighbors(const PointCloud& c, const std::unordered_map<Key, std::vector<std::uint32_t>, KeyHash>& g,
                  std::uint32_t id, float r, Fn fn) {
    const auto& p = c.points()[id].position;
    Key k = keyOf(p, r);
    float rr = r * r;
    for (int dz = -1; dz <= 1; ++dz)
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                auto it = g.find({k.x + dx, k.y + dy, k.z + dz});
                if (it == g.end())
                    continue;
                for (auto j : it->second)
                    if (j != id && dist2(p, c.points()[j].position) <= rr)
                        fn(j);
            }
}

std::vector<std::vector<std::uint32_t>> vertexNeighbors(const TriangleMesh& m) {
    auto c = m.vertices();
    std::vector<std::vector<std::uint32_t>> n(c ? c->size() : 0);
    const auto& ix = m.indices();
    for (std::size_t t = 0; t + 2 < ix.size(); t += 3) {
        auto a = ix[t], b = ix[t + 1], c0 = ix[t + 2];
        if (a >= n.size() || b >= n.size() || c0 >= n.size())
            continue;
        n[a].push_back(b);
        n[a].push_back(c0);
        n[b].push_back(a);
        n[b].push_back(c0);
        n[c0].push_back(a);
        n[c0].push_back(b);
    }
    for (auto& v : n) {
        std::sort(v.begin(), v.end());
        v.erase(std::unique(v.begin(), v.end()), v.end());
    }
    return n;
}
std::vector<unsigned char> boundaryVertices(const TriangleMesh& m) {
    struct E {
        uint32_t a, b;
        bool operator==(const E& o) const noexcept {
            return a == o.a && b == o.b;
        }
    };
    struct H {
        size_t operator()(const E& e) const noexcept {
            return (static_cast<size_t>(e.a) << 32) ^ e.b;
        }
    };
    std::unordered_map<E, unsigned, H> count;
    count.reserve(m.indices().size());
    for (size_t t = 0; t + 2 < m.indices().size(); t += 3) {
        uint32_t v[3] = {m.indices()[t], m.indices()[t + 1], m.indices()[t + 2]};
        for (int e = 0; e < 3; ++e) {
            uint32_t a = v[e], b = v[(e + 1) % 3];
            if (a > b)
                std::swap(a, b);
            ++count[{a, b}];
        }
    }
    std::vector<unsigned char> out(m.vertices() ? m.vertices()->size() : 0, 0);
    for (auto& kv : count)
        if (kv.second == 1) {
            if (kv.first.a < out.size())
                out[kv.first.a] = 1;
            if (kv.first.b < out.size())
                out[kv.first.b] = 1;
        }
    return out;
}

Vec3f smallestEigenVectorSymmetric3x3(double a00, double a01, double a02, double a11, double a12, double a22) {
    double a[3][3] = {{a00, a01, a02}, {a01, a11, a12}, {a02, a12, a22}};
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int iter = 0; iter < 12; ++iter) {
        int p = 0, q = 1;
        double m = std::fabs(a[0][1]);
        if (std::fabs(a[0][2]) > m) {
            p = 0;
            q = 2;
            m = std::fabs(a[0][2]);
        }
        if (std::fabs(a[1][2]) > m) {
            p = 1;
            q = 2;
            m = std::fabs(a[1][2]);
        }
        if (m < 1e-14)
            break;
        double phi = .5 * std::atan2(2 * a[p][q], a[q][q] - a[p][p]);
        double c = std::cos(phi), sn = std::sin(phi);
        for (int k = 0; k < 3; ++k) {
            double apk = a[p][k], aqk = a[q][k];
            a[p][k] = c * apk - sn * aqk;
            a[q][k] = sn * apk + c * aqk;
        }
        for (int k = 0; k < 3; ++k) {
            double akp = a[k][p], akq = a[k][q];
            a[k][p] = c * akp - sn * akq;
            a[k][q] = sn * akp + c * akq;
        }
        for (int k = 0; k < 3; ++k) {
            double vkp = v[k][p], vkq = v[k][q];
            v[k][p] = c * vkp - sn * vkq;
            v[k][q] = sn * vkp + c * vkq;
        }
    }
    int e = 0;
    if (a[1][1] < a[e][e])
        e = 1;
    if (a[2][2] < a[e][e])
        e = 2;
    return norm({(float)v[0][e], (float)v[1][e], (float)v[2][e]});
}

void compactUnreferenced(TriangleMesh& mesh) {
    auto cloud = mesh.vertices();
    if (!cloud)
        return;
    std::vector<unsigned char> used(cloud->size(), 0);
    for (auto id : mesh.indices())
        if (id < used.size())
            used[id] = 1;
    std::vector<uint32_t> map(cloud->size(), UINT32_MAX);
    PointCloud::Container pts;
    pts.reserve(cloud->size());
    for (uint32_t i = 0; i < cloud->size(); ++i)
        if (used[i]) {
            map[i] = (uint32_t)pts.size();
            pts.push_back(cloud->points()[i]);
        }
    for (auto& id : mesh.indices())
        id = map[id];
    mesh.setVertices(std::make_shared<PointCloud>(std::move(pts)));
    mesh.setIndices(mesh.indices());
}

std::shared_ptr<TriangleMesh> cloneMesh(const TriangleMesh& m) {
    auto cp = std::make_shared<PointCloud>(m.vertices()->points());
    auto out = std::make_shared<TriangleMesh>(cp, m.indices());
    out->triangleFlags() = m.triangleFlags();
    return out;
}
} // namespace

OperationDescriptor VoxelDownsampleOperation::descriptor() const {
    return {"voxel",
            "体素降采样",
            "点云",
            ModelKind::PointCloud,
            {{"voxel_mm", "体素大小", ParameterKind::Real, 2.0, 0.01, 100.0, 0.1, "mm"},
             {"average_color", "平均颜色", ParameterKind::Boolean, 1, 0, 1, 1, ""},
             {"average_normal", "平均法向", ParameterKind::Boolean, 1, 0, 1, 1, ""}}};
}
ProcessResult VoxelDownsampleOperation::run(const ProcessInput& i, const ParameterMap& p, const ProgressCallback& cb,
                                            const CancelToken& ct) const {
    ProcessResult r;
    if (!i.cloud) {
        r.message = "需要点云";
        return r;
    }
    r.inputPoints = i.cloud->activeCount();
    float s = std::max(1e-9f, millimetersToCloudUnits(*i.cloud, realParam(p, "voxel_mm", 2.0)));
    prog(cb, .05f, "构建体素");
    struct A {
        double x = 0, y = 0, z = 0, nx = 0, ny = 0, nz = 0;
        std::uint64_t rr = 0, gg = 0, bb = 0, aa = 0, n = 0;
    };
    std::unordered_map<Key, A, KeyHash> all;
#ifdef JMENGINE_USE_OPENMP
    int nt = processingThreadCount();
    std::vector<std::unordered_map<Key, A, KeyHash>> local(nt);
#pragma omp parallel num_threads(nt)
    {
        int tid = omp_get_thread_num();
        auto& mp = local[tid];
#pragma omp for schedule(static)
        for (long long ii = 0; ii < (long long)i.cloud->size(); ++ii) {
            const auto& q = i.cloud->points()[(size_t)ii];
            if (q.flags & PointDeleted)
                continue;
            auto& a = mp[keyOf(q.position, s)];
            a.x += q.position.x;
            a.y += q.position.y;
            a.z += q.position.z;
            a.nx += q.normal.x;
            a.ny += q.normal.y;
            a.nz += q.normal.z;
            a.rr += (q.rgba) & 255;
            a.gg += (q.rgba >> 8) & 255;
            a.bb += (q.rgba >> 16) & 255;
            a.aa += (q.rgba >> 24) & 255;
            ++a.n;
        }
    }
    for (auto& mp : local)
        for (auto& kv : mp) {
            auto &a = all[kv.first], &b = kv.second;
            a.x += b.x;
            a.y += b.y;
            a.z += b.z;
            a.nx += b.nx;
            a.ny += b.ny;
            a.nz += b.nz;
            a.rr += b.rr;
            a.gg += b.gg;
            a.bb += b.bb;
            a.aa += b.aa;
            a.n += b.n;
        }
#else
    for (const auto& q : i.cloud->points()) {
        if (q.flags & PointDeleted)
            continue;
        auto& a = all[keyOf(q.position, s)];
        a.x += q.position.x;
        a.y += q.position.y;
        a.z += q.position.z;
        a.nx += q.normal.x;
        a.ny += q.normal.y;
        a.nz += q.normal.z;
        a.rr += (q.rgba) & 255;
        a.gg += (q.rgba >> 8) & 255;
        a.bb += (q.rgba >> 16) & 255;
        a.aa += (q.rgba >> 24) & 255;
        ++a.n;
    }
#endif
    if (ct.cancelled()) {
        r.cancelled = true;
        return r;
    }
    prog(cb, .75f, "生成点云");
    PointCloud::Container out;
    out.reserve(all.size());
    for (auto& kv : all) {
        auto& a = kv.second;
        if (!a.n)
            continue;
        Point q;
        double n = (double)a.n;
        q.position = {(float)(a.x / n), (float)(a.y / n), (float)(a.z / n)};
        q.normal = norm({(float)(a.nx / n), (float)(a.ny / n), (float)(a.nz / n)});
        q.rgba = (std::uint32_t(a.rr / a.n)) | (std::uint32_t(a.gg / a.n) << 8) | (std::uint32_t(a.bb / a.n) << 16) |
                 (std::uint32_t(a.aa / a.n) << 24);
        out.push_back(q);
    }
    r.cloud = std::make_shared<PointCloud>(std::move(out));
    r.outputPoints = r.cloud->size();
    r.success = true;
    r.geometryChanged = r.topologyChanged = true;
    prog(cb, 1, "完成");
    return r;
}

OperationDescriptor RadiusOutlierOperation::descriptor() const {
    return {"radius_outlier",
            "半径离群点去噪",
            "点云",
            ModelKind::PointCloud,
            {{"radius_mm", "搜索半径", ParameterKind::Real, 3, 0.01, 100, 0.1, "mm"},
             {"min_neighbors", "最少邻居", ParameterKind::Integer, 5, 1, 500, 1, ""}}};
}
ProcessResult RadiusOutlierOperation::run(const ProcessInput& i, const ParameterMap& p, const ProgressCallback& cb,
                                          const CancelToken& ct) const {
    ProcessResult r;
    if (!i.cloud) {
        r.message = "需要点云";
        return r;
    }
    float rad = std::max(1e-9f, millimetersToCloudUnits(*i.cloud, realParam(p, "radius_mm", 3)));
    int mn = (int)intParam(p, "min_neighbors", 5);
    auto g = grid(*i.cloud, rad);
    std::vector<unsigned char> keep(i.cloud->size(), 1);
    int nt = processingThreadCount();
#ifdef JMENGINE_USE_OPENMP
#pragma omp parallel for schedule(dynamic, 256) num_threads(nt)
#endif
    for (long long id = 0; id < (long long)i.cloud->size(); ++id) {
        if (i.cloud->points()[(size_t)id].flags & PointDeleted) {
            keep[(size_t)id] = 0;
            continue;
        }
        int c = 0;
        forNeighbors(*i.cloud, g, (uint32_t)id, rad, [&](uint32_t) { ++c; });
        if (c < mn)
            keep[(size_t)id] = 0;
    }
    if (ct.cancelled()) {
        r.cancelled = true;
        return r;
    }
    PointCloud::Container out;
    out.reserve(i.cloud->size());
    for (size_t k = 0; k < i.cloud->size(); ++k)
        if (keep[k])
            out.push_back(i.cloud->points()[k]);
    r.inputPoints = i.cloud->activeCount();
    r.cloud = std::make_shared<PointCloud>(std::move(out));
    r.outputPoints = r.cloud->size();
    r.success = true;
    r.geometryChanged = r.topologyChanged = true;
    prog(cb, 1, "完成");
    return r;
}

OperationDescriptor StatisticalOutlierOperation::descriptor() const {
    return {"statistical_outlier",
            "统计离群点去噪",
            "点云",
            ModelKind::PointCloud,
            {{"k", "K 邻居", ParameterKind::Integer, 20, 2, 200, 1, ""},
             {"stddev", "标准差倍数", ParameterKind::Real, 1.5, .1, 10, .1, ""}}};
}
ProcessResult StatisticalOutlierOperation::run(const ProcessInput& i, const ParameterMap& p, const ProgressCallback& cb,
                                               const CancelToken& ct) const {
    ProcessResult r;
    if (!i.cloud) {
        r.message = "需要点云";
        return r;
    }
    const int k = std::clamp((int)intParam(p, "k", 20), 2, 200);
    const double mulv = std::max(0.0, realParam(p, "stddev", 1.5));
    r.inputPoints = i.cloud->activeCount();
    if (r.inputPoints <= 2) {
        r.cloud = std::make_shared<PointCloud>(i.cloud->points());
        r.outputPoints = r.inputPoints;
        r.success = true;
        return r;
    }

    prog(cb, .05f, "构建 KNN 索引");
    PointKdTree tree(*i.cloud);
    std::vector<double> meanDistance(i.cloud->size(), std::numeric_limits<double>::quiet_NaN());
    const int nt = processingThreadCount();
#ifdef JMENGINE_USE_OPENMP
#pragma omp parallel for schedule(dynamic, 128) num_threads(nt)
#endif
    for (long long id = 0; id < (long long)i.cloud->size(); ++id) {
        const auto uid = static_cast<std::uint32_t>(id);
        const auto& point = i.cloud->points()[static_cast<std::size_t>(id)];
        if (point.flags & PointDeleted)
            continue;
        std::array<std::uint32_t, 256> ids{};
        std::array<float, 256> d2{};
        const auto count = tree.knn(uid, static_cast<std::size_t>(k), 0.0f, ids, d2);
        if (count == 0) {
            meanDistance[static_cast<std::size_t>(id)] = std::numeric_limits<double>::infinity();
            continue;
        }
        double sum = 0.0;
        for (std::size_t n = 0; n < count; ++n)
            sum += std::sqrt(std::max(0.0f, d2[n]));
        meanDistance[static_cast<std::size_t>(id)] = sum / static_cast<double>(count);
    }
    if (ct.cancelled()) {
        r.cancelled = true;
        return r;
    }

    double sum = 0.0, sq = 0.0;
    std::size_t n = 0;
    for (double d : meanDistance) {
        if (!std::isfinite(d))
            continue;
        sum += d;
        sq += d * d;
        ++n;
    }
    if (!n) {
        r.message = "没有有效点";
        return r;
    }
    const double mean = sum / static_cast<double>(n);
    const double variance = std::max(0.0, sq / static_cast<double>(n) - mean * mean);
    const double threshold = mean + mulv * std::sqrt(variance);

    prog(cb, .8f, "删除统计离群点");
    PointCloud::Container out;
    out.reserve(r.inputPoints);
    for (std::size_t id = 0; id < i.cloud->size(); ++id) {
        const double d = meanDistance[id];
        if (std::isfinite(d) && d <= threshold)
            out.push_back(i.cloud->points()[id]);
    }
    r.cloud = std::make_shared<PointCloud>(std::move(out));
    r.outputPoints = r.cloud->size();
    r.success = true;
    r.geometryChanged = r.topologyChanged = true;
    r.message = "统计离群点去噪完成（KD-tree KNN）";
    prog(cb, 1, "完成");
    return r;
}

OperationDescriptor SmallClusterOperation::descriptor() const {
    return {"small_cluster",
            "删除小点云簇",
            "点云",
            ModelKind::PointCloud,
            {{"radius_mm", "连接半径", ParameterKind::Real, 3, 0.01, 100, .1, "mm"},
             {"min_points", "最小点数", ParameterKind::Integer, 1000, 1, 10000000, 10, ""}}};
}
ProcessResult SmallClusterOperation::run(const ProcessInput& i, const ParameterMap& p, const ProgressCallback& cb,
                                         const CancelToken& ct) const {
    ProcessResult r;
    if (!i.cloud) {
        r.message = "需要点云";
        return r;
    }
    float rad = std::max(1e-9f, millimetersToCloudUnits(*i.cloud, realParam(p, "radius_mm", 3)));
    size_t minp = (size_t)intParam(p, "min_points", 1000);
    auto g = grid(*i.cloud, rad);
    std::vector<int> comp(i.cloud->size(), -1);
    std::vector<std::vector<uint32_t>> groups;
    for (uint32_t s = 0; s < i.cloud->size(); ++s) {
        if (comp[s] >= 0 || (i.cloud->points()[s].flags & PointDeleted))
            continue;
        int ci = (int)groups.size();
        groups.emplace_back();
        std::queue<uint32_t> q;
        q.push(s);
        comp[s] = ci;
        while (!q.empty()) {
            auto u = q.front();
            q.pop();
            groups.back().push_back(u);
            forNeighbors(*i.cloud, g, u, rad, [&](uint32_t v) {
                if (comp[v] < 0) {
                    comp[v] = ci;
                    q.push(v);
                }
            });
        }
        if (ct.cancelled()) {
            r.cancelled = true;
            return r;
        }
    }
    PointCloud::Container out;
    for (auto& gr : groups)
        if (gr.size() >= minp)
            for (auto id : gr)
                out.push_back(i.cloud->points()[id]);
    r.inputPoints = i.cloud->activeCount();
    r.cloud = std::make_shared<PointCloud>(std::move(out));
    r.outputPoints = r.cloud->size();
    r.success = true;
    r.geometryChanged = r.topologyChanged = true;
    prog(cb, 1, "完成");
    return r;
}

OperationDescriptor NormalEstimationOperation::descriptor() const {
    return {"normal_estimation",
            "估算法向",
            "点云",
            ModelKind::PointCloud,
            {{"knn", "KNN 邻居数", ParameterKind::Integer, 24, 6, 64, 1, ""},
             {"max_distance_mm", "最大邻域距离(0=不限制)", ParameterKind::Real, 0, 0, 1000, 0.5, "mm"},
             {"threads", "法线线程数", ParameterKind::Integer, 0, 0, 64, 1, ""}}};
}
ProcessResult NormalEstimationOperation::run(const ProcessInput& i, const ParameterMap& p, const ProgressCallback& cb,
                                             const CancelToken& ct) const {
    ProcessResult r;
    if (!i.cloud) {
        r.message = "需要点云";
        return r;
    }

    auto out = std::make_shared<PointCloud>(i.cloud->points());
    const std::size_t knn = static_cast<std::size_t>(std::clamp<std::int64_t>(intParam(p, "knn", 24), 6, 64));
    const double maxDistanceMm = realParam(p, "max_distance_mm", 0.0);
    const float maxDistance = maxDistanceMm > 0.0 ? millimetersToCloudUnits(*i.cloud, maxDistanceMm) : 0.0f;

    prog(cb, 0.01f, "构建 KNN KD-tree");
    PointKdTree tree(*i.cloud);
    if (ct.cancelled()) {
        r.cancelled = true;
        return r;
    }

    int nt = static_cast<int>(intParam(p, "threads", 0));
    if (nt <= 0)
        nt = processingThreadCount();
    nt = std::max(1, std::min(nt, processingThreadCount() + 1));

    const long long count = static_cast<long long>(i.cloud->size());
    constexpr long long kProgressBlock = 65536;
    for (long long base = 0; base < count; base += kProgressBlock) {
        if (ct.cancelled()) {
            r.cancelled = true;
            return r;
        }
        const long long stop = std::min(count, base + kProgressBlock);
#ifdef JMENGINE_USE_OPENMP
#pragma omp parallel for schedule(static) num_threads(nt)
#endif
        for (long long id = base; id < stop; ++id) {
            if (ct.cancelled())
                continue;
            const auto idx = static_cast<std::size_t>(id);
            const auto& p0 = i.cloud->points()[idx];
            if (p0.flags & PointDeleted)
                continue;

            const auto& existing = out->points()[idx].normal;
            if (std::isfinite(existing.x) && std::isfinite(existing.y) && std::isfinite(existing.z) &&
                dot(existing, existing) > 0.25f)
                continue;

            std::array<std::uint32_t, 64> neighborIds{};
            std::array<float, 64> neighborD2{};
            const std::size_t nNeighbors = tree.knn(static_cast<std::uint32_t>(id), knn, maxDistance,
                                                     neighborIds, neighborD2);
            if (nNeighbors < 3)
                continue;

            // 把查询点自身作为第一个样本。KNN 只提供真正的相邻点，PCA 使用 K+1 个局部样本。
            std::size_t n = 1;
            double sx = 0.0, sy = 0.0, sz = 0.0;
            double sxx = 0.0, sxy = 0.0, sxz = 0.0, syy = 0.0, syz = 0.0, szz = 0.0;
            for (std::size_t ni = 0; ni < nNeighbors; ++ni) {
                const auto& q = i.cloud->points()[neighborIds[ni]].position;
                const double x = double(q.x) - p0.position.x;
                const double y = double(q.y) - p0.position.y;
                const double z = double(q.z) - p0.position.z;
                sx += x; sy += y; sz += z;
                sxx += x*x; sxy += x*y; sxz += x*z;
                syy += y*y; syz += y*z; szz += z*z;
                ++n;
            }

            const double inv = 1.0 / static_cast<double>(n);
            const double c00 = sxx - sx*sx*inv;
            const double c01 = sxy - sx*sy*inv;
            const double c02 = sxz - sx*sz*inv;
            const double c11 = syy - sy*sy*inv;
            const double c12 = syz - sy*sz*inv;
            const double c22 = szz - sz*sz*inv;
            out->points()[idx].normal = smallestEigenVectorSymmetric3x3(c00, c01, c02, c11, c12, c22);
        }
        const float p01 = count > 0 ? static_cast<float>(stop) / static_cast<float>(count) : 1.0f;
        prog(cb, 0.02f + 0.98f * p01, "并行 KNN PCA 法向估计");
    }

    if (ct.cancelled()) {
        r.cancelled = true;
        return r;
    }
    r.inputPoints = i.cloud->activeCount();
    r.outputPoints = out->activeCount();
    r.cloud = out;
    r.success = true;
    r.geometryChanged = true;
    r.message = "并行 KNN PCA 法向估计完成";
    prog(cb, 1, "完成");
    return r;
}

OperationDescriptor MeshCleanupOperation::descriptor() const {
    return {"mesh_cleanup",
            "网格清理",
            "网格",
            ModelKind::TriangleMesh,
            {{"remove_degenerate", "删除退化三角形", ParameterKind::Boolean, 1, 0, 1, 1, ""},
             {"remove_duplicate", "删除重复三角形", ParameterKind::Boolean, 1, 0, 1, 1, ""},
             {"merge_vertices", "合并重复顶点", ParameterKind::Boolean, 1, 0, 1, 1, ""},
             {"merge_tolerance_mm", "顶点合并容差", ParameterKind::Real, .001, 0, 1, .001, "mm"},
             {"remove_unreferenced", "删除孤立顶点", ParameterKind::Boolean, 1, 0, 1, 1, ""},
             {"remove_small_components", "删除小连通域", ParameterKind::Boolean, 0, 0, 1, 1, ""},
             {"min_triangles", "最小连通域三角形数", ParameterKind::Integer, 100, 1, 1000000, 1, ""}}};
}
ProcessResult MeshCleanupOperation::run(const ProcessInput& i, const ParameterMap& p, const ProgressCallback& cb,
                                        const CancelToken& ct) const {
    ProcessResult r;
    if (!i.mesh || !i.mesh->vertices()) {
        r.message = "需要网格";
        return r;
    }
    auto out = cloneMesh(*i.mesh);
    const auto& src = out->indices();
    std::vector<uint32_t> clean;
    clean.reserve(src.size());
    std::unordered_set<std::uint64_t> seen;
    for (size_t t = 0; t + 2 < src.size(); t += 3) {
        if (ct.cancelled()) {
            r.cancelled = true;
            return r;
        }
        uint32_t a = src[t], b = src[t + 1], c = src[t + 2];
        if (a >= out->vertices()->size() || b >= out->vertices()->size() || c >= out->vertices()->size())
            continue;
        bool bad = (a == b || b == c || a == c);
        if (!bad && boolParam(p, "remove_degenerate", true)) {
            auto& pa = out->vertices()->points()[a].position;
            auto& pb = out->vertices()->points()[b].position;
            auto& pc = out->vertices()->points()[c].position;
            auto n = cross(sub(pb, pa), sub(pc, pa));
            bad = dot(n, n) < 1e-20f;
        }
        if (bad)
            continue;
        if (boolParam(p, "remove_duplicate", true)) {
            std::array<uint32_t, 3> key{a, b, c};
            std::sort(key.begin(), key.end());
            std::uint64_t h = (static_cast<std::uint64_t>(key[0]) * 73856093ull) ^
                              (static_cast<std::uint64_t>(key[1]) * 19349663ull) ^
                              (static_cast<std::uint64_t>(key[2]) * 83492791ull);
            if (!seen.insert(h).second)
                continue;
        }
        clean.insert(clean.end(), {a, b, c});
    }
    out->setIndices(std::move(clean));
    prog(cb, .45f, "基础清理");
    if (boolParam(p, "remove_small_components", false) && out->triangleCount() > 0) {
        const size_t tc = out->triangleCount();
        std::vector<std::vector<uint32_t>> vertexTris(out->vertices()->size());
        for (uint32_t t = 0; t < tc; ++t)
            for (int k = 0; k < 3; ++k)
                vertexTris[out->indices()[3ull * t + k]].push_back(t);
        std::vector<int> comp(tc, -1);
        std::vector<std::vector<uint32_t>> groups;
        for (uint32_t s0 = 0; s0 < tc; ++s0) {
            if (comp[s0] >= 0)
                continue;
            int ci = (int)groups.size();
            groups.emplace_back();
            std::queue<uint32_t> q;
            q.push(s0);
            comp[s0] = ci;
            while (!q.empty()) {
                auto t = q.front();
                q.pop();
                groups.back().push_back(t);
                for (int k = 0; k < 3; ++k) {
                    auto v = out->indices()[3ull * t + k];
                    for (auto nt : vertexTris[v])
                        if (comp[nt] < 0) {
                            comp[nt] = ci;
                            q.push(nt);
                        }
                }
            }
        }
        size_t minTri = (size_t)intParam(p, "min_triangles", 100);
        std::vector<uint32_t> filtered;
        for (auto& g : groups)
            if (g.size() >= minTri)
                for (auto t : g) {
                    size_t b = 3ull * t;
                    filtered.insert(filtered.end(), {out->indices()[b], out->indices()[b + 1], out->indices()[b + 2]});
                }
        out->setIndices(std::move(filtered));
    }
    if (boolParam(p, "merge_vertices", true) && out->vertices() && !out->vertices()->empty()) {
        float tol = millimetersToCloudUnits(*out->vertices(), realParam(p, "merge_tolerance_mm", .001));
        if (tol > 0) {
            std::unordered_map<Key, uint32_t, KeyHash> first;
            std::vector<uint32_t> remap(out->vertices()->size());
            for (uint32_t v = 0; v < out->vertices()->size(); ++v) {
                auto k = keyOf(out->vertices()->points()[v].position, tol);
                auto it = first.find(k);
                if (it == first.end()) {
                    first.emplace(k, v);
                    remap[v] = v;
                } else
                    remap[v] = it->second;
            }
            for (auto& id : out->indices())
                id = remap[id];
            std::vector<uint32_t> f;
            f.reserve(out->indices().size());
            for (size_t t = 0; t + 2 < out->indices().size(); t += 3) {
                auto a = out->indices()[t], b = out->indices()[t + 1], c = out->indices()[t + 2];
                if (a != b && b != c && a != c)
                    f.insert(f.end(), {a, b, c});
            }
            out->setIndices(std::move(f));
        }
    }
    if (boolParam(p, "remove_unreferenced", true))
        compactUnreferenced(*out);
    r.inputPoints = i.mesh->vertices()->activeCount();
    r.outputPoints = out->vertices()->activeCount();
    r.inputTriangles = i.mesh->triangleCount();
    r.outputTriangles = out->triangleCount();
    r.mesh = out;
    r.cloud = out->vertices();
    r.success = true;
    r.topologyChanged = true;
    r.message = "网格清理完成";
    prog(cb, 1, "完成");
    return r;
}

OperationDescriptor MeshDenoiseOperation::descriptor() const {
    return {"mesh_denoise",
            "网格去噪",
            "网格",
            ModelKind::TriangleMesh,
            {{"remove_small_components", "删除小连通域", ParameterKind::Boolean, 1, 0, 1, 1, ""},
             {"min_component_triangles", "最小连通域三角形数", ParameterKind::Integer, 100, 1, 10000000, 10, ""},
             {"remove_spike_triangles", "删除异常尖刺/长边三角形", ParameterKind::Boolean, 1, 0, 1, 1, ""},
             {"long_edge_factor", "异常长边倍数", ParameterKind::Real, 6.0, 2.0, 30.0, .5, "x中位边长"},
             {"min_area_ratio", "最小形状面积比", ParameterKind::Real, .002, 0.0, .1, .001, ""},
             {"recompute_normals", "重新计算法线", ParameterKind::Boolean, 1, 0, 1, 1, ""}}};
}

ProcessResult MeshDenoiseOperation::run(const ProcessInput& i, const ParameterMap& p, const ProgressCallback& cb,
                                        const CancelToken& ct) const {
    ProcessResult r;
    if (!i.mesh || !i.mesh->vertices() || i.mesh->triangleCount() == 0) {
        r.message = "需要网格";
        return r;
    }
    auto out = cloneMesh(*i.mesh);
    r.inputPoints = out->vertices()->activeCount();
    r.inputTriangles = out->triangleCount();
    const auto& pts = out->vertices()->points();

    // 用采样中位边长作为鲁棒尺度；不会因少量飞点把阈值拉大。
    std::vector<float> sampledEdges;
    const std::size_t tc = out->triangleCount();
    const std::size_t maxSamples = 60000;
    const std::size_t step = std::max<std::size_t>(1, tc / std::max<std::size_t>(1, maxSamples));
    sampledEdges.reserve(std::min(tc, maxSamples) * 3);
    for (std::size_t t = 0; t < tc; t += step) {
        const std::size_t b = t * 3;
        const auto a = out->indices()[b], b0 = out->indices()[b + 1], c = out->indices()[b + 2];
        if (a >= pts.size() || b0 >= pts.size() || c >= pts.size())
            continue;
        sampledEdges.push_back(std::sqrt(dist2(pts[a].position, pts[b0].position)));
        sampledEdges.push_back(std::sqrt(dist2(pts[b0].position, pts[c].position)));
        sampledEdges.push_back(std::sqrt(dist2(pts[c].position, pts[a].position)));
    }
    float medianEdge = 0.0f;
    if (!sampledEdges.empty()) {
        const auto mid = sampledEdges.begin() + static_cast<std::ptrdiff_t>(sampledEdges.size() / 2);
        std::nth_element(sampledEdges.begin(), mid, sampledEdges.end());
        medianEdge = *mid;
    }
    const float maxEdge = medianEdge * static_cast<float>(realParam(p, "long_edge_factor", 6.0));
    const float minAreaRatio = static_cast<float>(realParam(p, "min_area_ratio", .002));
    const bool removeSpike = boolParam(p, "remove_spike_triangles", true) && medianEdge > 0.0f;

    prog(cb, .15f, "检测异常三角形");
    std::vector<std::uint32_t> filtered;
    filtered.reserve(out->indices().size());
    for (std::size_t t = 0; t < tc; ++t) {
        if (ct.cancelled()) { r.cancelled = true; return r; }
        const std::size_t b = 3 * t;
        const auto a = out->indices()[b], b0 = out->indices()[b + 1], c = out->indices()[b + 2];
        if (a >= pts.size() || b0 >= pts.size() || c >= pts.size() || a == b0 || b0 == c || c == a)
            continue;
        const float e0 = std::sqrt(dist2(pts[a].position, pts[b0].position));
        const float e1 = std::sqrt(dist2(pts[b0].position, pts[c].position));
        const float e2 = std::sqrt(dist2(pts[c].position, pts[a].position));
        const float longest = std::max({e0, e1, e2});
        const Vec3f cr = cross(sub(pts[b0].position, pts[a].position), sub(pts[c].position, pts[a].position));
        const float twiceArea = std::sqrt(std::max(0.0f, dot(cr, cr)));
        const float edgeSqSum = e0 * e0 + e1 * e1 + e2 * e2;
        const float shapeRatio = edgeSqSum > 1e-20f ? twiceArea / edgeSqSum : 0.0f;
        if (removeSpike && (longest > maxEdge || shapeRatio < minAreaRatio))
            continue;
        filtered.insert(filtered.end(), {a, b0, c});
    }
    out->setIndices(std::move(filtered));

    if (boolParam(p, "remove_small_components", true) && out->triangleCount()) {
        prog(cb, .55f, "删除小连通域");
        const std::size_t triCount = out->triangleCount();
        std::vector<std::vector<std::uint32_t>> vertexTris(out->vertices()->size());
        for (std::uint32_t t = 0; t < triCount; ++t)
            for (int k = 0; k < 3; ++k)
                vertexTris[out->indices()[3ull * t + k]].push_back(t);
        std::vector<int> comp(triCount, -1);
        std::vector<std::vector<std::uint32_t>> groups;
        for (std::uint32_t seed = 0; seed < triCount; ++seed) {
            if (comp[seed] >= 0) continue;
            const int ci = static_cast<int>(groups.size());
            groups.emplace_back();
            std::queue<std::uint32_t> q;
            q.push(seed); comp[seed] = ci;
            while (!q.empty()) {
                const auto t = q.front(); q.pop();
                groups.back().push_back(t);
                for (int k = 0; k < 3; ++k) {
                    const auto v = out->indices()[3ull * t + k];
                    for (const auto nt : vertexTris[v])
                        if (comp[nt] < 0) { comp[nt] = ci; q.push(nt); }
                }
            }
            if (ct.cancelled()) { r.cancelled = true; return r; }
        }
        const std::size_t minTri = static_cast<std::size_t>(std::max<std::int64_t>(1, intParam(p, "min_component_triangles", 100)));
        std::vector<std::uint32_t> kept;
        kept.reserve(out->indices().size());
        for (const auto& group : groups) {
            if (group.size() < minTri) continue;
            for (const auto t : group) {
                const std::size_t b = 3ull * t;
                kept.insert(kept.end(), {out->indices()[b], out->indices()[b + 1], out->indices()[b + 2]});
            }
        }
        out->setIndices(std::move(kept));
    }

    compactUnreferenced(*out);
    if (boolParam(p, "recompute_normals", true))
        JMEngine::recomputeVertexNormals(*out);
    r.outputPoints = out->vertices()->activeCount();
    r.outputTriangles = out->triangleCount();
    r.mesh = out;
    r.cloud = out->vertices();
    r.success = true;
    r.geometryChanged = true;
    r.topologyChanged = true;
    r.message = "网格去噪完成";
    prog(cb, 1.0f, "完成");
    return r;
}

OperationDescriptor LaplacianSmoothOperation::descriptor() const {
    return {"laplacian",
            "Laplacian 平滑",
            "网格",
            ModelKind::TriangleMesh,
            {{"iterations", "迭代次数", ParameterKind::Integer, 10, 1, 100, 1, ""},
             {"lambda", "Lambda", ParameterKind::Real, .3, 0, 1, .01, ""},
             {"preserve_boundary", "保持边界", ParameterKind::Boolean, 1, 0, 1, 1, ""}}};
}
static ProcessResult smoothMesh(const ProcessInput& i, const ParameterMap& p, const ProgressCallback& cb,
                                const CancelToken& ct, bool taubin) {
    ProcessResult r;
    if (!i.mesh || !i.mesh->vertices()) {
        r.message = "需要网格";
        return r;
    }
    auto out = cloneMesh(*i.mesh);
    auto neigh = vertexNeighbors(*out);
    const bool preserveBoundary = boolParam(p, "preserve_boundary", true);
    auto boundary = preserveBoundary ? boundaryVertices(*out) : std::vector<unsigned char>(out->vertices()->size(), 0);
    int iters = (int)intParam(p, "iterations", 10);
    float lam = (float)realParam(p, "lambda", taubin ? .5 : .3), mu = (float)realParam(p, "mu", -.53);
    std::vector<Vec3f> tmp(out->vertices()->size());
    auto step = [&](float alpha) {
        const int threadCount = processingThreadCount();
#ifdef JMENGINE_USE_OPENMP
#pragma omp parallel for schedule(static) num_threads(threadCount)
#endif
        for (long long v = 0; v < (long long)out->vertices()->size(); ++v) {
            auto pos = out->vertices()->points()[(size_t)v].position;
            if (boundary[(size_t)v]) {
                tmp[(size_t)v] = pos;
                continue;
            }
            auto& ns = neigh[(size_t)v];
            if (ns.empty()) {
                tmp[(size_t)v] = pos;
                continue;
            }
            Vec3f avg{};
            for (auto n : ns)
                avg = add(avg, out->vertices()->points()[n].position);
            avg = mul(avg, 1.0f / ns.size());
            tmp[(size_t)v] = add(pos, mul(sub(avg, pos), alpha));
        }
        for (size_t v = 0; v < tmp.size(); ++v)
            out->vertices()->points()[v].position = tmp[v];
    };
    for (int q = 0; q < iters; ++q) {
        if (ct.cancelled()) {
            r.cancelled = true;
            return r;
        }
        step(lam);
        if (taubin)
            step(mu);
        prog(cb, float(q + 1) / iters, "平滑");
    }
    r.inputTriangles = i.mesh->triangleCount();
    r.outputTriangles = out->triangleCount();
    r.inputPoints = i.mesh->vertices()->activeCount();
    r.outputPoints = out->vertices()->activeCount();
    r.mesh = out;
    r.cloud = out->vertices();
    r.success = true;
    r.geometryChanged = true;
    r.message = taubin ? "Taubin 平滑完成" : "Laplacian 平滑完成";
    return r;
}

ProcessResult LaplacianSmoothOperation::run(const ProcessInput& i, const ParameterMap& p, const ProgressCallback& cb,
                                            const CancelToken& ct) const {
    return smoothMesh(i, p, cb, ct, false);
}
OperationDescriptor TaubinSmoothOperation::descriptor() const {
    return {"taubin",
            "Taubin 平滑",
            "网格",
            ModelKind::TriangleMesh,
            {{"iterations", "迭代次数", ParameterKind::Integer, 10, 1, 100, 1, ""},
             {"lambda", "Lambda", ParameterKind::Real, .5, 0, 1, .01, ""},
             {"mu", "Mu", ParameterKind::Real, -.53, -1, 0, .01, ""},
             {"preserve_boundary", "保持边界", ParameterKind::Boolean, 1, 0, 1, 1, ""}}};
}
ProcessResult TaubinSmoothOperation::run(const ProcessInput& i, const ParameterMap& p, const ProgressCallback& cb,
                                         const CancelToken& ct) const {
    return smoothMesh(i, p, cb, ct, true);
}

OperationDescriptor QemDecimateOperation::descriptor() const {
    return {"qem_decimate",
            "QEM 网格简化",
            "网格",
            ModelKind::TriangleMesh,
            {{"ratio", "保留比例", ParameterKind::Real, .5, .01, 1, .01, ""},
             {"preserve_boundary", "保持边界", ParameterKind::Boolean, 1, 0, 1, 1, ""}}};
}
ProcessResult QemDecimateOperation::run(const ProcessInput& i, const ParameterMap& p, const ProgressCallback& cb,
                                        const CancelToken& ct) const {
    ProcessResult r;
    if (!i.mesh || !i.mesh->vertices()) {
        r.message = "需要网格";
        return r;
    }
    const double ratio = std::clamp(realParam(p, "ratio", .5), .01, 1.0);
    const size_t target = std::max<size_t>(1, (size_t)std::llround(i.mesh->triangleCount() * ratio));
    if (target >= i.mesh->triangleCount()) {
        r.success = true;
        r.mesh = cloneMesh(*i.mesh);
        r.cloud = r.mesh->vertices();
        r.inputTriangles = r.outputTriangles = i.mesh->triangleCount();
        r.inputPoints = r.outputPoints = i.mesh->vertices()->activeCount();
        r.message = "无需简化";
        return r;
    }
    struct Q {
        double m[10]{};
        void addPlane(double a, double b, double c, double d) {
            double v[4] = {a, b, c, d};
            int k = 0;
            for (int x = 0; x < 4; ++x)
                for (int y = x; y < 4; ++y)
                    m[k++] += v[x] * v[y];
        }
        Q& operator+=(const Q& o) {
            for (int k = 0; k < 10; ++k)
                m[k] += o.m[k];
            return *this;
        }
    };
    auto eval = [](const Q& q, const Vec3f& p0) {
        double v[4] = {p0.x, p0.y, p0.z, 1};
        double sum = 0;
        int k = 0;
        for (int x = 0; x < 4; ++x)
            for (int y = x; y < 4; ++y) {
                double f = (x == y ? 1.0 : 2.0);
                sum += f * q.m[k++] * v[x] * v[y];
            }
        return sum;
    };
    const size_t nv = i.mesh->vertices()->size();
    std::vector<Q> quad(nv);
    const auto& src = i.mesh->indices();
    for (size_t t = 0; t + 2 < src.size(); t += 3) {
        auto a = src[t], b = src[t + 1], c = src[t + 2];
        auto pa = i.mesh->vertices()->points()[a].position, pb = i.mesh->vertices()->points()[b].position,
             pc = i.mesh->vertices()->points()[c].position;
        auto n = cross(sub(pb, pa), sub(pc, pa));
        float len = std::sqrt(dot(n, n));
        if (len < 1e-12f)
            continue;
        n = mul(n, 1.0f / len);
        double d = -dot(n, pa);
        quad[a].addPlane(n.x, n.y, n.z, d);
        quad[b].addPlane(n.x, n.y, n.z, d);
        quad[c].addPlane(n.x, n.y, n.z, d);
    }
    auto boundary =
        boolParam(p, "preserve_boundary", true) ? boundaryVertices(*i.mesh) : std::vector<unsigned char>(nv, 0);
    struct E {
        uint32_t a, b;
        double cost;
        Vec3f pos;
    };
    std::unordered_set<uint64_t> seen;
    std::vector<E> edges;
    edges.reserve(src.size());
    auto key = [](uint32_t a, uint32_t b) {
        if (a > b)
            std::swap(a, b);
        return (uint64_t(a) << 32) | b;
    };
    for (size_t t = 0; t + 2 < src.size(); t += 3) {
        uint32_t v[3] = {src[t], src[t + 1], src[t + 2]};
        for (int z = 0; z < 3; ++z) {
            uint32_t a = v[z], b = v[(z + 1) % 3];
            auto kk = key(a, b);
            if (!seen.insert(kk).second)
                continue;
            if (!boundary.empty() && (boundary[a] || boundary[b]))
                continue;
            Vec3f pos =
                mul(add(i.mesh->vertices()->points()[a].position, i.mesh->vertices()->points()[b].position), .5f);
            Q q = quad[a];
            q += quad[b];
            edges.push_back({a, b, eval(q, pos), pos});
        }
    }
    std::sort(edges.begin(), edges.end(), [](const E& a, const E& b) { return a.cost < b.cost; });
    std::vector<uint32_t> parent(nv);
    std::iota(parent.begin(), parent.end(), 0u);
    std::vector<Vec3f> pos(nv);
    for (uint32_t v = 0; v < nv; ++v)
        pos[v] = i.mesh->vertices()->points()[v].position;
    auto find = [&](uint32_t x) {
        uint32_t r0 = x;
        while (parent[r0] != r0)
            r0 = parent[r0];
        while (parent[x] != x) {
            uint32_t n = parent[x];
            parent[x] = r0;
            x = n;
        }
        return r0;
    };
    size_t collapsed = 0;
    for (size_t ei = 0; ei < edges.size(); ++ei) {
        if (ct.cancelled()) {
            r.cancelled = true;
            return r;
        }
        uint32_t a = find(edges[ei].a), b = find(edges[ei].b);
        if (a == b)
            continue;
        parent[b] = a;
        pos[a] = edges[ei].pos;
        quad[a] += quad[b];
        ++collapsed;
        const bool checkNow = (i.mesh->triangleCount() < 10000u) || ((collapsed & 127u) == 0u);
        if (checkNow) {
            size_t count = 0;
            for (size_t t = 0; t + 2 < src.size(); t += 3) {
                uint32_t a0 = find(src[t]), b0 = find(src[t + 1]), c0 = find(src[t + 2]);
                if (a0 != b0 && b0 != c0 && a0 != c0)
                    ++count;
            }
            prog(cb,
                 .1f + .8f * float(i.mesh->triangleCount() - std::min(i.mesh->triangleCount(), count)) /
                           float(std::max<size_t>(1, i.mesh->triangleCount() - target)),
                 "QEM Edge Collapse");
            if (count <= target)
                break;
        }
    }
    auto cp = std::make_shared<PointCloud>(i.mesh->vertices()->points());
    for (uint32_t v = 0; v < nv; ++v) {
        uint32_t root = find(v);
        if (root == v)
            cp->points()[v].position = pos[v];
    }
    std::vector<uint32_t> ix;
    ix.reserve(src.size());
    std::unordered_set<uint64_t> triSeen;
    for (size_t t = 0; t + 2 < src.size(); t += 3) {
        uint32_t a = find(src[t]), b = find(src[t + 1]), c = find(src[t + 2]);
        if (a == b || b == c || a == c)
            continue;
        std::array<uint32_t, 3> sk{a, b, c};
        std::sort(sk.begin(), sk.end());
        uint64_t h =
            (uint64_t(sk[0]) * 73856093ull) ^ (uint64_t(sk[1]) * 19349663ull) ^ (uint64_t(sk[2]) * 83492791ull);
        if (!triSeen.insert(h).second)
            continue;
        ix.insert(ix.end(), {a, b, c});
        if (ix.size() / 3 >= target && ratio < 1.0) {
        }
    }
    auto out = std::make_shared<TriangleMesh>(cp, std::move(ix));
    compactUnreferenced(*out);
    r.inputTriangles = i.mesh->triangleCount();
    r.outputTriangles = out->triangleCount();
    r.inputPoints = i.mesh->vertices()->activeCount();
    r.outputPoints = out->vertices()->activeCount();
    r.mesh = out;
    r.cloud = out->vertices();
    r.success = true;
    r.topologyChanged = true;
    r.geometryChanged = true;
    r.message = "QEM 简化完成";
    prog(cb, 1, "完成");
    return r;
}

OperationDescriptor HoleFillOperation::descriptor() const {
    return {"hole_fill",
            "检测并填充孔洞",
            "修复",
            ModelKind::TriangleMesh,
            {{"max_edges", "最大孔洞边数", ParameterKind::Integer, 1000, 3, 100000, 1, ""},
             {"fill", "填充孔洞", ParameterKind::Boolean, 1, 0, 1, 1, ""}}};
}
ProcessResult HoleFillOperation::run(const ProcessInput& i, const ParameterMap& p, const ProgressCallback& cb,
                                     const CancelToken& ct) const {
    (void)ct;
    ProcessResult r;
    if (!i.mesh || !i.mesh->vertices()) {
        r.message = "需要网格";
        return r;
    }
    auto out = cloneMesh(*i.mesh);
    struct E {
        uint32_t a, b;
        bool operator==(E const& o) const {
            return a == o.a && b == o.b;
        }
    };
    struct H {
        size_t operator()(E const& e) const {
            return ((size_t)e.a << 32) ^ e.b;
        }
    };
    std::unordered_map<E, int, H> cnt;
    for (size_t t = 0; t + 2 < out->indices().size(); t += 3) {
        uint32_t v[3] = {out->indices()[t], out->indices()[t + 1], out->indices()[t + 2]};
        for (int e = 0; e < 3; ++e) {
            uint32_t a = v[e], b = v[(e + 1) % 3];
            if (a > b)
                std::swap(a, b);
            cnt[{a, b}]++;
        }
    }
    std::unordered_map<uint32_t, std::vector<uint32_t>> adj;
    for (auto& kv : cnt)
        if (kv.second == 1) {
            adj[kv.first.a].push_back(kv.first.b);
            adj[kv.first.b].push_back(kv.first.a);
        }
    std::unordered_set<uint64_t> used;
    std::vector<std::vector<uint32_t>> loops;
    auto ek = [](uint32_t a, uint32_t b) {
        if (a > b)
            std::swap(a, b);
        return ((uint64_t)a << 32) | b;
    };
    for (auto& kv : adj)
        for (auto n : kv.second) {
            if (used.count(ek(kv.first, n)))
                continue;
            std::vector<uint32_t> loop{kv.first};
            uint32_t prev = kv.first, cur = n;
            used.insert(ek(prev, cur));
            while (cur != loop.front() && loop.size() < 100000) {
                loop.push_back(cur);
                auto& ns = adj[cur];
                uint32_t next = UINT32_MAX;
                for (auto x : ns)
                    if (x != prev && !used.count(ek(cur, x))) {
                        next = x;
                        break;
                    }
                if (next == UINT32_MAX)
                    break;
                prev = cur;
                cur = next;
                used.insert(ek(prev, cur));
            }
            if (cur == loop.front() && loop.size() >= 3)
                loops.push_back(std::move(loop));
        }
    r.holesDetected = loops.size();
    if (boolParam(p, "fill", true)) {
        size_t maxe = (size_t)intParam(p, "max_edges", 1000);
        auto& pts = out->vertices()->points();
        for (auto& lp : loops) {
            if (lp.size() > maxe)
                continue;
            Vec3f c{};
            for (auto v : lp)
                c = add(c, pts[v].position);
            c = mul(c, 1.0f / lp.size());
            Point q;
            q.position = c;
            uint32_t center = (uint32_t)pts.size();
            pts.push_back(q);
            for (size_t z = 0; z < lp.size(); ++z)
                out->indices().insert(out->indices().end(), {lp[z], lp[(z + 1) % lp.size()], center});
        }
        out->setIndices(out->indices());
    }
    r.inputTriangles = i.mesh->triangleCount();
    r.outputTriangles = out->triangleCount();
    r.inputPoints = i.mesh->vertices()->activeCount();
    r.outputPoints = out->vertices()->activeCount();
    r.mesh = out;
    r.cloud = out->vertices();
    r.success = true;
    r.topologyChanged = true;
    prog(cb, 1, "完成");
    return r;
}

} // namespace JMEngine::processing
