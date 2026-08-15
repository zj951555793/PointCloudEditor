#pragma once
#include <atomic>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include <pceditor/PointCloud.h>
#include <pceditor/TriangleMesh.h>

namespace pceditor::processing {

enum class ModelKind { PointCloud, TriangleMesh };
enum class OutputPolicy { ReplaceCurrent, AddModelOnKindChange };
enum class ParameterKind { Integer, Real, Boolean, Choice };

struct ParameterSpec {
    std::string key;
    std::string label;
    ParameterKind kind{ParameterKind::Real};
    double defaultValue{0.0};
    double minValue{0.0};
    double maxValue{1.0};
    double step{1.0};
    std::string unit;
    std::vector<std::string> choices;

    ParameterSpec() = default;
    ParameterSpec(std::string k, std::string l, ParameterKind t, double dv, double mn, double mx, double st,
                  std::string u = {}, std::vector<std::string> c = {})
        : key(std::move(k)), label(std::move(l)), kind(t), defaultValue(dv), minValue(mn), maxValue(mx), step(st),
          unit(std::move(u)), choices(std::move(c)) {}
};

using ParameterValue = std::variant<std::int64_t, double, bool, std::string>;
using ParameterMap = std::unordered_map<std::string, ParameterValue>;

struct OperationDescriptor {
    std::string id;
    std::string title;
    std::string category;
    ModelKind inputKind{ModelKind::PointCloud};
    std::vector<ParameterSpec> parameters;
    OutputPolicy outputPolicy{OutputPolicy::ReplaceCurrent};
};

struct ProgressInfo {
    float progress{0.0f};
    std::string stage;
};
using ProgressCallback = std::function<void(const ProgressInfo&)>;

class CancelToken {
  public:
    CancelToken() : state_(std::make_shared<std::atomic_bool>(false)) {}
    explicit CancelToken(std::shared_ptr<std::atomic_bool> state) : state_(std::move(state)) {}
    bool cancelled() const noexcept {
        return state_ && state_->load(std::memory_order_relaxed);
    }
    void cancel() const noexcept {
        if (state_)
            state_->store(true, std::memory_order_relaxed);
    }
    std::shared_ptr<std::atomic_bool> state() const noexcept {
        return state_;
    }

  private:
    std::shared_ptr<std::atomic_bool> state_;
};

struct ProcessInput {
    std::shared_ptr<PointCloud> cloud;
    std::shared_ptr<TriangleMesh> mesh;
};

struct ProcessResult {
    bool success{false};
    bool cancelled{false};
    bool geometryChanged{false};
    bool topologyChanged{false};
    std::string message;
    std::shared_ptr<PointCloud> cloud;
    std::shared_ptr<TriangleMesh> mesh;
    std::size_t inputPoints{0}, outputPoints{0};
    std::size_t inputTriangles{0}, outputTriangles{0};
    std::size_t holesDetected{0};
    std::size_t degenerateTriangles{0};
    std::size_t boundaryEdges{0};
    std::size_t nonManifoldEdges{0};
    std::size_t connectedComponents{0};
};

inline std::int64_t intParam(const ParameterMap& p, const char* key, std::int64_t d) {
    auto it = p.find(key);
    if (it == p.end())
        return d;
    if (auto v = std::get_if<std::int64_t>(&it->second))
        return *v;
    if (auto v = std::get_if<double>(&it->second))
        return static_cast<std::int64_t>(*v);
    return d;
}
inline double realParam(const ParameterMap& p, const char* key, double d) {
    auto it = p.find(key);
    if (it == p.end())
        return d;
    if (auto v = std::get_if<double>(&it->second))
        return *v;
    if (auto v = std::get_if<std::int64_t>(&it->second))
        return static_cast<double>(*v);
    return d;
}
inline bool boolParam(const ParameterMap& p, const char* key, bool d) {
    auto it = p.find(key);
    if (it == p.end())
        return d;
    if (auto v = std::get_if<bool>(&it->second))
        return *v;
    return d;
}

class IProcessingOperation {
  public:
    virtual ~IProcessingOperation() = default;
    virtual OperationDescriptor descriptor() const = 0;
    virtual ProcessResult run(const ProcessInput&, const ParameterMap&, const ProgressCallback&,
                              const CancelToken&) const = 0;
};

std::unique_ptr<IProcessingOperation> createOperation(const std::string& id);
std::vector<OperationDescriptor> builtinOperations();

// 是否已经编译进官方 Kazhdan PoissonRecon 工业后端。
bool industrialPoissonAvailable() noexcept;

// 根据当前模型的尺度、密度和数量调整算法默认参数。
OperationDescriptor estimateOperationDescriptor(const IProcessingOperation& operation, const ProcessInput& input);

} // namespace pceditor::processing
