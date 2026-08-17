#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <JMEngine/processing/Processing.h>

namespace JMEngine::processing {

// 模型健康度统计。所有计数都针对当前 active 数据，不修改模型。
struct ModelDiagnostics {
    ModelKind kind{ModelKind::PointCloud};
    std::size_t points{0};
    std::size_t deletedPoints{0};
    std::size_t invalidPoints{0};
    std::size_t validNormals{0};
    std::size_t triangles{0};
    std::size_t deletedTriangles{0};
    std::size_t degenerateTriangles{0};
    std::size_t boundaryEdges{0};
    std::size_t nonManifoldEdges{0};
    std::size_t connectedComponents{0};
    double bboxDiagonal{0.0};
    double estimatedSpacing{0.0};
    double normalCoverage{0.0};
};

// 重处理任务启动前的量产预检。内存值是保守估计，目的是规避 OOM，而不是替代 profiler。
struct ProcessingPreflight {
    bool allowed{true};
    std::size_t estimatedWorkingSetBytes{0};
    std::size_t availableMemoryBytes{0};
    int requestedDepth{0};
    int recommendedDepth{0};
    ModelDiagnostics diagnostics;
    std::vector<std::string> warnings;
};

ModelDiagnostics analyzeModel(const ProcessInput& input);
ProcessingPreflight preflightOperation(const IProcessingOperation& operation, const ProcessInput& input,
                                       const ParameterMap& params);
std::string diagnosticsSummary(const ModelDiagnostics& diagnostics);
std::string preflightSummary(const ProcessingPreflight& preflight);

} // namespace JMEngine::processing
