#pragma once
#include "PointCloud.h"
#include "Types.h"
#include <array>
#include <cstddef>

namespace JMEngine {

struct AlignmentResult {
    bool success{false};
    Mat4f transform{Mat4f::identity()};
    float rmsError{0.0f};
};

AlignmentResult alignThreePoints(const std::array<Vec3f, 3>& source,
                                 const std::array<Vec3f, 3>& target) noexcept;

enum class AutoAlignmentStatus {
    Success,
    InvalidInput,
    NotEnoughPoints,
    NoCorrespondence,
    NotConverged,
    QualityRejected
};

struct AutoAlignmentOptions {
    // 0 = 根据目标包围盒对角线自动估计。否则使用模型坐标单位。
    float coarseVoxelSize{0.0f};
    float fineVoxelSize{0.0f};
    float maxCorrespondenceDistance{0.0f};
    std::size_t maxSamplePoints{120000};
    int coarseIterations{30};
    int fineIterations{45};
    // 每轮只保留误差最小的这部分对应点，降低离群点/部分重叠影响。
    float trimFraction{0.75f};
    // 工业质量门槛：不足则不允许应用结果。
    float minInlierRatio{0.30f};
    float maxAcceptedRms{0.0f}; // 0 = 自动：目标 bbox 对角线 * 0.01
};

struct AutoAlignmentResult {
    AutoAlignmentStatus status{AutoAlignmentStatus::InvalidInput};
    bool success{false};
    Mat4f transform{Mat4f::identity()}; // source(world) -> target(world) 的增量变换
    float rmsError{0.0f};
    float inlierRatio{0.0f};
    std::size_t correspondenceCount{0};
    int iterations{0};
};

// 自动刚体对齐：PCA 多假设粗配准 + trimmed ICP 精配准。
// sourceTransform/targetTransform 是当前场景中的非破坏式模型变换；结果 transform 是
// 一个 world-space 增量，可直接 left-multiply 到 sourceTransform。
AutoAlignmentResult alignPointClouds(const PointCloud& source,
                                     const Mat4f& sourceTransform,
                                     const PointCloud& target,
                                     const Mat4f& targetTransform,
                                     const AutoAlignmentOptions& options = {});

} // namespace JMEngine
