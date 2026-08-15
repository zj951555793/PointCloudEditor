#pragma once
#include "TriangleMesh.h"
#include "Types.h"
#include <cstddef>

namespace pceditor {

struct DistanceMeasurement { Vec3f a{}; Vec3f b{}; double distance{0.0}; };
struct AngleMeasurement { Vec3f a{}; Vec3f vertex{}; Vec3f c{}; double radians{0.0}; double degrees{0.0}; };

struct MeshMeasureOptions {
    // 忽略面积小于该阈值的退化三角形。0 表示只忽略数值退化三角形。
    double minTriangleArea{0.0};
    // 体积仅在闭合、2-manifold 网格上返回 valid=true。
    bool requireWatertightForVolume{true};
};

struct SurfaceAreaMeasurement {
    bool valid{false};
    double area{0.0};
    std::size_t triangleCount{0};
    std::size_t degenerateTriangleCount{0};
};

struct VolumeMeasurement {
    bool valid{false};
    double volume{0.0};
    double signedVolume{0.0};
    std::size_t triangleCount{0};
    std::size_t degenerateTriangleCount{0};
    std::size_t boundaryEdgeCount{0};
    std::size_t nonManifoldEdgeCount{0};
};

DistanceMeasurement measureDistance(const Vec3f& a, const Vec3f& b) noexcept;
AngleMeasurement measureAngle(const Vec3f& a, const Vec3f& vertex, const Vec3f& c) noexcept;

// 工业测量：仅对 TriangleMesh 给出面积/体积。PointCloud 没有唯一的面积/体积定义，
// 必须先重建/三角化成具有明确拓扑的网格，避免用包围盒/凸包伪造“精确体积”。
SurfaceAreaMeasurement measureSurfaceArea(const TriangleMesh& mesh,
                                          const MeshMeasureOptions& options = {}) noexcept;
VolumeMeasurement measureVolume(const TriangleMesh& mesh,
                                const MeshMeasureOptions& options = {}) noexcept;

} // namespace pceditor
