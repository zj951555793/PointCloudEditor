#pragma once

#include <cstddef>
#include <cstdint>

namespace pceditor {
namespace texture {
namespace detail {

struct CudaPointPod {
    float x;
    float y;
    float z;
};

struct CudaCameraPod {
    float fx;
    float fy;
    float cx;
    float cy;
    int imageWidth;
    int imageHeight;
    float worldToCamera[16];
};

struct CudaConfigPod {
    int visibilityWidth;
    int visibilityHeight;
    float maxViewAngleDeg;
    float visibilityTolerance;
    float borderMarginRatio;
    int buildVisibilityDepth;
};

bool cudaApiRuntimeAvailable(char* error, std::size_t errorCapacity) noexcept;

bool cudaApiSelectBestCameras(const CudaPointPod* points,
                              std::size_t pointCount,
                              const std::uint32_t* indices,
                              std::size_t indexCount,
                              const CudaCameraPod* cameras,
                              int cameraCount,
                              const CudaConfigPod& config,
                              const float* packedDepth,
                              std::size_t packedDepthCount,
                              float* outputPackedDepth,
                              std::size_t outputPackedDepthCount,
                              int* bestCamera,
                              float* bestScore,
                              std::size_t triangleCount,
                              char* error,
                              std::size_t errorCapacity) noexcept;

} // namespace detail
} // namespace texture
} // namespace pceditor
