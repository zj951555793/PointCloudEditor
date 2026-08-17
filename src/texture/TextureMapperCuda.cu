#include "TextureMapperCudaApi.h"

#include <cuda_runtime.h>
#include <math_constants.h>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace JMEngine {
namespace texture {
namespace detail {
namespace {

constexpr float kPi = 3.14159265358979323846f;

__device__ float3 sub3(float3 a, float3 b) {
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

__device__ float dot3(float3 a, float3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ float3 cross3(float3 a, float3 b) {
    return make_float3(a.y * b.z - a.z * b.y,
                       a.z * b.x - a.x * b.z,
                       a.x * b.y - a.y * b.x);
}

__device__ float3 norm3(float3 a) {
    const float length = sqrtf(fmaxf(0.0f, dot3(a, a)));
    if (length > 1e-12f) {
        a.x /= length;
        a.y /= length;
        a.z /= length;
    }
    return a;
}

__device__ float3 transformPoint(const CudaCameraPod& camera, float3 point) {
    const float* m = camera.worldToCamera;
    return make_float3(m[0] * point.x + m[4] * point.y + m[8] * point.z + m[12],
                       m[1] * point.x + m[5] * point.y + m[9] * point.z + m[13],
                       m[2] * point.x + m[6] * point.y + m[10] * point.z + m[14]);
}

__device__ float3 cameraCenter(const CudaCameraPod& camera) {
    const float* m = camera.worldToCamera;
    const float tx = m[12];
    const float ty = m[13];
    const float tz = m[14];
    return make_float3(-(m[0] * tx + m[1] * ty + m[2] * tz),
                       -(m[4] * tx + m[5] * ty + m[6] * tz),
                       -(m[8] * tx + m[9] * ty + m[10] * tz));
}

__device__ float edge2(float ax, float ay, float bx, float by, float px, float py) {
    return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
}

__device__ void atomicMinPositiveFloat(float* address, float value) {
    int* addressAsInt = reinterpret_cast<int*>(address);
    int old = *addressAsInt;
    while (value < __int_as_float(old)) {
        const int assumed = old;
        old = atomicCAS(addressAsInt, assumed, __float_as_int(value));
        if (old == assumed) break;
    }
}

__global__ void fillDepthKernel(float* depth, std::size_t count) {
    const std::size_t index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
    if (index < count) depth[index] = CUDART_INF_F;
}

__global__ void rasterDepthKernel(const CudaPointPod* points,
                                  const std::uint32_t* indices,
                                  std::size_t triangleCount,
                                  CudaCameraPod camera,
                                  int depthWidth,
                                  int depthHeight,
                                  float* depth) {
    const std::size_t triangle = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
    if (triangle >= triangleCount) return;

    const std::uint32_t ia = indices[triangle * 3u];
    const std::uint32_t ib = indices[triangle * 3u + 1u];
    const std::uint32_t ic = indices[triangle * 3u + 2u];
    const CudaPointPod pa = points[ia];
    const CudaPointPod pb = points[ib];
    const CudaPointPod pc = points[ic];
    const float3 a = transformPoint(camera, make_float3(pa.x, pa.y, pa.z));
    const float3 b = transformPoint(camera, make_float3(pb.x, pb.y, pb.z));
    const float3 c = transformPoint(camera, make_float3(pc.x, pc.y, pc.z));
    if (a.z <= 1e-6f || b.z <= 1e-6f || c.z <= 1e-6f) return;

    const float sx = static_cast<float>(depthWidth) / static_cast<float>(camera.imageWidth);
    const float sy = static_cast<float>(depthHeight) / static_cast<float>(camera.imageHeight);
    const float ua = (camera.fx * a.x / a.z + camera.cx) * sx;
    const float va = (camera.fy * a.y / a.z + camera.cy) * sy;
    const float ub = (camera.fx * b.x / b.z + camera.cx) * sx;
    const float vb = (camera.fy * b.y / b.z + camera.cy) * sy;
    const float uc = (camera.fx * c.x / c.z + camera.cx) * sx;
    const float vc = (camera.fy * c.y / c.z + camera.cy) * sy;
    const float area = edge2(ua, va, ub, vb, uc, vc);
    if (fabsf(area) < 1e-8f) return;

    const int minx = max(0, static_cast<int>(floorf(fminf(ua, fminf(ub, uc)))));
    const int maxx = min(depthWidth - 1, static_cast<int>(ceilf(fmaxf(ua, fmaxf(ub, uc)))));
    const int miny = max(0, static_cast<int>(floorf(fminf(va, fminf(vb, vc)))));
    const int maxy = min(depthHeight - 1, static_cast<int>(ceilf(fmaxf(va, fmaxf(vb, vc)))));
    if (minx > maxx || miny > maxy) return;

    for (int y = miny; y <= maxy; ++y) {
        for (int x = minx; x <= maxx; ++x) {
            const float px = static_cast<float>(x) + 0.5f;
            const float py = static_cast<float>(y) + 0.5f;
            const float w0 = edge2(ub, vb, uc, vc, px, py) / area;
            const float w1 = edge2(uc, vc, ua, va, px, py) / area;
            const float w2 = 1.0f - w0 - w1;
            if (w0 < -1e-4f || w1 < -1e-4f || w2 < -1e-4f) continue;
            const float inverseZ = w0 / a.z + w1 / b.z + w2 / c.z;
            if (inverseZ <= 1e-12f) continue;
            atomicMinPositiveFloat(&depth[static_cast<std::size_t>(y) * depthWidth + x], 1.0f / inverseZ);
        }
    }
}

__global__ void selectKernel(const CudaPointPod* points,
                             const std::uint32_t* indices,
                             std::size_t triangleCount,
                             const CudaCameraPod* cameras,
                             int cameraCount,
                             const float* depth,
                             int depthWidth,
                             int depthHeight,
                             float minCos,
                             float tolerance,
                             float margin,
                             int* bestCamera,
                             float* bestScore) {
    const std::size_t triangle = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
    if (triangle >= triangleCount) return;

    const std::uint32_t ia = indices[triangle * 3u];
    const std::uint32_t ib = indices[triangle * 3u + 1u];
    const std::uint32_t ic = indices[triangle * 3u + 2u];
    const CudaPointPod pa = points[ia];
    const CudaPointPod pb = points[ib];
    const CudaPointPod pc = points[ic];
    const float3 a = make_float3(pa.x, pa.y, pa.z);
    const float3 b = make_float3(pb.x, pb.y, pb.z);
    const float3 c = make_float3(pc.x, pc.y, pc.z);
    const float3 center = make_float3((a.x + b.x + c.x) / 3.0f,
                                      (a.y + b.y + c.y) / 3.0f,
                                      (a.z + b.z + c.z) / 3.0f);
    const float3 normal = norm3(cross3(sub3(b, a), sub3(c, a)));
    if (dot3(normal, normal) < 1e-8f) {
        bestCamera[triangle] = -1;
        bestScore[triangle] = -1.0f;
        return;
    }

    float selectedScore = -1.0f;
    int selectedCamera = -1;
    for (int cameraIndex = 0; cameraIndex < cameraCount; ++cameraIndex) {
        const CudaCameraPod& camera = cameras[cameraIndex];
        const float3 view = norm3(sub3(cameraCenter(camera), center));
        const float angleScore = fabsf(dot3(normal, view));
        if (angleScore < minCos) continue;

        const float3 q = transformPoint(camera, center);
        if (q.z <= 1e-6f) continue;
        const float u = camera.fx * q.x / q.z + camera.cx;
        const float v = camera.fy * q.y / q.z + camera.cy;
        const float mx = margin * static_cast<float>(camera.imageWidth);
        const float my = margin * static_cast<float>(camera.imageHeight);
        if (u < mx || u >= static_cast<float>(camera.imageWidth) - mx ||
            v < my || v >= static_cast<float>(camera.imageHeight) - my) continue;

        // The complete projected triangle must stay in one source image. Checking only
        // the centroid allowed UVs to leave this camera's atlas tile and read pixels from
        // a neighbouring tile.
        const float3 qa = transformPoint(camera, a);
        const float3 qb = transformPoint(camera, b);
        const float3 qc = transformPoint(camera, c);
        if (qa.z <= 1e-6f || qb.z <= 1e-6f || qc.z <= 1e-6f) continue;
        const float ua = camera.fx * qa.x / qa.z + camera.cx;
        const float va = camera.fy * qa.y / qa.z + camera.cy;
        const float ub = camera.fx * qb.x / qb.z + camera.cx;
        const float vb = camera.fy * qb.y / qb.z + camera.cy;
        const float uc = camera.fx * qc.x / qc.z + camera.cx;
        const float vc = camera.fy * qc.y / qc.z + camera.cy;
        if (ua < mx || ua >= static_cast<float>(camera.imageWidth)-mx || va < my || va >= static_cast<float>(camera.imageHeight)-my ||
            ub < mx || ub >= static_cast<float>(camera.imageWidth)-mx || vb < my || vb >= static_cast<float>(camera.imageHeight)-my ||
            uc < mx || uc >= static_cast<float>(camera.imageWidth)-mx || vc < my || vc >= static_cast<float>(camera.imageHeight)-my) continue;

        if (depth) {
            const int x = max(0, min(depthWidth - 1, static_cast<int>(u * depthWidth / camera.imageWidth)));
            const int y = max(0, min(depthHeight - 1, static_cast<int>(v * depthHeight / camera.imageHeight)));
            const float d = depth[(static_cast<std::size_t>(cameraIndex) * depthHeight + y) * depthWidth + x];
            if (isfinite(d) && fabsf(d - q.z) > tolerance) continue;
        }

        const float cxn = (u - camera.cx) / fmaxf(1.0f, camera.imageWidth * 0.5f);
        const float cyn = (v - camera.cy) / fmaxf(1.0f, camera.imageHeight * 0.5f);
        const float centerScore = fmaxf(0.0f, 1.0f - 0.35f * sqrtf(cxn * cxn + cyn * cyn));
        const float score = 0.8f * angleScore + 0.2f * centerScore;
        if (score > selectedScore) {
            selectedScore = score;
            selectedCamera = cameraIndex;
        }
    }

    bestCamera[triangle] = selectedCamera;
    bestScore[triangle] = selectedScore;
}

void setError(char* error, std::size_t capacity, const char* stage, cudaError_t code) noexcept {
    if (!error || capacity == 0u) return;
    if (code == cudaSuccess) {
        error[0] = '\0';
        return;
    }
    std::snprintf(error, capacity, "%s: %s", stage ? stage : "CUDA", cudaGetErrorString(code));
    error[capacity - 1u] = '\0';
}

void setTextError(char* error, std::size_t capacity, const char* text) noexcept {
    if (!error || capacity == 0u) return;
    std::snprintf(error, capacity, "%s", text ? text : "CUDA error");
    error[capacity - 1u] = '\0';
}

} // namespace

bool cudaApiRuntimeAvailable(char* error, std::size_t errorCapacity) noexcept {
    int count = 0;
    const cudaError_t result = cudaGetDeviceCount(&count);
    if (result != cudaSuccess) {
        setError(error, errorCapacity, "cudaGetDeviceCount", result);
        return false;
    }
    if (count <= 0) {
        setTextError(error, errorCapacity, "no CUDA device");
        return false;
    }
    if (error && errorCapacity) error[0] = '\0';
    return true;
}

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
                              std::size_t errorCapacity) noexcept {
    if (!points || pointCount == 0u || !indices || indexCount != triangleCount * 3u ||
        !cameras || cameraCount <= 0 || !bestCamera || !bestScore || triangleCount == 0u) {
        setTextError(error, errorCapacity, "invalid CUDA texture mapping input");
        return false;
    }

    CudaPointPod* devicePoints = nullptr;
    std::uint32_t* deviceIndices = nullptr;
    CudaCameraPod* deviceCameras = nullptr;
    float* deviceDepth = nullptr;
    int* deviceBestCamera = nullptr;
    float* deviceBestScore = nullptr;

    auto cleanup = [&]() {
        cudaFree(devicePoints);
        cudaFree(deviceIndices);
        cudaFree(deviceCameras);
        cudaFree(deviceDepth);
        cudaFree(deviceBestCamera);
        cudaFree(deviceBestScore);
    };

    auto check = [&](cudaError_t code, const char* stage) {
        if (code == cudaSuccess) return true;
        setError(error, errorCapacity, stage, code);
        cleanup();
        return false;
    };

    if (!check(cudaMalloc(reinterpret_cast<void**>(&devicePoints), pointCount * sizeof(CudaPointPod)), "cudaMalloc points")) return false;
    if (!check(cudaMalloc(reinterpret_cast<void**>(&deviceIndices), indexCount * sizeof(std::uint32_t)), "cudaMalloc indices")) return false;
    if (!check(cudaMalloc(reinterpret_cast<void**>(&deviceCameras), static_cast<std::size_t>(cameraCount) * sizeof(CudaCameraPod)), "cudaMalloc cameras")) return false;
    if (!check(cudaMalloc(reinterpret_cast<void**>(&deviceBestCamera), triangleCount * sizeof(int)), "cudaMalloc best camera")) return false;
    if (!check(cudaMalloc(reinterpret_cast<void**>(&deviceBestScore), triangleCount * sizeof(float)), "cudaMalloc best score")) return false;

    if (!check(cudaMemcpy(devicePoints, points, pointCount * sizeof(CudaPointPod), cudaMemcpyHostToDevice), "copy points")) return false;
    if (!check(cudaMemcpy(deviceIndices, indices, indexCount * sizeof(std::uint32_t), cudaMemcpyHostToDevice), "copy indices")) return false;
    if (!check(cudaMemcpy(deviceCameras, cameras, static_cast<std::size_t>(cameraCount) * sizeof(CudaCameraPod), cudaMemcpyHostToDevice), "copy cameras")) return false;

    const int depthWidth = config.visibilityWidth;
    const int depthHeight = config.visibilityHeight;
    const std::size_t expectedDepthCount = config.buildVisibilityDepth
        ? static_cast<std::size_t>(cameraCount) * static_cast<std::size_t>(depthWidth) * static_cast<std::size_t>(depthHeight)
        : 0u;

    if (expectedDepthCount > 0u) {
        if (!check(cudaMalloc(reinterpret_cast<void**>(&deviceDepth), expectedDepthCount * sizeof(float)), "cudaMalloc depth")) return false;
        if (packedDepth && packedDepthCount > 0u) {
            if (packedDepthCount != expectedDepthCount) {
                setTextError(error, errorCapacity, "packed depth size mismatch");
                cleanup();
                return false;
            }
            if (!check(cudaMemcpy(deviceDepth, packedDepth, expectedDepthCount * sizeof(float), cudaMemcpyHostToDevice), "copy depth")) return false;
        } else {
            const int fillThreads = 256;
            const int fillBlocks = static_cast<int>((expectedDepthCount + fillThreads - 1u) / fillThreads);
            fillDepthKernel<<<fillBlocks, fillThreads>>>(deviceDepth, expectedDepthCount);
            if (!check(cudaGetLastError(), "fillDepth launch")) return false;

            const int rasterThreads = 128;
            const int rasterBlocks = static_cast<int>((triangleCount + rasterThreads - 1u) / rasterThreads);
            for (int cameraIndex = 0; cameraIndex < cameraCount; ++cameraIndex) {
                rasterDepthKernel<<<rasterBlocks, rasterThreads>>>(
                    devicePoints, deviceIndices, triangleCount, cameras[cameraIndex], depthWidth, depthHeight,
                    deviceDepth + static_cast<std::size_t>(cameraIndex) * depthWidth * depthHeight);
                if (!check(cudaGetLastError(), "rasterDepth launch")) return false;
            }
            if (!check(cudaDeviceSynchronize(), "rasterDepth sync")) return false;
        }
    }

    const int threads = 128;
    const int blocks = static_cast<int>((triangleCount + threads - 1u) / threads);
    const float minCos = cosf(config.maxViewAngleDeg * kPi / 180.0f);
    selectKernel<<<blocks, threads>>>(devicePoints, deviceIndices, triangleCount,
                                      deviceCameras, cameraCount, deviceDepth,
                                      depthWidth, depthHeight, minCos,
                                      config.visibilityTolerance, config.borderMarginRatio,
                                      deviceBestCamera, deviceBestScore);
    if (!check(cudaGetLastError(), "selectKernel launch")) return false;
    if (!check(cudaDeviceSynchronize(), "selectKernel sync")) return false;
    if (outputPackedDepth && expectedDepthCount > 0u) {
        if (outputPackedDepthCount != expectedDepthCount) {
            setTextError(error, errorCapacity, "output packed depth size mismatch"); cleanup(); return false;
        }
        if (!check(cudaMemcpy(outputPackedDepth, deviceDepth, expectedDepthCount * sizeof(float), cudaMemcpyDeviceToHost), "copy visibility depth")) return false;
    }
    if (!check(cudaMemcpy(bestCamera, deviceBestCamera, triangleCount * sizeof(int), cudaMemcpyDeviceToHost), "copy best camera")) return false;
    if (!check(cudaMemcpy(bestScore, deviceBestScore, triangleCount * sizeof(float), cudaMemcpyDeviceToHost), "copy best score")) return false;

    cleanup();
    if (error && errorCapacity) error[0] = '\0';
    return true;
}

} // namespace detail
} // namespace texture
} // namespace JMEngine
