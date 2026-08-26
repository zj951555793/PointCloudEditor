#pragma once

#include <JMEngine/PointCloud.h>
#include <JMEngine/Types.h>

#include <cstdint>
#include <vector>
#include <string>

namespace jmengine_android {

class GlesPointCloudRenderer {
  ublic:
    struct Mat4 {
        float m[16]{};
    };
    void onSurfaceCreated();
    void onResize(int width, int height);
    void render(const JMEngine::PointCloud* cloud, std::uint64_t revision);

    void orbit(float dxPixels, float dyPixels);
    void zoom(float scaleFactor);
    void fitNextFrame();

    std::vector<JMEngine::PointId> selectRectangle(const JMEngine::PointCloud& cloud, int x0, int y0, int x1, int y1,
                                                   bool surfaceOnly);

    void clearSelection();
    void setSelection(const std::vector<JMEngine::PointId>& ids, std::size_t pointCount);

    bool gles31Available() const noexcept {
        return gles31Available_;
    }
    const char* statusText() const noexcept {
        return statusText_.c_str();
    }

  rivate:
    bool ensurePrograms();
    void uploadCloud(const JMEngine::PointCloud& cloud, std::uint64_t revision);
    void updateCameraFromCloud(const JMEngine::PointCloud& cloud);
    Mat4 currentMvp() const;
    void renderDepthPass();
    std::vector<JMEngine::PointId> dispatchSelection(const JMEngine::PointCloud& cloud, int x0, int y0, int x1, int y1,
                                                     bool surfaceOnly);

    unsigned compileShader(unsigned type, const char* source);
    unsigned linkProgram(unsigned vs, unsigned fs);
    unsigned linkComputeProgram(unsigned cs);
    void ensureDepthTarget();

    unsigned renderProgram_{0};
    unsigned depthProgram_{0};
    unsigned selectProgram_{0};
    unsigned vao_{0};
    unsigned positionBuffer_{0};
    unsigned colorBuffer_{0};
    unsigned flagBuffer_{0};
    unsigned selectionBuffer_{0};
    unsigned depthFbo_{0};
    unsigned depthTexture_{0};
    unsigned depthColorTexture_{0};

    int width_{1};
    int height_{1};
    std::size_t pointCount_{0};
    std::uint64_t uploadedRevision_{~std::uint64_t{0}};

    float centerX_{0.0f};
    float centerY_{0.0f};
    float centerZ_{0.0f};
    float radius_{1.0f};
    float yaw_{0.0f};
    float pitch_{0.0f};
    float distance_{3.0f};
    bool fitPending_{true};
    bool gles31Available_{false};
    std::string statusText_{"GLES not initialized"};
};

} // namespace jmengine_android
