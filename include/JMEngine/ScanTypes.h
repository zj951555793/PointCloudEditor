#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace JMEngine {

enum class ScanState : std::uint8_t {
    Idle,
    Initializing,
    Scanning,
    Stopping,
    ReadyForReconstruction,
    Reconstructing,
    Error
};

enum class ScanRegistrationMode : std::uint8_t {
    Geometry,
    Texture,
    Marker
};

struct ScanConfig {
    int maxFrames{20000};
    int maxInflightFrames{6};
    int previewPointsPerFrame{12000};
    int previewPointLimit{500000};
    int textureKeyframeStride{5};
    int textureMaxKeyframes{120};
    double offlineVoxel{3.0};
    int offlineIterations{30};
    float depthScale{0.001f};
    float minDepth{0.05f};
    float maxDepth{3.0f};
    float fx{0.0f};
    float fy{0.0f};
    float cx{0.0f};
    float cy{0.0f};
    ScanRegistrationMode registrationMode{ScanRegistrationMode::Texture};
    // Controls periodic online pose refresh/application. Keeping this in the engine config
    // avoids running expensive full-history getResults() scans when the UI switch is off.
    bool liveOptimizationEnabled{true};
    std::string calibrationPath;
    std::string vocabularyPath;
    // Optional scan project persistence: pose/frame cloud are saved without affecting SLAM memory.
    bool saveScanProject{false};
    std::string scanProjectPath;
};

struct Pose {
    std::array<float, 16> matrix{
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1};
};

struct FramePoseUpdate {
    int frameId{-1};
    Pose pose;
};

// Pixel memory is owned by the frame. Producers may reuse their capture
// buffers immediately after JMScanner::submit() returns.
struct CameraFrame {
    std::shared_ptr<std::vector<std::uint8_t>> rgb;
    std::shared_ptr<std::vector<std::uint8_t>> code;
    std::shared_ptr<std::vector<std::uint16_t>> depth;
    int width{0};
    int height{0};
    // Structured-light code image may intentionally use a different native
    // resolution from RGB (e.g. OneShot code width 216/343). Never resize it
    // merely to fit the RGB frame dimensions.
    int codeWidth{0};
    int codeHeight{0};
    std::uint64_t timestampUs{0};
    int frameId{-1};

    bool valid() const noexcept {
        const auto pixelCount = width > 0 && height > 0
            ? std::size_t(width) * std::size_t(height)
            : 0u;
        const bool hasDepth = depth && depth->size() >= pixelCount;
        const std::size_t codePixelCount = codeWidth > 0 && codeHeight > 0
            ? std::size_t(codeWidth) * std::size_t(codeHeight)
            : pixelCount;
        const bool hasStructuredLight = code && codePixelCount > 0 &&
                                        code->size() >= codePixelCount;
        return pixelCount > 0 && (hasDepth || hasStructuredLight) && rgb &&
               rgb->size() >= pixelCount * 3u;
    }
};

struct ScanStatistics {
    ScanState state{ScanState::Idle};
    std::uint64_t submittedFrames{0};
    std::uint64_t processedFrames{0};
    std::uint64_t replacedFrames{0};
    std::uint64_t rejectedFrames{0};
    std::uint64_t livePoints{0};
};

struct ScanMarker {
    int localId{-1};
    float imageX{0.0f};
    float imageY{0.0f};
    bool hasDepth{false};
    std::array<float, 3> point3d{0, 0, 0};
};

struct ScanMarkerFrame {
    int frameId{-1};
    std::uint64_t timestampUs{0};
    std::vector<ScanMarker> markers;
};

struct TextureKeyframe {
    int frameId{-1};
    int width{0};
    int height{0};
    float fx{0.0f};
    float fy{0.0f};
    float cx{0.0f};
    float cy{0.0f};
    Pose worldToCamera;
    std::shared_ptr<std::vector<std::uint8_t>> rgb;
};

} // namespace JMEngine
