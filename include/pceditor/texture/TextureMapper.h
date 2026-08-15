#pragma once

#include <pceditor/Types.h>
#include <pceditor/TriangleMesh.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace pceditor::texture {

enum class Backend {
    Auto = 0,
    Cpu,
    Cuda
};

enum class Quality {
    Fast = 0,
    High,
    Ultra,
    OpenMVS
};

struct ImageRGB8 {
    int width{0};
    int height{0};
    std::vector<std::uint8_t> pixels; // tightly packed RGBRGB...

    bool valid() const noexcept {
        return width > 0 && height > 0 && pixels.size() == static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3u;
    }
};

struct CameraFrame {
    int frameId{-1};
    ImageRGB8 image;
    float fx{0.0f}, fy{0.0f}, cx{0.0f}, cy{0.0f};
    Mat4f worldToCamera{Mat4f::identity()};
};

struct Config {
    Backend backend{Backend::Auto};
    Quality quality{Quality::High};
    int maxKeyframes{200};
    int maxAtlasSize{8192};
    int visibilityWidth{320};
    int visibilityHeight{240};
    float maxViewAngleDeg{70.0f};
    float visibilityTolerance{3.0f}; // model units, default scanner pipeline uses millimetres
    float borderMarginRatio{0.02f};
    bool buildVisibilityDepth{true};
    bool exposureCompensation{true};
    // Remove isolated per-triangle camera-label speckles while preserving real view boundaries.
    bool smoothCameraLabels{true};
    int cameraLabelSmoothIterations{2};
    float cameraLabelSwitchScoreLoss{0.06f};
    // OpenMVS-style global face-label regularization. High/Ultra enable this by default.
    bool globalViewSelection{true};
    int globalViewIterations{5};
    int candidateCameraCount{4};
    float globalSmoothness{0.18f};
    float projectedAreaWeight{0.28f};
    float angleWeight{0.52f};
    float imageCenterWeight{0.10f};
    float borderWeight{0.10f};
    float exposureGainMin{0.72f};
    float exposureGainMax{1.38f};
    // Industrial image-quality gate. Frames that are severely blurred are removed before
    // view selection. Threshold is relative to the median sharpness of the input sequence.
    bool rejectBlurredFrames{true};
    float minRelativeSharpness{0.18f};
    // Use color observations from real camera-label boundaries on the mesh to solve robust
    // per-camera RGB gains. This is substantially more stable than whole-image mean matching.
    bool seamAwareExposureCompensation{true};
    int exposureSolveIterations{24};
    // Empty gutters around atlas tiles prevent bilinear/mipmap sampling from leaking pixels
    // from a neighbouring source photograph.
    int atlasPaddingPixels{8};
    // OpenMVS-style patch atlas. Connected faces using the same camera are cropped
    // from the ORIGINAL source image and packed as independent texture patches.
    bool patchAtlas{true};
    int patchBorderPixels{4};
    // Edge-aware Potts weight used by the global label optimization.
    float patchSmoothness{0.22f};
    // Keep low-confidence / invisible geometry as a neutral material instead of
    // inventing a camera observation.
    bool keepUntexturedFaces{true};
    // 0 = auto-detect global triangle winding from calibrated views; +1/-1 force it.
    int meshWindingSign{0};
    bool bakePreviewVertexColors{true};
};

struct Result {
    bool ok{false};
    Backend backendUsed{Backend::Cpu};
    std::string message;
    double elapsedMs{0.0};

    // Texture-ready split mesh. Vertices are duplicated per triangle corner so a geometric
    // vertex may carry independent UV coordinates in different source photographs.
    std::shared_ptr<PointCloud> vertices;
    std::vector<std::uint32_t> indices;
    std::vector<Vec2f> texcoords;
    // One entry per original input triangle; value is CameraFrame::frameId, -1 means unmapped.
    std::vector<int> triangleCameraIds;
    ImageRGB8 atlas;
    std::size_t inputCameraCount{0};
    std::size_t acceptedCameraCount{0};
    std::size_t usedCameraCount{0};
    std::size_t mappedTriangleCount{0};
    std::size_t unmappedTriangleCount{0};
    std::size_t texturePatchCount{0};
};

class TextureMapper {
  public:
    TextureMapper() = default;

    static bool cudaCompiled() noexcept;
    static bool cudaAvailable(std::string* reason = nullptr) noexcept;

    Result map(const TriangleMesh& mesh, const std::vector<CameraFrame>& cameras,
               const Config& config = {}) const;
};

// Save the result as an OBJ + MTL + TGA texture set without OpenCV/VTK dependencies.
// path must end in .obj. The exporter writes <stem>.mtl and <stem>_texture.tga next to it.
bool saveObj(const Result& result, const std::string& path, std::string* message = nullptr,
             const TriangleMesh* activeTopology = nullptr);

} // namespace pceditor::texture
