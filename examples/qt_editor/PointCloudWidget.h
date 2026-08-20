#pragma once

#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLFramebufferObject>
#include <QOpenGLWidget>
#include <QColor>
#include <QPoint>
#include <QRect>
#include <QThreadPool>
#include <QElapsedTimer>

#include <JMEngine/ObjModelLoader.h>
#include <JMEngine/Alignment.h>
#include <JMEngine/Measurement.h>
#include <JMEngine/JMEngine.h>
#include <JMEngine/CpuMeshSelector.h>
#include <JMEngine/MeshSelectionClosure.h>
#include <JMEngine/TriangleMesh.h>
#include <JMEngine/edit/MeshEditSession.h>
#include <JMEngine/processing/Processing.h>
#include <JMEngine/processing/Diagnostics.h>
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
#include <JMEngine/texture/TextureMapper.h>
#endif

#include "../common/ExampleUtils.h"
#include <JMEngine/render/RenderBackend.h>

#include <cstdint>
#include <functional>
#include <string>
#include <atomic>
#include <array>
#include <optional>
#include <memory>
#include <vector>
#include <unordered_map>

class QEvent;
class QKeyEvent;
class QMouseEvent;
class QTouchEvent;
class QWheelEvent;

// 唯一 Qt 编辑器的 OpenGL 渲染控件。
//
// 上层只维护这一份代码，Desktop / RK3588 的差异由 IRenderBackend 隔离：
// - Desktop 正常渲染兼容 OpenGL 2.1；现代 GPU Picking 要求 OpenGL 3.2+；
// - OpenGL ES 3.1：原生 VAO + VBO + GLSL ES 3.10；
// - Desktop 现代 R32UI GPU Picking 直接复用 Render VAO/VBO/EBO；不支持时 CPU fallback；
// - 鼠标、触摸、模型管理、选择和编辑逻辑完全共用。
class PointCloudWidget final : public QOpenGLWidget, protected QOpenGLExtraFunctions {
  public:
    enum class InteractionMode { Rectangle, Lasso, Circle, Brush };

    enum class SelectionDepthMode { Surface, Through };

    enum class PickingMode { Gpu, Cpu };

    enum class DisplayMode { Points, Solid, Wireframe, SolidWireframe };

    enum class UtilityMode { None, MeasureDistance, MeasureAngle, AlignThreePoint, AutoAlign };

    explicit PointCloudWidget(QWidget* parent = nullptr);
    ~PointCloudWidget() override;

    void loadModelAsync(const QString& path);

    // Scan preview scene updates run on the UI thread; acquisition and SLAM run in JMEngine.
    QString beginScanPreview(std::size_t reservePoints = 2000000);
    void appendScanPreview(const std::shared_ptr<std::vector<JMEngine::Point>>& points, std::size_t pointLimit = 2000000);
    struct LiveFramePoseUpdate { int frameId{-1}; std::array<float,16> pose{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}; };
    // Local points are uploaded once; optimization updates frame poses only.
    void appendScanLocalFrame(int frameId, const std::shared_ptr<std::vector<JMEngine::Point>>& localPoints,
                              const std::array<float,16>& pose);
    void setScanFrameMarkers(int frameId, const std::vector<std::array<float,3>>& localMarkers);
    void updateScanFramePoses(const std::shared_ptr<std::vector<LiveFramePoseUpdate>>& updates);
    // Normal: current frame = green. Lost: last valid frame = yellow reference, current lost frame = green.
    void setCurrentScanFrame(const std::shared_ptr<std::vector<JMEngine::Point>>& points, bool trackingOk);
    // Commit the last valid/recovery reference to RGB history, then remove temporary status layers.
    void finalizeCurrentScanFrame();
    void clearCurrentScanFrame();
    void replaceScanPreview(const std::shared_ptr<JMEngine::PointCloud>& cloud);
    void updateOptimizedScanPreview(const std::shared_ptr<JMEngine::PointCloud>& cloud);
    void clearScanPreview();
    QString scanPreviewPath() const { return scanPreviewPath_; }
    // Unified scan renderer: real-camera and virtual scan use the exact same
    // SLAM-frame render clock and the same incremental VBO uploader. Source type
    // must never change the rendering policy. Kept as a no-op for source compatibility.
    void setScanRenderDrivenByCamera(bool enabled) { Q_UNUSED(enabled); }
    void requestScanRenderFrame() { update(); }
    double renderFps() const noexcept { return double(renderFpsTenths_.load(std::memory_order_relaxed)) / 10.0; }

    struct ScanCameraViewPose {
        JMEngine::Vec3f position{};
        JMEngine::Vec3f right{1.0f, 0.0f, 0.0f};
        JMEngine::Vec3f up{0.0f, 1.0f, 0.0f};
        JMEngine::Vec3f forward{0.0f, 0.0f, 1.0f};
        bool trackingOk{false};
        int frameId{-1};
    };
    void updateScanCameraPose(const ScanCameraViewPose& pose);
    void clearScanCameraPose();

    bool activateModel(const QString& path);
    void setModelVisible(const QString& path, bool visible);
    void removeModel(const QString& path);
    QString activeModelPath() const;

    void setInteractionMode(InteractionMode mode);
    void setSelectionDepthMode(SelectionDepthMode mode);
    void setPickingMode(PickingMode mode);
    void setDisplayMode(DisplayMode mode);
    void setModelDisplayColor(const QString& path, const QColor& color);
    QColor modelDisplayColor(const QString& path) const;
    using ModelAddedCallback = std::function<void(const QString&)>;
    void setModelAddedCallback(ModelAddedCallback cb) {
        modelAddedCallback_ = std::move(cb);
    }
    PickingMode pickingMode() const noexcept {
        return pickingMode_;
    }

    // 触摸设备没有 Ctrl 键，因此使用菜单栏“编辑模式”替代 Ctrl。
    // editTouchMode=false 时：单指永远旋转；true 时：单指执行当前选择工具。
    void setTouchEditMode(bool enabled);
    bool touchEditMode() const noexcept {
        return touchEditMode_;
    }

    // 对象移动模式：开启后左键拖动当前激活模型。
    // 无论此模式是否开启，Alt + 左键都可临时移动当前激活模型。
    void setObjectMoveMode(bool enabled);
    void startDistanceMeasurement();
    void startAngleMeasurement();
    void startThreePointAlignment();
    void measureActiveSurfaceArea();
    void measureActiveVolume();
    void startOrRunAutoAlignment();
    void cancelUtilityMode();
    bool objectMoveMode() const noexcept {
        return objectMoveMode_;
    }

    void deleteSelection();
    void keepSelectionOnly();
    void invertSelection();
    void compactActiveModel();
    void clearSelection();
    void undoEdit();
    void redoEdit();
    void fitView();
    // 基底裁剪：先用当前框选/套索选择拟合平面，随后可在视图中拖动中心手柄沿法向上下微调。
    bool fitBasePlaneFromSelection(QString* message = nullptr);
    bool applyBasePlaneCut(QString* message = nullptr);
    void cancelBasePlaneCut();
    bool basePlaneCutActive() const noexcept { return basePlane_.active; }
    void saveActiveModel();
    bool exportActiveModel(const QString& path, QString* message = nullptr);
    bool exportActiveModelAsync(const QString& path, std::function<void(bool, const QString&)> finished);

    // 处理算法全部位于 JMEngine Core；这里只负责把任务提交到现有加载线程池。
    using ProcessingProgressCallback = std::function<void(float, const QString&)>;
    using ProcessingFinishedCallback = std::function<void(bool, const QString&)>;
    JMEngine::processing::OperationDescriptor processingDescriptor(const std::string& operationId) const;
    JMEngine::processing::ModelDiagnostics activeModelDiagnostics() const;
    JMEngine::processing::ProcessingPreflight
    processingPreflight(const std::string& operationId, const JMEngine::processing::ParameterMap& params) const;
    using DiagnosticsFinishedCallback =
        std::function<void(bool, const JMEngine::processing::ModelDiagnostics&, const QString&)>;
    bool analyzeActiveModelAsync(DiagnosticsFinishedCallback finished);
    bool startProcessingOperation(const std::string& operationId, JMEngine::processing::ParameterMap params,
                                  ProcessingProgressCallback progress, ProcessingFinishedCallback finished);
    void cancelProcessing();
    bool processingBusy() const noexcept {
        return processingBusy_;
    }
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    using TextureFramesPtr = std::shared_ptr<std::vector<JMEngine::texture::CameraFrame>>;
    void setTextureFrames(TextureFramesPtr frames);
    std::size_t textureFrameCount() const noexcept { return textureFrames_ ? textureFrames_->size() : 0u; }
    bool startTextureMappingAsync(JMEngine::texture::Backend backend, ProcessingFinishedCallback finished);
    // Scan workflow helper: if the active scan result is still a point cloud, first run
    // industrial Poisson reconstruction with auto-tuned defaults, then texture the new mesh.
    bool startScanTextureMappingAsync(JMEngine::texture::Backend backend,
                                      ProcessingProgressCallback progress,
                                      ProcessingFinishedCallback finished);
#endif

  protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    bool event(QEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

  private:
    struct ProcessingSnapshot {
        QString path;
        std::shared_ptr<JMEngine::PointCloud> cloud;
        std::shared_ptr<JMEngine::TriangleMesh> mesh;
        bool meshMode{false};
        DisplayMode displayMode{DisplayMode::Solid};
        JMEngine::Mat4f modelTransform{JMEngine::Mat4f::identity()};
        QColor displayColor{220, 220, 220};
        bool useDisplayColor{false};
    };

    struct Model {
        QString path;
        bool visible{true};
        bool meshMode{false};
        DisplayMode displayMode{DisplayMode::Solid};

        std::shared_ptr<JMEngine::PointCloud> cloud;
        JMEngine::Engine editor;
        std::shared_ptr<JMEngine::TriangleMesh> mesh;
        JMEngine::MeshEditSession meshEditor;
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
        std::shared_ptr<JMEngine::texture::Result> textureResult;
        GLuint textureGl{0};
#endif

        std::vector<std::uint8_t> selectionMask;
        std::vector<JMEngine::PointId> selectedIds;
        std::vector<JMEngine::TriangleId> selectedTriangleIds;

        // GPU Buffer 结构统一使用 VAO + VBO/EBO。
        IRenderBackend::Buffers gpu;
        GLsizei drawPointCount{0};
        GLsizei drawIndexCount{0};
        GLsizei selectedMeshIndexCount{0};
        GLuint wireEbo{0};
        GLsizei wireIndexCount{0};
        bool wireDirty{true};

        // 分块 GPU 上传状态。
        std::size_t uploadPointCursor{0};
        std::size_t uploadIndexCursor{0};
        // 初始加载直接使用 mesh->indices()，不复制一份完整 EBO。
        // 只有发生删除/Undo/Redo 后才生成过滤后的 visibleMeshIndices。
        std::vector<std::uint32_t> visibleMeshIndices;
        std::vector<JMEngine::TriangleId> visibleTriangleIds;
        bool meshFiltered{false};
        bool meshUploadComplete{false};
        bool glCreated{false};
        bool gpuRecreatePending{false};
        bool meshIndexUploadPending{false};
        // 实时扫描模型可预留固定 GPU 容量，避免每帧增长时重新创建 VBO。
        std::size_t gpuReservedPointCapacity{0};
        // Live RT 优化使用真正的前/后台双 VBO。
        // gpu 始终是正在显示的 front；liveBackGpu 只负责接收 getResults() 的优化结果。
        // back 全部上传完成后一次 swap，绝不边显示边覆盖 front，避免撕裂/闪烁。
        IRenderBackend::Buffers liveBackGpu;
        bool liveBackCreated{false};
        std::size_t liveBackCapacity{0};
        std::shared_ptr<JMEngine::PointCloud> liveBackCloud;
        std::size_t liveBackUploadCursor{0};
        // back 上传期间继续到来的正常实时帧，swap 后再追加，避免优化刷新吃掉最新历史帧。
        std::vector<JMEngine::Point> livePostSwapPoints;

        // Live SLAM history is stored in frame-local coordinates. Each frame's points are appended
        // to the shared VBO exactly once. Backend optimization only changes pose matrices.
        struct LiveFrameRange {
            int frameId{-1};
            std::size_t first{0};
            std::size_t count{0};
            JMEngine::Mat4f pose{JMEngine::Mat4f::identity()};
        };
        bool liveFramePoseMode{false};
        std::vector<LiveFrameRange> liveFrames;
        std::unordered_map<int, std::size_t> liveFrameIndex;
        struct LiveMarkerRange { int frameId{-1}; std::size_t first{0}; std::size_t count{0}; };
        std::vector<LiveMarkerRange> liveMarkerRanges;
        std::unordered_map<int, std::size_t> liveMarkerIndex;
        std::unordered_map<int, std::vector<std::array<float,3>>> pendingLiveMarkers;

        // 场景级非破坏变换。当前交互器只修改平移，不重写点/三角形数据。
        JMEngine::Mat4f modelTransform{JMEngine::Mat4f::identity()};
        QColor displayColor{220, 220, 220};
        bool useDisplayColor{false};

        // 工业 Picking 空间索引：按规则 3D Grid 将 PointId 聚成空间相干块。
        // 选择时先投影 Block AABB 做粗筛，再仅对候选点做精确手势/深度测试。
        // 删除只改变 flags，不影响索引；点坐标/点数发生变化时会自动重建。
        struct PickBlock {
            JMEngine::Vec3f min{};
            JMEngine::Vec3f max{};
            std::size_t offset{0};
            std::size_t count{0};
        };
        std::vector<JMEngine::PointId> pickGridIds;
        std::vector<PickBlock> pickBlocks;
        const JMEngine::PointCloud* pickIndexedCloud{nullptr};
        std::size_t pickIndexedPointCount{0};

        // 每个模型独立保存 Processing 历史，避免多模型交替操作时全局栈互相阻塞。
        std::vector<ProcessingSnapshot> processingUndo;
        std::vector<ProcessingSnapshot> processingRedo;

        Model(QString p, std::shared_ptr<JMEngine::PointCloud> c, JMEngine::ObjMeshData m, bool meshModeFlag);
        Model(QString p, std::shared_ptr<JMEngine::TriangleMesh> meshValue);
    };

    const std::vector<std::uint32_t>& meshDrawIndices(const Model& model) const;

    ProcessingSnapshot captureProcessingSnapshot(const Model& model) const;
    bool restoreProcessingSnapshot(const ProcessingSnapshot& snapshot);

    Model* activeModel();
    const Model* activeModel() const;
    void updateOrbitPivotForActiveModel();
    int findModel(const QString& path) const;

    bool createPrograms();
    bool createPickingFramebuffer(int w, int h);
    void destroyPickingFramebuffer();
    void createModelGl(Model& model);
    void destroyModelGl(Model& model);
    void destroyLiveBackGl(Model& model);
    void uploadPointRangeNow(Model& model, std::size_t first, std::size_t count);
    void uploadLiveBackIncremental(Model& model, std::size_t& byteBudget);
    void uploadModelIncremental(Model& model, std::size_t& byteBudget);
    void uploadSelectionMask(Model& model);
    void uploadChangedFlags(Model& model, const std::vector<JMEngine::PointId>& ids);

    void rebuildVisibleMeshAsync(Model& model);
    void applyTriangleSelection(Model& model, std::vector<JMEngine::TriangleId> ids, Qt::KeyboardModifiers modifiers);

    void drawScene();
    void drawModel(Model& model);
    void drawSelectionOverlay(Model& model);
    void rebuildWireframeBuffer(Model& model);
    void drawGestureOverlay();
    void drawUtilityOverlay();
    void drawBasePlaneOverlay();
    void drawScanCameraOverlay();

    void beginEditGesture(const QPoint& pos, Qt::KeyboardModifiers modifiers = {});
    void updateEditGesture(const QPoint& pos);
    void finishEditGesture(const QPoint& pos, Qt::KeyboardModifiers modifiers = {});
    void cancelEditGesture();

    void performSurfaceSelection(Qt::KeyboardModifiers modifiers);
    void performThroughSelection(Qt::KeyboardModifiers modifiers);
    void ensurePointPickIndex(Model& model);
    std::vector<JMEngine::PointId> pointPickCandidates(Model& model, const JMEngine::Mat4f& mvp,
                                                       const QRect& logicalBounds);
    std::vector<JMEngine::PointId> filterPickedPointIds(const Model& model,
                                                        const std::vector<std::uint32_t>& ids) const;
    void applySelection(Model& model, std::vector<JMEngine::PointId> ids, Qt::KeyboardModifiers modifiers);

    bool handleTouchEvent(QTouchEvent* event);

    JMEngine::Mat4f currentMvp() const;
    JMEngine::Mat4f modelMvp(const Model& model) const;
    void moveActiveModelByPixels(float dxPixels, float dyPixels);
    std::optional<JMEngine::Vec3f> pickActiveWorldPoint(const QPoint& pos);
    void handleUtilityClick(const QPoint& pos);
    QPoint toPhysical(const QPointF& p) const;
    struct ScanStatusGpu {
        GLuint vbo[2]{0, 0};
        std::size_t capacity[2]{0, 0};
        int front{0};
        GLsizei count{0};
        bool visible{false};
        bool dirty{false};
        std::shared_ptr<std::vector<JMEngine::Point>> pending;
        std::vector<JMEngine::Vec3f> staging;
    };
    void uploadScanStatusFrame(ScanStatusGpu& layer);
    void drawScanStatusOverlays();
    void destroyScanStatusGpu();
    void advanceScanCameraFollow();

  private:
    std::vector<std::unique_ptr<Model>> models_;
    std::vector<QString> loadingPaths_;
    QString scanPreviewPath_;
    std::shared_ptr<std::vector<JMEngine::Point>> currentScanFrameSource_;
    bool currentScanFrameTrackingOk_{false};
    std::shared_ptr<std::vector<JMEngine::Point>> recoveryScanFrameSource_;
    ScanStatusGpu currentScanGpu_;
    ScanStatusGpu recoveryScanGpu_;
    std::size_t scanPreviewPointLimit_{2000000};
    // 实时扫描默认跟随最新一帧扫描区域。第一帧自动 fit，后续只平滑移动
    // camera target，不扫描整个累计点云，也不改变用户当前观察方向。
    bool scanPreviewViewInitialized_{false};
    std::uint64_t scanPreviewFrameCount_{0};
    std::optional<ScanCameraViewPose> scanCameraPose_;
    std::optional<ScanCameraViewPose> lastValidScanCameraPose_;
    bool scanCameraFollowEnabled_{true};
    bool scanCameraFollowInitialized_{false};
    JMEngine::Vec3f scanFollowTarget_{};
    JMEngine::example::OrbitCamera::Quat scanFollowOrientation_{};
    float scanFollowDistance_{0.0f};
    struct BasePlaneInteractor {
        bool active{false};
        bool dragging{false};
        JMEngine::Vec3f point{};      // 拟合平面上的基准点（模型局部坐标）
        JMEngine::Vec3f normal{0.0f, 1.0f, 0.0f}; // 指向模型主体一侧
        float offset{0.0f};           // 沿 normal 的交互偏移
        float visualRadius{1.0f};
        QPoint dragLast{};
    } basePlane_;

    int activeModelIndex_{-1};

    JMEngine::example::OrbitCamera camera_;

    QOpenGLShaderProgram pointProgram_;
    QOpenGLShaderProgram meshProgram_;
    QOpenGLShaderProgram scanStatusProgram_;
    QOpenGLShaderProgram pointPickProgram_;
    QOpenGLShaderProgram meshPickProgram_;

    GLuint pickFbo_{0};
    GLuint pickTexture_{0};
    GLuint pickDepth_{0};
    int pickWidth_{0};
    int pickHeight_{0};

    InteractionMode interactionMode_{InteractionMode::Rectangle};
    SelectionDepthMode selectionDepthMode_{SelectionDepthMode::Surface};
    PickingMode pickingMode_{PickingMode::Gpu};
    bool touchEditMode_{false};
    bool objectMoveMode_{false};
    UtilityMode utilityMode_{UtilityMode::None};
    std::vector<JMEngine::Vec3f> utilityPoints_;
    QString alignmentSourcePath_;
    std::array<JMEngine::Vec3f, 3> alignmentSourceWorld_{};
    int alignmentSourceCount_{0};
    QString autoAlignmentSourcePath_;
    bool autoAlignmentBusy_{false};

    // 对象拖动与相机浏览严格分离。
    bool objectDragging_{false};
    QPoint lastObjectDragPos_;

    // 鼠标/触摸编辑手势状态。
    bool editGestureActive_{false};
    QPoint pressPos_;
    QPoint currentPos_;
    std::vector<QPoint> stroke_;
    int brushRadiusPixels_{28};

    // 浏览手势状态。
    bool viewDragging_{false};
    QPoint lastViewPos_;
    Qt::MouseButton viewButton_{Qt::NoButton};

    // 双指触摸状态：同时完成平移和 pinch 缩放。
    bool twoFingerActive_{false};
    QPointF lastTouchCenter_;
    qreal lastTouchDistance_{0.0};

    QThreadPool workerPool_;
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    TextureFramesPtr textureFrames_;
#endif
    bool editBusy_{false};
    bool processingBusy_{false};
    bool exportBusy_{false};
    bool diagnosticsBusy_{false};
    JMEngine::processing::CancelToken processingCancel_;
    std::unique_ptr<IRenderBackend> backend_;
    bool renderReady_{false};
    bool pickingReady_{false};
    ModelAddedCallback modelAddedCallback_;
    QString statusText_;

    // Actual QOpenGLWidget paint rate, sampled over ~0.5 s windows.  Stored atomically so
    // the compact scan panel can poll it without coupling rendering to UI labels.
    QElapsedTimer renderFpsClock_;
    int renderFpsFrameCount_{0};
    std::atomic<int> renderFpsTenths_{0};
};
