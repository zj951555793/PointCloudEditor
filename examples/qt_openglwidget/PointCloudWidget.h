#pragma once

#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <QPoint>
#include <QString>
#include <QThreadPool>

#include <JMEngine/ObjModelLoader.h>
#include <JMEngine/JMEngine.h>

#include "../common/ExampleUtils.h"

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

class QKeyEvent;
class QMouseEvent;
class QWheelEvent;
class QString;

// Qt 只承担 UI + OpenGL 显示。
// Core 模型解析、点云编辑、CPU 穿透选择均来自无 Qt 的 JMEngine 库。
class PointCloudWidget final : public QOpenGLWidget,
                               protected QOpenGLFunctions_3_3_Core {
public:
    enum class InteractionMode {
        View,
        Rectangle,
        Lasso,
        Circle,
        BrushSelect
    };

    // 表面：只选择当前视角最前面的可见表面；
    // 穿透：忽略遮挡，选择屏幕区域前后全部几何。
    enum class SelectionDepthMode {
        Surface,
        Through
    };

    explicit PointCloudWidget(const std::string& fileName = {}, QWidget* parent = nullptr);
    ~PointCloudWidget() override;

    void setInteractionMode(InteractionMode mode);
    InteractionMode interactionMode() const noexcept { return mode_; }

    void setSelectionDepthMode(SelectionDepthMode mode);
    SelectionDepthMode selectionDepthMode() const noexcept { return depthMode_; }

    void deleteSelection();
    void clearCurrentSelection();
    void undoEdit();
    void redoEdit();
    void fitView();
    void saveModel();

    // 多模型场景接口。path 是模型唯一键；同一路径只加载一次。
    void loadModelAsync(const QString& fileName);
    bool activateModel(const QString& fileName);
    void setModelVisible(const QString& fileName, bool visible);
    void removeModel(const QString& fileName);
    void clearModel();

    QString activeModelPath() const;

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    enum class DirtyBuffer { Flags, Selection, Position };

    struct DirtyRange {
        JMEngine::PointId begin{0};
        JMEngine::PointId end{0}; // 闭区间
        DirtyBuffer buffer{DirtyBuffer::Flags};
    };

    // 每个模型拥有独立 CPU 编辑状态和 GPU Buffer。
    // 因此多个模型可以同时显示，而编辑/Undo/Selection 只作用于 activeModel_。
    struct SceneModel {
        QString path;
        bool visible{true};

        std::shared_ptr<JMEngine::PointCloud> cloud;
        JMEngine::Engine editor;
        JMEngine::ObjMeshData mesh;
        bool meshMode{false};

        std::vector<std::uint32_t> selectedMask;
        std::vector<JMEngine::PointId> selectedIds;
        std::deque<DirtyRange> dirtyRanges;
        std::size_t cachedActiveCount{0};

        GLuint vao{0};
        GLuint positionVbo{0};
        GLuint colorVbo{0};
        GLuint normalVbo{0};
        GLuint pointIdVbo{0};
        GLuint flagsVbo{0};
        GLuint selectionVbo{0};
        GLuint meshEbo{0};

        GLsizei gpuPointCapacity{0};
        GLsizei gpuPointCount{0};
        GLsizei gpuMeshIndexCount{0};
        std::size_t gpuUploadCursor{0};
        std::size_t meshUploadCursor{0};

        SceneModel(QString modelPath,
                   std::shared_ptr<JMEngine::PointCloud> modelCloud,
                   JMEngine::ObjMeshData modelMesh,
                   bool isMesh);
    };

    struct PendingPick {
        InteractionMode mode{InteractionMode::Rectangle};
        QPoint press;
        QPoint current;
        std::vector<QPoint> stroke; // 物理像素坐标
        Qt::KeyboardModifiers modifiers{};
        int left{0};
        int top{0};
        int width{0};
        int height{0};
        int readY{0};
        int framebufferWidth{0};
        int framebufferHeight{0};
        int brushRadiusPhysical{0};
        bool meshIds{false}; // true = FBO 中保存 TriangleId；false = PointId
    };

    SceneModel* activeModel();
    const SceneModel* activeModel() const;
    int findModel(const QString& path) const;

    bool createPrograms();
    bool createPickingFramebuffer(int width, int height);
    void destroyPickingObjects();
    void destroySceneModelGl(SceneModel& model);
    void createSceneModelGl(SceneModel& model);

    void processInitialGpuUpload(SceneModel& model, std::size_t& byteBudget);
    void processDirtyGpuUpdates(SceneModel& model, std::size_t& byteBudget);
    void queueDirtyIds(SceneModel& model,
                       const std::vector<JMEngine::PointId>& ids,
                       DirtyBuffer buffer);

    void drawScene();
    void drawModel(SceneModel& model, bool active);
    void drawSelectedOverlay(SceneModel& model);
    void drawOverlay();

    // Surface Picking：点云输出 PointId；网格输出真实 TriangleId。
    void drawPointPickingPass(SceneModel& model);
    void drawMeshPickingPass(SceneModel& model);

    void startSelection(Qt::KeyboardModifiers modifiers);
    void startSurfacePickAsync(Qt::KeyboardModifiers modifiers);
    void startThroughPickAsync(Qt::KeyboardModifiers modifiers);
    void pollPickReadback();

    std::vector<JMEngine::PointId> filterSurfacePixels(
        const PendingPick& pending,
        const std::vector<std::uint32_t>& pixels) const;
    std::vector<JMEngine::PointId> runThroughSelection(
        const PendingPick& pending,
        const JMEngine::Mat4f& mvp,
        const JMEngine::PointCloud& cloud,
        const JMEngine::ObjMeshData* mesh) const;
    std::vector<JMEngine::PointId> expandTriangleIdsToVertices(
        const SceneModel& model,
        const std::vector<std::uint32_t>& triangleIds) const;
    std::vector<JMEngine::PointId> expandVertexSelectionToWholeTriangles(
        const SceneModel& model,
        const std::vector<JMEngine::PointId>& vertexIds) const;

    void applySelection(std::vector<JMEngine::PointId> ids,
                        Qt::KeyboardModifiers modifiers);
    void setSelectionVisual(SceneModel& model,
                            const std::vector<JMEngine::PointId>& ids);
    void clearSelectionVisual(SceneModel& model);

    void deleteIdsAsync(std::vector<JMEngine::PointId> ids);
    void undoAsync();
    void redoAsync();
    void saveAsync();

    void setMode(InteractionMode mode);
    const char* modeName() const noexcept;
    const char* depthModeName() const noexcept;

    JMEngine::Mat4f currentMvp() const;
    QPoint toPhysical(const QPoint& logical) const;
    PendingPick buildPendingPick(Qt::KeyboardModifiers modifiers, bool physical) const;

private:
    std::vector<std::unique_ptr<SceneModel>> models_;
    std::vector<QString> loadingPaths_; // 防止模型管理器重复点击时重复解析同一文件。
    int activeModel_{-1};

    JMEngine::example::OrbitCamera camera_;

    QOpenGLShaderProgram colorProgram_;
    QOpenGLShaderProgram meshProgram_;
    QOpenGLShaderProgram pointPickProgram_;
    QOpenGLShaderProgram meshPickProgram_;

    GLuint pickFbo_{0};
    GLuint pickTexture_{0};
    GLuint pickDepth_{0};
    GLuint pickPbo_{0};
    GLsync pickFence_{nullptr};
    int pickWidth_{0};
    int pickHeight_{0};

    InteractionMode mode_{InteractionMode::View};
    SelectionDepthMode depthMode_{SelectionDepthMode::Surface};
    QPoint pressPos_;
    QPoint currentPos_;
    QPoint lastViewPos_;
    std::vector<QPoint> stroke_;
    bool dragging_{false};
    // true 仅表示“Ctrl + 左键”已经锁定为一次编辑选择手势。
    // 不能只用 dragging_ 判断，因为普通左键 Orbit 同样会设置 dragging_。
    // Overlay 只在 editGestureActive_ 为 true 时绘制矩形/Lasso/Circle/Brush 预览。
    bool editGestureActive_{false};
    Qt::MouseButton viewDragButton_{Qt::NoButton};
    int brushRadiusPixels_{22};

    bool editBusy_{false};
    bool pickBusy_{false};
    bool loadBusy_{false};
    PendingPick pendingPick_{};
    std::string statusText_;

    // 编辑任务串行；OpenMP 在 Core 内部负责单个大模型的并行计算。
    QThreadPool workerPool_;
};
