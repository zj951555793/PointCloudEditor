#pragma once

#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <QPoint>
#include <QThreadPool>

#include <JMEngine/ObjModelLoader.h>
#include <JMEngine/JMEngine.h>

#include "../common/ExampleUtils.h"

#include <cstdint>
#include <memory>
#include <vector>

class QEvent;
class QKeyEvent;
class QMouseEvent;
class QTouchEvent;
class QWheelEvent;

// RK3588 专用 Qt + OpenGL ES 3.1 渲染控件。
//
// 设计目标：
// 1. 不使用 Desktop OpenGL 3.3 Core 专用类；
// 2. 不使用 Geometry Shader（GLES 3.1 标准不保证支持）；
// 3. 使用 R32UI + uint 输出做 GPU Picking；
// 4. 支持鼠标和多点触摸；
// 5. Core 仍然完全无 Qt 依赖。
class GlesPointCloudWidget final : public QOpenGLWidget,
                                   protected QOpenGLExtraFunctions
{
public:
    enum class InteractionMode {
        Rectangle,
        Lasso,
        Circle,
        Brush
    };

    enum class SelectionDepthMode {
        Surface,
        Through
    };

    explicit GlesPointCloudWidget(QWidget* parent = nullptr);
    ~GlesPointCloudWidget() override;

    void loadModelAsync(const QString& path);
    bool activateModel(const QString& path);
    void setModelVisible(const QString& path, bool visible);
    void removeModel(const QString& path);
    QString activeModelPath() const;

    void setInteractionMode(InteractionMode mode);
    void setSelectionDepthMode(SelectionDepthMode mode);

    // 触摸设备没有 Ctrl 键，因此使用工具栏的“编辑模式”按钮替代 Ctrl。
    // editTouchMode=false 时：单指永远旋转；true 时：单指执行当前选择工具。
    void setTouchEditMode(bool enabled);
    bool touchEditMode() const noexcept { return touchEditMode_; }

    void deleteSelection();
    void clearSelection();
    void undoEdit();
    void redoEdit();
    void fitView();
    void saveActiveModel();

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
    struct PickVertex {
        JMEngine::Vec3f position{};
        std::uint32_t id{0};
    };

    struct Model {
        QString path;
        bool visible{true};
        bool meshMode{false};

        std::shared_ptr<JMEngine::PointCloud> cloud;
        JMEngine::Engine editor;
        JMEngine::ObjMeshData mesh;

        std::vector<std::uint32_t> selectionMask;
        std::vector<JMEngine::PointId> selectedIds;

        // GPU 正常显示 Buffer。
        GLuint vao{0};
        GLuint positionVbo{0};
        GLuint colorVbo{0};
        GLuint normalVbo{0};
        GLuint pointIdVbo{0};
        GLuint flagsVbo{0};
        GLuint selectionVbo{0};
        GLuint meshEbo{0};
        GLsizei drawPointCount{0};
        GLsizei drawIndexCount{0};

        // OBJ 表面 Picking：三角面展开为独立顶点，每个三角形 3 个顶点都携带相同 TriangleId。
        // 这样 GLES 3.1 不需要 gl_PrimitiveID / Geometry Shader。
        GLuint pickVao{0};
        GLuint pickPositionVbo{0};
        GLuint pickIdVbo{0};
        GLsizei pickVertexCount{0};

        // 分块 GPU 上传状态。
        std::size_t uploadPointCursor{0};
        std::size_t uploadIndexCursor{0};
        std::size_t uploadPickCursor{0};
        std::vector<std::uint32_t> visibleMeshIndices;
        std::vector<PickVertex> expandedPickVertices;
        bool glCreated{false};
        bool meshIndexUploadPending{false};

        Model(QString p,
              std::shared_ptr<JMEngine::PointCloud> c,
              JMEngine::ObjMeshData m,
              bool mesh);
    };

    Model* activeModel();
    const Model* activeModel() const;
    int findModel(const QString& path) const;

    bool createPrograms();
    bool createPickingFramebuffer(int w, int h);
    void destroyPickingFramebuffer();
    void createModelGl(Model& model);
    void destroyModelGl(Model& model);
    void uploadModelIncremental(Model& model, std::size_t& byteBudget);
    void uploadSelectionMask(Model& model);
    void uploadChangedFlags(Model& model, const std::vector<JMEngine::PointId>& ids);

    void rebuildVisibleMeshAsync(Model& model);

    void drawScene();
    void drawModel(Model& model);
    void drawSelectionOverlay(Model& model);
    void drawGestureOverlay();

    void beginEditGesture(const QPoint& pos, Qt::KeyboardModifiers modifiers = {});
    void updateEditGesture(const QPoint& pos);
    void finishEditGesture(const QPoint& pos, Qt::KeyboardModifiers modifiers = {});
    void cancelEditGesture();

    void performSurfaceSelection(Qt::KeyboardModifiers modifiers);
    void performThroughSelection(Qt::KeyboardModifiers modifiers);
    std::vector<JMEngine::PointId> filterPickedIds(
        const Model& model,
        const std::vector<std::uint32_t>& ids,
        bool triangleIds) const;
    void applySelection(Model& model,
                        std::vector<JMEngine::PointId> ids,
                        Qt::KeyboardModifiers modifiers);

    bool handleTouchEvent(QTouchEvent* event);

    JMEngine::Mat4f currentMvp() const;
    QPoint toPhysical(const QPointF& p) const;

private:
    std::vector<std::unique_ptr<Model>> models_;
    std::vector<QString> loadingPaths_;
    int activeModelIndex_{-1};

    JMEngine::example::OrbitCamera camera_;

    QOpenGLShaderProgram pointProgram_;
    QOpenGLShaderProgram meshProgram_;
    QOpenGLShaderProgram pointPickProgram_;
    QOpenGLShaderProgram meshPickProgram_;

    GLuint pickFbo_{0};
    GLuint pickTexture_{0};
    GLuint pickDepth_{0};
    int pickWidth_{0};
    int pickHeight_{0};

    InteractionMode interactionMode_{InteractionMode::Rectangle};
    SelectionDepthMode selectionDepthMode_{SelectionDepthMode::Surface};
    bool touchEditMode_{false};

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
    bool editBusy_{false};
    bool glesReady_{false};
    bool pickingReady_{false};
    QString statusText_;
};
