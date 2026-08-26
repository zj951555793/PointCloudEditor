#include "MainWindow.h"
#include "PointCloudWidget.h"
#include "ProcessingDialog.h"

#include <JMEngine/processing/Processing.h>
#include <JMEngine/ScanProject.h>

#include <algorithm>
#include <cmath>
#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QColorDialog>
#include <QDockWidget>
#include <QFileDialog>
#include <QFileInfo>
#include <QKeySequence>
#include <QIcon>
#include <QImage>
#include <QListWidget>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QCheckBox>
#include <QCoreApplication>
#include <QDir>
#include <QDateTime>
#include <QElapsedTimer>
#include <QFormLayout>
#include <QGridLayout>
#include <QToolButton>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QStandardPaths>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QMenuBar>
#include <QMenu>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QVBoxLayout>
#include <QWidget>
#include <QPointer>
#include <QPixmap>
#include <QPainter>
#include <QPen>
#include <QPushButton>
#include <QResizeEvent>
#include <QTimer>
#include <thread>
#include <chrono>

namespace {
constexpr int kPathRole = Qt::UserRole + 1;

std::filesystem::path fsPathFromQString(const QString& path) {
#ifdef _WIN32
    return std::filesystem::path(path.toStdWString());
#else
    return std::filesystem::path(path.toStdString());
#endif
}

QString qStringFromFsPath(const std::filesystem::path& path) {
#ifdef _WIN32
    return QString::fromStdWString(path.wstring());
#else
    return QString::fromStdString(path.string());
#endif
}

QString defaultProjectWorkspacePath() {
    return qStringFromFsPath(JMEngine::ProjectManager::defaultWorkspace());
}

std::filesystem::path nextDatedProjectPath() {
    const auto now = QDateTime::currentDateTime();
    const auto root = fsPathFromQString(defaultProjectWorkspacePath());
    auto base = root / now.toString(QStringLiteral("yyyyMMdd")).toStdString() /
                now.toString(QStringLiteral("scan_HHmmss")).toStdString();
    auto candidate = base;
    int suffix = 2;
    while (std::filesystem::exists(candidate))
        candidate = std::filesystem::path(base.string() + "_" + std::to_string(suffix++));
    return candidate;
}

bool writeProjectOptimizedFlag(const QString& projectPath, bool optimized, QString* error = nullptr) {
    const QString jsonPath = QDir(projectPath).filePath(QStringLiteral("project.json"));
    QFile file(jsonPath);
    QJsonObject root;
    if (file.exists()) {
        if (!file.open(QIODevice::ReadOnly)) {
            if (error) *error = QString::fromUtf8("无法读取工程文件: %1").arg(jsonPath);
            return false;
        }
        QJsonParseError parseError{};
        const auto doc = QJsonDocument::fromJson(file.readAll(), &parseError);
        file.close();
        if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
            if (error) *error = QString::fromUtf8("工程文件格式错误: %1").arg(parseError.errorString());
            return false;
        }
        root = doc.object();
    }
    root.insert(QStringLiteral("version"), root.value(QStringLiteral("version")).toInt(3));
    root.insert(QStringLiteral("optimized"), optimized);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (error) *error = QString::fromUtf8("无法写入工程文件: %1").arg(jsonPath);
        return false;
    }
    return file.write(QJsonDocument(root).toJson(QJsonDocument::Indented)) >= 0;
}

QString defaultCameraModelJsonPath() {
#ifdef Q_OS_ANDROID
    const QString root = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    const QString configDir = QDir(root).filePath(QStringLiteral("config"));
    QDir().mkpath(configDir);
    const QString dst = QDir(configDir).filePath(QStringLiteral("camera_models.json"));
    if (!QFile::exists(dst)) {
        QFile bundled(QStringLiteral(":/config/camera_models.json"));
        if (bundled.exists()) {
            bundled.copy(dst);
            QFile::setPermissions(dst, QFileDevice::ReadOwner | QFileDevice::WriteOwner |
                                        QFileDevice::ReadUser | QFileDevice::WriteUser);
        }
    }
    return dst;
#else
    return QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("config/camera_models.json"));
#endif
}

QString loadCalibrationPath(const QString& path, QString* error = nullptr) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) *error = QString::fromUtf8("无法打开相机型号 JSON: %1").arg(path);
        return {};
    }
    QJsonParseError parseError{};
    const auto doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (error) *error = QString::fromUtf8("相机型号 JSON 格式错误: %1").arg(parseError.errorString());
        return {};
    }
    return doc.object().value(QStringLiteral("scan")).toObject()
        .value(QStringLiteral("lastCalibrationPath")).toString();
}

bool saveCalibrationPath(const QString& path, const QString& calibration, QString* error = nullptr) {
    QFile file(path);
    QJsonObject root;
    if (file.exists()) {
        if (!file.open(QIODevice::ReadOnly)) {
            if (error) *error = QString::fromUtf8("无法读取相机型号 JSON: %1").arg(path);
            return false;
        }
        QJsonParseError parseError{};
        const auto doc = QJsonDocument::fromJson(file.readAll(), &parseError);
        file.close();
        if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
            if (error) *error = QString::fromUtf8("相机型号 JSON 格式错误: %1").arg(parseError.errorString());
            return false;
        }
        root = doc.object();
    }
    auto scan = root.value(QStringLiteral("scan")).toObject();
    scan.insert(QStringLiteral("lastCalibrationPath"), calibration);
    root.insert(QStringLiteral("scan"), scan);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (error) *error = QString::fromUtf8("无法写入相机型号 JSON: %1").arg(path);
        return false;
    }
    return file.write(QJsonDocument(root).toJson(QJsonDocument::Indented)) >= 0;
}

QString scanStateText(JMEngine::ScanState state) {
    switch (state) {
    case JMEngine::ScanState::Idle: return QString::fromUtf8("空闲");
    case JMEngine::ScanState::Initializing: return QString::fromUtf8("初始化");
    case JMEngine::ScanState::Scanning: return QString::fromUtf8("扫描中");
    case JMEngine::ScanState::Stopping: return QString::fromUtf8("停止中");
    case JMEngine::ScanState::ReadyForReconstruction: return QString::fromUtf8("可离线优化");
    case JMEngine::ScanState::Reconstructing: return QString::fromUtf8("离线优化中");
    case JMEngine::ScanState::Error: return QString::fromUtf8("错误");
    }
    return QString::fromUtf8("未知");
}
}

QString MainWindow::CameraDeviceInfo::displayText() const {
    return QStringLiteral("%1 | %2 | %3:%4")
        .arg(modelName.isEmpty() ? QString::fromUtf8("未配置型号") : modelName,
             friendlyName.isEmpty() ? QString::fromUtf8("Camera") : friendlyName,
             vid.isEmpty() ? QStringLiteral("????") : vid,
             pid.isEmpty() ? QStringLiteral("????") : pid);
}

MainWindow::MainWindow(const QString& initialFile, QWidget* parent) : QMainWindow(parent) {
    view_ = new PointCloudWidget(this);
    setCentralWidget(view_);

    // Camera B (color) live preview overlays the 3D viewport.
    cameraPreviewLabel_ = new QLabel(view_);
    cameraPreviewLabel_->setAlignment(Qt::AlignCenter);
    cameraPreviewLabel_->setFixedSize(320, 200);
    cameraPreviewLabel_->setStyleSheet(QStringLiteral("QLabel { background: rgba(0,0,0,180); border: 1px solid #707070; color: white; }") );
    cameraPreviewLabel_->setText(QString::fromUtf8("相机 B 彩色预览"));
    cameraPreviewLabel_->hide();

    createActions();
    createModelManager();
    createProjectManager();
    createScanControl();
    view_->setModelAddedCallback([this](const QString& path) {
        for (int i = 0; i < modelList_->count(); ++i)
            if (modelPathAt(i) == path) {
                modelList_->setCurrentRow(i);
                return;
            }
        auto* item = new QListWidgetItem(QFileInfo(path).fileName(), modelList_);
        item->setData(kPathRole, path);
        item->setToolTip(path);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable);
        item->setCheckState(Qt::Checked);
        const QColor c = view_->modelDisplayColor(path);
        QPixmap swatch(14, 14);
        swatch.fill(c.isValid() ? c : QColor(184, 184, 184));
        item->setIcon(QIcon(swatch));
        modelList_->setCurrentItem(item);
    });
    createMenus();

    setWindowTitle(QString::fromUtf8("JMEngine - Unified Qt Editor"));
    resize(1280, 800);
    statusBar()->showMessage(
        QString::fromUtf8("左键旋转 | Alt+左键移动当前对象 | Ctrl+左键选择 | 右/中键平移 | Delete删除高亮"));

    if (!initialFile.isEmpty())
        addModelPath(initialFile);
}

MainWindow::~MainWindow() {
    if (scanner_ && scanner_->state() == JMEngine::ScanState::Scanning)
        scanner_->stop();
    if (reconstructionThread_.joinable())
        reconstructionThread_.join();
}


void MainWindow::resizeEvent(QResizeEvent* event) {
    QMainWindow::resizeEvent(event);
    updateCameraPreviewGeometry();
}

void MainWindow::updateCameraPreviewGeometry() {
    if (!cameraPreviewLabel_ || !view_) return;
    constexpr int margin = 12;
    const int x = std::max(margin, view_->width() - cameraPreviewLabel_->width() - margin);
    cameraPreviewLabel_->move(x, margin);
    cameraPreviewLabel_->raise();
}

void MainWindow::createActions() {
    openAction_ = new QAction(QString::fromUtf8("打开模型"), this);
    openAction_->setShortcut(QKeySequence::Open);
    connect(openAction_, &QAction::triggered, this, [this] { openModels(); });

    saveAction_ = new QAction(QString::fromUtf8("快速保存 PLY"), this);
    saveAction_->setShortcut(QKeySequence::Save);
    connect(saveAction_, &QAction::triggered, this, [this] { view_->saveActiveModel(); });

    exportAction_ = new QAction(QString::fromUtf8("导出模型..."), this);
    connect(exportAction_, &QAction::triggered, this, [this] { exportModel(); });

    touchEditAction_ = new QAction(QString::fromUtf8("编辑模式"), this);
    touchEditAction_->setCheckable(true);
    connect(touchEditAction_, &QAction::toggled, this, [this](bool on) {
        view_->setTouchEditMode(on);
        statusBar()->showMessage(on ? QString::fromUtf8("编辑模式：单指执行当前选择工具；双指仍可平移/缩放")
                                    : QString::fromUtf8("浏览模式：单指旋转；双指平移/缩放"));
    });

    objectMoveAction_ = new QAction(QString::fromUtf8("对象移动模式"), this);
    objectMoveAction_->setCheckable(true);
    objectMoveAction_->setToolTip(QString::fromUtf8("开启后左键拖动当前激活对象；也可随时按住 Alt + 左键临时移动"));
    connect(objectMoveAction_, &QAction::toggled, this, [this](bool on) {
        view_->setObjectMoveMode(on);
        statusBar()->showMessage(on ? QString::fromUtf8("对象移动模式：左键拖动当前激活对象；Ctrl+左键仍用于选择")
                                    : QString::fromUtf8("对象移动模式关闭；仍可按 Alt+左键临时移动当前激活对象"));
    });

    toolGroup_ = new QActionGroup(this);
    toolGroup_->setExclusive(true);
    auto makeTool = [this](const QString& text, PointCloudWidget::InteractionMode mode) {
        auto* a = new QAction(text, this);
        a->setCheckable(true);
        toolGroup_->addAction(a);
        connect(a, &QAction::triggered, this, [this, mode] { view_->setInteractionMode(mode); });
        return a;
    };
    rectangleAction_ = makeTool(QString::fromUtf8("矩形"), PointCloudWidget::InteractionMode::Rectangle);
    lassoAction_ = makeTool(QString::fromUtf8("套索"), PointCloudWidget::InteractionMode::Lasso);
    circleAction_ = makeTool(QString::fromUtf8("圆形"), PointCloudWidget::InteractionMode::Circle);
    brushAction_ = makeTool(QString::fromUtf8("画刷"), PointCloudWidget::InteractionMode::Brush);
    lassoAction_->setChecked(true);

    depthGroup_ = new QActionGroup(this);
    depthGroup_->setExclusive(true);
    surfaceAction_ = new QAction(QString::fromUtf8("表面选择"), this);
    throughAction_ = new QAction(QString::fromUtf8("穿透选择"), this);
    throughAction_->setVisible(true);
    throughAction_->setEnabled(true);
    surfaceAction_->setCheckable(true);
    throughAction_->setCheckable(true);
    surfaceAction_->setChecked(true);
    depthGroup_->addAction(surfaceAction_);
    depthGroup_->addAction(throughAction_);
    connect(surfaceAction_, &QAction::triggered, this,
            [this] { view_->setSelectionDepthMode(PointCloudWidget::SelectionDepthMode::Surface); });
    connect(throughAction_, &QAction::triggered, this,
            [this] { view_->setSelectionDepthMode(PointCloudWidget::SelectionDepthMode::Through); });

    deleteAction_ = new QAction(QString::fromUtf8("删除高亮"), this);
    deleteAction_->setShortcut(QKeySequence::Delete);
    connect(deleteAction_, &QAction::triggered, this, [this] { view_->deleteSelection(); });

    clearAction_ = new QAction(QString::fromUtf8("清除选择"), this);
    connect(clearAction_, &QAction::triggered, this, [this] { view_->clearSelection(); });

    keepAction_ = new QAction(QString::fromUtf8("仅保留选中"), this);
    connect(keepAction_, &QAction::triggered, this, [this] { view_->keepSelectionOnly(); });

    invertAction_ = new QAction(QString::fromUtf8("反选"), this);
    connect(invertAction_, &QAction::triggered, this, [this] { view_->invertSelection(); });

    compactAction_ = new QAction(QString::fromUtf8("压缩已删除数据"), this);
    connect(compactAction_, &QAction::triggered, this, [this] { view_->compactActiveModel(); });

    fitBasePlaneAction_ = new QAction(QString::fromUtf8("由当前选择拟合基底平面"), this);
    connect(fitBasePlaneAction_, &QAction::triggered, this, [this] {
        QString message;
        const bool ok = view_->fitBasePlaneFromSelection(&message);
        statusBar()->showMessage(message, 8000);
        if (!ok) QMessageBox::warning(this, QString::fromUtf8("基底平面"), message);
    });
    applyBasePlaneCutAction_ = new QAction(QString::fromUtf8("应用：删除基底平面以下内容"), this);
    connect(applyBasePlaneCutAction_, &QAction::triggered, this, [this] {
        QString message;
        const bool ok = view_->applyBasePlaneCut(&message);
        statusBar()->showMessage(message, 8000);
        if (!ok) QMessageBox::warning(this, QString::fromUtf8("基底裁剪"), message);
    });
    cancelBasePlaneCutAction_ = new QAction(QString::fromUtf8("取消基底平面"), this);
    connect(cancelBasePlaneCutAction_, &QAction::triggered, this, [this] { view_->cancelBasePlaneCut(); });

    undoAction_ = new QAction(QString::fromUtf8("撤销"), this);
    undoAction_->setShortcut(QKeySequence::Undo);
    connect(undoAction_, &QAction::triggered, this, [this] { view_->undoEdit(); });

    redoAction_ = new QAction(QString::fromUtf8("重做"), this);
    redoAction_->setShortcut(QKeySequence::Redo);
    connect(redoAction_, &QAction::triggered, this, [this] { view_->redoEdit(); });

    fitAction_ = new QAction(QString::fromUtf8("适配视图"), this);
    connect(fitAction_, &QAction::triggered, this, [this] { view_->fitView(); });

    // Picking：Desktop OpenGL 3.2+ 使用现代 R32UI/Geometry Shader；否则自动 CPU。
    pickingGroup_ = new QActionGroup(this);
    pickingGroup_->setExclusive(true);
    gpuPickingAction_ = new QAction(QString::fromUtf8("GPU Picking"), this);
    cpuPickingAction_ = new QAction(QString::fromUtf8("CPU Picking"), this);
    gpuPickingAction_->setCheckable(true);
    cpuPickingAction_->setCheckable(true);
    gpuPickingAction_->setChecked(true);
    pickingGroup_->addAction(gpuPickingAction_);
    pickingGroup_->addAction(cpuPickingAction_);
    connect(gpuPickingAction_, &QAction::triggered, this,
            [this] { view_->setPickingMode(PointCloudWidget::PickingMode::Gpu); });
    connect(cpuPickingAction_, &QAction::triggered, this, [this] {
        view_->setPickingMode(PointCloudWidget::PickingMode::Cpu);
        statusBar()->showMessage(QString::fromUtf8("CPU Picking：Surface 使用软件深度；Through 使用全点投影"));
    });

    displayGroup_ = new QActionGroup(this);
    displayGroup_->setExclusive(true);
    auto makeDisplay = [this](const QString& text, PointCloudWidget::DisplayMode mode) {
        auto* a = new QAction(text, this);
        a->setCheckable(true);
        displayGroup_->addAction(a);
        connect(a, &QAction::triggered, this, [this, mode] { view_->setDisplayMode(mode); });
        return a;
    };
    displayPointsAction_ = makeDisplay(QString::fromUtf8("点"), PointCloudWidget::DisplayMode::Points);
    displaySolidAction_ = makeDisplay(QString::fromUtf8("实体网格"), PointCloudWidget::DisplayMode::Solid);
    displayWireAction_ = makeDisplay(QString::fromUtf8("线框"), PointCloudWidget::DisplayMode::Wireframe);
    displaySolidWireAction_ =
        makeDisplay(QString::fromUtf8("实体 + 线框"), PointCloudWidget::DisplayMode::SolidWireframe);
    displaySolidAction_->setChecked(true);

    removeModelAction_ = new QAction(QString::fromUtf8("移除当前模型"), this);
    connect(removeModelAction_, &QAction::triggered, this, [this] { removeCurrentModel(); });

    modelColorAction_ = new QAction(QString::fromUtf8("设置模型颜色..."), this);
    connect(modelColorAction_, &QAction::triggered, this, [this] { setCurrentModelColor(); });

    measureDistanceAction_ = new QAction(QString::fromUtf8("两点距离"), this);
    connect(measureDistanceAction_, &QAction::triggered, this, [this] {
        view_->startDistanceMeasurement();
        statusBar()->showMessage(QString::fromUtf8("测量距离：依次点击两个表面点，Esc 可取消"));
    });
    measureAngleAction_ = new QAction(QString::fromUtf8("三点夹角"), this);
    connect(measureAngleAction_, &QAction::triggered, this, [this] {
        view_->startAngleMeasurement();
        statusBar()->showMessage(QString::fromUtf8("测量角度：依次点击 A、顶点 B、C，Esc 可取消"));
    });
    measureAreaAction_ = new QAction(QString::fromUtf8("网格表面积"), this);
    connect(measureAreaAction_, &QAction::triggered, this, [this] { view_->measureActiveSurfaceArea(); });
    measureVolumeAction_ = new QAction(QString::fromUtf8("封闭网格体积"), this);
    connect(measureVolumeAction_, &QAction::triggered, this, [this] { view_->measureActiveVolume(); });
    alignThreePointAction_ = new QAction(QString::fromUtf8("三点对齐"), this);
    connect(alignThreePointAction_, &QAction::triggered, this, [this] {
        view_->startThreePointAlignment();
        statusBar()->showMessage(QString::fromUtf8("三点对齐：先在移动模型点 3 点，再激活基准模型并点对应 3 点"));
    });
    autoAlignAction_ = new QAction(QString::fromUtf8("自动对齐（PCA + Trimmed ICP）"), this);
    connect(autoAlignAction_, &QAction::triggered, this, [this] {
        view_->startOrRunAutoAlignment();
        statusBar()->showMessage(QString::fromUtf8("自动对齐：首次选择移动模型，再切换到基准模型并再次执行"));
    });
    cancelToolAction_ = new QAction(QString::fromUtf8("取消测量/对齐"), this);
    cancelToolAction_->setShortcut(Qt::Key_Escape);
    connect(cancelToolAction_, &QAction::triggered, this, [this] { view_->cancelUtilityMode(); });
}

void MainWindow::createMenus() {
    auto* file = menuBar()->addMenu(QString::fromUtf8("文件"));
    file->addAction(openAction_);
    file->addAction(saveAction_);
    file->addAction(exportAction_);
    file->addSeparator();
    file->addAction(QString::fromUtf8("退出"), qApp, &QApplication::quit);

    auto* project = menuBar()->addMenu(QString::fromUtf8("工程"));
    auto* newProjectAction = project->addAction(QString::fromUtf8("新建工程..."));
    auto* openProjectAction = project->addAction(QString::fromUtf8("打开工程..."));
    auto* loadProjectAction = project->addAction(QString::fromUtf8("重建并载入工程点云"));
    project->addSeparator();
    auto* closeProjectAction = project->addAction(QString::fromUtf8("关闭工程"));
    connect(newProjectAction, &QAction::triggered, this, [this] { newProject(); });
    connect(openProjectAction, &QAction::triggered, this, [this] { openProject(); });
    connect(loadProjectAction, &QAction::triggered, this, [this] { loadProjectMergedCloud(); });
    connect(closeProjectAction, &QAction::triggered, this, [this] { closeProject(); });

    auto* edit = menuBar()->addMenu(QString::fromUtf8("编辑"));
    edit->addAction(touchEditAction_);
    edit->addAction(objectMoveAction_);
    edit->addSeparator();
    edit->addAction(deleteAction_);
    edit->addAction(keepAction_);
    edit->addAction(invertAction_);
    edit->addAction(clearAction_);
    edit->addSeparator();
    edit->addAction(undoAction_);
    edit->addAction(redoAction_);
    edit->addSeparator();
    edit->addAction(compactAction_);
    edit->addSeparator();
    auto* basePlaneMenu = edit->addMenu(QString::fromUtf8("基底平面裁剪"));
    basePlaneMenu->addAction(fitBasePlaneAction_);
    basePlaneMenu->addAction(applyBasePlaneCutAction_);
    basePlaneMenu->addAction(cancelBasePlaneCutAction_);

    auto* select = menuBar()->addMenu(QString::fromUtf8("选择"));
    select->addAction(rectangleAction_);
    select->addAction(lassoAction_);
    select->addAction(circleAction_);
    select->addAction(brushAction_);
    select->addSeparator();
    auto* depth = select->addMenu(QString::fromUtf8("选择深度"));
    depth->addAction(surfaceAction_);
    depth->addAction(throughAction_);

    // 所有处理算法共用一个 ProcessingDialog；菜单只选择 Core operation id。
    auto* pointMenu = menuBar()->addMenu(QString::fromUtf8("点云"));
    auto* pointDownsample = pointMenu->addMenu(QString::fromUtf8("降采样"));
    auto* pointDenoise = pointMenu->addMenu(QString::fromUtf8("去噪"));
    auto* pointNormal = pointMenu->addMenu(QString::fromUtf8("法向"));
    auto addProcessAction = [this](QMenu* menu, const QString& text, const char* id) {
        auto* action = menu->addAction(text);
        connect(action, &QAction::triggered, this, [this, op = std::string(id)] { openProcessingDialog(op); });
        return action;
    };
    addProcessAction(pointDownsample, QString::fromUtf8("体素降采样..."), "voxel");
    addProcessAction(pointDenoise, QString::fromUtf8("半径离群点..."), "radius_outlier");
    addProcessAction(pointDenoise, QString::fromUtf8("统计离群点..."), "statistical_outlier");
    addProcessAction(pointDenoise, QString::fromUtf8("删除小点云簇..."), "small_cluster");
    addProcessAction(pointNormal, QString::fromUtf8("估算法向..."), "normal_estimation");

    auto* meshProcessMenu = menuBar()->addMenu(QString::fromUtf8("网格"));
    auto* meshDenoise = meshProcessMenu->addMenu(QString::fromUtf8("去噪"));
    auto* meshSmooth = meshProcessMenu->addMenu(QString::fromUtf8("平滑"));
    auto* meshRepair = meshProcessMenu->addMenu(QString::fromUtf8("修复"));
    auto* meshReconstruct = meshProcessMenu->addMenu(QString::fromUtf8("重建"));
    addProcessAction(meshProcessMenu, QString::fromUtf8("网格清理..."), "mesh_cleanup");
    addProcessAction(meshDenoise, QString::fromUtf8("网格去噪..."), "mesh_denoise");
    addProcessAction(meshSmooth, QString::fromUtf8("Laplacian..."), "laplacian");
    addProcessAction(meshSmooth, QString::fromUtf8("Taubin..."), "taubin");
    addProcessAction(meshProcessMenu, QString::fromUtf8("QEM 简化..."), "qem_decimate");
    addProcessAction(meshRepair, QString::fromUtf8("检测/填充孔洞..."), "hole_fill");
    addProcessAction(meshReconstruct, QString::fromUtf8("工业泊松重建..."), "poisson_octree");
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    auto* textureMenu = meshProcessMenu->addMenu(QString::fromUtf8("纹理映射"));
    auto runTextureMapping = [this](JMEngine::texture::Backend backend) {
        statusBar()->showMessage(QString::fromUtf8("正在后台执行纹理映射..."));
        view_->startTextureMappingAsync(backend, [this](bool ok, const QString& message) {
            statusBar()->showMessage(message, 10000);
            if (!ok) QMessageBox::warning(this, QString::fromUtf8("纹理映射"), message);
        });
    };
    auto* textureAuto = textureMenu->addAction(QString::fromUtf8("自动（CUDA 优先）"));
    auto* textureCpu = textureMenu->addAction(QString::fromUtf8("CPU"));
    auto* textureCuda = textureMenu->addAction(QString::fromUtf8("CUDA"));
    connect(textureAuto, &QAction::triggered, this, [runTextureMapping] { runTextureMapping(JMEngine::texture::Backend::Auto); });
    connect(textureCpu, &QAction::triggered, this, [runTextureMapping] { runTextureMapping(JMEngine::texture::Backend::Cpu); });
    connect(textureCuda, &QAction::triggered, this, [runTextureMapping] { runTextureMapping(JMEngine::texture::Backend::Cuda); });
#endif

    auto* tools = menuBar()->addMenu(QString::fromUtf8("工具"));
    auto* measureMenu = tools->addMenu(QString::fromUtf8("测量"));
    measureMenu->addAction(measureDistanceAction_);
    measureMenu->addAction(measureAngleAction_);
    measureMenu->addSeparator();
    measureMenu->addAction(measureAreaAction_);
    measureMenu->addAction(measureVolumeAction_);
    auto* alignMenu = tools->addMenu(QString::fromUtf8("对齐"));
    alignMenu->addAction(alignThreePointAction_);
    alignMenu->addAction(autoAlignAction_);
    tools->addAction(cancelToolAction_);
    tools->addSeparator();
    auto* diagnosticsAction = tools->addAction(QString::fromUtf8("模型诊断..."));
    connect(diagnosticsAction, &QAction::triggered, this, [this] {
        statusBar()->showMessage(QString::fromUtf8("正在后台诊断当前模型..."));
        view_->analyzeActiveModelAsync([this](bool ok, const JMEngine::processing::ModelDiagnostics& diagnostics,
                                              const QString& error) {
            if (!ok) {
                statusBar()->showMessage(error);
                if (!error.isEmpty())
                    QMessageBox::warning(this, QString::fromUtf8("模型诊断"), error);
                return;
            }
            statusBar()->showMessage(QString::fromUtf8("模型诊断完成"));
            QMessageBox::information(this, QString::fromUtf8("模型诊断"),
                                     QString::fromUtf8(JMEngine::processing::diagnosticsSummary(diagnostics).c_str()));
        });
    });

    auto* view = menuBar()->addMenu(QString::fromUtf8("视图"));
    view->addAction(fitAction_);
    auto* display = view->addMenu(QString::fromUtf8("显示模式"));
    display->addAction(displayPointsAction_);
    display->addAction(displaySolidAction_);
    display->addAction(displayWireAction_);
    display->addAction(displaySolidWireAction_);
    view->addAction(modelDock_->toggleViewAction());
    if (scanDock_)
        view->addAction(scanDock_->toggleViewAction());

    auto* model = menuBar()->addMenu(QString::fromUtf8("模型"));
    model->addAction(openAction_);
    model->addAction(modelColorAction_);
    model->addAction(removeModelAction_);

    auto* settings = menuBar()->addMenu(QString::fromUtf8("设置"));
    auto* picking = settings->addMenu(QString::fromUtf8("Picking 后端"));
    picking->addAction(gpuPickingAction_);
    picking->addAction(cpuPickingAction_);

    auto* help = menuBar()->addMenu(QString::fromUtf8("帮助"));
    auto* about = help->addAction(QString::fromUtf8("关于"));
    connect(about, &QAction::triggered, this, [this] {
        QMessageBox::information(
            this, QString::fromUtf8("关于"),
            QString::fromUtf8("JMEngine\n统一 Qt 编辑器 + 可切换渲染后端\nDesktop OpenGL 2.1 / OpenGL ES "
                              "3.1\n支持触摸、OBJ/PLY/TXT/ASC、现代 R32UI GPU 表面选择（不可用自动 CPU）。"));
    });
}

void MainWindow::createModelManager() {
    modelDock_ = new QDockWidget(QString::fromUtf8("模型管理器"), this);
    auto* panel = new QWidget(modelDock_);
    auto* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(4);
    modelList_ = new QListWidget(panel);
    layout->addWidget(modelList_, 1);
    auto* colorButton = new QPushButton(QString::fromUtf8("模型颜色..."), panel);
    connect(colorButton, &QPushButton::clicked, this, [this] { setCurrentModelColor(); });
    layout->addWidget(colorButton);
    modelDock_->setWidget(panel);
    modelDock_->setMinimumWidth(240);
    addDockWidget(Qt::RightDockWidgetArea, modelDock_);

    connect(modelList_, &QListWidget::currentRowChanged, this, [this](int row) { activateRow(row); });
    connect(modelList_, &QListWidget::itemChanged, this, [this](QListWidgetItem* item) {
        if (!item)
            return;
        view_->setModelVisible(item->data(kPathRole).toString(), item->checkState() == Qt::Checked);
    });

    modelList_->setContextMenuPolicy(Qt::ActionsContextMenu);
    modelList_->addAction(modelColorAction_);
    modelList_->addAction(removeModelAction_);
    connect(modelList_, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem*) { setCurrentModelColor(); });
}

void MainWindow::createProjectManager() {
    projectDock_ = new QDockWidget(QString::fromUtf8("工程管理"), this);
    auto* panel = new QWidget(projectDock_);
    auto* root = new QVBoxLayout(panel);
    root->setContentsMargins(6, 6, 6, 6);
    root->setSpacing(6);

    auto* summary = new QGroupBox(QString::fromUtf8("当前工程"), panel);
    auto* form = new QFormLayout(summary);
    form->setContentsMargins(8, 8, 8, 8);
    form->setHorizontalSpacing(8);
    form->setVerticalSpacing(5);
    projectNameLabel_ = new QLabel(QStringLiteral("-"), summary);
    projectPathLabel_ = new QLabel(QStringLiteral("-"), summary);
    projectPathLabel_->setWordWrap(true);
    projectFrameCountLabel_ = new QLabel(QStringLiteral("0"), summary);
    projectOptimizedLabel_ = new QLabel(QString::fromUtf8("否"), summary);
    form->addRow(QString::fromUtf8("名称"), projectNameLabel_);
    form->addRow(QString::fromUtf8("路径"), projectPathLabel_);
    form->addRow(QString::fromUtf8("帧数"), projectFrameCountLabel_);
    form->addRow(QString::fromUtf8("已优化"), projectOptimizedLabel_);
    root->addWidget(summary);

    projectSaveScanCheck_ = new QCheckBox(QString::fromUtf8("扫描时写入当前工程"), panel);
    projectSaveScanCheck_->setChecked(true);
    root->addWidget(projectSaveScanCheck_);

    auto* buttonGrid = new QGridLayout();
    buttonGrid->setContentsMargins(0, 0, 0, 0);
    buttonGrid->setHorizontalSpacing(6);
    buttonGrid->setVerticalSpacing(6);
    projectNewButton_ = new QPushButton(QString::fromUtf8("新建"), panel);
    projectOpenButton_ = new QPushButton(QString::fromUtf8("打开"), panel);
    projectLoadMergedButton_ = new QPushButton(QString::fromUtf8("载入点云"), panel);
    projectCloseButton_ = new QPushButton(QString::fromUtf8("关闭"), panel);
    buttonGrid->addWidget(projectNewButton_, 0, 0);
    buttonGrid->addWidget(projectOpenButton_, 0, 1);
    buttonGrid->addWidget(projectLoadMergedButton_, 1, 0);
    buttonGrid->addWidget(projectCloseButton_, 1, 1);
    root->addLayout(buttonGrid);
    root->addStretch(1);

    connect(projectNewButton_, &QPushButton::clicked, this, [this] { newProject(); });
    connect(projectOpenButton_, &QPushButton::clicked, this, [this] { openProject(); });
    connect(projectLoadMergedButton_, &QPushButton::clicked, this, [this] { loadProjectMergedCloud(); });
    connect(projectCloseButton_, &QPushButton::clicked, this, [this] { closeProject(); });

    projectDock_->setWidget(panel);
    projectDock_->setMinimumWidth(260);
    addDockWidget(Qt::RightDockWidgetArea, projectDock_);
    if (modelDock_)
        tabifyDockWidget(modelDock_, projectDock_);
    updateProjectUi();
}

void MainWindow::updateProjectUi() {
    const bool hasProject = !currentProjectPath_.isEmpty();
    if (projectNameLabel_)
        projectNameLabel_->setText(hasProject ? QString::fromStdString(currentProject_.name) : QStringLiteral("-"));
    if (projectPathLabel_) {
        projectPathLabel_->setText(hasProject ? currentProjectPath_ : QStringLiteral("-"));
        projectPathLabel_->setToolTip(hasProject ? currentProjectPath_ : QString{});
    }
    if (projectFrameCountLabel_)
        projectFrameCountLabel_->setText(hasProject ? QString::number(currentProject_.frameCount) : QStringLiteral("0"));
    if (projectOptimizedLabel_)
        projectOptimizedLabel_->setText(hasProject && currentProject_.optimized ? QString::fromUtf8("是")
                                                                                : QString::fromUtf8("否"));
    if (projectSaveScanCheck_)
        projectSaveScanCheck_->setEnabled(hasProject);
    if (projectCloseButton_)
        projectCloseButton_->setEnabled(hasProject);
    if (projectLoadMergedButton_)
        projectLoadMergedButton_->setEnabled(hasProject);
    if (scanStartButton_) {
        const bool canStartForProject =
            hasProject && (!scanner_ || scanner_->state() == JMEngine::ScanState::Idle ||
                           scanner_->state() == JMEngine::ScanState::Error);
        scanStartButton_->setEnabled(canStartForProject);
    }
}

void MainWindow::newProject() {
    const auto path = nextDatedProjectPath();
    if (!projectManager_.createProject(path)) {
        QMessageBox::warning(this, QString::fromUtf8("新建工程"), QString::fromUtf8("无法创建工程目录或 project.json"));
        return;
    }
    JMEngine::ProjectInfo info;
    if (!projectManager_.openProject(path, info)) {
        QMessageBox::warning(this, QString::fromUtf8("新建工程"), QString::fromUtf8("工程已创建，但读取 project.json 失败"));
        return;
    }
    currentProject_ = info;
    currentProjectPath_ = qStringFromFsPath(info.path);
    updateProjectUi();
    statusBar()->showMessage(QString::fromUtf8("已按日期新建工程：") + currentProjectPath_, 8000);
}

void MainWindow::openProject() {
    const QString dir = QFileDialog::getExistingDirectory(
        this, QString::fromUtf8("打开工程目录"), defaultProjectWorkspacePath(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (dir.isEmpty())
        return;

    JMEngine::ProjectInfo info;
    if (!projectManager_.openProject(fsPathFromQString(dir), info)) {
        QMessageBox::warning(this, QString::fromUtf8("打开工程"), QString::fromUtf8("该目录不是有效工程，缺少 project.json"));
        return;
    }
    currentProject_ = info;
    currentProjectPath_ = qStringFromFsPath(info.path);
    updateProjectUi();
    statusBar()->showMessage(QString::fromUtf8("已打开工程：") + currentProjectPath_, 8000);
}

void MainWindow::closeProject() {
    currentProject_ = JMEngine::ProjectInfo{};
    currentProjectPath_.clear();
    updateProjectUi();
    statusBar()->showMessage(QString::fromUtf8("已关闭当前工程"), 5000);
}

void MainWindow::loadProjectMergedCloud() {
    if (currentProjectPath_.isEmpty()) {
        QMessageBox::information(this, QString::fromUtf8("工程点云"), QString::fromUtf8("请先打开或新建工程"));
        return;
    }
    JMEngine::ScanProject project;
    if (!project.openExisting(currentProjectPath_.toStdString())) {
        QMessageBox::warning(this, QString::fromUtf8("工程点云"), QString::fromUtf8("无法读取工程 frames 目录"));
        return;
    }
    const QDir resultDir(QDir(currentProjectPath_).filePath(QStringLiteral("result")));
    QDir().mkpath(resultDir.absolutePath());
    const QString output = resultDir.filePath(QStringLiteral("project_merged.ply"));
    if (!project.rebuildProjectCloud(output.toStdString())) {
        QMessageBox::warning(this, QString::fromUtf8("工程点云"), QString::fromUtf8("工程帧为空或合并点云失败"));
        return;
    }
    addModelPath(output);
    if (projectManager_.openProject(fsPathFromQString(currentProjectPath_), currentProject_))
        updateProjectUi();
    statusBar()->showMessage(QString::fromUtf8("已载入工程点云：") + output, 8000);
}

void MainWindow::createScanControl() {
    scanner_ = std::make_unique<JMEngine::JMScanner>();
    scanDock_ = new QDockWidget(QString::fromUtf8("扫描控制"), this);
    auto* panel = new QWidget(scanDock_);
    auto* root = new QVBoxLayout(panel);
    root->setContentsMargins(6, 6, 6, 6);
    root->setSpacing(6);

    // Full-width bottom scan panel. The dock owns the complete bottom edge while the
    // controls themselves use the available width instead of being stacked at the left.
    auto* quickGrid = new QGridLayout();
    quickGrid->setContentsMargins(0, 0, 0, 0);
    quickGrid->setHorizontalSpacing(8);
    quickGrid->setVerticalSpacing(5);

    scanSourceModeCombo_ = new QComboBox(panel);
    scanSourceModeCombo_->addItem(QString::fromUtf8("虚拟采集"), int(ScanSourceMode::Virtual));
    scanSourceModeCombo_->addItem(QString::fromUtf8("相机采集"), int(ScanSourceMode::Camera));
    scanSourceModeCombo_->setMinimumWidth(108);

    scanRegistrationModeCombo_ = new QComboBox(panel);
    scanRegistrationModeCombo_->addItem(QString::fromUtf8("几何拼接"), int(JMEngine::ScanRegistrationMode::Geometry));
    scanRegistrationModeCombo_->addItem(QString::fromUtf8("纹理拼接"), int(JMEngine::ScanRegistrationMode::Texture));
    scanRegistrationModeCombo_->setCurrentIndex(scanRegistrationModeCombo_->findData(int(JMEngine::ScanRegistrationMode::Texture)));
    scanRegistrationModeCombo_->addItem(QString::fromUtf8("标记点拼接"), int(JMEngine::ScanRegistrationMode::Marker));
    scanRegistrationModeCombo_->setMinimumWidth(118);
    scanRegistrationModeCombo_->setToolTip(QString::fromUtf8(
        "几何拼接：使用几何/深度约束；纹理拼接：使用图像特征辅助；标记点拼接：预留标记点约束入口。"));

    scanDataDirEdit_ = new QLineEdit(panel);
    scanCalibEdit_ = new QLineEdit(panel);
    scanVocabEdit_ = new QLineEdit(panel);
    cameraModelJsonEdit_ = new QLineEdit(panel);
    recordRawDataCheck_ = new QCheckBox(QString::fromUtf8("保存原始数据"), panel);
    recordRawDataCheck_->setChecked(false);
    rawDataDirEdit_ = new QLineEdit(panel);
    rawDataDirEdit_->setPlaceholderText(QString::fromUtf8("相机原始扫描保存目录"));
    scanMaxFramesSpin_ = new QSpinBox(panel);
    scanMaxFramesSpin_->setRange(1, 100000);
    scanMaxFramesSpin_->setValue(20000);

    auto makePathRow = [panel](QLineEdit* edit, const QString& buttonText, const std::function<void()>& choose) {
        auto* row = new QWidget(panel);
        auto* layout = new QHBoxLayout(row);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(4);
        layout->addWidget(edit, 1);
        auto* button = new QPushButton(buttonText, row);
        button->setFixedWidth(52);
        layout->addWidget(button);
        QObject::connect(button, &QPushButton::clicked, row, choose);
        return row;
    };

    cameraRefreshButton_ = new QPushButton(QString::fromUtf8("刷新"), panel);
    cameraRefreshButton_->setFixedWidth(58);
    cameraACombo_ = new QComboBox(panel);
    cameraBCombo_ = new QComboBox(panel);
    cameraACombo_->setMinimumWidth(170);
    cameraBCombo_->setMinimumWidth(170);

    cameraAModelLabel_ = new QLabel(QStringLiteral("-"), panel);
    cameraBModelLabel_ = new QLabel(QStringLiteral("-"), panel);
    cameraAModelLabel_->setToolTip(QString::fromUtf8("相机 A 型号 / VID / PID / 视频格式"));
    cameraBModelLabel_->setToolTip(QString::fromUtf8("相机 B 型号 / VID / PID / 视频格式"));
    cameraAExposureSpin_ = new QDoubleSpinBox(panel);
    cameraBExposureSpin_ = new QDoubleSpinBox(panel);
    // Exposure uses a compact progress-style slider in the visible scan bar.  The hidden
    // spin boxes keep the existing double-valued config/controller plumbing intact.
    cameraAExposureSlider_ = new QSlider(Qt::Horizontal, panel);
    cameraBExposureSlider_ = new QSlider(Qt::Horizontal, panel);
    cameraAExposureValueLabel_ = new QLabel(QStringLiteral("-6"), panel);
    cameraBExposureValueLabel_ = new QLabel(QStringLiteral("-6"), panel);
    liveOptimizationCheck_ = new QCheckBox(QString::fromUtf8("实时优化"), panel);
    liveOptimizationCheck_->setChecked(true);
    cameraABacklightSlider_ = new QSlider(Qt::Horizontal, panel);
    cameraABacklightSpin_ = new QDoubleSpinBox(panel);
    cameraBBacklightSlider_ = new QSlider(Qt::Horizontal, panel);
    cameraBBacklightSpin_ = new QDoubleSpinBox(panel);

    for (auto* spin : {cameraABacklightSpin_, cameraBBacklightSpin_}) {
        spin->setDecimals(0);
        spin->setRange(5.0, 25.0);
        spin->setSingleStep(1.0);
        spin->setValue(25.0);
        spin->setFixedWidth(54);
    }
    for (auto* slider : {cameraABacklightSlider_, cameraBBacklightSlider_}) {
        slider->setRange(5, 25);
        slider->setValue(25);
        slider->setMinimumWidth(100);
    }
    for (auto* spin : {cameraAExposureSpin_, cameraBExposureSpin_}) {
        spin->setRange(-20.0, 20.0);
        spin->setDecimals(2);
        spin->setSingleStep(1.0);
        spin->setValue(-6.0);
        spin->setFixedWidth(72);
    }
    for (auto* slider : {cameraAExposureSlider_, cameraBExposureSlider_}) {
        slider->setRange(-20, 20);
        slider->setValue(-6);
        slider->setMinimumWidth(150);
        slider->setMaximumWidth(240);
    }
    for (auto* label : {cameraAExposureValueLabel_, cameraBExposureValueLabel_})
        label->setFixedWidth(34);
    cameraAExposureSpin_->hide();
    cameraBExposureSpin_->hide();

    scanStateLabel_ = new QLabel(QString::fromUtf8("状态：空闲"), panel);
    scanStateLabel_->setMinimumWidth(115);
    scanStateLabel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    scanRenderFpsLabel_ = new QLabel(QString::fromUtf8("渲染 FPS：0.0"), panel);
    scanRenderFpsLabel_->setMinimumWidth(105);
    scanRenderFpsLabel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

    scanStartButton_ = new QPushButton(QString::fromUtf8("开始"), panel);
    scanStopButton_ = new QPushButton(QString::fromUtf8("结束"), panel);
    scanOfflineButton_ = new QPushButton(QString::fromUtf8("离线优化"), panel);
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    scanTextureButton_ = new QPushButton(QString::fromUtf8("一键处理"), panel);
    scanTextureButton_->setToolTip(QString::fromUtf8(
        "扫描完成并执行离线优化后使用。流程：点云去噪 → 删除小点云 → 泊松重建 → 网格去噪 → 纹理映射（有纹理帧时）。"));
    scanTextureFramesLabel_ = new QLabel(QString::fromUtf8("纹理帧：0"), panel);
    scanTextureFramesLabel_->setMinimumWidth(82);
#endif
    scanResetButton_ = new QPushButton(QString::fromUtf8("重置"), panel);
    for (auto* button : {scanStartButton_, scanStopButton_, scanOfflineButton_, scanResetButton_})
        button->setMinimumWidth(76);
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    scanTextureButton_->setMinimumWidth(86);
#endif

    // Row 0: keep acquisition/registration modes on their own compact row so they are
    // always visible even on narrow scan panels.  Previously they were placed before
    // the camera selectors in one very wide row and could be clipped completely.
    quickGrid->addWidget(new QLabel(QString::fromUtf8("采集"), panel), 0, 0);
    quickGrid->addWidget(scanSourceModeCombo_, 0, 1);
    quickGrid->addWidget(new QLabel(QString::fromUtf8("拼接模式"), panel), 0, 2);
    quickGrid->addWidget(scanRegistrationModeCombo_, 0, 3);
    quickGrid->addWidget(scanStateLabel_, 0, 4, 1, 2);
    quickGrid->setColumnStretch(5, 1);

    // Row 1: camera selection and workflow actions.
    quickGrid->addWidget(cameraRefreshButton_, 1, 0);
    quickGrid->addWidget(new QLabel(QString::fromUtf8("A码图"), panel), 1, 1);
    quickGrid->addWidget(cameraACombo_, 1, 2, 1, 2);
    quickGrid->addWidget(new QLabel(QString::fromUtf8("B彩色"), panel), 1, 4);
    quickGrid->addWidget(cameraBCombo_, 1, 5, 1, 2);
    quickGrid->addWidget(scanStartButton_, 1, 7);
    quickGrid->addWidget(scanStopButton_, 1, 8);
    quickGrid->addWidget(scanOfflineButton_, 1, 9);
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    quickGrid->addWidget(scanTextureButton_, 1, 10);
    quickGrid->addWidget(scanResetButton_, 1, 11);
#else
    quickGrid->addWidget(scanResetButton_, 1, 10);
#endif

    // Row 2: operator-facing exposure and live optimization controls.
    quickGrid->addWidget(new QLabel(QString::fromUtf8("A曝光"), panel), 2, 0);
    quickGrid->addWidget(cameraAExposureSlider_, 2, 1, 1, 3);
    quickGrid->addWidget(cameraAExposureValueLabel_, 2, 4);
    quickGrid->addWidget(new QLabel(QString::fromUtf8("B曝光"), panel), 2, 5);
    quickGrid->addWidget(cameraBExposureSlider_, 2, 6, 1, 3);
    quickGrid->addWidget(cameraBExposureValueLabel_, 2, 9);
    quickGrid->addWidget(liveOptimizationCheck_, 2, 10, 1, 2);

    quickGrid->addWidget(new QLabel(QString::fromUtf8("A逆光"), panel), 3, 0);
    quickGrid->addWidget(cameraABacklightSlider_, 3, 1, 1, 3);
    quickGrid->addWidget(cameraABacklightSpin_, 3, 4);
    quickGrid->addWidget(new QLabel(QString::fromUtf8("B逆光"), panel), 3, 5);
    quickGrid->addWidget(cameraBBacklightSlider_, 3, 6, 1, 3);
    quickGrid->addWidget(cameraBBacklightSpin_, 3, 9);

    quickGrid->addWidget(scanRenderFpsLabel_, 4, 0, 1, 2);
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    quickGrid->addWidget(scanTextureFramesLabel_, 4, 2, 1, 2);
#endif
    auto* renderFpsTimer = new QTimer(panel);
    renderFpsTimer->setInterval(500);
    connect(renderFpsTimer, &QTimer::timeout, panel, [this] {
        if (scanRenderFpsLabel_ && view_)
            scanRenderFpsLabel_->setText(QString::fromUtf8("渲染 FPS：%1").arg(view_->renderFps(), 0, 'f', 1));
    });
    renderFpsTimer->start();

    // Rendering must not be clocked by irregular SLAM-result callbacks.  A fixed UI render
    // cadence keeps camera following smooth while acquisition/SLAM continue at their own rate.
    auto* scanRenderTimer = new QTimer(panel);
    scanRenderTimer->setTimerType(Qt::PreciseTimer);
    scanRenderTimer->setInterval(33); // ~30 FPS; enough for a 10 FPS scanner without wasting GPU.
    connect(scanRenderTimer, &QTimer::timeout, panel, [this] {
        if (!view_ || !scanner_)
            return;
        const auto state = scanner_->state();
        if (state == JMEngine::ScanState::Scanning ||
            state == JMEngine::ScanState::Stopping) {
            view_->requestScanRenderFrame();
        }
    });
    scanRenderTimer->start();

    quickGrid->setColumnStretch(1, 2);
    quickGrid->setColumnStretch(6, 2);
    root->addLayout(quickGrid);

    auto* advancedButton = new QToolButton(panel);
    advancedButton->setText(QString::fromUtf8("高级设置"));
    advancedButton->setCheckable(true);
    advancedButton->setChecked(false);
    advancedButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    advancedButton->setArrowType(Qt::RightArrow);
    advancedButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    root->addWidget(advancedButton, 0, Qt::AlignLeft);

    auto* advancedWidget = new QWidget(panel);
    auto* advancedGrid = new QGridLayout(advancedWidget);
    advancedGrid->setContentsMargins(0, 2, 0, 0);
    advancedGrid->setHorizontalSpacing(8);
    advancedGrid->setVerticalSpacing(4);

    advancedGrid->addWidget(new QLabel(QString::fromUtf8("虚拟数据"), advancedWidget), 0, 0);
    advancedGrid->addWidget(makePathRow(scanDataDirEdit_, QString::fromUtf8("浏览"), [this] {
        const QString dir = QFileDialog::getExistingDirectory(this, QString::fromUtf8("选择虚拟扫描数据目录"),
                                                              scanDataDirEdit_->text());
        if (!dir.isEmpty()) scanDataDirEdit_->setText(dir);
    }), 0, 1);
    advancedGrid->addWidget(new QLabel(QString::fromUtf8("标定"), advancedWidget), 0, 2);
    advancedGrid->addWidget(makePathRow(scanCalibEdit_, QString::fromUtf8("浏览"), [this] {
        const QString file = QFileDialog::getOpenFileName(this, QString::fromUtf8("选择 rulermvs calib.txt"),
                                                          scanCalibEdit_->text(), QString::fromUtf8("Calibration (*.txt);;All (*.*)"));
        if (!file.isEmpty()) scanCalibEdit_->setText(file);
    }), 0, 3);

    advancedGrid->addWidget(new QLabel(QString::fromUtf8("Vocabulary"), advancedWidget), 1, 0);
    advancedGrid->addWidget(makePathRow(scanVocabEdit_, QString::fromUtf8("浏览"), [this] {
        const QString file = QFileDialog::getOpenFileName(this, QString::fromUtf8("选择 DBoW3 vocabulary"), scanVocabEdit_->text());
        if (!file.isEmpty()) scanVocabEdit_->setText(file);
    }), 1, 1);
    advancedGrid->addWidget(new QLabel(QString::fromUtf8("相机JSON"), advancedWidget), 1, 2);
    advancedGrid->addWidget(makePathRow(cameraModelJsonEdit_, QString::fromUtf8("浏览"), [this] {
        const QString file = QFileDialog::getOpenFileName(this, QString::fromUtf8("选择相机型号 JSON"),
                                                          cameraModelJsonEdit_->text(), QString::fromUtf8("JSON (*.json)"));
        if (!file.isEmpty()) cameraModelJsonEdit_->setText(file);
    }), 1, 3);

    advancedGrid->addWidget(recordRawDataCheck_, 2, 0);
    advancedGrid->addWidget(makePathRow(rawDataDirEdit_, QString::fromUtf8("浏览"), [this] {
        const QString dir = QFileDialog::getExistingDirectory(this, QString::fromUtf8("选择原始扫描保存目录"),
                                                              rawDataDirEdit_->text());
        if (!dir.isEmpty()) rawDataDirEdit_->setText(dir);
    }), 2, 1, 1, 3);

    cameraSyncToleranceSpin_ = new QDoubleSpinBox(panel);
    cameraSyncToleranceSpin_->setRange(0.1, 100.0);
    cameraSyncToleranceSpin_->setDecimals(1);
    cameraSyncToleranceSpin_->setSingleStep(1.0);
    cameraSyncToleranceSpin_->setValue(10.0);
    advancedGrid->addWidget(new QLabel(QString::fromUtf8("最大帧"), advancedWidget), 3, 0);
    advancedGrid->addWidget(scanMaxFramesSpin_, 3, 1);
    advancedGrid->addWidget(new QLabel(QString::fromUtf8("同步(ms)"), advancedWidget), 3, 2);
    advancedGrid->addWidget(cameraSyncToleranceSpin_, 3, 3);
    advancedGrid->addWidget(new QLabel(QString::fromUtf8("A型号"), advancedWidget), 4, 0);
    advancedGrid->addWidget(cameraAModelLabel_, 4, 1);
    advancedGrid->addWidget(new QLabel(QString::fromUtf8("B型号"), advancedWidget), 4, 2);
    advancedGrid->addWidget(cameraBModelLabel_, 4, 3);
    advancedGrid->setColumnStretch(1, 1);
    advancedGrid->setColumnStretch(3, 1);
    advancedWidget->setVisible(false);
    root->addWidget(advancedWidget);

    connect(advancedButton, &QToolButton::toggled, this, [advancedButton, advancedWidget](bool checked) {
        advancedWidget->setVisible(checked);
        advancedButton->setArrowType(checked ? Qt::DownArrow : Qt::RightArrow);
    });

    // The dock intentionally spans the complete bottom edge. Assign the bottom area to
    // both corners so a right-side model dock does not steal the bottom-right corner.
    panel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    scanDock_->setWidget(panel);
    scanDock_->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    scanDock_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    setCorner(Qt::BottomLeftCorner, Qt::BottomDockWidgetArea);
    setCorner(Qt::BottomRightCorner, Qt::BottomDockWidgetArea);
    addDockWidget(Qt::BottomDockWidgetArea, scanDock_);

    scanSourceModeCombo_->setCurrentIndex(0);
    scanDataDirEdit_->clear();
    scanCalibEdit_->clear();
    QString defaultVocabulary;
    {
        QDir appDir(QCoreApplication::applicationDirPath());
        const QStringList vocabFiles = appDir.entryList(QStringList() << QStringLiteral("*.yml.gz"), QDir::Files, QDir::Name);
        if (!vocabFiles.isEmpty()) defaultVocabulary = appDir.filePath(vocabFiles.first());
    }
    scanVocabEdit_->setText(defaultVocabulary);
    scanMaxFramesSpin_->setValue(20000);
    if (liveOptimizationCheck_) liveOptimizationCheck_->setChecked(true);
    cameraModelJsonEdit_->setText(defaultCameraModelJsonPath());
    {
        QString jsonError;
        const auto calibration = loadCalibrationPath(cameraModelJsonEdit_->text().trimmed(), &jsonError);
        if (!calibration.isEmpty()) scanCalibEdit_->setText(calibration);
    }
    cameraAExposureSpin_->setValue(-6.0);
    cameraBExposureSpin_->setValue(-6.0);
    cameraABacklightSpin_->setValue(25.0);
    cameraABacklightSlider_->setValue(25);
    cameraBBacklightSpin_->setValue(25.0);
    cameraBBacklightSlider_->setValue(25);
    cameraSyncToleranceSpin_->setValue(10.0);
    if (rawDataDirEdit_)
        rawDataDirEdit_->setText(QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("scan_raw")));
    if (scanSourceModeCombo_->currentData().toInt() == int(ScanSourceMode::Virtual) &&
        !scanDataDirEdit_->text().trimmed().isEmpty()) {
        scanCalibEdit_->setText(QDir(scanDataDirEdit_->text().trimmed()).filePath(QStringLiteral("calib.txt")));
    }

    connect(scanDataDirEdit_, &QLineEdit::textChanged, this, [this](const QString& dir) {
        if (!scanSourceModeCombo_ || !scanCalibEdit_)
            return;
        if (scanSourceModeCombo_->currentData().toInt() != int(ScanSourceMode::Virtual))
            return;
        const QString trimmed = dir.trimmed();
        scanCalibEdit_->setText(trimmed.isEmpty() ? QString{} : QDir(trimmed).filePath(QStringLiteral("calib.txt")));
    });

    connect(scanSourceModeCombo_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] {
        applyScanSourceUi();
        if (scanSourceModeCombo_->currentData().toInt() == int(ScanSourceMode::Virtual) &&
            scanDataDirEdit_ && scanCalibEdit_ && !scanDataDirEdit_->text().trimmed().isEmpty()) {
            scanCalibEdit_->setText(QDir(scanDataDirEdit_->text().trimmed()).filePath(QStringLiteral("calib.txt")));
        }
        if (scanSourceModeCombo_->currentData().toInt() == int(ScanSourceMode::Camera)) {
            if (cameraModelJsonEdit_ && scanCalibEdit_) {
                QString jsonError;
                const auto calibration = loadCalibrationPath(cameraModelJsonEdit_->text().trimmed(), &jsonError);
                if (!calibration.isEmpty()) scanCalibEdit_->setText(calibration);
            }
            refreshCameras();
        }
    });
    connect(recordRawDataCheck_, &QCheckBox::toggled, this, [this](bool enabled) {
        if (rawDataDirEdit_)
            rawDataDirEdit_->setEnabled(enabled && scanSourceModeCombo_ &&
                scanSourceModeCombo_->currentData().toInt() == int(ScanSourceMode::Camera));
    });
    connect(cameraModelJsonEdit_, &QLineEdit::editingFinished, this, [this] {
        if (!scanSourceModeCombo_ || !scanCalibEdit_ || !cameraModelJsonEdit_) return;
        if (scanSourceModeCombo_->currentData().toInt() != int(ScanSourceMode::Camera)) return;
        QString jsonError;
        const auto calibration = loadCalibrationPath(cameraModelJsonEdit_->text().trimmed(), &jsonError);
        if (!calibration.isEmpty()) scanCalibEdit_->setText(calibration);
    });

    connect(cameraRefreshButton_, &QPushButton::clicked, this, [this] {
        statusBar()->showMessage(QString::fromUtf8("正在后台枚举相机..."));
        refreshCameras();
    });
    connect(cameraACombo_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] { updateCameraSelectionUi(); });
    connect(cameraBCombo_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] { updateCameraSelectionUi(); });
    connect(cameraAExposureSlider_, &QSlider::valueChanged, this, [this](int value) {
        if (cameraAExposureValueLabel_) cameraAExposureValueLabel_->setText(QString::number(value));
        if (cameraAExposureSpin_) { QSignalBlocker b(cameraAExposureSpin_); cameraAExposureSpin_->setValue(double(value)); }
        if (scanner_) scanner_->setCameraExposure(0, double(value));
    });
    connect(cameraBExposureSlider_, &QSlider::valueChanged, this, [this](int value) {
        if (cameraBExposureValueLabel_) cameraBExposureValueLabel_->setText(QString::number(value));
        if (cameraBExposureSpin_) { QSignalBlocker b(cameraBExposureSpin_); cameraBExposureSpin_->setValue(double(value)); }
        if (scanner_) scanner_->setCameraExposure(1, double(value));
    });
    connect(cameraABacklightSlider_, &QSlider::valueChanged, this, [this](int value) {
        if (!cameraABacklightSpin_) return;
        const QSignalBlocker blocker(cameraABacklightSpin_);
        cameraABacklightSpin_->setValue(double(value));
        if (scanner_) scanner_->setCameraBacklight(0, double(value));
    });
    connect(cameraABacklightSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (cameraABacklightSlider_) {
            const QSignalBlocker blocker(cameraABacklightSlider_);
            cameraABacklightSlider_->setValue(int(std::lround(value)));
        }
        if (scanner_) scanner_->setCameraBacklight(0, value);
    });

    connect(cameraBBacklightSlider_, &QSlider::valueChanged, this, [this](int value) {
        if (!cameraBBacklightSpin_) return;
        const QSignalBlocker blocker(cameraBBacklightSpin_);
        cameraBBacklightSpin_->setValue(double(value));
        if (scanner_) scanner_->setCameraBacklight(1, double(value));
    });
    connect(cameraBBacklightSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (cameraBBacklightSlider_) {
            const QSignalBlocker blocker(cameraBBacklightSlider_);
            cameraBBacklightSlider_->setValue(int(std::lround(value)));
        }
        if (scanner_) scanner_->setCameraBacklight(1, value);
    });

    connect(scanStartButton_, &QPushButton::clicked, this, [this] {
        if (currentProjectPath_.isEmpty()) {
            QMessageBox::information(this, QString::fromUtf8("开始扫描"), QString::fromUtf8("请先新建或打开工程"));
            statusBar()->showMessage(QString::fromUtf8("请先新建工程，再开始扫描"), 8000);
            updateProjectUi();
            return;
        }
        ScanUiConfig cfg = scanConfigFromUi();
        if (cfg.sourceMode == ScanSourceMode::Camera) {
            const CameraDeviceInfo* cameraAInfo = nullptr;
            const CameraDeviceInfo* cameraBInfo = nullptr;
            for (const auto& d : cameraInfos_) {
                if (d.deviceId == cfg.cameraADeviceId) cameraAInfo = &d;
                if (d.deviceId == cfg.cameraBDeviceId) cameraBInfo = &d;
            }
            if (!cameraAInfo || !cameraBInfo) {
                statusBar()->showMessage(QString::fromUtf8("请选择有效的 A/B 相机"), 8000);
                return;
            }
            if (cameraAInfo->deviceId == cameraBInfo->deviceId) {
                statusBar()->showMessage(QString::fromUtf8("A/B 不能选择同一个相机设备"), 8000);
                return;
            }
            if (!cameraAInfo->modelConfigured || !cameraBInfo->modelConfigured) {
                const QString missing = !cameraAInfo->modelConfigured
                    ? cameraAInfo->friendlyName : cameraBInfo->friendlyName;
                statusBar()->showMessage(
                    QString::fromUtf8("相机未匹配 camera_models.json 的 model + VID + PID：%1").arg(missing), 10000);
                return;
            }
            if (cameraAInfo->modelName.compare(cameraBInfo->modelName, Qt::CaseInsensitive) != 0) {
                statusBar()->showMessage(
                    QString::fromUtf8("A/B 相机型号不一致：%1 / %2")
                        .arg(cameraAInfo->modelName, cameraBInfo->modelName), 10000);
                return;
            }
        }
        // Virtual mode always derives <dataDir>/calib.txt. Camera mode keeps the
        // last calibration path in camera_models.json.
        if (cfg.sourceMode == ScanSourceMode::Camera && !cfg.cameraModelJsonPath.isEmpty()) {
            QString saveError;
            if (!saveCalibrationPath(cfg.cameraModelJsonPath, QString::fromStdString(cfg.engine.calibrationPath), &saveError) && !saveError.isEmpty())
                statusBar()->showMessage(saveError);
        }
        if (cfg.sourceMode == ScanSourceMode::Camera && cfg.recordRawData && cfg.rawDataDir.isEmpty()) {
            statusBar()->showMessage(QString::fromUtf8("请选择原始扫描保存目录"), 8000);
            return;
        }
        if (cfg.sourceMode == ScanSourceMode::Camera && cfg.recordRawData) {
            const QString sessionName = QStringLiteral("scan_%1").arg(
                QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")));
            cfg.rawDataDir = QDir(cfg.rawDataDir).filePath(sessionName);
            cfg.cameras.rawDataDirectory = cfg.rawDataDir.toStdString();
            statusBar()->showMessage(QString::fromUtf8("原始数据保存到：%1").arg(cfg.rawDataDir), 8000);
        }
        removeScanModelListEntry();
        latestMarkerFrame_ = JMEngine::ScanMarkerFrame{};
        lastScanVisualFrameId_ = -1;
        view_->clearScanPreview();
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
        view_->setTextureFrames(nullptr);
        scanTextureFramesReady_ = false;
        if (scanTextureFramesLabel_) scanTextureFramesLabel_->setText(QString::fromUtf8("纹理帧：0"));
        if (scanTextureButton_) scanTextureButton_->setEnabled(false);
#endif
        view_->beginScanPreview(static_cast<std::size_t>(cfg.previewPointLimit));
        activeScanConfig_ = cfg;
        if (!scanner_->initialize(cfg.engine)) {
            statusBar()->showMessage(QString::fromStdString(scanner_->lastError()), 12000);
            return;
        }
        const bool started = cfg.sourceMode == ScanSourceMode::Camera
            ? scanner_->startCameras(cfg.cameras)
            : scanner_->startDataset(cfg.dataDir.toStdString());
        if (!started) statusBar()->showMessage(QString::fromStdString(scanner_->lastError()), 12000);
    });
    connect(scanStopButton_, &QPushButton::clicked, this, [this] { if (scanner_) scanner_->stop(); });
    connect(scanOfflineButton_, &QPushButton::clicked, this, [this] {
        if (!scanner_) return;
        if (reconstructionThread_.joinable()) reconstructionThread_.join();
        const QString projectPath = currentProjectPath_;
        reconstructionThread_ = std::thread([this, projectPath] {
            const bool ok = scanner_->reconstruct();
            const auto cloud = scanner_->resultCloud();
            const auto error = scanner_->lastError();
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
            PointCloudWidget::TextureFramesPtr textureFrames;
            const auto keyframes = scanner_->takeTextureKeyframes();
            textureFrames = std::make_shared<std::vector<JMEngine::texture::CameraFrame>>();
            textureFrames->reserve(keyframes.size());
            for (const auto& keyframe : keyframes) {
                if (!keyframe.rgb) continue;
                JMEngine::texture::CameraFrame frame;
                frame.frameId = keyframe.frameId;
                frame.image.width = keyframe.width;
                frame.image.height = keyframe.height;
                frame.image.pixels = *keyframe.rgb;
                frame.fx = keyframe.fx; frame.fy = keyframe.fy;
                frame.cx = keyframe.cx; frame.cy = keyframe.cy;
                frame.worldToCamera.m = keyframe.worldToCamera.matrix;
                textureFrames->push_back(std::move(frame));
            }
            QMetaObject::invokeMethod(this, [this, ok, cloud, error, projectPath, textureFrames = std::move(textureFrames)] {
                if (ok && cloud) view_->replaceScanPreview(cloud);
                else if (!error.empty()) statusBar()->showMessage(QString::fromStdString(error), 12000);
                if (ok && !projectPath.isEmpty()) {
                    QString projectError;
                    if (!writeProjectOptimizedFlag(projectPath, true, &projectError) && !projectError.isEmpty())
                        statusBar()->showMessage(projectError, 10000);
                    if (projectManager_.openProject(fsPathFromQString(projectPath), currentProject_)) {
                        currentProjectPath_ = projectPath;
                        updateProjectUi();
                    }
                }
                const std::size_t count = textureFrames ? textureFrames->size() : 0u;
                scanTextureFramesReady_ = count > 0u;
                if (scanTextureFramesLabel_)
                    scanTextureFramesLabel_->setText(QString::fromUtf8("纹理帧：%1").arg(static_cast<qulonglong>(count)));
                if (view_) view_->setTextureFrames(textureFrames);
                if (scanTextureButton_) scanTextureButton_->setEnabled(ok);
            }, Qt::QueuedConnection);
#else
            QMetaObject::invokeMethod(this, [this, ok, cloud, error, projectPath] {
                if (ok && cloud) view_->replaceScanPreview(cloud);
                else if (!error.empty()) statusBar()->showMessage(QString::fromStdString(error), 12000);
                if (ok && !projectPath.isEmpty()) {
                    QString projectError;
                    if (!writeProjectOptimizedFlag(projectPath, true, &projectError) && !projectError.isEmpty())
                        statusBar()->showMessage(projectError, 10000);
                    if (projectManager_.openProject(fsPathFromQString(projectPath), currentProject_)) {
                        currentProjectPath_ = projectPath;
                        updateProjectUi();
                    }
                }
            }, Qt::QueuedConnection);
#endif
        });
    });
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    connect(scanTextureButton_, &QPushButton::clicked, this, [this] {
        if (!view_) return;
        scanTextureButton_->setEnabled(false);
        statusBar()->showMessage(QString::fromUtf8("正在一键处理：点云去噪 → 小点云去除 → 泊松 → 网格去噪 → 纹理..."));
        const bool started = view_->startScanTextureMappingAsync(
            JMEngine::texture::Backend::Auto,
            [this](float progress, const QString& stage) {
                statusBar()->showMessage(QString::fromUtf8("一键处理 %1%：%2")
                                             .arg(int(std::lround(progress * 100.0f)))
                                             .arg(stage));
            },
            [this](bool ok, const QString& message) {
                const auto state = scanner_ ? scanner_->state() : JMEngine::ScanState::Idle;
                if (scanTextureButton_)
                    scanTextureButton_->setEnabled(state == JMEngine::ScanState::ReadyForReconstruction);
                statusBar()->showMessage(message, 12000);
                if (!ok) QMessageBox::warning(this, QString::fromUtf8("一键处理"), message);
            });
        if (!started) {
            const auto state = scanner_ ? scanner_->state() : JMEngine::ScanState::Idle;
            scanTextureButton_->setEnabled(state == JMEngine::ScanState::ReadyForReconstruction);
        }
    });
#endif
    connect(scanResetButton_, &QPushButton::clicked, this, [this] {
        if (scanner_) scanner_->reset();
        removeScanModelListEntry();
        latestMarkerFrame_ = JMEngine::ScanMarkerFrame{};
        lastScanVisualFrameId_ = -1;
        view_->clearScanPreview();
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
        view_->setTextureFrames(nullptr);
        scanTextureFramesReady_ = false;
        if (scanTextureFramesLabel_) scanTextureFramesLabel_->setText(QString::fromUtf8("纹理帧：0"));
        if (scanTextureButton_) scanTextureButton_->setEnabled(false);
#endif
    });

    scanner_->setStateCallback([this](JMEngine::ScanState state) {
        QMetaObject::invokeMethod(this, [this, state] { applyScanState(state); }, Qt::QueuedConnection);
    });
    scanner_->setMessageCallback([this](const std::string& message) {
        QMetaObject::invokeMethod(this, [this, message] { statusBar()->showMessage(QString::fromStdString(message)); }, Qt::QueuedConnection);
    });
    scanner_->setProgressCallback([this](int progress) {
        QMetaObject::invokeMethod(this, [this, progress] {
            statusBar()->showMessage(QString::fromUtf8("离线优化 %1%").arg(progress));
        }, Qt::QueuedConnection);
    });
    scanner_->setFrameCallback([this](int frameId, const JMEngine::Pose& pose,
                                      std::shared_ptr<JMEngine::PointCloud> cloud,
                                      std::shared_ptr<JMEngine::PointCloud> statusCloud,
                                      bool trackingOk) {
        const auto queuedAt = std::chrono::steady_clock::now();
        QMetaObject::invokeMethod(this, [this, frameId, pose, cloud = std::move(cloud),
                                              statusCloud = std::move(statusCloud),
                                              trackingOk, queuedAt]() mutable {
            const double queueMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - queuedAt).count();
            QElapsedTimer dispatchPerf;
            dispatchPerf.start();

            if (view_ && trackingOk && cloud && !cloud->empty()) {
                auto points = std::make_shared<std::vector<JMEngine::Point>>(
                    std::move(cloud->points()));
                view_->appendScanLocalFrame(frameId, points, pose.matrix);
            }

            // Accumulated live cloud may accept a late/out-of-order frame, but the current
            // frame and observer must never move backwards to an older SLAM result. RGBDFusion
            // can invoke trace callbacks from several worker threads, so callback arrival order
            // is not guaranteed to match frameId order.
            const bool newestVisualFrame = frameId > lastScanVisualFrameId_;
            if (newestVisualFrame) {
                lastScanVisualFrameId_ = frameId;
                if (view_ && statusCloud && !statusCloud->empty()) {
                    auto points = std::make_shared<std::vector<JMEngine::Point>>(
                        std::move(statusCloud->points()));
                    view_->setCurrentScanFrame(points, trackingOk);
                }

                PointCloudWidget::ScanCameraViewPose viewPose;
                viewPose.position = {pose.matrix[12], pose.matrix[13], pose.matrix[14]};
                viewPose.right = {pose.matrix[0], pose.matrix[1], pose.matrix[2]};
                viewPose.up = {-pose.matrix[4], -pose.matrix[5], -pose.matrix[6]};
                viewPose.forward = {pose.matrix[8], pose.matrix[9], pose.matrix[10]};
                viewPose.trackingOk = trackingOk;
                viewPose.frameId = frameId;
                if (view_)
                    view_->updateScanCameraPose(viewPose);
            }

            const double dispatchMs = double(dispatchPerf.nsecsElapsed()) / 1000000.0;
            if (queueMs > 20.0 || dispatchMs > 10.0) {
                qInfo().noquote() << QStringLiteral("[SCAN STALL][QT] frame=%1 queued=%2ms dispatch=%3ms")
                    .arg(frameId).arg(queueMs, 0, 'f', 2).arg(dispatchMs, 0, 'f', 2);
            }
        }, Qt::QueuedConnection);
    });
    scanner_->setPoseUpdateCallback(
        [this](std::vector<JMEngine::FramePoseUpdate> updates) {
            if (updates.empty())
                return;
            auto renderUpdates =
                std::make_shared<std::vector<PointCloudWidget::LiveFramePoseUpdate>>();
            renderUpdates->reserve(updates.size());
            for (const auto& update : updates)
                renderUpdates->push_back({update.frameId, update.pose.matrix});

            QMetaObject::invokeMethod(
                this,
                [this, renderUpdates] {
                    if (view_ && activeScanConfig_.liveOptimizationEnabled)
                        view_->updateScanFramePoses(renderUpdates);
                },
                Qt::QueuedConnection);
        });

    scanner_->setMarkerCallback([this](const JMEngine::ScanMarkerFrame& frame) {
        QMetaObject::invokeMethod(this, [this, frame] {
        latestMarkerFrame_ = frame;
        if (scanRegistrationModeCombo_ &&
            scanRegistrationModeCombo_->currentData().toInt() == int(JMEngine::ScanRegistrationMode::Marker)) {
            std::vector<std::array<float,3>> markers3d;
            markers3d.reserve(frame.markers.size());
            for (const auto& m : frame.markers) if (m.hasDepth) markers3d.push_back(m.point3d);
            if (view_ && !markers3d.empty()) view_->setScanFrameMarkers(frame.frameId, markers3d);
            statusBar()->showMessage(QString::fromUtf8("标记点 frame=%1：3D标记 %2 个")
                                         .arg(frame.frameId).arg(markers3d.size()), 1000);
        }
        }, Qt::QueuedConnection);
    });

    scanner_->setCameraPreviewCallback([this](std::shared_ptr<std::vector<std::uint8_t>> pixels, int width, int height) {
        if (!pixels || width <= 0 || height <= 0) return;
        QImage image(pixels->data(), width, height, width * 3, QImage::Format_RGB888);
        const QImage owned = image.copy();
        QMetaObject::invokeMethod(this, [this, owned] {
        if (!cameraPreviewLabel_ || owned.isNull() || !scanner_) return;
        const bool cameraScanning = scanner_->state() == JMEngine::ScanState::Scanning &&
                                    scanSourceModeCombo_ &&
                                    scanSourceModeCombo_->currentData().toInt() == int(ScanSourceMode::Camera);
        if (!cameraScanning) {
            cameraPreviewLabel_->clear();
            cameraPreviewLabel_->hide();
            return;
        }
        QImage display = owned.convertToFormat(QImage::Format_RGB888);
        const QSize box = cameraPreviewLabel_->size();
        cameraPreviewLabel_->setPixmap(QPixmap::fromImage(display).scaled(box, Qt::KeepAspectRatio, Qt::FastTransformation));
        cameraPreviewLabel_->show();
        cameraPreviewLabel_->raise();
        updateCameraPreviewGeometry();
 
        }, Qt::QueuedConnection);
    });

    applyScanSourceUi();
    applyScanState(JMEngine::ScanState::Idle);
    if (scanSourceModeCombo_->currentData().toInt() == int(ScanSourceMode::Camera)) {
        refreshCameras();
    }
}

MainWindow::ScanUiConfig MainWindow::scanConfigFromUi() const {
    ScanUiConfig cfg;
    cfg.sourceMode = scanSourceModeCombo_ && scanSourceModeCombo_->currentData().toInt() == int(ScanSourceMode::Camera)
                         ? ScanSourceMode::Camera : ScanSourceMode::Virtual;
    if (scanRegistrationModeCombo_) {
        const int mode = scanRegistrationModeCombo_->currentData().toInt();
        if (mode == int(JMEngine::ScanRegistrationMode::Texture)) cfg.engine.registrationMode = JMEngine::ScanRegistrationMode::Texture;
        else if (mode == int(JMEngine::ScanRegistrationMode::Marker)) cfg.engine.registrationMode = JMEngine::ScanRegistrationMode::Marker;
        else cfg.engine.registrationMode = JMEngine::ScanRegistrationMode::Geometry;
    }
    cfg.dataDir = scanDataDirEdit_ ? scanDataDirEdit_->text().trimmed() : QString{};
    cfg.engine.calibrationPath = scanCalibEdit_ ? scanCalibEdit_->text().trimmed().toStdString() : std::string{};
    cfg.engine.vocabularyPath = scanVocabEdit_ ? scanVocabEdit_->text().trimmed().toStdString() : std::string{};
    cfg.engine.maxFrames = scanMaxFramesSpin_ ? scanMaxFramesSpin_->value() : 20000;
    cfg.engine.maxInflightFrames = 6;
    // Live preview is bounded but must represent the whole scan. PointCloudWidget compacts
    // old preview samples when this budget is reached instead of dropping all later frames.
    cfg.engine.previewPointsPerFrame = 300;
    cfg.previewPointLimit = 20000000;
    cfg.engine.previewPointLimit = cfg.previewPointLimit;
    cfg.engine.offlineVoxel = 3.0;
    cfg.engine.offlineIterations = 30;
    cfg.liveOptimizationEnabled = liveOptimizationCheck_ ? liveOptimizationCheck_->isChecked() : true;
    cfg.engine.liveOptimizationEnabled = cfg.liveOptimizationEnabled;
    cfg.engine.saveScanProject = projectSaveScanCheck_ && projectSaveScanCheck_->isChecked() &&
                                 !currentProjectPath_.isEmpty();
    cfg.engine.scanProjectPath = cfg.engine.saveScanProject ? currentProjectPath_.toStdString() : std::string{};
    cfg.cameraModelJsonPath = cameraModelJsonEdit_ ? cameraModelJsonEdit_->text().trimmed() : QString{};
    cfg.recordRawData = recordRawDataCheck_ && recordRawDataCheck_->isChecked();
    cfg.rawDataDir = rawDataDirEdit_ ? rawDataDirEdit_->text().trimmed() : QString{};
    cfg.cameraADeviceId = cameraACombo_ ? cameraACombo_->currentData().toString() : QString{};
    cfg.cameraBDeviceId = cameraBCombo_ ? cameraBCombo_->currentData().toString() : QString{};
    cfg.cameras.cameraA.exposure = cameraAExposureSpin_ ? cameraAExposureSpin_->value() : -6.0;
    cfg.cameras.cameraB.exposure = cameraBExposureSpin_ ? cameraBExposureSpin_->value() : -6.0;
    cfg.cameras.cameraA.backlight = cameraABacklightSpin_ ? cameraABacklightSpin_->value() : 25.0;
    cfg.cameras.cameraB.backlight = cameraBBacklightSpin_ ? cameraBBacklightSpin_->value() : 25.0;
    cfg.cameras.syncToleranceMs = cameraSyncToleranceSpin_ ? cameraSyncToleranceSpin_->value() : 10.0;
    cfg.cameras.queueDepth = 3;
    cfg.cameras.recordRawData = cfg.sourceMode == ScanSourceMode::Camera && cfg.recordRawData;
    cfg.cameras.rawDataDirectory = cfg.rawDataDir.toStdString();
    cfg.cameras.calibrationPath = cfg.engine.calibrationPath;
    auto applyCamera = [this](const QString& id, JMEngine::CameraDeviceConfig& out) {
        for (const auto& d : cameraInfos_) if (d.deviceId == id) {
            out.index = d.cvIndex;
            out.width = d.width;
            out.height = d.height;
            out.fps = int(std::lround(d.fps));
            out.fourcc = d.fourcc.toStdString();
            out.model = d.modelName.toStdString();
            out.rotate = d.rotate;
            return;
        }
    };
    applyCamera(cfg.cameraADeviceId, cfg.cameras.cameraA);
    applyCamera(cfg.cameraBDeviceId, cfg.cameras.cameraB);
    return cfg;
}

void MainWindow::refreshCameras() {
    const QString jsonPath = cameraModelJsonEdit_ ? cameraModelJsonEdit_->text().trimmed() : QString{};
    QPointer<MainWindow> self(this);
    std::thread([self, jsonPath] {
        std::string nativeError;
        const auto nativeDevices = JMEngine::enumerateCameraDevices(&nativeError);
        QJsonArray profiles;
        QFile file(jsonPath);
        QString jsonError;
        if (file.open(QIODevice::ReadOnly)) {
            QJsonParseError parseError{};
            const auto doc = QJsonDocument::fromJson(file.readAll(), &parseError);
            if (parseError.error == QJsonParseError::NoError && doc.isObject())
                profiles = doc.object().value(QStringLiteral("cameras")).toArray();
            else
                jsonError = QString::fromUtf8("相机型号 JSON 格式错误: %1").arg(parseError.errorString());
        } else if (!jsonPath.isEmpty()) {
            jsonError = QString::fromUtf8("无法打开相机型号 JSON: %1").arg(jsonPath);
        }

        std::vector<CameraDeviceInfo> devices;
        devices.reserve(nativeDevices.size());
        for (const auto& native : nativeDevices) {
            CameraDeviceInfo info;
            info.cvIndex = native.index;
            info.deviceId = QString::fromStdString(native.id);
            info.friendlyName = QString::fromStdString(native.name);
            info.vid = QString::fromStdString(native.vid).toUpper();
            info.pid = QString::fromStdString(native.pid).toUpper();
            info.modelName = QString::fromUtf8("未配置型号");
            // A camera profile is valid only when all three identifiers match:
            //   1) FriendlyName contains the configured model (JMC1S/JMC1M/JMC1L)
            //   2) USB VID matches
            //   3) USB PID matches
            // Each physical scanner model therefore has two JSON entries, one
            // for its A endpoint and one for its B endpoint (for example
            // 0BDA:300A and 0BDA:300B). Never fall back to model-only matching.
            const QString friendlyUpper = info.friendlyName.toUpper();
            auto normalizeUsbId = [](QString value) {
                value = value.trimmed().toUpper();
                if (value.startsWith(QStringLiteral("0X")))
                    value.remove(0, 2);
                return value.rightJustified(4, QLatin1Char('0')).right(4);
            };
            const QString deviceVid = normalizeUsbId(info.vid);
            const QString devicePid = normalizeUsbId(info.pid);
            QJsonObject matchedProfile;
            QString matchedModel;
            int matchedModelLength = -1;
            for (const auto& value : profiles) {
                const auto profile = value.toObject();
                const QString model = profile.value(QStringLiteral("model")).toString().trimmed();
                const QString profileVidRaw = profile.value(QStringLiteral("vid")).toString().trimmed();
                const QString profilePidRaw = profile.value(QStringLiteral("pid")).toString().trimmed();
                if (model.isEmpty() || profileVidRaw.isEmpty() || profilePidRaw.isEmpty())
                    continue;

                const QString modelUpper = model.toUpper();
                if (!friendlyUpper.contains(modelUpper))
                    continue;
                if (normalizeUsbId(profileVidRaw) != deviceVid ||
                    normalizeUsbId(profilePidRaw) != devicePid)
                    continue;

                // Prefer the longest model token if future model names share a
                // prefix. VID/PID must still match for the selected entry.
                if (modelUpper.size() <= matchedModelLength)
                    continue;
                matchedProfile = profile;
                matchedModel = model;
                matchedModelLength = modelUpper.size();
            }

            if (!matchedProfile.isEmpty()) {
                info.modelConfigured = true;
                info.modelName = matchedModel;
                info.width = matchedProfile.value(QStringLiteral("width")).toInt(info.width);
                info.height = matchedProfile.value(QStringLiteral("height")).toInt(info.height);
                info.fps = matchedProfile.value(QStringLiteral("fps")).toDouble(info.fps);
                info.fourcc = matchedProfile.value(QStringLiteral("fourcc")).toString(info.fourcc);
                // `rotate` deliberately follows cv::flip semantics. JSON null or
                // an omitted field means no transform.
                info.rotate = 2;
                const auto rotateValue = matchedProfile.value(QStringLiteral("rotate"));
                if (rotateValue.isDouble()) {
                    const int rotate = rotateValue.toInt(2);
                    if (rotate >= -1 && rotate <= 1)
                        info.rotate = rotate;
                }
                const auto exposure = matchedProfile.value(QStringLiteral("exposure")).toObject();
                info.exposure = {exposure.value(QStringLiteral("min")).toDouble(-13.0),
                                 exposure.value(QStringLiteral("max")).toDouble(0.0),
                                 exposure.value(QStringLiteral("step")).toDouble(1.0),
                                 exposure.value(QStringLiteral("default")).toDouble(-6.0)};
                const auto backlight = matchedProfile.value(QStringLiteral("backlight")).toObject();
                info.backlight = {backlight.value(QStringLiteral("min")).toDouble(0.0),
                                  backlight.value(QStringLiteral("max")).toDouble(25.0),
                                  backlight.value(QStringLiteral("step")).toDouble(1.0),
                                  backlight.value(QStringLiteral("default")).toDouble(25.0)};
            }
            devices.push_back(std::move(info));
        }
        const QString error = !jsonError.isEmpty() ? jsonError : QString::fromStdString(nativeError);
        if (!self) return;
        QMetaObject::invokeMethod(self, [self, devices = std::move(devices), error] {
            if (!self) return;
            const QString previousA = self->cameraACombo_ ? self->cameraACombo_->currentData().toString() : QString{};
            const QString previousB = self->cameraBCombo_ ? self->cameraBCombo_->currentData().toString() : QString{};
            self->cameraInfos_ = devices;
            self->cameraACombo_->clear(); self->cameraBCombo_->clear();
            int indexA = -1, indexB = -1;
            for (int i = 0; i < int(devices.size()); ++i) {
                const auto& d = devices[std::size_t(i)];
                self->cameraACombo_->addItem(d.displayText(), d.deviceId);
                self->cameraBCombo_->addItem(d.displayText(), d.deviceId);
                if (d.deviceId == previousA) indexA = i;
                if (d.deviceId == previousB) indexB = i;
            }
            if (indexA >= 0) self->cameraACombo_->setCurrentIndex(indexA);
            else if (!devices.empty()) self->cameraACombo_->setCurrentIndex(0);
            if (indexB >= 0) self->cameraBCombo_->setCurrentIndex(indexB);
            else if (devices.size() > 1) self->cameraBCombo_->setCurrentIndex(1);
            self->updateCameraSelectionUi();
            self->statusBar()->showMessage(error.isEmpty()
                ? QString::fromUtf8("已枚举 %1 个相机").arg(devices.size()) : error);
        }, Qt::QueuedConnection);
    }).detach();
}

void MainWindow::updateCameraSelectionUi() {
    auto apply = [this](QComboBox* combo, QLabel* label, QDoubleSpinBox* exposure, QSlider* slider, QLabel* valueLabel) {
        if (!combo || !label || !exposure || !slider || !valueLabel) return;
        const QString id = combo->currentData().toString();
        for (const auto& d : cameraInfos_) {
            if (d.deviceId != id) continue;
            const QString orientation = (d.rotate >= -1 && d.rotate <= 1)
                ? QStringLiteral("rotate=%1").arg(d.rotate)
                : QStringLiteral("rotate=null");
            label->setText(QStringLiteral("%1  VID=%2 PID=%3  %4x%5@%6  %7")
                           .arg(d.modelName, d.vid, d.pid)
                           .arg(d.width).arg(d.height).arg(d.fps, 0, 'f', 1)
                           .arg(orientation));
            const QSignalBlocker blocker(exposure);
            exposure->setRange(d.exposure.minimum, d.exposure.maximum);
            exposure->setSingleStep(d.exposure.step);
            exposure->setValue(d.exposure.defaultValue);
            const QSignalBlocker sliderBlock(slider);
            slider->setRange(int(std::lround(d.exposure.minimum)), int(std::lround(d.exposure.maximum)));
            slider->setSingleStep(std::max(1, int(std::lround(d.exposure.step))));
            slider->setValue(int(std::lround(exposure->value())));
            valueLabel->setText(QString::number(slider->value()));
            return;
        }
        label->setText(QStringLiteral("-"));
    };
    apply(cameraACombo_, cameraAModelLabel_, cameraAExposureSpin_, cameraAExposureSlider_, cameraAExposureValueLabel_);
    apply(cameraBCombo_, cameraBModelLabel_, cameraBExposureSpin_, cameraBExposureSlider_, cameraBExposureValueLabel_);

    auto applyBacklight = [this](QComboBox* combo, QDoubleSpinBox* spin, QSlider* slider) {
        if (!combo || !spin || !slider) return;
        const QString id = combo->currentData().toString();
        for (const auto& d : cameraInfos_) {
            if (d.deviceId != id) continue;
            const QSignalBlocker spinBlock(spin);
            const QSignalBlocker sliderBlock(slider);
            spin->setRange(d.backlight.minimum, d.backlight.maximum);
            spin->setSingleStep(d.backlight.step);
            slider->setRange(int(std::lround(d.backlight.minimum)), int(std::lround(d.backlight.maximum)));
            spin->setValue(d.backlight.defaultValue);
            slider->setValue(int(std::lround(d.backlight.defaultValue)));
            return;
        }
    };
    applyBacklight(cameraACombo_, cameraABacklightSpin_, cameraABacklightSlider_);
    applyBacklight(cameraBCombo_, cameraBBacklightSpin_, cameraBBacklightSlider_);
}

void MainWindow::applyScanSourceUi() {
    if (cameraPreviewLabel_ && scanSourceModeCombo_ &&
        scanSourceModeCombo_->currentData().toInt() != int(ScanSourceMode::Camera))
        cameraPreviewLabel_->hide();

    const bool cameraMode = scanSourceModeCombo_ &&
                            scanSourceModeCombo_->currentData().toInt() == int(ScanSourceMode::Camera);
    if (scanDataDirEdit_) scanDataDirEdit_->setEnabled(!cameraMode);
    if (cameraModelJsonEdit_) cameraModelJsonEdit_->setEnabled(cameraMode);
    if (cameraRefreshButton_) cameraRefreshButton_->setEnabled(cameraMode);
    if (cameraACombo_) cameraACombo_->setEnabled(cameraMode);
    if (cameraBCombo_) cameraBCombo_->setEnabled(cameraMode);
    if (cameraAExposureSpin_) cameraAExposureSpin_->setEnabled(cameraMode);
    if (cameraBExposureSpin_) cameraBExposureSpin_->setEnabled(cameraMode);
    if (cameraAExposureSlider_) cameraAExposureSlider_->setEnabled(cameraMode);
    if (cameraBExposureSlider_) cameraBExposureSlider_->setEnabled(cameraMode);
    if (liveOptimizationCheck_) liveOptimizationCheck_->setEnabled(true);
    if (cameraABacklightSlider_) cameraABacklightSlider_->setEnabled(cameraMode);
    if (cameraABacklightSpin_) cameraABacklightSpin_->setEnabled(cameraMode);
    if (cameraBBacklightSlider_) cameraBBacklightSlider_->setEnabled(cameraMode);
    if (cameraBBacklightSpin_) cameraBBacklightSpin_->setEnabled(cameraMode);
    if (cameraSyncToleranceSpin_) cameraSyncToleranceSpin_->setEnabled(cameraMode);
    if (recordRawDataCheck_) recordRawDataCheck_->setEnabled(cameraMode);
    if (rawDataDirEdit_) rawDataDirEdit_->setEnabled(cameraMode && recordRawDataCheck_ && recordRawDataCheck_->isChecked());
}

void MainWindow::applyScanState(JMEngine::ScanState state) {
    if (cameraPreviewLabel_ && state != JMEngine::ScanState::Scanning) {
        cameraPreviewLabel_->clear();
        cameraPreviewLabel_->hide();
    }

    // Remove live scan aids when scanning ends.
    if (view_ && state != JMEngine::ScanState::Scanning) {
        view_->clearScanCameraPose();
        if (state == JMEngine::ScanState::Stopping ||
            state == JMEngine::ScanState::ReadyForReconstruction ||
            state == JMEngine::ScanState::Reconstructing) {
            view_->finalizeCurrentScanFrame();
            if (state == JMEngine::ScanState::ReadyForReconstruction)
                view_->centerScanOrbitPivot();
        } else {
            view_->clearCurrentScanFrame();
        }
    }
    if (scanStateLabel_)
        scanStateLabel_->setText(QString::fromUtf8("状态：") + scanStateText(state));
    const bool idleLike = state == JMEngine::ScanState::Idle || state == JMEngine::ScanState::Error;
    const bool scanning = state == JMEngine::ScanState::Scanning;
    const bool ready = state == JMEngine::ScanState::ReadyForReconstruction;
    if (scanStartButton_) scanStartButton_->setEnabled(idleLike && !currentProjectPath_.isEmpty());
    if (scanStopButton_) scanStopButton_->setEnabled(scanning);
    if (scanOfflineButton_) scanOfflineButton_->setEnabled(ready);
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    if (scanTextureButton_) scanTextureButton_->setEnabled(ready);
#endif
    if (scanResetButton_)
        scanResetButton_->setEnabled(state != JMEngine::ScanState::Stopping &&
                                     state != JMEngine::ScanState::Reconstructing &&
                                     state != JMEngine::ScanState::Initializing);

    if (!currentProjectPath_.isEmpty() &&
        (state == JMEngine::ScanState::ReadyForReconstruction || state == JMEngine::ScanState::Idle ||
         state == JMEngine::ScanState::Error)) {
        if (projectManager_.openProject(fsPathFromQString(currentProjectPath_), currentProject_))
            updateProjectUi();
    }

    const bool editable = idleLike;
    if (scanSourceModeCombo_) scanSourceModeCombo_->setEnabled(editable);
    if (scanRegistrationModeCombo_) scanRegistrationModeCombo_->setEnabled(editable);
    if (scanCalibEdit_) scanCalibEdit_->setEnabled(editable);
    if (scanVocabEdit_) scanVocabEdit_->setEnabled(editable);
    if (scanMaxFramesSpin_) scanMaxFramesSpin_->setEnabled(editable);
    applyScanSourceUi();
    if (liveOptimizationCheck_) liveOptimizationCheck_->setEnabled(editable);
    if (!editable) {
        if (scanDataDirEdit_) scanDataDirEdit_->setEnabled(false);
        if (cameraModelJsonEdit_) cameraModelJsonEdit_->setEnabled(false);
        if (cameraRefreshButton_) cameraRefreshButton_->setEnabled(false);
        if (cameraACombo_) cameraACombo_->setEnabled(false);
        if (cameraBCombo_) cameraBCombo_->setEnabled(false);
        if (cameraSyncToleranceSpin_) cameraSyncToleranceSpin_->setEnabled(false);
        if (recordRawDataCheck_) recordRawDataCheck_->setEnabled(false);
        if (rawDataDirEdit_) rawDataDirEdit_->setEnabled(false);
        // Exposure remains editable during camera scanning; setter is posted to camera worker threads.
        const bool cameraScanning = scanning && scanSourceModeCombo_ &&
                                    scanSourceModeCombo_->currentData().toInt() == int(ScanSourceMode::Camera);
        if (cameraAExposureSpin_) cameraAExposureSpin_->setEnabled(cameraScanning);
        if (cameraBExposureSpin_) cameraBExposureSpin_->setEnabled(cameraScanning);
        if (cameraAExposureSlider_) cameraAExposureSlider_->setEnabled(cameraScanning);
        if (cameraBExposureSlider_) cameraBExposureSlider_->setEnabled(cameraScanning);
        if (cameraABacklightSlider_) cameraABacklightSlider_->setEnabled(cameraScanning);
        if (cameraABacklightSpin_) cameraABacklightSpin_->setEnabled(cameraScanning);
        if (cameraBBacklightSlider_) cameraBBacklightSlider_->setEnabled(cameraScanning);
        if (cameraBBacklightSpin_) cameraBBacklightSpin_->setEnabled(cameraScanning);
    }
}

void MainWindow::removeScanModelListEntry() {
    if (!modelList_ || !view_)
        return;
    const QString path = view_->scanPreviewPath();
    if (path.isEmpty())
        return;
    for (int i = modelList_->count() - 1; i >= 0; --i) {
        if (modelPathAt(i) == path) {
            delete modelList_->takeItem(i);
            break;
        }
    }
}

void MainWindow::openModels() {
    const QStringList files = QFileDialog::getOpenFileNames(
        this, QString::fromUtf8("打开模型 / 点云"), {},
        QString::fromUtf8("支持格式 (*.obj *.ply *.txt *.asc);;OBJ (*.obj);;PLY (*.ply);;ASC 点云 (*.asc);;TXT 点云 (*.txt)"));
    for (const auto& f : files)
        addModelPath(f);
}

void MainWindow::exportModel() {
    if (view_->activeModelPath().isEmpty()) {
        QMessageBox::information(this, QString::fromUtf8("导出模型"), QString::fromUtf8("当前没有激活模型"));
        return;
    }
    const QString path = QFileDialog::getSaveFileName(
        this, QString::fromUtf8("导出模型"), {}, QString::fromUtf8("PLY (*.ply);;ASC 点云 (*.asc);;Wavefront OBJ (*.obj);;STL (*.stl)"));
    if (path.isEmpty())
        return;
    if (exportAction_) exportAction_->setEnabled(false);
    if (saveAction_) saveAction_->setEnabled(false);
    const bool started = view_->exportActiveModelAsync(path, [this](bool ok, const QString& message) {
        if (exportAction_) exportAction_->setEnabled(true);
        if (saveAction_) saveAction_->setEnabled(true);
        if (!ok) QMessageBox::warning(this, QString::fromUtf8("导出失败"), message);
        else statusBar()->showMessage(message);
    });
    if (!started) {
        if (exportAction_) exportAction_->setEnabled(true);
        if (saveAction_) saveAction_->setEnabled(true);
    } else {
        statusBar()->showMessage(QString::fromUtf8("正在后台导出：") + QFileInfo(path).fileName());
    }
}

QString MainWindow::modelPathAt(int row) const {
    if (!modelList_ || row < 0 || row >= modelList_->count())
        return {};
    auto* item = modelList_->item(row);
    return item ? item->data(kPathRole).toString() : QString{};
}

void MainWindow::addModelPath(const QString& path) {
    const QString absolute = QFileInfo(path).absoluteFilePath();
    for (int i = 0; i < modelList_->count(); ++i) {
        if (modelPathAt(i) == absolute) {
            modelList_->setCurrentRow(i);
            return;
        }
    }

    auto* item = new QListWidgetItem(QFileInfo(absolute).fileName(), modelList_);
    item->setData(kPathRole, absolute);
    item->setToolTip(absolute);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    item->setCheckState(Qt::Checked);
    {
        const QSignalBlocker blocker(modelList_);
        modelList_->setCurrentItem(item);
    }
    statusBar()->showMessage(QString::fromUtf8("后台加载模型..."));
    view_->loadModelAsync(absolute);
}

void MainWindow::activateRow(int row) {
    const QString path = modelPathAt(row);
    if (!path.isEmpty())
        view_->activateModel(path);
}

void MainWindow::removeCurrentModel() {
    if (!modelList_)
        return;
    const int row = modelList_->currentRow();
    if (row < 0)
        return;
    const QString path = modelPathAt(row);
    view_->removeModel(path);
    delete modelList_->takeItem(row);
    if (modelList_->count() > 0)
        modelList_->setCurrentRow(qMin(row, modelList_->count() - 1));
}

void MainWindow::setCurrentModelColor() {
    if (!modelList_ || modelList_->currentRow() < 0)
        return;
    const QString path = modelPathAt(modelList_->currentRow());
    if (path.isEmpty())
        return;
    const QColor initial = view_->modelDisplayColor(path);
    const QColor color = QColorDialog::getColor(initial.isValid() ? initial : QColor(184, 184, 184), this,
                                                QString::fromUtf8("设置模型显示颜色"));
    if (!color.isValid())
        return;
    view_->setModelDisplayColor(path, color);
    if (auto* item = modelList_->currentItem()) {
        QPixmap swatch(14, 14);
        swatch.fill(color);
        item->setIcon(QIcon(swatch));
    }
}

void MainWindow::openProcessingDialog(const std::string& operationId) {
    auto operation = JMEngine::processing::createOperation(operationId);
    if (!operation) {
        QMessageBox::warning(this, QString::fromUtf8("处理"), QString::fromUtf8("未知算法"));
        return;
    }
    if (view_->processingBusy()) {
        QMessageBox::information(this, QString::fromUtf8("处理"), QString::fromUtf8("已有处理任务正在运行"));
        return;
    }

    auto* dialog = new ProcessingDialog(view_->processingDescriptor(operationId), this);
    dialog->setAttribute(Qt::WA_DeleteOnClose, true);
    QPointer<ProcessingDialog> safeDialog(dialog);
    connect(dialog, &QObject::destroyed, this, [this] {
        if (view_->processingBusy())
            view_->cancelProcessing();
    });

    connect(dialog->cancelButton(), &QPushButton::clicked, dialog, [this, safeDialog] {
        if (!safeDialog)
            return;
        if (safeDialog->running()) {
            view_->cancelProcessing();
            safeDialog->setResultSummary(QString::fromUtf8("正在取消，请等待当前算法检查取消点..."));
        } else {
            safeDialog->reject();
        }
    });

    connect(dialog->applyButton(), &QPushButton::clicked, dialog, [this, safeDialog, operationId] {
        if (!safeDialog || safeDialog->running())
            return;
        auto params = safeDialog->parameters();

        // Poisson 前置确认保持 O(1)：不扫描点云/法向；完整检查进入 worker 并尽量并行。
        const auto preflight = view_->processingPreflight(operationId, params);
        if (!preflight.warnings.empty()) {
            const QString details = QString::fromUtf8(JMEngine::processing::preflightSummary(preflight).c_str());
            if (!preflight.allowed) {
                QMessageBox::warning(this, QString::fromUtf8("处理预检未通过"), details);
                safeDialog->setResultSummary(QString::fromUtf8("未启动：模型预检未通过"));
                return;
            }
            const auto answer = QMessageBox::question(this, QString::fromUtf8("处理预检警告"),
                                                      details + QString::fromUtf8("\n\n仍然继续处理吗？"),
                                                      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
            if (answer != QMessageBox::Yes)
                return;
        }

        safeDialog->setRunning(true);
        safeDialog->setProgress(0.0f, QString::fromUtf8("提交到后台线程池..."));
        safeDialog->setResultSummary(QString::fromUtf8("处理中；OpenMP 默认保留 1 个逻辑核给 UI/系统"));

        const bool started = view_->startProcessingOperation(
            operationId, params,
            [safeDialog](float value, const QString& stage) {
                if (safeDialog)
                    safeDialog->setProgress(value, stage);
            },
            [safeDialog](bool ok, const QString& message) {
                if (!safeDialog)
                    return;
                safeDialog->setRunning(false);
                safeDialog->setResultSummary(message);
                if (ok) {
                    safeDialog->setProgress(1.0f, QString::fromUtf8("完成"));
                    safeDialog->accept();
                } else {
                    safeDialog->setProgress(0.0f, QString::fromUtf8("未应用"));
                }
            });
        if (!started && safeDialog)
            safeDialog->setRunning(false);
    });

    dialog->open();
}
