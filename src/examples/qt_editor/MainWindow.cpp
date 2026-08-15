#include "MainWindow.h"
#include "PointCloudWidget.h"
#include "ProcessingDialog.h"

#include <pceditor/processing/Processing.h>

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
#include <QFormLayout>
#include <QGridLayout>
#include <QToolButton>
#include <QHBoxLayout>
#include <QSettings>
#include <QMenuBar>
#include <QMenu>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QVBoxLayout>
#include <QWidget>
#include <QPointer>
#include <QPixmap>
#include <QPushButton>
#include <QResizeEvent>
#include <QTimer>

namespace {
constexpr int kPathRole = Qt::UserRole + 1;
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

    setWindowTitle(QString::fromUtf8("PointCloudEditor - Unified Qt Editor"));
    resize(1280, 800);
    statusBar()->showMessage(
        QString::fromUtf8("左键旋转 | Alt+左键移动当前对象 | Ctrl+左键选择 | 右/中键平移 | Delete删除高亮"));

    if (!initialFile.isEmpty())
        addModelPath(initialFile);
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
    rectangleAction_->setChecked(true);

    depthGroup_ = new QActionGroup(this);
    depthGroup_->setExclusive(true);
    surfaceAction_ = new QAction(QString::fromUtf8("表面选择"), this);
    throughAction_ = new QAction(QString::fromUtf8("穿透选择"), this);
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
    auto* meshSmooth = meshProcessMenu->addMenu(QString::fromUtf8("平滑"));
    auto* meshRepair = meshProcessMenu->addMenu(QString::fromUtf8("修复"));
    auto* meshReconstruct = meshProcessMenu->addMenu(QString::fromUtf8("重建"));
    addProcessAction(meshProcessMenu, QString::fromUtf8("网格清理..."), "mesh_cleanup");
    addProcessAction(meshSmooth, QString::fromUtf8("Laplacian..."), "laplacian");
    addProcessAction(meshSmooth, QString::fromUtf8("Taubin..."), "taubin");
    addProcessAction(meshProcessMenu, QString::fromUtf8("QEM 简化..."), "qem_decimate");
    addProcessAction(meshRepair, QString::fromUtf8("检测/填充孔洞..."), "hole_fill");
    addProcessAction(meshReconstruct, QString::fromUtf8("工业泊松重建..."), "poisson_octree");
#ifdef PCEDITOR_HAS_TEXTURE_MAPPING
    auto* textureMenu = meshProcessMenu->addMenu(QString::fromUtf8("纹理映射"));
    auto runTextureMapping = [this](pceditor::texture::Backend backend) {
        statusBar()->showMessage(QString::fromUtf8("正在后台执行纹理映射..."));
        view_->startTextureMappingAsync(backend, [this](bool ok, const QString& message) {
            statusBar()->showMessage(message, 10000);
            if (!ok) QMessageBox::warning(this, QString::fromUtf8("纹理映射"), message);
        });
    };
    auto* textureAuto = textureMenu->addAction(QString::fromUtf8("自动（CUDA 优先）"));
    auto* textureCpu = textureMenu->addAction(QString::fromUtf8("CPU"));
    auto* textureCuda = textureMenu->addAction(QString::fromUtf8("CUDA"));
    connect(textureAuto, &QAction::triggered, this, [runTextureMapping] { runTextureMapping(pceditor::texture::Backend::Auto); });
    connect(textureCpu, &QAction::triggered, this, [runTextureMapping] { runTextureMapping(pceditor::texture::Backend::Cpu); });
    connect(textureCuda, &QAction::triggered, this, [runTextureMapping] { runTextureMapping(pceditor::texture::Backend::Cuda); });
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
        view_->analyzeActiveModelAsync([this](bool ok, const pceditor::processing::ModelDiagnostics& diagnostics,
                                              const QString& error) {
            if (!ok) {
                statusBar()->showMessage(error);
                if (!error.isEmpty())
                    QMessageBox::warning(this, QString::fromUtf8("模型诊断"), error);
                return;
            }
            statusBar()->showMessage(QString::fromUtf8("模型诊断完成"));
            QMessageBox::information(this, QString::fromUtf8("模型诊断"),
                                     QString::fromUtf8(pceditor::processing::diagnosticsSummary(diagnostics).c_str()));
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
            QString::fromUtf8("PointCloudEditor\n统一 Qt 编辑器 + 可切换渲染后端\nDesktop OpenGL 2.1 / OpenGL ES "
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

void MainWindow::createScanControl() {
    scanController_ = std::make_unique<ScanFlowController>(this);
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
    scanSourceModeCombo_->setMinimumWidth(120);

    scanDataDirEdit_ = new QLineEdit(panel);
    scanCalibEdit_ = new QLineEdit(panel);
    scanVocabEdit_ = new QLineEdit(panel);
    cameraModelJsonEdit_ = new QLineEdit(panel);
    scanMaxFramesSpin_ = new QSpinBox(panel);
    scanMaxFramesSpin_->setRange(1, 100000);
    scanMaxFramesSpin_->setValue(2000);

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
        spin->setRange(0.0, 10.0);
        spin->setSingleStep(1.0);
        spin->setValue(10.0);
        spin->setFixedWidth(54);
    }
    for (auto* slider : {cameraABacklightSlider_, cameraBBacklightSlider_}) {
        slider->setRange(0, 10);
        slider->setValue(10);
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
    // Backlight remains configured/applied from camera_models.json but is intentionally
    // removed from the compact operator UI.
    cameraABacklightSlider_->hide();
    cameraABacklightSpin_->hide();
    cameraBBacklightSlider_->hide();
    cameraBBacklightSpin_->hide();

    scanStateLabel_ = new QLabel(QString::fromUtf8("状态：空闲"), panel);
    scanStateLabel_->setMinimumWidth(115);
    scanStateLabel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    scanRenderFpsLabel_ = new QLabel(QString::fromUtf8("渲染 FPS：0.0"), panel);
    scanRenderFpsLabel_->setMinimumWidth(105);
    scanRenderFpsLabel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

    scanStartButton_ = new QPushButton(QString::fromUtf8("开始"), panel);
    scanStopButton_ = new QPushButton(QString::fromUtf8("结束"), panel);
    scanOfflineButton_ = new QPushButton(QString::fromUtf8("离线重建"), panel);
#ifdef PCEDITOR_HAS_TEXTURE_MAPPING
    scanTextureButton_ = new QPushButton(QString::fromUtf8("纹理映射"), panel);
    scanTextureButton_->setToolTip(QString::fromUtf8(
        "扫描完成并执行离线重建后使用。若当前结果还是点云，会先自动进行工业泊松重建，再执行纹理映射。"));
    scanTextureFramesLabel_ = new QLabel(QString::fromUtf8("纹理帧：0"), panel);
    scanTextureFramesLabel_->setMinimumWidth(82);
#endif
    scanResetButton_ = new QPushButton(QString::fromUtf8("重置"), panel);
    for (auto* button : {scanStartButton_, scanStopButton_, scanOfflineButton_, scanResetButton_})
        button->setMinimumWidth(76);
#ifdef PCEDITOR_HAS_TEXTURE_MAPPING
    scanTextureButton_->setMinimumWidth(86);
#endif

    // First row: scan mode, both devices, state and the complete workflow actions.
    quickGrid->addWidget(new QLabel(QString::fromUtf8("模式"), panel), 0, 0);
    quickGrid->addWidget(scanSourceModeCombo_, 0, 1);
    quickGrid->addWidget(cameraRefreshButton_, 0, 2);
    quickGrid->addWidget(new QLabel(QString::fromUtf8("A码图"), panel), 0, 3);
    quickGrid->addWidget(cameraACombo_, 0, 4);
    quickGrid->addWidget(new QLabel(QString::fromUtf8("B彩色"), panel), 0, 5);
    quickGrid->addWidget(cameraBCombo_, 0, 6);
    quickGrid->addWidget(scanStateLabel_, 0, 7);
    quickGrid->addWidget(scanStartButton_, 0, 8);
    quickGrid->addWidget(scanStopButton_, 0, 9);
    quickGrid->addWidget(scanOfflineButton_, 0, 10);
#ifdef PCEDITOR_HAS_TEXTURE_MAPPING
    quickGrid->addWidget(scanTextureButton_, 0, 11);
    quickGrid->addWidget(scanResetButton_, 0, 12);
#else
    quickGrid->addWidget(scanResetButton_, 0, 11);
#endif

    // Second row: only operator-facing controls. Backlight is applied from JSON and hidden.
    quickGrid->addWidget(new QLabel(QString::fromUtf8("A曝光"), panel), 1, 0);
    quickGrid->addWidget(cameraAExposureSlider_, 1, 1, 1, 3);
    quickGrid->addWidget(cameraAExposureValueLabel_, 1, 4);
    quickGrid->addWidget(new QLabel(QString::fromUtf8("B曝光"), panel), 1, 5);
    quickGrid->addWidget(cameraBExposureSlider_, 1, 6, 1, 3);
    quickGrid->addWidget(cameraBExposureValueLabel_, 1, 9);
    quickGrid->addWidget(liveOptimizationCheck_, 1, 10, 1, 2);

    // Runtime rendering cadence: this is the actual paintGL rate, not camera/SLAM FPS.
    quickGrid->addWidget(scanRenderFpsLabel_, 2, 0, 1, 2);
#ifdef PCEDITOR_HAS_TEXTURE_MAPPING
    quickGrid->addWidget(scanTextureFramesLabel_, 2, 2, 1, 2);
#endif
    auto* renderFpsTimer = new QTimer(panel);
    renderFpsTimer->setInterval(500);
    connect(renderFpsTimer, &QTimer::timeout, panel, [this] {
        if (scanRenderFpsLabel_ && view_)
            scanRenderFpsLabel_->setText(QString::fromUtf8("渲染 FPS：%1").arg(view_->renderFps(), 0, 'f', 1));
    });
    renderFpsTimer->start();

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

    cameraSyncToleranceSpin_ = new QDoubleSpinBox(panel);
    cameraSyncToleranceSpin_->setRange(0.1, 100.0);
    cameraSyncToleranceSpin_->setDecimals(1);
    cameraSyncToleranceSpin_->setSingleStep(1.0);
    cameraSyncToleranceSpin_->setValue(50.0);
    advancedGrid->addWidget(new QLabel(QString::fromUtf8("最大帧"), advancedWidget), 2, 0);
    advancedGrid->addWidget(scanMaxFramesSpin_, 2, 1);
    advancedGrid->addWidget(new QLabel(QString::fromUtf8("同步(ms)"), advancedWidget), 2, 2);
    advancedGrid->addWidget(cameraSyncToleranceSpin_, 2, 3);
    advancedGrid->addWidget(new QLabel(QString::fromUtf8("A型号"), advancedWidget), 3, 0);
    advancedGrid->addWidget(cameraAModelLabel_, 3, 1);
    advancedGrid->addWidget(new QLabel(QString::fromUtf8("B型号"), advancedWidget), 3, 2);
    advancedGrid->addWidget(cameraBModelLabel_, 3, 3);
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

    QSettings settings;
    scanSourceModeCombo_->setCurrentIndex(settings.value(QStringLiteral("scan/sourceMode"), 0).toInt());
    scanDataDirEdit_->setText(settings.value(QStringLiteral("scan/virtualDataDir")).toString());
    scanCalibEdit_->clear();
    QString defaultVocabulary;
    {
        QDir appDir(QCoreApplication::applicationDirPath());
        const QStringList vocabFiles = appDir.entryList(QStringList() << QStringLiteral("*.yml.gz"), QDir::Files, QDir::Name);
        if (!vocabFiles.isEmpty()) defaultVocabulary = appDir.filePath(vocabFiles.first());
    }
    scanVocabEdit_->setText(settings.value(QStringLiteral("scan/vocabularyPath"), defaultVocabulary).toString());
    scanMaxFramesSpin_->setValue(settings.value(QStringLiteral("scan/maxFrames"), 2000).toInt());
    if (liveOptimizationCheck_)
        liveOptimizationCheck_->setChecked(settings.value(QStringLiteral("scan/liveOptimizationEnabled"), true).toBool());
    const QString defaultJson = QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("config/camera_models.json"));
    cameraModelJsonEdit_->setText(settings.value(QStringLiteral("scan/cameraModelJson"), defaultJson).toString());
    {
        QString jsonError;
        const auto savedScan = CameraDeviceManager::loadScanSettings(cameraModelJsonEdit_->text().trimmed(), &jsonError);
        if (!savedScan.lastCalibrationPath.isEmpty())
            scanCalibEdit_->setText(savedScan.lastCalibrationPath);
    }
    cameraAExposureSpin_->setValue(settings.value(QStringLiteral("scan/cameraAExposure"), -6.0).toDouble());
    cameraBExposureSpin_->setValue(settings.value(QStringLiteral("scan/cameraBExposure"), -6.0).toDouble());
    const double savedBacklightA = settings.value(QStringLiteral("scan/cameraABacklight"), 10.0).toDouble();
    const double savedBacklightB = settings.value(QStringLiteral("scan/cameraBBacklight"), 10.0).toDouble();
    cameraABacklightSpin_->setValue(savedBacklightA);
    cameraABacklightSlider_->setValue(int(std::lround(savedBacklightA)));
    cameraBBacklightSpin_->setValue(savedBacklightB);
    cameraBBacklightSlider_->setValue(int(std::lround(savedBacklightB)));
    cameraSyncToleranceSpin_->setValue(settings.value(QStringLiteral("scan/cameraSyncToleranceMs"), 50.0).toDouble());
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
                const auto savedScan = CameraDeviceManager::loadScanSettings(cameraModelJsonEdit_->text().trimmed(), &jsonError);
                if (!savedScan.lastCalibrationPath.isEmpty())
                    scanCalibEdit_->setText(savedScan.lastCalibrationPath);
            }
            ScanConfig cfg = scanConfigFromUi();
            scanController_->setConfig(cfg);
            scanController_->refreshCameras();
        }
    });
    connect(cameraModelJsonEdit_, &QLineEdit::editingFinished, this, [this] {
        if (!scanSourceModeCombo_ || !scanCalibEdit_ || !cameraModelJsonEdit_) return;
        if (scanSourceModeCombo_->currentData().toInt() != int(ScanSourceMode::Camera)) return;
        QString jsonError;
        const auto savedScan = CameraDeviceManager::loadScanSettings(cameraModelJsonEdit_->text().trimmed(), &jsonError);
        if (!savedScan.lastCalibrationPath.isEmpty()) scanCalibEdit_->setText(savedScan.lastCalibrationPath);
    });

    connect(cameraRefreshButton_, &QPushButton::clicked, this, [this] {
        ScanConfig cfg = scanConfigFromUi();
        scanController_->setConfig(cfg);
        statusBar()->showMessage(QString::fromUtf8("正在后台枚举相机..."));
        scanController_->refreshCameras();
    });
    connect(cameraACombo_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] { updateCameraSelectionUi(); });
    connect(cameraBCombo_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] { updateCameraSelectionUi(); });
    connect(cameraAExposureSlider_, &QSlider::valueChanged, this, [this](int value) {
        if (cameraAExposureValueLabel_) cameraAExposureValueLabel_->setText(QString::number(value));
        if (cameraAExposureSpin_) { QSignalBlocker b(cameraAExposureSpin_); cameraAExposureSpin_->setValue(double(value)); }
        if (scanController_) scanController_->setCameraExposure(ScanCameraRole::CameraA, double(value));
    });
    connect(cameraBExposureSlider_, &QSlider::valueChanged, this, [this](int value) {
        if (cameraBExposureValueLabel_) cameraBExposureValueLabel_->setText(QString::number(value));
        if (cameraBExposureSpin_) { QSignalBlocker b(cameraBExposureSpin_); cameraBExposureSpin_->setValue(double(value)); }
        if (scanController_) scanController_->setCameraExposure(ScanCameraRole::CameraB, double(value));
    });
    connect(cameraABacklightSlider_, &QSlider::valueChanged, this, [this](int value) {
        if (!cameraABacklightSpin_) return;
        const QSignalBlocker blocker(cameraABacklightSpin_);
        cameraABacklightSpin_->setValue(double(value));
        if (scanController_) scanController_->setCameraBacklight(ScanCameraRole::CameraA, double(value));
    });
    connect(cameraABacklightSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (cameraABacklightSlider_) {
            const QSignalBlocker blocker(cameraABacklightSlider_);
            cameraABacklightSlider_->setValue(int(std::lround(value)));
        }
        if (scanController_) scanController_->setCameraBacklight(ScanCameraRole::CameraA, value);
    });

    connect(cameraBBacklightSlider_, &QSlider::valueChanged, this, [this](int value) {
        if (!cameraBBacklightSpin_) return;
        const QSignalBlocker blocker(cameraBBacklightSpin_);
        cameraBBacklightSpin_->setValue(double(value));
        if (scanController_) scanController_->setCameraBacklight(ScanCameraRole::CameraB, double(value));
    });
    connect(cameraBBacklightSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (cameraBBacklightSlider_) {
            const QSignalBlocker blocker(cameraBBacklightSlider_);
            cameraBBacklightSlider_->setValue(int(std::lround(value)));
        }
        if (scanController_) scanController_->setCameraBacklight(ScanCameraRole::CameraB, value);
    });

    connect(scanStartButton_, &QPushButton::clicked, this, [this] {
        const ScanConfig cfg = scanConfigFromUi();
        QSettings settings;
        settings.setValue(QStringLiteral("scan/sourceMode"), int(cfg.sourceMode));
        settings.setValue(QStringLiteral("scan/virtualDataDir"), cfg.dataDir);
        settings.setValue(QStringLiteral("scan/vocabularyPath"), cfg.vocabularyPath);
        settings.setValue(QStringLiteral("scan/maxFrames"), cfg.maxFrames);
        settings.setValue(QStringLiteral("scan/liveOptimizationEnabled"), cfg.liveOptimizationEnabled);
        settings.setValue(QStringLiteral("scan/cameraModelJson"), cfg.cameraModelJsonPath);
        settings.setValue(QStringLiteral("scan/cameraADeviceId"), cfg.cameraADeviceId);
        settings.setValue(QStringLiteral("scan/cameraBDeviceId"), cfg.cameraBDeviceId);
        settings.setValue(QStringLiteral("scan/cameraAExposure"), cfg.cameraAExposure);
        settings.setValue(QStringLiteral("scan/cameraBExposure"), cfg.cameraBExposure);
        settings.setValue(QStringLiteral("scan/cameraABacklight"), cfg.cameraABacklight);
        settings.setValue(QStringLiteral("scan/cameraBBacklight"), cfg.cameraBBacklight);
        settings.setValue(QStringLiteral("scan/cameraSyncToleranceMs"), cfg.cameraSyncToleranceMs);
        // Production-camera calibration is persisted in camera_models.json, not QSettings.
        // Virtual mode always derives <dataDir>/calib.txt and must never overwrite this path.
        if (cfg.sourceMode == ScanSourceMode::Camera && !cfg.cameraModelJsonPath.isEmpty()) {
            CameraScanSettings persisted;
            persisted.lastCalibrationPath = cfg.calibrationPath;
            QString saveError;
            if (!CameraDeviceManager::saveScanSettings(cfg.cameraModelJsonPath, persisted, &saveError) && !saveError.isEmpty())
                statusBar()->showMessage(saveError);
        }
        removeScanModelListEntry();
        view_->clearScanPreview();
#ifdef PCEDITOR_HAS_TEXTURE_MAPPING
        view_->setTextureFrames(nullptr);
        scanTextureFramesReady_ = false;
        if (scanTextureFramesLabel_) scanTextureFramesLabel_->setText(QString::fromUtf8("纹理帧：0"));
        if (scanTextureButton_) scanTextureButton_->setEnabled(false);
#endif
        view_->beginScanPreview(static_cast<std::size_t>(cfg.previewPointLimit));
        scanController_->setConfig(cfg);
        scanController_->startScan();
    });
    connect(scanStopButton_, &QPushButton::clicked, this, [this] { scanController_->stopScan(); });
    connect(scanOfflineButton_, &QPushButton::clicked, this, [this] { scanController_->offlineReconstruct(); });
#ifdef PCEDITOR_HAS_TEXTURE_MAPPING
    connect(scanTextureButton_, &QPushButton::clicked, this, [this] {
        if (!view_) return;
        scanTextureButton_->setEnabled(false);
        statusBar()->showMessage(QString::fromUtf8("正在准备扫描网格并执行纹理映射..."));
        const bool started = view_->startScanTextureMappingAsync(
            pceditor::texture::Backend::Auto,
            [this](float progress, const QString& stage) {
                statusBar()->showMessage(QString::fromUtf8("纹理流程 %1%：%2")
                                             .arg(int(std::lround(progress * 100.0f)))
                                             .arg(stage));
            },
            [this](bool ok, const QString& message) {
                const auto state = scanController_ ? scanController_->state() : ScanFlowController::State::Idle;
                if (scanTextureButton_)
                    scanTextureButton_->setEnabled(scanTextureFramesReady_ &&
                        state == ScanFlowController::State::ReadyForReconstruction);
                statusBar()->showMessage(message, 12000);
                if (!ok) QMessageBox::warning(this, QString::fromUtf8("扫描纹理映射"), message);
            });
        if (!started) {
            const auto state = scanController_ ? scanController_->state() : ScanFlowController::State::Idle;
            scanTextureButton_->setEnabled(scanTextureFramesReady_ &&
                state == ScanFlowController::State::ReadyForReconstruction);
        }
    });
#endif
    connect(scanResetButton_, &QPushButton::clicked, this, [this] {
        scanController_->reset();
        removeScanModelListEntry();
        view_->clearScanPreview();
#ifdef PCEDITOR_HAS_TEXTURE_MAPPING
        view_->setTextureFrames(nullptr);
        scanTextureFramesReady_ = false;
        if (scanTextureFramesLabel_) scanTextureFramesLabel_->setText(QString::fromUtf8("纹理帧：0"));
        if (scanTextureButton_) scanTextureButton_->setEnabled(false);
#endif
    });

    scanController_->setStateCallback([this](ScanFlowController::State state) { applyScanState(state); });
    scanController_->setMessageCallback([this](const QString& message) { statusBar()->showMessage(message); });
    scanController_->setPreviewCallback([this](ScanFlowController::PointChunkPtr chunk) {
        const ScanConfig cfg = scanController_->config();
        view_->appendScanPreview(chunk, static_cast<std::size_t>(cfg.previewPointLimit));
    });
    scanController_->setLiveFrameCallback([this](ScanFlowController::PointChunkPtr localPoints,
                                                   const std::array<float,16>& pose, int frameId) {
        if (view_ && localPoints) view_->appendScanLocalFrame(frameId, localPoints, pose);
    });
    scanController_->setLivePoseUpdatesCallback([this](std::shared_ptr<std::vector<LiveFramePoseUpdate>> updates) {
        if (!view_ || !updates) return;
        auto converted = std::make_shared<std::vector<PointCloudWidget::LiveFramePoseUpdate>>();
        converted->reserve(updates->size());
        for (const auto& u : *updates) converted->push_back({u.frameId, u.pose});
        view_->updateScanFramePoses(converted);
    });
    scanController_->setCurrentFrameCallback([this](ScanFlowController::PointChunkPtr chunk, bool trackingOk, int) {
        view_->setCurrentScanFrame(chunk, trackingOk);
    });
    scanController_->setPoseCallback([this](const ScanPoseState& pose) {
        PointCloudWidget::ScanCameraViewPose viewPose;
        viewPose.position = {pose.position[0], pose.position[1], pose.position[2]};
        viewPose.right = {pose.right[0], pose.right[1], pose.right[2]};
        viewPose.up = {pose.up[0], pose.up[1], pose.up[2]};
        viewPose.forward = {pose.forward[0], pose.forward[1], pose.forward[2]};
        viewPose.trackingOk = pose.trackingOk;
        viewPose.frameId = pose.frameId;
        view_->updateScanCameraPose(viewPose);
    });
    scanController_->setReconstructionCallback([this](ScanFlowController::CloudPtr cloud) { view_->replaceScanPreview(cloud); });
#ifdef PCEDITOR_HAS_TEXTURE_MAPPING
    scanController_->setTextureFramesCallback([this](ScanFlowController::TextureFramesPtr frames) {
        const std::size_t count = frames ? frames->size() : 0u;
        scanTextureFramesReady_ = count > 0u;
        if (scanTextureFramesLabel_)
            scanTextureFramesLabel_->setText(QString::fromUtf8("纹理帧：%1").arg(static_cast<qulonglong>(count)));
        if (view_) view_->setTextureFrames(std::move(frames));
        if (scanTextureButton_ && scanController_)
            scanTextureButton_->setEnabled(scanTextureFramesReady_ &&
                scanController_->state() == ScanFlowController::State::ReadyForReconstruction);
    });
#endif
    scanController_->setCameraListCallback([this](const QVector<CameraDeviceInfo>& list, const QString& error) {
        QSettings cameraSettings;
        const QString previousA = cameraSettings.value(QStringLiteral("scan/cameraADeviceId")).toString();
        const QString previousB = cameraSettings.value(QStringLiteral("scan/cameraBDeviceId")).toString();
        cameraInfos_.assign(list.begin(), list.end());
        cameraACombo_->clear();
        cameraBCombo_->clear();
        int indexA = -1, indexB = -1;
        for (int i = 0; i < list.size(); ++i) {
            const auto& d = list[i];
            cameraACombo_->addItem(d.displayText(), d.deviceId);
            cameraBCombo_->addItem(d.displayText(), d.deviceId);
            if (d.deviceId == previousA) indexA = i;
            if (d.deviceId == previousB) indexB = i;
        }
        if (indexA >= 0) cameraACombo_->setCurrentIndex(indexA);
        else if (!list.isEmpty()) cameraACombo_->setCurrentIndex(0);
        if (indexB >= 0) cameraBCombo_->setCurrentIndex(indexB);
        else if (list.size() > 1) cameraBCombo_->setCurrentIndex(1);
        updateCameraSelectionUi();
        if (!error.isEmpty()) statusBar()->showMessage(error);
        else statusBar()->showMessage(QString::fromUtf8("已枚举 %1 个相机").arg(list.size()));
    });
    scanController_->setCameraPreviewCallback([this](const QImage& image) {
        if (!cameraPreviewLabel_ || image.isNull() || !scanController_) return;
        const bool cameraScanning = scanController_->state() == ScanFlowController::State::Scanning &&
                                    scanSourceModeCombo_ &&
                                    scanSourceModeCombo_->currentData().toInt() == int(ScanSourceMode::Camera);
        if (!cameraScanning) {
            cameraPreviewLabel_->clear();
            cameraPreviewLabel_->hide();
            return;
        }
        const QSize box = cameraPreviewLabel_->size();
        cameraPreviewLabel_->setPixmap(QPixmap::fromImage(image).scaled(box, Qt::KeepAspectRatio, Qt::FastTransformation));
        cameraPreviewLabel_->show();
        cameraPreviewLabel_->raise();
        updateCameraPreviewGeometry();
    });

    applyScanSourceUi();
    applyScanState(ScanFlowController::State::Idle);
    if (scanSourceModeCombo_->currentData().toInt() == int(ScanSourceMode::Camera)) {
        scanController_->setConfig(scanConfigFromUi());
        scanController_->refreshCameras();
    }
}

ScanConfig MainWindow::scanConfigFromUi() const {
    ScanConfig cfg;
    cfg.sourceMode = scanSourceModeCombo_ && scanSourceModeCombo_->currentData().toInt() == int(ScanSourceMode::Camera)
                         ? ScanSourceMode::Camera : ScanSourceMode::Virtual;
    cfg.dataDir = scanDataDirEdit_ ? scanDataDirEdit_->text().trimmed() : QString{};
    cfg.calibrationPath = scanCalibEdit_ ? scanCalibEdit_->text().trimmed() : QString{};
    cfg.vocabularyPath = scanVocabEdit_ ? scanVocabEdit_->text().trimmed() : QString{};
    cfg.maxFrames = scanMaxFramesSpin_ ? scanMaxFramesSpin_->value() : 2000;
    cfg.maxInflightFrames = 2;
    // Live preview is bounded but must represent the whole scan. PointCloudWidget compacts
    // old preview samples when this budget is reached instead of dropping all later frames.
    cfg.previewPointsPerFrame = 15000;
    cfg.previewPointLimit = 20000000;
    cfg.offlineVoxel = 3.0;
    cfg.offlineIterations = 30; // Preserve the user's current ScanFlowController parameter.
    cfg.liveOptimizationEnabled = liveOptimizationCheck_ ? liveOptimizationCheck_->isChecked() : true;
    cfg.cameraModelJsonPath = cameraModelJsonEdit_ ? cameraModelJsonEdit_->text().trimmed() : QString{};
    cfg.cameraADeviceId = cameraACombo_ ? cameraACombo_->currentData().toString() : QString{};
    cfg.cameraBDeviceId = cameraBCombo_ ? cameraBCombo_->currentData().toString() : QString{};
    cfg.cameraAExposure = cameraAExposureSpin_ ? cameraAExposureSpin_->value() : -6.0;
    cfg.cameraBExposure = cameraBExposureSpin_ ? cameraBExposureSpin_->value() : -6.0;
    // Backlight UI is hidden.  The first camera open must use the model default from
    // camera_models.json, not a stale QSettings value (for example 10 while JSON says 25).
    auto modelBacklightDefault = [this](QComboBox* combo, double fallback) {
        if (!combo) return fallback;
        const QString id = combo->currentData().toString();
        for (const auto& d : cameraInfos_) {
            if (d.deviceId == id) return d.backlight.defaultValue;
        }
        return fallback;
    };
    cfg.cameraABacklight = modelBacklightDefault(cameraACombo_, 10.0);
    cfg.cameraBBacklight = modelBacklightDefault(cameraBCombo_, 10.0);
    cfg.cameraSyncToleranceMs = cameraSyncToleranceSpin_ ? cameraSyncToleranceSpin_->value() : 50.0;
    cfg.cameraQueueDepth = 3;
    return cfg;
}

void MainWindow::updateCameraSelectionUi() {
    auto apply = [this](QComboBox* combo, QLabel* label, QDoubleSpinBox* exposure, QSlider* slider, QLabel* valueLabel) {
        if (!combo || !label || !exposure || !slider || !valueLabel) return;
        const QString id = combo->currentData().toString();
        for (const auto& d : cameraInfos_) {
            if (d.deviceId != id) continue;
            label->setText(QStringLiteral("%1  VID=%2 PID=%3  %4x%5@%6")
                           .arg(d.modelName, d.vid, d.pid).arg(d.width).arg(d.height).arg(d.fps, 0, 'f', 1));
            const QSignalBlocker blocker(exposure);
            exposure->setRange(d.exposure.minimum, d.exposure.maximum);
            exposure->setSingleStep(d.exposure.step);
            if (exposure->value() < d.exposure.minimum || exposure->value() > d.exposure.maximum)
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
            // Backlight is not exposed in the normal UI.  When a model is selected,
            // synchronize the hidden control state with the VID/PID JSON default.
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
}

void MainWindow::applyScanState(ScanFlowController::State state) {
    if (cameraPreviewLabel_ && state != ScanFlowController::State::Scanning) {
        cameraPreviewLabel_->clear();
        cameraPreviewLabel_->hide();
    }

    // The 3D camera/frustum is only a live scanning aid.  As soon as scanning leaves the
    // Scanning state (Stopping / ReadyForReconstruction / Error / Idle), remove it and stop
    // forcing the 3D observer to follow the last SLAM pose.  The completed cloud remains.
    if (view_ && state != ScanFlowController::State::Scanning) {
        view_->clearScanCameraPose();
        // On a normal scan finish, preserve the last valid current frame by promoting it to
        // the RGB history before removing the green/yellow temporary layer. Reset clears the
        // whole preview immediately in its button handler, so there is nothing to preserve.
        if (state == ScanFlowController::State::Stopping ||
            state == ScanFlowController::State::ReadyForReconstruction ||
            state == ScanFlowController::State::Reconstructing)
            view_->finalizeCurrentScanFrame();
        else
            view_->clearCurrentScanFrame();
    }
    if (scanStateLabel_)
        scanStateLabel_->setText(QString::fromUtf8("状态：") + ScanFlowController::stateText(state));
    const bool idleLike = state == ScanFlowController::State::Idle || state == ScanFlowController::State::Error;
    const bool scanning = state == ScanFlowController::State::Scanning;
    const bool ready = state == ScanFlowController::State::ReadyForReconstruction;
    if (scanStartButton_) scanStartButton_->setEnabled(idleLike);
    if (scanStopButton_) scanStopButton_->setEnabled(scanning);
    if (scanOfflineButton_) scanOfflineButton_->setEnabled(ready);
#ifdef PCEDITOR_HAS_TEXTURE_MAPPING
    if (scanTextureButton_) scanTextureButton_->setEnabled(ready && scanTextureFramesReady_);
#endif
    if (scanResetButton_)
        scanResetButton_->setEnabled(state != ScanFlowController::State::Stopping &&
                                     state != ScanFlowController::State::Reconstructing &&
                                     state != ScanFlowController::State::Initializing);

    const bool editable = idleLike;
    if (scanSourceModeCombo_) scanSourceModeCombo_->setEnabled(editable);
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
    auto operation = pceditor::processing::createOperation(operationId);
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
            const QString details = QString::fromUtf8(pceditor::processing::preflightSummary(preflight).c_str());
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
