#include "GlesMainWindow.h"
#include "GlesPointCloudWidget.h"

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QDockWidget>
#include <QFileDialog>
#include <QFileInfo>
#include <QKeySequence>
#include <QListWidget>
#include <QMenuBar>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QToolBar>

namespace {
constexpr int kPathRole = Qt::UserRole + 1;
}

GlesMainWindow::GlesMainWindow(const QString& initialFile, QWidget* parent)
    : QMainWindow(parent)
{
    view_ = new GlesPointCloudWidget(this);
    setCentralWidget(view_);

    createActions();
    createModelManager();
    createMenus();
    createToolbar();

    setWindowTitle(QString::fromUtf8("PointCloudEditor RK3588 - Qt OpenGL ES 3.1"));
    resize(1280, 800);
    statusBar()->showMessage(QString::fromUtf8(
        "单指旋转 | 双指平移/缩放 | 开启编辑后单指选择 | Delete删除高亮"));

    if (!initialFile.isEmpty()) addModelPath(initialFile);
}

void GlesMainWindow::createActions()
{
    openAction_ = new QAction(QString::fromUtf8("打开模型"), this);
    openAction_->setShortcut(QKeySequence::Open);
    connect(openAction_, &QAction::triggered, this, [this] { openModels(); });

    saveAction_ = new QAction(QString::fromUtf8("保存"), this);
    saveAction_->setShortcut(QKeySequence::Save);
    connect(saveAction_, &QAction::triggered, this, [this] { view_->saveActiveModel(); });

    touchEditAction_ = new QAction(QString::fromUtf8("编辑模式"), this);
    touchEditAction_->setCheckable(true);
    connect(touchEditAction_, &QAction::toggled, this, [this](bool on) {
        view_->setTouchEditMode(on);
        statusBar()->showMessage(on
            ? QString::fromUtf8("编辑模式：单指执行当前选择工具；双指仍可平移/缩放")
            : QString::fromUtf8("浏览模式：单指旋转；双指平移/缩放"));
    });

    toolGroup_ = new QActionGroup(this);
    toolGroup_->setExclusive(true);
    auto makeTool = [this](const QString& text, GlesPointCloudWidget::InteractionMode mode) {
        auto* a = new QAction(text, this);
        a->setCheckable(true);
        toolGroup_->addAction(a);
        connect(a, &QAction::triggered, this, [this, mode] { view_->setInteractionMode(mode); });
        return a;
    };
    rectangleAction_ = makeTool(QString::fromUtf8("矩形"), GlesPointCloudWidget::InteractionMode::Rectangle);
    lassoAction_ = makeTool(QString::fromUtf8("套索"), GlesPointCloudWidget::InteractionMode::Lasso);
    circleAction_ = makeTool(QString::fromUtf8("圆形"), GlesPointCloudWidget::InteractionMode::Circle);
    brushAction_ = makeTool(QString::fromUtf8("画刷"), GlesPointCloudWidget::InteractionMode::Brush);
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
    connect(surfaceAction_, &QAction::triggered, this, [this] {
        view_->setSelectionDepthMode(GlesPointCloudWidget::SelectionDepthMode::Surface);
    });
    connect(throughAction_, &QAction::triggered, this, [this] {
        view_->setSelectionDepthMode(GlesPointCloudWidget::SelectionDepthMode::Through);
    });

    deleteAction_ = new QAction(QString::fromUtf8("删除高亮"), this);
    deleteAction_->setShortcut(QKeySequence::Delete);
    connect(deleteAction_, &QAction::triggered, this, [this] { view_->deleteSelection(); });

    clearAction_ = new QAction(QString::fromUtf8("清除选择"), this);
    connect(clearAction_, &QAction::triggered, this, [this] { view_->clearSelection(); });

    undoAction_ = new QAction(QString::fromUtf8("撤销"), this);
    undoAction_->setShortcut(QKeySequence::Undo);
    connect(undoAction_, &QAction::triggered, this, [this] { view_->undoEdit(); });

    redoAction_ = new QAction(QString::fromUtf8("重做"), this);
    redoAction_->setShortcut(QKeySequence::Redo);
    connect(redoAction_, &QAction::triggered, this, [this] { view_->redoEdit(); });

    fitAction_ = new QAction(QString::fromUtf8("适配视图"), this);
    connect(fitAction_, &QAction::triggered, this, [this] { view_->fitView(); });

    removeModelAction_ = new QAction(QString::fromUtf8("移除当前模型"), this);
    connect(removeModelAction_, &QAction::triggered, this, [this] { removeCurrentModel(); });
}

void GlesMainWindow::createMenus()
{
    auto* file = menuBar()->addMenu(QString::fromUtf8("文件"));
    file->addAction(openAction_);
    file->addAction(saveAction_);
    file->addSeparator();
    file->addAction(QString::fromUtf8("退出"), qApp, &QApplication::quit);

    auto* edit = menuBar()->addMenu(QString::fromUtf8("编辑"));
    edit->addAction(touchEditAction_);
    edit->addSeparator();
    edit->addAction(deleteAction_);
    edit->addAction(clearAction_);
    edit->addSeparator();
    edit->addAction(undoAction_);
    edit->addAction(redoAction_);

    auto* select = menuBar()->addMenu(QString::fromUtf8("选择"));
    select->addAction(rectangleAction_);
    select->addAction(lassoAction_);
    select->addAction(circleAction_);
    select->addAction(brushAction_);
    select->addSeparator();
    auto* depth = select->addMenu(QString::fromUtf8("选择深度"));
    depth->addAction(surfaceAction_);
    depth->addAction(throughAction_);

    auto* view = menuBar()->addMenu(QString::fromUtf8("视图"));
    view->addAction(fitAction_);
    view->addAction(modelDock_->toggleViewAction());

    auto* model = menuBar()->addMenu(QString::fromUtf8("模型"));
    model->addAction(openAction_);
    model->addAction(removeModelAction_);

    auto* help = menuBar()->addMenu(QString::fromUtf8("帮助"));
    auto* about = help->addAction(QString::fromUtf8("关于"));
    connect(about, &QAction::triggered, this, [this] {
        QMessageBox::information(this, QString::fromUtf8("关于"),
            QString::fromUtf8("PointCloudEditor RK3588\nQt6 + QOpenGLWidget + OpenGL ES 3.1\n支持触摸、OBJ/PLY、GPU 表面选择。"));
    });
}

void GlesMainWindow::createToolbar()
{
    auto* bar = addToolBar(QString::fromUtf8("触摸工具栏"));
    bar->setMovable(false);
    bar->setIconSize(QSize(32, 32));
    // 3588 触摸屏：扩大按钮命中区域，避免桌面尺寸的 QAction 难以点击。
    bar->setStyleSheet(QString::fromUtf8("QToolButton { min-width: 72px; min-height: 48px; font-size: 16px; padding: 4px; }"));
    bar->addAction(openAction_);
    bar->addAction(saveAction_);
    bar->addSeparator();
    bar->addAction(touchEditAction_);
    bar->addAction(rectangleAction_);
    bar->addAction(lassoAction_);
    bar->addAction(circleAction_);
    bar->addAction(brushAction_);
    bar->addSeparator();
    bar->addAction(surfaceAction_);
    bar->addAction(throughAction_);
    bar->addSeparator();
    bar->addAction(deleteAction_);
    bar->addAction(clearAction_);
    bar->addAction(undoAction_);
    bar->addAction(redoAction_);
    bar->addAction(fitAction_);
}

void GlesMainWindow::createModelManager()
{
    modelDock_ = new QDockWidget(QString::fromUtf8("模型管理器"), this);
    modelList_ = new QListWidget(modelDock_);
    modelDock_->setWidget(modelList_);
    modelDock_->setMinimumWidth(240);
    addDockWidget(Qt::RightDockWidgetArea, modelDock_);

    connect(modelList_, &QListWidget::currentRowChanged, this, [this](int row) { activateRow(row); });
    connect(modelList_, &QListWidget::itemChanged, this, [this](QListWidgetItem* item) {
        if (!item) return;
        view_->setModelVisible(item->data(kPathRole).toString(), item->checkState() == Qt::Checked);
    });

    modelList_->setContextMenuPolicy(Qt::ActionsContextMenu);
    modelList_->addAction(removeModelAction_);
}

void GlesMainWindow::openModels()
{
    const QStringList files = QFileDialog::getOpenFileNames(
        this, QString::fromUtf8("打开 OBJ / PLY"), {}, QString::fromUtf8("3D模型 (*.obj *.ply)"));
    for (const auto& f : files) addModelPath(f);
}

QString GlesMainWindow::modelPathAt(int row) const
{
    if (!modelList_ || row < 0 || row >= modelList_->count()) return {};
    auto* item = modelList_->item(row);
    return item ? item->data(kPathRole).toString() : QString{};
}

void GlesMainWindow::addModelPath(const QString& path)
{
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

void GlesMainWindow::activateRow(int row)
{
    const QString path = modelPathAt(row);
    if (!path.isEmpty()) view_->activateModel(path);
}

void GlesMainWindow::removeCurrentModel()
{
    if (!modelList_) return;
    const int row = modelList_->currentRow();
    if (row < 0) return;
    const QString path = modelPathAt(row);
    view_->removeModel(path);
    delete modelList_->takeItem(row);
    if (modelList_->count() > 0) modelList_->setCurrentRow(qMin(row, modelList_->count() - 1));
}
