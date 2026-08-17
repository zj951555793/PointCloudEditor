#include "MainWindow.h"
#include "PointCloudWidget.h"

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QDockWidget>
#include <QFileDialog>
#include <QFileInfo>
#include <QKeySequence>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QToolBar>
#include <QVariant>

#include <algorithm>

namespace {
constexpr int kPathRole = Qt::UserRole + 1;
}

MainWindow::MainWindow(const QString& initialFile, QWidget* parent)
    : QMainWindow(parent)
{
    view_ = new PointCloudWidget({}, this);
    setCentralWidget(view_);

    createActions();
    createModelManager();
    createMenus();
    createToolbar();

    setWindowTitle(QString::fromUtf8("JMEngine - 多模型网格/点云编辑器"));
    resize(1380, 860);
    statusBar()->showMessage(QString::fromUtf8(
        "左键绕模型中心旋转 | Ctrl+左键执行当前选择工具 | Shift追加 / Alt减选 | Delete 删除高亮"));

    if (!initialFile.isEmpty()) addModelPath(initialFile);
}

void MainWindow::createActions() {
    openAction_ = new QAction(QString::fromUtf8("打开模型..."), this);
    openAction_->setShortcut(QKeySequence::Open);
    connect(openAction_, &QAction::triggered, this, [this] { openModel(); });

    saveAction_ = new QAction(QString::fromUtf8("保存当前模型编辑结果"), this);
    saveAction_->setShortcut(QKeySequence::Save);
    connect(saveAction_, &QAction::triggered, this, [this] { view_->saveModel(); });

    exitAction_ = new QAction(QString::fromUtf8("退出"), this);
    exitAction_->setShortcut(QKeySequence::Quit);
    connect(exitAction_, &QAction::triggered, qApp, &QApplication::quit);

    modeGroup_ = new QActionGroup(this);
    modeGroup_->setExclusive(true);
    auto makeMode = [this](const QString& text,
                           PointCloudWidget::InteractionMode mode,
                           const QKeySequence& key) {
        auto* action = new QAction(text, this);
        action->setCheckable(true);
        action->setShortcut(key);
        modeGroup_->addAction(action);
        connect(action, &QAction::triggered, this, [this, mode] { view_->setInteractionMode(mode); });
        return action;
    };

    viewAction_ = makeMode(QString::fromUtf8("浏览/旋转"), PointCloudWidget::InteractionMode::View, QKeySequence(Qt::Key_V));
    rectangleAction_ = makeMode(QString::fromUtf8("矩形选择（Ctrl+拖动）"), PointCloudWidget::InteractionMode::Rectangle, QKeySequence(Qt::Key_R));
    lassoAction_ = makeMode(QString::fromUtf8("套索选择（Ctrl+拖动）"), PointCloudWidget::InteractionMode::Lasso, QKeySequence(Qt::Key_L));
    circleAction_ = makeMode(QString::fromUtf8("圆形选择（Ctrl+拖动）"), PointCloudWidget::InteractionMode::Circle, QKeySequence(Qt::Key_C));
    brushAction_ = makeMode(QString::fromUtf8("画刷选择（Ctrl+拖动）"), PointCloudWidget::InteractionMode::BrushSelect, QKeySequence(Qt::Key_B));
    viewAction_->setChecked(true);

    // 选择深度模式：与矩形/套索/圆/画刷正交，所有工具都共用。
    depthGroup_ = new QActionGroup(this);
    depthGroup_->setExclusive(true);
    surfaceSelectAction_ = new QAction(QString::fromUtf8("表面选择（只选可见面）"), this);
    throughSelectAction_ = new QAction(QString::fromUtf8("穿透选择（选择前后全部几何）"), this);
    surfaceSelectAction_->setCheckable(true);
    throughSelectAction_->setCheckable(true);
    surfaceSelectAction_->setChecked(true);
    depthGroup_->addAction(surfaceSelectAction_);
    depthGroup_->addAction(throughSelectAction_);
    connect(surfaceSelectAction_, &QAction::triggered, this, [this] {
        view_->setSelectionDepthMode(PointCloudWidget::SelectionDepthMode::Surface);
    });
    connect(throughSelectAction_, &QAction::triggered, this, [this] {
        view_->setSelectionDepthMode(PointCloudWidget::SelectionDepthMode::Through);
    });

    deleteAction_ = new QAction(QString::fromUtf8("删除高亮选择"), this);
    deleteAction_->setShortcut(QKeySequence::Delete);
    connect(deleteAction_, &QAction::triggered, this, [this] { view_->deleteSelection(); });

    undoAction_ = new QAction(QString::fromUtf8("撤销"), this);
    undoAction_->setShortcut(QKeySequence::Undo);
    connect(undoAction_, &QAction::triggered, this, [this] { view_->undoEdit(); });

    redoAction_ = new QAction(QString::fromUtf8("重做"), this);
    redoAction_->setShortcut(QKeySequence::Redo);
    connect(redoAction_, &QAction::triggered, this, [this] { view_->redoEdit(); });

    clearSelectionAction_ = new QAction(QString::fromUtf8("清除选择"), this);
    clearSelectionAction_->setShortcut(QKeySequence(Qt::Key_Escape));
    connect(clearSelectionAction_, &QAction::triggered, this, [this] { view_->clearCurrentSelection(); });

    fitAction_ = new QAction(QString::fromUtf8("适配活动模型到视图"), this);
    fitAction_->setShortcut(QKeySequence(Qt::Key_F));
    connect(fitAction_, &QAction::triggered, this, [this] { view_->fitView(); });

    removeModelAction_ = new QAction(QString::fromUtf8("移除当前模型"), this);
    connect(removeModelAction_, &QAction::triggered, this, [this] { removeCurrentModel(); });
}

void MainWindow::createMenus() {
    QMenu* fileMenu = menuBar()->addMenu(QString::fromUtf8("文件(&F)"));
    fileMenu->addAction(openAction_);
    fileMenu->addAction(saveAction_);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAction_);

    QMenu* editMenu = menuBar()->addMenu(QString::fromUtf8("编辑(&E)"));
    editMenu->addAction(undoAction_);
    editMenu->addAction(redoAction_);
    editMenu->addSeparator();
    editMenu->addAction(deleteAction_);

    QMenu* selectMenu = menuBar()->addMenu(QString::fromUtf8("选择(&S)"));
    selectMenu->addAction(viewAction_);
    selectMenu->addSeparator();
    selectMenu->addAction(rectangleAction_);
    selectMenu->addAction(lassoAction_);
    selectMenu->addAction(circleAction_);
    selectMenu->addAction(brushAction_);
    selectMenu->addSeparator();

    // 用户要求放到菜单栏：表面/穿透是互斥选择深度模式。
    QMenu* depthMenu = selectMenu->addMenu(QString::fromUtf8("选择深度"));
    depthMenu->addAction(surfaceSelectAction_);
    depthMenu->addAction(throughSelectAction_);
    selectMenu->addSeparator();
    selectMenu->addAction(clearSelectionAction_);

    QMenu* viewMenu = menuBar()->addMenu(QString::fromUtf8("视图(&V)"));
    viewMenu->addAction(fitAction_);
    if (modelDock_) viewMenu->addAction(modelDock_->toggleViewAction());

    QMenu* modelMenu = menuBar()->addMenu(QString::fromUtf8("模型(&M)"));
    modelMenu->addAction(openAction_);
    modelMenu->addAction(removeModelAction_);

    QMenu* helpMenu = menuBar()->addMenu(QString::fromUtf8("帮助(&H)"));
    QAction* about = helpMenu->addAction(QString::fromUtf8("关于 JMEngine"));
    connect(about, &QAction::triggered, this, [this] {
        QMessageBox::information(this, QString::fromUtf8("关于"),
            QString::fromUtf8(
                "JMEngine 1.8.0\n"
                "Core 无 Qt 依赖；OBJ/PLY + OpenMP；多模型；表面/穿透选择。"));
    });
}

void MainWindow::createToolbar() {
    auto* bar = addToolBar(QString::fromUtf8("常用工具"));
    bar->setMovable(false);
    bar->addAction(openAction_);
    bar->addAction(saveAction_);
    bar->addSeparator();
    bar->addAction(viewAction_);
    bar->addAction(rectangleAction_);
    bar->addAction(lassoAction_);
    bar->addAction(circleAction_);
    bar->addAction(brushAction_);
    bar->addSeparator();
    bar->addAction(surfaceSelectAction_);
    bar->addAction(throughSelectAction_);
    bar->addSeparator();
    bar->addAction(deleteAction_);
    bar->addAction(undoAction_);
    bar->addAction(redoAction_);
    bar->addAction(clearSelectionAction_);
    bar->addAction(fitAction_);
}

void MainWindow::createModelManager() {
    modelDock_ = new QDockWidget(QString::fromUtf8("模型管理器"), this);
    modelList_ = new QListWidget(modelDock_);
    modelDock_->setWidget(modelList_);
    addDockWidget(Qt::RightDockWidgetArea, modelDock_);
    modelDock_->setMinimumWidth(260);

    // 单击哪一行，哪一个模型就成为活动模型，后续选择/删除/Undo 都只编辑它。
    connect(modelList_, &QListWidget::currentRowChanged, this,
            [this](int row) { activateModel(row); });

    // 每个模型前面的复选框控制显示/隐藏；隐藏不改变活动模型和编辑历史。
    connect(modelList_, &QListWidget::itemChanged, this,
            [this](QListWidgetItem* item) {
                if (!item) return;
                const QString path = item->data(kPathRole).toString();
                view_->setModelVisible(path, item->checkState() == Qt::Checked);
            });

    modelList_->setContextMenuPolicy(Qt::ActionsContextMenu);
    modelList_->addAction(removeModelAction_);
}

void MainWindow::openModel() {
    const QStringList files = QFileDialog::getOpenFileNames(
        this, QString::fromUtf8("打开一个或多个模型"), {},
        QString::fromUtf8("3D 模型 (*.obj *.ply);;OBJ 网格 (*.obj);;PLY 点云 (*.ply)"));
    for (const auto& fileName : files) addModelPath(fileName);
}

QString MainWindow::modelPathAt(int row) const {
    if (!modelList_ || row < 0 || row >= modelList_->count()) return {};
    auto* item = modelList_->item(row);
    return item ? item->data(kPathRole).toString() : QString{};
}

void MainWindow::addModelPath(const QString& fileName) {
    const QString absolute = QFileInfo(fileName).absoluteFilePath();
    for (int i = 0; i < modelList_->count(); ++i) {
        if (modelPathAt(i) == absolute) {
            modelList_->setCurrentRow(i);
            view_->activateModel(absolute);
            return;
        }
    }

    auto* item = new QListWidgetItem(QFileInfo(absolute).fileName(), modelList_);
    item->setData(kPathRole, absolute);
    item->setToolTip(absolute);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable | Qt::ItemIsEnabled);
    item->setCheckState(Qt::Checked);
    {
        // 避免 setCurrentItem() 立即触发 currentRowChanged，造成同一文件被提交两次加载任务。
        const QSignalBlocker blocker(modelList_);
        modelList_->setCurrentItem(item);
    }

    statusBar()->showMessage(QString::fromUtf8("后台加载模型；CPU 解析阶段启用 OpenMP..."));
    view_->loadModelAsync(absolute);
}

void MainWindow::activateModel(int row) {
    const QString path = modelPathAt(row);
    if (path.isEmpty()) return;
    if (!view_->activateModel(path)) {
        // 可能是模型还在后台加载；loadModelAsync 会在完成后自动设为活动模型。
        view_->loadModelAsync(path);
    }
}

void MainWindow::removeCurrentModel() {
    const int row = modelList_->currentRow();
    if (row < 0 || row >= modelList_->count()) return;
    const QString path = modelPathAt(row);

    view_->removeModel(path);
    delete modelList_->takeItem(row);

    if (modelList_->count() > 0) {
        const int nextRow = std::clamp(row, 0, modelList_->count() - 1);
        modelList_->setCurrentRow(nextRow);
        activateModel(nextRow);
    } else {
        view_->clearModel();
    }
}
