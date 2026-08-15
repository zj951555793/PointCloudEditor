#pragma once

#include <QMainWindow>
#include <QStringList>

class QAction;
class QActionGroup;
class QDockWidget;
class QListWidget;
class QListWidgetItem;
class PointCloudWidget;

// Qt 主窗口只管理 UI。Core 库仍然完全无 Qt 依赖。
class MainWindow final : public QMainWindow {
public:
    explicit MainWindow(const QString& initialFile = {}, QWidget* parent = nullptr);

private:
    void createActions();
    void createMenus();
    void createToolbar();
    void createModelManager();

    void openModel();
    void addModelPath(const QString& fileName);
    void activateModel(int row);
    void removeCurrentModel();
    QString modelPathAt(int row) const;

private:
    PointCloudWidget* view_{nullptr};
    QDockWidget* modelDock_{nullptr};
    QListWidget* modelList_{nullptr};

    QActionGroup* modeGroup_{nullptr};
    QActionGroup* depthGroup_{nullptr};

    QAction* openAction_{nullptr};
    QAction* saveAction_{nullptr};
    QAction* exitAction_{nullptr};
    QAction* deleteAction_{nullptr};
    QAction* undoAction_{nullptr};
    QAction* redoAction_{nullptr};
    QAction* clearSelectionAction_{nullptr};
    QAction* fitAction_{nullptr};
    QAction* removeModelAction_{nullptr};

    QAction* viewAction_{nullptr};
    QAction* rectangleAction_{nullptr};
    QAction* lassoAction_{nullptr};
    QAction* circleAction_{nullptr};
    QAction* brushAction_{nullptr};

    QAction* surfaceSelectAction_{nullptr};
    QAction* throughSelectAction_{nullptr};
};
