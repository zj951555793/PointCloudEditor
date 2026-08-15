#pragma once

#include <QMainWindow>

class QAction;
class QActionGroup;
class QDockWidget;
class QListWidget;
class QListWidgetItem;
class GlesPointCloudWidget;

class GlesMainWindow final : public QMainWindow
{
public:
    explicit GlesMainWindow(const QString& initialFile = {}, QWidget* parent = nullptr);

private:
    void createActions();
    void createMenus();
    void createToolbar();
    void createModelManager();
    void openModels();
    void addModelPath(const QString& path);
    void activateRow(int row);
    void removeCurrentModel();
    QString modelPathAt(int row) const;

private:
    GlesPointCloudWidget* view_{nullptr};
    QListWidget* modelList_{nullptr};
    QDockWidget* modelDock_{nullptr};

    QAction* openAction_{nullptr};
    QAction* saveAction_{nullptr};
    QAction* touchEditAction_{nullptr};
    QAction* rectangleAction_{nullptr};
    QAction* lassoAction_{nullptr};
    QAction* circleAction_{nullptr};
    QAction* brushAction_{nullptr};
    QAction* surfaceAction_{nullptr};
    QAction* throughAction_{nullptr};
    QAction* deleteAction_{nullptr};
    QAction* clearAction_{nullptr};
    QAction* undoAction_{nullptr};
    QAction* redoAction_{nullptr};
    QAction* fitAction_{nullptr};
    QAction* removeModelAction_{nullptr};
    QActionGroup* toolGroup_{nullptr};
    QActionGroup* depthGroup_{nullptr};
};
