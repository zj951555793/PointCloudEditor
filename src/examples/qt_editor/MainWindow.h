#pragma once

#include <QMainWindow>
#include <string>
#include <memory>
#include <vector>
#include "ScanFlowController.h"

class QAction;
class QActionGroup;
class QDockWidget;
class QListWidget;
class QListWidgetItem;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QComboBox;
class QDoubleSpinBox;
class QSlider;
class QCheckBox;
class PointCloudWidget;
class QResizeEvent;

class MainWindow final : public QMainWindow {
  public:
    explicit MainWindow(const QString& initialFile = {}, QWidget* parent = nullptr);

  protected:
    void resizeEvent(QResizeEvent* event) override;

  private:
    void createActions();
    void createMenus();
    void createModelManager();
    void createScanControl();
    void applyScanState(ScanFlowController::State state);
    void applyScanSourceUi();
    void updateCameraSelectionUi();
    void removeScanModelListEntry();
    void updateCameraPreviewGeometry();
    ScanConfig scanConfigFromUi() const;
    void openModels();
    void exportModel();
    void addModelPath(const QString& path);
    void activateRow(int row);
    void removeCurrentModel();
    void setCurrentModelColor();
    void openProcessingDialog(const std::string& operationId);
    QString modelPathAt(int row) const;

  private:
    PointCloudWidget* view_{nullptr};
    QListWidget* modelList_{nullptr};
    QDockWidget* modelDock_{nullptr};
    QDockWidget* scanDock_{nullptr};
    QComboBox* scanSourceModeCombo_{nullptr};
    QLineEdit* scanDataDirEdit_{nullptr};
    QLineEdit* scanCalibEdit_{nullptr};
    QLineEdit* scanVocabEdit_{nullptr};
    QSpinBox* scanMaxFramesSpin_{nullptr};
    QLineEdit* cameraModelJsonEdit_{nullptr};
    QPushButton* cameraRefreshButton_{nullptr};
    QComboBox* cameraACombo_{nullptr};
    QComboBox* cameraBCombo_{nullptr};
    QLabel* cameraAModelLabel_{nullptr};
    QLabel* cameraBModelLabel_{nullptr};
    QDoubleSpinBox* cameraAExposureSpin_{nullptr};
    QDoubleSpinBox* cameraBExposureSpin_{nullptr};
    QSlider* cameraAExposureSlider_{nullptr};
    QSlider* cameraBExposureSlider_{nullptr};
    QLabel* cameraAExposureValueLabel_{nullptr};
    QLabel* cameraBExposureValueLabel_{nullptr};
    QCheckBox* liveOptimizationCheck_{nullptr};
    QSlider* cameraABacklightSlider_{nullptr};
    QDoubleSpinBox* cameraABacklightSpin_{nullptr};
    QSlider* cameraBBacklightSlider_{nullptr};
    QDoubleSpinBox* cameraBBacklightSpin_{nullptr};
    QDoubleSpinBox* cameraSyncToleranceSpin_{nullptr};
    QLabel* cameraPreviewLabel_{nullptr};
    std::vector<CameraDeviceInfo> cameraInfos_;
    QLabel* scanStateLabel_{nullptr};
    QLabel* scanRenderFpsLabel_{nullptr};
    QPushButton* scanStartButton_{nullptr};
    QPushButton* scanStopButton_{nullptr};
    QPushButton* scanOfflineButton_{nullptr};
#ifdef PCEDITOR_HAS_TEXTURE_MAPPING
    QPushButton* scanTextureButton_{nullptr};
    QLabel* scanTextureFramesLabel_{nullptr};
    bool scanTextureFramesReady_{false};
#endif
    QPushButton* scanResetButton_{nullptr};
    std::unique_ptr<ScanFlowController> scanController_;

    QAction* openAction_{nullptr};
    QAction* saveAction_{nullptr};
    QAction* exportAction_{nullptr};
    QAction* touchEditAction_{nullptr};
    QAction* objectMoveAction_{nullptr};
    QAction* rectangleAction_{nullptr};
    QAction* lassoAction_{nullptr};
    QAction* circleAction_{nullptr};
    QAction* brushAction_{nullptr};
    QAction* surfaceAction_{nullptr};
    QAction* throughAction_{nullptr};
    QAction* deleteAction_{nullptr};
    QAction* clearAction_{nullptr};
    QAction* keepAction_{nullptr};
    QAction* invertAction_{nullptr};
    QAction* compactAction_{nullptr};
    QAction* fitBasePlaneAction_{nullptr};
    QAction* applyBasePlaneCutAction_{nullptr};
    QAction* cancelBasePlaneCutAction_{nullptr};
    QAction* measureDistanceAction_{nullptr};
    QAction* measureAngleAction_{nullptr};
    QAction* measureAreaAction_{nullptr};
    QAction* measureVolumeAction_{nullptr};
    QAction* alignThreePointAction_{nullptr};
    QAction* autoAlignAction_{nullptr};
    QAction* cancelToolAction_{nullptr};
    QAction* undoAction_{nullptr};
    QAction* redoAction_{nullptr};
    QAction* fitAction_{nullptr};
    QAction* removeModelAction_{nullptr};
    QAction* modelColorAction_{nullptr};
    QAction* displayPointsAction_{nullptr};
    QAction* displaySolidAction_{nullptr};
    QAction* displayWireAction_{nullptr};
    QAction* displaySolidWireAction_{nullptr};

    QAction* gpuPickingAction_{nullptr};
    QAction* cpuPickingAction_{nullptr};
    QActionGroup* toolGroup_{nullptr};
    QActionGroup* depthGroup_{nullptr};
    QActionGroup* pickingGroup_{nullptr};
    QActionGroup* displayGroup_{nullptr};
};
