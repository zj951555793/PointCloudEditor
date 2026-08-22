#pragma once

#include <QMainWindow>
#include <QString>
#include <string>
#include <memory>
#include <thread>
#include <vector>
#include <JMEngine/JMScanner.h>

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
    ~MainWindow() override;

  protected:
    void resizeEvent(QResizeEvent* event) override;

  private:
    enum class ScanSourceMode { Virtual = 0, Camera = 1 };
    struct CameraRange { double minimum{-13.0}, maximum{0.0}, step{1.0}, defaultValue{-6.0}; };
    struct CameraDeviceInfo {
        int cvIndex{-1};
        QString deviceId, friendlyName, vid, pid, modelName, fourcc{QStringLiteral("MJPG")};
        int width{1920}, height{1200};
        double fps{10.0};
        CameraRange exposure;
        CameraRange backlight{5.0, 25.0, 1.0, 25.0};
        QString displayText() const;
    };
    struct ScanUiConfig {
        ScanSourceMode sourceMode{ScanSourceMode::Virtual};
        JMEngine::ScanConfig engine;
        JMEngine::DualCameraConfig cameras;
        QString dataDir, cameraModelJsonPath, cameraADeviceId, cameraBDeviceId;
        QString rawDataDir;
        int previewPointLimit{20000000};
        bool liveOptimizationEnabled{true};
        bool recordRawData{false};
    };
    void createActions();
    void createMenus();
    void createModelManager();
    void createScanControl();
    void applyScanState(JMEngine::ScanState state);
    void applyScanSourceUi();
    void refreshCameras();
    void updateCameraSelectionUi();
    void removeScanModelListEntry();
    void updateCameraPreviewGeometry();
    ScanUiConfig scanConfigFromUi() const;
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
    QComboBox* scanRegistrationModeCombo_{nullptr};
    QLineEdit* scanDataDirEdit_{nullptr};
    QLineEdit* scanCalibEdit_{nullptr};
    QLineEdit* scanVocabEdit_{nullptr};
    QSpinBox* scanMaxFramesSpin_{nullptr};
    QLineEdit* cameraModelJsonEdit_{nullptr};
    QCheckBox* recordRawDataCheck_{nullptr};
    QLineEdit* rawDataDirEdit_{nullptr};
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
    JMEngine::ScanMarkerFrame latestMarkerFrame_;
    std::vector<CameraDeviceInfo> cameraInfos_;
    QLabel* scanStateLabel_{nullptr};
    QLabel* scanRenderFpsLabel_{nullptr};
    QPushButton* scanStartButton_{nullptr};
    QPushButton* scanStopButton_{nullptr};
    QPushButton* scanOfflineButton_{nullptr};
#ifdef JMENGINE_HAS_TEXTURE_MAPPING
    QPushButton* scanTextureButton_{nullptr};
    QLabel* scanTextureFramesLabel_{nullptr};
    bool scanTextureFramesReady_{false};
#endif
    QPushButton* scanResetButton_{nullptr};
    std::unique_ptr<JMEngine::JMScanner> scanner_;
    std::thread reconstructionThread_;
    ScanUiConfig activeScanConfig_;
    int lastScanVisualFrameId_{-1};

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
