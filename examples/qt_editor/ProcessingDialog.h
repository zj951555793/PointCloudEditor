#pragma once
#include <QDialog>
#include <JMEngine/processing/Processing.h>
#include <unordered_map>

class QLabel;
class QProgressBar;
class QPushButton;
class QFormLayout;
class QWidget;

class ProcessingDialog final : public QDialog {
  public:
    explicit ProcessingDialog(const JMEngine::processing::OperationDescriptor& descriptor, QWidget* parent = nullptr);

    JMEngine::processing::ParameterMap parameters() const;
    QPushButton* applyButton() const noexcept {
        return applyButton_;
    }
    QPushButton* cancelButton() const noexcept {
        return cancelButton_;
    }

    void setRunning(bool running);
    void setProgress(float progress, const QString& stage);
    void setResultSummary(const QString& text);
    bool running() const noexcept {
        return running_;
    }

  private:
    JMEngine::processing::OperationDescriptor descriptor_;
    std::unordered_map<std::string, QWidget*> editors_;
    QLabel* summaryLabel_{nullptr};
    QLabel* stageLabel_{nullptr};
    QProgressBar* progressBar_{nullptr};
    QPushButton* applyButton_{nullptr};
    QPushButton* cancelButton_{nullptr};
    bool running_{false};
};
