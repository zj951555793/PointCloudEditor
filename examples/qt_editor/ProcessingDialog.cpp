#include "ProcessingDialog.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>
#include <algorithm>

ProcessingDialog::ProcessingDialog(const pceditor::processing::OperationDescriptor& descriptor, QWidget* parent)
    : QDialog(parent), descriptor_(descriptor) {
    setWindowTitle(QString::fromUtf8(descriptor_.title.c_str()));
    setModal(true);
    resize(430, 320);

    auto* root = new QVBoxLayout(this);
    auto* title = new QLabel(QStringLiteral("<b>%1</b>").arg(QString::fromUtf8(descriptor_.title.c_str())), this);
    root->addWidget(title);

    auto* form = new QFormLayout;
    for (const auto& spec : descriptor_.parameters) {
        QWidget* editor = nullptr;
        if (spec.kind == pceditor::processing::ParameterKind::Integer) {
            auto* spin = new QSpinBox(this);
            spin->setRange(static_cast<int>(spec.minValue), static_cast<int>(spec.maxValue));
            spin->setSingleStep(std::max(1, static_cast<int>(spec.step)));
            spin->setValue(static_cast<int>(spec.defaultValue));
            if (!spec.unit.empty())
                spin->setSuffix(QStringLiteral(" ") + QString::fromUtf8(spec.unit.c_str()));
            editor = spin;
        } else if (spec.kind == pceditor::processing::ParameterKind::Real) {
            auto* spin = new QDoubleSpinBox(this);
            spin->setDecimals(5);
            spin->setRange(spec.minValue, spec.maxValue);
            spin->setSingleStep(spec.step);
            spin->setValue(spec.defaultValue);
            if (!spec.unit.empty())
                spin->setSuffix(QStringLiteral(" ") + QString::fromUtf8(spec.unit.c_str()));
            editor = spin;
        } else if (spec.kind == pceditor::processing::ParameterKind::Boolean) {
            auto* check = new QCheckBox(this);
            check->setChecked(spec.defaultValue != 0.0);
            editor = check;
        } else {
            auto* combo = new QComboBox(this);
            for (const auto& c : spec.choices)
                combo->addItem(QString::fromUtf8(c.c_str()));
            combo->setCurrentIndex(static_cast<int>(spec.defaultValue));
            editor = combo;
        }
        editors_[spec.key] = editor;
        form->addRow(QString::fromUtf8(spec.label.c_str()), editor);
    }
    root->addLayout(form);

    summaryLabel_ = new QLabel(QString::fromUtf8("等待处理"), this);
    summaryLabel_->setWordWrap(true);
    root->addWidget(summaryLabel_);

    stageLabel_ = new QLabel(QString::fromUtf8("就绪"), this);
    root->addWidget(stageLabel_);
    progressBar_ = new QProgressBar(this);
    progressBar_->setRange(0, 1000);
    progressBar_->setValue(0);
    root->addWidget(progressBar_);

    auto* buttons = new QDialogButtonBox(this);
    cancelButton_ = buttons->addButton(QString::fromUtf8("取消"), QDialogButtonBox::RejectRole);
    applyButton_ = buttons->addButton(QString::fromUtf8("应用"), QDialogButtonBox::ApplyRole);
    root->addWidget(buttons);
}

pceditor::processing::ParameterMap ProcessingDialog::parameters() const {
    pceditor::processing::ParameterMap out;
    for (const auto& spec : descriptor_.parameters) {
        auto it = editors_.find(spec.key);
        if (it == editors_.end())
            continue;
        QWidget* w = it->second;
        switch (spec.kind) {
        case pceditor::processing::ParameterKind::Integer:
            out[spec.key] = static_cast<std::int64_t>(static_cast<QSpinBox*>(w)->value());
            break;
        case pceditor::processing::ParameterKind::Real:
            out[spec.key] = static_cast<QDoubleSpinBox*>(w)->value();
            break;
        case pceditor::processing::ParameterKind::Boolean:
            out[spec.key] = static_cast<QCheckBox*>(w)->isChecked();
            break;
        case pceditor::processing::ParameterKind::Choice:
            out[spec.key] = static_cast<QComboBox*>(w)->currentText().toStdString();
            break;
        }
    }
    return out;
}

void ProcessingDialog::setRunning(bool running) {
    running_ = running;
    for (auto& item : editors_)
        item.second->setEnabled(!running);
    applyButton_->setEnabled(!running);
    cancelButton_->setText(running ? QString::fromUtf8("停止") : QString::fromUtf8("取消"));
}

void ProcessingDialog::setProgress(float progress, const QString& stage) {
    progressBar_->setValue(std::clamp(static_cast<int>(progress * 1000.0f), 0, 1000));
    stageLabel_->setText(stage);
}

void ProcessingDialog::setResultSummary(const QString& text) {
    summaryLabel_->setText(text);
}
