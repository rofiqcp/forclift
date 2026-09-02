// Extracted reusable components for BAB IV acquisition layout refactor.
// Single data source, multiple renderers. No duplicate ROS subscriptions, timers,
// or recorders inside these widgets.
//
// This header carries ONLY declarations (for AUTOMOC / moc generation). All method
// bodies live in experiment_components.cpp, which is #included as a translation unit
// from agv_gui.cpp AFTER ReportPlotWidget / YamlStore / ExperimentParameterField are
// fully defined. Keeping bodies out of this header avoids "incomplete type" errors in
// moc_experiment_components.cpp (the moc-generated TU only sees forward declarations
// and the catalog, not the local widget/store classes).
#pragma once
#include <QDialog>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QMap>
#include <QPointer>
#include <QPushButton>
#include <QScrollArea>
#include <QString>
#include <QTimer>
#include <QVector>
#include <QWidget>
#include <QVBoxLayout>
#include <memory>
#include "../agv_experiment_catalog.hpp"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

// Forward declarations for types defined in other translation-unit modules.
class ReportPlotWidget;
class ExperimentWorkspacePage;
class TelemetryStore;
class YamlStore;
class ExperimentParameterField;

// ============================================================
// GraphCard — reusable wrapper around ReportPlotWidget.
// Header: [title]  [⛶ maximize]
// Owns NO data: data is pushed from ExperimentWorkspacePage via setData().
// ============================================================
class GraphCard : public QWidget {
  Q_OBJECT
  public:
  explicit GraphCard(const QString &title, QWidget *parent = nullptr);
  void setTitle(const QString &t);
  // Pushed from owner. Single acquisition pipeline, single data model.
  void setData(const QString &title, const QString &xLabel, const QString &yLabel,
  const QMap<QString, QVector<QPointF>> &series, bool connectPoints = true,
  const QString &emptyMessage = QString());
  void clear();
  ReportPlotWidget *plot() const;
  signals:
  void maximizeRequested();
  private:
  QString title_;
  QLabel *titleLabel_;
  QPushButton *maximize_;
  ReportPlotWidget *plot_;
};

// ============================================================
// GraphFullscreenDialog — single extra renderer, same data model.
// NO QTimer, NO ROS subscriber, NO recorder. Refreshed via updateData()
// called by ExperimentWorkspacePage refresh flow (single source).
// ============================================================
class GraphFullscreenDialog : public QDialog {
  Q_OBJECT
  public:
  explicit GraphFullscreenDialog(const QString &title, QWidget *parent = nullptr);
  void updateData(const QString &title, const QString &xLabel, const QString &yLabel,
  const QMap<QString, QVector<QPointF>> &series, bool connectPoints = true,
  const QString &emptyMessage = QString());
  protected:
  void keyPressEvent(QKeyEvent *e) override;
  private:
  ReportPlotWidget *plot_;
};

// ============================================================
// ExperimentParameterPanel — dynamic parameter editor for active leaf.
// Lives in the LEFT sidebar (bottom area). Rebuilt per leaf.
// Backed by ExperimentSpec.parameterFields (data-driven, no hard-coded branches).
// ============================================================
class ExperimentParameterPanel : public QWidget {
  Q_OBJECT
  public:
  explicit ExperimentParameterPanel(QWidget *parent = nullptr);
  // Build the panel for a new leaf. `stores` is used for yaml_readonly fields.
  void buildFor(const QString &subsystem, const QString &leafId,
  const QVector<ExperimentParameterField> &fields,
  const QMap<QString, std::shared_ptr<YamlStore>> &stores);
  // Read current value for a key (used by ExperimentWorkspacePage for recording/summary).
  QVariant value(const QString &key) const;
  QString text(const QString &key) const;
  void setText(const QString &key, const QString &value);
  // Get all filled run identity fields (variation/condition/gt) for manifest.
  QMap<QString, QString> runIdentity() const;
  signals:
  // Emitted when an editable (non-yaml) field changes; owner may autosave or apply.
  void parameterEdited(const QString &key, const QVariant &value);
  private:
  QGroupBox *newGroup(const QString &title);
  void addField(QWidget *group, const ExperimentParameterField &f);
  void clearWidgets();
  struct ParamWidget {
    QString key;
    QString kind;
    QLineEdit *line = nullptr;
    QPushButton *clearBtn = nullptr;  // for yaml_readonly reset (unused, kept minimal)
  };
  QLabel *header_;
  QScrollArea *scroll_;
  QWidget *container_;
  QVBoxLayout *containerLayout_;
  QVector<QGroupBox *> groups_;
  QMap<QString, ParamWidget> widgets_;
  QVector<ExperimentParameterField> fields_;
  QMap<QString, std::shared_ptr<YamlStore>> stores_;
  QString currentSubsystem_, currentLeafId_;
};

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
