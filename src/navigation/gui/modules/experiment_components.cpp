// Implementation file for experiment_components.hpp.
// This file is #included as a translation unit from agv_gui.cpp AFTER all local
// widget/store classes (ReportPlotWidget, YamlStore, etc.) are fully defined.
// Keeping method bodies here (not inline in the header) avoids "incomplete type"
// errors when moc compiles the header.
#include "experiment_components.hpp"
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QVBoxLayout>

// ============================================================
// GraphCard implementation
// ============================================================
GraphCard::GraphCard(const QString &title, QWidget *parent)
: QWidget(parent), title_(title) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(4);
  auto *header = new QHBoxLayout();
  titleLabel_ = new QLabel(title);
  titleLabel_->setObjectName(QStringLiteral("graphCardTitle"));
  header->addWidget(titleLabel_, 1);
  maximize_ = new QPushButton(QStringLiteral("⛶"));
  maximize_->setObjectName(QStringLiteral("graphCardMaximize"));
  maximize_->setFixedSize(26, 22);
  maximize_->setToolTip(QStringLiteral("Perbesar grafik ke layar penuh"));
  header->addWidget(maximize_);
  layout->addLayout(header);
  plot_ = new ReportPlotWidget();
  plot_->setMinimumHeight(220);
  layout->addWidget(plot_, 1);
  connect(maximize_, &QPushButton::clicked, this, [this]() {
    emit maximizeRequested();
  });
}

void GraphCard::setTitle(const QString &t) {
  title_ = t;
  titleLabel_->setText(t);
}

void GraphCard::setData(const QString &title, const QString &xLabel, const QString &yLabel,
const QMap<QString, QVector<QPointF>> &series, bool connectPoints, const QString &emptyMessage) {
  plot_->setPlot(title, xLabel, yLabel, series, connectPoints, emptyMessage);
}

void GraphCard::clear() {
  plot_->clear();
}

ReportPlotWidget *GraphCard::plot() const {
  return plot_;
}

// ============================================================
// GraphFullscreenDialog implementation
// ============================================================
GraphFullscreenDialog::GraphFullscreenDialog(const QString &title, QWidget *parent)
: QDialog(parent) {
  setWindowTitle(title);
  setWindowFlags(windowFlags() | Qt::WindowMaximizeButtonHint);
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  plot_ = new ReportPlotWidget();
  plot_->setMinimumHeight(400);
  layout->addWidget(plot_, 1);
  setMinimumSize(640, 480);
  resize(1100, 720);
}

void GraphFullscreenDialog::updateData(const QString &title, const QString &xLabel, const QString &yLabel,
const QMap<QString, QVector<QPointF>> &series, bool connectPoints, const QString &emptyMessage) {
  plot_->setPlot(title, xLabel, yLabel, series, connectPoints, emptyMessage);
}

void GraphFullscreenDialog::keyPressEvent(QKeyEvent *e) {
  if (e->key() == Qt::Key_Escape) {
    accept();
    return;
  }
  QDialog::keyPressEvent(e);
}

// ============================================================
// ExperimentParameterPanel implementation
// ============================================================
ExperimentParameterPanel::ExperimentParameterPanel(QWidget *parent) : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(6, 6, 6, 6);
  layout->setSpacing(6);
  header_ = new QLabel(QStringLiteral("Pengujian Aktif"));
  header_->setObjectName(QStringLiteral("paramPanelHeader"));
  layout->addWidget(header_);
  scroll_ = new QScrollArea();
  scroll_->setWidgetResizable(true);
  scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  container_ = new QWidget();
  containerLayout_ = new QVBoxLayout(container_);
  containerLayout_->setContentsMargins(0, 0, 0, 0);
  containerLayout_->setSpacing(8);
  scroll_->setWidget(container_);
  layout->addWidget(scroll_, 1);
  currentSubsystem_ = QStringLiteral("navigation");
  currentLeafId_ = QStringLiteral("4.2.1");
}

void ExperimentParameterPanel::buildFor(const QString &subsystem, const QString &leafId,
const QVector<ExperimentParameterField> &fields,
const QMap<QString, std::shared_ptr<YamlStore>> &stores) {
  currentSubsystem_ = subsystem;
  currentLeafId_ = leafId;
  header_->setText(QStringLiteral("%1 — %2").arg(subsystem.toUpper()).arg(leafId));
  clearWidgets();
  stores_ = stores;
  fields_ = fields;
  // Group: Akuisisi
  auto *acqGroup = newGroup(QStringLiteral("Akuisisi"));
  // Group: Identitas Run
  auto *runGroup = newGroup(QStringLiteral("Identitas Run"));
  // Group: Ground Truth
  auto *gtGroup = newGroup(QStringLiteral("Ground Truth"));
  bool hasCalibration = false;
  for (const ExperimentParameterField &f : fields)
    if (f.key.startsWith(QStringLiteral("calib_"))) { hasCalibration = true; break; }
  QGroupBox *calibGroup = hasCalibration ? newGroup(QStringLiteral("Kalibrasi Konversi")) : nullptr;
  if (calibGroup) calibGroup->setObjectName(QStringLiteral("calibrationGroup"));
  // Group: Parameter / YAML
  auto *yamlGroup = newGroup(QStringLiteral("Parameter / YAML"));
  for (const ExperimentParameterField &f : fields) {
    QWidget *target = yamlGroup;
    if (f.key == QStringLiteral("sample_rate") || f.key == QStringLiteral("duration"))
    target = acqGroup;
    else if (f.key == QStringLiteral("variation") || f.key == QStringLiteral("condition"))
    target = runGroup;
    else if (f.key.startsWith(QStringLiteral("gt_")))
    target = gtGroup;
    else if (calibGroup && f.key.startsWith(QStringLiteral("calib_")))
    target = calibGroup;
    addField(target, f);
  }
  containerLayout_->addStretch(1);
}

QVariant ExperimentParameterPanel::value(const QString &key) const {
  auto it = widgets_.find(key);
  if (it == widgets_.end() || !it->line) return {};
  const QString text = it->line->text().trimmed();
  if (text.isEmpty()) return {};
  if (it->kind == QStringLiteral("float") || it->kind == QStringLiteral("int")) {
    bool ok = false;
    double v = text.toDouble(&ok);
    return ok ? QVariant(v) : QVariant();
  }
  return QVariant(text);
}

QString ExperimentParameterPanel::text(const QString &key) const {
  auto it = widgets_.find(key);
  if (it == widgets_.end() || !it->line) return QString();
  return it->line->text().trimmed();
}

void ExperimentParameterPanel::setText(const QString &key, const QString &value) {
  auto it = widgets_.find(key);
  if (it != widgets_.end() && it->line) it->line->setText(value);
}

QMap<QString, QString> ExperimentParameterPanel::runIdentity() const {
  QMap<QString, QString> out;
  for (auto it = widgets_.begin(); it != widgets_.end(); ++it) {
    const QString t = it->line ? it->line->text().trimmed() : QString();
    if (!t.isEmpty()) out[it.key()] = t;
  }
  return out;
}

QGroupBox *ExperimentParameterPanel::newGroup(const QString &title) {
  auto *box = new QGroupBox(title);
  box->setObjectName(QStringLiteral("paramGroup"));
  auto *l = new QVBoxLayout(box);
  l->setContentsMargins(6, 6, 6, 6);
  l->setSpacing(4);
  groups_ << box;
  containerLayout_->addWidget(box);
  return box;
}

void ExperimentParameterPanel::addField(QWidget *group, const ExperimentParameterField &f) {
  auto *box = qobject_cast<QGroupBox *>(group);
  if (!box) return;
  auto *l = qobject_cast<QVBoxLayout *>(box->layout());
  if (!l) return;
  auto *row = new QHBoxLayout();
  auto *label = new QLabel(f.label);
  label->setWordWrap(true);
  row->addWidget(label, 1);
  auto *edit = new QLineEdit();
  const bool calibrationField = f.key.startsWith(QStringLiteral("calib_"));
  if (calibrationField) label->setObjectName(QStringLiteral("calibrationFieldLabel"));
  if (!f.placeholder.isEmpty()) edit->setPlaceholderText(f.placeholder);
  if (f.kind == QStringLiteral("yaml_readonly") || f.locked) {
    edit->setReadOnly(true);
    edit->setObjectName(calibrationField ? QStringLiteral("calibrationReadonlyField")
                                         : (f.locked ? QStringLiteral("lockedParameterField")
                                                     : QStringLiteral("yamlReadonlyField")));
    QString value = f.lockedValue;
    const auto store = stores_.value(f.yamlFileKey);
    if (store && !f.yamlPath.isEmpty()) {
      const QVariant v = store->get(f.yamlPath);
      if (v.isValid()) value = v.toString();
    }
    if (!value.isEmpty()) edit->setText(value);
    if (f.locked) label->setText(QStringLiteral("LOCKED — %1").arg(f.label));
  } else if (!f.yamlFileKey.isEmpty() && !f.yamlPath.isEmpty()) {
    if (calibrationField) edit->setObjectName(QStringLiteral("calibrationParameterField"));
    // Parameter tuning editable selalu menampilkan nilai aktual dari YAML source.
    // Dengan demikian operator tidak pernah mulai dari placeholder yang berbeda
    // dari konfigurasi yang benar-benar dipakai autonomous.launch.py.
    const auto store = stores_.value(f.yamlFileKey);
    if (store) {
      const QVariant value = store->get(f.yamlPath);
      if (value.isValid()) edit->setText(value.toString());
    }
  } else if (f.isGroundTruth) {
    edit->setObjectName(QStringLiteral("groundTruthField"));
  }
  row->addWidget(edit, 1);
  l->addLayout(row);
  ParamWidget w;
  w.key = f.key;
  w.kind = f.kind;
  w.line = edit;
  widgets_[f.key] = w;
  // Editable, YAML-backed fields write straight to the backing YamlStore so the
  // value takes effect when autonomous.launch.py is (re)launched. Single source
  // of truth: the YAML file already consumed by the navigation stack.
  if (f.kind != QStringLiteral("yaml_readonly") && !f.locked && !f.yamlFileKey.isEmpty() && !f.yamlPath.isEmpty()) {
    const QString fk = f.yamlFileKey;
    const QString fp = f.yamlPath;
    const QString fkind = f.kind;
    connect(edit, &QLineEdit::textChanged, this, [this, fk, fp, fkind](const QString &t) {
      auto store = stores_.value(fk);
      if (!store) return;
      QVariant v = t.trimmed();
      if (fkind == QStringLiteral("float") || fkind == QStringLiteral("int")) {
        bool ok = false;
        const double d = t.toDouble(&ok);
        if (!ok) return;  // ignore non-numeric while typing
        v = fkind == QStringLiteral("int")
            ? QVariant::fromValue<qlonglong>(static_cast<qlonglong>(std::llround(d)))
            : QVariant(d);
      } else if (fkind == QStringLiteral("bool")) {
        const QString low = t.trimmed().toLower();
        if (low == QStringLiteral("true") || low == QStringLiteral("on") || low == QStringLiteral("1"))
          v = true;
        else if (low == QStringLiteral("false") || low == QStringLiteral("off") || low == QStringLiteral("0"))
          v = false;
        else
          return;
      }
      store->set(fp, v);
    });
  } else if (f.kind != QStringLiteral("yaml_readonly") && !f.locked) {
    connect(edit, &QLineEdit::textChanged, this, [this, f](const QString &t) {
      emit parameterEdited(f.key, t);
    });
  }
}

void ExperimentParameterPanel::clearWidgets() {
  for (QGroupBox *g : groups_) {
    g->deleteLater();
  }
  groups_.clear();
  widgets_.clear();
}
