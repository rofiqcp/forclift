// Extracted from agv_gui.cpp for maintainability.
class DiagnosticPage:public QWidget{
  public:DiagnosticPage(const QString&title,const QString&desc,TelemetryStore*telemetry,ReportManager*reports,const QMap<QString,QString>&series,const QStringList&raw,QWidget*p=nullptr):QWidget(p),telemetry_(telemetry),reports_(reports),series_(series),raw_(raw),title_(title){
    auto*l=new QVBoxLayout(this);
    auto*h=new QLabel(title);
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto*d=new QLabel(desc);
    d->setWordWrap(true);
    d->setObjectName("pageDescription");
    l->addWidget(d);
    auto*cards=new QGridLayout();
    int c=0;
    for(auto it=series_.cbegin();
    it!=series_.cend();
    ++it){
      auto*lab=new QLabel(it.key()+": --");
      lab->setObjectName("metricCard");
      metrics_[it.key()]=lab;
      cards->addWidget(lab,c/3,c%3);
      ++c;
    }
    l->addLayout(cards);
    plot_=new LivePlotWidget(title);
    l->addWidget(plot_,1);
    rawText_=new QPlainTextEdit();
    rawText_->setReadOnly(true);
    rawText_->setMaximumBlockCount(250);
    rawText_->setMaximumHeight(170);
    l->addWidget(rawText_);
    auto*bar=new QHBoxLayout();
    record_=new QPushButton("● Mulai Rekam CSV");
    auto*clear=new QPushButton("Bersihkan Grafik");
    auto*png=new QPushButton("Ekspor PNG");
    bar->addWidget(record_);
    bar->addWidget(clear);
    bar->addWidget(png);
    bar->addStretch();
    l->addLayout(bar);
    connect(record_,&QPushButton::clicked,this,[this](){
      toggleRecording();
    });
    connect(clear,&QPushButton::clicked,plot_,&LivePlotWidget::clear);
    connect(png,&QPushButton::clicked,this,[this](){
      QString p=reports_->savePng(slug(title_),plot_);
      if(!p.isEmpty())QMessageBox::information(this,"Ekspor PNG",p);
    });
    timer_=new QTimer(this);
    timer_->setInterval(200);
    connect(timer_,&QTimer::timeout,this,[this](){
      refresh();
    });
    timer_->start();
  }
  void refresh(){
    QMap<QString,double>vals;
    for(auto it=series_.cbegin();
    it!=series_.cend();
    ++it){
      QVariant v=telemetry_->get(it.value());
      double x=number(v);
      metrics_[it.key()]->setText(it.key()+": "+variantText(v));
      if(std::isfinite(x))vals[it.key()]=x;
    }
    if(!vals.isEmpty())plot_->append(vals);
    QStringList rawLines;
    for(const QString&ch:raw_){
      QVariant v=telemetry_->get(ch);
      if(v.isValid()){
        QString s;
        if(v.userType()==QMetaType::QVariantMap)s=QString::fromUtf8(QJsonDocument(QJsonObject::fromVariantMap(v.toMap())).toJson(QJsonDocument::Compact));
        else s=v.toString();
        rawLines<<ch+": "+s;
      }
    }
    rawText_->setPlainText(rawLines.join('\n'));
    if(recording_){
      QVariantMap row;
      row["time_iso"]=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
      for(auto it=series_.cbegin();
      it!=series_.cend();
      ++it)row[it.key()]=telemetry_->get(it.value());
      for(const QString&ch:raw_)row[ch]=telemetry_->get(ch);
      rows_.push_back(row);
    }
  }
  private:TelemetryStore*telemetry_;
  ReportManager*reports_;
  QMap<QString,QString>series_;
  QStringList raw_;
  QString title_;
  QHash<QString,QLabel*>metrics_;
  LivePlotWidget*plot_;
  QPlainTextEdit*rawText_;
  QPushButton*record_;
  QTimer*timer_;
  bool recording_=false;
  QVector<QVariantMap>rows_;
  void toggleRecording(){
    if(!recording_){
      rows_.clear();
      recording_=true;
      record_->setText("■ Stop + Simpan CSV");
    }
    else{
      recording_=false;
      record_->setText("● Mulai Rekam CSV");
      QString p=reports_->saveCsv(slug(title_),rows_);
      QMessageBox::information(this,"CSV tersimpan",p);
    }
  }
};
class ConnectionPage:public QWidget{
  public:explicit ConnectionPage(TelemetryStore*t,QWidget*p=nullptr):QWidget(p),t_(t){
    auto*l=new QVBoxLayout(this);
    auto*h=new QLabel("Koneksi & Readiness Sistem");
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto*g=new QGridLayout();
    const QStringList names={
      "GNSS","IMU","CAMERA","ESC READY","ESC ARMED","NAV2","AUTONOMY","MOTION","E-STOP"
    };
    const QStringList keys={
      "connected.gnss","connected.imu","connected.camera","connected.esc_ready","connected.esc_armed","system.nav2_ready","system.autonomy_ready","system.motion_ready","system.estop"
    };
    for(int i=0;
    i<names.size();
    ++i){
      auto*pill=new StatusPill(names[i]);
      pills_.push_back(pill);
      keys_.push_back(keys[i]);
      g->addWidget(new QLabel(names[i]),i/3*2,i%3);
      g->addWidget(pill,i/3*2+1,i%3);
    }
    l->addLayout(g);
    auto*note=new QLabel("Hijau berarti data/readiness aktif. E-STOP hijau hanya saat false (aman). Periksa source serial dan launch jika status tetap OFF.");
    note->setWordWrap(true);
    l->addWidget(note);
    l->addStretch();
    auto*tm=new QTimer(this);
    tm->setInterval(250);
    connect(tm,&QTimer::timeout,this,[this](){
      for(int i=0;
      i<pills_.size();
      ++i){
        bool v=t_->get(keys_[i],false).toBool();
        if(keys_[i]=="system.estop")pills_[i]->setStatus(v?"bad":"ok",v?"E-STOP":"AMAN");
        else pills_[i]->setStatus(v?"ok":"bad",v?"READY":"OFF");
      }
    });
    tm->start();
  }
  private:TelemetryStore*t_;
  QVector<StatusPill*>pills_;
  QStringList keys_;
};
class NavigationTuningPage:public QWidget{
  public:NavigationTuningPage(TelemetryStore*t,ReportManager*r,const QMap<QString,std::shared_ptr<YamlStore>>&s,QWidget*p=nullptr):QWidget(p),t_(t),r_(r),s_(s){
    auto*l=new QVBoxLayout(this);
    auto*h=new QLabel("Navigasi & MPPI — Tuning Terukur");
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto*d=new QLabel("Bandingkan command Nav2 dengan feedback kendaraan. Profile A/B/C hanya mengubah YAML dan selalu membuat backup; lakukan restart lifecycle sebelum run pengujian.");
    d->setWordWrap(true);
    l->addWidget(d);
    auto*box=new QGroupBox("Profil MPPI");
    auto*g=new QGridLayout(box);
    profile_=new NoWheelComboBox();
    profile_->addItems({
      "A - Konservatif","B - Balanced","C - Responsif"
    });
    auto*apply=new QPushButton("Terapkan Profil");
    auto*sweep=new QPushButton("Buat Paket Sweep");
    g->addWidget(new QLabel("Profil"),0,0);
    g->addWidget(profile_,0,1);
    g->addWidget(apply,0,2);
    g->addWidget(sweep,0,3);
    summary_=new QLabel();
    summary_->setWordWrap(true);
    g->addWidget(summary_,1,0,1,4);
    l->addWidget(box);
    auto*sm=new QGroupBox("Velocity Smoother");
    auto*sg=new QHBoxLayout(sm);
    smoother_=new NoWheelComboBox();
    smoother_->addItems({
      "OPEN_LOOP","CLOSED_LOOP"
    });
    auto*save=new QPushButton("Tulis Mode ke YAML");
    eligible_=new StatusPill("BELUM QUALIFIED");
    sg->addWidget(new QLabel("Feedback"));
    sg->addWidget(smoother_);
    sg->addWidget(save);
    sg->addWidget(eligible_);
    sg->addStretch();
    l->addWidget(sm);
    plot_=new LivePlotWidget("Command vs Feedback / Error");
    l->addWidget(plot_,1);
    record_=new QPushButton("● Mulai Run Tuning");
    l->addWidget(record_);
    connect(profile_,&QComboBox::currentTextChanged,this,[this](){
      updateSummary();
    });
    connect(apply,&QPushButton::clicked,this,[this](){
      applyProfile();
    });
    connect(sweep,&QPushButton::clicked,this,[this](){
      generateSweep();
    });
    connect(save,&QPushButton::clicked,this,[this](){
      if(smoother_->currentText()=="CLOSED_LOOP"&&!t_->get("smoother_closed_loop_eligible",false).toBool()){
        QMessageBox::warning(this,"CLOSED_LOOP","Qualification belum PASS.");
        return;
      }
      s_["nav2"]->set("velocity_smoother.ros__parameters.feedback",smoother_->currentText());
      QMessageBox::information(this,"Velocity smoother","YAML tersimpan. Restart velocity_smoother sebelum pengujian.");
    });
    connect(record_,&QPushButton::clicked,this,[this](){
      if(!rec_){
        rows_.clear();
        rec_=true;
        record_->setText("■ Stop + Analisis CSV");
      }
      else{
        rec_=false;
        record_->setText("● Mulai Run Tuning");
        QMessageBox::information(this,"Run selesai",r_->saveCsv("mppi_tuning",rows_));
      }
    });
    timer_=new QTimer(this);
    timer_->setInterval(150);
    connect(timer_,&QTimer::timeout,this,[this](){
      refresh();
    });
    timer_->start();
    updateSummary();
  }
  private:TelemetryStore*t_;
  ReportManager*r_;
  QMap<QString,std::shared_ptr<YamlStore>>s_;
  QComboBox*profile_;
  QComboBox*smoother_;
  StatusPill*eligible_;
  QLabel*summary_;
  LivePlotWidget*plot_;
  QPushButton*record_;
  QTimer*timer_;
  bool rec_=false;
  QVector<QVariantMap>rows_;
  QVariantMap profileValues()const{
    QString p=profile_->currentText();
    if(p.startsWith('A'))return{
      {
        "vx_max",0.20
      },{
        "vx_std",0.10
      },{
        "wz_std",0.15
      },{
        "temperature",0.20
      },{
        "batch_size",1000
      },{
        "path_align",18.0
      },{
        "path_follow",8.0
      },{
        "path_angle",4.0
      }
    };
    if(p.startsWith('C'))return{
      {
        "vx_max",0.40
      },{
        "vx_std",0.25
      },{
        "wz_std",0.35
      },{
        "temperature",0.35
      },{
        "batch_size",1800
      },{
        "path_align",12.0
      },{
        "path_follow",12.0
      },{
        "path_angle",6.0
      }
    };
    return{
      {
        "vx_max",0.30
      },{
        "vx_std",0.20
      },{
        "wz_std",0.25
      },{
        "temperature",0.30
      },{
        "batch_size",1400
      },{
        "path_align",15.0
      },{
        "path_follow",10.0
      },{
        "path_angle",5.0
      }
    };
  }
  void updateSummary(){
    auto v=profileValues();
    summary_->setText(QString("vx_max=%1 m/s • vx_std=%2 • wz_std=%3 • temperature=%4 • batch=%5").arg(v["vx_max"].toDouble()).arg(v["vx_std"].toDouble()).arg(v["wz_std"].toDouble()).arg(v["temperature"].toDouble()).arg(v["batch_size"].toInt()));
  }
  void applyProfile(){
    auto st=s_.value("nav2");
    auto veh=s_.value("vehicle");
    if(!st||!veh)return;
    const QString backup=st->path()+".before_"+nowStamp()+".bak";
    QFile::copy(st->path(),backup);
    auto v=profileValues();
    double reqV=v["vx_max"].toDouble(),vehicleV=number(veh->get("vehicle.ros__parameters.max_forward_speed_mps",0.5),0.5),rmin=number(veh->get("vehicle.ros__parameters.minimum_turning_radius_m",1.712159378317),1.712159378317),vehicleYaw=number(veh->get("vehicle.ros__parameters.max_yaw_rate_rps",0.292028888392),0.292028888392);
    double vx=std::min(reqV,vehicleV),wz=std::min(vehicleYaw,vx/std::max(0.10,rmin));
    st->set("controller_server.ros__parameters.FollowPath.vx_max",vx);
    st->set("controller_server.ros__parameters.FollowPath.wz_max",wz);
    st->set("controller_server.ros__parameters.FollowPath.vx_std",v["vx_std"]);
    st->set("controller_server.ros__parameters.FollowPath.wz_std",v["wz_std"]);
    st->set("controller_server.ros__parameters.FollowPath.temperature",v["temperature"]);
    st->set("controller_server.ros__parameters.FollowPath.batch_size",v["batch_size"]);
    st->set("controller_server.ros__parameters.FollowPath.PathAlignCritic.cost_weight",v["path_align"]);
    st->set("controller_server.ros__parameters.FollowPath.PathFollowCritic.cost_weight",v["path_follow"]);
    st->set("controller_server.ros__parameters.FollowPath.PathAngleCritic.cost_weight",v["path_angle"]);
    st->set("velocity_smoother.ros__parameters.max_velocity",QVariantList{
      vx,0.0,wz
    });
    st->set("velocity_smoother.ros__parameters.min_velocity",QVariantList{
      0.0,0.0,-wz
    });
    auto ts=s_.value("trajectory_safety");
    if(ts){
      ts->set("trajectory_safety_supervisor.ros__parameters.maximum_yaw_rate_rps",wz);
      ts->set("trajectory_safety_supervisor.ros__parameters.minimum_turning_radius_m",rmin);
    }
    QMessageBox::information(this,"MPPI profile",QString("Profile tersimpan aman: vx_max=%1 m/s, wz_max=%2 rad/s, Rmin=%3 m.\nBackup: %4\nRestart controller_server + velocity_smoother sebelum run.").arg(vx).arg(wz).arg(rmin).arg(backup));
  }
  void generateSweep(){
    QString dir=QDir(r_->root()).filePath("mppi_sweep_"+nowStamp());
    QDir().mkpath(dir);
    QString old=profile_->currentText();
    for(const QString&p:{
      QString("A - Konservatif"),QString("B - Balanced"),QString("C - Responsif")
    }){
      profile_->setCurrentText(p);
      QJsonObject o=QJsonObject::fromVariantMap(profileValues());
      QSaveFile f(QDir(dir).filePath(slug(p)+".json"));
      if(f.open(QIODevice::WriteOnly)){
        f.write(QJsonDocument(o).toJson(QJsonDocument::Indented));
        f.commit();
      }
    }
    profile_->setCurrentText(old);
    QMessageBox::information(this,"Sweep Pack",dir);
  }
  void refresh(){
    bool ok=t_->get("smoother_closed_loop_eligible",false).toBool();
    eligible_->setStatus(ok?"ok":"warn",ok?"CLOSED_LOOP ELIGIBLE":"OPEN_LOOP DISARANKAN");
    QMap<QString,double>v;
    for(auto p:{
      std::pair<QString,QString>{
        "v err","mppi_velocity_error"
      },{
        "steer err","mppi_steering_error"
      },{
        "yaw err","mppi_yaw_error"
      },{
        "target v","esc_drive_target"
      },{
        "actual v","esc_drive_actual"
      }
    }){
      double x=number(t_->get(p.second));
      if(std::isfinite(x))v[p.first]=x;
    }
    if(!v.isEmpty())plot_->append(v);
    if(rec_){
      QVariantMap row{
        {
          "time",QDateTime::currentDateTime().toString(Qt::ISODateWithMs)
        }
      };
      for(auto it=v.cbegin();
      it!=v.cend();
      ++it)row[it.key()]=it.value();
      row["smoother_eligible"]=ok;
      rows_.push_back(row);
    }
  }
};
class HostMetricsSampler {
  public:
  QVariant value(const QString &path) {
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if (nowMs - lastSampleMs_ > 900) {
      sample();
      lastSampleMs_ = nowMs;
    }
    return cache_.value(path);
  }
  private:
  qint64 lastSampleMs_{
    0
  };
  qulonglong previousTotal_{
    0
  }, previousIdle_{
    0
  };
  QVariantMap cache_;
  static double readNumber(const QString &path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return std::numeric_limits<double>::quiet_NaN();
    bool ok = false;
    const double value = QString::fromUtf8(file.readAll()).trimmed().toDouble(&ok);
    return ok ? value : std::numeric_limits<double>::quiet_NaN();
  }
  void sample() {
    QFile mem(QStringLiteral("/proc/meminfo"));
    if (mem.open(QIODevice::ReadOnly | QIODevice::Text)) {
      const QString text = QString::fromUtf8(mem.readAll());
      const auto totalMatch = QRegularExpression(QStringLiteral("MemTotal:\\s+([0-9]+)")).match(text);
      const auto availableMatch = QRegularExpression(QStringLiteral("MemAvailable:\\s+([0-9]+)")).match(text);
      if (totalMatch.hasMatch() && availableMatch.hasMatch()) {
        const double totalKb = totalMatch.captured(1).toDouble();
        const double availableKb = availableMatch.captured(1).toDouble();
        cache_[QStringLiteral("host.ram_used_gb")] = (totalKb - availableKb) / 1048576.0;
        cache_[QStringLiteral("host.ram_percent")] = totalKb > 0.0 ? 100.0 * (totalKb - availableKb) / totalKb : 0.0;
      }
    }
    QFile stat(QStringLiteral("/proc/stat"));
    if (stat.open(QIODevice::ReadOnly | QIODevice::Text)) {
      const QStringList fields = QString::fromUtf8(stat.readLine()).simplified().split(' ');
      if (fields.size() >= 8 && fields[0] == QStringLiteral("cpu")) {
        qulonglong total = 0;
        for (int index = 1;
        index < fields.size();
        ++index) total += fields[index].toULongLong();
        const qulonglong idle = fields[4].toULongLong() + (fields.size() > 5 ? fields[5].toULongLong() : 0ULL);
        if (previousTotal_ > 0 && total > previousTotal_) {
          const qulonglong deltaTotal = total - previousTotal_;
          const qulonglong deltaIdle = idle - previousIdle_;
          cache_[QStringLiteral("host.cpu_percent")] = 100.0 * double(deltaTotal - std::min(deltaIdle, deltaTotal)) / double(deltaTotal);
        }
        previousTotal_ = total;
        previousIdle_ = idle;
      }
    }
    double maximumTemperature = std::numeric_limits<double>::quiet_NaN();
    QDirIterator thermal(QStringLiteral("/sys/class/thermal"), QStringList{
      QStringLiteral("thermal_zone*")
    }, QDir::Dirs | QDir::NoDotAndDotDot);
    while (thermal.hasNext()) {
      const QString directory = thermal.next();
      double temperature = readNumber(QDir(directory).filePath(QStringLiteral("temp")));
      if (std::isfinite(temperature) && temperature > 1000.0) temperature /= 1000.0;
      if (std::isfinite(temperature)) maximumTemperature = std::isfinite(maximumTemperature) ? std::max(maximumTemperature, temperature) : temperature;
    }
    if (std::isfinite(maximumTemperature)) cache_[QStringLiteral("host.temperature_c")] = maximumTemperature;
    for (const QString &candidate : {
      QStringLiteral("/sys/class/drm/card0/device/gpu_busy_percent"),
      QStringLiteral("/sys/devices/gpu.0/load")
    }) {
      double load = readNumber(candidate);
      if (!std::isfinite(load)) continue;
      if (load > 100.0) load /= 10.0;
      cache_[QStringLiteral("host.gpu_percent")] = std::clamp(load, 0.0, 100.0);
      break;
    }
  }
};
struct ExperimentSessionData {
  QVector<QVariantMap> rawRows;
  QVector<QVariantMap> summaryRows;
  QMap<QString, QVector<QPointF>> liveSeries;
  QMap<int, QVector<QPointF>> liveScatter;
};
class ExperimentWorkspacePage : public QWidget {
  public:
  explicit ExperimentWorkspacePage(const QString &subsystem, TelemetryStore *telemetry, ReportManager *reports,
  const QMap<QString, std::shared_ptr<YamlStore>> &stores, QWidget *parent = nullptr)
  : QWidget(parent), subsystem_(subsystem), telemetry_(telemetry), reports_(reports), stores_(stores),
  catalog_(buildExperimentCatalog(subsystem)) {
    for (int i = 0;
    i < catalog_.size();
    ++i) leafIndex_[catalog_[i].id] = i;
    auto *layout = new QVBoxLayout(this);
    auto *title = new QLabel(subsystemTitle() + QStringLiteral(" — Akuisisi Data BAB IV"));
    title->setObjectName(QStringLiteral("pageTitle"));
    layout->addWidget(title);
    auto *description = new QLabel(QStringLiteral(
    "Tabel mengikuti kolom laporan. Hijau diisi otomatis dari topic/source yang tersedia; kuning harus diisi dari ground truth atau instrumen eksternal. "
    "Tidak ada nilai estimasi yang dipakai sebagai hasil aktual."));
    description->setWordWrap(true);
    description->setObjectName(QStringLiteral("pageDescription"));
    layout->addWidget(description);
    breadcrumb_ = new QLabel(subsystemTitle());
    breadcrumb_->setObjectName(QStringLiteral("breadcrumb"));
    breadcrumb_->setWordWrap(true);
    layout->addWidget(breadcrumb_);
    auto *selectorBox = new QGroupBox(QStringLiteral("Tabel aktif"));
    auto *selectorLayout = new QGridLayout(selectorBox);
    tableSelector_ = new NoWheelComboBox();
    plotMode_ = new NoWheelComboBox();
    plotMode_->addItems({
      QStringLiteral("Grafik live per waktu"), QStringLiteral("Grafik ringkasan tabel")
    });
    selectorLayout->addWidget(new QLabel(QStringLiteral("Tabel")), 0, 0);
    selectorLayout->addWidget(tableSelector_, 0, 1);
    selectorLayout->addWidget(new QLabel(QStringLiteral("Mode")), 0, 2);
    selectorLayout->addWidget(plotMode_, 0, 3);
    tableCaption_ = new QLabel();
    tableCaption_->setWordWrap(true);
    tableCaption_->setObjectName(QStringLiteral("metricCard"));
    selectorLayout->addWidget(tableCaption_, 1, 0, 1, 4);
    layout->addWidget(selectorBox);
    // Toolbar kecil: rekam / ringkasan / simpan / bersihkan
    auto *buttons = new QHBoxLayout();
    record_ = new QPushButton(QStringLiteral("● Mulai Rekam Run"));
    auto *snapshot = new QPushButton(QStringLiteral("Ambil Ringkasan"));
    auto *manual = new QPushButton(QStringLiteral("Tambah Baris"));
    auto *remove = new QPushButton(QStringLiteral("Hapus Baris"));
    auto *save = new QPushButton(QStringLiteral("Simpan CSV + PNG + Raw"));
    auto *clear = new QPushButton(QStringLiteral("Bersihkan"));
    buttons->addWidget(record_);
    buttons->addWidget(snapshot);
    buttons->addWidget(manual);
    buttons->addWidget(remove);
    buttons->addStretch();
    buttons->addWidget(save);
    buttons->addWidget(clear);
    layout->addLayout(buttons);

    // Perception-only validation preview. It reuses the SAME annotated image and
    // TelemetryStore values already received by the GUI: no duplicate ROS node,
    // subscriber, recorder, or perception processing path is introduced.
    if (subsystem_ == QStringLiteral("perception")) {
      perceptionPreviewBox_ = new QGroupBox(QStringLiteral("Preview Validasi Perception — Data Aktual"));
      auto *previewLayout = new QHBoxLayout(perceptionPreviewBox_);
      perceptionPreviewImage_ = new QLabel(QStringLiteral("Menunggu frame anotasi perception..."));
      perceptionPreviewImage_->setAlignment(Qt::AlignCenter);
      perceptionPreviewImage_->setMinimumSize(460, 260);
      perceptionPreviewImage_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
      perceptionPreviewImage_->setStyleSheet(QStringLiteral(
        "QLabel{background:#090d13;border:1px solid #344255;border-radius:6px;color:#8fa4bd;padding:4px;}"));
      previewLayout->addWidget(perceptionPreviewImage_, 3);

      auto *side = new QWidget();
      auto *sideLayout = new QVBoxLayout(side);
      sideLayout->setContentsMargins(0, 0, 0, 0);
      perceptionPreviewTitle_ = new QLabel();
      perceptionPreviewTitle_->setWordWrap(true);
      perceptionPreviewTitle_->setStyleSheet(QStringLiteral("font-weight:800;font-size:14px;"));
      sideLayout->addWidget(perceptionPreviewTitle_);
      perceptionPreviewHint_ = new QLabel();
      perceptionPreviewHint_->setWordWrap(true);
      perceptionPreviewHint_->setStyleSheet(QStringLiteral("color:#9fb2c8;"));
      sideLayout->addWidget(perceptionPreviewHint_);
      perceptionPreviewStatus_ = new QLabel(QStringLiteral("FRAME: menunggu | DATA: menunggu"));
      perceptionPreviewStatus_->setWordWrap(true);
      sideLayout->addWidget(perceptionPreviewStatus_);
      perceptionMetricsHost_ = new QWidget();
      perceptionMetricsLayout_ = new QGridLayout(perceptionMetricsHost_);
      perceptionMetricsLayout_->setContentsMargins(0, 0, 0, 0);
      perceptionMetricsLayout_->setHorizontalSpacing(6);
      perceptionMetricsLayout_->setVerticalSpacing(6);
      sideLayout->addWidget(perceptionMetricsHost_, 1);
      previewLayout->addWidget(side, 2);
      layout->addWidget(perceptionPreviewBox_);
    }

    // Multi-graph workspace (scrollable vertical stack of GraphCards)
    graphScroll_ = new QScrollArea();
    graphScroll_->setWidgetResizable(true);
    graphScroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphContainer_ = new QWidget();
    graphLayout_ = new QVBoxLayout(graphContainer_);
    graphLayout_->setContentsMargins(0, 0, 0, 0);
    graphLayout_->setSpacing(10);
    graphScroll_->setWidget(graphContainer_);
    layout->addWidget(graphScroll_, 1);
    // Table section (collapsible)
    tableBox_ = new QGroupBox(QStringLiteral("Tabel & Ringkasan"));
    auto *tableBoxLayout = new QVBoxLayout(tableBox_);
    table_ = new QTableWidget();
    table_->setAlternatingRowColors(true);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    table_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    table_->verticalHeader()->setVisible(false);
    tableBoxLayout->addWidget(table_);
    layout->addWidget(tableBox_);
    status_ = new QLabel(QStringLiteral("Siap. Isi varian/ground truth lalu mulai run."));
    status_->setWordWrap(true);
    layout->addWidget(status_);
    availability_ = new QLabel();
    availability_->setWordWrap(true);
    layout->addWidget(availability_);
    connect(tableSelector_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int i){
      setTableIndex(i);
    });
    connect(plotMode_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int){
      refreshGraphs();
    });
    connect(record_, &QPushButton::clicked, this, [this](){
      toggleRecording();
    });
    connect(snapshot, &QPushButton::clicked, this, [this](){
      appendSummaryRow();
    });
    connect(manual, &QPushButton::clicked, this, [this](){
      addManualRow();
    });
    connect(remove, &QPushButton::clicked, this, [this](){
      removeSelectedRows();
    });
    connect(save, &QPushButton::clicked, this, [this](){
      saveEvidence();
    });
    connect(clear, &QPushButton::clicked, this, [this](){
      clearCurrent();
    });
    connect(table_, &QTableWidget::itemChanged, this, [this](QTableWidgetItem*){
      if(!loading_)saveTableState();
      refreshSummaryPlotIfNeeded();
    });
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this](){
      captureSample();
    });
    availabilityTimer_ = new QTimer(this);
    availabilityTimer_->setInterval(1000);
    connect(availabilityTimer_, &QTimer::timeout, this, [this](){
      updateAvailability();
    });
    availabilityTimer_->start();
    // Keep the acquisition view live even before a recording run starts.
    // captureSample() only appends to rawRows while recording, so this does not
    // contaminate the saved run buffer.
    liveElapsed_.start();
    timer_->start(200);
    if (!catalog_.isEmpty()) selectLeaf(catalog_.first().id);
  }
  // Called by the floating BAB IV menu: open a specific report leaf.
  void selectLeaf(const QString &id) {
    if (!leafIndex_.contains(id)) return;
    if (recording_) stopRecording(true);
    if (!currentId_.isEmpty()) saveTableState();
    currentId_ = id;
    currentTableIndex_ = 0;
    applyLeaf();
  }
  // Called by MainWindow with the same annotated QImage already shown by CameraPage.
  void setPerceptionImage(const QImage &image) {
    if (subsystem_ != QStringLiteral("perception") || image.isNull()) return;
    perceptionFrame_ = image;
    if (perceptionFrameClock_.isValid()) perceptionFrameClock_.restart();
    else perceptionFrameClock_.start();
  }
  // Used by MainWindow to push the active leaf's parameter definition to the sidebar.
  const QVector<ExperimentParameterField> &parameterFields() const {
    static const QVector<ExperimentParameterField> empty;
    int idx = leafIndex_.value(currentId_, -1);
    return idx >= 0 ? catalog_[idx].parameterFields : empty;
  }
  QString currentLeafId() const {
    return currentId_;
  }
  // Read a run-identity / ground-truth field value from the active leaf panel.
  QVariant parameterValue(const QString &key) const {
    if (!paramPanel_) return {};
    return paramPanel_->value(key);
  }
  QString parameterText(const QString &key) const {
    if (!paramPanel_) return QString();
    return paramPanel_->text(key);
  }
  void setParameterPanel(ExperimentParameterPanel *panel) {
    paramPanel_ = panel;
    if (panel) {
      connect(panel, &ExperimentParameterPanel::parameterEdited, this, [this](const QString &, const QVariant &) {
        // No-op: values are read live during recording/summary. Backend unchanged.
      });
    }
  }
  void pushParameterPanel() {
    if (!paramPanel_) return;
    paramPanel_->buildFor(subsystem_, currentId_, parameterFields(), stores_);
  }
  private:
  QString subsystem_;
  TelemetryStore *telemetry_;
  ReportManager *reports_;
  QMap<QString, std::shared_ptr<YamlStore>> stores_;
  QVector<ExperimentSpec> catalog_;
  QMap<QString, ExperimentSessionData> sessions_;
  QComboBox *plotMode_, *tableSelector_;
  QLabel *breadcrumb_, *tableCaption_, *availability_, *status_;
  QPushButton *record_;
  QScrollArea *graphScroll_;
  QWidget *graphContainer_;
  QVBoxLayout *graphLayout_;
  QGroupBox *tableBox_;
  QVector<GraphCard*> graphCards_;
  QTableWidget *table_;

  // Perception preview widgets/data. Null for Navigation and ESC pages.
  QGroupBox *perceptionPreviewBox_ = nullptr;
  QLabel *perceptionPreviewImage_ = nullptr;
  QLabel *perceptionPreviewTitle_ = nullptr;
  QLabel *perceptionPreviewHint_ = nullptr;
  QLabel *perceptionPreviewStatus_ = nullptr;
  QWidget *perceptionMetricsHost_ = nullptr;
  QGridLayout *perceptionMetricsLayout_ = nullptr;
  QMap<QString, QLabel*> perceptionMetricLabels_;
  QImage perceptionFrame_;
  QElapsedTimer perceptionFrameClock_;
  QTimer *timer_, *availabilityTimer_;
  QElapsedTimer elapsed_;
  QElapsedTimer liveElapsed_;
  HostMetricsSampler hostMetrics_;
  ExperimentParameterPanel *paramPanel_ = nullptr;
  GraphFullscreenDialog *fullscreenDialog_ = nullptr;
  int fullscreenGraphIndex_ = -1;
  bool recording_{
    false
  }, loading_{
    false
  };
  int activeRecordingSummaryRow_{-1};
  QElapsedTimer summaryUpdateClock_;
  QString currentId_;
  int currentTableIndex_{
    0
  };
  QMap<QString, int> leafIndex_;
  QString subsystemTitle() const {
    if (subsystem_ == QStringLiteral("navigation")) return QStringLiteral("NAVIGASI");
    if (subsystem_ == QStringLiteral("perception")) return QStringLiteral("PERSEPSI");
    return QStringLiteral("ESC / FOC");
  }
  const ExperimentSpec &spec() const {
    static ExperimentSpec empty;
    int idx = leafIndex_.value(currentId_, -1);
    return idx >= 0 ? catalog_[idx] : empty;
  }
  ExperimentSessionData &session() {
    return sessions_[currentId_];
  }
  QStringList currentColumns() const {
    return spec().tableColumns.value(currentTableIndex_);
  }
  QString currentTableCaption() const {
    return spec().tableNames.value(currentTableIndex_, QStringLiteral("Tabel 1"));
  }
  QString selectedGraphCaption() const {
    // Legacy: multi-graph sudah tidak pakai selector. Return first or empty.
    const ExperimentSpec &s = spec();
    return s.graphCaptions.isEmpty() ? QString() : s.graphCaptions.first();
  }
  static bool numericText(QString text, double &value) {
    text = text.trimmed();
    text.replace(',', '.');
    const auto match = QRegularExpression(QStringLiteral("[-+]?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][-+]?[0-9]+)?")).match(text);
    if (!match.hasMatch()) return false;
    bool ok = false;
    value = match.captured(0).toDouble(&ok);
    return ok && std::isfinite(value);
  }
  QVariant groundTruth(const QString &key) const {
    double value=0.0;
    return numericText(parameterText(key), value) ? QVariant(value) : QVariant();
  }
  #if defined(__GNUC__) || defined(__clang__)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wmisleading-indentation"
  #endif
  void setTableIndex(int i) {
    if (i < 0 || i >= spec().tableColumns.size()) return;
    currentTableIndex_ = i;
    applyLeaf();
  }
  void rebuildGraphCards() {
    // Clear old cards (no leak: delete widgets, clear vector).
    for (GraphCard *c : graphCards_) c->deleteLater();
    graphCards_.clear();
    const ExperimentSpec &s = spec();
    for (int i = 0; i < s.graphCaptions.size(); ++i) {
      GraphCard *card = new GraphCard(s.graphCaptions.at(i));
      connect(card, &GraphCard::maximizeRequested, this, [this, i]() {
        openFullscreenGraph(i);
      });
      graphLayout_->addWidget(card);
      graphCards_ << card;
    }
    if (graphCards_.isEmpty()) {
      auto *placeholder = new QLabel(QStringLiteral("Tidak ada grafik terdefinisi untuk subbab ini."));
      placeholder->setObjectName(QStringLiteral("metricCard"));
      placeholder->setWordWrap(true);
      graphLayout_->addWidget(placeholder);
    }
    graphScroll_->setVisible(!graphCards_.isEmpty());
  }
  void applyLeaf() {
    if (catalog_.isEmpty() || currentId_.isEmpty()) return;
    const ExperimentSpec &s = spec();
    // Report-only leaves (notably Navigation 4.1) are populated directly from
    // the DOCX-derived catalog. They remain editable/exportable but cannot start
    // a meaningless ROS acquisition run.
    if (session().summaryRows.isEmpty() && !s.defaultRows.isEmpty())
      session().summaryRows = s.defaultRows;
    record_->setEnabled(s.recordable);
    record_->setText(s.recordable ? QStringLiteral("● Mulai Rekam Run")
                                  : QStringLiteral("Ringkasan — tidak perlu Run"));
    breadcrumb_->setText(subsystemTitle() + QStringLiteral("  >  ") + s.groupId + QStringLiteral("  >  ") + s.section);
    tableCaption_->setText(QStringLiteral("TABEL: ") + currentTableCaption());
    {
      const QSignalBlocker blocker(tableSelector_);
      tableSelector_->clear();
      tableSelector_->addItems(s.tableNames);
      tableSelector_->setEnabled(s.tableNames.size() > 1);
      tableSelector_->setCurrentIndex(currentTableIndex_);
    }
    rebuildGraphCards();
    if (subsystem_ == QStringLiteral("perception")) {
      rebuildPerceptionPreviewMetrics();
      refreshPerceptionPreview();
    }
    loading_ = true;
    const QStringList cols = currentColumns();
    table_->clear();
    table_->setColumnCount(cols.size());
    table_->setHorizontalHeaderLabels(cols);
    table_->setRowCount(0);
    for (const QVariantMap &row : session().summaryRows) insertRow(row);
    table_->resizeColumnsToContents();
    loading_ = false;
    refreshGraphs();
    pushParameterPanel();
    updateAvailability();
  }
  QString perceptionPreviewHint() const {
    if (currentId_.startsWith(QStringLiteral("4.2")))
      return QStringLiteral("Cek bounding box halangan, confidence, posisi pusat deteksi, warning/path status, lalu cocokkan dengan kondisi/jarak aktual.");
    if (currentId_.startsWith(QStringLiteral("4.3")))
      return QStringLiteral("Cek class pallet/hole pallet, pusat deteksi pallet, confidence, kestabilan deteksi, lalu cocokkan dengan jarak/orientasi/pencahayaan aktual.");
    if (currentId_.startsWith(QStringLiteral("4.4")))
      return QStringLiteral("Crosshair menunjukkan pusat citra. Bandingkan error pixel/normalisasi serta error lateral/yaw runtime dengan ground truth pengukuran.");
    if (currentId_.startsWith(QStringLiteral("4.5")))
      return QStringLiteral("Pantau error lateral/yaw menuju nol, output kontrol, status toleransi, steering, dan safety selama proses docking.");
    if (currentId_ == QStringLiteral("4.1.5"))
      return QStringLiteral("Subbab training: preview runtime dipakai sebagai sanity-check implementasi model. Nilai training utama tetap berasal dari artefak training, bukan estimasi GUI.");
    return QStringLiteral("Cek frame anotasi, jumlah deteksi, confidence, FPS, kesehatan kamera, dan kesesuaian visual dengan kondisi fisik sebelum merekam data.");
  }
  QStringList perceptionPreviewPaths() const {
    QStringList paths;
    auto add=[&paths](const QString &p){ if(!p.isEmpty() && !paths.contains(p)) paths << p; };
    add(QStringLiteral("camera_healthy"));
    add(QStringLiteral("perception_performance.fps"));
    add(QStringLiteral("raw_detections.count"));
    add(QStringLiteral("raw_detections.mean_confidence"));
    if(currentId_.startsWith(QStringLiteral("4.2"))){
      add(QStringLiteral("raw_detections.best_class_name"));
      add(QStringLiteral("raw_detections.best_confidence"));
      add(QStringLiteral("raw_detections.best_center_x_px"));
      add(QStringLiteral("raw_detections.best_center_y_px"));
      add(QStringLiteral("raw_detections.warning_active"));
    }
    if(currentId_.startsWith(QStringLiteral("4.3"))){
      add(QStringLiteral("raw_detections.pallet_count"));
      add(QStringLiteral("raw_detections.pallet_best_class_name"));
      add(QStringLiteral("raw_detections.pallet_best_confidence"));
      add(QStringLiteral("raw_detections.pallet_best_center_x_px"));
      add(QStringLiteral("raw_detections.pallet_best_center_y_px"));
      add(QStringLiteral("alignment_state.detection_stable"));
    }
    if(currentId_.startsWith(QStringLiteral("4.4"))){
      add(QStringLiteral("derived.visual_error_px"));
      add(QStringLiteral("derived.visual_error_normalized"));
      add(QStringLiteral("derived.alignment_lateral_error_cm"));
      add(QStringLiteral("alignment_state.error_yaw_deg"));
      add(QStringLiteral("alignment_state.confidence"));
      add(QStringLiteral("alignment_state.data_valid"));
    }
    if(currentId_.startsWith(QStringLiteral("4.5"))){
      add(QStringLiteral("derived.alignment_lateral_error_cm"));
      add(QStringLiteral("alignment_state.error_yaw_deg"));
      add(QStringLiteral("alignment_state.pid_lateral_output"));
      add(QStringLiteral("alignment_state.pid_yaw_output"));
      add(QStringLiteral("alignment_state.estimated_steering_deg"));
      add(QStringLiteral("alignment_state.linear_velocity_cmd"));
      add(QStringLiteral("alignment_state.state_text"));
      add(QStringLiteral("alignment_state.ready_for_insertion"));
      add(QStringLiteral("alignment_state.safety_stop_active"));
    }
    for(auto it=spec().liveSeries.cbegin(); it!=spec().liveSeries.cend(); ++it) add(it.value());
    return paths.mid(0, 12);
  }
  QString perceptionPreviewLabel(const QString &path) const {
    static const QMap<QString,QString> names={
      {"camera_healthy","Camera"},
      {"perception_performance.fps","FPS"},
      {"perception_performance.mean_ms","Process mean"},
      {"perception_performance.capture_dropped","Capture drop"},
      {"raw_detections.count","Detection count"},
      {"raw_detections.mean_confidence","Mean confidence"},
      {"raw_detections.best_class_name","Best class"},
      {"raw_detections.best_confidence","Best confidence"},
      {"raw_detections.best_center_x_px","Best center X"},
      {"raw_detections.best_center_y_px","Best center Y"},
      {"raw_detections.warning_active","Warning"},
      {"raw_detections.pallet_count","Pallet count"},
      {"raw_detections.pallet_best_class_name","Pallet class"},
      {"raw_detections.pallet_best_confidence","Pallet confidence"},
      {"raw_detections.pallet_best_center_x_px","Pallet center X"},
      {"raw_detections.pallet_best_center_y_px","Pallet center Y"},
      {"camera_health_state.mean_luma","Mean luma"},
      {"camera_health_state.stddev_luma","Std luma"},
      {"derived.visual_error_px","Visual error"},
      {"derived.visual_error_normalized","Normalized error"},
      {"derived.alignment_lateral_error_cm","Lateral error"},
      {"alignment_state.error_yaw_deg","Yaw error"},
      {"alignment_state.confidence","Align confidence"},
      {"alignment_state.detection_stable","Detection stable"},
      {"alignment_state.data_valid","Alignment data"},
      {"alignment_state.pid_lateral_output","PID lateral"},
      {"alignment_state.pid_yaw_output","PID yaw"},
      {"alignment_state.estimated_steering_deg","Steering estimate"},
      {"alignment_state.linear_velocity_cmd","Velocity cmd"},
      {"alignment_state.state_text","Alignment state"},
      {"alignment_state.ready_for_insertion","Ready insertion"},
      {"alignment_state.safety_stop_active","Safety stop"}
    };
    return names.value(path,path);
  }
  QString formatPerceptionPreviewValue(const QString &path,const QVariant &value) const {
    if(!value.isValid()) return QStringLiteral("NO DATA");
    if(value.type()==QVariant::Bool) return value.toBool()?QStringLiteral("TRUE"):QStringLiteral("FALSE");
    bool ok=false;
    const double d=value.toDouble(&ok);
    if(ok && std::isfinite(d)){
      QString suffix;
      if(path.endsWith(QStringLiteral("_px")) || path==QStringLiteral("derived.visual_error_px")) suffix=QStringLiteral(" px");
      else if(path.endsWith(QStringLiteral("_deg"))) suffix=QStringLiteral("°");
      else if(path.endsWith(QStringLiteral("_cm"))) suffix=QStringLiteral(" cm");
      else if(path.endsWith(QStringLiteral(".fps"))) suffix=QStringLiteral(" Hz");
      else if(path.endsWith(QStringLiteral("_ms"))) suffix=QStringLiteral(" ms");
      return QString::number(d,'f',std::abs(d)>=100.0?1:3)+suffix;
    }
    QString text=value.toString().trimmed();
    return text.isEmpty()?QStringLiteral("NO DATA"):text;
  }
  double perceptionPathAge(const QString &path) const {
    QString channel=path.section('.',0,0);
    if(channel==QStringLiteral("derived")){
      if(path.contains(QStringLiteral("visual_error"))) channel=QStringLiteral("raw_detections");
      else if(path.contains(QStringLiteral("alignment"))) channel=QStringLiteral("alignment_state");
    }
    return telemetry_->age(channel);
  }
  void rebuildPerceptionPreviewMetrics(){
    if(!perceptionMetricsLayout_)return;
    while(QLayoutItem *item=perceptionMetricsLayout_->takeAt(0)){
      if(QWidget *w=item->widget())w->deleteLater();
      delete item;
    }
    perceptionMetricLabels_.clear();
    const QStringList paths=perceptionPreviewPaths();
    for(int i=0;i<paths.size();++i){
      const QString path=paths.at(i);
      auto *card=new QLabel();
      card->setWordWrap(true);
      card->setMinimumWidth(145);
      card->setAlignment(Qt::AlignLeft|Qt::AlignVCenter);
      card->setToolTip(path);
      perceptionMetricsLayout_->addWidget(card,i/2,i%2);
      perceptionMetricLabels_[path]=card;
    }
    perceptionPreviewTitle_->setText(QStringLiteral("%1 — %2").arg(spec().id,spec().section));
    perceptionPreviewHint_->setText(perceptionPreviewHint());
  }
  void refreshPerceptionPreview(){
    if(subsystem_!=QStringLiteral("perception") || !perceptionPreviewBox_)return;
    int live=0,total=0;
    for(auto it=perceptionMetricLabels_.begin();it!=perceptionMetricLabels_.end();++it){
      ++total;
      const QVariant v=instantValue(it.key());
      const double age=perceptionPathAge(it.key());
      const bool fresh=v.isValid() && std::isfinite(age) && age<=2.0;
      if(fresh)++live;
      it.value()->setText(QStringLiteral("<b>%1</b><br>%2")
        .arg(perceptionPreviewLabel(it.key()),formatPerceptionPreviewValue(it.key(),v).toHtmlEscaped()));
      it.value()->setStyleSheet(fresh
        ?QStringLiteral("QLabel{background:#12251c;border:1px solid #2e7d4d;border-radius:5px;padding:6px;}")
        :QStringLiteral("QLabel{background:#2b2114;border:1px solid #8b6425;border-radius:5px;padding:6px;}"));
    }

    QString frameText=QStringLiteral("FRAME: NO IMAGE");
    if(!perceptionFrame_.isNull()){
      const double age=perceptionFrameClock_.isValid()?perceptionFrameClock_.elapsed()/1000.0:0.0;
      frameText=QStringLiteral("FRAME: %1×%2 | age %3 s")
        .arg(perceptionFrame_.width()).arg(perceptionFrame_.height()).arg(age,0,'f',2);
      QImage canvas=perceptionFrame_.convertToFormat(QImage::Format_RGB32);
      QPainter painter(&canvas);
      painter.setRenderHint(QPainter::Antialiasing,true);
      const int cx=canvas.width()/2, cy=canvas.height()/2;
      QPen centerPen(QColor(40,220,240,210),2);
      painter.setPen(centerPen);
      painter.drawLine(cx,0,cx,canvas.height());
      painter.drawLine(0,cy,canvas.width(),cy);

      QString centerPath=currentId_.startsWith(QStringLiteral("4.3"))||
                         currentId_.startsWith(QStringLiteral("4.4"))||
                         currentId_.startsWith(QStringLiteral("4.5"))
        ?QStringLiteral("raw_detections.pallet_best_center_x_px")
        :QStringLiteral("raw_detections.best_center_x_px");
      QString centerYPath=centerPath;
      centerYPath.replace(QStringLiteral("center_x_px"),QStringLiteral("center_y_px"));
      const double tx=number(telemetry_->get(centerPath));
      const double ty=number(telemetry_->get(centerYPath));
      if(std::isfinite(tx)&&std::isfinite(ty)){
        QPen targetPen(QColor(255,205,55,235),3);
        painter.setPen(targetPen);
        painter.drawEllipse(QPointF(tx,ty),10,10);
        painter.drawLine(QPointF(tx-16,ty),QPointF(tx+16,ty));
        painter.drawLine(QPointF(tx,ty-16),QPointF(tx,ty+16));
      }
      const double visual=number(instantValue(QStringLiteral("derived.visual_error_px")));
      const double lat=number(instantValue(QStringLiteral("derived.alignment_lateral_error_cm")));
      const double yaw=number(telemetry_->get(QStringLiteral("alignment_state.error_yaw_deg")));
      QStringList hud;
      hud << spec().id;
      if(std::isfinite(visual))hud << QStringLiteral("err_px=%1").arg(visual,0,'f',1);
      if(std::isfinite(lat))hud << QStringLiteral("lat=%1cm").arg(lat,0,'f',2);
      if(std::isfinite(yaw))hud << QStringLiteral("yaw=%1deg").arg(yaw,0,'f',2);
      painter.setPen(Qt::white);
      painter.setBrush(QColor(0,0,0,150));
      QRect hudRect(8,8,std::min(560,canvas.width()-16),34);
      painter.drawRect(hudRect);
      painter.drawText(hudRect.adjusted(8,0,-4,0),Qt::AlignVCenter|Qt::AlignLeft,hud.join(QStringLiteral(" | ")));
      painter.end();
      const QSize targetSize=perceptionPreviewImage_->size().expandedTo(QSize(460,260));
      perceptionPreviewImage_->setPixmap(QPixmap::fromImage(canvas).scaled(
        targetSize,Qt::KeepAspectRatio,Qt::SmoothTransformation));
    }else{
      perceptionPreviewImage_->setText(QStringLiteral(
        "Menunggu /obstacle_detection/visualization\\n"
        "(fallback kompatibel: /camera/yolop/image_annotated)"));
    }
    perceptionPreviewStatus_->setText(QStringLiteral("%1 | DATA LIVE: %2/%3 | %4")
      .arg(frameText).arg(live).arg(total)
      .arg(recording_?QStringLiteral("RECORDING"):QStringLiteral("PREVIEW")));
  }

  QStringList commonPaths() const {
    if (subsystem_ == QStringLiteral("navigation")) return {
      // GNSS / quality (kept because the master GUI also supports the broader navigation stack).
      "gnss_quality.sat","gnss_quality.dop","gnss_quality.hacc_m","gnss_fix.lat","gnss_fix.lon","gnss_fix.alt","gnss_fix.status","gnss_fix.measurement_stamp_sec",
      // LiDAR: all scalar diagnostics emitted by the GUI bridge, including source timestamp and beam counts.
      "lidar.range_center_m","lidar.sample_angle_deg","lidar.sample_range_m","lidar.sample_intensity","lidar.sample_valid","lidar.valid_ratio_pct","lidar.dropout_pct","lidar.scan_rate_hz","lidar.period_ms","lidar.timestamp_jitter_ms","lidar.latency_ms","lidar.beam_count","lidar.valid_beam_count","lidar.message_count","lidar.publisher_count","lidar.frame_id","lidar.measurement_stamp_sec",
      "lidar_raw.range_center_m","lidar_raw.sample_angle_deg","lidar_raw.sample_range_m","lidar_raw.sample_intensity","lidar_raw.sample_valid","lidar_raw.valid_ratio_pct","lidar_raw.dropout_pct","lidar_raw.rate_hz","lidar_raw.period_ms","lidar_raw.timestamp_jitter_ms","lidar_raw.latency_ms","lidar_raw.beam_count","lidar_raw.valid_beam_count","lidar_raw.message_count","lidar_raw.publisher_count","lidar_raw.frame_id","lidar_raw.source_topic","lidar_raw.measurement_stamp_sec","lidar_raw.angle_min_rad","lidar_raw.angle_max_rad","lidar_raw.angle_increment_rad","lidar_raw.range_min_m","lidar_raw.range_max_m","lidar_raw.scan_time_s","lidar_raw.time_increment_s",
      "lidar_nav_raw.range_center_m","lidar_nav_raw.valid_ratio_pct","lidar_nav_raw.rate_hz","lidar_nav_raw.message_count","lidar_nav_raw.publisher_count","lidar_nav_raw.frame_id","lidar_nav_raw.measurement_stamp_sec",
      "lidar_nav.range_center_m","lidar_nav.valid_ratio_pct","lidar_nav.rate_hz","lidar_nav.message_count","lidar_nav.publisher_count","lidar_nav.frame_id","lidar_nav.measurement_stamp_sec",
      "lidar_safety.range_center_m","lidar_safety.valid_ratio_pct","lidar_safety.rate_hz","lidar_safety.message_count","lidar_safety.publisher_count","lidar_safety.frame_id","lidar_safety.measurement_stamp_sec",
      "lidar_driver_status.raw","lidar_safety_health.raw","lidar_safety_healthy",
      // IMU: fused message plus every component topic already published by imu_node.
      "imu.roll_rad","imu.pitch_rad","imu.yaw_rad","imu.qx","imu.qy","imu.qz","imu.qw","imu.gx","imu.gy","imu.gz","imu.ax","imu.ay","imu.az","imu.orientation_valid","imu.var_roll","imu.var_pitch","imu.var_yaw","imu.var_gx","imu.var_gy","imu.var_gz","imu.var_ax","imu.var_ay","imu.var_az","imu.rate_hz","imu.period_ms","imu.timestamp_jitter_ms","imu.latency_ms","imu.message_count","imu.publisher_count","imu.frame_id","imu.measurement_stamp_sec","imu_status.yaw_residual",
      "imu_gyro.gx","imu_gyro.gy","imu_gyro.gz","imu_gyro.var_gx","imu_gyro.var_gy","imu_gyro.var_gz","imu_gyro.rate_hz","imu_gyro.period_ms","imu_gyro.timestamp_jitter_ms","imu_gyro.latency_ms","imu_gyro.message_count","imu_gyro.publisher_count","imu_gyro.frame_id","imu_gyro.measurement_stamp_sec",
      "imu_accel.ax","imu_accel.ay","imu_accel.az","imu_accel.var_ax","imu_accel.var_ay","imu_accel.var_az","imu_accel.rate_hz","imu_accel.period_ms","imu_accel.timestamp_jitter_ms","imu_accel.latency_ms","imu_accel.message_count","imu_accel.publisher_count","imu_accel.frame_id","imu_accel.measurement_stamp_sec",
      "imu_euler.roll_rad","imu_euler.pitch_rad","imu_euler.yaw_rad","imu_euler.rate_hz","imu_euler.message_count","imu_euler.publisher_count","imu_euler.frame_id","imu_euler.measurement_stamp_sec",
      "imu_mag.mx","imu_mag.my","imu_mag.mz","imu_mag.rate_hz","imu_mag.message_count","imu_mag.publisher_count","imu_mag.frame_id","imu_mag.measurement_stamp_sec",
      "imu_mag_field.mx","imu_mag_field.my","imu_mag_field.mz","imu_mag_field.var_mx","imu_mag_field.var_my","imu_mag_field.var_mz","imu_mag_field.rate_hz","imu_mag_field.message_count","imu_mag_field.publisher_count","imu_mag_field.frame_id","imu_mag_field.measurement_stamp_sec","imu_driver_status.raw",
      // Odometry / EKF sources with pose, twist, covariance and measurement timestamps.
      "odom.x","odom.y","odom.yaw","odom.v","odom.vy","odom.w","odom.var_x","odom.var_y","odom.var_yaw","odom.rate_hz","odom.period_ms","odom.timestamp_jitter_ms","odom.latency_ms","odom.message_count","odom.publisher_count","odom.frame_id","odom.child_frame_id","odom.measurement_stamp_sec",
      "esc_odom.x","esc_odom.y","esc_odom.yaw","esc_odom.v","esc_odom.w","esc_odom.var_x","esc_odom.var_y","esc_odom.var_yaw","esc_odom.measurement_stamp_sec",
      "ekf_local.x","ekf_local.y","ekf_local.yaw","ekf_local.v","ekf_local.w","ekf_local.var_x","ekf_local.var_y","ekf_local.var_yaw","ekf_local.measurement_stamp_sec",
      "ekf_global.x","ekf_global.y","ekf_global.yaw","ekf_global.v","ekf_global.w","ekf_global.var_x","ekf_global.var_y","ekf_global.var_yaw","ekf_global.measurement_stamp_sec",
      "localization_state.map_x","localization_state.map_y","localization_state.yaw",
      // SLAM / AMCL scalars required to reproduce table metrics.
      "amcl.x","amcl.y","amcl.yaw","amcl.var_x","amcl.var_y","amcl.var_yaw","amcl.measurement_stamp_sec",
      "slam.width_cells","slam.height_cells","slam.resolution_m","slam.width_m","slam.height_m","slam.map_update_hz","slam.unknown_pct","slam.occupied_pct","slam.measurement_stamp_sec",
      // Commands, feedback and planner metadata used by end-to-end analysis.
      "cmd_nav.linear_x","cmd_nav.angular_z","cmd_autonomy_integrated.linear_x","cmd_autonomy_integrated.angular_z",
      "cmd_final.linear_x","cmd_final.angular_z","esc_drive_target","esc_drive_actual","esc_steer_target","esc_steer_actual","esc_yaw_rate","esc_kinematic_yaw_rate",
      "mppi_velocity_error","mppi_steering_error","mppi_yaw_error","nav_path.count","nav_path.length_m","nav_path.heading_variation_rad","nav_path.planning_latency_ms","nav_path.measurement_stamp_sec",
      "goal_pose.x","goal_pose.y","goal_pose.yaw","goal_state.state","goal_state.duration_s",
      // Derived values are stored alongside source values so exported raw CSV can be audited without replaying the GUI.
      "derived.cte_m","derived.path_heading_error_rad","derived.endpoint_error_m","derived.goal_yaw_error_rad",
      "derived.velocity_error_mps","derived.steering_error_rad","derived.yaw_error_rps",
      // Host resource evidence.
      "host.cpu_percent","host.ram_used_gb","host.ram_total_gb","host.temperature_c","host.gpu_percent"
    };
    if (subsystem_ == QStringLiteral("perception")) return {
      "perception_performance.fps","perception_performance.mean_ms","perception_performance.p95_ms","perception_performance.capture_dropped",
      "perception_performance.rviz_dropped","raw_detections.count","raw_detections.mean_confidence","raw_detections.max_confidence",
      "raw_detections.dynamic_count","raw_detections.static_count","raw_detections.floor_count","raw_detections.pallet_count","raw_detections.other_count",
      "raw_detections.best_confidence","raw_detections.best_center_x_px","raw_detections.best_center_y_px",
      "raw_detections.pallet_best_confidence","raw_detections.pallet_best_center_x_px","raw_detections.pallet_best_center_y_px",
      "obstacle_metrics.count","obstacle_metrics.nearest_forward_m","obstacle_metrics.nearest_left_m","obstacle_metrics.mean_confidence",
      "object_points.count","path_relevant_points.count","planning_relevant_points.count","drivable_boundary_points.count",
      "drivable_space.valid_rows","drivable_space.valid","lane_state.valid","lane_state.left_clearance_m","lane_state.right_clearance_m",
      "lane_state.center_error_m","lane_state.heading_error_rad","lane_state.state","camera_healthy","camera_health_state.mean_luma",
      "camera_health_state.stddev_luma","camera_health_state.mean_gradient","camera_health_state.width","camera_health_state.height",
      "perception_emergency","near_field_state.confidence","near_field_state.near_field_drivable_fraction",
      "alignment_state.state","alignment_state.state_text","alignment_state.pallet_detected","alignment_state.detection_stable",
      "alignment_state.confidence","alignment_state.depth_quality","alignment_state.error_lateral_m","alignment_state.error_yaw_deg",
      "alignment_state.pid_lateral_output","alignment_state.pid_yaw_output","alignment_state.desired_yaw_rate",
      "alignment_state.estimated_steering_deg","alignment_state.linear_velocity_cmd","alignment_state.angular_velocity_cmd",
      "alignment_state.steering_limit_active","alignment_state.safety_stop_active","alignment_state.data_valid",
      "alignment_state.lateral_within_tolerance","alignment_state.yaw_within_tolerance","alignment_state.steering_centered",
      "alignment_state.ready_for_insertion","trajectory_safety_state.speed_scale","trajectory_safety_state.decision",
      "derived.visual_error_px","derived.visual_error_normalized","derived.alignment_lateral_error_cm",
      "derived.obstacle_error_x_m","derived.obstacle_error_y_m","derived.obstacle_error_2d_m","host.gpu_percent","host.ram_used_gb","host.temperature_c"
    };
    return {
      "esc_steer_target","esc_steer_actual","esc_drive_target","esc_drive_actual","esc_yaw_rate","esc_kinematic_yaw_rate",
      "derived.steering_error_rad","derived.steering_target_deg","derived.steering_actual_deg","derived.target_rpm","derived.actual_rpm",
      "foc_telemetry.ia_a","foc_telemetry.ib_a","foc_telemetry.id_a","foc_telemetry.iq_a","foc_telemetry.iq_ref_a",
      "foc_telemetry.vd_v","foc_telemetry.vq_v","foc_telemetry.vbus_v","foc_telemetry.encoder_count","foc_telemetry.electrical_sector"
    };
  }
  QVariant instantValue(const QString &path) {
    if (path.startsWith(QStringLiteral("host."))) return hostMetrics_.value(path);
    if (!path.startsWith(QStringLiteral("derived."))) return telemetry_->get(path);
    if (path == QStringLiteral("derived.velocity_error_mps")) {
      const double target=number(telemetry_->get("esc_drive_target")), actual=number(telemetry_->get("esc_drive_actual"));
      return std::isfinite(target)&&std::isfinite(actual)?QVariant(target-actual):QVariant();
    }
    if (path == QStringLiteral("derived.steering_error_rad")) {
      const double target=number(telemetry_->get("esc_steer_target")), actual=number(telemetry_->get("esc_steer_actual"));
      return std::isfinite(target)&&std::isfinite(actual)?QVariant(normalizeAngle(target-actual)):QVariant();
    }
    if (path == QStringLiteral("derived.yaw_error_rps")) {
      const double direct=number(telemetry_->get("mppi_yaw_error"));
      if(std::isfinite(direct))return direct;
      const double target=number(telemetry_->get("cmd_final.angular_z")), actual=number(telemetry_->get("esc_yaw_rate"));
      return std::isfinite(target)&&std::isfinite(actual)?QVariant(target-actual):QVariant();
    }
    if (path == QStringLiteral("derived.steering_target_deg")) {
      const double v=number(telemetry_->get("esc_steer_target"));
      return std::isfinite(v)?QVariant(v*180.0/kPi):QVariant();
    }
    if (path == QStringLiteral("derived.steering_actual_deg")) {
      const double v=number(telemetry_->get("esc_steer_actual"));
      return std::isfinite(v)?QVariant(v*180.0/kPi):QVariant();
    }
    if (path == QStringLiteral("derived.target_rpm") || path == QStringLiteral("derived.actual_rpm")) {
      const QString statusKey = path.endsWith(QStringLiteral("target_rpm"))
      ? QStringLiteral("esc_status.right_target") : QStringLiteral("esc_status.right");
      const double measuredRpm = number(telemetry_->get(statusKey));
      if (std::isfinite(measuredRpm)) return measuredRpm;
      auto store=stores_.value("esc");
      if(!store)return {
      };
      const double maxRpm=number(store->get("esc_ackermann.ros__parameters.right_max_rpm",300.0),300.0);
      const double maxSpeed=number(store->get("esc_ackermann.ros__parameters.speed_max_mps",1.0),1.0);
      const double speed=number(telemetry_->get(path.endsWith("target_rpm")?"esc_drive_target":"esc_drive_actual"));
      return std::isfinite(speed)&&maxSpeed>0.0?QVariant(speed/maxSpeed*maxRpm):QVariant();
    }
    if (path.startsWith(QStringLiteral("derived.obstacle_error_"))) {
      const double gx=number(groundTruth(QStringLiteral("gt_x"))), gy=number(groundTruth(QStringLiteral("gt_y")));
      const double sx=number(telemetry_->get("obstacle_metrics.nearest_forward_m")), sy=number(telemetry_->get("obstacle_metrics.nearest_left_m"));
      if(!std::isfinite(gx)||!std::isfinite(gy)||!std::isfinite(sx)||!std::isfinite(sy))return {
      };
      if(path.endsWith("x_m"))return sx-gx;
      if(path.endsWith("y_m"))return sy-gy;
      return std::hypot(sx-gx,sy-gy);
    }
    if(path==QStringLiteral("derived.visual_error_px") ||
       path==QStringLiteral("derived.visual_error_normalized")){
      double center=number(telemetry_->get(QStringLiteral("raw_detections.pallet_best_center_x_px")));
      if(!std::isfinite(center))center=number(telemetry_->get(QStringLiteral("raw_detections.best_center_x_px")));
      const double width=number(telemetry_->get(QStringLiteral("camera_health_state.width")));
      if(!std::isfinite(center)||!std::isfinite(width)||width<=1.0)return {};
      const double error=center-width*0.5;
      if(path.endsWith(QStringLiteral("normalized")))return error/(width*0.5);
      return error;
    }
    if(path==QStringLiteral("derived.alignment_lateral_error_cm")){
      const double value=number(telemetry_->get(QStringLiteral("alignment_state.error_lateral_m")));
      return std::isfinite(value)?QVariant(value*100.0):QVariant();
    }
    return navigationDerived(path);
  }
  QVariant navigationDerived(const QString &path) const {
    const double x=number(telemetry_->get("amcl.x"),number(telemetry_->get("localization_state.map_x"),number(telemetry_->get("ekf_global.x"))));
    const double y=number(telemetry_->get("amcl.y"),number(telemetry_->get("localization_state.map_y"),number(telemetry_->get("ekf_global.y"))));
    const double yaw=number(telemetry_->get("amcl.yaw"),number(telemetry_->get("localization_state.yaw"),number(telemetry_->get("ekf_global.yaw"))));
    if(path==QStringLiteral("derived.amcl_pos_error_m")){
      const double gx=number(groundTruth(QStringLiteral("gt_x"))),gy=number(groundTruth(QStringLiteral("gt_y")));
      return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(gx)&&std::isfinite(gy)?QVariant(std::hypot(x-gx,y-gy)):QVariant();
    }
    if(path==QStringLiteral("derived.amcl_yaw_error_rad")){
      const double referenceDeg=number(groundTruth(QStringLiteral("gt_yaw_deg")));
      return std::isfinite(yaw)&&std::isfinite(referenceDeg)?QVariant(std::abs(normalizeAngle(yaw-referenceDeg*kPi/180.0))):QVariant();
    }
    if(path==QStringLiteral("derived.endpoint_error_m")){
      const double gx=number(telemetry_->get("goal_pose.x")),gy=number(telemetry_->get("goal_pose.y"));
      return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(gx)&&std::isfinite(gy)?QVariant(std::hypot(x-gx,y-gy)):QVariant();
    }
    if(path==QStringLiteral("derived.goal_yaw_error_rad")){
      const double gyaw=number(telemetry_->get("goal_pose.yaw"));
      return std::isfinite(yaw)&&std::isfinite(gyaw)?QVariant(std::abs(normalizeAngle(yaw-gyaw))):QVariant();
    }
    const QVariantList points=telemetry_->get("nav_path.points").toList();
    if(!std::isfinite(x)||!std::isfinite(y)||points.size()<2)return {
    };
    double best=std::numeric_limits<double>::infinity(),bestHeading=0.0;
    for(int i=1;
    i<points.size();
    ++i){
      const QVariantList a=points[i-1].toList(),b=points[i].toList();
      if(a.size()<2||b.size()<2)continue;
      const double ax=a[0].toDouble(),ay=a[1].toDouble(),bx=b[0].toDouble(),by=b[1].toDouble();
      const double dx=bx-ax,dy=by-ay,l2=dx*dx+dy*dy;
      if(l2<1e-12)continue;
      const double u=std::clamp(((x-ax)*dx+(y-ay)*dy)/l2,0.0,1.0);
      const double distance=std::hypot(x-(ax+u*dx),y-(ay+u*dy));
      if(distance<best){
        best=distance;
        bestHeading=std::atan2(dy,dx);
      }
    }
    if(path==QStringLiteral("derived.cte_m"))return std::isfinite(best)?QVariant(best):QVariant();
    if(path==QStringLiteral("derived.path_heading_error_rad"))return std::isfinite(yaw)?QVariant(std::abs(normalizeAngle(yaw-bestHeading))):QVariant();
    return {
    };
  }
  void toggleRecording() {
    if(recording_)stopRecording(false);
    else startRecording();
  }
  void startRecording() {
    if (!spec().recordable) return;
    saveTableState();
    session().rawRows.clear();
    session().liveSeries.clear();
    session().liveScatter.clear();
    elapsed_.restart();
    activeRecordingSummaryRow_ = -1;
    recording_=true;
    double rate=number(parameterValue(QStringLiteral("sample_rate")));
    if(!std::isfinite(rate) && stores_.contains("gui"))
      rate=number(stores_["gui"]->get("reporting.sample_rate_hz",5.0),5.0);
    rate=std::clamp(std::isfinite(rate)?rate:5.0,0.5,50.0);
    timer_->start(std::max(20,int(std::lround(1000.0/rate))));
    record_->setText(QStringLiteral("■ Stop + Tambahkan Ringkasan"));
    status_->setText(QStringLiteral("MEREKAM run ")+spec().id+QStringLiteral(" @ ")+QString::number(rate,'f',1)+QStringLiteral(" Hz"));
  }
  void stopRecording(bool switching) {
    recording_=false;
    record_->setText(QStringLiteral("● Mulai Rekam Run"));
    if(!switching&& !session().rawRows.isEmpty()){
      if(!appendSpecial41Rows()) updateRecordingSummaryRow();
      activeRecordingSummaryRow_ = -1;
    } else {
      activeRecordingSummaryRow_ = -1;
    }
    status_->setText(QStringLiteral("Run berhenti: %1 sampel mentah.").arg(session().rawRows.size()));
  }
  void captureSample() {
    QVariantMap row;
    row["time_iso"]=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    row["elapsed_s"]=(recording_ ? elapsed_.elapsed() : liveElapsed_.elapsed())/1000.0;
    row["subsystem"]=subsystem_;
    row["section_id"]=spec().id;
    row["table_name"]=currentTableCaption();
    row["variation"]=parameterText(QStringLiteral("variation"));
    row["condition"]=parameterText(QStringLiteral("condition"));
    // Persist every active experiment/YAML/ground-truth field into each raw
    // sample. This makes every run self-describing and prevents a CSV from
    // becoming detached from the parameters that produced it.
    for (const ExperimentParameterField &field : spec().parameterFields) {
      const QVariant value = parameterValue(field.key);
      if (value.isValid()) row[QStringLiteral("param.") + field.key] = value;
    }
    QStringList paths=commonPaths();
    for(auto it=spec().liveSeries.cbegin();
    it!=spec().liveSeries.cend();
    ++it)if(!paths.contains(it.value()))paths<<it.value();
    for(const QString&path:paths){
      QVariant value=instantValue(path);
      if(value.isValid())row[path]=value;
    }
    if(recording_) session().rawRows<<row;
    const double time=row["elapsed_s"].toDouble();
    for(auto it=spec().liveSeries.cbegin();
    it!=spec().liveSeries.cend();
    ++it){
      const double value=number(row.value(it.value()));
      if(std::isfinite(value))session().liveSeries[it.key()]<<QPointF(time,value);
    }
    // Keep the plot live before, during, and after a run. Bound the live
    // renderer buffer so leaving the GUI open cannot grow memory forever.
    for(auto it=spec().liveSeries.cbegin();
    it!=spec().liveSeries.cend();
    ++it){
      auto &series=session().liveSeries[it.key()];
      while(series.size()>1200)series.removeFirst();
    }
    // Live scatter buffer (e.g. 4.1.1 GNSS position scatter): rendered from
    // the moment the source topic arrives — before any Start Record. This
    // buffer is display-only; saved evidence still comes from rawRows.
    for (int gi = 0; gi < spec().graphs.size(); ++gi) {
      const ExperimentGraphSpec &g = spec().graphs.at(gi);
      if (g.type != QStringLiteral("scatter")) continue;
      const double x = number(row.value(g.xSeries));
      const double y = number(row.value(g.ySeries));
      if (std::isfinite(x) && std::isfinite(y)) {
        auto &pts = session().liveScatter[gi];
        pts << QPointF(x, y);
        while (pts.size() > 2000) pts.removeFirst();
      }
    }
    if(recording_ && !isSpecial41MultiRowLeaf() && (!summaryUpdateClock_.isValid() || summaryUpdateClock_.elapsed() >= 500)){
      updateRecordingSummaryRow();
      summaryUpdateClock_.restart();
    }
    if(plotMode_->currentIndex()==0)refreshGraphs();
    if(subsystem_==QStringLiteral("perception"))refreshPerceptionPreview();
  }
  QVector<double> values(const QString &key) const {
    QVector<double> out;
    for(const QVariantMap&r:sessions_.value(currentId_).rawRows){
      const double v=number(r.value(key));
      if(std::isfinite(v))out<<v;
    }
    return out;
  }
  QVariant aggregate(const QString &key,const QString &mode) const {
    const QVector<double> data=values(key);
    if(data.isEmpty())return {
    };
    if(mode=="last")return data.last();
    if(mode=="max")return *std::max_element(data.begin(),data.end());
    if(mode=="min")return *std::min_element(data.begin(),data.end());
    const double mean=std::accumulate(data.begin(),data.end(),0.0)/data.size();
    if(mode=="mean")return mean;
    double sum=0.0;
    if(mode=="std"){
      for(double v:data)sum+=(v-mean)*(v-mean);
      return std::sqrt(sum/data.size());
    }
    if(mode=="mae"){
      for(double v:data)sum+=std::abs(v);
      return sum/data.size();
    }
    for(double v:data)sum+=v*v;
    return std::sqrt(sum/data.size());
  }
  QVariant errorAggregate(const QString &key,double reference,const QString &mode,bool angular=false) const {
    if(!std::isfinite(reference))return {};
    const QVector<double> data=values(key);
    if(data.isEmpty())return {};
    QVector<double> errors;
    errors.reserve(data.size());
    for(double value:data)errors<<(angular?normalizeAngle(value-reference):value-reference);
    const double mean=std::accumulate(errors.begin(),errors.end(),0.0)/errors.size();
    if(mode==QStringLiteral("mean"))return mean;
    if(mode==QStringLiteral("mae")){
      double sum=0.0;for(double value:errors)sum+=std::abs(value);return sum/errors.size();
    }
    if(mode==QStringLiteral("std")){
      double sum=0.0;for(double value:errors)sum+=(value-mean)*(value-mean);return std::sqrt(sum/errors.size());
    }
    double sum=0.0;for(double value:errors)sum+=value*value;return std::sqrt(sum/errors.size());
  }
  QVariant percentTrue(const QString &key) const {
    int valid=0,yes=0;
    for(const QVariantMap&r:sessions_.value(currentId_).rawRows){
      if(!r.contains(key))continue;
      ++valid;
      if(r.value(key).toBool())++yes;
    }
    return valid?QVariant(100.0*yes/valid):QVariant();
  }
  QVariant lastText(const QString &key) const {
    const auto&rows=sessions_.value(currentId_).rawRows;
    for(auto it=rows.crbegin();
    it!=rows.crend();
    ++it)if(it->contains(key))return it->value(key);
    return {
    };
  }
  bool isSpecial41MultiRowLeaf() const {
    // BAB IV TA terbaru: 4.1.1 komunikasi dan 4.1.3/4.1.4 raw data
    // membutuhkan beberapa baris sampel; 4.1.2 LiDAR adalah ringkasan akurasi
    // per jarak referensi sehingga satu Run menghasilkan satu baris statistik.
    return currentId_==QStringLiteral("4.1.1") ||
           currentId_==QStringLiteral("4.1.3") || currentId_==QStringLiteral("4.1.4") ||
           currentId_==QStringLiteral("4.1.5");
  }
  QString sensorRatePath(const QString &prefix) const {
    return prefix==QStringLiteral("lidar")?QStringLiteral("lidar.scan_rate_hz"):prefix+QStringLiteral(".rate_hz");
  }
  QString sensorTargetKey(const QString &prefix) const {
    if(prefix.startsWith(QStringLiteral("lidar")))return QStringLiteral("target_lidar_rate_hz");
    if(prefix.startsWith(QStringLiteral("imu")))return QStringLiteral("target_imu_rate_hz");
    return QStringLiteral("target_odom_rate_hz");
  }
  QString preferredLidar41Prefix() const {
    const QStringList prefixes={QStringLiteral("lidar_raw"),QStringLiteral("lidar"),QStringLiteral("lidar_nav_raw"),QStringLiteral("lidar_nav"),QStringLiteral("lidar_safety")};
    const auto &rows=sessions_.value(currentId_).rawRows;
    for(const QString &prefix:prefixes){
      const QString countKey=prefix+QStringLiteral(".message_count");
      const QString stampKey=prefix+QStringLiteral(".measurement_stamp_sec");
      for(auto it=rows.crbegin();it!=rows.crend();++it){
        if(it->contains(countKey)||it->contains(stampKey))return prefix;
      }
    }
    return QStringLiteral("lidar");
  }
  QString preferredImuYawPath() const {
    const auto &rows=sessions_.value(currentId_).rawRows;
    for(auto it=rows.crbegin();it!=rows.crend();++it){
      if(it->contains(QStringLiteral("imu.yaw_rad")))return QStringLiteral("imu.yaw_rad");
      if(it->contains(QStringLiteral("imu_euler.yaw_rad")))return QStringLiteral("imu_euler.yaw_rad");
    }
    return QStringLiteral("imu.yaw_rad");
  }
  QVariant messageDropoutPct(const QString &prefix) const {
    const double target=number(parameterValue(sensorTargetKey(prefix)));
    const auto &rows=sessions_.value(currentId_).rawRows;
    if(!std::isfinite(target)||target<=0.0||rows.size()<2)return {};
    const double t0=number(rows.first().value(QStringLiteral("elapsed_s")));
    const double t1=number(rows.last().value(QStringLiteral("elapsed_s")));
    const double c0=number(rows.first().value(prefix+QStringLiteral(".message_count")));
    const double c1=number(rows.last().value(prefix+QStringLiteral(".message_count")));
    const double duration=t1-t0;
    if(!std::isfinite(duration)||duration<=0.0||!std::isfinite(c0)||!std::isfinite(c1))return {};
    const double expected=target*duration;
    const double received=std::max(0.0,c1-c0);
    if(expected<=0.0)return {};
    return std::clamp(100.0*(expected-received)/expected,0.0,100.0);
  }
  QString communicationStatus(const QString &prefix) const {
    const QVariant rate=aggregate(sensorRatePath(prefix),QStringLiteral("mean"));
    const QVariant publishers=lastText(prefix+QStringLiteral(".publisher_count"));
    bool pubOk=false;const double pub=number(publishers);
    pubOk=std::isfinite(pub)&&pub>0.0;
    return rate.isValid()&&pubOk?QStringLiteral("Aktif"):QStringLiteral("Periksa");
  }
  static QString metricText(const QVariant &value,const QString &suffix=QString()) {
    if(!value.isValid())return QStringLiteral("-");
    bool ok=false;const double v=value.toDouble(&ok);
    return ok&&std::isfinite(v)?QString::number(v,'f',3)+suffix:value.toString();
  }
  QVector<QVariantMap> buildSpecial41Rows() const {
    QVector<QVariantMap> out;
    const auto &rows=sessions_.value(currentId_).rawRows;
    if(rows.isEmpty())return out;
    if(currentId_==QStringLiteral("4.1.1")){
      struct SensorDef{const char*label;const char*topic;const char*prefix;const char*targetKey;const char*frameFallback;};
      const SensorDef defs[]={{"LiDAR","/scan","lidar","target_lidar_rate_hz","laser_frame"},
                              {"IMU","/imu/data","imu","target_imu_rate_hz","imu_link"},
                              {"Odometri","/odom","odom","target_odom_rate_hz","odom"}};
      for(const auto &d:defs){
        QString prefix=QString::fromUtf8(d.prefix);
        if(prefix==QStringLiteral("lidar"))prefix=preferredLidar41Prefix();
        QVariantMap row;
        row[QStringLiteral("Sensor")]=QString::fromUtf8(d.label);
        row[QStringLiteral("Topic")]=QString::fromUtf8(d.topic);
        QVariant frame=lastText(prefix+QStringLiteral(".frame_id"));
        row[QStringLiteral("Frame")]=frame.isValid()&&!frame.toString().isEmpty()?frame:QVariant(QString::fromUtf8(d.frameFallback));
        row[QStringLiteral("Target Rate")]=parameterValue(QString::fromUtf8(d.targetKey));
        row[QStringLiteral("Actual Rate")]=aggregate(sensorRatePath(prefix),QStringLiteral("mean"));
        row[QStringLiteral("Jitter")]=aggregate(prefix+QStringLiteral(".timestamp_jitter_ms"),QStringLiteral("mean"));
        row[QStringLiteral("Dropout")]=messageDropoutPct(prefix);
        row[QStringLiteral("Latency")]=aggregate(prefix+QStringLiteral(".latency_ms"),QStringLiteral("mean"));
        row[QStringLiteral("Status")]=communicationStatus(prefix);
        out<<row;
      }
      return out;
    }
    auto sampledIndices=[&](int wanted){
      QVector<int> indices;
      const int count=std::min(wanted,rows.size());
      if(count<=0)return indices;
      if(count==1){indices<<rows.size()-1;return indices;}
      for(int i=0;i<count;++i)indices<<int(std::lround(double(i)*(rows.size()-1)/double(count-1)));
      return indices;
    };
    if(currentId_==QStringLiteral("4.1.2")){
      auto lidarValue=[](const QVariantMap &raw,const QString &suffix)->QVariant{
        const QStringList prefixes={QStringLiteral("lidar_raw"),QStringLiteral("lidar"),QStringLiteral("lidar_nav_raw"),QStringLiteral("lidar_nav"),QStringLiteral("lidar_safety")};
        for(const QString &prefix:prefixes){
          const QVariant value=raw.value(prefix+QStringLiteral(".")+suffix);
          if(value.isValid()&&!value.isNull())return value;
        }
        return {};
      };
      int sample=1;
      for(int idx:sampledIndices(10)){
        const QVariantMap &raw=rows.at(idx);QVariantMap row;
        row[QStringLiteral("Sampel")]=sample++;
        row[QStringLiteral("Sudut (deg)")]=lidarValue(raw,QStringLiteral("sample_angle_deg"));
        row[QStringLiteral("Range (m)")]=lidarValue(raw,QStringLiteral("sample_range_m"));
        row[QStringLiteral("Intensity")]=lidarValue(raw,QStringLiteral("sample_intensity"));
        const QVariant valid=lidarValue(raw,QStringLiteral("sample_valid"));
        row[QStringLiteral("Valid/Invalid")]=valid.isValid()?(valid.toBool()?QStringLiteral("Valid"):QStringLiteral("Invalid")):QVariant();
        row[QStringLiteral("Timestamp")]=lidarValue(raw,QStringLiteral("measurement_stamp_sec"));
        out<<row;
      }
      return out;
    }
    if(currentId_==QStringLiteral("4.1.3")){
      int sample=1;
      for(int idx:sampledIndices(8)){
        const QVariantMap &raw=rows.at(idx);QVariantMap row;
        row[QStringLiteral("Sampel")]=sample++;
        row[QStringLiteral("qx")]=raw.value(QStringLiteral("imu.qx"));
        row[QStringLiteral("qy")]=raw.value(QStringLiteral("imu.qy"));
        row[QStringLiteral("qz")]=raw.value(QStringLiteral("imu.qz"));
        row[QStringLiteral("qw")]=raw.value(QStringLiteral("imu.qw"));
        double yaw=number(raw.value(QStringLiteral("imu.yaw_rad")));
        if(!std::isfinite(yaw))yaw=number(raw.value(QStringLiteral("imu_euler.yaw_rad")));
        if(std::isfinite(yaw))row[QStringLiteral("Yaw (deg)")]=yaw*180.0/kPi;
        QVariant gz=raw.value(QStringLiteral("imu.gz"));
        if(!gz.isValid()||!std::isfinite(number(gz)))gz=raw.value(QStringLiteral("imu_gyro.gz"));
        row[QStringLiteral("Angular z")]=gz;
        QVariant stamp=raw.value(QStringLiteral("imu.measurement_stamp_sec"));
        if(!stamp.isValid())stamp=raw.value(QStringLiteral("imu_euler.measurement_stamp_sec"));
        row[QStringLiteral("Timestamp")]=stamp;
        out<<row;
      }
      return out;
    }
    if(currentId_==QStringLiteral("4.1.4")){
      int sample=1;
      for(int idx:sampledIndices(8)){
        const QVariantMap &raw=rows.at(idx);QVariantMap row;
        row[QStringLiteral("Sampel")]=sample++;
        row[QStringLiteral("Timestamp")]=raw.value(QStringLiteral("odom.measurement_stamp_sec"));
        row[QStringLiteral("vx")]=raw.value(QStringLiteral("odom.v"));
        row[QStringLiteral("vy")]=raw.value(QStringLiteral("odom.vy"));
        row[QStringLiteral("wz")]=raw.value(QStringLiteral("odom.w"));
        row[QStringLiteral("Frame")]=raw.value(QStringLiteral("odom.frame_id"));
        out<<row;
      }
      return out;
    }
    if(currentId_==QStringLiteral("4.1.5")){
      const QString condition=parameterText(QStringLiteral("condition")).trimmed().isEmpty()?QStringLiteral("Diam"):parameterText(QStringLiteral("condition")).trimmed();
      auto syncRow=[&](const QString &sensor,const QString &prefix){
        QVariantMap row;
        row[QStringLiteral("Kondisi")]=condition;
        row[QStringLiteral("Sensor")]=sensor;
        row[QStringLiteral("Frequency")]=aggregate(sensorRatePath(prefix),QStringLiteral("mean"));
        row[QStringLiteral("Mean Timestamp Gap")]=aggregate(prefix+QStringLiteral(".period_ms"),QStringLiteral("mean"));
        row[QStringLiteral("Max Gap")]=aggregate(prefix+QStringLiteral(".period_ms"),QStringLiteral("max"));
        row[QStringLiteral("Jitter")]=aggregate(prefix+QStringLiteral(".timestamp_jitter_ms"),QStringLiteral("mean"));
        row[QStringLiteral("Dropout")]=messageDropoutPct(prefix);
        row[QStringLiteral("Status")]=communicationStatus(prefix);
        return row;
      };
      if(condition.contains(QStringLiteral("diam"),Qt::CaseInsensitive)){
        out<<syncRow(QStringLiteral("LiDAR"),QStringLiteral("lidar"));
        out<<syncRow(QStringLiteral("IMU"),QStringLiteral("imu"));
        out<<syncRow(QStringLiteral("Odom"),QStringLiteral("odom"));
      }else{
        const QVariantMap l=syncRow(QStringLiteral("LiDAR"),QStringLiteral("lidar"));
        const QVariantMap i=syncRow(QStringLiteral("IMU"),QStringLiteral("imu"));
        const QVariantMap o=syncRow(QStringLiteral("Odom"),QStringLiteral("odom"));
        QVariantMap row;
        row[QStringLiteral("Kondisi")]=condition;
        row[QStringLiteral("Sensor")]=QStringLiteral("LiDAR/IMU/Odom");
        for(const QString &column:{QStringLiteral("Frequency"),QStringLiteral("Mean Timestamp Gap"),QStringLiteral("Max Gap"),QStringLiteral("Jitter"),QStringLiteral("Dropout")})
          row[column]=QStringLiteral("L:%1 | I:%2 | O:%3").arg(metricText(l.value(column)),metricText(i.value(column)),metricText(o.value(column)));
        row[QStringLiteral("Status")]=(l.value(QStringLiteral("Status")).toString()==QStringLiteral("Aktif")&&i.value(QStringLiteral("Status")).toString()==QStringLiteral("Aktif")&&o.value(QStringLiteral("Status")).toString()==QStringLiteral("Aktif"))?QStringLiteral("Aktif"):QStringLiteral("Periksa");
        out<<row;
      }
      return out;
    }
    return out;
  }
  bool appendSpecial41Rows(){
    if(!isSpecial41MultiRowLeaf())return false;
    const QVector<QVariantMap> rows=buildSpecial41Rows();
    if(rows.isEmpty())return true;
    loading_=true;
    for(const QVariantMap &row:rows){session().summaryRows<<row;insertRow(row);}
    loading_=false;
    table_->resizeColumnsToContents();
    refreshSummaryPlotIfNeeded();
    status_->setText(QStringLiteral("Data 4.1 ditambahkan dari sampel ROS aktual. Sel kosong tetap dapat dilengkapi manual sesuai metode ukur."));
    return true;
  }
  QVariant responseMetric(const QString &metric) const {
    const auto &rows=sessions_.value(currentId_).rawRows;
    if(rows.size()<3)return {
    };
    QVector<double>time,target,actual;
    const bool rawOdomVelocityResponse = subsystem_ == QStringLiteral("navigation") && currentId_ == QStringLiteral("4.1.4c");
    const bool velocityResponse = subsystem_ == QStringLiteral("navigation") && (currentId_ == QStringLiteral("4.4.3") || rawOdomVelocityResponse);
    const QString actualKey = velocityResponse ? QStringLiteral("odom.v") : QStringLiteral("esc_steer_actual");
    const QString targetKey = velocityResponse ? QStringLiteral("esc_drive_target") : QStringLiteral("esc_steer_target");
    const double rawOdomTarget=number(groundTruth(QStringLiteral("gt_vx_mps")));
    for(const auto&r:rows){
      double t=number(r.value("elapsed_s")),a=number(r.value(actualKey)),g=rawOdomVelocityResponse?rawOdomTarget:number(r.value(targetKey));
      if(std::isfinite(t)&&std::isfinite(a)&&std::isfinite(g)){
        time<<t;
        actual<<a;
        target<<g;
      }
    }
    if(time.size()<3)return {
    };
    const int tail=std::max(1,time.size()/10);
    double finalTarget=0.0,finalActual=0.0;
    for(int i=time.size()-tail;
    i<time.size();
    ++i){
      finalTarget+=target[i];
      finalActual+=actual[i];
    }
    finalTarget/=tail;
    finalActual/=tail;
    const double initial=actual.first(),delta=finalTarget-initial;
    if(std::abs(delta)<1e-6)return {
    };
    if(metric=="ess")return velocityResponse ? QVariant(finalTarget-finalActual)
                                             : QVariant((finalTarget-finalActual)*180.0/kPi);
    if(metric=="overshoot"){
      const double peak=delta>0?*std::max_element(actual.begin(),actual.end()):*std::min_element(actual.begin(),actual.end());
      return std::max(0.0,(peak-finalTarget)/delta*100.0);
    }
    auto crossing=[&](double ratio){
      const double threshold=initial+ratio*delta;
      for(int i=0;
      i<actual.size();
      ++i)if((delta>0&&actual[i]>=threshold)||(delta<0&&actual[i]<=threshold))return time[i];
      return std::numeric_limits<double>::quiet_NaN();
    };
    if(metric=="rise"){
      const double t10=crossing(.1),t90=crossing(.9);
      return std::isfinite(t10)&&std::isfinite(t90)?QVariant(t90-t10):QVariant();
    }
    if(metric=="settling"){
      const double minimumBand = velocityResponse ? 0.005 : 0.25*kPi/180.0;
      const double band=std::max(std::abs(delta)*.02,minimumBand);
      int lastOutside=-1;
      for(int i=0;
      i<actual.size();
      ++i)if(std::abs(actual[i]-finalTarget)>band)lastOutside=i;
      return lastOutside+1<time.size()?QVariant(time[std::max(0,lastOutside+1)]):QVariant();
    }
    if(metric=="gain"){
      const auto [tmin,tmax]=std::minmax_element(target.begin(),target.end());
      const auto [amin,amax]=std::minmax_element(actual.begin(),actual.end());
      const double input=*tmax-*tmin;
      return input>1e-9?QVariant((*amax-*amin)/input):QVariant();
    }
    return {
    };
  }
  QVariant summaryValue(const QString &column) const {
    QString key=column.toLower();
    key.replace('\\',' ');
    key=key.simplified();
    const QString variant=parameterText(QStringLiteral("variation")),condition=parameterText(QStringLiteral("condition"));
    const double gtDistance=number(groundTruth(QStringLiteral("gt_distance_m")));
    const double gtYawDeg=number(groundTruth(QStringLiteral("gt_yaw_deg")));
    const double gtYaw=std::isfinite(gtYawDeg)?gtYawDeg*kPi/180.0:std::numeric_limits<double>::quiet_NaN();
    const double gtVx=number(groundTruth(QStringLiteral("gt_vx_mps")));
    const auto degrees=[](const QVariant &value)->QVariant{
      bool ok=false;const double radians=value.toDouble(&ok);
      return ok&&std::isfinite(radians)?QVariant(radians*180.0/kPi):QVariant();
    };
    // BAB IV 4.1 terbaru: komunikasi + raw LiDAR/IMU/odometri.  Dispatch
    // dibuat sebelum aturan BAB IV lama agar kolom bernama sama tidak tertukar.
    if(currentId_.startsWith(QStringLiteral("4.1"))){
      const auto availableStatus=[&]()->QVariant{
        return sessions_.value(currentId_).rawRows.isEmpty()?QVariant():QVariant(QStringLiteral("Data tersedia"));
      };
      if(key==QStringLiteral("kondisi"))return condition;
      if(currentId_==QStringLiteral("4.1.2")){
        const QString prefix=preferredLidar41Prefix();
        if(key.contains(QStringLiteral("jarak ref")))return std::isfinite(gtDistance)?QVariant(gtDistance):QVariant();
        if(key==QStringLiteral("mean lidar (m)"))return aggregate(prefix+QStringLiteral(".range_center_m"),QStringLiteral("mean"));
        if(key==QStringLiteral("error mean (m)"))return errorAggregate(prefix+QStringLiteral(".range_center_m"),gtDistance,QStringLiteral("mean"));
        if(key==QStringLiteral("mae (m)"))return errorAggregate(prefix+QStringLiteral(".range_center_m"),gtDistance,QStringLiteral("mae"));
        if(key==QStringLiteral("rmse (m)"))return errorAggregate(prefix+QStringLiteral(".range_center_m"),gtDistance,QStringLiteral("rmse"));
        if(key==QStringLiteral("std dev (m)"))return aggregate(prefix+QStringLiteral(".range_center_m"),QStringLiteral("std"));
        if(key.contains(QStringLiteral("valid ratio")))return aggregate(prefix+QStringLiteral(".valid_ratio_pct"),QStringLiteral("mean"));
        if(key==QStringLiteral("status"))return availableStatus();
      }
      if(currentId_==QStringLiteral("4.1.1b")){
        if(key==QStringLiteral("lidar rate"))return aggregate(sensorRatePath(preferredLidar41Prefix()),QStringLiteral("mean"));
        if(key==QStringLiteral("imu rate"))return aggregate(QStringLiteral("imu.rate_hz"),QStringLiteral("mean"));
        if(key==QStringLiteral("odom rate"))return aggregate(QStringLiteral("odom.rate_hz"),QStringLiteral("mean"));
        if(key==QStringLiteral("lidar dropout"))return messageDropoutPct(preferredLidar41Prefix());
        if(key==QStringLiteral("imu dropout"))return messageDropoutPct(QStringLiteral("imu"));
        if(key==QStringLiteral("odom dropout"))return messageDropoutPct(QStringLiteral("odom"));
        if(key==QStringLiteral("catatan"))return QVariant();
      }
      if(currentId_==QStringLiteral("4.1.2b")){
        if(key.contains(QStringLiteral("jarak ref")))return std::isfinite(gtDistance)?QVariant(gtDistance):QVariant();
        if(key==QStringLiteral("mean lidar (m)"))return aggregate(preferredLidar41Prefix()+QStringLiteral(".range_center_m"),QStringLiteral("mean"));
        if(key==QStringLiteral("min"))return aggregate(preferredLidar41Prefix()+QStringLiteral(".range_center_m"),QStringLiteral("min"));
        if(key==QStringLiteral("max"))return aggregate(preferredLidar41Prefix()+QStringLiteral(".range_center_m"),QStringLiteral("max"));
        if(key==QStringLiteral("error mean"))return errorAggregate(preferredLidar41Prefix()+QStringLiteral(".range_center_m"),gtDistance,QStringLiteral("mean"));
        if(key==QStringLiteral("mae"))return errorAggregate(preferredLidar41Prefix()+QStringLiteral(".range_center_m"),gtDistance,QStringLiteral("mae"));
        if(key==QStringLiteral("rmse"))return errorAggregate(preferredLidar41Prefix()+QStringLiteral(".range_center_m"),gtDistance,QStringLiteral("rmse"));
        if(key==QStringLiteral("std dev"))return aggregate(preferredLidar41Prefix()+QStringLiteral(".range_center_m"),QStringLiteral("std"));
        if(key==QStringLiteral("valid ratio"))return aggregate(preferredLidar41Prefix()+QStringLiteral(".valid_ratio_pct"),QStringLiteral("mean"));
      }
      if(currentId_==QStringLiteral("4.1.2c")){
        if(key==QStringLiteral("rmse range"))return errorAggregate(preferredLidar41Prefix()+QStringLiteral(".range_center_m"),gtDistance,QStringLiteral("rmse"));
        if(key==QStringLiteral("std dev"))return aggregate(preferredLidar41Prefix()+QStringLiteral(".range_center_m"),QStringLiteral("std"));
        if(key==QStringLiteral("valid ratio"))return aggregate(preferredLidar41Prefix()+QStringLiteral(".valid_ratio_pct"),QStringLiteral("mean"));
        if(key==QStringLiteral("dropout"))return messageDropoutPct(preferredLidar41Prefix());
        if(key==QStringLiteral("scan rate"))return aggregate(sensorRatePath(preferredLidar41Prefix()),QStringLiteral("mean"));
        if(key==QStringLiteral("status"))return availableStatus();
      }
      if(currentId_==QStringLiteral("4.1.2d")){
        if(key==QStringLiteral("vx agv (m/s)"))return std::isfinite(gtVx)?QVariant(gtVx):QVariant();
        if(key==QStringLiteral("effective scan rate"))return aggregate(sensorRatePath(preferredLidar41Prefix()),QStringLiteral("mean"));
        if(key==QStringLiteral("dropout"))return messageDropoutPct(preferredLidar41Prefix());
        if(key==QStringLiteral("valid ratio"))return aggregate(preferredLidar41Prefix()+QStringLiteral(".valid_ratio_pct"),QStringLiteral("mean"));
        if(key==QStringLiteral("timestamp jitter"))return aggregate(preferredLidar41Prefix()+QStringLiteral(".timestamp_jitter_ms"),QStringLiteral("mean"));
        if(key==QStringLiteral("rmse range"))return errorAggregate(preferredLidar41Prefix()+QStringLiteral(".range_center_m"),gtDistance,QStringLiteral("rmse"));
        if(key==QStringLiteral("status"))return availableStatus();
      }
      if(currentId_==QStringLiteral("4.1.3b")){
        if(key==QStringLiteral("yaw ref"))return std::isfinite(gtYawDeg)?QVariant(gtYawDeg):QVariant();
        if(key==QStringLiteral("yaw mean imu"))return degrees(aggregate(preferredImuYawPath(),QStringLiteral("mean")));
        if(key==QStringLiteral("error"))return degrees(errorAggregate(preferredImuYawPath(),gtYaw,QStringLiteral("mean"),true));
        if(key==QStringLiteral("|error|"))return degrees(errorAggregate(preferredImuYawPath(),gtYaw,QStringLiteral("mae"),true));
        if(key==QStringLiteral("std dev"))return degrees(aggregate(preferredImuYawPath(),QStringLiteral("std")));
        if(key==QStringLiteral("rmse"))return degrees(errorAggregate(preferredImuYawPath(),gtYaw,QStringLiteral("rmse"),true));
        if(key==QStringLiteral("status"))return availableStatus();
      }
      if(currentId_==QStringLiteral("4.1.3c")){
        const QVector<double> yaw=values(preferredImuYawPath());
        if(key==QStringLiteral("durasi"))return parameterValue(QStringLiteral("duration"));
        if(key==QStringLiteral("yaw awal"))return yaw.isEmpty()?QVariant():QVariant(yaw.first()*180.0/kPi);
        if(key==QStringLiteral("yaw akhir"))return yaw.isEmpty()?QVariant():QVariant(yaw.last()*180.0/kPi);
        if(key==QStringLiteral("drift")||key==QStringLiteral("drift/min")){
          if(yaw.size()<2)return {};
          const double driftDeg=normalizeAngle(yaw.last()-yaw.first())*180.0/kPi;
          if(key==QStringLiteral("drift"))return driftDeg;
          const auto &rr=sessions_.value(currentId_).rawRows;
          const double seconds=rr.size()>1?number(rr.last().value(QStringLiteral("elapsed_s")))-number(rr.first().value(QStringLiteral("elapsed_s"))):std::numeric_limits<double>::quiet_NaN();
          return std::isfinite(seconds)&&seconds>0.0?QVariant(driftDeg/(seconds/60.0)):QVariant();
        }
        if(key==QStringLiteral("std dev"))return degrees(aggregate(preferredImuYawPath(),QStringLiteral("std")));
      }
      if(currentId_==QStringLiteral("4.1.3d")){
        if(key==QStringLiteral("mean shift yaw")){
          const QVector<double> data=values(preferredImuYawPath());
          if(data.isEmpty())return {};const double first=data.first();double sum=0.0;for(double value:data)sum+=normalizeAngle(value-first);
          return sum/data.size()*180.0/kPi;
        }
        if(key==QStringLiteral("std dev yaw"))return degrees(aggregate(preferredImuYawPath(),QStringLiteral("std")));
        if(key==QStringLiteral("spike maksimum")){
          const QVector<double> data=values(preferredImuYawPath());
          if(data.isEmpty())return {};const double first=data.first();double peak=0.0;for(double value:data)peak=std::max(peak,std::abs(normalizeAngle(value-first)));
          return peak*180.0/kPi;
        }
        if(key==QStringLiteral("status"))return availableStatus();
      }
      if(currentId_==QStringLiteral("4.1.4b")){
        if(key==QStringLiteral("v_ref (m/s)"))return std::isfinite(gtVx)?QVariant(gtVx):QVariant();
        if(key==QStringLiteral("vx mean"))return aggregate(QStringLiteral("odom.v"),QStringLiteral("mean"));
        if(key==QStringLiteral("error"))return errorAggregate(QStringLiteral("odom.v"),gtVx,QStringLiteral("mean"));
        if(key==QStringLiteral("error relatif")){
          const QVariant e=errorAggregate(QStringLiteral("odom.v"),gtVx,QStringLiteral("mean"));
          return e.isValid()&&std::abs(gtVx)>1e-9?QVariant(100.0*e.toDouble()/std::abs(gtVx)):QVariant();
        }
        if(key==QStringLiteral("rmse"))return errorAggregate(QStringLiteral("odom.v"),gtVx,QStringLiteral("rmse"));
        if(key==QStringLiteral("std dev"))return aggregate(QStringLiteral("odom.v"),QStringLiteral("std"));
        if(key==QStringLiteral("status"))return availableStatus();
      }
      if(currentId_==QStringLiteral("4.1.4c")){
        if(key==QStringLiteral("step kecepatan"))return variant.isEmpty()?QVariant(condition):QVariant(variant);
        if(key==QStringLiteral("rise time"))return responseMetric(QStringLiteral("rise"));
        if(key==QStringLiteral("overshoot"))return responseMetric(QStringLiteral("overshoot"));
        if(key==QStringLiteral("settling time"))return responseMetric(QStringLiteral("settling"));
        if(key==QStringLiteral("steady-state error"))return responseMetric(QStringLiteral("ess"));
        if(key==QStringLiteral("status"))return availableStatus();
      }
      if(key==QStringLiteral("status"))return availableStatus();
    }
    // NAVIGATION BAB IV terbaru (TA_Ernanta_Revisi_Final.docx).
    // Dispatch ini sengaja diletakkan sebelum aturan katalog lama agar nomor
    // 4.2/4.3/4.4/4.5 tidak tertukar dengan struktur BAB IV sebelumnya.
    if(subsystem_==QStringLiteral("navigation") && currentId_.startsWith(QStringLiteral("4.2"))){
      if(key==QStringLiteral("map"))return parameterText(QStringLiteral("map_name")).isEmpty()?QVariant(variant):QVariant(parameterText(QStringLiteral("map_name")));
      if(key.contains(QStringLiteral("resolution")))return aggregate(QStringLiteral("slam.resolution_m"),QStringLiteral("last"));
      if(key==QStringLiteral("width (m)"))return aggregate(QStringLiteral("slam.width_m"),QStringLiteral("last"));
      if(key==QStringLiteral("height (m)"))return aggregate(QStringLiteral("slam.height_m"),QStringLiteral("last"));
      if(key.contains(QStringLiteral("unknown")))return aggregate(QStringLiteral("slam.unknown_pct"),QStringLiteral("mean"));
      if(key.contains(QStringLiteral("occupied")))return aggregate(QStringLiteral("slam.occupied_pct"),QStringLiteral("mean"));
      if(key.contains(QStringLiteral("map update")))return aggregate(QStringLiteral("slam.map_update_hz"),QStringLiteral("mean"));
      if(key.contains(QStringLiteral("cpu mean")))return aggregate(QStringLiteral("host.cpu_percent"),QStringLiteral("mean"));
      if(key.contains(QStringLiteral("ram mean")))return aggregate(QStringLiteral("host.ram_used_gb"),QStringLiteral("mean"));
      if(key==QStringLiteral("error loop closure (m)"))return parameterValue(QStringLiteral("loop_error_m"));
      if(key==QStringLiteral("error yaw closure (deg)"))return parameterValue(QStringLiteral("loop_yaw_error_deg"));
      if(key==QStringLiteral("titik/dimensi referensi"))return parameterText(QStringLiteral("reference_name"));
      if(key==QStringLiteral("fisik (m)"))return parameterValue(QStringLiteral("reference_dimension_m"));
      if(key==QStringLiteral("map (m)"))return parameterValue(QStringLiteral("map_dimension_m"));
      if(key==QStringLiteral("error (m)")){
        const double physical=number(parameterValue(QStringLiteral("reference_dimension_m")));
        const double mapped=number(parameterValue(QStringLiteral("map_dimension_m")));
        return std::isfinite(physical)&&std::isfinite(mapped)?QVariant(mapped-physical):QVariant();
      }
      if(key==QStringLiteral("map terpilih"))return parameterText(QStringLiteral("selected_map"));
      if(key==QStringLiteral("frame map"))return QStringLiteral("map");
      if(key==QStringLiteral("status"))return sessions_.value(currentId_).rawRows.isEmpty()?QVariant():QVariant(QStringLiteral("Data tersedia"));
    }
    if(subsystem_==QStringLiteral("navigation") && currentId_.startsWith(QStringLiteral("4.3"))){
      const double gx=number(groundTruth(QStringLiteral("gt_x")));
      const double gy=number(groundTruth(QStringLiteral("gt_y")));
      const double gyawDeg=number(groundTruth(QStringLiteral("gt_yaw_deg")));
      const double gyaw=std::isfinite(gyawDeg)?gyawDeg*kPi/180.0:std::numeric_limits<double>::quiet_NaN();
      const QVariant axV=aggregate(QStringLiteral("amcl.x"),QStringLiteral("last"));
      const QVariant ayV=aggregate(QStringLiteral("amcl.y"),QStringLiteral("last"));
      const QVariant ayawV=aggregate(QStringLiteral("amcl.yaw"),QStringLiteral("last"));
      const double ax=number(axV),ay=number(ayV),ayaw=number(ayawV);
      if(key==QStringLiteral("run"))return variant;
      if(key==QStringLiteral("pose uji"))return condition.isEmpty()?QVariant(variant):QVariant(condition);
      if(key==QStringLiteral("gt x (m)"))return std::isfinite(gx)?QVariant(gx):QVariant();
      if(key==QStringLiteral("gt y (m)"))return std::isfinite(gy)?QVariant(gy):QVariant();
      if(key==QStringLiteral("gt yaw (deg)"))return std::isfinite(gyawDeg)?QVariant(gyawDeg):QVariant();
      if(key==QStringLiteral("amcl x (m)"))return axV;
      if(key==QStringLiteral("amcl y (m)"))return ayV;
      if(key==QStringLiteral("amcl yaw (deg)"))return std::isfinite(ayaw)?QVariant(ayaw*180.0/kPi):QVariant();
      if(key.contains(QStringLiteral("error posisi"))){
        return std::isfinite(ax)&&std::isfinite(ay)&&std::isfinite(gx)&&std::isfinite(gy)
          ?QVariant(100.0*std::hypot(ax-gx,ay-gy)):QVariant();
      }
      if(key.contains(QStringLiteral("error yaw"))){
        return std::isfinite(ayaw)&&std::isfinite(gyaw)
          ?QVariant(normalizeAngle(ayaw-gyaw)*180.0/kPi):QVariant();
      }
      if(key==QStringLiteral("var x")||key==QStringLiteral("var x akhir"))return aggregate(QStringLiteral("amcl.var_x"),QStringLiteral("last"));
      if(key==QStringLiteral("var y")||key==QStringLiteral("var y akhir"))return aggregate(QStringLiteral("amcl.var_y"),QStringLiteral("last"));
      if(key==QStringLiteral("var yaw")||key==QStringLiteral("var yaw akhir"))return aggregate(QStringLiteral("amcl.var_yaw"),QStringLiteral("last"));
      if(key==QStringLiteral("var x maks"))return aggregate(QStringLiteral("amcl.var_x"),QStringLiteral("max"));
      if(key==QStringLiteral("var y maks"))return aggregate(QStringLiteral("amcl.var_y"),QStringLiteral("max"));
      if(key==QStringLiteral("var yaw maks"))return aggregate(QStringLiteral("amcl.var_yaw"),QStringLiteral("max"));
      if(key==QStringLiteral("min_particles"))return parameterValue(QStringLiteral("min_particles"));
      if(key==QStringLiteral("max_particles"))return parameterValue(QStringLiteral("max_particles"));
      if(key.contains(QStringLiteral("cpu mean")))return aggregate(QStringLiteral("host.cpu_percent"),QStringLiteral("mean"));
      if(key==QStringLiteral("status"))return axV.isValid()&&ayV.isValid()?QVariant(QStringLiteral("Data tersedia")):QVariant();
    }
    if(subsystem_==QStringLiteral("navigation") && currentId_.startsWith(QStringLiteral("4.4"))){
      if(key==QStringLiteral("run"))return variant;
      if(key==QStringLiteral("skenario"))return condition.isEmpty()?QVariant(variant):QVariant(condition);
      if(key==QStringLiteral("minimum_turning_radius (m)"))return parameterValue(QStringLiteral("minimum_turning_radius"));
      if(key==QStringLiteral("planning time (ms)"))return aggregate(QStringLiteral("nav_path.planning_latency_ms"),QStringLiteral("last"));
      if(key==QStringLiteral("path length (m)"))return aggregate(QStringLiteral("nav_path.length_m"),QStringLiteral("last"));
      if(key==QStringLiteral("heading variation (rad)"))return aggregate(QStringLiteral("nav_path.heading_variation_rad"),QStringLiteral("last"));
      if(key.contains(QStringLiteral("tracking error"))||key.contains(QStringLiteral("cte")))return aggregate(QStringLiteral("derived.cte_m"),QStringLiteral("rmse"));
      if(key==QStringLiteral("radius fisik agv (m)"))return parameterValue(QStringLiteral("minimum_turning_radius"));
      if(key==QStringLiteral("success")){
        const QVariant count=aggregate(QStringLiteral("nav_path.count"),QStringLiteral("last"));
        return count.isValid()?QVariant(count.toLongLong()>1):QVariant();
      }
      if(key==QStringLiteral("status")){
        const QVariant count=aggregate(QStringLiteral("nav_path.count"),QStringLiteral("last"));
        return count.isValid()?QVariant(count.toLongLong()>1?QStringLiteral("Berhasil"):QStringLiteral("Gagal")):QVariant();
      }
    }
    if(subsystem_==QStringLiteral("navigation") && currentId_.startsWith(QStringLiteral("4.5"))){
      if(key==QStringLiteral("run"))return variant;
      if(key==QStringLiteral("skenario"))return condition.isEmpty()?QVariant(variant):QVariant(condition);
      if(key==QStringLiteral("waktu (s)"))return aggregate(QStringLiteral("goal_state.duration_s"),QStringLiteral("last"));
      if(key==QStringLiteral("path length (m)"))return aggregate(QStringLiteral("nav_path.length_m"),QStringLiteral("last"));
      if(key==QStringLiteral("error posisi akhir (m)"))return aggregate(QStringLiteral("derived.endpoint_error_m"),QStringLiteral("last"));
      if(key==QStringLiteral("error yaw akhir (deg)")){
        const QVariant v=aggregate(QStringLiteral("derived.goal_yaw_error_rad"),QStringLiteral("last"));
        return degrees(v);
      }
      if(key==QStringLiteral("cte rmse (m)"))return aggregate(QStringLiteral("derived.cte_m"),QStringLiteral("rmse"));
      if(key==QStringLiteral("status")){
        const QString state=lastText(QStringLiteral("goal_state.state")).toString();
        return state.isEmpty()?QVariant():QVariant(state.contains(QStringLiteral("SUCCEEDED"),Qt::CaseInsensitive)?QStringLiteral("Sukses"):QStringLiteral("Gagal"));
      }
    }
    if(key=="run")return variant;
    if(key.contains("jarak ref"))return std::isfinite(gtDistance)?QVariant(gtDistance):QVariant();
    if(key=="rmse (m)"||key.contains("rmse 2 m"))return errorAggregate("lidar.range_center_m",gtDistance,"rmse");
    if(key==QString::fromUtf8("σ (m)"))return aggregate("lidar.range_center_m","std");
    if(key.contains("valid ratio"))return aggregate("lidar.valid_ratio_pct","mean");
    if(key.contains("dropout")||key.contains("invalid (%)"))return aggregate("lidar.dropout_pct","mean");
    if(key.contains("scan rate"))return aggregate("lidar.scan_rate_hz","mean");
    if(key.contains("timestamp jitter"))return aggregate("lidar.timestamp_jitter_ms","mean");
    if(key=="frekuensi (hz)")return parameterValue(QStringLiteral("scan_frequency"));
    if(key.contains("rmse range"))return errorAggregate("lidar.range_center_m",gtDistance,"rmse");
    if(key=="vx (m/s)"||key=="v_ref (m/s)"||key=="v_ref"||key=="|v_ref|")return std::isfinite(gtVx)?QVariant(std::abs(gtVx)):QVariant();
    if(key=="yaw ref")return std::isfinite(gtYawDeg)?QVariant(gtYawDeg):QVariant();
    if(key=="yaw imu")return degrees(aggregate("imu.yaw_rad","mean"));
    if(key=="error"&&currentId_.startsWith("4.3"))return degrees(errorAggregate("imu.yaw_rad",gtYaw,"mean",true));
    if(key=="|error|")return degrees(errorAggregate("imu.yaw_rad",gtYaw,"mae",true));
    if(key=="yaw awal"){
      const QVector<double> data=values("imu.yaw_rad");return data.isEmpty()?QVariant():QVariant(data.first()*180.0/kPi);
    }
    if(key=="yaw akhir"){
      const QVector<double> data=values("imu.yaw_rad");return data.isEmpty()?QVariant():QVariant(data.last()*180.0/kPi);
    }
    if(key=="drift"){
      const QVector<double> data=values("imu.yaw_rad");return data.size()<2?QVariant():QVariant(normalizeAngle(data.last()-data.first())*180.0/kPi);
    }
    if(key.contains(QString::fromUtf8("σ yaw"))||key.contains(QString::fromUtf8("σ run")))return degrees(aggregate("imu.yaw_rad","std"));
    if(key=="yaw mean")return degrees(aggregate("imu.yaw_rad","mean"));
    if(key=="rmse yaw"&&currentId_.startsWith("4.3"))return degrees(errorAggregate("imu.yaw_rad",gtYaw,"rmse",true));
    if(key=="mean shift yaw"){
      const QVector<double> data=values("imu.yaw_rad");
      if(data.isEmpty())return {};const double first=data.first();double sum=0.0;for(double value:data)sum+=normalizeAngle(value-first);
      return sum/data.size()*180.0/kPi;
    }
    if(key.contains("spike maksimum")){
      const QVector<double> data=values("imu.yaw_rad");
      if(data.isEmpty())return {};const double first=data.first();double peak=0.0;for(double value:data)peak=std::max(peak,std::abs(normalizeAngle(value-first)));
      return peak*180.0/kPi;
    }
    if(key=="vx odom (m/s)"||key=="|vx odom|"||key=="vx mean")return aggregate("odom.v","mean");
    if((key=="error"||key=="rmse")&&currentId_.startsWith("4.4"))return errorAggregate("odom.v",gtVx,key=="rmse"?"rmse":"mean");
    if(key=="error relatif"){
      QVariant error=errorAggregate("odom.v",gtVx,"mean");
      return error.isValid()&&std::abs(gtVx)>1e-9?QVariant(100.0*error.toDouble()/std::abs(gtVx)):QVariant();
    }
    if(key==QString::fromUtf8("σ vx")||key==QString::fromUtf8("σ"))return aggregate("odom.v","std");
    if(key=="rmse vx")return errorAggregate("odom.v",gtVx,"rmse");
    if(key=="map update efektif")return aggregate("slam.map_update_hz","mean");
    if(key.contains("cpu mean")||key=="cpu")return aggregate("host.cpu_percent","mean");
    if(key.contains("cpu peak"))return aggregate("host.cpu_percent","max");
    if(key.contains("ram mean"))return aggregate("host.ram_used_gb","mean");
    if(key.contains("ram peak"))return aggregate("host.ram_used_gb","max");
    if(key.contains("unknown tidak semestinya"))return aggregate("slam.unknown_pct","mean");
    if(key=="min/max")return QStringLiteral("%1/%2").arg(parameterText("min_particles"),parameterText("max_particles"));
    if(key=="update_min_d")return parameterValue("update_min_d");
    if(key=="update_min_a")return parameterValue("update_min_a");
    if(key=="pf_err")return parameterValue("pf_err");
    if(key=="pf_z")return parameterValue("pf_z");
    if(key=="sigma_hit")return parameterValue("sigma_hit");
    if(key=="max_beams")return parameterValue("max_beams");
    if(key=="max dist")return parameterValue("laser_likelihood_max_dist");
    if(key=="rmse pos")return aggregate("derived.amcl_pos_error_m","rmse");
    if(key=="rmse yaw"&&currentId_.startsWith("4.6"))return degrees(aggregate("derived.amcl_yaw_error_rad","rmse"));
    if(key=="resolution")return parameterValue("costmap_resolution");
    if(key=="padding")return parameterValue("footprint_padding");
    if(key=="radius")return parameterValue("inflation_radius");
    if(key=="scaling")return parameterValue("cost_scaling_factor");
    if(key=="rmin")return parameterValue("minimum_turning_radius");
    if(key=="mode"&&currentId_=="4.8.2")return QStringLiteral("%1 / %2").arg(parameterText("downsample_costmap"),parameterText("downsampling_factor"));
    if(key=="bins")return parameterValue("angle_quantization_bins");
    if(key=="penalty"&&currentId_=="4.8.4")return parameterValue("cost_penalty");
    if(key=="penalty"&&currentId_=="4.8.6")return parameterValue("non_straight_penalty");
    if(key=="penalty"&&currentId_=="4.8.7")return parameterValue("retrospective_penalty");
    if(key=="max length")return parameterValue("analytic_expansion_max_length");
    if(key=="w_smooth")return parameterValue("w_smooth");
    if(key=="final e_pos")return aggregate("derived.endpoint_error_m","last");
    if(key=="final e_yaw")return degrees(aggregate("derived.goal_yaw_error_rad","last"));
    if(key=="cte")return aggregate("derived.cte_m","rmse");
    if(key=="waktu (s)")return aggregate("goal_state.duration_s","last");
    if(key=="status"&&currentId_.startsWith("4.9")){
      const QString state=lastText("goal_state.state").toString();
      return state.isEmpty()?QVariant():QVariant(state.contains("SUCCEEDED",Qt::CaseInsensitive)?"Sukses":"Gagal");
    }
    if(key=="gt x")return groundTruth(QStringLiteral("gt_x"));
    if(key=="gt y")return groundTruth(QStringLiteral("gt_y"));
    if(key.contains("sistem x"))return aggregate("obstacle_metrics.nearest_forward_m","mean");
    if(key.contains("sistem y"))return aggregate("obstacle_metrics.nearest_left_m","mean");
    if(key=="error x")return aggregate("derived.obstacle_error_x_m","mean");
    if(key=="error y")return aggregate("derived.obstacle_error_y_m","mean");
    if(key.contains("error 2d")||key.contains("rmse 2d"))return aggregate("derived.obstacle_error_2d_m",key.contains("rmse")?"rmse":"mean");
    if(key.contains("pipeline fps"))return aggregate("perception_performance.fps","mean");
    if(key=="mean"||key.contains("mean processframe"))return aggregate("perception_performance.mean_ms","mean");
    if(key.contains("p95"))return aggregate("perception_performance.p95_ms","mean");
    if(key.contains("capture drop")){
      const auto v=values("perception_performance.capture_dropped");
      return v.isEmpty()?QVariant():QVariant(v.last()-v.first());
    }
    if(key.contains("rviz drop")){
      const auto v=values("perception_performance.rviz_dropped");
      return v.isEmpty()?QVariant():QVariant(v.last()-v.first());
    }
    if(key=="gpu")return aggregate("host.gpu_percent","mean");
    if(key=="ram")return aggregate("host.ram_used_gb","mean");
    if(key.contains("temperatur"))return aggregate("host.temperature_c","mean");
    if(key.contains("jumlah frame"))return sessions_.value(currentId_).rawRows.size();
    if(key.contains("frame terdeteksi")){
      int count=0;
      for(const auto&r:sessions_.value(currentId_).rawRows)if(number(r.value("raw_detections.count"),0)>0)++count;
      return count;
    }
    if(key.contains("detection rate")){
      const int total=sessions_.value(currentId_).rawRows.size();
      int detected=0;
      for(const auto&r:sessions_.value(currentId_).rawRows)if(number(r.value("raw_detections.count"),0)>0)++detected;
      return total?QVariant(100.0*detected/total):QVariant();
    }
    if(key.contains("miss rate")){
      QVariant detection=summaryValue("Detection Rate");
      return detection.isValid()?QVariant(100.0-detection.toDouble()):QVariant();
    }
    if(key.contains("mean confidence"))return aggregate("raw_detections.mean_confidence","mean");
    if(key.contains("std center error")||key.contains("standar deviasi x"))return aggregate(key.contains("center")?"lane_state.center_error_m":"obstacle_metrics.nearest_forward_m","std");
    if(key=="valid lane")return percentTrue("lane_state.valid");
    if(key.contains("system left"))return aggregate("lane_state.left_clearance_m","mean");
    if(key.contains("system right"))return aggregate("lane_state.right_clearance_m","mean");
    if(key=="center error")return aggregate("lane_state.center_error_m","mean");
    if(key=="state")return lastText("lane_state.state");
    if(key.contains("healthy detection"))return lastText("camera_healthy");
    if(key.contains("detection success"))return percentTrue("camera_healthy");
    if(key.contains("trigger emergency"))return lastText("perception_emergency");
    if(key=="candidate")return aggregate("object_points.count","max");
    if(key.contains("path relevant"))return aggregate("path_relevant_points.count","max");
    if(key.contains("planning relevant"))return aggregate("planning_relevant_points.count","max");
    if(key=="keputusan")return lastText("trajectory_safety_state.decision");
    if(key.contains("rmse v"))return aggregate("derived.velocity_error_mps","rmse");
    if(key.contains("rmse yaw"))return aggregate("derived.yaw_error_rps","rmse");
    if(key.contains("cte rmse")||key.contains("tracking rmse")||key.contains("rmse posisi")||key.contains("rmse dinamis"))return aggregate("derived.cte_m","rmse");
    if(key.contains("max cte"))return aggregate("derived.cte_m","max");
    if(key.contains("heading rmse"))return aggregate("derived.path_heading_error_rad","rmse");
    if(key.contains("endpoint rmse")||key.contains("error akhir")||key.contains("endpoint error")||key.contains("stop error"))return aggregate("derived.endpoint_error_m","last");
    if(key.contains("mean endpoint"))return aggregate("derived.endpoint_error_m","mean");
    if(key.contains("std endpoint"))return aggregate("derived.endpoint_error_m","std");
    if(key.contains("path length"))return aggregate("nav_path.length_m","last");
    if(key.contains("planning time")){
      const QVariant milliseconds=aggregate("nav_path.planning_latency_ms","last");
      return milliseconds.isValid()?QVariant(milliseconds.toDouble()/1000.0):QVariant();
    }
    if(key.contains("time-to-goal")||key=="waktu")return aggregate("goal_state.duration_s","last");
    if(key.contains("actual speed"))return aggregate("esc_drive_actual","mean");
    if(key.contains("speed oscillation")||key.contains("noise output v"))return aggregate("esc_drive_actual","std");
    if(key.contains("overshoot speed"))return aggregate("derived.velocity_error_mps","max");
    if(key.contains("rpm perintah"))return aggregate("derived.target_rpm","mean");
    if(key.contains("rpm feedback"))return aggregate("derived.actual_rpm","mean");
    if(key=="steering perintah")return aggregate("derived.steering_target_deg","mean");
    if(key=="feedback"||key.contains("feedback encoder"))return aggregate("derived.steering_actual_deg","mean");
    if(key=="error"||key.contains("error relatif"))return aggregate("derived.steering_error_rad",key.contains("relatif")?"mae":"mean");
    if(key.contains("rise time"))return responseMetric("rise");
    if(key.contains("settling"))return responseMetric("settling");
    if(key.contains("overshoot"))return responseMetric("overshoot");
    if(key.contains("e ss"))return responseMetric("ess");
    if(key.contains("gain amplitudo"))return responseMetric("gain");
    if(key.contains("iq peak"))return aggregate("foc_telemetry.iq_a","max");
    if(key.contains("iq rms")||key.contains("ripple iq"))return aggregate("foc_telemetry.iq_a",key.contains("ripple")?"std":"rmse");
    if(key.contains("mean gyro")){
      const QString axis=variant.toLower();
      return aggregate(axis.startsWith('x')?"imu.gx":axis.startsWith('y')?"imu.gy":"imu.gz","mean");
    }
    if(key=="std (rad/s)"){
      const QString axis=variant.toLower();
      return aggregate(axis.startsWith('x')?"imu.gx":axis.startsWith('y')?"imu.gy":"imu.gz","std");
    }
    if(key.contains("heading imu"))return aggregate("imu.yaw_rad","mean");
    if(key=="success"||key.contains("nav2 success")){
      const QString state=lastText("goal_state.state").toString();
      return state.contains("SUCCEEDED",Qt::CaseInsensitive);
    }
    const QStringList identityTokens={
      "frequency","sensor timeout","threshold","set","mode","alpha","padding","radius","faktor","weight","penalty","max length","command","distance","tolerance","target","rmin","downsampling","bins","parameter","kanal","sumbu","heading referensi","metode","jarak","publish rate","mode capture","kondisi","skenario","id","lapisan","kelompok","tahap","item","bagian","posisi lateral"
    };
    for(const QString&token:identityTokens)if(key.contains(token))return key.contains("kondisi")||key.contains("skenario")?QVariant(condition.isEmpty()?variant:condition):QVariant(variant);
    return {
    };
  }
  void appendSummaryRow() {
    if(isSpecial41MultiRowLeaf()){
      if(recording_){status_->setText(QStringLiteral("Tabel 4.1 raw/komunikasi akan ditambahkan otomatis ketika Run dihentikan."));return;}
      if(session().rawRows.isEmpty()){status_->setText(QStringLiteral("Tidak ada sampel. Jalankan Run terlebih dahulu."));return;}
      appendSpecial41Rows();
      return;
    }
    if(session().rawRows.isEmpty()&&!recording_){
      status_->setText("Tidak ada sampel. Mulai run atau gunakan baris manual.");
      return;
    }
    QVariantMap row;
    const QStringList cols=currentColumns();
    for(const QString&column:cols)row[column]=summaryValue(column);
    const QString condition = parameterText(QStringLiteral("condition"));
    if(!condition.isEmpty()&&!cols.isEmpty()&&!row.value(cols.first()).isValid())row[cols.first()]=condition;
    session().summaryRows<<row;
    loading_=true;
    insertRow(row);
    loading_=false;
    table_->resizeColumnsToContents();
    refreshSummaryPlotIfNeeded();
    status_->setText(QStringLiteral("Ringkasan run ditambahkan. Sel kuning wajib dilengkapi secara manual sebelum dipakai di laporan."));
  }
  void addManualRow(){
    QVariantMap row;
    const QStringList cols=currentColumns();
    const QString variation=parameterText(QStringLiteral("variation"));
    const QString condition=parameterText(QStringLiteral("condition"));
    if(!cols.isEmpty())row[cols.first()]=variation.isEmpty()?condition:variation;
    session().summaryRows<<row;
    loading_=true;
    insertRow(row);
    loading_=false;
  }
  void insertRow(const QVariantMap&row){
    const int r=table_->rowCount();
    table_->insertRow(r);
    const QStringList cols=currentColumns();
    for(int c=0;
    c<cols.size();
    ++c){
      const QVariant value=row.value(cols[c]);
      QString text;
      if(value.isValid()){
        if(value.userType()==QMetaType::Bool)text=value.toBool()?"true":"false";
        else{
          bool ok=false;
          double numberValue=value.toDouble(&ok);
          text=ok&&std::isfinite(numberValue)?QString::number(numberValue,'g',10):value.toString();
        }
      }
      auto*item=new QTableWidgetItem(text);
      item->setBackground(text.isEmpty()?QColor("#5a4317"):QColor("#173b25"));
      item->setToolTip(text.isEmpty()?"Tidak tersedia otomatis — isi dari ground truth/instrumen eksternal":"Diisi otomatis; tetap dapat dikoreksi jika metode pengujian mensyaratkan ground truth");
      table_->setItem(r,c,item);
    }
  }
  // While a run is recording, keep the active summary row live so the table
  // reflects the in-progress acquisition instead of waiting for STOP.
  void updateRecordingSummaryRow(){
    if(isSpecial41MultiRowLeaf())return;
    if(session().rawRows.isEmpty()||currentId_.isEmpty())return;
    QVariantMap row;
    const QStringList cols=currentColumns();
    for(const QString&column:cols)row[column]=summaryValue(column);
    const QString condition=parameterText(QStringLiteral("condition"));
    if(!condition.isEmpty()&&!cols.isEmpty()&&!row.value(cols.first()).isValid())row[cols.first()]=condition;
    if(activeRecordingSummaryRow_<0||activeRecordingSummaryRow_>=table_->rowCount()){
      loading_=true;
      insertRow(row);
      loading_=false;
      activeRecordingSummaryRow_=table_->rowCount()-1;
    }else{
      loading_=true;
      const int r=activeRecordingSummaryRow_;
      for(int c=0;c<cols.size();++c){
        const QVariant value=row.value(cols[c]);
        QString text;
        if(value.isValid()){
          if(value.userType()==QMetaType::Bool)text=value.toBool()?"true":"false";
          else{
            bool ok=false;
            double numberValue=value.toDouble(&ok);
            text=ok&&std::isfinite(numberValue)?QString::number(numberValue,'g',10):value.toString();
          }
        }
        auto*item=table_->item(r,c);
        if(!item){
          item=new QTableWidgetItem();
          table_->setItem(r,c,item);
        }
        item->setText(text);
        item->setBackground(text.isEmpty()?QColor("#5a4317"):QColor("#173b25"));
        item->setToolTip(text.isEmpty()?"Tidak tersedia otomatis — isi dari ground truth/instrumen eksternal":"Diisi otomatis; tetap dapat dikoreksi jika metode pengujian mensyaratkan ground truth");
      }
      loading_=false;
    }
    table_->resizeColumnsToContents();
  }
  void saveTableState(){
    if(currentId_.isEmpty())return;
    QVector<QVariantMap>rows;
    const QStringList cols=currentColumns();
    for(int r=0;
    r<table_->rowCount();
    ++r){
      QVariantMap row;
      for(int c=0;
      c<cols.size();
      ++c)row[cols[c]]=table_->item(r,c)?table_->item(r,c)->text():QString();
      rows<<row;
    }
    sessions_[currentId_].summaryRows=rows;
  }
  void removeSelectedRows(){
    QSet<int>rows;
    for(const auto&range:table_->selectedRanges())for(int r=range.topRow();
    r<=range.bottomRow();
    ++r)rows.insert(r);
    QList<int>ordered=rows.values();
    std::sort(ordered.begin(),ordered.end(),std::greater<int>());
    loading_=true;
    for(int r:ordered)table_->removeRow(r);
    loading_=false;
    saveTableState();
    refreshSummaryPlotIfNeeded();
  }
  void clearCurrent(){
    if(QMessageBox::question(this,"Bersihkan data","Hapus tabel, raw sample, dan grafik untuk subpengujian ini?")!=QMessageBox::Yes)return;
    sessions_[currentId_]=ExperimentSessionData{
    };
    loading_=true;
    table_->setRowCount(0);
    loading_=false;
    for (GraphCard *c : graphCards_) c->clear();
    status_->setText("Data subpengujian dibersihkan.");
  }
  struct GraphSnapshot {
    QMap<QString, QVector<QPointF>> series;
    QString xLabel = QStringLiteral("Waktu (s)");
    QString yLabel = QStringLiteral("Nilai");
    bool connectPoints = true;
    QString emptyMessage;
  };
  // Human-readable reason shown on a graph card when the expected source has
  // not delivered any data yet. Data-driven: derived from the graph spec and
  // the live telemetry store — no hardcoded per-leaf UI branches.
  static QString sourceNameForPath(const QString &path) {
    if (path.startsWith(QStringLiteral("lidar"))) return QStringLiteral("LiDAR /scan");
    if (path.startsWith(QStringLiteral("slam"))) return QStringLiteral("SLAM /map");
    if (path.startsWith(QStringLiteral("amcl")) || path.startsWith(QStringLiteral("derived.amcl"))) return QStringLiteral("AMCL /amcl_pose");
    if (path.startsWith(QStringLiteral("odom"))) return QStringLiteral("odometri /odom");
    if (path.startsWith(QStringLiteral("ekf_global"))) return QStringLiteral("EKF global /odometry/filtered_map");
    if (path.startsWith(QStringLiteral("ekf_local"))) return QStringLiteral("EKF lokal /odometry/filtered");
    if (path.startsWith(QStringLiteral("esc_odom"))) return QStringLiteral("odometri ESC /esc/odom");
    if (path.startsWith(QStringLiteral("localization_state"))) return QStringLiteral("localization_state (anchor global)");
    if (path.startsWith(QStringLiteral("gnss_quality"))) return QStringLiteral("GNSS /gnss/quality");
    if (path.startsWith(QStringLiteral("gnss_fix"))) return QStringLiteral("GNSS /gnss/fix_raw");
    if (path.startsWith(QStringLiteral("gnss_vel"))) return QStringLiteral("kecepatan GNSS");
    if (path.startsWith(QStringLiteral("gnss_cog"))) return QStringLiteral("COG /gnss/cog_heading_fusion");
    if (path.startsWith(QStringLiteral("imu"))) return QStringLiteral("IMU /imu/data");
    if (path.startsWith(QStringLiteral("esc_"))) return QStringLiteral("ESC");
    if (path.startsWith(QStringLiteral("foc_telemetry"))) return QStringLiteral("ESC FOC /esc/foc/telemetry");
    if (path.startsWith(QStringLiteral("cmd_"))) return QStringLiteral("perintah /cmd_vel");
    if (path.startsWith(QStringLiteral("mppi_"))) return QStringLiteral("MPPI supervisor");
    if (path.startsWith(QStringLiteral("nav_path"))) return QStringLiteral("path Nav2 /plan");
    if (path.startsWith(QStringLiteral("goal_state"))) return QStringLiteral("status goal");
    if (path.startsWith(QStringLiteral("raw_detections"))) return QStringLiteral("deteksi YOLO /obstacle_detection/obstacles");
    if (path.startsWith(QStringLiteral("perception_performance"))) return QStringLiteral("performa YOLO /obstacle_detection/performance");
    if (path.startsWith(QStringLiteral("obstacle_metrics"))) return QStringLiteral("metrik obstacle");
    if (path.startsWith(QStringLiteral("lane_state"))) return QStringLiteral("lane safety state");
    if (path.startsWith(QStringLiteral("drivable_"))) return QStringLiteral("drivable space YOLOP");
    if (path.startsWith(QStringLiteral("camera_"))) return QStringLiteral("kesehatan kamera");
    if (path.startsWith(QStringLiteral("trajectory_safety"))) return QStringLiteral("trajectory safety");
    if (path.startsWith(QStringLiteral("near_field"))) return QStringLiteral("near-field state");
    if (path.startsWith(QStringLiteral("derived."))) return QStringLiteral("nilai turunan");
    if (path.startsWith(QStringLiteral("host."))) return QStringLiteral("metrik host");
    return path;
  }
  static QString emptyMessageForSeries(const QStringList &missing, const QString &mode) {
    if (missing.isEmpty()) return QString();
    QStringList names;
    for (const QString &p : missing) {
      const QString n = sourceNameForPath(p);
      if (!names.contains(n)) names << n;
    }
    std::sort(names.begin(), names.end());
    const QString head = mode == QStringLiteral("scatter")
      ? QStringLiteral("BELUM ADA DATA\nMenunggu data valid dari:\n")
      : QStringLiteral("BELUM ADA DATA\nGrafik aktif otomatis begitu data masuk dari:\n");
    QString msg = head + names.join(QStringLiteral("\n")) +
      QStringLiteral("\n\nPeriksa status sensor dan jalannya stack; jangan isi manual.");
    msg += QStringLiteral("\n\nTips:\n") + troubleshootMissingTelemetry(missing);
    return msg;
  }
  static QString troubleshootMissingTelemetry(const QStringList &missing) {
    QStringList tips;
    bool wantEKFLocal = false, wantEKFGlobal = false, wantESC = false, wantIMU = false, wantGNSS = false, wantPerception = false;
    for (const QString &p : missing) {
      if (p.startsWith(QStringLiteral("ekf_local"))) wantEKFLocal = true;
      else if (p.startsWith(QStringLiteral("ekf_global"))) wantEKFGlobal = true;
      else if (p.startsWith(QStringLiteral("esc_")) || p.startsWith(QStringLiteral("foc_telemetry"))) wantESC = true;
      else if (p.startsWith(QStringLiteral("imu"))) wantIMU = true;
      else if (p.startsWith(QStringLiteral("gnss")) || p.startsWith(QStringLiteral("localization_state"))) wantGNSS = true;
      else if (p.startsWith(QStringLiteral("raw_detections")) || p.startsWith(QStringLiteral("perception_performance")) ||
               p.startsWith(QStringLiteral("obstacle_metrics")) || p.startsWith(QStringLiteral("lane_state")) ||
               p.startsWith(QStringLiteral("camera_"))) wantPerception = true;
    }
    if (wantEKFLocal) tips << QStringLiteral("- EKF lokal: cek /odometry/filtered dan /imu/data; pastikan IMU fresh dan sensor_timeout tidak terlewat.");
    if (wantEKFGlobal) tips << QStringLiteral("- EKF global: cek /odometry/filtered_map; butuh GNSS fiks + anchor map dari LocalizationCore.");
    if (wantESC) tips << QStringLiteral("- ESC/aktuator: cek ESC armed/ready, port serial, dan topik /esc/status.");
    if (wantIMU) tips << QStringLiteral("- IMU: cek koneksi serial/USB dan /imu/connected.");
    if (wantGNSS) tips << QStringLiteral("- GNSS: cek /gnss/connected, fix GPS, satelit, DOP/hAcc; outdoor dengan view langit.");
    if (wantPerception) tips << QStringLiteral("- Persepsi: cek /camera/color/image_raw, /obstacle_detection/obstacles, dan /obstacle_detection/performance; GUI menjembatani topic produksi ini ke key grafik/CSV.");
    return tips.join(QStringLiteral("\n"));
  }
  // Build one renderer snapshot from the SAME raw/session data. No acquisition,
  // subscriber, or timer lives here. Special graph semantics are data-driven via
  // ExperimentSpec::graphs; unspecified graphs safely fall back to all liveSeries.
  GraphSnapshot liveDataForGraph(int index) const {
    GraphSnapshot out;
    const ExperimentSpec &s = spec();
    if (index < 0 || index >= s.graphs.size()) {
      out.series = sessions_.value(currentId_).liveSeries;
      if (out.series.isEmpty()) out.emptyMessage = emptyMessageForSeries(
        s.liveSeries.values(), QStringLiteral("time_series"));
      return out;
    }
    const ExperimentGraphSpec &g = s.graphs.at(index);
    if (g.type == QStringLiteral("scatter")) {
      out.xLabel = g.xSeries;
      out.yLabel = g.ySeries;
      out.connectPoints = false;
      // The live scatter buffer starts as soon as the GUI receives valid
      // telemetry. startRecording() clears it together with rawRows, therefore
      // after a recorded run it contains exactly that run's display points.
      out.series[QStringLiteral("Posisi aktual")] = sessions_.value(currentId_).liveScatter.value(index);
      if (out.series.value(QStringLiteral("Posisi aktual")).isEmpty()) {
        out.emptyMessage = emptyMessageForSeries({g.xSeries, g.ySeries}, QStringLiteral("scatter"));
      }
      return out;
    }
    const QMap<QString, QVector<QPointF>> &all = sessions_.value(currentId_).liveSeries;
    QStringList missing;
    for (const QString &label : g.series) {
      if (all.contains(label) && !all.value(label).isEmpty()) out.series[label] = all.value(label);
      else missing << s.liveSeries.value(label, label);
    }
    if (!missing.isEmpty()) out.emptyMessage = emptyMessageForSeries(missing, QStringLiteral("time_series"));
    return out;
  }
  // Multi-graph refresh. Both normal cards and optional fullscreen receive the
  // same GraphSnapshot from this single refresh flow.
  void refreshGraphs(){
    const ExperimentSpec &s = spec();
    if (graphCards_.size() != s.graphCaptions.size()) rebuildGraphCards();
    for (int i = 0; i < graphCards_.size(); ++i) {
      const QString caption = s.graphCaptions.value(i);
      const QString title = QStringLiteral("Format mengacu %1 — Data Aktual GUI").arg(caption.isEmpty() ? s.section : caption);
      if (plotMode_->currentIndex() == 0) {
        const GraphSnapshot snap = liveDataForGraph(i);
        graphCards_[i]->setData(title, snap.xLabel, snap.yLabel, snap.series, snap.connectPoints, snap.emptyMessage);
      } else {
        refreshSummaryForCard(i, title);
      }
    }
    if (fullscreenDialog_ && fullscreenDialog_->isVisible()) {
      const int idx = fullscreenGraphIndex_;
      if (idx >= 0 && idx < graphCards_.size()) {
        const QString caption = s.graphCaptions.value(idx);
        const QString title = QStringLiteral("Format mengacu %1 — Data Aktual GUI").arg(caption.isEmpty() ? s.section : caption);
        if (plotMode_->currentIndex() == 0) {
          const GraphSnapshot snap = liveDataForGraph(idx);
          fullscreenDialog_->updateData(title, snap.xLabel, snap.yLabel, snap.series, snap.connectPoints, snap.emptyMessage);
        } else refreshSummaryForCard(idx, title, fullscreenDialog_);
      }
    }
  }
  void refreshSummaryForCard(int cardIndex, const QString &title, GraphFullscreenDialog *target = nullptr) {
    Q_UNUSED(cardIndex);
    QMap<QString, QVector<QPointF>> series;
    const QStringList cols = currentColumns();
    for (int c = 1; c < cols.size(); ++c) {
      for (int r = 0; r < table_->rowCount(); ++r) {
        double y = 0.0;
        if (!table_->item(r, c) || !numericText(table_->item(r, c)->text(), y)) continue;
        double x = r + 1.0;
        if (table_->item(r, 0)) {
          double parsed = 0.0;
          if (numericText(table_->item(r, 0)->text(), parsed)) x = parsed;
        }
        series[cols[c]] << QPointF(x, y);
      }
    }
    const QString axisX = cols.value(0, QStringLiteral("Varian"));
    if (target) target->updateData(title, axisX, QStringLiteral("Nilai tabel"), series, true);
    else if (cardIndex >= 0 && cardIndex < graphCards_.size())
    graphCards_[cardIndex]->setData(title, axisX, QStringLiteral("Nilai tabel"), series, true);
  }
  void refreshSummaryPlotIfNeeded(){
    if(plotMode_->currentIndex()==1)refreshGraphs();
  }
  // Fullscreen: single extra renderer, same data model, NO timer/subscriber.
  void openFullscreenGraph(int index){
    const ExperimentSpec &s = spec();
    if (index < 0 || index >= s.graphCaptions.size()) return;
    const QString caption = s.graphCaptions.at(index);
    const QString title = QStringLiteral("Format mengacu %1 — Data Aktual GUI").arg(caption.isEmpty() ? s.section : caption);
    if (!fullscreenDialog_) {
      fullscreenDialog_ = new GraphFullscreenDialog(title, this);
      fullscreenDialog_->setWindowModality(Qt::NonModal);
    } else {
      fullscreenDialog_->setWindowTitle(title);
    }
    fullscreenGraphIndex_ = index;
    if (plotMode_->currentIndex() == 0)
    fullscreenDialog_->updateData(title, QStringLiteral("Waktu (s)"), QStringLiteral("Nilai"), session().liveSeries, true);
    else refreshSummaryForCard(index, title, fullscreenDialog_);
    fullscreenDialog_->show();
    fullscreenDialog_->raise();
    fullscreenDialog_->activateWindow();
  }
  void updateAvailability(){
    if(catalog_.isEmpty())return;
    if(!spec().recordable && spec().liveSeries.isEmpty()){
      availability_->setText(QStringLiteral("Tabel referensi DOCX — tidak memerlukan topic ROS atau Run."));
      availability_->setStyleSheet("color:#46b36b");
      return;
    }
    QStringList available,missing;
    for(auto it=spec().liveSeries.cbegin();
    it!=spec().liveSeries.cend();
    ++it){
      const QVariant value=instantValue(it.value());
      if(value.isValid())available<<it.key();
      else missing<<it.key();
    }
    QString text=QStringLiteral("AUTO %1/%2: %3").arg(available.size()).arg(spec().liveSeries.size()).arg(available.join(", "));
    if(!missing.isEmpty())text+=QStringLiteral(" | TIDAK TERSEDIA: ")+missing.join(", ");
    if(subsystem_=="steering"&&telemetry_->age("foc_telemetry")>2.0)text+=QStringLiteral(" | Source ESC tidak mempublish /esc/foc/telemetry: Id/Iq/Vd/Vq/Vbus wajib tetap kosong/manual.");
    availability_->setText(text);
    availability_->setStyleSheet(missing.isEmpty()?"color:#46b36b":"color:#e7953f");
  }
  QStringList missingTopics() {
    QStringList out;
    for(auto it=spec().liveSeries.cbegin();
    it!=spec().liveSeries.cend();
    ++it)if(!instantValue(it.value()).isValid())out<<it.value();
    return out;
  }
  void saveEvidence(){
    saveTableState();
    const QString variation = parameterText(QStringLiteral("variation"));
    const QString stem=slug(subsystem_+"_"+spec().id+"_"+variation);
    const QStringList cols=currentColumns();
    const QString tablePath=reports_->saveCsvOrdered(stem+"_table",cols,session().summaryRows);
    const QString rawPath=reports_->saveCsv(stem+"_raw",session().rawRows);
    // Save ALL graphs defined for this leaf, not just one.
    QStringList pngPaths;
    for (int i = 0; i < graphCards_.size(); ++i) {
      const QString cap = spec().graphCaptions.value(i);
      const QString pngName = QStringLiteral("%1_graph%2%3").arg(stem).arg(i + 1).arg(cap.isEmpty() ? QString() : QStringLiteral("_") + slug(cap));
      const QString p = reports_->savePng(pngName, graphCards_[i]->plot());
      if (!p.isEmpty()) pngPaths << p;
    }
    const QString pngPath = pngPaths.isEmpty() ? QString() : pngPaths.join("; ");
    QString startTime=session().rawRows.isEmpty()?QString():session().rawRows.first().value("time_iso").toString();
    QString stopTime=session().rawRows.isEmpty()?QString():session().rawRows.last().value("time_iso").toString();
    QJsonObject manifest{
      {
        "subsystem",subsystem_
      },{
        "section_id",spec().id
      },{
        "section_title",spec().section
      },
      {
        "table_name",currentTableCaption()
      },{
        "graph_options",QJsonArray::fromStringList(spec().graphCaptions)
      },{
        "graph_png_files",QJsonArray::fromStringList(pngPaths)
      },
      {
        "variation",variation
      },{
        "condition",parameterText(QStringLiteral("condition"))
      },
      {
        "sample_rate_hz",stores_.contains("gui")?number(stores_["gui"]->get("reporting.sample_rate_hz",5.0),5.0):5.0
      },
      {
        "start_time",startTime
      },{
        "stop_time",stopTime
      },
      {
        "raw_sample_count",session().rawRows.size()
      },{
        "summary_row_count",session().summaryRows.size()
      },
      {
        "topic_availability",availability_->text()
      },{
        "missing_topics",QJsonArray::fromStringList(missingTopics())
      },
      {
        "table_csv",tablePath
      },{
        "raw_csv",rawPath
      },{
        "graph_png",pngPath
      },
      {
        "note","Blank/yellow cells are not published or require external ground truth; no estimated report values were injected."
      }
    };
    const QString manifestPath=QDir(reports_->root()).filePath(stem+"_manifest.json");
    QSaveFile file(manifestPath);
    if(file.open(QIODevice::WriteOnly)){
      file.write(QJsonDocument(manifest).toJson(QJsonDocument::Indented));
      file.commit();
    }
    QMessageBox::information(this,"Bukti pengujian tersimpan",QString("Tabel: %1\nRaw: %2\nGrafik: %3\nManifest: %4").arg(tablePath,rawPath,pngPath,manifestPath));
  }
};
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
class ImuCalibrationPage:public QWidget{
  public:
  ImuCalibrationPage(
  TelemetryStore *t, ReportManager *r,
  const QMap<QString,std::shared_ptr<YamlStore>> &s, QWidget *p=nullptr)
  : QWidget(p), t_(t), r_(r), s_(s)
  {
    auto *l=new QVBoxLayout(this);
    auto *h=new QLabel("Wizard Kalibrasi Presisi IMU — Stage 2");
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto *d=new QLabel(
    "Letakkan kendaraan benar-benar diam. Stage 2 tidak hanya menghitung bias/covariance, "
    "tetapi juga memverifikasi durasi capture, gyro-Z noise, dan error magnitudo gravitasi. "
    "Autonomous motion tetap diblokir sampai stationary_calibration_valid=PASS.");
    d->setWordWrap(true);
    l->addWidget(d);
    auto *b=new QHBoxLayout();
    start_=new QPushButton("▶ Mulai Stationary Capture");
    stop_=new QPushButton("■ Stop + Analisis");
    apply_=new QPushButton("✓ Terapkan + Certify IMU");
    export_=new QPushButton("Ekspor CSV");
    apply_->setEnabled(false);
    export_->setEnabled(false);
    for(auto *w:{
      start_,stop_,apply_,export_
    }) b->addWidget(w);
    l->addLayout(b);
    status_=new QLabel("READY • minimum mengikuti imu.yaml; rekomendasi 30–60 s");
    l->addWidget(status_);
    summary_=new QPlainTextEdit();
    summary_->setReadOnly(true);
    summary_->setMinimumHeight(190);
    l->addWidget(summary_);
    plot_=new LivePlotWidget("IMU: gyro Z, Ackermann yaw-rate & residual");
    l->addWidget(plot_,1);
    connect(start_,&QPushButton::clicked,this,[this](){
      samples_.clear();
      result_.clear();
      rec_=true;
      lastStamp_=-1.0;
      apply_->setEnabled(false);
      export_->setEnabled(false);
      plot_->clear();
      status_->setText("CAPTURING • kendaraan WAJIB diam dan tidak disentuh");
    });
    connect(stop_,&QPushButton::clicked,this,[this](){
      analyze();
    });
    connect(apply_,&QPushButton::clicked,this,[this](){
      applyYaml();
    });
    connect(export_,&QPushButton::clicked,this,[this](){
      QMessageBox::information(this,"CSV",r_->saveCsv("imu_stationary_calibration_stage2",samples_));
    });
    timer_=new QTimer(this);
    timer_->setInterval(50);
    connect(timer_,&QTimer::timeout,this,[this](){
      refresh();
    });
    timer_->start();
  }
  private:
  TelemetryStore *t_;
  ReportManager *r_;
  QMap<QString,std::shared_ptr<YamlStore>> s_;
  QPushButton *start_,*stop_,*apply_,*export_;
  QLabel *status_,*live_;
  QPlainTextEdit *summary_;
  LivePlotWidget *plot_,*headingPlot_;
  QTimer *timer_;
  bool rec_=false;
  QVector<QVariantMap> samples_;
  QVariantMap result_;
  double lastStamp_=-1.0;
  double imuParam(const QString &name,double fallback)const{
    auto st=s_.value("imu");
    return st ? number(st->get("data_imu_node.ros__parameters."+name,fallback),fallback) : fallback;
  }
  static double mean(const QVector<double>&v){
    return v.isEmpty()?0.0:std::accumulate(v.begin(),v.end(),0.0)/v.size();
  }
  static double variance(const QVector<double>&v){
    if(v.size()<2) return 0.0;
    const double m=mean(v);
    double sum=0.0;
    for(double x:v) sum+=(x-m)*(x-m);
    return sum/(v.size()-1);
  }
  QVector<double> col(const QString&k)const{
    QVector<double> v;
    for(const auto&r:samples_){
      const double x=number(r.value(k));
      if(std::isfinite(x)) v<<x;
    }
    return v;
  }
  void refresh(){
    const QVariantMap imu=t_->get("imu").toMap();
    if(imu.isEmpty()) return;
    const double gyroZ=number(imu.value("gz"));
    const double ackW=number(t_->get("esc_kinematic_yaw_rate"));
    const double residual=(std::isfinite(gyroZ)&&std::isfinite(ackW))?
    ackW-gyroZ:std::numeric_limits<double>::quiet_NaN();
    plot_->append({
      {
        "gyro Z",gyroZ
      },{
        "Ackermann w",ackW
      },{
        "model-IMU residual",residual
      }
    });
    if(!rec_||t_->age("imu")>0.5) return;
    const double stamp=t_->timestamp("imu");
    if(stamp<0.0||stamp==lastStamp_) return;
    lastStamp_=stamp;
    QVariantMap row{
      {
        "t_monotonic",stamp
      }
    };
    for(const char *k:{
      "roll_rad","pitch_rad","yaw_rad","gx","gy","gz","ax","ay","az"
    }){
      const QString key=QString::fromLatin1(k);
      row[key]=imu.value(key);
    }
    row["ackermann_w"]=ackW;
    row["model_imu_residual"]=residual;
    samples_.push_back(row);
    if(samples_.size()%10==0){
      status_->setText(QString("CAPTURING • %1 sampel").arg(samples_.size()));
    }
  }
  void analyze(){
    rec_=false;
    const int minSamples=static_cast<int>(std::lround(imuParam("stationary_calibration_min_samples",80.0)));
    const double minDuration=imuParam("stationary_calibration_min_duration_sec",8.0);
    const double maxGzStd=imuParam("stationary_calibration_max_gyro_z_std_rps",0.03);
    const double maxAccelErr=imuParam("stationary_calibration_max_accel_norm_error_mps2",0.75);
    if(samples_.size()<2){
      status_->setText("FAIL • tidak ada cukup sampel IMU");
      return;
    }
    const QVector<double> gx=col("gx"),gy=col("gy"),gz=col("gz");
    const QVector<double> ax=col("ax"),ay=col("ay"),az=col("az");
    const QVector<double> roll=col("roll_rad"),pitch=col("pitch_rad"),yaw=col("yaw_rad");
    const QVector<double> ts=col("t_monotonic");
    if(gx.isEmpty()||gy.isEmpty()||gz.isEmpty()||ax.isEmpty()||ay.isEmpty()||az.isEmpty()||
    roll.isEmpty()||pitch.isEmpty()||yaw.isEmpty()||ts.size()<2){
      status_->setText("FAIL • kolom IMU tidak lengkap");
      return;
    }
    const double duration=*std::max_element(ts.cbegin(),ts.cend())-
    *std::min_element(ts.cbegin(),ts.cend());
    const double gzStd=std::sqrt(std::max(0.0,variance(gz)));
    QVector<double> accNorm;
    const int n=std::min({
      ax.size(),ay.size(),az.size()
    });
    for(int i=0;
    i<n;
    ++i) accNorm<<std::sqrt(ax[i]*ax[i]+ay[i]*ay[i]+az[i]*az[i]);
    const double accelNormError=std::abs(mean(accNorm)-9.80665);
    const bool pass=samples_.size()>=minSamples && duration>=minDuration &&
    gzStd<=maxGzStd && accelNormError<=maxAccelErr;
    QVariantList oldG=s_.value("imu")->get(
    "data_imu_node.ros__parameters.gyro_bias",QVariantList{
      0.0,0.0,0.0
    }).toList();
    QVariantList oldA=s_.value("imu")->get(
    "data_imu_node.ros__parameters.accel_bias",QVariantList{
      0.0,0.0,0.0
    }).toList();
    while(oldG.size()<3) oldG<<0.0;
    while(oldA.size()<3) oldA<<0.0;
    const double mr=mean(roll),mp=mean(pitch),g=9.80665;
    const QVector<double> accMean={
      mean(ax),mean(ay),mean(az)
    };
    const QVector<double> expected={
      -g*std::sin(mp),g*std::sin(mr)*std::cos(mp),g*std::cos(mr)*std::cos(mp)
    };
    const QVector<double> gm={
      mean(gx),mean(gy),mean(gz)
    };
    const QVector<double> gv={
      variance(gx),variance(gy),variance(gz)
    };
    const QVector<double> av={
      variance(ax),variance(ay),variance(az)
    };
    QVariantList gb,ab,gcov,acov,ocov;
    for(int i=0;
    i<3;
    ++i){
      gb<<oldG[i].toDouble()+gm[i];
      ab<<oldA[i].toDouble()+(accMean[i]-expected[i]);
      gcov<<std::max(1e-7,2.0*gv[i]);
      acov<<std::max(1e-5,2.0*av[i]);
    }
    double sy=0.0,cy=0.0;
    for(double y:yaw){
      sy+=std::sin(y);
      cy+=std::cos(y);
    }
    const double ym=std::atan2(sy/yaw.size(),cy/yaw.size());
    QVector<double> yd;
    for(double y:yaw) yd<<normalizeAngle(y-ym);
    ocov<<std::max(1e-7,2.0*variance(roll))
    <<std::max(1e-7,2.0*variance(pitch))
    <<std::max(1e-7,2.0*variance(yd));
    result_={
      {
        "pass",pass
      },{
        "gyro_bias",gb
      },{
        "accel_bias",ab
      },
      {
        "angular_velocity_covariance",gcov
      },{
        "linear_acceleration_covariance",acov
      },
      {
        "orientation_covariance",ocov
      },{
        "sample_count",samples_.size()
      },
      {
        "duration_sec",duration
      },{
        "gyro_z_std_rps",gzStd
      },
      {
        "accel_norm_error_mps2",accelNormError
      },{
        "mean_yaw_rad",ym
      },
      {
        "limit_min_samples",minSamples
      },{
        "limit_min_duration_sec",minDuration
      },
      {
        "limit_max_gyro_z_std_rps",maxGzStd
      },{
        "limit_max_accel_norm_error_mps2",maxAccelErr
      }
    };
    summary_->setPlainText(QString::fromUtf8(
    QJsonDocument(QJsonObject::fromVariantMap(result_)).toJson(QJsonDocument::Indented)));
    status_->setText(QString("%1 • N=%2 duration=%3 s • gyroZ std=%4 rad/s • |a|-g=%5 m/s²")
    .arg(pass?"PASS":"FAIL").arg(samples_.size()).arg(duration,0,'f',1)
    .arg(gzStd,0,'f',5).arg(accelNormError,0,'f',3));
    apply_->setEnabled(pass);
    export_->setEnabled(true);
  }
  void applyYaml(){
    if(result_.isEmpty()||!result_.value("pass").toBool()) return;
    const QString saved=QDateTime::currentDateTime().toString(Qt::ISODate);
    for(const QString &key:{
      QString("imu"),QString("imu_calibration")
    }){
      if(!s_.contains(key)) continue;
      for(const QString &name:{
        QString("gyro_bias"),QString("accel_bias"),
        QString("angular_velocity_covariance"),QString("linear_acceleration_covariance"),
        QString("orientation_covariance")
      }){
        s_[key]->set("data_imu_node.ros__parameters."+name,result_.value(name));
      }
      s_[key]->set("data_imu_node.ros__parameters.stationary_calibration_valid",true);
      s_[key]->set("data_imu_node.ros__parameters.stationary_calibration_saved_at",saved);
      s_[key]->set("data_imu_node.ros__parameters.stationary_calibration_sample_count",result_.value("sample_count"));
      s_[key]->set("data_imu_node.ros__parameters.stationary_calibration_duration_sec",result_.value("duration_sec"));
      s_[key]->set("data_imu_node.ros__parameters.stationary_calibration_gyro_z_std_rps",result_.value("gyro_z_std_rps"));
      s_[key]->set("data_imu_node.ros__parameters.stationary_calibration_accel_norm_error_mps2",result_.value("accel_norm_error_mps2"));
    }
    if(s_.contains("localization")){
      auto loc=s_.value("localization");
      // Estimator policy is invariant: GNSS contributes body vx+vyaw while
      // absolute yaw remains IMU-only. Calibration revocation only clears
      // evidence used by the autonomy quality gate; it must not starve EKF.
      loc->set("localization_core.ros__parameters.enable_global_gnss_velocity_fusion",true);
      loc->set("localization_core.ros__parameters.enable_global_gnss_cog_fusion",false);
      loc->set("localization_core.ros__parameters.enable_gnss_course_yaw_correction",false);
      loc->set("localization_core.ros__parameters.gnss_velocity_calibration_valid",false);
      loc->set("localization_core.ros__parameters.gnss_cog_calibration_valid",false);
      loc->set("localization_core.ros__parameters.gnss_velocity_calibration_saved_at",QString());
      loc->set("localization_core.ros__parameters.gnss_cog_calibration_saved_at",QString());
      loc->set("localization_core.ros__parameters.gnss_velocity_calibration_epoch_count",0);
      loc->set("localization_core.ros__parameters.gnss_velocity_calibration_qualified_ratio",0.0);
      loc->set("localization_core.ros__parameters.gnss_velocity_calibration_sync_p95_sec",0.0);
      loc->set("localization_core.ros__parameters.gnss_velocity_calibration_wheel_residual_p95_mps",0.0);
      loc->set("localization_core.ros__parameters.gnss_velocity_calibration_lateral_p95_mps",0.0);
      loc->set("localization_core.ros__parameters.gnss_cog_calibration_epoch_count",0);
      loc->set("localization_core.ros__parameters.gnss_cog_calibration_qualified_ratio",0.0);
      loc->set("localization_core.ros__parameters.gnss_cog_calibration_residual_p95_rad",0.0);
    }
    status_->setText("IMU STAGE-2 CERTIFIED ✓ • GNSS evidence dicabut; EKF GNSS vx+vyaw tetap ON, ulangi straight-run Stage 2");
    apply_->setEnabled(false);
  }
};
class GnssCalibrationPage:public QWidget{
  public:
  GnssCalibrationPage(
  TelemetryStore *t, ReportManager *r,
  const QMap<QString,std::shared_ptr<YamlStore>> &s, QWidget *p=nullptr)
  : QWidget(p), t_(t), r_(r), s_(s)
  {
    auto *l=new QVBoxLayout(this);
    auto *h=new QLabel("GNSS Motion Qualification & Certified Fusion — Stage 2");
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto *d=new QLabel(
    "Live qualification hanya menunjukkan epoch saat ini. Stage 2 mewajibkan satu straight-run "
    "lengkap dengan statistik PASS untuk quality gate autonomy. Estimator selalu memakai GNSS vx+vyaw, "
    "sedangkan absolute yaw EKF hanya dari IMU. COG GNSS divalidasi sebagai diagnostik, bukan yaw source. "
    "Drive odometry dan IMU stationary calibration harus PASS untuk sertifikasi commissioning.");
    d->setWordWrap(true);
    l->addWidget(d);
    auto *bar=new QHBoxLayout();
    stationary_=new QPushButton("▶ Stationary Capture");
    cog_=new QPushButton("▶ Straight Motion Run");
    stop_=new QPushButton("■ Stop + Analisis");
    export_=new QPushButton("Ekspor CSV");
    for(auto *w:{
      stationary_,cog_,stop_,export_
    }) bar->addWidget(w);
    l->addLayout(bar);
    auto *fuse=new QHBoxLayout();
    velEnable_=new QPushButton("Certify GNSS vx + vyaw");
    cogEnable_=new QPushButton("Certify COG Diagnostic");
    disable_=new QPushButton("Restore EKF Sensor Policy");
    revoke_=new QPushButton("Revoke Certifications");
    velPill_=new StatusPill("VEL --");
    cogPill_=new StatusPill("COG --");
    for(auto *w:{
      velEnable_,cogEnable_,disable_,revoke_
    }) fuse->addWidget(w);
    fuse->addWidget(velPill_);
    fuse->addWidget(cogPill_);
    l->addLayout(fuse);
    status_=new QLabel("READY • lakukan IMU + drive odometry calibration lebih dulu");
    status_->setWordWrap(true);
    l->addWidget(status_);
    live_=new QLabel("GNSS live: --");
    live_->setWordWrap(true);
    live_->setObjectName("telemetryCard");
    l->addWidget(live_);
    summary_=new QPlainTextEdit();
    summary_->setReadOnly(true);
    summary_->setMinimumHeight(210);
    l->addWidget(summary_);
    plot_=new LivePlotWidget("GNSS speed: wheel / Doppler / point-fit");
    l->addWidget(plot_,1);
    headingPlot_=new LivePlotWidget("GNSS heading: COG yaw / derived vyaw");
    l->addWidget(headingPlot_,1);
    connect(stationary_,&QPushButton::clicked,this,[this](){
      start("stationary");
    });
    connect(cog_,&QPushButton::clicked,this,[this](){
      start("motion");
    });
    connect(stop_,&QPushButton::clicked,this,[this](){
      analyze();
    });
    connect(export_,&QPushButton::clicked,this,[this](){
      QMessageBox::information(this,"CSV",r_->saveCsv(
      mode_=="motion"?"gnss_stage2_motion_run":"gnss_stationary_accuracy",samples_));
    });
    connect(velEnable_,&QPushButton::clicked,this,[this](){
      certifyVelocity();
    });
    connect(cogEnable_,&QPushButton::clicked,this,[this](){
      certifyCog();
    });
    connect(disable_,&QPushButton::clicked,this,[this](){
      restoreFusionPolicy();
    });
    connect(revoke_,&QPushButton::clicked,this,[this](){
      revokeCertifications();
    });
    timer_=new QTimer(this);
    timer_->setInterval(100);
    connect(timer_,&QTimer::timeout,this,[this](){
      refresh();
    });
    timer_->start();
  }
  private:
  TelemetryStore *t_;
  ReportManager *r_;
  QMap<QString,std::shared_ptr<YamlStore>> s_;
  QPushButton *stationary_,*cog_,*stop_,*export_,*velEnable_,*cogEnable_,*disable_,*revoke_;
  StatusPill *velPill_,*cogPill_;
  QLabel *status_,*live_;
  QPlainTextEdit *summary_;
  LivePlotWidget *plot_,*headingPlot_;
  QTimer *timer_;
  QString mode_="stationary";
  bool rec_=false;
  bool velocityRunPass_=false;
  bool cogRunPass_=false;
  QVector<QVariantMap> samples_;
  QVariantMap result_;
  double lastStamp_=-1.0;
  QVariant localParam(const QString &name,const QVariant &fallback=QVariant())const{
    auto st=s_.value("localization");
    return st?st->get("localization_core.ros__parameters."+name,fallback):fallback;
  }
  bool driveCalibrated()const{
    auto st=s_.value("vehicle");
    return st&&st->get("vehicle.ros__parameters.drive_odometry_calibration_valid",false).toBool();
  }
  bool imuCalibrated()const{
    auto st=s_.value("imu");
    return st&&st->get("data_imu_node.ros__parameters.stationary_calibration_valid",false).toBool();
  }
  bool velocityCertified()const{
    return localParam("gnss_velocity_calibration_valid",false).toBool();
  }
  bool cogCertified()const{
    return localParam("gnss_cog_calibration_valid",false).toBool();
  }
  static double percentile(QVector<double>v,double p){
    QVector<double> finite;
    for(double x:v) if(std::isfinite(x)) finite<<x;
    if(finite.isEmpty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(finite.begin(),finite.end());
    const double x=(finite.size()-1)*std::clamp(p,0.0,1.0);
    const int lo=static_cast<int>(std::floor(x));
    const int hi=static_cast<int>(std::ceil(x));
    return lo==hi?finite[lo]:finite[lo]+(finite[hi]-finite[lo])*(x-lo);
  }
  void start(const QString &m){
    mode_=m;
    samples_.clear();
    result_.clear();
    velocityRunPass_=false;
    cogRunPass_=false;
    rec_=true;
    lastStamp_=-1.0;
    plot_->clear();
    status_->setText(m=="motion"?
    "CAPTURE MOTION • maju lurus stabil; jangan sertifikasi dari belokan/reverse":
    "CAPTURE STATIONARY • kendaraan diam");
  }
  void refresh(){
    const bool liveV=t_->get("gnss_velocity_qualified",false).toBool();
    const bool liveC=t_->get("gnss_cog_qualified",false).toBool();
    const bool certV=velocityCertified();
    const bool certC=cogCertified();
    velPill_->setStatus(certV?"ok":(liveV?"warn":"bad"),
    certV?"VEL CERTIFIED":(liveV?"VEL LIVE PASS":"VEL WAIT"));
    cogPill_->setStatus(certC?"ok":(liveC?"warn":"bad"),
    certC?"COG CERTIFIED":(liveC?"COG LIVE PASS":"COG WAIT"));
    const QVariantMap q=t_->get("gnss_quality").toMap();
    const QVariantMap fit=t_->get("gnss_vel_fit").toMap();
    const QVariantMap base=t_->get("gnss_base_vel").toMap();
    const QVariantMap val=t_->get("gnss_motion_validation").toMap();
    const QVariantMap fix=t_->get("gnss_fix").toMap();
    const QVariantMap gs=t_->get("gnss_status").toMap();
    const double cog=number(q.value("course_enu_rad"));
    const double vyaw=number(gs.value("gnss_vyaw"));
    const bool vyawValid=gs.value("gnss_vyaw_valid").toBool();
    live_->setText(QString("lat %1 | lon %2 | sat %3 | fixType %4 | gnssFixOK %5 | DOP %6 | hAcc %7 m | sAcc %8 m/s\nCOG/yaw %9 deg | base vx %10 m/s | vyaw %11 deg/s (%12) | PVT %13 Hz")
      .arg(variantText(fix.value("lat"))).arg(variantText(fix.value("lon")))
      .arg(variantText(q.value("sat"))).arg(variantText(q.value("fix_type")))
      .arg(q.value("gnss_fix_ok").toBool()?"true":"false")
      .arg(variantText(q.value("dop"))).arg(variantText(q.value("hacc_m")))
      .arg(variantText(q.value("sacc_mps")))
      .arg(std::isfinite(cog)?QString::number(cog*180.0/kPi,'f',2):"--")
      .arg(variantText(base.value("vx")))
      .arg(vyawValid&&std::isfinite(vyaw)?QString::number(vyaw*180.0/kPi,'f',2):"--")
      .arg(vyawValid?"VALID":"LOW-SPEED/INVALID")
      .arg(variantText(q.value("pvt_rate_hz"))));
    QMap<QString,double> pl{
      {
        "Doppler",number(q.value("ground_speed_mps"))
      },
      {
        "Position fit",number(fit.value("speed"))
      },{
        "Base vx",number(base.value("vx"))
      },
      {
        "Wheel vx",number(val.value("wheel_vx_at_measurement_mps"))
      }
    };
    plot_->append(pl);
    QMap<QString,double> hp;
    if(std::isfinite(cog)) hp["COG yaw deg"]=cog*180.0/kPi;
    if(vyawValid&&std::isfinite(vyaw)) hp["GNSS vyaw deg/s"]=vyaw*180.0/kPi;
    if(!hp.isEmpty()) headingPlot_->append(hp);
    if(!rec_) return;
    const double stamp=t_->timestamp("gnss_fix");
    if(stamp<0.0||stamp==lastStamp_) return;
    lastStamp_=stamp;
    QVariantMap row{
      {
        "t_monotonic",stamp
      },{
        "lat",fix.value("lat")
      },
      {
        "lon",fix.value("lon")
      },{
        "alt",fix.value("alt")
      }
    };
    for(const char *k:{
      "sat","dop","hacc_m","vacc_m","sacc_mps","ground_speed_mps",
      "course_enu_rad","course_accuracy_rad","itow_ms","vel_e_mps","vel_n_mps","vel_d_mps",
      "pvt_rate_hz","measurement_age_sec","hdop","vdop","nav_cov_vel_valid"
    }){
      const QString key=QString::fromLatin1(k);
      row[key]=q.value(key);
    }
    for(const char *k:{
      "vx","vy","speed","course_enu_rad"
    }){
      const QString key=QString::fromLatin1(k);
      row[QStringLiteral("fit_")+key]=fit.value(key);
    }
    for(const char *k:{
      "vx","vy","speed"
    }){
      const QString key=QString::fromLatin1(k);
      row[QStringLiteral("base_")+key]=base.value(key);
    }
    for(const char *k:{
      "sync_gap_sec","yaw_at_measurement_rad","yaw_rate_at_measurement_rps",
      "wheel_vx_at_measurement_mps","wheel_minus_gnss_mps","cog_minus_vel_course_rad",
      "fit_minus_gnss_speed_mps","fit_minus_cog_rad","velocity_qualified","cog_qualified",
      "wheel_slip","quality_fresh","velocity_covariance_valid","reject_reason",
      "timestamp_reject_count","covariance_reject_count","sync_reject_count"
    }){
      const QString key=QString::fromLatin1(k);
      row[QStringLiteral("validation_")+key]=val.value(key);
    }
    samples_.push_back(row);
  }
  QVector<double> numericColumn(const QString &key,bool absolute=false)const{
    QVector<double> out;
    for(const auto &r:samples_){
      double x=number(r.value(key));
      if(!std::isfinite(x)||std::abs(x)>100.0) continue;
      if(absolute) x=std::abs(x);
      out<<x;
    }
    return out;
  }
  double trueRatio(const QString &key)const{
    if(samples_.isEmpty()) return 0.0;
    int valid=0,total=0;
    for(const auto&r:samples_){
      if(!r.contains(key)) continue;
      ++total;
      if(r.value(key).toBool()) ++valid;
    }
    return total>0?static_cast<double>(valid)/total:0.0;
  }
  void analyze(){
    rec_=false;
    if(samples_.size()<10){
      status_->setText("FAIL • sampel terlalu sedikit");
      return;
    }
    if(mode_=="stationary"){
      QVector<double> lat,lon;
      for(const auto&r:samples_){
        const double a=number(r.value("lat")),b=number(r.value("lon"));
        if(std::isfinite(a)&&std::isfinite(b)){
          lat<<a;
          lon<<b;
        }
      }
      if(lat.size()<5){
        status_->setText("FAIL • GNSS fix stationary tidak cukup");
        return;
      }
      const double lat0=std::accumulate(lat.begin(),lat.end(),0.0)/lat.size();
      const double lon0=std::accumulate(lon.begin(),lon.end(),0.0)/lon.size();
      QVector<double> rad;
      for(int i=0;
      i<lat.size();
      ++i){
        const double x=6378137.0*(lon[i]-lon0)*kPi/180.0*std::cos(lat0*kPi/180.0);
        const double y=6378137.0*(lat[i]-lat0)*kPi/180.0;
        rad<<std::hypot(x,y);
      }
      double rms=0.0;
      for(double x:rad) rms+=x*x;
      rms=std::sqrt(rms/rad.size());
      result_={
        {
          "mode","stationary"
        },{
          "samples",lat.size()
        },{
          "mean_lat",lat0
        },{
          "mean_lon",lon0
        },
        {
          "rms_m",rms
        },{
          "cep50_m",percentile(rad,.5)
        },{
          "cep95_m",percentile(rad,.95)
        }
      };
      summary_->setPlainText(QString::fromUtf8(
      QJsonDocument(QJsonObject::fromVariantMap(result_)).toJson(QJsonDocument::Indented)));
      status_->setText(QString("STATIONARY • RMS=%1 m • CEP95=%2 m • evidence only, bukan fusion certificate")
      .arg(rms,0,'f',2).arg(percentile(rad,.95),0,'f',2));
      export_->setEnabled(true);
      return;
    }
    const int minVel=localParam("stage2_min_velocity_epochs",50).toInt();
    const int minCog=localParam("stage2_min_cog_epochs",30).toInt();
    const double minVelRatio=number(localParam("stage2_min_velocity_qualified_ratio",0.85),0.85);
    const double minCogRatio=number(localParam("stage2_min_cog_qualified_ratio",0.70),0.70);
    const double maxSync=number(localParam("stage2_max_sync_gap_p95_sec",0.20),0.20);
    const double maxWheel=number(localParam("stage2_max_wheel_gnss_residual_p95_mps",0.25),0.25);
    const double maxLat=number(localParam("stage2_max_lateral_velocity_p95_mps",0.15),0.15);
    const double maxCog=number(localParam("stage2_max_cog_doppler_residual_p95_rad",0.1745329252),0.1745329252);
    const double velRatio=trueRatio("validation_velocity_qualified");
    const double cogRatio=trueRatio("validation_cog_qualified");
    const double covRatio=trueRatio("validation_velocity_covariance_valid");
    const double qualityFreshRatio=trueRatio("validation_quality_fresh");
    const double syncP95=percentile(numericColumn("validation_sync_gap_sec",true),.95);
    const double wheelP95=percentile(numericColumn("validation_wheel_minus_gnss_mps",true),.95);
    const double latP95=percentile(numericColumn("base_vy",true),.95);
    const QVector<double> cogResidual=numericColumn("validation_cog_minus_vel_course_rad",true);
    const double cogP95=percentile(cogResidual,.95);
    const int cogEpochs=cogResidual.size();
    const bool prereq=driveCalibrated()&&imuCalibrated();
    velocityRunPass_=prereq && samples_.size()>=minVel && velRatio>=minVelRatio &&
    covRatio>=minVelRatio && qualityFreshRatio>=minVelRatio &&
    std::isfinite(syncP95)&&syncP95<=maxSync && std::isfinite(wheelP95)&&wheelP95<=maxWheel &&
    std::isfinite(latP95)&&latP95<=maxLat;
    cogRunPass_=velocityRunPass_ && cogEpochs>=minCog && cogRatio>=minCogRatio &&
    std::isfinite(cogP95)&&cogP95<=maxCog;
    result_={
      {
        "mode","motion"
      },{
        "total_epochs",samples_.size()
      },{
        "drive_odometry_calibrated",driveCalibrated()
      },
      {
        "imu_stationary_calibrated",imuCalibrated()
      },{
        "velocity_qualified_ratio",velRatio
      },
      {
        "cog_qualified_ratio",cogRatio
      },{
        "velocity_covariance_valid_ratio",covRatio
      },
      {
        "quality_fresh_ratio",qualityFreshRatio
      },{
        "sync_gap_p95_sec",syncP95
      },
      {
        "wheel_gnss_residual_p95_mps",wheelP95
      },{
        "base_lateral_velocity_p95_mps",latP95
      },
      {
        "cog_residual_epochs",cogEpochs
      },{
        "cog_vs_doppler_p95_deg",cogP95*180.0/kPi
      },
      {
        "velocity_run_pass",velocityRunPass_
      },{
        "cog_run_pass",cogRunPass_
      },
      {
        "limit_min_velocity_epochs",minVel
      },{
        "limit_min_cog_epochs",minCog
      },
      {
        "limit_velocity_ratio",minVelRatio
      },{
        "limit_cog_ratio",minCogRatio
      },
      {
        "limit_sync_p95_sec",maxSync
      },{
        "limit_wheel_residual_p95_mps",maxWheel
      },
      {
        "limit_lateral_p95_mps",maxLat
      },{
        "limit_cog_p95_deg",maxCog*180.0/kPi
      }
    };
    summary_->setPlainText(QString::fromUtf8(
    QJsonDocument(QJsonObject::fromVariantMap(result_)).toJson(QJsonDocument::Indented)));
    status_->setText(QString("MOTION %1 • COG %2 • vel ratio=%3% • wheel p95=%4 m/s • sync p95=%5 s • COG p95=%6°")
    .arg(velocityRunPass_?"PASS":"FAIL").arg(cogRunPass_?"PASS":"WAIT/FAIL")
    .arg(100.0*velRatio,0,'f',1).arg(wheelP95,0,'f',3).arg(syncP95,0,'f',3)
    .arg(cogP95*180.0/kPi,0,'f',2));
    export_->setEnabled(true);
  }
  void certifyVelocity(){
    if(!velocityRunPass_){
      QMessageBox::warning(this,"GNSS Velocity","Straight-run Stage-2 belum PASS. Stop + Analisis terlebih dahulu.");
      return;
    }
    auto st=s_.value("localization");
    if(!st) return;
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_valid",true);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_saved_at",QDateTime::currentDateTime().toString(Qt::ISODate));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_epoch_count",result_.value("total_epochs"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_qualified_ratio",result_.value("velocity_qualified_ratio"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_sync_p95_sec",result_.value("sync_gap_p95_sec"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_wheel_residual_p95_mps",result_.value("wheel_gnss_residual_p95_mps"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_lateral_p95_mps",result_.value("base_lateral_velocity_p95_mps"));
    st->set("localization_core.ros__parameters.enable_global_gnss_velocity_fusion",true);
    st->set("localization_core.ros__parameters.enable_global_gnss_cog_fusion",false);
    st->set("localization_core.ros__parameters.enable_gnss_course_yaw_correction",false);
    status_->setText("GNSS vx+vyaw CERTIFIED ✓ • EKF policy: GNSS motion + IMU absolute yaw");
  }
  void certifyCog(){
    if(!cogRunPass_){
      QMessageBox::warning(this,"GNSS COG","COG straight-run Stage-2 belum PASS.");
      return;
    }
    if(!velocityCertified()&&!velocityRunPass_){
      QMessageBox::warning(this,"GNSS COG","Velocity certification harus PASS lebih dulu.");
      return;
    }
    auto st=s_.value("localization");
    if(!st) return;
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_valid",true);
    st->set("localization_core.ros__parameters.gnss_cog_calibration_valid",true);
    const QString saved=QDateTime::currentDateTime().toString(Qt::ISODate);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_saved_at",saved);
    st->set("localization_core.ros__parameters.gnss_cog_calibration_saved_at",saved);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_epoch_count",result_.value("total_epochs"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_qualified_ratio",result_.value("velocity_qualified_ratio"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_sync_p95_sec",result_.value("sync_gap_p95_sec"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_wheel_residual_p95_mps",result_.value("wheel_gnss_residual_p95_mps"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_lateral_p95_mps",result_.value("base_lateral_velocity_p95_mps"));
    st->set("localization_core.ros__parameters.gnss_cog_calibration_epoch_count",result_.value("cog_residual_epochs"));
    st->set("localization_core.ros__parameters.gnss_cog_calibration_qualified_ratio",result_.value("cog_qualified_ratio"));
    st->set("localization_core.ros__parameters.gnss_cog_calibration_residual_p95_rad",number(result_.value("cog_vs_doppler_p95_deg"),0.0)*kPi/180.0);
    st->set("localization_core.ros__parameters.enable_global_gnss_velocity_fusion",true);
    st->set("localization_core.ros__parameters.enable_global_gnss_cog_fusion",true);
    st->set("localization_core.ros__parameters.enable_gnss_course_yaw_correction",false);
    status_->setText("GNSS COG CERTIFIED ✓ • EKF global: COG absolute yaw + IMU gyro continuity");
  }
  void restoreFusionPolicy(){
    auto st=s_.value("localization");
    if(!st) return;
    st->set("localization_core.ros__parameters.enable_global_gnss_velocity_fusion",true);
    st->set("localization_core.ros__parameters.enable_global_gnss_cog_fusion",true);
    st->set("localization_core.ros__parameters.enable_gnss_course_yaw_correction",false);
    status_->setText("EKF SENSOR POLICY RESTORED ✓ • GNSS=vx, COG=yaw(abs), IMU=vyaw(continuity)");
  }
  void revokeCertifications(){
    if(QMessageBox::question(this,"Revoke Stage-2",
    "Hapus certification velocity + COG? Estimator GNSS vx+vyaw tetap aktif; autonomy quality gate kembali WAIT. Gunakan setelah perubahan mounting, odometry scale, IMU, GNSS lever arm, atau wiring.")!=QMessageBox::Yes) return;
    auto st=s_.value("localization");
    if(!st) return;
    st->set("localization_core.ros__parameters.enable_global_gnss_velocity_fusion",true);
    st->set("localization_core.ros__parameters.enable_global_gnss_cog_fusion",false);
    st->set("localization_core.ros__parameters.enable_gnss_course_yaw_correction",false);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_valid",false);
    st->set("localization_core.ros__parameters.gnss_cog_calibration_valid",false);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_saved_at",QString());
    st->set("localization_core.ros__parameters.gnss_cog_calibration_saved_at",QString());
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_epoch_count",0);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_qualified_ratio",0.0);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_sync_p95_sec",0.0);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_wheel_residual_p95_mps",0.0);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_lateral_p95_mps",0.0);
    st->set("localization_core.ros__parameters.gnss_cog_calibration_epoch_count",0);
    st->set("localization_core.ros__parameters.gnss_cog_calibration_qualified_ratio",0.0);
    st->set("localization_core.ros__parameters.gnss_cog_calibration_residual_p95_rad",0.0);
    velocityRunPass_=false;
    cogRunPass_=false;
    st->set("localization_core.ros__parameters.enable_gnss_course_yaw_correction",false);
    status_->setText("STAGE-2 CERTIFICATION REVOKED • estimator GNSS vx+vyaw tetap ON; autonomy gate WAIT");
  }
};
