// Extracted from agv_gui.cpp for maintainability.
class CameraPage:public QWidget{
  public:CameraPage(TelemetryStore*t,ReportManager*r,const QMap<QString,std::shared_ptr<YamlStore>>&s,QWidget*p=nullptr):QWidget(p),t_(t),r_(r),s_(s){
    per_=s_.value("perception");
    gui_=s_.value("gui");
    auto*l=new QVBoxLayout(this);
    auto*h=new QLabel("Kamera Persepsi — Metric Calibration & Safety");
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto*d=new QLabel("Ground Plane, Obstacle ROI, dan Lane ROI dapat ditarik langsung. Perubahan disimpan ke YAML. Certification production memerlukan camera health, geometri metric valid, jumlah titik validasi, dan RMSE yang memenuhi batas.");
    d->setWordWrap(true);
    l->addWidget(d);
    auto*vis=new QHBoxLayout();
    ground_=new QCheckBox("Ground Plane");
    obstacle_=new QCheckBox("Obstacle ROI");
    lane_=new QCheckBox("Safety Jalur");
    ground_->setChecked(gui_->get("camera_overlay.show_ground_plane",true).toBool());
    obstacle_->setChecked(gui_->get("camera_overlay.show_obstacle_roi",true).toBool());
    lane_->setChecked(gui_->get("camera_overlay.show_lane_safety",true).toBool());
    vis->addWidget(ground_);
    vis->addWidget(obstacle_);
    vis->addWidget(lane_);
    vis->addStretch();
    l->addLayout(vis);
    canvas_=new CameraCalibrationCanvas(s_);
    l->addWidget(canvas_,1);
    auto*metric=new QGroupBox("Metric Safety Validation");
    auto*mg=new QGridLayout(metric);
    rmse_=new QDoubleSpinBox();
    rmse_->setRange(0,10);
    rmse_->setDecimals(4);
    limit_=new QDoubleSpinBox();
    limit_->setRange(0.001,5);
    limit_->setValue(number(gui_->get("camera_metric_validation.max_rmse_m",0.20),0.20));
    points_=new QSpinBox();
    points_->setRange(0,10000);
    auto*cert=new QPushButton("✓ Certify Metric Safety");
    auto*revokeBtn=new QPushButton("Revoke Safety");
    metricSummary_=new QPlainTextEdit();
    metricSummary_->setReadOnly(true);
    metricSummary_->setMaximumHeight(110);
    mg->addWidget(new QLabel("Measured RMSE (m)"),0,0);
    mg->addWidget(rmse_,0,1);
    mg->addWidget(new QLabel("PASS limit (m)"),0,2);
    mg->addWidget(limit_,0,3);
    mg->addWidget(new QLabel("Validation points"),1,0);
    mg->addWidget(points_,1,1);
    mg->addWidget(cert,1,2);
    mg->addWidget(revokeBtn,1,3);
    mg->addWidget(metricSummary_,2,0,1,4);
    l->addWidget(metric);
    auto*bottom=new QHBoxLayout();
    record_=new QPushButton("● Rekam Runtime CSV");
    auto*frame=new QPushButton("Ekspor Frame PNG");
    auto*overlay=new QPushButton("Ekspor Overlay PNG");
    bottom->addWidget(record_);
    bottom->addWidget(frame);
    bottom->addWidget(overlay);
    bottom->addStretch();
    l->addLayout(bottom);
    connect(ground_,&QCheckBox::toggled,this,[this](bool v){
      gui_->set("camera_overlay.show_ground_plane",v);
      canvas_->setVisibleLayer("ground",v);
    });
    connect(obstacle_,&QCheckBox::toggled,this,[this](bool v){
      gui_->set("camera_overlay.show_obstacle_roi",v);
      canvas_->setVisibleLayer("obstacle",v);
    });
    connect(lane_,&QCheckBox::toggled,this,[this](bool v){
      gui_->set("camera_overlay.show_lane_safety",v);
      canvas_->setVisibleLayer("lane",v);
    });
    connect(canvas_,&CameraCalibrationCanvas::pointsChanged,this,[this](const QString&layer,const QVariantList&pts){
      savePoints(layer,pts);
    });
    connect(cert,&QPushButton::clicked,this,[this](){
      certify();
    });
    connect(revokeBtn,&QPushButton::clicked,this,[this](){
      this->revoke();
    });
    connect(record_,&QPushButton::clicked,this,[this](){
      toggleRecord();
    });
    connect(frame,&QPushButton::clicked,this,[this](){
      if(last_.isNull())return;
      QString p=QDir(r_->root()).filePath("perception_frame_"+nowStamp()+".png");
      last_.save(p);
      QMessageBox::information(this,"PNG",p);
    });
    connect(overlay,&QPushButton::clicked,this,[this](){
      QMessageBox::information(this,"PNG",r_->savePng("camera_calibration_overlay",canvas_->view()));
    });
    timer_=new QTimer(this);
    timer_->setInterval(250);
    connect(timer_,&QTimer::timeout,this,[this](){
      refresh();
    });
    timer_->start();
    refreshMetric();
  }
  void setImage(const QImage&i){
    last_=i.copy();
    canvas_->setImage(last_);
  }
  private:TelemetryStore*t_;
  ReportManager*r_;
  QMap<QString,std::shared_ptr<YamlStore>>s_;
  std::shared_ptr<YamlStore>per_,gui_;
  CameraCalibrationCanvas*canvas_;
  QCheckBox*ground_,*obstacle_,*lane_;
  QDoubleSpinBox*rmse_,*limit_;
  QSpinBox*points_;
  QPlainTextEdit*metricSummary_;
  QPushButton*record_;
  QTimer*timer_;
  QImage last_;
  bool rec_=false;
  QVector<QVariantMap>rows_;
  void savePoints(const QString&layer,const QVariantList&pts){
    if(layer=="ground"){
      per_->set("perception.ros__parameters.ground_src_points",pts);
      applyObstacle();
      applyLane();
    }
    else if(layer=="obstacle"){
      gui_->set("camera_overlay.obstacle_roi_points_px",pts);
      applyObstacle();
    }
    else if(layer=="lane"){
      gui_->set("camera_overlay.lane_safety_points_px",pts);
      applyLane();
    }
    refreshMetric();
  }
  void applyObstacle(){
    auto m=canvas_->metricPoints("obstacle");
    if(m.size()!=4)return;
    double fmin=1e9,fmax=-1e9,lat=0;
    for(auto&p:m){
      fmin=std::min(fmin,p.x());
      fmax=std::max(fmax,p.x());
      lat=std::max(lat,std::abs(p.y()));
    }
    fmin=std::max(0.0,fmin);
    fmax=std::max(fmin+0.05,fmax);
    lat=std::max(.1,lat);
    per_->set("perception.ros__parameters.metric_minimum_forward_m",fmin);
    per_->set("perception.ros__parameters.metric_maximum_forward_m",fmax);
    per_->set("perception.ros__parameters.metric_maximum_abs_left_m",lat);
    per_->set("perception.ros__parameters.obstacle_forward_min_m",fmin);
    per_->set("perception.ros__parameters.obstacle_forward_max_m",fmax);
  }
  void applyLane(){
    auto m=canvas_->metricPoints("lane");
    if(m.size()!=4)return;
    double near=(m[0].x()+m[1].x())/2,far=(m[2].x()+m[3].x())/2;
    if(near>far)std::swap(near,far);
    near=std::max(.05,near);
    far=std::max(near+.1,far);
    double w=(std::abs(m[0].y()-m[1].y())+std::abs(m[3].y()-m[2].y()))/2;
    w=std::max(.6,w);
    per_->set("perception.ros__parameters.lane_lookahead_near_m",near);
    per_->set("perception.ros__parameters.lane_lookahead_far_m",far);
    per_->set("perception.ros__parameters.nominal_road_width_m",w);
  }
  bool geometry(QString*detail=nullptr){
    double mppx=number(per_->get("perception.ros__parameters.ground_meters_per_pixel_x")),mppy=number(per_->get("perception.ros__parameters.ground_meters_per_pixel_y")),fmin=number(per_->get("perception.ros__parameters.metric_minimum_forward_m")),fmax=number(per_->get("perception.ros__parameters.metric_maximum_forward_m")),lat=number(per_->get("perception.ros__parameters.metric_maximum_abs_left_m"));
    bool ok=mppx>0&&mppy>0&&lat>0&&fmin>=0&&fmax>fmin&&per_->get("perception.ros__parameters.ground_src_points",QVariantList{
    }).toList().size()==8;
    if(detail)*detail=QString("mpp=(%1,%2) • forward=%3..%4 • lateral=%5").arg(mppx).arg(mppy).arg(fmin).arg(fmax).arg(lat);
    return ok;
  }
  void refreshMetric(){
    QString d;
    geometry(&d);
    metricSummary_->setPlainText(d+QString("\nCamera connected=%1 • health=%2 • certified=%3").arg(t_->get("connected.camera",false).toBool()).arg(t_->get("camera_healthy",false).toBool()).arg(gui_->get("camera_metric_validation.certified",false).toBool()));
  }
  void certify(){
    QString d;
    bool ok=geometry(&d)&&t_->get("connected.camera",false).toBool()&&t_->get("camera_healthy",false).toBool()&&points_->value()>=6&&rmse_->value()<=limit_->value();
    if(!ok){
      QMessageBox::warning(this,"Metric safety","Belum lulus.\n"+d);
      return;
    }
    if(QMessageBox::question(this,"Certify","Aktifkan camera metric gate + Collision Monitor production?")!=QMessageBox::Yes)return;
    auto nav=s_.value("navigation_core");
    nav->set("navigation_core.ros__parameters.require_camera_metric_calibration",true);
    nav->set("navigation_core.ros__parameters.camera_metric_calibration_validated",true);
    nav->set("navigation_core.ros__parameters.collision_monitor_enabled",true);
    per_->set("perception.ros__parameters.camera_metric_calibration_validated",true);
    gui_->set("camera_metric_validation.last_rmse_m",rmse_->value());
    gui_->set("camera_metric_validation.max_rmse_m",limit_->value());
    gui_->set("camera_metric_validation.validation_points",points_->value());
    gui_->set("camera_metric_validation.certified",true);
    gui_->set("camera_metric_validation.certified_at",QDateTime::currentDateTime().toString(Qt::ISODate));
    QMessageBox::information(this,"Certified","Safety certified. Restart autonomous.launch.");
    refreshMetric();
  }
  void revoke(){
    auto nav=s_.value("navigation_core");
    nav->set("navigation_core.ros__parameters.camera_metric_calibration_validated",false);
    nav->set("navigation_core.ros__parameters.collision_monitor_enabled",false);
    per_->set("perception.ros__parameters.camera_metric_calibration_validated",false);
    gui_->set("camera_metric_validation.certified",false);
    gui_->set("camera_metric_validation.certified_at",QString());
    refreshMetric();
  }
  void toggleRecord(){
    if(!rec_){
      rows_.clear();
      rec_=true;
      record_->setText("■ Stop + Simpan CSV");
    }
    else{
      rec_=false;
      record_->setText("● Rekam Runtime CSV");
      QMessageBox::information(this,"CSV",r_->saveCsv("perception_camera_runtime",rows_));
    }
  }
  void refresh(){
    refreshMetric();
    if(rec_)rows_<<QVariantMap{
      {
        "time",QDateTime::currentDateTime().toString(Qt::ISODateWithMs)
      },{
        "camera_connected",t_->get("connected.camera")
      },{
        "camera_healthy",t_->get("camera_healthy")
      },{
        "performance",t_->get("perception_performance")
      },{
        "raw_detections",t_->get("raw_detections")
      }
    };
  }
};
class ReportsPage:public QWidget{
  public:ReportsPage(ReportManager*r,const QMap<QString,std::shared_ptr<YamlStore>>&s,QWidget*p=nullptr):QWidget(p),r_(r),s_(s){
    auto*l=new QVBoxLayout(this);
    auto*h=new QLabel("Pelaporan & Bukti Regresi");
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto*d=new QLabel("Semua output disimpan di folder laporan. Rosbag merekam topic sensor, localization, command, safety dan GNSS fusion. Snapshot konfigurasi menyimpan YAML persis yang dipakai pada sesi pengujian.");
    d->setWordWrap(true);
    l->addWidget(d);
    folder_=new QLabel(r_->root());
    folder_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    l->addWidget(folder_);
    auto*g=new QGridLayout();
    startBag_=new QPushButton("● Mulai Rosbag");
    stopBag_=new QPushButton("■ Hentikan Rosbag");
    profile_=new QPushButton("Profil CPU/GPU 60 dtk");
    rank_=new QPushButton("Peringkat Eksperimen");
    snapshot_=new QPushButton("Snapshot Semua Konfigurasi");
    open_=new QPushButton("Buka Folder");
    choose_=new QPushButton("Pilih Folder");
    refresh_=new QPushButton("Refresh File");
    g->addWidget(startBag_,0,0);
    g->addWidget(stopBag_,0,1);
    g->addWidget(profile_,0,2);
    g->addWidget(rank_,1,0);
    g->addWidget(snapshot_,1,1);
    g->addWidget(open_,1,2);
    g->addWidget(choose_,2,0);
    g->addWidget(refresh_,2,1);
    l->addLayout(g);
    table_=new QTableWidget(0,4);
    table_->setHorizontalHeaderLabels({
      "File","Tipe","Ukuran","Diubah"
    });
    table_->horizontalHeader()->setSectionResizeMode(0,QHeaderView::Stretch);
    l->addWidget(table_,1);
    status_=new QPlainTextEdit();
    status_->setReadOnly(true);
    status_->setMaximumHeight(170);
    l->addWidget(status_);
    connect(startBag_,&QPushButton::clicked,this,[this](){
      startRosbag();
    });
    connect(stopBag_,&QPushButton::clicked,this,[this](){
      stopRosbag(false);
    });
    connect(profile_,&QPushButton::clicked,this,[this](){
      startProfile();
    });
    connect(rank_,&QPushButton::clicked,this,[this](){
      rankExperiments();
    });
    connect(snapshot_,&QPushButton::clicked,this,[this](){
      snapshotConfigs();
    });
    connect(open_,&QPushButton::clicked,this,[this](){
      QProcess::startDetached("xdg-open",{
        r_->root()
      });
    });
    connect(choose_,&QPushButton::clicked,this,[this](){
      QString dir=QFileDialog::getExistingDirectory(this,"Folder output laporan",r_->root());
      if(!dir.isEmpty()&&s_.contains("gui")){
        s_["gui"]->set("reporting.output_directory",dir);
        folder_->setText(dir);
        refreshFiles();
      }
    });
    connect(refresh_,&QPushButton::clicked,this,[this](){
      refreshFiles();
    });
    refreshFiles();
  }
  ~ReportsPage()override{
    stopRosbag(true);
    stopProfile();
  }
  void stopRosbagSilent(){
    stopRosbag(true);
  }
  private:ReportManager*r_;
  QMap<QString,std::shared_ptr<YamlStore>>s_;
  QLabel*folder_;
  QPushButton*startBag_,*stopBag_,*profile_,*rank_,*snapshot_,*open_,*choose_,*refresh_;
  QTableWidget*table_;
  QPlainTextEdit*status_;
  QProcess*bag_=nullptr,*prof_=nullptr;
  QTimer*profTimer_=nullptr;
  QString profilePath_;
  void refreshFiles(){
    QDir dir(r_->root());
    folder_->setText(dir.absolutePath());
    QFileInfoList files=dir.entryInfoList(QDir::Files|QDir::Dirs|QDir::NoDotAndDotDot,QDir::Time);
    if(files.size()>200)files=files.mid(0,200);
    table_->setRowCount(files.size());
    for(int row=0;
    row<files.size();
    ++row){
      const QFileInfo&fi=files[row];
      QString type=fi.isDir()?"DIR":fi.suffix().toUpper();
      QString size=fi.isDir()?"--":QString::number(fi.size()/1024.0,'f',1)+" KB";
      QStringList vals={
        fi.fileName(),type,size,fi.lastModified().toString(Qt::ISODate)
      };
      for(int col=0;
      col<4;
      ++col)table_->setItem(row,col,new QTableWidgetItem(vals[col]));
    }
  }
  QStringList bagTopics()const{
    return {
      "/tf","/tf_static","/scan","/imu/data","/odom","/esc/odom","/odometry/filtered",
      "/map","/amcl_pose","/particle_cloud","/plan","/local_plan",
      "/global_costmap/costmap","/local_costmap/costmap","/goal_pose",
      "/cmd_vel_nav_smoothed","/cmd_vel/autonomy_integrated","/cmd_vel/nav2_pre_collision",
      "/cmd_vel/collision_preview","/cmd_vel","/cmd_vel/actuator",
      "/esc/drive_target_mps","/esc/drive_actual_mps","/esc/steering_target_rad",
      "/esc/steering_actual_rad","/navigation/mppi_closed_loop/status",
      "/navigation/trajectory_safety_state","/collision_monitor/state",
      "/system/localization_state","/navigation/goal_state"
    };
  }
  void startRosbag(){
    if(bag_&&bag_->state()!=QProcess::NotRunning)return;
    QString out=QDir(r_->root()).filePath("rosbag_"+nowStamp());
    bag_=new QProcess(this);
    QStringList args={
      "bag","record","-o",out
    };
    args<<bagTopics();
    bag_->setProgram("ros2");
    bag_->setArguments(args);
    bag_->setProcessChannelMode(QProcess::MergedChannels);
    connect(bag_,&QProcess::readyRead,this,[this](){
      status_->appendPlainText(QString::fromUtf8(bag_->readAll()));
    });
    bag_->start();
    status_->appendPlainText("Rosbag START: "+out);
  }
  void stopRosbag(bool silent){
    if(!bag_||bag_->state()==QProcess::NotRunning)return;
    #ifdef Q_OS_UNIX
    ::kill(static_cast<pid_t>(bag_->processId()),SIGINT);
    #endif
    if(!bag_->waitForFinished(5000)){
      bag_->terminate();
      bag_->waitForFinished(1500);
    }
    refreshFiles();
    if(!silent)QMessageBox::information(this,"Rosbag","Rosbag dihentikan dan metadata difinalisasi.");
  }
  void startProfile(){
    if(prof_&&prof_->state()!=QProcess::NotRunning)return;
    profilePath_=QDir(r_->root()).filePath("tegrastats_"+nowStamp()+".txt");
    prof_=new QProcess(this);
    prof_->setProgram("tegrastats");
    prof_->setArguments({
      "--interval","1000"
    });
    prof_->setProcessChannelMode(QProcess::MergedChannels);
    QFile*file=new QFile(profilePath_,prof_);
    file->open(QIODevice::WriteOnly|QIODevice::Text);
    connect(prof_,&QProcess::readyRead,this,[prof=prof_,file](){
      file->write(prof->readAll());
      file->flush();
    });
    prof_->start();
    profTimer_=new QTimer(this);
    profTimer_->setSingleShot(true);
    connect(profTimer_,&QTimer::timeout,this,[this](){
      stopProfile();
      QMessageBox::information(this,"Runtime profile",profilePath_);
    });
    profTimer_->start(60000);
    status_->appendPlainText("Profiling 60 s: "+profilePath_);
  }
  void stopProfile(){
    if(prof_&&prof_->state()!=QProcess::NotRunning){
      prof_->terminate();
      prof_->waitForFinished(1000);
    }
    if(profTimer_)profTimer_->stop();
  }
  void snapshotConfigs(){
    QString dir=QDir(r_->root()).filePath("config_snapshot_"+nowStamp());
    QDir().mkpath(dir);
    for(auto it=s_.cbegin();
    it!=s_.cend();
    ++it)if(!it.value()->path().isEmpty())QFile::copy(it.value()->path(),QDir(dir).filePath(it.key()+"_"+QFileInfo(it.value()->path()).fileName()));
    refreshFiles();
    QMessageBox::information(this,"Snapshot",dir);
  }
  void rankExperiments(){
    QDir dir(r_->root());
    QStringList metas=dir.entryList({
      "*.meta.json"
    },QDir::Files,QDir::Time);
    QString out=dir.filePath("experiment_ranking_"+nowStamp()+".csv");
    QSaveFile f(out);
    if(!f.open(QIODevice::WriteOnly|QIODevice::Text))return;
    QTextStream ts(&f);
    ts<<"file,session_id,category,variant,row_count\n";
    for(const QString&fn:metas){
      QFile in(dir.filePath(fn));
      if(!in.open(QIODevice::ReadOnly))continue;
      QJsonDocument doc=QJsonDocument::fromJson(in.readAll());
      QJsonObject o=doc.object();
      ts<<csvEscape(fn)<<','<<csvEscape(o.value("session_id").toString())<<','<<csvEscape(o.value("experiment_category").toString())<<','<<csvEscape(o.value("experiment_variant").toString())<<','<<o.value("row_count").toInt()<<'\n';
    }
    f.commit();
    refreshFiles();
    QMessageBox::information(this,"Experiment index",out);
  }
};
