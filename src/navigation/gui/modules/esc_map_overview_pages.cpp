// Extracted from agv_gui.cpp for maintainability.
class EscCalibrationPage:public QWidget{
  public:EscCalibrationPage(const QMap<QString,std::shared_ptr<YamlStore>>&s,TelemetryStore*t,ReportManager*r,RosBridge*ros,QWidget*p=nullptr):QWidget(p),s_(s),t_(t),r_(r),ros_(ros){
    auto*l=new QVBoxLayout(this);
    auto*h=new QLabel("ESC / Odometry Calibration — Distance & Ackermann");
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto*d=new QLabel("Ambil Titik Awal/End untuk referensi odometri, tetapi masukkan X/Y ground-truth yang diukur independen. Straight trial mengkalibrasi skala drive. Circle trial kiri/kanan mengidentifikasi effective kinematic wheelbase dan certified minimum turning radius.");
    d->setWordWrap(true);
    l->addWidget(d);
    auto*form=new QGridLayout();
    name_=new QLineEdit("Trial");
    realX_=new QDoubleSpinBox();
    realY_=new QDoubleSpinBox();
    realYaw_=new QDoubleSpinBox();
    realSteer_=new QDoubleSpinBox();
    for(auto*w:{
      realX_,realY_
    }){
      w->setRange(-1000,1000);
      w->setDecimals(4);
    }
    realYaw_->setRange(-360,360);
    realSteer_->setRange(-180,180);
    form->addWidget(new QLabel("Nama"),0,0);
    form->addWidget(name_,0,1);
    form->addWidget(new QLabel("Real X forward (m)"),1,0);
    form->addWidget(realX_,1,1);
    form->addWidget(new QLabel("Real Y left (m)"),1,2);
    form->addWidget(realY_,1,3);
    form->addWidget(new QLabel("Real Δyaw (deg)"),2,0);
    form->addWidget(realYaw_,2,1);
    form->addWidget(new QLabel("Sudut roda nyata (deg)"),2,2);
    form->addWidget(realSteer_,2,3);
    l->addLayout(form);
    auto*captureBar=new QHBoxLayout();
    start_=new QPushButton("Ambil Titik Awal");
    end_=new QPushButton("Ambil Titik Akhir");
    save_=new QPushButton("Simpan Trial");
    new_=new QPushButton("Trial Baru");
    load_=new QPushButton("Muat Trial");
    delete_=new QPushButton("Hapus Trial");
    clear_=new QPushButton("Hapus Semua");
    for(auto*w:{
      start_,end_,save_,new_,load_,delete_,clear_
    })captureBar->addWidget(w);
    l->addLayout(captureBar);
    auto*calBar=new QHBoxLayout();
    calDrive_=new QPushButton("Kalibrasi Skala Drive");
    calWheel_=new QPushButton("Kalibrasi Wheelbase Efektif");
    calAll_=new QPushButton("Kalibrasi Drive + Wheelbase");
    export_=new QPushButton("Ekspor CSV");
    exportPng_=new QPushButton("Ekspor PNG");
    for(auto*w:{
      calDrive_,calWheel_,calAll_,export_,exportPng_
    })calBar->addWidget(w);
    l->addLayout(calBar);
    auto*circleBox=new QGroupBox("Part 3 — Circle Test & Kinematic Geometry Certification");
    auto*circle=new QGridLayout(circleBox);
    circleMargin_=new QDoubleSpinBox();
    circleMargin_->setRange(0.0,30.0);
    circleMargin_->setDecimals(1);
    circleMargin_->setSuffix(" %");
    circleMargin_->setValue(number(s_.value("vehicle")->get("vehicle.ros__parameters.turning_radius_safety_margin_pct",10.0),10.0));
    circleRmseMax_=new QDoubleSpinBox();
    circleRmseMax_->setRange(0.01,2.0);
    circleRmseMax_->setDecimals(3);
    circleRmseMax_->setSuffix(" m");
    circleRmseMax_->setValue(0.25);
    analyzeCircle_=new QPushButton("Analisis Circle Test");
    applyCircle_=new QPushButton("TERAPKAN GEOMETRI PART 3");
    applyCircle_->setObjectName("primaryButton");
    exportCircle_=new QPushButton("Ekspor Circle CSV");
    circleStatus_=new QLabel("Belum dianalisis");
    circleStatus_->setWordWrap(true);
    circle->addWidget(new QLabel("Safety margin radius"),0,0);
    circle->addWidget(circleMargin_,0,1);
    circle->addWidget(new QLabel("Maks RMSE fit"),0,2);
    circle->addWidget(circleRmseMax_,0,3);
    circle->addWidget(analyzeCircle_,1,0);
    circle->addWidget(applyCircle_,1,1);
    circle->addWidget(exportCircle_,1,2);
    circle->addWidget(circleStatus_,2,0,1,4);
    l->addWidget(circleBox);
    live_=new QLabel("Belum capture");
    live_->setWordWrap(true);
    l->addWidget(live_);
    table_=new QTableWidget(0,10);
    table_->setHorizontalHeaderLabels({
      "Name","Odom dist","Odom yaw°","Real X","Real Y","Real yaw°","Steer°","R center (m)","L eff (m)","Time"
    });
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    l->addWidget(table_,1);
    connect(start_,&QPushButton::clicked,this,[this](){
      captureStart();
    });
    connect(end_,&QPushButton::clicked,this,[this](){
      captureEnd();
    });
    connect(save_,&QPushButton::clicked,this,[this](){
      saveTrial();
    });
    connect(new_,&QPushButton::clicked,this,[this](){
      newTrial();
    });
    connect(load_,&QPushButton::clicked,this,[this](){
      loadSelectedTrial();
    });
    connect(delete_,&QPushButton::clicked,this,[this](){
      deleteSelectedTrial();
    });
    connect(clear_,&QPushButton::clicked,this,[this](){
      clearTrials();
    });
    connect(calDrive_,&QPushButton::clicked,this,[this](){
      calibrateDrive();
    });
    connect(calWheel_,&QPushButton::clicked,this,[this](){
      calibrateWheelbase();
    });
    connect(calAll_,&QPushButton::clicked,this,[this](){
      calibrateAll();
    });
    connect(export_,&QPushButton::clicked,this,[this](){
      QMessageBox::information(this,"CSV",r_->saveCsv("esc_calibration_trials",trials()));
    });
    connect(exportPng_,&QPushButton::clicked,this,[this](){
      QMessageBox::information(this,"PNG",r_->savePng("esc_calibration",this));
    });
    connect(table_,&QTableWidget::cellDoubleClicked,this,[this](int,int){
      loadSelectedTrial();
    });
    connect(analyzeCircle_,&QPushButton::clicked,this,[this](){
      analyzeCircle(false);
    });
    connect(applyCircle_,&QPushButton::clicked,this,[this](){
      analyzeCircle(true);
    });
    connect(exportCircle_,&QPushButton::clicked,this,[this](){
      exportCircleCsv();
    });
    refreshTable();
    refreshCircleSaved();
  }
  private:QMap<QString,std::shared_ptr<YamlStore>>s_;
  TelemetryStore*t_;
  ReportManager*r_;
  RosBridge*ros_;
  QLineEdit*name_;
  QDoubleSpinBox*realX_,*realY_,*realYaw_,*realSteer_,*circleMargin_,*circleRmseMax_;
  QPushButton*start_,*end_,*save_,*new_,*load_,*delete_,*clear_,*calDrive_,*calWheel_,*calAll_,*export_,*exportPng_,*analyzeCircle_,*applyCircle_,*exportCircle_;
  QLabel*live_,*circleStatus_;
  QTableWidget*table_;
  QVariantMap begin_,endPose_;
  struct CircleFit{
    bool valid=false;
    double leff=0,leftLeff=0,rightLeff=0,leftRadius=0,rightRadius=0,certifiedRadius=0,rmse=0;
    int nLeft=0,nRight=0;
    QVector<QVariantMap> rows;
    QString reason;
  };
  QVariantMap odom(){
    return t_->get("esc_odom").toMap();
  }
  QVector<QVariantMap>trials()const{
    QVector<QVariantMap>out;
    for(const QVariant&v:s_.value("gui")->get("esc_calibration.trials",QVariantList{
    }).toList())out<<v.toMap();
    return out;
  }
  void captureStart(){
    begin_=odom();
    if(begin_.isEmpty())QMessageBox::warning(this,"ESC Odom","/esc/odom belum tersedia");
    else live_->setText(QString("START x=%1 y=%2 yaw=%3").arg(begin_["x"].toDouble()).arg(begin_["y"].toDouble()).arg(begin_["yaw"].toDouble()));
  }
  void captureEnd(){
    endPose_=odom();
    if(begin_.isEmpty()||endPose_.isEmpty())return;
    double dx=endPose_["x"].toDouble()-begin_["x"].toDouble(),dy=endPose_["y"].toDouble()-begin_["y"].toDouble(),yaw0=begin_["yaw"].toDouble(),c=std::cos(yaw0),ss=std::sin(yaw0),lx=c*dx+ss*dy,ly=-ss*dx+c*dy,dyaw=normalizeAngle(endPose_["yaw"].toDouble()-yaw0),chord=std::hypot(lx,ly),arc=chord;
    if(std::abs(dyaw)>kPi/180.0){
      double den=2*std::sin(std::abs(dyaw)/2);
      if(std::abs(den)>1e-9)arc=std::abs(chord/den*dyaw);
    }
    capture_={
      {
        "odom_dx",lx
      },{
        "odom_dy",ly
      },{
        "odom_dyaw_rad",dyaw
      },{
        "odom_distance_m",arc
      },{
        "steering_actual_rad",t_->get("esc_steer_actual")
      },{
        "speed_actual_mps",t_->get("esc_drive_actual")
      }
    };
    live_->setText(QString("END • odom dist=%1 m • Δyaw=%2° • local endpoint=(%3,%4)").arg(arc,0,'f',3).arg(dyaw*180/kPi,0,'f',2).arg(lx,0,'f',3).arg(ly,0,'f',3));
  }
  QVariantMap capture_;
  void saveTrial(){
    if(capture_.isEmpty()){
      QMessageBox::warning(this,"Trial","Ambil Titik Awal/End dahulu.");
      return;
    }
    QVariantMap row=capture_;
    row["name"]=name_->text();
    row["real_x_m"]=realX_->value();
    row["real_y_m"]=realY_->value();
    row["real_yaw_deg"]=realYaw_->value();
    row["real_steering_deg"]=realSteer_->value();
    row["created"]=QDateTime::currentDateTime().toString(Qt::ISODate);
    QVariantList list=s_.value("gui")->get("esc_calibration.trials",QVariantList{
    }).toList();
    list<<row;
    s_.value("gui")->set("esc_calibration.trials",list);
    refreshTable();
  }
  void newTrial(){
    begin_.clear();
    endPose_.clear();
    capture_.clear();
    name_->setText(QString("Trial_%1").arg(trials().size()+1));
    realX_->setValue(0.0);
    realY_->setValue(0.0);
    realYaw_->setValue(0.0);
    realSteer_->setValue(0.0);
    table_->clearSelection();
    live_->setText("Trial baru siap. Ambil Titik Awal, jalankan kendaraan, lalu Ambil Titik Akhir.");
  }
  void loadSelectedTrial(){
    int row=table_->currentRow();
    auto tr=trials();
    if(row<0||row>=tr.size()){
      QMessageBox::information(this,"Muat Trial","Pilih satu baris trial terlebih dahulu.");
      return;
    }
    const auto&r=tr[row];
    name_->setText(r.value("name").toString());
    realX_->setValue(number(r.value("real_x_m")));
    realY_->setValue(number(r.value("real_y_m")));
    realYaw_->setValue(number(r.value("real_yaw_deg")));
    realSteer_->setValue(number(r.value("real_steering_deg")));
    capture_=r;
    live_->setText(QString("Trial dimuat: %1 • odom=%2 m • yaw=%3°").arg(name_->text()).arg(number(r.value("odom_distance_m")),0,'f',3).arg(number(r.value("odom_dyaw_rad"))*180.0/kPi,0,'f',2));
  }
  void deleteSelectedTrial(){
    int row=table_->currentRow();
    QVariantList list=s_.value("gui")->get("esc_calibration.trials",QVariantList{
    }).toList();
    if(row<0||row>=list.size()){
      QMessageBox::information(this,"Hapus Trial","Pilih trial yang akan dihapus.");
      return;
    }
    if(QMessageBox::question(this,"Hapus Trial",QString("Hapus trial '%1'?").arg(list[row].toMap().value("name").toString()))!=QMessageBox::Yes)return;
    list.removeAt(row);
    s_.value("gui")->set("esc_calibration.trials",list);
    newTrial();
    refreshTable();
  }
  void clearTrials(){
    if(QMessageBox::question(this,"Hapus Semua Trial","Hapus seluruh data trial ESC/odometri? Tindakan ini tidak dapat dibatalkan.")!=QMessageBox::Yes)return;
    s_.value("gui")->set("esc_calibration.trials",QVariantList{
    });
    newTrial();
    refreshTable();
  }
  void refreshTable(){
    auto tr=trials();
    double track=number(s_.value("vehicle")->get("vehicle.ros__parameters.track_width_m",0.48),0.48);
    table_->setRowCount(tr.size());
    for(int i=0;
    i<tr.size();
    ++i){
      const auto&r=tr[i];
      double x=number(r.value("real_x_m")),y=number(r.value("real_y_m")),steer=number(r.value("real_steering_deg"),NAN);
      if(!std::isfinite(steer)||std::abs(steer)<1.0)steer=number(r.value("steering_actual_rad"),NAN)*180.0/kPi;
      double den=x*x+y*y,rc=(den>=0.25&&std::abs(y)>=0.05)?den/(2.0*std::abs(y)):NAN,leff=(std::isfinite(rc)&&std::isfinite(steer)&&std::abs(steer)>=3.0)?(rc-0.5*track)*std::tan(std::abs(steer)*kPi/180.0):NAN;
      QStringList vals={
        r.value("name").toString(),variantText(r.value("odom_distance_m")),QString::number(number(r.value("odom_dyaw_rad"))*180/kPi,'f',2),variantText(r.value("real_x_m")),variantText(r.value("real_y_m")),variantText(r.value("real_yaw_deg")),QString::number(steer,'f',2),std::isfinite(rc)?QString::number(rc,'f',3):"--",std::isfinite(leff)?QString::number(leff,'f',4):"--",r.value("created").toString()
      };
      for(int c=0;
      c<vals.size();
      ++c)table_->setItem(i,c,new QTableWidgetItem(vals[c]));
    }
  }
  void calibrateDrive(){
    QVector<double> ratios;
    for(const auto &r:trials()){
      const double od=number(r["odom_distance_m"]);
      const double real=std::hypot(number(r["real_x_m"],0),number(r["real_y_m"],0));
      if(od>0.2 && real>0.2 && std::abs(number(r["real_y_m"],0))<0.3*std::max(0.1,real)){
        ratios<<real/od;
      }
    }
    if(ratios.isEmpty()){
      QMessageBox::warning(this,"Drive calibration","Butuh straight trial valid.");
      return;
    }
    std::sort(ratios.begin(),ratios.end());
    const double ratio=ratios[ratios.size()/2];
    auto st=s_.value("esc");
    const double old=number(st->get("esc_ackermann.ros__parameters.speed_max"),1.0);
    const double updated=old*ratio;
    st->set("esc_ackermann.ros__parameters.speed_max",updated);
    s_.value("gui")->set("esc_calibration.last_drive_scale",ratio);
    auto veh=s_.value("vehicle");
    if(veh){
      const QString vb="vehicle.ros__parameters.";
      veh->set(vb+"drive_odometry_calibration_valid",true);
      veh->set(vb+"drive_odometry_calibration_scale",ratio);
      veh->set(vb+"drive_odometry_calibration_saved_at",QDateTime::currentDateTime().toString(Qt::ISODate));
    }
    // Stage-2 GNSS motion certification depends on the calibrated wheel-odometry scale.
    // A new drive scale invalidates prior wheel-vs-GNSS residual evidence.
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
    QMessageBox::information(
    this,"Drive calibration",
    QString("Scale median=%1. speed_max %2 → %3. Drive odometry VALID; evidence GNSS motion dicabut, tetapi EKF GNSS vx+vyaw tetap ON dan wajib diuji ulang.")
    .arg(ratio).arg(old).arg(updated));
  }
  static double median(QVector<double> v){
    if(v.isEmpty()) return NAN;
    std::sort(v.begin(),v.end());
    const int n=v.size();
    return n%2 ? v[n/2] : 0.5*(v[n/2-1]+v[n/2]);
  }
  CircleFit circleFit()const{
    CircleFit out;
    auto veh=s_.value("vehicle");
    if(!veh){
      out.reason="vehicle.yaml tidak tersedia";
      return out;
    }
    const QString vb="vehicle.ros__parameters.";
    const double track=number(veh->get(vb+"track_width_m",0.48),0.48);
    const double opDeg=std::abs(number(veh->get(vb+"operational_steering_angle_rad",0.45),0.45))*180.0/kPi;
    QVector<double> allL,leftL,rightL,leftR,rightR;
    for(const auto &r:trials()){
      const double x=number(r.value("real_x_m"));
      const double y=number(r.value("real_y_m"));
      double steerDeg=number(r.value("real_steering_deg"),NAN);
      if(!std::isfinite(steerDeg)||std::abs(steerDeg)<1.0){
        steerDeg=number(r.value("steering_actual_rad"),NAN)*180.0/kPi;
      }
      const double den=x*x+y*y;
      if(den<0.25||std::abs(y)<0.05||!std::isfinite(steerDeg)||std::abs(steerDeg)<3.0) continue;
      const double rc=den/(2.0*std::abs(y));
      const double ri=rc-0.5*track;
      const double leff=ri*std::tan(std::abs(steerDeg)*kPi/180.0);
      if(!std::isfinite(rc)||!std::isfinite(leff)||rc<=0.5*track+0.05||leff<0.1||leff>5.0) continue;
      QVariantMap d=r;
      d["circle_radius_center_m"]=rc;
      d["effective_wheelbase_m"]=leff;
      d["circle_side"]=y>0?"LEFT":"RIGHT";
      out.rows<<d;
      allL<<leff;
      if(y>0){
        leftL<<leff;
        if(std::abs(steerDeg)>=std::max(5.0,0.70*opDeg)) leftR<<rc;
      }
      else{
        rightL<<leff;
        if(std::abs(steerDeg)>=std::max(5.0,0.70*opDeg)) rightR<<rc;
      }
    }
    out.nLeft=leftR.size();
    out.nRight=rightR.size();
    out.leftLeff=median(leftL);
    out.rightLeff=median(rightL);
    if(std::isfinite(out.leftLeff)&&std::isfinite(out.rightLeff)) out.leff=0.5*(out.leftLeff+out.rightLeff);
    else out.leff=median(allL);
    out.leftRadius=median(leftR);
    out.rightRadius=median(rightR);
    if(!std::isfinite(out.leff)||out.nLeft<2||out.nRight<2||
    !std::isfinite(out.leftRadius)||!std::isfinite(out.rightRadius)){
      out.reason="Butuh minimal 2 circle trial kiri dan 2 kanan pada ≥70% batas steering operasional.";
      return out;
    }
    double sum2=0.0;
    int n=0;
    for(const auto &r:out.rows){
      double steerDeg=number(r.value("real_steering_deg"),NAN);
      if(!std::isfinite(steerDeg)||std::abs(steerDeg)<1.0){
        steerDeg=number(r.value("steering_actual_rad"),NAN)*180.0/kPi;
      }
      if(!std::isfinite(steerDeg)||std::abs(steerDeg)<3.0) continue;
      const double measured=number(r.value("circle_radius_center_m"),NAN);
      const double pred=0.5*track+out.leff/std::tan(std::abs(steerDeg)*kPi/180.0);
      if(std::isfinite(measured)&&std::isfinite(pred)){
        const double e=pred-measured;
        sum2+=e*e;
        ++n;
      }
    }
    out.rmse=n?std::sqrt(sum2/n):NAN;
    out.certifiedRadius=std::max(out.leftRadius,out.rightRadius)*(1.0+circleMargin_->value()/100.0);
    out.valid=std::isfinite(out.rmse)&&out.rmse<=circleRmseMax_->value()&&out.leff>0.1&&out.certifiedRadius>0.1;
    if(!out.valid){
      out.reason=QString("RMSE %1 m melewati batas %2 m atau geometri invalid")
      .arg(out.rmse,0,'f',3).arg(circleRmseMax_->value(),0,'f',3);
    }
    return out;
  }
  void calibrateWheelbase(){
    CircleFit f=circleFit();
    if(!f.valid){
      QMessageBox::warning(this,"Kinematic geometry",f.reason);
      circleStatus_->setText("BELUM LULUS: "+f.reason);
      return;
    }
    auto veh=s_.value("vehicle");
    QString vb="vehicle.ros__parameters.";
    veh->set(vb+"effective_wheelbase_m",f.leff);
    veh->set(vb+"effective_wheelbase_left_m",f.leftLeff);
    veh->set(vb+"effective_wheelbase_right_m",f.rightLeff);
    s_.value("esc")->set("esc_ackermann.ros__parameters.wheelbase_m",f.leff);
    s_.value("gui")->set("esc_calibration.last_effective_wheelbase_m",f.leff);
    if(ros_)ros_->setParametersAtomically("/esc_ackermann",{
      {
        "wheelbase_m",f.leff
      }
    },"steering_circle:wheelbase");
    QMessageBox::information(this,"Effective wheelbase",QString("Effective wheelbase=%1 m (LEFT=%2, RIGHT=%3).\nPhysical wheelbase tetap %4 m.").arg(f.leff,0,'f',4).arg(f.leftLeff,0,'f',4).arg(f.rightLeff,0,'f',4).arg(number(veh->get(vb+"wheelbase_m",0.70)),0,'f',4));
  }
  void analyzeCircle(bool apply){
    CircleFit f=circleFit();
    QString summary=QString("Circle trials L/R=%1/%2 • L_eff=%3 m (L=%4, R=%5) • R_left=%6 m • R_right=%7 m • RMSE=%8 m • certified R_min=%9 m (+%10%)").arg(f.nLeft).arg(f.nRight).arg(f.leff,0,'f',4).arg(f.leftLeff,0,'f',4).arg(f.rightLeff,0,'f',4).arg(f.leftRadius,0,'f',3).arg(f.rightRadius,0,'f',3).arg(f.rmse,0,'f',3).arg(f.certifiedRadius,0,'f',3).arg(circleMargin_->value(),0,'f',1);
    circleStatus_->setText((f.valid?"PASS • ":"FAIL • ")+summary+(f.reason.isEmpty()?QString():" • "+f.reason));
    if(!apply)return;
    if(!f.valid){
      QMessageBox::warning(this,"Circle Part 3",f.reason);
      return;
    }
    if(QMessageBox::question(this,"Terapkan Part 3",summary+"\n\nTerapkan effective wheelbase dan certified minimum turning radius ke ESC/NavigationCore/Perception/Nav2?")!=QMessageBox::Yes)return;
    auto veh=s_.value("vehicle");
    QString vb="vehicle.ros__parameters.";
    veh->set(vb+"effective_wheelbase_m",f.leff);
    veh->set(vb+"effective_wheelbase_left_m",f.leftLeff);
    veh->set(vb+"effective_wheelbase_right_m",f.rightLeff);
    veh->set(vb+"turning_radius_left_m",f.leftRadius);
    veh->set(vb+"turning_radius_right_m",f.rightRadius);
    veh->set(vb+"turning_radius_safety_margin_pct",circleMargin_->value());
    veh->set(vb+"minimum_turning_radius_m",f.certifiedRadius);
    veh->set(vb+"steering_circle_calibration_valid",true);
    veh->set(vb+"steering_circle_trial_count_left",f.nLeft);
    veh->set(vb+"steering_circle_trial_count_right",f.nRight);
    veh->set(vb+"steering_circle_fit_rmse_m",f.rmse);
    veh->set(vb+"steering_circle_calibration_saved_at",QDateTime::currentDateTime().toString(Qt::ISODate));
    veh->set(vb+"minimum_turning_radius_source",QString("measured_circle_part3"));
    double currentYaw=number(veh->get(vb+"max_yaw_rate_rps",1.0),1.0),maxForward=number(veh->get(vb+"max_forward_speed_mps",0.5),0.5),kinematicYawCap=maxForward/std::max(0.1,f.certifiedRadius);
    double certifiedYaw=std::min(currentYaw,kinematicYawCap);
    veh->set(vb+"max_yaw_rate_rps",certifiedYaw);
    s_.value("esc")->set("esc_ackermann.ros__parameters.wheelbase_m",f.leff);
    if(s_.contains("navigation_core")){
      s_["navigation_core"]->set("navigation_core.ros__parameters.wheelbase_m",f.leff);
      double cur=number(s_["navigation_core"]->get("navigation_core.ros__parameters.max_yaw_rate_rps",certifiedYaw),certifiedYaw);
      s_["navigation_core"]->set("navigation_core.ros__parameters.max_yaw_rate_rps",std::min(cur,certifiedYaw));
    }
    if(s_.contains("perception"))s_["perception"]->set("perception.ros__parameters.wheelbase_m",f.leff);
    if(s_.contains("nav2")){
      s_["nav2"]->set("planner_server.ros__parameters.GridBased.minimum_turning_radius",f.certifiedRadius);
      s_["nav2"]->set("controller_server.ros__parameters.FollowPath.AckermannConstraints.min_turning_r",f.certifiedRadius);
      double navV=number(s_["nav2"]->get("controller_server.ros__parameters.FollowPath.vx_max",maxForward),maxForward);
      double navYaw=std::min(certifiedYaw,navV/std::max(0.1,f.certifiedRadius));
      double wz=number(s_["nav2"]->get("controller_server.ros__parameters.FollowPath.wz_max",navYaw),navYaw);
      navYaw=std::min(navYaw,wz);
      s_["nav2"]->set("controller_server.ros__parameters.FollowPath.wz_max",navYaw);
      s_["nav2"]->set("velocity_smoother.ros__parameters.max_velocity",QVariantList{
        navV,0.0,navYaw
      });
      s_["nav2"]->set("velocity_smoother.ros__parameters.min_velocity",QVariantList{
        0.0,0.0,-navYaw
      });
      if(s_.contains("trajectory_safety")){
        double cur=number(s_["trajectory_safety"]->get("trajectory_safety_supervisor.ros__parameters.maximum_yaw_rate_rps",navYaw),navYaw);
        s_["trajectory_safety"]->set("trajectory_safety_supervisor.ros__parameters.maximum_yaw_rate_rps",std::min(cur,navYaw));
      }
    }
    s_.value("gui")->set("esc_calibration.last_effective_wheelbase_m",f.leff);
    if(ros_)ros_->setParametersAtomically("/esc_ackermann",{
      {
        "wheelbase_m",f.leff
      }
    },"steering_circle:apply_part3");
    refreshCircleSaved();
    QMessageBox::information(this,"Part 3 diterapkan",summary+"\nCertified yaw-rate ceiling juga dibatasi oleh v_max/R_min. Restart NavigationCore/Nav2/perception sebelum autonomous test.");
  }
  void exportCircleCsv(){
    CircleFit f=circleFit();
    QString fn=QFileDialog::getSaveFileName(this,"Ekspor Circle Test","steering_circle_part3.csv","CSV (*.csv)");
    if(fn.isEmpty())return;
    QSaveFile file(fn);
    if(!file.open(QIODevice::WriteOnly|QIODevice::Text)){
      QMessageBox::warning(this,"CSV","Tidak dapat membuka file.");
      return;
    }
    QTextStream out(&file);
    out<<"name,side,real_x_m,real_y_m,steering_deg,measured_center_radius_m,effective_wheelbase_m\n";
    for(const auto&r:f.rows){
      double steer=number(r.value("real_steering_deg"),NAN);
      if(!std::isfinite(steer)||std::abs(steer)<1.0)steer=number(r.value("steering_actual_rad"),NAN)*180.0/kPi;
      out<<r.value("name").toString()<<","<<r.value("circle_side").toString()<<","<<number(r.value("real_x_m"))<<","<<number(r.value("real_y_m"))<<","<<steer<<","<<number(r.value("circle_radius_center_m"))<<","<<number(r.value("effective_wheelbase_m"))<<"\n";
    }
    out<<"SUMMARY,,,,"<<f.leff<<","<<f.leftRadius<<","<<f.rightRadius<<"\n";
    if(!file.commit())QMessageBox::warning(this,"CSV","Gagal menyimpan.");
    else QMessageBox::information(this,"CSV",fn);
  }
  void refreshCircleSaved(){
    auto v=s_.value("vehicle");
    if(!v||!circleStatus_)return;
    QString b="vehicle.ros__parameters.";
    bool ok=v->get(b+"steering_circle_calibration_valid",false).toBool();
    if(ok)circleStatus_->setText(QString("TERSIMPAN • effective L=%1 m • R left/right=%2/%3 m • certified Rmin=%4 m • RMSE=%5 m • %6").arg(number(v->get(b+"effective_wheelbase_m",0.7)),0,'f',4).arg(number(v->get(b+"turning_radius_left_m",0)),0,'f',3).arg(number(v->get(b+"turning_radius_right_m",0)),0,'f',3).arg(number(v->get(b+"minimum_turning_radius_m",0)),0,'f',3).arg(number(v->get(b+"steering_circle_fit_rmse_m",0)),0,'f',3).arg(v->get(b+"steering_circle_calibration_saved_at").toString()));
  }
  void calibrateAll(){
    calibrateDrive();
    calibrateWheelbase();
  }
};
class MapGraphicsView:public QGraphicsView{
  public:std::function<void(double,double,Qt::MouseButton)>clickCb;
  std::function<void(double,double)>moveCb;
  explicit MapGraphicsView(QWidget*p=nullptr):QGraphicsView(p){
    setMouseTracking(true);
    setRenderHint(QPainter::Antialiasing);
    setDragMode(QGraphicsView::ScrollHandDrag);
  }
  protected:void wheelEvent(QWheelEvent*e)override{
    double f=e->angleDelta().y()>0?1.18:1/1.18;
    scale(f,f);
    e->accept();
  }
  void mousePressEvent(QMouseEvent*e)override{
    if(e->button()==Qt::LeftButton||e->button()==Qt::RightButton){
      QPointF p=mapToScene(e->pos());
      if(clickCb)clickCb(p.x(),p.y(),e->button());
    }
    QGraphicsView::mousePressEvent(e);
  }
  void mouseMoveEvent(QMouseEvent*e)override{
    QPointF p=mapToScene(e->pos());
    if(moveCb)moveCb(p.x(),p.y());
    QGraphicsView::mouseMoveEvent(e);
  }
};
class MapPage:public QWidget{
  public:MapPage(const WorkspacePaths&paths,const QMap<QString,std::shared_ptr<YamlStore>>&s,TelemetryStore*t,ReportManager*r,RosBridge*ros,QWidget*p=nullptr):QWidget(p),paths_(paths),s_(s),t_(t),r_(r),ros_(ros){
    auto*l=new QVBoxLayout(this);
    auto*h=new QLabel("Peta, Ground Truth & Target Tersimpan");
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto*toolbar=new QHBoxLayout();
    mode_=new NoWheelComboBox();
    mode_->addItems({
      "Pilih","Ground Truth","Tujuan"
    });
    yaw_=new QDoubleSpinBox();
    yaw_->setRange(-180,180);
    yaw_->setSuffix("°");
    fit_=new QPushButton("Sesuaikan Peta");
    osm_=new QPushButton("Perbarui OSM");
    auto*pngMap=new QPushButton("Ekspor PNG");
    auto*resetSolver=new QPushButton("Reset Solver");
    coord_=new QLabel("map=(--,--)");
    toolbar->addWidget(new QLabel("Mode"));
    toolbar->addWidget(mode_);
    toolbar->addWidget(new QLabel("Yaw"));
    toolbar->addWidget(yaw_);
    toolbar->addWidget(fit_);
    toolbar->addWidget(osm_);
    toolbar->addWidget(pngMap);
    toolbar->addWidget(resetSolver);
    toolbar->addWidget(coord_,1);
    l->addLayout(toolbar);
    scene_=new QGraphicsScene(this);
    view_=new MapGraphicsView();
    view_->setScene(scene_);
    view_->clickCb=[this](double sx,double sy,Qt::MouseButton b){
      onClick(sx,sy,b);
    };
    view_->moveCb=[this](double sx,double sy){
      onMove(sx,sy);
    };
    l->addWidget(view_,1);
    auto*bottom=new QSplitter(Qt::Horizontal);
    auto*targets=new QWidget();
    auto*tl=new QVBoxLayout(targets);
    targetTable_=new QTableWidget(0,5);
    targetTable_->setHorizontalHeaderLabels({
      "Target","X","Y","Yaw°","Keterangan"
    });
    targetTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    tl->addWidget(new QLabel("Target Tersimpan"));
    tl->addWidget(targetTable_);
    auto*tb=new QHBoxLayout();
    auto*addT=new QPushButton("+ Baru / Titik Klik");
    auto*editT=new QPushButton("Edit");
    auto*goT=new QPushButton("GO");
    auto*delT=new QPushButton("Hapus");
    auto*currentT=new QPushButton("Simpan Pose Sekarang");
    auto*reloadT=new QPushButton("Reload");
    for(auto*w:{
      addT,editT,goT,delT,currentT,reloadT
    })tb->addWidget(w);
    tl->addLayout(tb);
    auto*gt=new QWidget();
    auto*gl=new QVBoxLayout(gt);
    gtTable_=new QTableWidget(0,6);
    gtTable_->setHorizontalHeaderLabels({
      "GT","X","Y","Yaw°","hAcc","Created"
    });
    gtTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    gl->addWidget(new QLabel("Ground Truth"));
    gl->addWidget(gtTable_);
    auto*gb=new QHBoxLayout();
    auto*addGtBtn=new QPushButton("+ Baru");
    auto*editGtBtn=new QPushButton("Edit");
    auto*delGtBtn=new QPushButton("Hapus");
    auto*publishGt=new QPushButton("Publikasikan GT");
    auto*clearGt=new QPushButton("Clear All");
    auto*csvGt=new QPushButton("Ekspor CSV");
    for(auto*w:{
      addGtBtn,editGtBtn,delGtBtn,publishGt,csvGt,clearGt
    })gb->addWidget(w);
    gl->addLayout(gb);
    bottom->addWidget(targets);
    bottom->addWidget(gt);
    bottom->setMaximumHeight(230);
    l->addWidget(bottom);
    connect(fit_,&QPushButton::clicked,this,[this](){
      fitMap();
    });
    connect(osm_,&QPushButton::clicked,this,[this](){
      downloadOsm();
    });
    connect(pngMap,&QPushButton::clicked,this,[this](){
      QMessageBox::information(this,"PNG",r_->savePng("map_ground_truth",view_));
    });
    connect(resetSolver,&QPushButton::clicked,this,[this](){
      ros_->callTrigger("/localization/reset_calibration_samples","Reset calibration samples");
    });
    connect(addT,&QPushButton::clicked,this,[this](){
      addTarget();
    });
    connect(editT,&QPushButton::clicked,this,[this](){
      editTarget();
    });
    connect(goT,&QPushButton::clicked,this,[this](){
      goTarget();
    });
    connect(delT,&QPushButton::clicked,this,[this](){
      deleteTarget();
    });
    connect(currentT,&QPushButton::clicked,this,[this](){
      saveCurrentPoseTarget();
    });
    connect(reloadT,&QPushButton::clicked,this,[this](){
      refreshTargets();
    });
    connect(addGtBtn,&QPushButton::clicked,this,[this](){
      addGtDialog();
    });
    connect(editGtBtn,&QPushButton::clicked,this,[this](){
      editGt();
    });
    connect(delGtBtn,&QPushButton::clicked,this,[this](){
      deleteGt();
    });
    connect(publishGt,&QPushButton::clicked,this,[this](){
      publishSelectedGt();
    });
    connect(clearGt,&QPushButton::clicked,this,[this](){
      s_.value("gui")->set("ground_truth_points",QVariantList{
      });
      loadMap();
    });
    connect(csvGt,&QPushButton::clicked,this,[this](){
      QVector<QVariantMap>rows;
      for(const QVariant&v:s_.value("gui")->get("ground_truth_points",QVariantList{
      }).toList())rows<<v.toMap();
      QMessageBox::information(this,"CSV",r_->saveCsv("ground_truth_points",rows));
    });
    network_=new QNetworkAccessManager(this);
    connect(network_,&QNetworkAccessManager::finished,this,[this](QNetworkReply*reply){
      QByteArray data=reply->readAll();
      if(reply->error()==QNetworkReply::NoError){
        QSaveFile f(osmCache());
        if(f.open(QIODevice::WriteOnly)){
          f.write(data);
          f.commit();
        }
        drawOsm(data);
      }
      else QMessageBox::warning(this,"OSM",reply->errorString());
      reply->deleteLater();
    });
    loadMap();
    timer_=new QTimer(this);
    timer_->setInterval(250);
    connect(timer_,&QTimer::timeout,this,[this](){
      drawVehicle();
    });
    timer_->start();
  }
  void refreshTargets(){
    loadTables();
    drawTargetMarkers();
  }
  private:WorkspacePaths paths_;
  QMap<QString,std::shared_ptr<YamlStore>>s_;
  TelemetryStore*t_;
  ReportManager*r_;
  RosBridge*ros_;
  QGraphicsScene*scene_;
  MapGraphicsView*view_;
  QComboBox*mode_;
  QDoubleSpinBox*yaw_;
  QPushButton*fit_,*osm_;
  QLabel*coord_;
  QTableWidget*targetTable_,*gtTable_;
  QNetworkAccessManager*network_;
  QTimer*timer_;
  QGraphicsPixmapItem*pgm_=nullptr;
  QVector<QGraphicsItem*>dynamic_,targetItems_,osmItems_;
  double res_=0.1,widthM_=0,heightM_=0,originX_=0,originY_=0,originYaw_=0,lastX_=NAN,lastY_=NAN;
  double centerLat_=-7.0500161,centerLon_=110.4363513;
  QString mapPgm()const{
    return paths_.fileMap().value("map_pgm");
  }
  QString osmCache()const{
    return QString::fromStdString((paths_.navSource/"maps/undip/undip_nav2_raw.osm").string());
  }
  QPointF localToMap(double x,double y)const{
    double c=cos(originYaw_),ss=sin(originYaw_);
    return {
      originX_+c*x-ss*y,originY_+ss*x+c*y
    };
  }
  QPointF mapToLocal(double x,double y)const{
    double dx=x-originX_,dy=y-originY_,c=cos(originYaw_),ss=sin(originYaw_);
    return {
      c*dx+ss*dy,-ss*dx+c*dy
    };
  }
  void loadMap(){
    scene_->clear();
    dynamic_.clear();
    targetItems_.clear();
    osmItems_.clear();
    auto map=s_.value("map");
    if(map){
      res_=number(map->get("resolution",0.1),0.1);
      QVariantList o=map->get("origin",QVariantList{
        0.0,0.0,0.0
      }).toList();
      if(o.size()>=3){
        originX_=o[0].toDouble();
        originY_=o[1].toDouble();
        originYaw_=o[2].toDouble();
      }
    }
    QImage img(mapPgm());
    if(img.isNull()){
      scene_->addText("PGM map tidak dapat dibuka");
      return;
    }
    widthM_=img.width()*res_;
    heightM_=img.height()*res_;
    pgm_=scene_->addPixmap(QPixmap::fromImage(img));
    pgm_->setTransform(QTransform::fromScale(res_,res_));
    pgm_->setOpacity(number(s_.value("gui")->get("map_overlay.pgm_opacity",0.72),0.72));
    scene_->setSceneRect(0,0,widthM_,heightM_);
    loadTables();
    drawTargetMarkers();
    QFile f(osmCache());
    if(f.open(QIODevice::ReadOnly))drawOsm(f.readAll());
    fitMap();
  }
  void fitMap(){
    view_->fitInView(scene_->sceneRect(),Qt::KeepAspectRatio);
  }
  void onMove(double sx,double sy){
    if(sx<0||sy<0||sx>widthM_||sy>heightM_)return;
    double ly=heightM_-sy;
    QPointF m=localToMap(sx,ly);
    coord_->setText(QString("map=(%1, %2) m • raster=(%3,%4)").arg(m.x(),0,'f',3).arg(m.y(),0,'f',3).arg(sx,0,'f',3).arg(ly,0,'f',3));
  }
  void onClick(double sx,double sy,Qt::MouseButton b){
    if(b!=Qt::LeftButton||sx<0||sy<0||sx>widthM_||sy>heightM_)return;
    double ly=heightM_-sy;
    QPointF m=localToMap(sx,ly);
    lastX_=m.x();
    lastY_=m.y();
    if(mode_->currentText()=="Ground Truth")addGt(m.x(),m.y(),yaw_->value(),true);
    else if(mode_->currentText()=="Tujuan"){
      ros_->publishGoal(m.x(),m.y(),yaw_->value()*kPi/180.0);
      drawGoal(m.x(),m.y(),yaw_->value());
    }
  }
  void addGt(double x,double y,double yawDeg,bool publish){
    QVariantList list=s_.value("gui")->get("ground_truth_points",QVariantList{
    }).toList();
    QVariantMap fix=t_->get("gnss_fix").toMap();
    QVariantMap p{
      {
        "name",QString("GT%1").arg(list.size()+1,2,10,QChar('0'))
      },{
        "map_x_m",x
      },{
        "map_y_m",y
      },{
        "yaw_deg",yawDeg
      },{
        "gnss_latitude",fix.value("lat")
      },{
        "gnss_longitude",fix.value("lon")
      },{
        "gnss_hacc_m",t_->get("gnss_quality.hacc_m")
      },{
        "created",QDateTime::currentDateTime().toString(Qt::ISODate)
      }
    };
    list<<p;
    s_.value("gui")->set("ground_truth_points",list);
    if(publish)ros_->publishGroundTruth(x,y,yawDeg*kPi/180.0);
    loadTables();
    drawGtMarkers();
  }
  void loadTables(){
    QVariantMap targets=s_.value("saved_targets")->get("targets",QVariantMap{
    }).toMap();
    targetTable_->setRowCount(targets.size());
    int r=0;
    for(auto it=targets.cbegin();
    it!=targets.cend();
    ++it,++r){
      QVariantMap p=it.value().toMap();
      QStringList v={
        it.key(),variantText(p["x"]),variantText(p["y"]),QString::number(number(p["yaw_rad"])*180/kPi,'f',1),p.value("description").toString()
      };
      for(int c=0;
      c<5;
      ++c)targetTable_->setItem(r,c,new QTableWidgetItem(v[c]));
    }
    QVariantList g=s_.value("gui")->get("ground_truth_points",QVariantList{
    }).toList();
    gtTable_->setRowCount(g.size());
    for(int i=0;
    i<g.size();
    ++i){
      QVariantMap p=g[i].toMap();
      QStringList v={
        p["name"].toString(),variantText(p["map_x_m"]),variantText(p["map_y_m"]),variantText(p["yaw_deg"]),variantText(p["gnss_hacc_m"]),p.value("created").toString()
      };
      for(int c=0;
      c<6;
      ++c)gtTable_->setItem(i,c,new QTableWidgetItem(v[c]));
    }
    drawGtMarkers();
  }
  void addTarget(){
    double x=std::isfinite(lastX_)?lastX_:0.0,y=std::isfinite(lastY_)?lastY_:0.0;
    PoseEntryDialog d("Target Baru",QString("Target%1").arg(targetTable_->rowCount()+1),x,y,yaw_->value(),QString(),this);
    if(d.exec()!=QDialog::Accepted||d.name().isEmpty())return;
    QVariantMap targets=s_.value("saved_targets")->get("targets",QVariantMap{
    }).toMap();
    if(targets.contains(d.name())&&QMessageBox::question(this,"Target","Nama sudah ada. Timpa target?")!=QMessageBox::Yes)return;
    targets[d.name()] = QVariantMap{
      {
        "x",d.x()
      },{
        "y",d.y()
      },{
        "yaw_rad",d.yawDeg()*kPi/180.0
      },{
        "description",d.description()
      }
    };
    s_.value("saved_targets")->set("targets",targets);
    refreshTargets();
  }
  void editTarget(){
    int row=targetTable_->currentRow();
    if(row<0)return;
    QString oldName=targetTable_->item(row,0)->text();
    QVariantMap all=s_.value("saved_targets")->get("targets",QVariantMap{
    }).toMap(),p=all.value(oldName).toMap();
    PoseEntryDialog d("Edit Target",oldName,number(p["x"]),number(p["y"]),number(p["yaw_rad"])*180/kPi,p.value("description").toString(),this);
    if(d.exec()!=QDialog::Accepted||d.name().isEmpty())return;
    if(d.name()!=oldName)all.remove(oldName);
    all[d.name()] = QVariantMap{
      {
        "x",d.x()
      },{
        "y",d.y()
      },{
        "yaw_rad",d.yawDeg()*kPi/180.0
      },{
        "description",d.description()
      }
    };
    s_.value("saved_targets")->set("targets",all);
    refreshTargets();
  }
  void saveCurrentPoseTarget(){
    double x=number(t_->get("localization_state.map_x")),y=number(t_->get("localization_state.map_y")),yr=number(t_->get("localization_state.yaw"),0);
    if(!std::isfinite(x)||!std::isfinite(y)){
      QMessageBox::warning(this,"Target","Pose map kendaraan belum valid.");
      return;
    }
    lastX_=x;
    lastY_=y;
    yaw_->setValue(yr*180/kPi);
    addTarget();
  }
  void goTarget(){
    int row=targetTable_->currentRow();
    if(row<0)return;
    QString name=targetTable_->item(row,0)->text();
    QVariantMap all=s_.value("saved_targets")->get("targets",QVariantMap{
    }).toMap();
    QVariantMap p=all.value(name).toMap();
    ros_->publishGoal(number(p["x"]),number(p["y"]),number(p["yaw_rad"]));
  }
  void deleteTarget(){
    int row=targetTable_->currentRow();
    if(row<0)return;
    QString name=targetTable_->item(row,0)->text();
    QVariantMap targets=s_.value("saved_targets")->get("targets",QVariantMap{
    }).toMap();
    targets.remove(name);
    s_.value("saved_targets")->set("targets",targets);
    refreshTargets();
  }
  void addGtDialog(){
    double x=std::isfinite(lastX_)?lastX_:number(t_->get("localization_state.map_x"),0),y=std::isfinite(lastY_)?lastY_:number(t_->get("localization_state.map_y"),0);
    PoseEntryDialog d("Ground Truth Baru",QString("GT%1").arg(gtTable_->rowCount()+1,2,10,QChar('0')),x,y,yaw_->value(),QString(),this);
    if(d.exec()!=QDialog::Accepted)return;
    addGt(d.x(),d.y(),d.yawDeg(),false);
    QVariantList list=s_.value("gui")->get("ground_truth_points",QVariantList{
    }).toList();
    if(!list.isEmpty()){
      QVariantMap p=list.last().toMap();
      p["name"]=d.name().isEmpty()?p["name"]:d.name();
      p["description"]=d.description();
      list[list.size()-1]=p;
      s_.value("gui")->set("ground_truth_points",list);
    }
    loadTables();
    drawGtMarkers();
  }
  void editGt(){
    int row=gtTable_->currentRow();
    QVariantList list=s_.value("gui")->get("ground_truth_points",QVariantList{
    }).toList();
    if(row<0||row>=list.size())return;
    QVariantMap p=list[row].toMap();
    PoseEntryDialog d("Edit Ground Truth",p.value("name").toString(),number(p["map_x_m"]),number(p["map_y_m"]),number(p["yaw_deg"]),p.value("description").toString(),this);
    if(d.exec()!=QDialog::Accepted)return;
    p["name"]=d.name();
    p["map_x_m"]=d.x();
    p["map_y_m"]=d.y();
    p["yaw_deg"]=d.yawDeg();
    p["description"]=d.description();
    p["updated"]=QDateTime::currentDateTime().toString(Qt::ISODate);
    list[row]=p;
    s_.value("gui")->set("ground_truth_points",list);
    loadTables();
    drawGtMarkers();
  }
  void deleteGt(){
    int row=gtTable_->currentRow();
    QVariantList list=s_.value("gui")->get("ground_truth_points",QVariantList{
    }).toList();
    if(row<0||row>=list.size())return;
    if(QMessageBox::question(this,"Ground Truth","Hapus titik terpilih?")!=QMessageBox::Yes)return;
    list.removeAt(row);
    s_.value("gui")->set("ground_truth_points",list);
    loadTables();
    drawGtMarkers();
  }
  void publishSelectedGt(){
    int row=gtTable_->currentRow();
    if(row<0)return;
    QVariantList g=s_.value("gui")->get("ground_truth_points",QVariantList{
    }).toList();
    if(row>=g.size())return;
    QVariantMap p=g[row].toMap();
    ros_->publishGroundTruth(number(p["map_x_m"]),number(p["map_y_m"]),number(p["yaw_deg"])*kPi/180.0);
  }
  void clearItems(QVector<QGraphicsItem*>&v){
    for(auto*i:v)scene_->removeItem(i);
    v.clear();
  }
  void drawGtMarkers(){
    for(QGraphicsItem*i:dynamic_)if(i->data(0).toString()=="gt")scene_->removeItem(i);
    QVector<QGraphicsItem*>keep;
    for(auto*i:dynamic_)if(i->data(0).toString()!="gt")keep<<i;
    dynamic_=keep;
    for(const QVariant&v:s_.value("gui")->get("ground_truth_points",QVariantList{
    }).toList()){
      QVariantMap p=v.toMap();
      QPointF l=mapToLocal(number(p["map_x_m"]),number(p["map_y_m"]));
      double sy=heightM_-l.y();
      auto*e=scene_->addEllipse(l.x()-1.5,sy-1.5,3,3,QPen(Qt::white,0),QColor(kRed));
      e->setData(0,"gt");
      dynamic_<<e;
      auto*t=scene_->addSimpleText(p["name"].toString());
      t->setBrush(Qt::white);
      t->setPos(l.x()+2,sy-8);
      t->setFlag(QGraphicsItem::ItemIgnoresTransformations);
      t->setData(0,"gt");
      dynamic_<<t;
    }
  }
  void drawTargetMarkers(){
    clearItems(targetItems_);
    QVariantMap targets=s_.value("saved_targets")->get("targets",QVariantMap{
    }).toMap();
    for(auto it=targets.cbegin();
    it!=targets.cend();
    ++it){
      QVariantMap p=it.value().toMap();
      QPointF l=mapToLocal(number(p["x"]),number(p["y"]));
      if(l.x()<0||l.y()<0||l.x()>widthM_||l.y()>heightM_)continue;
      double sy=heightM_-l.y();
      auto*e=scene_->addEllipse(l.x()-1.2,sy-1.2,2.4,2.4,QPen(QColor(kGold),0.8),QColor(kDark));
      auto*t=scene_->addSimpleText(it.key());
      t->setBrush(QColor(kGold));
      t->setFlag(QGraphicsItem::ItemIgnoresTransformations);
      t->setPos(l.x()+2,sy);
      targetItems_<<e<<t;
    }
  }
  void drawGoal(double x,double y,double yawDeg){
    QPointF l=mapToLocal(x,y);
    double sy=heightM_-l.y();
    auto*e=scene_->addEllipse(l.x()-2,sy-2,4,4,QPen(QColor(kGreen),.8),QColor(kGreen));
    double a=yawDeg*kPi/180.0-originYaw_;
    auto*line=scene_->addLine(l.x(),sy,l.x()+8*cos(a),sy-8*sin(a),QPen(QColor(kGreen),1.5));
    dynamic_<<e<<line;
  }
  void drawVehicle(){
    for(QGraphicsItem*i:dynamic_)if(i->data(0).toString()=="vehicle")scene_->removeItem(i);
    QVector<QGraphicsItem*>keep;
    for(auto*i:dynamic_)if(i->data(0).toString()!="vehicle")keep<<i;
    dynamic_=keep;
    double x=number(t_->get("localization_state.map_x")),y=number(t_->get("localization_state.map_y")),yaw=number(t_->get("localization_state.yaw"),0);
    if(!std::isfinite(x)||!std::isfinite(y))return;
    QPointF l=mapToLocal(x,y);
    double sy=heightM_-l.y();
    auto*e=scene_->addEllipse(l.x()-1.5,sy-1.5,3,3,QPen(QColor(kBlue),.5),QColor(kBlue));
    double a=yaw-originYaw_;
    auto*line=scene_->addLine(l.x(),sy,l.x()+7*cos(a),sy-7*sin(a),QPen(QColor(kBlue),1.2));
    e->setData(0,"vehicle");
    line->setData(0,"vehicle");
    dynamic_<<e<<line;
  }
  void readCenter(){
    QFile f(QString::fromStdString((paths_.navSource/"tools/osm_pgm.py").string()));
    if(!f.open(QIODevice::ReadOnly|QIODevice::Text))return;
    QString text=QString::fromUtf8(f.readAll());
    auto val=[&](const QString&name,double def){
      QRegularExpression re(QStringLiteral("%1\\s*=\\s*([-+0-9.eE]+)").arg(name));
      auto m=re.match(text);
      return m.hasMatch()?m.captured(1).toDouble():def;
    };
    centerLon_=val("CENTER_LONGITUDE",centerLon_);
    centerLat_=val("CENTER_LATITUDE",centerLat_);
  }
  void downloadOsm(){
    readCenter();
    double latRad=centerLat_*kPi/180.0,dLat=(heightM_/2)/6378137.0*180/kPi,dLon=(widthM_/2)/(6378137.0*cos(latRad))*180/kPi;
    QUrl u("https://api.openstreetmap.org/api/0.6/map");
    QUrlQuery q;
    q.addQueryItem("bbox",QString("%1,%2,%3,%4").arg(centerLon_-dLon,0,'f',10).arg(centerLat_-dLat,0,'f',10).arg(centerLon_+dLon,0,'f',10).arg(centerLat_+dLat,0,'f',10));
    u.setQuery(q);
    QNetworkRequest req(u);
    req.setRawHeader("User-Agent","UNDIP-AGV-GUI-CPP/3.0");
    network_->get(req);
  }
  void drawOsm(const QByteArray&data){
    clearItems(osmItems_);
    readCenter();
    struct LL{
      double lat=0,lon=0;
    };
    QHash<QString,LL>nodes;
    struct Way{
      QString highway;
      QStringList refs;
    };
    QVector<Way>ways;
    QXmlStreamReader xml(data);
    Way cur;
    bool inWay=false;
    while(!xml.atEnd()){
      xml.readNext();
      if(xml.isStartElement()){
        if(xml.name()=="node"){
          auto a=xml.attributes();
          nodes[a.value("id").toString()]={
            a.value("lat").toDouble(),a.value("lon").toDouble()
          };
        }
        else if(xml.name()=="way"){
          inWay=true;
          cur={
          };
        }
        else if(inWay&&xml.name()=="nd")cur.refs<<xml.attributes().value("ref").toString();
        else if(inWay&&xml.name()=="tag"&&xml.attributes().value("k")=="highway")cur.highway=xml.attributes().value("v").toString();
      }
      else if(xml.isEndElement()&&xml.name()=="way"){
        inWay=false;
        ways<<cur;
      }
    }
    QSet<QString>drive={
      "primary","secondary","tertiary","residential","service","unclassified","living_street","road"
    };
    double opacity=number(s_.value("gui")->get("map_overlay.osm_opacity",0.95),0.95),lw=number(s_.value("gui")->get("map_overlay.osm_line_width_px",2.2),2.2);
    for(const Way&w:ways){
      if(!drive.contains(w.highway)||w.refs.size()<2)continue;
      QPainterPath path;
      bool first=true;
      for(const QString&id:w.refs){
        if(!nodes.contains(id))continue;
        LL p=nodes[id];
        double east=6378137.0*(p.lon-centerLon_)*kPi/180*cos(centerLat_*kPi/180),north=6378137.0*(p.lat-centerLat_)*kPi/180;
        double x=widthM_/2+east,y=heightM_/2+north,sy=heightM_-y;
        if(first){
          path.moveTo(x,sy);
          first=false;
        }
        else path.lineTo(x,sy);
      }
      auto*i=scene_->addPath(path,QPen(QColor(kGold),lw));
      i->setOpacity(opacity);
      i->setZValue(20);
      osmItems_<<i;
    }
  }
};
class OverviewPage:public QWidget{
  public:OverviewPage(const QMap<QString,std::shared_ptr<YamlStore>>&s,TelemetryStore*t,RosBridge*r,QWidget*p=nullptr):QWidget(p),s_(s),t_(t),ros_(r){
    auto*l=new QVBoxLayout(this);
    auto*h=new QLabel("Overview Operasional");
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto*g=new QGridLayout();
    QStringList names={
      "GNSS","IMU","Camera","ESC","Nav2","Autonomy","Motion","E-Stop"
    };
    QStringList keys={
      "connected.gnss","connected.imu","connected.camera","connected.esc_ready","system.nav2_ready","system.autonomy_ready","system.motion_ready","system.estop"
    };
    for(int i=0;
    i<names.size();
    ++i){
      auto*pill=new StatusPill(names[i]);
      pills_<<pill;
      keys_<<keys[i];
      g->addWidget(pill,i/4,i%4);
    }
    l->addLayout(g);
    auto*cards=new QGridLayout();
    pose_=new QLabel("Map pose: --");
    gnss_=new QLabel("GNSS: --");
    esc_=new QLabel("ESC: --");
    goal_=new QLabel("Goal: --");
    for(auto*w:{
      pose_,gnss_,esc_,goal_
    }){
      w->setObjectName("metricCard");
      w->setMinimumHeight(55);
    }
    cards->addWidget(pose_,0,0);
    cards->addWidget(gnss_,0,1);
    cards->addWidget(esc_,1,0);
    cards->addWidget(goal_,1,1);
    l->addLayout(cards);
    auto*targetBox=new QGroupBox("Target & Patrol");
    auto*tb=new QGridLayout(targetBox);
    targets_=new NoWheelComboBox();
    refreshTargets();
    auto*go=new QPushButton("Jalankan Target");
    auto*cancel=new QPushButton("Batalkan Navigasi");
    route_=new QListWidget();
    auto*add=new QPushButton("Tambah ke Patroli");
    auto*remove=new QPushButton("Hapus dari Patroli");
    auto*up=new QPushButton("Naik");
    auto*down=new QPushButton("Turun");
    auto*clear=new QPushButton("Bersihkan Rute");
    auto*start=new QPushButton("Mulai Patroli");
    auto*stop=new QPushButton("Hentikan Patroli");
    auto*pngOverview=new QPushButton("Ekspor PNG");
    dwell_=new QDoubleSpinBox();
    dwell_->setRange(0,120);
    dwell_->setValue(number(s_.value("gui")->get("patrol.dwell_sec",2.0),2.0));
    loop_=new QCheckBox("Ulangi");
    loop_->setChecked(s_.value("gui")->get("patrol.loop",true).toBool());
    tb->addWidget(targets_,0,0,1,2);
    tb->addWidget(go,0,2);
    tb->addWidget(cancel,0,3);
    tb->addWidget(route_,1,0,3,2);
    tb->addWidget(add,1,2);
    tb->addWidget(remove,1,3);
    tb->addWidget(up,2,2);
    tb->addWidget(down,2,3);
    tb->addWidget(clear,3,2);
    tb->addWidget(pngOverview,3,3);
    tb->addWidget(new QLabel("Dwell s"),4,2);
    tb->addWidget(dwell_,4,3);
    tb->addWidget(loop_,5,2);
    tb->addWidget(start,6,2);
    tb->addWidget(stop,6,3);
    l->addWidget(targetBox,1);
    plot_=new LivePlotWidget("Drive & Steering Command vs Feedback");
    l->addWidget(plot_,1);
    connect(go,&QPushButton::clicked,this,[this](){
      publishTarget(targets_->currentText());
    });
    connect(cancel,&QPushButton::clicked,ros_,&RosBridge::cancelNavigation);
    connect(add,&QPushButton::clicked,this,[this](){
      if(!targets_->currentText().isEmpty())route_->addItem(targets_->currentText());
      saveRoute();
    });
    connect(remove,&QPushButton::clicked,this,[this](){
      int r=route_->currentRow();
      if(r>=0)delete route_->takeItem(r);
      saveRoute();
    });
    connect(up,&QPushButton::clicked,this,[this](){
      moveRoute(-1);
    });
    connect(down,&QPushButton::clicked,this,[this](){
      moveRoute(1);
    });
    connect(clear,&QPushButton::clicked,this,[this](){
      route_->clear();
      saveRoute();
    });
    connect(pngOverview,&QPushButton::clicked,this,[this](){
      QString dir=s_.value("gui")->get("reporting.output_directory",QDir::homePath()+"/.ros/agv_gui_reports").toString();
      if(dir.startsWith('~'))dir=QDir::homePath()+dir.mid(1);
      QDir().mkpath(dir);
      QString out=QDir(dir).filePath("overview_"+nowStamp()+".png");
      this->grab().save(out);
      QMessageBox::information(this,"PNG",out);
    });
    connect(start,&QPushButton::clicked,this,[this](){
      patrol_=true;
      patrolIndex_=0;
      dispatch();
    });
    connect(stop,&QPushButton::clicked,this,[this](){
      patrol_=false;
      ros_->cancelNavigation();
    });
    timer_=new QTimer(this);
    timer_->setInterval(200);
    connect(timer_,&QTimer::timeout,this,[this](){
      refresh();
    });
    timer_->start();
    QVariantList route=s_.value("gui")->get("patrol.target_names",QVariantList{
    }).toList();
    for(const QVariant&v:route)route_->addItem(v.toString());
  }
  void refreshTargets(){
    if(!targets_)return;
    targets_->clear();
    QVariantMap m=s_.value("saved_targets")->get("targets",QVariantMap{
    }).toMap();
    targets_->addItems(m.keys());
  }
  private:QMap<QString,std::shared_ptr<YamlStore>>s_;
  TelemetryStore*t_;
  RosBridge*ros_;
  QVector<StatusPill*>pills_;
  QStringList keys_;
  QLabel*pose_,*gnss_,*esc_,*goal_;
  QComboBox*targets_;
  QListWidget*route_;
  QDoubleSpinBox*dwell_;
  QCheckBox*loop_;
  LivePlotWidget*plot_;
  QTimer*timer_;
  bool patrol_=false;
  int patrolIndex_=0;
  double dispatchAt_=0;
  bool publishTarget(const QString&name){
    QVariantMap all=s_.value("saved_targets")->get("targets",QVariantMap{
    }).toMap();
    QVariantMap p=all.value(name).toMap();
    if(p.isEmpty())return false;
    return ros_->publishGoal(number(p["x"]),number(p["y"]),number(p["yaw_rad"]));
  }
  void moveRoute(int delta){
    int row=route_->currentRow();
    int dst=row+delta;
    if(row<0||dst<0||dst>=route_->count())return;
    QListWidgetItem*item=route_->takeItem(row);
    route_->insertItem(dst,item);
    route_->setCurrentRow(dst);
    saveRoute();
  }
  void saveRoute(){
    QVariantList l;
    for(int i=0;
    i<route_->count();
    ++i)l<<route_->item(i)->text();
    s_.value("gui")->set("patrol.target_names",l);
    s_.value("gui")->set("patrol.dwell_sec",dwell_->value());
    s_.value("gui")->set("patrol.loop",loop_->isChecked());
  }
  void dispatch(){
    if(!patrol_||route_->count()==0)return;
    if(patrolIndex_>=route_->count()){
      if(loop_->isChecked())patrolIndex_=0;
      else{
        patrol_=false;
        return;
      }
    }
    publishTarget(route_->item(patrolIndex_)->text());
  }
  void refresh(){
    for(int i=0;
    i<pills_.size();
    ++i){
      bool v=t_->get(keys_[i],false).toBool();
      if(keys_[i]=="system.estop")pills_[i]->setStatus(v?"bad":"ok",v?"E-STOP":"SAFE");
      else pills_[i]->setStatus(v?"ok":"bad",v?"READY":"OFF");
    }
    pose_->setText(QString("Map pose\nX %1 m • Y %2 m • yaw %3°").arg(variantText(t_->get("localization_state.map_x"))).arg(variantText(t_->get("localization_state.map_y"))).arg(number(t_->get("localization_state.yaw"))*180/kPi,0,'f',1));
    gnss_->setText(QString("GNSS\nSat %1 • hAcc %2 m • PVT %3 Hz").arg(variantText(t_->get("gnss_quality.sat"),0)).arg(variantText(t_->get("gnss_quality.hacc_m"))).arg(variantText(t_->get("gnss_quality.pvt_rate_hz"))));
    esc_->setText(QString("ESC\nv=%1 m/s • steer=%2°").arg(variantText(t_->get("esc_drive_actual"))).arg(number(t_->get("esc_steer_actual"))*180/kPi,0,'f',1));
    QString gs=t_->get("goal_state.state","--").toString();
    goal_->setText("Goal\n"+gs);
    plot_->append({
      {
        "v target",number(t_->get("esc_drive_target"))
      },{
        "v actual",number(t_->get("esc_drive_actual"))
      },{
        "steer target",number(t_->get("esc_steer_target"))
      },{
        "steer actual",number(t_->get("esc_steer_actual"))
      }
    });
    if(patrol_&&gs=="SUCCEEDED"){
      double now=QDateTime::currentMSecsSinceEpoch()/1000.0;
      if(dispatchAt_==0)dispatchAt_=now+dwell_->value();
      if(now>=dispatchAt_){
        ++patrolIndex_;
        dispatchAt_=0;
        dispatch();
      }
    }
    else if(gs=="ABORTED"||gs=="REJECTED"||gs=="CANCELED"){
      patrol_=false;
      dispatchAt_=0;
    }
  }
};
bool solveHomography8(const QVariantList &src,const QVariantList &dst,double H[9]){
  if (src.size() != 8 || dst.size() != 8) {
    return false;
  }
  double A[8][9]{
  };
  for(int i=0;
  i<4;
  ++i){
    double x=src[2*i].toDouble(),y=src[2*i+1].toDouble(),u=dst[2*i].toDouble(),v=dst[2*i+1].toDouble();
    A[2*i][0]=x;
    A[2*i][1]=y;
    A[2*i][2]=1;
    A[2*i][6]=-u*x;
    A[2*i][7]=-u*y;
    A[2*i][8]=u;
    A[2*i+1][3]=x;
    A[2*i+1][4]=y;
    A[2*i+1][5]=1;
    A[2*i+1][6]=-v*x;
    A[2*i+1][7]=-v*y;
    A[2*i+1][8]=v;
  }
  for(int c=0;
  c<8;
  ++c){
    int piv=c;
    for(int r=c+1;
    r<8;
    ++r)if(std::abs(A[r][c])>std::abs(A[piv][c]))piv=r;
    if(std::abs(A[piv][c])<1e-12)return false;
    if(piv!=c)for(int j=c;
    j<9;
    ++j)std::swap(A[piv][j],A[c][j]);
    double d=A[c][c];
    for(int j=c;
    j<9;
    ++j)A[c][j]/=d;
    for(int r=0;
    r<8;
    ++r)if(r!=c){
      double f=A[r][c];
      for(int j=c;
      j<9;
      ++j)A[r][j]-=f*A[c][j];
    }
  }
  for (int i = 0;
  i < 8;
  ++i) {
    H[i] = A[i][8];
  }
  H[8] = 1;
  return true;
}
QPointF projectH(const double H[9],double x,double y,bool *ok=nullptr){
  double z=H[6]*x+H[7]*y+H[8];
  if(std::abs(z)<1e-12){
    if(ok)*ok=false;
    return{
    };
  }
  if(ok)*ok=true;
  return{
    (H[0]*x+H[1]*y+H[2])/z,(H[3]*x+H[4]*y+H[5])/z
  };
}
class ZoomGraphicsView:public QGraphicsView{
  public:explicit ZoomGraphicsView(QWidget*p=nullptr):QGraphicsView(p){
    setDragMode(QGraphicsView::ScrollHandDrag);
    setRenderHint(QPainter::Antialiasing);
  }
  protected:void wheelEvent(QWheelEvent*e)override{
    double f=e->angleDelta().y()>0?1.15:1/1.15;
    scale(f,f);
    e->accept();
  }
};
class DragHandle:public QGraphicsEllipseItem{
  public:std::function<void(QPointF)>moved;
  std::function<void()>released;
  DragHandle(QPointF p,QColor c):QGraphicsEllipseItem(-7,-7,14,14){
    setPos(p);
    setBrush(c);
    setPen(QPen(Qt::white,1));
    setFlags(ItemIsMovable|ItemSendsGeometryChanges);
    setZValue(120);
  }
  protected:QVariant itemChange(GraphicsItemChange c,const QVariant&v)override{
    if(c==ItemPositionHasChanged&&moved)moved(v.toPointF());
    return QGraphicsEllipseItem::itemChange(c,v);
  }
  void mouseReleaseEvent(QGraphicsSceneMouseEvent*e)override{
    QGraphicsEllipseItem::mouseReleaseEvent(e);
    if(released)released();
  }
};
