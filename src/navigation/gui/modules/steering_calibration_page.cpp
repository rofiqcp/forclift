// Extracted from agv_gui.cpp for maintainability.
class SteeringCalibrationPage:public QWidget{
  public:
  SteeringCalibrationPage(
  const QMap<QString,std::shared_ptr<YamlStore>>&s,
  TelemetryStore*t,RosBridge*r,QWidget*p=nullptr)
  :QWidget(p),s_(s),t_(t),ros_(r)
  {
    auto*l=new QVBoxLayout(this);
    auto*h=new QLabel("Kalibrasi Steering Fisik — Nilai ESC ≠ Sudut Roda");
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto*d=new QLabel(
    "ESC/STM tetap menggunakan skala protocol hingga -90 / 0 / +90. Nilai itu BUKAN sudut roda fisik. "
    "Part 1 menyimpan tiga referensi protocol+feedback (KIRI/LURUS/KANAN), kemudian Anda memasukkan sudut roda nyata "
    "yang diukur terhadap posisi lurus. Setelah Apply, /esc/steering_actual, odometri dan model Ackermann memakai sudut fisik tersebut.");
    d->setWordWrap(true);
    l->addWidget(d);
    auto*steps=new QGroupBox("Urutan Kalibrasi Part 1");
    auto*sl=new QVBoxLayout(steps);
    auto*st=new QLabel(
    "1) Kendaraan berhenti dan roda bebas bergerak.\n"
    "2) Aktifkan MODE KALIBRASI.\n"
    "3) Luruskan roda secara fisik, tahan ±1 detik, lalu CAPTURE LURUS.\n"
    "4) Gerakkan ke KIRI maksimum yang aman, tahan, CAPTURE KIRI, lalu ukur sudut RODA DALAM (roda kiri saat belok kiri) terhadap garis lurus chassis dan masukkan magnitudonya.\n"
    "5) Ulangi untuk KANAN: ukur RODA DALAM kanan saat belok kanan.\n"
    "6) Periksa ringkasan dan Apply. Batas operasional dibuat lebih kecil dari sisi mekanik terkecil.\n"
    "Catatan: model Ackermann project memakai sudut roda dalam. Jadi sudut fisik diukur antara arah RODA DALAM dan garis lurus chassis; bukan angka ESC/encoder.");
    st->setWordWrap(true);
    sl->addWidget(st);
    l->addWidget(steps);
    auto*mode=new QGroupBox("A. Mode Kalibrasi Protocol Langsung");
    auto*mg=new QHBoxLayout(mode);
    start_=new QPushButton("MULAI MODE KALIBRASI");
    stop_=new QPushButton("BATAL / KELUAR MODE");
    state_=new StatusPill("MODE NORMAL");
    mg->addWidget(start_);
    mg->addWidget(stop_);
    mg->addWidget(state_);
    l->addWidget(mode);
    auto*live=new QGroupBox("B. Data Live — Pisahkan Domain Protocol dan Fisik");
    auto*lg=new QGridLayout(live);
    cmd_=new QLabel("Protocol CMD: --");
    fb_=new QLabel("ACK FB raw: --");
    logicalT_=new QLabel("Target sudut roda fisik: --");
    logicalA_=new QLabel("Actual sudut roda fisik: --");
    lg->addWidget(cmd_,0,0);
    lg->addWidget(fb_,0,1);
    lg->addWidget(logicalT_,1,0);
    lg->addWidget(logicalA_,1,1);
    l->addWidget(live);
    auto*cap=new QGroupBox("C. Capture Endpoint Protocol + Feedback");
    auto*cg=new QGridLayout(cap);
    center_=new QPushButton("1. CAPTURE LURUS");
    left_=new QPushButton("2. CAPTURE KIRI MAX");
    right_=new QPushButton("3. CAPTURE KANAN MAX");
    centerV_=new QLabel("CENTER: --");
    leftV_=new QLabel("LEFT: --");
    rightV_=new QLabel("RIGHT: --");
    cg->addWidget(center_,0,0);
    cg->addWidget(left_,0,1);
    cg->addWidget(right_,0,2);
    cg->addWidget(centerV_,1,0);
    cg->addWidget(leftV_,1,1);
    cg->addWidget(rightV_,1,2);
    l->addWidget(cap);
    auto*physical=new QGroupBox("D. Masukkan Hasil Pengukuran Sudut Roda Nyata");
    auto*pg=new QGridLayout(physical);
    leftPhysical_=new QDoubleSpinBox();
    rightPhysical_=new QDoubleSpinBox();
    marginPct_=new QDoubleSpinBox();
    for(auto*w:{
      leftPhysical_,rightPhysical_
    }){
      w->setRange(1.0,70.0);
      w->setDecimals(2);
      w->setSingleStep(0.5);
      w->setSuffix("°");
      w->setValue(30.0);
    }
    marginPct_->setRange(70.0,100.0);
    marginPct_->setDecimals(1);
    marginPct_->setSingleStep(1.0);
    marginPct_->setSuffix(" %");
    marginPct_->setValue(95.0);
    pg->addWidget(new QLabel("Sudut fisik KIRI (magnitudo)"),0,0);
    pg->addWidget(leftPhysical_,0,1);
    pg->addWidget(new QLabel("Sudut fisik KANAN (magnitudo)"),1,0);
    pg->addWidget(rightPhysical_,1,1);
    pg->addWidget(new QLabel("Batas operasional dari sisi terkecil"),2,0);
    pg->addWidget(marginPct_,2,1);
    auto*help=new QLabel("Contoh: jika maksimum kiri nyata 31.5° dan kanan 28.0°, masukkan 31.5 dan 28.0. "
    "Dengan margin 95%, Teleop/Nav2 dibatasi ±26.6°. Endpoint protocol tetap tersimpan terpisah.");
    help->setWordWrap(true);
    pg->addWidget(help,3,0,1,2);
    l->addWidget(physical);
    auto*lut=new QGroupBox("E. Part 2 — Multi-Point LUT + Hysteresis / Backlash");
    auto*lutLayout=new QVBoxLayout(lut);
    auto*lutHelp=new QLabel(
    "Ambil minimal 5 sudut fisik yang SAMA pada dua sweep. Sweep 1 wajib monoton KIRI→KANAN; "
    "Sweep 2 monoton KANAN→KIRI. Pada setiap titik, ukur sudut roda fisik dengan angle gauge, "
    "masukkan angkanya, tahan steering ±1 detik, lalu Capture. Controller akan memakai interpolasi piecewise "
    "dan memilih kurva sesuai arah gerak steering. Jangan melompat bolak-balik di tengah satu sweep.");
    lutHelp->setWordWrap(true);
    lutLayout->addWidget(lutHelp);
    auto*lutBar=new QHBoxLayout();
    sweep_=new QComboBox();
    sweep_->addItems({
      "Sweep KIRI → KANAN (increasing)","Sweep KANAN → KIRI (decreasing)"
    });
    physicalPoint_=new QDoubleSpinBox();
    physicalPoint_->setRange(-70.0,70.0);
    physicalPoint_->setDecimals(2);
    physicalPoint_->setSingleStep(2.5);
    physicalPoint_->setSuffix("° fisik");
    captureLut_=new QPushButton("Capture Titik LUT");
    deleteLut_=new QPushButton("Hapus Titik");
    clearLut_=new QPushButton("Reset Draft LUT");
    lutBar->addWidget(sweep_);
    lutBar->addWidget(physicalPoint_);
    lutBar->addWidget(captureLut_);
    lutBar->addWidget(deleteLut_);
    lutBar->addWidget(clearLut_);
    lutLayout->addLayout(lutBar);
    lutTable_=new QTableWidget(0,7);
    lutTable_->setHorizontalHeaderLabels({
      "Sudut fisik°","CMD L→R°","FB L→R°","CMD R→L°","FB R→L°","ΔCMD°","ΔFB°"
    });
    lutTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    lutTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    lutLayout->addWidget(lutTable_);
    auto*lutAction=new QHBoxLayout();
    applyLut_=new QPushButton("TERAPKAN LUT PART 2 ke YAML & Runtime");
    applyLut_->setObjectName("primaryButton");
    exportLut_=new QPushButton("Ekspor LUT CSV");
    lutStatus_=new QLabel("LUT draft belum lengkap");
    lutStatus_->setWordWrap(true);
    lutAction->addWidget(applyLut_);
    lutAction->addWidget(exportLut_);
    lutAction->addWidget(lutStatus_,1);
    lutLayout->addLayout(lutAction);
    l->addWidget(lut);
    auto*validation=new QGroupBox("F. Validasi & Parameter Turunan");
    auto*vg=new QVBoxLayout(validation);
    calc_=new QPlainTextEdit();
    calc_->setReadOnly(true);
    calc_->setMaximumHeight(150);
    vg->addWidget(calc_);
    l->addWidget(validation);
    apply_=new QPushButton("TERAPKAN: Protocol + Feedback + Sudut Fisik ke YAML & Runtime");
    apply_->setObjectName("primaryButton");
    l->addWidget(apply_);
    saved_=new QLabel();
    saved_->setWordWrap(true);
    l->addWidget(saved_);
    l->addStretch();
    connect(start_,&QPushButton::clicked,this,[this](){
      startMode();
    });
    connect(stop_,&QPushButton::clicked,this,[this](){
      stopMode();
    });
    connect(center_,&QPushButton::clicked,this,[this](){
      capture("center");
    });
    connect(left_,&QPushButton::clicked,this,[this](){
      capture("left");
    });
    connect(right_,&QPushButton::clicked,this,[this](){
      capture("right");
    });
    connect(apply_,&QPushButton::clicked,this,[this](){
      applyCalibration();
    });
    connect(captureLut_,&QPushButton::clicked,this,[this](){
      captureLutPoint();
    });
    connect(deleteLut_,&QPushButton::clicked,this,[this](){
      deleteLutPoint();
    });
    connect(clearLut_,&QPushButton::clicked,this,[this](){
      clearLutDraft();
    });
    connect(applyLut_,&QPushButton::clicked,this,[this](){
      applyLutCalibration();
    });
    connect(exportLut_,&QPushButton::clicked,this,[this](){
      exportLutCsv();
    });
    connect(leftPhysical_,qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this](double){
      refreshCalculation();
    });
    connect(rightPhysical_,qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this](double){
      refreshCalculation();
    });
    connect(marginPct_,qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this](double){
      refreshCalculation();
    });
    connect(ros_,&RosBridge::serviceResult,this,[this](const QString&tag,bool ok,const QString&msg){
      handleResult(tag,ok,msg);
    });
    timer_=new QTimer(this);
    timer_->setInterval(100);
    connect(timer_,&QTimer::timeout,this,[this](){
      refresh();
    });
    timer_->start();
    loadPhysicalSaved();
    loadLutDraft();
    refreshLutTable();
    refreshSaved();
    refreshCalculation();
  }
  private:
  struct Point{
    double cmd=NAN,fb=NAN;
    bool valid=false;
  };
  QMap<QString,std::shared_ptr<YamlStore>>s_;
  TelemetryStore*t_;
  RosBridge*ros_;
  QPushButton*start_,*stop_,*left_,*center_,*right_,*apply_;
  StatusPill*state_;
  QLabel*cmd_,*fb_,*logicalT_,*logicalA_,*leftV_,*centerV_,*rightV_,*saved_;
  QDoubleSpinBox*leftPhysical_,*rightPhysical_,*marginPct_;
  QPlainTextEdit*calc_;
  QTimer*timer_;
  QComboBox*sweep_;
  QDoubleSpinBox*physicalPoint_;
  QPushButton*captureLut_,*deleteLut_,*clearLut_,*applyLut_,*exportLut_;
  QTableWidget*lutTable_;
  QLabel*lutStatus_;
  bool mode_=false,busy_=false;
  std::deque<double>cmdHist_,fbHist_;
  QMap<QString,Point>pts_;
  QMap<QString,QByteArray> backups_;
  QVector<QVariantMap>lutRows_;
  static double radDeg(const QVariant&v){
    double x=number(v);
    return std::isfinite(x)?x*180.0/kPi:NAN;
  }
  static std::pair<double,double> medSpread(const std::deque<double>&src){
    std::vector<double>v;
    int begin=std::max<int>(0,src.size()-9);
    for(int i=begin;
    i<(int)src.size();
    ++i)if(std::isfinite(src[i]))v.push_back(src[i]);
    if(v.empty())return{
      NAN,NAN
    };
    std::sort(v.begin(),v.end());
    double med=v[v.size()/2];
    return{
      med,v.back()-v.front()
    };
  }
  void loadPhysicalSaved(){
    auto st=s_.value("esc");
    if(!st)return;
    const QString b="esc_ackermann.ros__parameters.";
    double l=std::abs(number(st->get(b+"steering_physical_left_limit_deg",-30.0),-30.0));
    double r=std::abs(number(st->get(b+"steering_physical_right_limit_deg",30.0),30.0));
    if(l>=1&&l<=70)leftPhysical_->setValue(l);
    if(r>=1&&r<=70)rightPhysical_->setValue(r);
    double common=std::min(l,r),op=number(st->get(b+"steering_physical_operational_limit_deg",0.95*common),0.95*common);
    if(common>0)marginPct_->setValue(std::clamp(100.0*op/common,70.0,100.0));
  }
  void startMode(){
    if(busy_||mode_)return;
    if(std::abs(number(t_->get("esc_drive_actual"),0))>0.05){
      QMessageBox::warning(this,"Steering","Kendaraan harus berhenti sebelum kalibrasi.");
      return;
    }
    busy_=true;
    ros_->setParametersAtomically("/esc_ackermann",{
      {
        "steering_calibration_mode_enabled",true
      }
    },"steering_calibration:mode_on");
  }
  void stopMode(){
    if(busy_)return;
    busy_=true;
    ros_->setParametersAtomically("/esc_ackermann",{
      {
        "steering_calibration_mode_enabled",false
      }
    },"steering_calibration:mode_off");
  }
  void capture(const QString&name){
    if(!mode_){
      QMessageBox::warning(this,"Steering","Aktifkan mode kalibrasi terlebih dahulu.");
      return;
    }
    auto[c,cs]=medSpread(cmdHist_);
    auto[f,fs]=medSpread(fbHist_);
    if(!std::isfinite(c)||!std::isfinite(f)||cs>1.5||fs>2.5){
      QMessageBox::warning(this,"Capture","Data belum stabil. Tahan posisi sekitar 1 detik.");
      return;
    }
    double lim=std::abs(number(s_.value("esc")->get("esc_ackermann.ros__parameters.steering_calibration_direct_limit_deg",90.0),90.0));
    if(name=="left"&&c>-0.70*lim){
      QMessageBox::warning(this,"Kiri","Dorong steering lebih jauh ke kiri hingga endpoint mekanik yang aman.");
      return;
    }
    if(name=="right"&&c<0.70*lim){
      QMessageBox::warning(this,"Kanan","Dorong steering lebih jauh ke kanan hingga endpoint mekanik yang aman.");
      return;
    }
    if(name=="center"&&std::abs(c)>0.35*lim){
      QMessageBox::warning(this,"Lurus","Command masih terlalu jauh dari area center. Luruskan roda secara fisik terlebih dahulu.");
      return;
    }
    pts_[name]={
      c,f,true
    };
    refreshPointLabels();
    refreshCalculation();
  }
  bool geometryOk()const{
    return pts_.value("left").valid&&pts_.value("center").valid&&pts_.value("right").valid&&pts_.value("left").cmd<pts_.value("center").cmd&&pts_.value("center").cmd<pts_.value("right").cmd&&pts_.value("left").fb<pts_.value("center").fb&&pts_.value("center").fb<pts_.value("right").fb;
  }
  double commonMechanicalDeg()const{
    return std::min(leftPhysical_->value(),rightPhysical_->value());
  }
  double operationalDeg()const{
    return commonMechanicalDeg()*marginPct_->value()/100.0;
  }
  double theoreticalTurningRadius(double physicalDeg)const{
    auto v=s_.value("vehicle");
    double wb=number(v->get("vehicle.ros__parameters.effective_wheelbase_m",v->get("vehicle.ros__parameters.wheelbase_m",0.70)),0.70);
    double track=number(v->get("vehicle.ros__parameters.track_width_m",0.48),0.48);
    double a=std::abs(physicalDeg)*kPi/180.0;
    if(a<1e-4)return 999.0;
    double center=0.5*track+wb/std::tan(a);
    return std::max(0.1,center*1.10);
  }
  double temporaryTurningRadius()const{
    return theoreticalTurningRadius(operationalDeg());
  }
  void invalidateCircleCalibration(){
    auto veh=s_.value("vehicle");
    if(!veh)return;
    const QString vb="vehicle.ros__parameters.";
    veh->set(vb+"steering_circle_calibration_valid",false);
    veh->set(vb+"steering_circle_trial_count_left",0);
    veh->set(vb+"steering_circle_trial_count_right",0);
    veh->set(vb+"turning_radius_left_m",0.0);
    veh->set(vb+"turning_radius_right_m",0.0);
    veh->set(vb+"steering_circle_fit_rmse_m",0.0);
    veh->set(vb+"steering_circle_calibration_saved_at",QString());
  }
  void propagateKinematicAuthority(double opRad,double rmin,const QString&source){
    auto veh=s_.value("vehicle");
    if(!veh)return;
    const QString vb="vehicle.ros__parameters.";
    double maxForward=number(veh->get(vb+"max_forward_speed_mps",0.5),0.5);
    double oldYaw=number(veh->get(vb+"max_yaw_rate_rps",maxForward/std::max(0.1,rmin)),maxForward/std::max(0.1,rmin));
    double vehicleYaw=std::min(oldYaw,maxForward/std::max(0.1,rmin));
    veh->set(vb+"minimum_turning_radius_m",rmin);
    veh->set(vb+"minimum_turning_radius_source",source);
    veh->set(vb+"max_yaw_rate_rps",vehicleYaw);
    if(s_.contains("navigation_core")){
      s_["navigation_core"]->set("navigation_core.ros__parameters.max_steering_angle_rad",opRad);
      s_["navigation_core"]->set("navigation_core.ros__parameters.minimum_turning_radius_m",rmin);
      double cur=number(s_["navigation_core"]->get("navigation_core.ros__parameters.max_yaw_rate_rps",vehicleYaw),vehicleYaw);
      s_["navigation_core"]->set("navigation_core.ros__parameters.max_yaw_rate_rps",std::min(cur,vehicleYaw));
    }
    if(s_.contains("nav2")){
      s_["nav2"]->set("planner_server.ros__parameters.GridBased.minimum_turning_radius",rmin);
      s_["nav2"]->set("controller_server.ros__parameters.FollowPath.AckermannConstraints.min_turning_r",rmin);
      double navV=number(s_["nav2"]->get("controller_server.ros__parameters.FollowPath.vx_max",maxForward),maxForward);
      double navYaw=std::min(vehicleYaw,navV/std::max(0.1,rmin));
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
        s_["trajectory_safety"]->set("trajectory_safety_supervisor.ros__parameters.minimum_turning_radius_m",rmin);
        double cur=number(s_["trajectory_safety"]->get("trajectory_safety_supervisor.ros__parameters.maximum_yaw_rate_rps",navYaw),navYaw);
        s_["trajectory_safety"]->set("trajectory_safety_supervisor.ros__parameters.maximum_yaw_rate_rps",std::min(cur,navYaw));
      }
    }
  }
  void refreshCalculation(){
    double common=commonMechanicalDeg(),op=operationalDeg(),r=temporaryTurningRadius();
    auto v=s_.value("vehicle");
    double wb=number(v->get("vehicle.ros__parameters.effective_wheelbase_m",v->get("vehicle.ros__parameters.wheelbase_m",0.70)),0.70),track=number(v->get("vehicle.ros__parameters.track_width_m",0.48),0.48);
    calc_->setPlainText(QString("Measured physical LEFT = -%1°\nMeasured physical RIGHT = +%2°\nConservative mechanical common limit = ±%3°\nOperational limit (%4%) = ±%5°\nTemporary theoretical center turning radius = %6 m (L=%7 m, track=%8 m, includes 10% safety margin)\nPart 3 nanti mengganti radius teoritis ini dengan radius circle-test nyata.").arg(leftPhysical_->value(),0,'f',2).arg(rightPhysical_->value(),0,'f',2).arg(common,0,'f',2).arg(marginPct_->value(),0,'f',1).arg(op,0,'f',2).arg(r,0,'f',3).arg(wb,0,'f',3).arg(track,0,'f',3));
  }
  void applyCalibration(){
    if(!mode_||!geometryOk()){
      QMessageBox::warning(this,"Apply","Capture LURUS/KIRI/KANAN belum valid dan monoton.");
      return;
    }
    const double leftPhysical=-leftPhysical_->value(), rightPhysical=rightPhysical_->value(), op=operationalDeg();
    if(!(leftPhysical<-1.0&&rightPhysical>1.0&&op>=1.0&&op<=std::min(-leftPhysical,rightPhysical))){
      QMessageBox::warning(this,"Sudut fisik","Nilai sudut fisik atau margin operasional tidak valid.");
      return;
    }
    if(QMessageBox::question(this,"Apply Steering Fisik",QString("Terapkan mapping protocol ke sudut roda nyata?\nLEFT %1° • RIGHT +%2° • operational ±%3°\n\nSetelah ini /esc/steering_actual dan odometri memakai sudut fisik.").arg(leftPhysical,0,'f',2).arg(rightPhysical,0,'f',2).arg(op,0,'f',2))!=QMessageBox::Yes)return;
    auto st=s_.value("esc");
    backups_.clear();
    for(const QString&k:{
      QString("esc"),QString("vehicle"),QString("navigation_core"),QString("nav2"),QString("trajectory_safety")
    })if(s_.contains(k))backups_[k]=s_[k]->raw();
    QString base="esc_ackermann.ros__parameters.";
    QVariantMap params;
    params["steering_feedback_left_stop_deg"]=pts_["left"].cmd;
    params["steering_feedback_center_deg"]=pts_["center"].cmd;
    params["steering_feedback_right_stop_deg"]=pts_["right"].cmd;
    params["steering_feedback_left_reference_deg"]=pts_["left"].fb;
    params["steering_feedback_center_reference_deg"]=pts_["center"].fb;
    params["steering_feedback_right_reference_deg"]=pts_["right"].fb;
    params["steering_feedback_calibration_enabled"]=true;
    params["steering_feedback_force_symmetric_span"]=false;
    params["steering_physical_calibration_enabled"]=true;
    params["steering_physical_lut_enabled"]=false;
    // Part-1 re-apply invalidates any older LUT until Part 2 is re-qualified.
    params["steering_physical_left_limit_deg"]=leftPhysical;
    params["steering_physical_right_limit_deg"]=rightPhysical;
    params["steering_physical_operational_limit_deg"]=op;
    params["steering_physical_calibration_saved_at"]=QDateTime::currentDateTime().toString(Qt::ISODate)+"-PHYSICAL-P1";
    params["steering_center_hold_enabled"]=true;
    params["steering_center_hold_ki"]=0.0;
    params["steering_center_hold_integral_limit_deg_s"]=0.0;
    params["steering_center_bias_from_left_deg"]=0.0;
    params["steering_center_bias_from_right_deg"]=0.0;
    params["steering_calibration_mode_enabled"]=false;
    params["steering_feedback_calibration_saved_at"]=QDateTime::currentDateTime().toString(Qt::ISODate)+"-PHYSICAL-P1";
    params["steering_calibration_apply_token"]="PHYSICAL-P1:"+QString::number(QDateTime::currentMSecsSinceEpoch());
    for(auto it=params.cbegin();
    it!=params.cend();
    ++it){
      QString err;
      if(!st->set(base+it.key(),it.value(),&err)){
        for(auto b=backups_.cbegin();
        b!=backups_.cend();
        ++b)if(s_.contains(b.key()))s_[b.key()]->restoreRaw(b.value());
        QMessageBox::critical(this,"YAML",err);
        return;
      }
    }
    // Vehicle authority stores measured asymmetric endpoints plus conservative symmetric limits.
    auto veh=s_.value("vehicle");
    const QString vb="vehicle.ros__parameters.";
    const double common=commonMechanicalDeg(), commonRad=common*kPi/180.0, opRad=op*kPi/180.0;
    const double rmin=theoreticalTurningRadius(op);
    veh->set(vb+"measured_left_steering_limit_rad",leftPhysical*kPi/180.0);
    veh->set(vb+"measured_right_steering_limit_rad",rightPhysical*kPi/180.0);
    veh->set(vb+"max_steering_angle_rad",commonRad);
    veh->set(vb+"operational_steering_angle_rad",opRad);
    veh->set(vb+"steering_calibration_valid",true);
    invalidateCircleCalibration();
    propagateKinematicAuthority(opRad,rmin,QString("theoretical_part1_recalibration_required"));
    veh->set(vb+"steering_calibration_source",QString("physical_3point_part1_lut_disabled"));
    veh->set(vb+"steering_calibration_saved_at",QDateTime::currentDateTime().toString(Qt::ISODate));
    busy_=true;
    ros_->setParametersAtomically("/esc_ackermann",params,"steering_calibration:apply");
  }
  void handleResult(const QString&tag,bool ok,const QString&msg){
    if(!tag.startsWith("steering_calibration:"))return;
    busy_=false;
    if(tag.endsWith("mode_on")){
      mode_=ok;
      state_->setStatus(ok?"warn":"bad",ok?"CAL MODE ON":"MODE GAGAL");
    }
    else if(tag.endsWith("mode_off")){
      if(ok)mode_=false;
      state_->setStatus(ok?"unknown":"bad",ok?"MODE NORMAL":"MODE GAGAL");
    }
    else if(tag.contains("apply")){
      if(ok){
        mode_=false;
        state_->setStatus("ok",tag.contains("lut")?"LUT FISIK AKTIF":"FISIK KALIBRASI AKTIF");
        refreshSaved();
        refreshLutTable();
      }
      else{
        for(auto b=backups_.cbegin();
        b!=backups_.cend();
        ++b)if(s_.contains(b.key()))s_[b.key()]->restoreRaw(b.value());
        state_->setStatus("bad","TERAPKAN GAGAL");
      }
      QMessageBox::information(this,"Steering Runtime",msg);
    }
  }
  void refresh(){
    double c=radDeg(t_->get("esc_steer_protocol_cmd")),f=radDeg(t_->get("esc_steer_feedback_raw"));
    if(std::isfinite(c)){
      cmdHist_.push_back(c);
      while(cmdHist_.size()>25)cmdHist_.pop_front();
    }
    if(std::isfinite(f)){
      fbHist_.push_back(f);
      while(fbHist_.size()>25)fbHist_.pop_front();
    }
    cmd_->setText(QString("Protocol CMD: %1°").arg(std::isfinite(c)?QString::number(c,'f',3):"--"));
    fb_->setText(QString("ACK FB raw: %1°").arg(std::isfinite(f)?QString::number(f,'f',3):"--"));
    logicalT_->setText(QString("Target sudut roda fisik: %1°").arg(variantText(radDeg(t_->get("esc_steer_target")))));
    logicalA_->setText(QString("Actual sudut roda fisik: %1°").arg(variantText(radDeg(t_->get("esc_steer_actual")))));
  }
  void refreshPointLabels(){
    auto txt=[&](const QString&k){
      const auto&p=pts_[k];
      return p.valid?QString("CMD=%1°\nFB=%2°").arg(p.cmd,0,'f',3).arg(p.fb,0,'f',3):QString("--");
    };
    centerV_->setText("CENTER: "+txt("center"));
    leftV_->setText("LEFT: "+txt("left"));
    rightV_->setText("RIGHT: "+txt("right"));
  }
  static QVariantList vectorToList(const QVector<double>&v){
    QVariantList out;
    for(double x:v)out<<x;
    return out;
  }
  void saveLutDraft(){
    QVariantList list;
    for(const auto&r:lutRows_)list<<r;
    if(s_.contains("gui"))s_["gui"]->set("steering_calibration.lut_part2_draft",list);
  }
  void loadLutDraft(){
    lutRows_.clear();
    if(s_.contains("gui")){
      for(const QVariant&v:s_["gui"]->get("steering_calibration.lut_part2_draft",QVariantList{
      }).toList())lutRows_<<v.toMap();
    }
    if(!lutRows_.isEmpty())return;
    auto st=s_.value("esc");
    if(!st)return;
    QString b="esc_ackermann.ros__parameters.";
    QVariantList ph=st->get(b+"steering_lut_physical_deg",QVariantList{
    }).toList(),ci=st->get(b+"steering_lut_command_increasing_deg",QVariantList{
    }).toList(),cd=st->get(b+"steering_lut_command_decreasing_deg",QVariantList{
    }).toList(),fi=st->get(b+"steering_lut_feedback_increasing_deg",QVariantList{
    }).toList(),fd=st->get(b+"steering_lut_feedback_decreasing_deg",QVariantList{
    }).toList();
    int n=ph.size();
    if(n>=5&&ci.size()==n&&cd.size()==n&&fi.size()==n&&fd.size()==n){
      for(int i=0;
      i<n;
      ++i)lutRows_<<QVariantMap{
        {
          "physical",ph[i]
        },{
          "cmd_inc",ci[i]
        },{
          "fb_inc",fi[i]
        },{
          "cmd_dec",cd[i]
        },{
          "fb_dec",fd[i]
        }
      };
    }
  }
  int lutRowNear(double physical)const{
    for(int i=0;
    i<lutRows_.size();
    ++i)if(std::abs(number(lutRows_[i].value("physical"))-physical)<=0.20)return i;
    return -1;
  }
  static bool finiteKey(const QVariantMap&r,const QString&k){
    return std::isfinite(number(r.value(k)));
  }
  void captureLutPoint(){
    if(!mode_){
      QMessageBox::warning(this,"LUT","Aktifkan MODE KALIBRASI dahulu.");
      return;
    }
    auto[c,cs]=medSpread(cmdHist_);
    auto[f,fs]=medSpread(fbHist_);
    if(!std::isfinite(c)||!std::isfinite(f)||cs>1.5||fs>2.5){
      QMessageBox::warning(this,"LUT","Command/feedback belum stabil. Tahan posisi sekitar 1 detik.");
      return;
    }
    double physical=physicalPoint_->value();
    if(std::abs(physical)>70.0){
      QMessageBox::warning(this,"LUT","Sudut fisik di luar batas.");
      return;
    }
    int idx=lutRowNear(physical);
    if(idx<0){
      QVariantMap r{
        {
          "physical",physical
        }
      };
      lutRows_<<r;
      idx=lutRows_.size()-1;
    }
    else lutRows_[idx]["physical"]=physical;
    const bool inc=sweep_->currentIndex()==0;
    lutRows_[idx][inc?"cmd_inc":"cmd_dec"]=c;
    lutRows_[idx][inc?"fb_inc":"fb_dec"]=f;
    std::sort(lutRows_.begin(),lutRows_.end(),[](const QVariantMap&a,const QVariantMap&b){
      return number(a.value("physical"))<number(b.value("physical"));
    });
    saveLutDraft();
    refreshLutTable();
  }
  void deleteLutPoint(){
    int row=lutTable_->currentRow();
    if(row<0||row>=lutRows_.size())return;
    lutRows_.removeAt(row);
    saveLutDraft();
    refreshLutTable();
  }
  void clearLutDraft(){
    if(QMessageBox::question(this,"Reset LUT","Hapus seluruh draft multi-point LUT?")!=QMessageBox::Yes)return;
    lutRows_.clear();
    saveLutDraft();
    refreshLutTable();
  }
  bool buildLut(QVector<double>&ph,QVector<double>&ci,QVector<double>&cd,QVector<double>&fi,QVector<double>&fd,QString&why)const{
    for(const auto&r:lutRows_){
      if(!finiteKey(r,"physical")||!finiteKey(r,"cmd_inc")||!finiteKey(r,"cmd_dec")||!finiteKey(r,"fb_inc")||!finiteKey(r,"fb_dec"))continue;
      ph<<number(r.value("physical"));
      ci<<number(r.value("cmd_inc"));
      cd<<number(r.value("cmd_dec"));
      fi<<number(r.value("fb_inc"));
      fd<<number(r.value("fb_dec"));
    }
    if(ph.size()<5){
      why="Butuh minimal 5 sudut fisik yang lengkap pada KEDUA sweep.";
      return false;
    }
    auto increasing=[](const QVector<double>&v){
      for(int i=1;
      i<v.size();
      ++i)if(!(v[i]>v[i-1]+1e-4))return false;
      return true;
    };
    if(!increasing(ph)||!increasing(ci)||!increasing(cd)||!increasing(fi)||!increasing(fd)){
      why="Physical/CMD/FB harus monoton naik dari kiri ke kanan pada kedua sweep. Ulangi titik yang melanggar.";
      return false;
    }
    if(!(ph.front()<-1&&ph.back()>1)){
      why="LUT harus mencakup sisi kiri dan kanan.";
      return false;
    }
    double minAbs=1e9;
    for(double x:ph)minAbs=std::min(minAbs,std::abs(x));
    if(minAbs>1.0){
      why="LUT wajib mempunyai titik center sekitar 0° fisik.";
      return false;
    }
    if(ph.front()>-0.75*leftPhysical_->value()||ph.back()<0.75*rightPhysical_->value()){
      why="Coverage LUT belum cukup dekat endpoint fisik kiri/kanan (minimal 75%).";
      return false;
    }
    return true;
  }
  void refreshLutTable(){
    lutTable_->setRowCount(lutRows_.size());
    double maxCmd=0,maxFb=0;
    int complete=0;
    for(int i=0;
    i<lutRows_.size();
    ++i){
      const auto&r=lutRows_[i];
      double p=number(r.value("physical")),ci=number(r.value("cmd_inc")),cd=number(r.value("cmd_dec")),fi=number(r.value("fb_inc")),fd=number(r.value("fb_dec"));
      bool ok=std::isfinite(ci)&&std::isfinite(cd)&&std::isfinite(fi)&&std::isfinite(fd);
      if(ok){
        ++complete;
        maxCmd=std::max(maxCmd,std::abs(ci-cd));
        maxFb=std::max(maxFb,std::abs(fi-fd));
      }
      QStringList vals={
        QString::number(p,'f',2),std::isfinite(ci)?QString::number(ci,'f',3):"--",std::isfinite(fi)?QString::number(fi,'f',3):"--",std::isfinite(cd)?QString::number(cd,'f',3):"--",std::isfinite(fd)?QString::number(fd,'f',3):"--",ok?QString::number(std::abs(ci-cd),'f',3):"--",ok?QString::number(std::abs(fi-fd),'f',3):"--"
      };
      for(int c=0;
      c<vals.size();
      ++c)lutTable_->setItem(i,c,new QTableWidgetItem(vals[c]));
    }
    QVector<double>ph,ci,cd,fi,fd;
    QString why;
    bool valid=buildLut(ph,ci,cd,fi,fd,why);
    lutStatus_->setText(valid?QString("READY ✓ • %1 titik lengkap • max hysteresis CMD=%2° FB=%3°").arg(complete).arg(maxCmd,0,'f',3).arg(maxFb,0,'f',3):QString("BELUM VALID • %1").arg(why));
  }
  void applyLutCalibration(){
    if(!mode_){
      QMessageBox::warning(this,"LUT","Aktifkan mode kalibrasi sebelum Apply supaya steering ditahan aman.");
      return;
    }
    QVector<double>ph,ci,cd,fi,fd;
    QString why;
    if(!buildLut(ph,ci,cd,fi,fd,why)){
      QMessageBox::warning(this,"LUT invalid",why);
      return;
    }
    double maxCmd=0,maxFb=0;
    for(int i=0;
    i<ph.size();
    ++i){
      maxCmd=std::max(maxCmd,std::abs(ci[i]-cd[i]));
      maxFb=std::max(maxFb,std::abs(fi[i]-fd[i]));
    }
    int center=0;
    for(int i=1;
    i<ph.size();
    ++i)if(std::abs(ph[i])<std::abs(ph[center]))center=i;
    double centerCmd=0.5*(ci[center]+cd[center]),centerFb=0.5*(fi[center]+fd[center]);
    double leftCmd=0.5*(ci.front()+cd.front()),rightCmd=0.5*(ci.back()+cd.back()),leftFb=0.5*(fi.front()+fd.front()),rightFb=0.5*(fi.back()+fd.back());
    double leftLimit=ph.front(),rightLimit=ph.back(),common=std::min(std::abs(leftLimit),std::abs(rightLimit)),op=common*marginPct_->value()/100.0;
    if(QMessageBox::question(this,"Apply LUT Part 2",QString("Aktifkan %1 titik LUT?\nCoverage %2° .. +%3° • operational ±%4°\nMax hysteresis protocol CMD=%5° FB=%6°\n\nSetelah Apply, command dan feedback memakai piecewise LUT arah-gerak.").arg(ph.size()).arg(leftLimit,0,'f',2).arg(rightLimit,0,'f',2).arg(op,0,'f',2).arg(maxCmd,0,'f',3).arg(maxFb,0,'f',3))!=QMessageBox::Yes)return;
    backups_.clear();
    for(const QString&k:{
      QString("esc"),QString("vehicle"),QString("navigation_core"),QString("nav2"),QString("trajectory_safety")
    })if(s_.contains(k))backups_[k]=s_[k]->raw();
    QVariantMap params{
      {
        "steering_feedback_left_stop_deg",leftCmd
      },{
        "steering_feedback_center_deg",centerCmd
      },{
        "steering_feedback_right_stop_deg",rightCmd
      },{
        "steering_feedback_left_reference_deg",leftFb
      },{
        "steering_feedback_center_reference_deg",centerFb
      },{
        "steering_feedback_right_reference_deg",rightFb
      },{
        "steering_feedback_calibration_enabled",true
      },{
        "steering_feedback_force_symmetric_span",false
      },{
        "steering_physical_calibration_enabled",true
      },{
        "steering_physical_left_limit_deg",leftLimit
      },{
        "steering_physical_right_limit_deg",rightLimit
      },{
        "steering_physical_operational_limit_deg",op
      },{
        "steering_physical_lut_enabled",true
      },{
        "steering_lut_physical_deg",vectorToList(ph)
      },{
        "steering_lut_command_increasing_deg",vectorToList(ci)
      },{
        "steering_lut_command_decreasing_deg",vectorToList(cd)
      },{
        "steering_lut_feedback_increasing_deg",vectorToList(fi)
      },{
        "steering_lut_feedback_decreasing_deg",vectorToList(fd)
      },{
        "steering_lut_direction_deadband_deg",0.15
      },{
        "steering_center_bias_from_left_deg",ci[center]-centerCmd
      },{
        "steering_center_bias_from_right_deg",cd[center]-centerCmd
      },{
        "steering_physical_calibration_saved_at",QDateTime::currentDateTime().toString(Qt::ISODate)+"-PHYSICAL-P2-LUT"
      },{
        "steering_lut_calibration_saved_at",QDateTime::currentDateTime().toString(Qt::ISODate)+"-P2-LUT"
      },{
        "steering_feedback_calibration_saved_at",QDateTime::currentDateTime().toString(Qt::ISODate)+"-P2-LUT"
      },{
        "steering_calibration_apply_token","PHYSICAL-P2-LUT:"+QString::number(QDateTime::currentMSecsSinceEpoch())
      },{
        "steering_calibration_mode_enabled",false
      }
    };
    auto st=s_.value("esc");
    QString base="esc_ackermann.ros__parameters.";
    for(auto it=params.cbegin();
    it!=params.cend();
    ++it){
      QString err;
      if(!st->set(base+it.key(),it.value(),&err)){
        for(auto b=backups_.cbegin();
        b!=backups_.cend();
        ++b)if(s_.contains(b.key()))s_[b.key()]->restoreRaw(b.value());
        QMessageBox::critical(this,"YAML",err);
        return;
      }
    }
    auto veh=s_.value("vehicle");
    QString vb="vehicle.ros__parameters.";
    double opRad=op*kPi/180.0;
    veh->set(vb+"measured_left_steering_limit_rad",leftLimit*kPi/180.0);
    veh->set(vb+"measured_right_steering_limit_rad",rightLimit*kPi/180.0);
    veh->set(vb+"max_steering_angle_rad",common*kPi/180.0);
    veh->set(vb+"operational_steering_angle_rad",opRad);
    veh->set(vb+"steering_calibration_valid",true);
    veh->set(vb+"steering_calibration_source",QString("physical_multipoint_lut_part2"));
    veh->set(vb+"steering_calibration_saved_at",QDateTime::currentDateTime().toString(Qt::ISODate));
    veh->set(vb+"steering_lut_point_count",ph.size());
    veh->set(vb+"steering_lut_max_command_hysteresis_deg",maxCmd);
    veh->set(vb+"steering_lut_max_feedback_hysteresis_deg",maxFb);
    invalidateCircleCalibration();
    double rmin=theoreticalTurningRadius(op);
    propagateKinematicAuthority(opRad,rmin,QString("theoretical_part2_recalibration_required"));
    busy_=true;
    ros_->setParametersAtomically("/esc_ackermann",params,"steering_calibration:apply_lut");
  }
  void exportLutCsv(){
    QString fn=QFileDialog::getSaveFileName(this,"Ekspor LUT","steering_lut_part2.csv","CSV (*.csv)");
    if(fn.isEmpty())return;
    QSaveFile f(fn);
    if(!f.open(QIODevice::WriteOnly|QIODevice::Text)){
      QMessageBox::warning(this,"CSV","Tidak dapat membuka file.");
      return;
    }
    QTextStream out(&f);
    out<<"physical_deg,cmd_increasing_deg,fb_increasing_deg,cmd_decreasing_deg,fb_decreasing_deg,cmd_hysteresis_deg,fb_hysteresis_deg\n";
    for(const auto&r:lutRows_){
      double p=number(r.value("physical")),ci=number(r.value("cmd_inc")),fi=number(r.value("fb_inc")),cd=number(r.value("cmd_dec")),fd=number(r.value("fb_dec"));
      out<<QString::number(p,'f',4)<<","<<QString::number(ci,'f',4)<<","<<QString::number(fi,'f',4)<<","<<QString::number(cd,'f',4)<<","<<QString::number(fd,'f',4)<<","<<QString::number(std::abs(ci-cd),'f',4)<<","<<QString::number(std::abs(fi-fd),'f',4)<<"\n";
    }
    if(!f.commit())QMessageBox::warning(this,"CSV","Gagal menyimpan CSV.");
    else QMessageBox::information(this,"CSV","LUT Part 2 tersimpan: "+fn);
  }
  void refreshSaved(){
    auto st=s_.value("esc");
    if(!st)return;
    QString b="esc_ackermann.ros__parameters.";
    bool physical=st->get(b+"steering_physical_calibration_enabled",false).toBool();
    bool lut=st->get(b+"steering_physical_lut_enabled",false).toBool();
    int n=st->get(b+"steering_lut_physical_deg",QVariantList{
    }).toList().size();
    saved_->setText(QString("Physical=%1 • LUT=%2 (%3 titik) • wheel L/R=%4 / +%5° • operational ±%6°\nProtocol CMD L/C/R=%7 / %8 / %9° • ACK FB L/C/R=%10 / %11 / %12°\nSaved=%13 • LUT Saved=%14").arg(physical?"ACTIVE":"BELUM DIKALIBRASI").arg(lut?"ACTIVE":"OFF").arg(n).arg(st->get(b+"steering_physical_left_limit_deg",-30.0).toDouble(),0,'f',2).arg(st->get(b+"steering_physical_right_limit_deg",30.0).toDouble(),0,'f',2).arg(st->get(b+"steering_physical_operational_limit_deg",28.0).toDouble(),0,'f',2).arg(st->get(b+"steering_feedback_left_stop_deg").toDouble(),0,'f',2).arg(st->get(b+"steering_feedback_center_deg").toDouble(),0,'f',2).arg(st->get(b+"steering_feedback_right_stop_deg").toDouble(),0,'f',2).arg(st->get(b+"steering_feedback_left_reference_deg").toDouble(),0,'f',2).arg(st->get(b+"steering_feedback_center_reference_deg").toDouble(),0,'f',2).arg(st->get(b+"steering_feedback_right_reference_deg").toDouble(),0,'f',2).arg(st->get(b+"steering_physical_calibration_saved_at").toString()).arg(st->get(b+"steering_lut_calibration_saved_at").toString()));
  }
};
