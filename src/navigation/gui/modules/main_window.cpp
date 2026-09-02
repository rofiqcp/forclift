// Extracted from agv_gui.cpp for maintainability.
class MainWindow:public QMainWindow{
  public:MainWindow(){
    paths_=WorkspacePaths::detect();
    files_=paths_.fileMap();
    ensureGuiYaml();
    for(auto it=files_.cbegin();
    it!=files_.cend();
    ++it){
      if((it.value().endsWith(".yaml")||it.value().endsWith(".yml"))&&QFileInfo::exists(it.value()))stores_[it.key()]=std::make_shared<YamlStore>(it.value());
    }
    telemetry_=std::make_unique<TelemetryStore>();
    reports_=std::make_unique<ReportManager>(stores_.value("gui"),stores_);
    ros_=std::make_unique<RosBridge>();
    tabs_=buildTabs();
    setWindowTitle(kAppTitle+" — C++/Qt ROS 2");
    setMinimumSize(900,600);
    buildUi();
    applyStyle();
    connect(ros_.get(),&RosBridge::telemetry,this,[this](const QString&ch,const QVariant&v){
      telemetry_->update(ch,v);
    });
    connect(ros_.get(),&RosBridge::image,this,[this](const QImage&i){
      if(cameraPage_)cameraPage_->setImage(i);
    });
    connect(ros_.get(),&RosBridge::ready,this,[this](bool ok,const QString&m){
      rosPill_->setStatus(ok?"ok":"bad",ok?"ROS C++ OK":"ROS OFF");
      rosPill_->setToolTip(m);
    });
    connect(ros_.get(),&RosBridge::serviceResult,this,[this](const QString&tag,bool ok,const QString&m){
      if(tag.startsWith("steering_calibration:"))return;
      if(tag.startsWith("live_yaml:")){
        saveLabel_->setText(ok?QString("YAML + runtime sinkron ✓  %1").arg(tag.mid(10)):QString("YAML tersimpan • runtime belum menerima (%1): %2 • restart node bila parameter tidak dinamis").arg(tag.mid(10),m));
        return;
      }
      if(ok) QMessageBox::information(this,tag,m);
      else QMessageBox::warning(this,tag,m);
    });
    connect(ros_.get(),&RosBridge::parametersResult,this,[this](const QString&tag,bool ok,const QVariantMap&v){
      handleRuntimeAudit(tag,ok,v);
    });
    try{
      syncVehicleAuthority();
    }
    catch(const std::exception&e){
      saveLabel_->setText("VEHICLE AUTHORITY ERROR: "+QString::fromUtf8(e.what()));
    }
    ros_->start();
    QTimer::singleShot(0,this,[this](){
      showMaximized();
      responsiveSplit();
    });
  }
  ~MainWindow()override{
    if(ros_)ros_->shutdown();
  }
  protected:void closeEvent(QCloseEvent*e)override{
    if(!pending_.isEmpty()){
      auto r=QMessageBox::question(this,"Parameter belum tersimpan",QString("Ada %1 perubahan. Simpan sebelum menutup?").arg(pending_.size()),QMessageBox::Save|QMessageBox::Discard|QMessageBox::Cancel);
      if(r==QMessageBox::Cancel){
        e->ignore();
        return;
      }
      if(r==QMessageBox::Save){
        saveAll();
        if(!pending_.isEmpty()){
          e->ignore();
          return;
        }
      }
    }
    if(reportsPage_)reportsPage_->stopRosbagSilent();
    ros_->shutdown();
    e->accept();
  }
  void resizeEvent(QResizeEvent*e)override{
    QMainWindow::resizeEvent(e);
    if(menuPopup_)menuPopup_->setMaximumWidth(std::max(420,std::min(760,width()-40)));
  }
  private:WorkspacePaths paths_;
  QMap<QString,QString>files_;
  QMap<QString,std::shared_ptr<YamlStore>>stores_;
  std::unique_ptr<TelemetryStore>telemetry_;
  std::unique_ptr<ReportManager>reports_;
  std::unique_ptr<RosBridge>ros_;
  QVector<TabDef>tabs_;
  QVector<SettingsForm*>forms_;
  QVector<QWidget*>pages_;
  QMap<QString,QVariant>pending_;
  QSplitter*splitter_;
  QStackedWidget*settingsStack_,*pageStack_;
  QFrame*menuPopup_;
  QVBoxLayout*menuLayout_;
  QTabWidget*menuTabs_=nullptr;
  QMap<QString,QTreeWidget*>menuTrees_;
  QLabel*tabLabel_,*saveLabel_;
  StatusPill*rosPill_;
  QTimer*autosave_;
  ExperimentParameterPanel*experimentParamPanel_=nullptr;
  CameraPage*cameraPage_=nullptr;
  ReportsPage*reportsPage_=nullptr;
  MapPage*mapPage_=nullptr;
  OverviewPage*overviewPage_=nullptr;
  SystemOverviewPage*systemOverviewPage_=nullptr;
  QMap<QString,ExperimentWorkspacePage*>experimentPages_;
  QMap<QString,QVariantMap>runtimeExpected_;
  QMap<QString,QVariantMap>runtimeResults_;
  void ensureGuiYaml(){
    QString p=files_.value("gui");
    QDir().mkpath(QFileInfo(p).absolutePath());
    if(!QFileInfo::exists(files_.value("saved_targets"))){
      QSaveFile f(files_.value("saved_targets"));
      if(f.open(QIODevice::WriteOnly)){
        f.write("targets: {}\n");
        f.commit();
      }
    }
    if(QFileInfo::exists(p))return;
    QSaveFile f(p);
    if(f.open(QIODevice::WriteOnly|QIODevice::Text)){
      f.write("# GUI C++ calibration/reporting state\nruntime:\n  live_apply_yaml: true\nnavigation_menu:\n  active_subsystem: navigation\n  active_leaf: 4.2.1\nmap_overlay:\n  pgm_opacity: 0.72\n  osm_opacity: 0.95\n  osm_line_width_px: 2.2\n  offset_x_m: 0.0\n  offset_y_m: 0.0\n  yaw_deg: 0.0\n  scale: 1.0\nground_truth_points: []\nreporting:\n  output_directory: ~/.ros/agv_gui_reports\n  sample_rate_hz: 5.0\n  project_name: UNDIP AGV\n  operator: ''\n  session_id: ''\n  experiment_category: ''\n  experiment_variant: ''\n  experiment_notes: ''\nesc_calibration:\n  trials: []\n  last_drive_scale: 1.0\n  last_effective_wheelbase_m: 0.70\ncamera_overlay:\n  obstacle_roi_points_px: [180, 700, 1100, 700, 780, 380, 500, 380]\n  lane_safety_points_px: [340, 700, 940, 700, 740, 420, 540, 420]\n  show_ground_plane: true\n  show_obstacle_roi: true\n  show_lane_safety: true\npatrol:\n  target_names: []\n  dwell_sec: 2.0\n  loop: true\n");
      f.commit();
    }
  }
  QString logo()const{
    QString a=QString::fromStdString((paths_.navShare/"gui/assets/undip_logo.png").string());
    return QFileInfo::exists(a)?a:QString::fromStdString((paths_.navSource/"gui/assets/undip_logo.png").string());
  }
  void buildUi(){
    auto*root=new QWidget();
    setCentralWidget(root);
    auto*rl=new QVBoxLayout(root);
    rl->setContentsMargins(0,0,0,0);
    splitter_=new QSplitter(Qt::Horizontal);
    splitter_->setChildrenCollapsible(false);
    splitter_->setHandleWidth(6);
    rl->addWidget(splitter_);
    auto*left=new QFrame();
    left->setObjectName("leftPanel");
    auto*ll=new QVBoxLayout(left);
    ll->setContentsMargins(0,0,0,0);
    auto*brand=new QFrame();
    brand->setObjectName("brandHeader");
    auto*bl=new QHBoxLayout(brand);
    auto*logoL=new QLabel();
    QPixmap pix(logo());
    if(!pix.isNull())logoL->setPixmap(pix.scaled(58,58,Qt::KeepAspectRatio,Qt::SmoothTransformation));
    logoL->setFixedSize(64,64);
    auto*title=new QLabel("Autonomous Vehicle Interface\nSekolah Vokasi • Universitas Diponegoro");
    title->setObjectName("brandTitle");
    bl->addWidget(logoL);
    bl->addWidget(title,1);
    ll->addWidget(brand);
    auto*nav=new QHBoxLayout();
    auto*menu=new QPushButton(QStringLiteral("☰"));
    menu->setObjectName(QStringLiteral("floatingMenuButton"));
    menu->setToolTip(QStringLiteral("Buka menu pengujian BAB IV: Navigasi, Persepsi, dan ESC"));
    menu->setFixedSize(44,40);
    auto*overviewBtn=new QPushButton(QStringLiteral("OVERVIEW"));
    overviewBtn->setObjectName(QStringLiteral("floatingMenuButton"));
    overviewBtn->setFixedSize(96,40);
    overviewBtn->setToolTip(QStringLiteral("Buka halaman System Overview / Vehicle System Health"));
    tabLabel_=new QLabel();
    tabLabel_->setObjectName("currentTabLabel");
    tabLabel_->setWordWrap(true);
    nav->addWidget(menu);
    nav->addWidget(overviewBtn);
    nav->addWidget(tabLabel_,1);
    ll->addLayout(nav);
    settingsStack_=new QStackedWidget();
    for(const auto&t:tabs_){
      auto*scroll=new QScrollArea();
      scroll->setWidgetResizable(true);
      scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
      auto*form=new SettingsForm(t.name,t.specs,stores_);
      forms_<<form;
      connect(form,&SettingsForm::valueEdited,this,[this](const QString&fk,const QString&path,const QVariant&v){
        stageChange(fk,path,v);
      });
      scroll->setWidget(form);
      settingsStack_->addWidget(scroll);
    }
    // Sidebar: navigation tree (old settings stack, demoted) + dynamic experiment
    // parameter panel (focus for BAB IV acquisition).
    settingsStack_->setObjectName(QStringLiteral("legacySettingsStack"));
    ll->addWidget(settingsStack_,1);
    experimentParamPanel_=new ExperimentParameterPanel();
    ll->addWidget(experimentParamPanel_,1);
    saveLabel_=new QLabel("YAML sumber konfigurasi utama • autosave 350 ms");
    saveLabel_->setWordWrap(true);
    ll->addWidget(saveLabel_);
    splitter_->addWidget(left);
    auto*right=new QFrame();
    right->setObjectName("rightPanel");
    auto*rr=new QVBoxLayout(right);
    auto*top=new QHBoxLayout();
    rosPill_=new StatusPill("ROS START");
    auto*saveNow=new QPushButton("Simpan YAML");
    auto*reloadYamlBtn=new QPushButton("Muat Ulang YAML");
    auto*verify=new QPushButton("Verifikasi Runtime");
    auto*max=new QPushButton("□ Maksimalkan");
    auto*full=new QPushButton("⛶ Layar Penuh");
    top->addWidget(rosPill_);
    top->addStretch();
    top->addWidget(saveNow);
    top->addWidget(reloadYamlBtn);
    top->addWidget(verify);
    top->addWidget(max);
    top->addWidget(full);
    rr->addLayout(top);
    pageStack_=new QStackedWidget();
    for(const auto&t:tabs_){
      QWidget*p=makePage(t);
      pages_<<p;
      pageStack_->addWidget(p);
    }
    rr->addWidget(pageStack_,1);
    splitter_->addWidget(right);
    menuPopup_=new QFrame(this,Qt::Popup);
    menuPopup_->setObjectName(QStringLiteral("menuPopup"));
    menuPopup_->setMinimumSize(650,520);
    menuPopup_->setMaximumSize(760,720);
    menuLayout_=new QVBoxLayout(menuPopup_);
    menuLayout_->setContentsMargins(12,12,12,12);
    menuLayout_->setSpacing(8);
    auto*menuTitle=new QLabel(QStringLiteral("Pengujian BAB IV"));
    menuTitle->setObjectName(QStringLiteral("floatingMenuTitle"));
    auto*menuHelp=new QLabel(QStringLiteral("Pilih subsistem, lalu buka subbab 4.1, 4.2, dan seterusnya. Klik subpengujian untuk membuka workspace yang terintegrasi ROS/YAML."));
    menuHelp->setObjectName(QStringLiteral("floatingMenuHelp"));
    menuHelp->setWordWrap(true);
    menuLayout_->addWidget(menuTitle);
    menuLayout_->addWidget(menuHelp);
    menuTabs_=new QTabWidget();
    menuTabs_->setObjectName(QStringLiteral("bab4MenuTabs"));
    menuTabs_->setDocumentMode(true);
    menuLayout_->addWidget(menuTabs_,1);

    auto addSubsystemMenu=[this](const QString&subsystem,const QString&label){
      auto*tree=new QTreeWidget();
      tree->setObjectName(QStringLiteral("bab4MenuTree_")+subsystem);
      tree->setHeaderHidden(true);
      tree->setColumnCount(1);
      tree->setSelectionMode(QAbstractItemView::SingleSelection);
      tree->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
      tree->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
      tree->setIndentation(18);
      tree->setAnimated(true);
      const QVector<ExperimentSpec>specs=buildExperimentCatalog(subsystem);
      QMap<QString,QTreeWidgetItem*>groups;
      for(const ExperimentSpec&spec:specs){
        QTreeWidgetItem*&group=groups[spec.groupId];
        if(!group){
          group=new QTreeWidgetItem(tree);
          group->setText(0,spec.groupTitle);
          group->setData(0,Qt::UserRole,QStringLiteral("group:" )+subsystem+QStringLiteral(":")+spec.groupId);
          group->setExpanded(spec.groupId==QStringLiteral("4.1"));
        }
        auto*leaf=new QTreeWidgetItem(group);
        leaf->setText(0,spec.section);
        leaf->setToolTip(0,spec.section);
        leaf->setData(0,Qt::UserRole,QStringLiteral("leaf:")+subsystem+QStringLiteral(":")+spec.id);
      }
      connect(tree,&QTreeWidget::itemClicked,this,[this,tree](QTreeWidgetItem*item,int){
        if(!item)return;
        const QString data=item->data(0,Qt::UserRole).toString();
        if(data.startsWith(QStringLiteral("group:"))){
          item->setExpanded(!item->isExpanded());
          return;
        }
        if(!data.startsWith(QStringLiteral("leaf:")))return;
        const QStringList parts=data.split(':');
        if(parts.size()!=3)return;
        selectExperimentLeaf(parts[1],parts[2],true);
        tree->setCurrentItem(item);
        menuPopup_->hide();
      });
      menuTrees_[subsystem]=tree;
      menuTabs_->addTab(tree,label);
    };
    addSubsystemMenu(QStringLiteral("navigation"),QStringLiteral("Navigasi"));
    addSubsystemMenu(QStringLiteral("perception"),QStringLiteral("Persepsi"));
    addSubsystemMenu(QStringLiteral("steering"),QStringLiteral("ESC"));

    connect(menu,&QPushButton::clicked,this,[this,menu](){
      if(menuPopup_->isVisible()){
        menuPopup_->hide();
        return;
      }
      menuPopup_->adjustSize();
      const QPoint p=menu->mapToGlobal(QPoint(0,menu->height()+6));
      menuPopup_->move(p);
      menuPopup_->show();
      menuPopup_->raise();
    });
    connect(overviewBtn,&QPushButton::clicked,this,[this](){
      const int i=systemOverviewTabIndex();
      if(i>=0)selectTab(i);
    });
    connect(saveNow,&QPushButton::clicked,this,[this](){
      if(pending_.isEmpty()){
        saveLabel_->setText("Tidak ada perubahan YAML");
        return;
      }
      saveAll();
    });
    connect(reloadYamlBtn,&QPushButton::clicked,this,[this](){
      reloadAllYaml();
    });
    connect(max,&QPushButton::clicked,this,[this](){
      isMaximized()?showNormal():showMaximized();
    });
    connect(full,&QPushButton::clicked,this,[this](){
      isFullScreen()?showMaximized():showFullScreen();
    });
    connect(verify,&QPushButton::clicked,this,[this](){
      verifyRuntime();
    });
    autosave_=new QTimer(this);
    autosave_->setSingleShot(true);
    autosave_->setInterval(350);
    connect(autosave_,&QTimer::timeout,this,[this](){
      saveAll();
    });
    restoreFloatingMenuSelection();
    responsiveSplit();
  }
  QWidget*makePage(const TabDef&t){
    if(t.pageKind=="connection")return new ConnectionPage(telemetry_.get());
    if(t.pageKind=="overview"){
      overviewPage_=new OverviewPage(stores_,telemetry_.get(),ros_.get());
      return overviewPage_;
    }
    if(t.pageKind=="system_overview"){
      systemOverviewPage_=new SystemOverviewPage(telemetry_.get());
      return systemOverviewPage_;
    }
    if(t.pageKind=="experiment_navigation"){
      auto*p=new ExperimentWorkspacePage("navigation",telemetry_.get(),reports_.get(),stores_);
      experimentPages_["navigation"]=p;
      return p;
    }
    if(t.pageKind=="experiment_perception"){
      auto*p=new ExperimentWorkspacePage("perception",telemetry_.get(),reports_.get(),stores_);
      experimentPages_["perception"]=p;
      return p;
    }
    if(t.pageKind=="experiment_steering"){
      auto*p=new ExperimentWorkspacePage("steering",telemetry_.get(),reports_.get(),stores_);
      experimentPages_["steering"]=p;
      return p;
    }
    if(t.pageKind=="map"){
      mapPage_=new MapPage(paths_,stores_,telemetry_.get(),reports_.get(),ros_.get());
      return mapPage_;
    }
    if(t.pageKind=="steering_calibration")return new SteeringCalibrationPage(stores_,telemetry_.get(),ros_.get());
    if(t.pageKind=="odom")return new EscCalibrationPage(stores_,telemetry_.get(),reports_.get(),ros_.get());
    if(t.pageKind=="imu")return new ImuCalibrationPage(telemetry_.get(),reports_.get(),stores_);
    if(t.pageKind=="gnss")return new GnssCalibrationPage(telemetry_.get(),reports_.get(),stores_);
    if(t.pageKind=="camera"){
      cameraPage_=new CameraPage(telemetry_.get(),reports_.get(),stores_);
      return cameraPage_;
    }
    if(t.pageKind=="reports"){
      reportsPage_=new ReportsPage(reports_.get(),stores_);
      return reportsPage_;
    }
    if(t.pageKind=="navigation")return new NavigationTuningPage(telemetry_.get(),reports_.get(),stores_);
    QMap<QString,std::tuple<QString,QString,QMap<QString,QString>,QStringList>>cfg;
    cfg["foc"]={
      "FOC / Steering — BAB IV","Monitor steering target/feedback dan telemetry FOC. Semua parameter eksperimen berada di panel kiri.",{
        {
          "Steer target","esc_steer_target"
        },{
          "Steer actual","esc_steer_actual"
        },{
          "Iq ref","foc_telemetry.iq_ref_a"
        },{
          "Iq","foc_telemetry.iq_a"
        }
      },{
        "foc_telemetry","esc_status"
      }
    };
    cfg["ekf"]={
      "EKF Local / Global Validation","Bandingkan pose, velocity dan covariance local/global.",{
        {
          "Local X","ekf_local.x"
        },{
          "Global X","ekf_global.x"
        },{
          "Local Y","ekf_local.y"
        },{
          "Global Y","ekf_global.y"
        }
      },{
        "ekf_local","ekf_global","ekf_local_status","ekf_global_status"
      }
    };
    cfg["localization"]={
      "Lokalisasi Multi-Point","Monitor map pose, GNSS-map dan quality anchor.",{
        {
          "Map X","localization_state.map_x"
        },{
          "Map Y","localization_state.map_y"
        },{
          "hAcc","localization_state.hacc_m"
        },{
          "Samples","localization_state.calibration_samples"
        }
      },{
        "localization_state","gnss_fusion_status"
      }
    };
    cfg["ground_plane"]={
      "Ground Plane / Pixel-to-Meter","Validasi homography dan metric obstacle/lane.",{
        {
          "Obstacle x","obstacle_metrics.nearest_forward_m"
        },{
          "Obstacle y","obstacle_metrics.nearest_left_m"
        },{
          "Count","object_points.count"
        }
      },{
        "obstacle_metrics","perception_performance"
      }
    };
    cfg["lane"]={
      "Safety Jalur","Validasi clearance, centering dan lane control.",{
        {
          "Center error","lane_state.center_error_m"
        },{
          "Left clearance","lane_state.left_clearance_m"
        },{
          "Right clearance","lane_state.right_clearance_m"
        }
      },{
        "lane_state","lane_control"
      }
    };
    cfg["obstacle"]={
      "Persepsi Obstacle","Validasi confidence, distance, tracking dan blocked/clear.",{
        {
          "Obstacle x","obstacle_metrics.nearest_forward_m"
        },{
          "Obstacle y","obstacle_metrics.nearest_left_m"
        },{
          "Count","object_points.count"
        }
      },{
        "obstacle_metrics","raw_detections","perception_performance"
      }
    };
    cfg["safety"]={
      "Trajectory Safety & Collision Monitor","Monitor command chain dan near-field safety production.",{
        {
          "Nav v","cmd_nav.linear_x"
        },{
          "Integrated v","cmd_autonomy_integrated.linear_x"
        },{
          "Final v","cmd_final.linear_x"
        },{
          "Planning pts","planning_relevant_points.count"
        }
      },{
        "trajectory_safety_state","collision_monitor_state","obstacle_metrics"
      }
    };
    auto c=cfg.value(t.pageKind,std::make_tuple(t.name,t.subtitle,QMap<QString,QString>{
    },QStringList{
    }));
    return new DiagnosticPage(std::get<0>(c),std::get<1>(c),telemetry_.get(),reports_.get(),std::get<2>(c),std::get<3>(c));
  }
  int experimentTabIndex(const QString&subsystem)const{
    const QString kind=QStringLiteral("experiment_")+subsystem;
    for(int i=0;i<tabs_.size();++i){
      if(tabs_[i].pageKind==kind)return i;
    }
    return -1;
  }
  int systemOverviewTabIndex()const{
    for(int i=0;i<tabs_.size();++i){
      if(tabs_[i].pageKind==QStringLiteral("system_overview"))return i;
    }
    return -1;
  }
  QString experimentSection(const QString&subsystem,const QString&id)const{
    const QVector<ExperimentSpec>specs=buildExperimentCatalog(subsystem);
    for(const ExperimentSpec&spec:specs){
      if(spec.id==id)return spec.section;
    }
    return id;
  }
  int subsystemMenuIndex(const QString&subsystem)const{
    if(subsystem==QStringLiteral("navigation"))return 0;
    if(subsystem==QStringLiteral("perception"))return 1;
    if(subsystem==QStringLiteral("steering"))return 2;
    return 0;
  }
  void syncFloatingMenuSelection(const QString&subsystem,const QString&id){
    if(menuTabs_)menuTabs_->setCurrentIndex(subsystemMenuIndex(subsystem));
    QTreeWidget*tree=menuTrees_.value(subsystem,nullptr);
    if(!tree)return;
    QTreeWidgetItemIterator it(tree);
    const QString expected=QStringLiteral("leaf:")+subsystem+QStringLiteral(":")+id;
    while(*it){
      QTreeWidgetItem*item=*it;
      if(item->data(0,Qt::UserRole).toString()==expected){
        tree->setCurrentItem(item);
        if(item->parent())item->parent()->setExpanded(true);
        tree->scrollToItem(item,QAbstractItemView::PositionAtCenter);
        return;
      }
      ++it;
    }
  }
  void selectExperimentLeaf(const QString&subsystem,const QString&id,bool persist){
    const int idx=experimentTabIndex(subsystem);
    if(idx<0)return;
    ExperimentWorkspacePage*p=experimentPages_.value(subsystem,nullptr);
    if(!p)return;
    selectTab(idx);
    p->setParameterPanel(experimentParamPanel_);
    p->selectLeaf(id);
    const QString subsystemLabel=subsystem==QStringLiteral("navigation")?QStringLiteral("Navigasi"):
      subsystem==QStringLiteral("perception")?QStringLiteral("Persepsi"):QStringLiteral("ESC / FOC");
    tabLabel_->setText(subsystemLabel+QStringLiteral(" • ")+experimentSection(subsystem,id));
    syncFloatingMenuSelection(subsystem,id);
    if(persist&&stores_.contains(QStringLiteral("gui"))){
      stores_[QStringLiteral("gui")]->set(QStringLiteral("navigation_menu.active_subsystem"),subsystem);
      stores_[QStringLiteral("gui")]->set(QStringLiteral("navigation_menu.active_leaf"),id);
    }
  }
  void restoreFloatingMenuSelection(){
    QString subsystem=QStringLiteral("navigation");
    QString leaf=QStringLiteral("4.2.1");
    if(stores_.contains(QStringLiteral("gui"))){
      subsystem=stores_[QStringLiteral("gui")]->get(QStringLiteral("navigation_menu.active_subsystem"),subsystem).toString();
      leaf=stores_[QStringLiteral("gui")]->get(QStringLiteral("navigation_menu.active_leaf"),leaf).toString();
    }
    if(!menuTrees_.contains(subsystem)||experimentTabIndex(subsystem)<0){
      subsystem=QStringLiteral("navigation");
      leaf=QStringLiteral("4.2.1");
    }
    bool validLeaf=false;
    for(const ExperimentSpec&spec:buildExperimentCatalog(subsystem)){
      if(spec.id==leaf){validLeaf=true;break;}
    }
    if(!validLeaf){
      const auto specs=buildExperimentCatalog(subsystem);
      leaf=specs.isEmpty()?QStringLiteral("4.1.1"):specs.first().id;
    }
    selectExperimentLeaf(subsystem,leaf,false);
  }
  void selectTab(int i){
    if(i<0||i>=tabs_.size())return;
    settingsStack_->setCurrentIndex(i);
    pageStack_->setCurrentIndex(i);
    tabLabel_->setText(tabs_[i].icon+"  "+tabs_[i].name+" — "+tabs_[i].subtitle);
    if(overviewPage_)overviewPage_->refreshTargets();
    if(mapPage_)mapPage_->refreshTargets();
    if(systemOverviewPage_)systemOverviewPage_->refreshTargets();
  }
  void responsiveSplit(){
    int w=std::max(900,width());
    splitter_->setSizes({
      std::max(280,int(w*.24)),std::max(620,int(w*.76))
    });
  }
  void stageChange(const QString&fk,const QString&path,const QVariant&v){
    pending_[fk+"|"+path]=v;
    saveLabel_->setText(QString("UNSAVED %1 • autosave...").arg(pending_.size()));
    autosave_->start();
  }
  void reloadAllYaml(){
    if(!pending_.isEmpty()&&QMessageBox::question(this,"Muat Ulang YAML",QString("Ada %1 perubahan yang belum tersimpan. Buang perubahan tersebut dan muat ulang file YAML dari disk?").arg(pending_.size()))!=QMessageBox::Yes)return;
    autosave_->stop();
    pending_.clear();
    QStringList failed;
    for(auto it=stores_.begin();
    it!=stores_.end();
    ++it)if(!it.value()->reload())failed<<it.key();
    for(auto*f:forms_)f->reloadValues();
    if(mapPage_)mapPage_->refreshTargets();
    if(overviewPage_)overviewPage_->refreshTargets();
    saveLabel_->setText(failed.isEmpty()?"YAML dimuat ulang dari disk ✓":"Sebagian YAML gagal dimuat: "+failed.join(", "));
  }
  void saveAll(){
    if(pending_.isEmpty())return;
    QMap<QString,QByteArray>backup;
    QSet<QString>touched;
    for(auto it=pending_.cbegin();
    it!=pending_.cend();
    ++it)touched.insert(it.key().section('|',0,0));
    for(const QString&k:touched)if(stores_.contains(k))backup[k]=stores_[k]->raw();
    QString error;
    QSet<QString>changedKeys;
    for(auto it=pending_.cbegin();
    it!=pending_.cend();
    ++it){
      QString fk=it.key().section('|',0,0),path=it.key().section('|',1);
      if(!stores_.contains(fk)||!stores_[fk]->set(path,it.value(),&error)){
        for(auto b=backup.cbegin();
        b!=backup.cend();
        ++b)stores_[b.key()]->restoreRaw(b.value());
        saveLabel_->setText("SAVE ERROR: "+error);
        return;
      }
      changedKeys.insert(it.key());
    }
    pending_.clear();
    try{
      syncVehicleAuthority();
    }
    catch(const std::exception&e){
      saveLabel_->setText("SAVED, tetapi vehicle sync error: "+QString::fromUtf8(e.what()));
      return;
    }
    for(auto*f:forms_)f->reloadValues();
    saveLabel_->setText(QString("SAVED %1 parameter ke YAML").arg(changedKeys.size()));
    if(mapPage_)mapPage_->refreshTargets();
    const bool live=stores_.contains("gui")?stores_["gui"]->get("runtime.live_apply_yaml",true).toBool():true;
    if(live)liveApplyChanges(changedKeys);
    else saveLabel_->setText(QString("SAVED %1 parameter ke YAML • live apply OFF").arg(changedKeys.size()));
  }
  static bool same(const QVariant&a,const QVariant&b){
    const double da=number(a),db=number(b);
    if(std::isfinite(da)&&std::isfinite(db))return std::abs(da-db)<1e-9;
    if(a.userType()==QMetaType::QVariantList&&b.userType()==QMetaType::QVariantList){
      const QVariantList la=a.toList();
      const QVariantList lb=b.toList();
      if(la.size()!=lb.size())return false;
      for(int i=0;
      i<la.size();
      ++i)if(!same(la.at(i),lb.at(i)))return false;
      return true;
    }
    return a==b;
  }
  void syncVehicleAuthority(){
    auto v=stores_.value("vehicle");
    if(!v)return;
    auto vg=[&](const QString&n,double d){
      return number(v->get("vehicle.ros__parameters."+n,d),d);
    };
    double physicalWb=vg("wheelbase_m",.70),wb=vg("effective_wheelbase_m",physicalWb),track=vg("track_width_m",.48),width=vg("total_width_m",.55),steer=vg("max_steering_angle_rad",0.523598775598),opSteer=vg("operational_steering_angle_rad",0.488692190558),turn=vg("minimum_turning_radius_m",1.712159378317),fwd=vg("max_forward_speed_mps",.5),rev=vg("max_reverse_speed_mps",.3),yaw=vg("max_yaw_rate_rps",0.292028888392),acc=vg("max_accel_mps2",.75),dec=vg("max_decel_mps2",-1),yacc=vg("max_yaw_accel_rps2",2.5);
    QVariant footprint=v->get("vehicle.ros__parameters.footprint");
    if(physicalWb<=0||wb<=0||track<=0||steer<=0||opSteer<=0||opSteer>steer+1e-9||turn<=0||fwd<=0||rev<0||yaw<=0||acc<=0||dec>=0||yacc<=0)throw std::runtime_error("vehicle.yaml geometry/limit invalid");
    const double kinematicYawCap=fwd/turn;
    if(yaw>kinematicYawCap+1e-9){
      yaw=kinematicYawCap;
      v->set("vehicle.ros__parameters.max_yaw_rate_rps",yaw);
    }
    auto set=[&](const QString&fk,const QString&p,const QVariant&x){
      if(stores_.contains(fk)&&!same(stores_[fk]->get(p),x))stores_[fk]->set(p,x);
    };
    set("esc","esc_ackermann.ros__parameters.wheelbase_m",wb);
    set("esc","esc_ackermann.ros__parameters.track_width_m",track);
    set("navigation_core","navigation_core.ros__parameters.wheelbase_m",wb);
    set("navigation_core","navigation_core.ros__parameters.track_width_m",track);
    set("navigation_core","navigation_core.ros__parameters.max_steering_angle_rad",opSteer);
    set("navigation_core","navigation_core.ros__parameters.minimum_turning_radius_m",turn);
    set("perception","perception.ros__parameters.wheelbase_m",wb);
    set("perception","perception.ros__parameters.lane_vehicle_width_m",width);
    set("nav2","planner_server.ros__parameters.GridBased.minimum_turning_radius",turn);
    set("nav2","controller_server.ros__parameters.FollowPath.AckermannConstraints.min_turning_r",turn);
    if(footprint.isValid()){
      set("nav2","local_costmap.local_costmap.ros__parameters.footprint",footprint);
      set("nav2","global_costmap.global_costmap.ros__parameters.footprint",footprint);
    }
    auto clampPath=[&](const QString&fk,const QString&p,double limit,bool upper){
      double cur=number(stores_[fk]->get(p,limit),limit);
      set(fk,p,upper?std::min(cur,limit):std::max(cur,limit));
    };
    clampPath("navigation_core","navigation_core.ros__parameters.max_forward_speed_mps",fwd,true);
    clampPath("navigation_core","navigation_core.ros__parameters.max_reverse_speed_mps",rev,true);
    clampPath("navigation_core","navigation_core.ros__parameters.max_yaw_rate_rps",yaw,true);
    clampPath("nav2","controller_server.ros__parameters.FollowPath.vx_max",fwd,true);
    clampPath("nav2","controller_server.ros__parameters.FollowPath.wz_max",yaw,true);
    clampPath("nav2","controller_server.ros__parameters.FollowPath.ax_max",acc,true);
    clampPath("nav2","controller_server.ros__parameters.FollowPath.ax_min",dec,false);
    double navV=number(stores_["nav2"]->get("controller_server.ros__parameters.FollowPath.vx_max",fwd),fwd);
    double navYaw=std::min(yaw,navV/turn);
    clampPath("nav2","controller_server.ros__parameters.FollowPath.wz_max",navYaw,true);
    set("nav2","velocity_smoother.ros__parameters.max_velocity",QVariantList{
      navV,0.0,navYaw
    });
    set("nav2","velocity_smoother.ros__parameters.min_velocity",QVariantList{
      0.0,0.0,-navYaw
    });
    if(stores_.contains("trajectory_safety")){
      double cur=number(stores_["trajectory_safety"]->get("trajectory_safety_supervisor.ros__parameters.maximum_yaw_rate_rps",navYaw),navYaw);
      set("trajectory_safety","trajectory_safety_supervisor.ros__parameters.maximum_yaw_rate_rps",std::min(cur,navYaw));
      set("trajectory_safety","trajectory_safety_supervisor.ros__parameters.minimum_turning_radius_m",turn);
    }
  }
  void liveApplyChanges(const QSet<QString>&changedKeys){
    // Only parameters with an explicit runtime callback are pushed live. The
    // YAML file remains authoritative for every other node and is consumed on
    // the next launch/restart. This avoids a dangerous false-positive where the
    // ROS parameter server changes but a custom node keeps using cached values.
    const QString escPrefix="esc_ackermann.ros__parameters.";
    const QSet<QString> escDynamic={
      "serial_enabled",
      "steering_feedback_calibration_enabled","steering_calibration_mode_enabled",
      "steering_calibration_direct_limit_deg","steering_feedback_center_deg",
      "steering_feedback_right_stop_deg","steering_feedback_left_stop_deg",
      "steering_feedback_center_reference_deg","steering_feedback_right_reference_deg",
      "steering_feedback_left_reference_deg","steering_physical_calibration_enabled",
      "steering_physical_left_limit_deg","steering_physical_right_limit_deg",
      "steering_physical_operational_limit_deg","steering_physical_calibration_saved_at",
      "steering_physical_lut_enabled","steering_lut_physical_deg",
      "steering_lut_command_increasing_deg","steering_lut_command_decreasing_deg",
      "steering_lut_feedback_increasing_deg","steering_lut_feedback_decreasing_deg",
      "steering_lut_direction_deadband_deg","steering_lut_calibration_saved_at",
      "steering_center_bias_from_left_deg","steering_center_bias_from_right_deg",
      "serial_left_max_deg","steering_feedback_force_symmetric_span",
      "steering_feedback_calibration_saved_at","steering_calibration_apply_token"
    };
    QVariantMap escParams;
    int restartOnly=0;
    for(const QString&key:changedKeys){
      const QString file=key.section('|',0,0),path=key.section('|',1);
      if(file=="esc"&&path.startsWith(escPrefix)&&stores_.contains("esc")){
        QString param=path.mid(escPrefix.size());
        QString yamlPath=path;
        const QString tail=param.section('.',-1);
        bool numeric=false;
        tail.toInt(&numeric);
        if(numeric){
          param=param.section('.',0,-2);
          yamlPath=escPrefix+param;
        }
        if(escDynamic.contains(param))escParams[param]=stores_["esc"]->get(yamlPath);
        else ++restartOnly;
      }
      else if(file!="gui") {
        ++restartOnly;
      }
    }
    if(!escParams.isEmpty()){
      ros_->setParametersAtomically("/esc_ackermann",escParams,"live_yaml:/esc_ackermann");
      saveLabel_->setText(QString("YAML tersimpan ✓ • live-apply ESC (%1) • %2 perubahan lain berlaku saat restart")
      .arg(escParams.size()).arg(restartOnly));
    }
    else{
      saveLabel_->setText(restartOnly>0?
      QString("YAML tersimpan ✓ • %1 perubahan berlaku saat node/launch berikutnya").arg(restartOnly):
      QString("YAML tersimpan ✓"));
    }
  }
  void verifyRuntime(){
    runtimeExpected_.clear();
    runtimeResults_.clear();
    struct Audit{
      QString node,tag;
      QStringList names;
      QString file,prefix;
    };
    QVector<Audit>a={
      {
        "/esc_ackermann","esc",{
          "serial_enabled","wheelbase_m","track_width_m","steering_physical_calibration_enabled","steering_physical_left_limit_deg","steering_physical_right_limit_deg","steering_physical_operational_limit_deg","steering_physical_lut_enabled","steering_lut_physical_deg","steering_lut_command_increasing_deg","steering_lut_command_decreasing_deg","steering_lut_feedback_increasing_deg","steering_lut_feedback_decreasing_deg"
        },"esc","esc_ackermann.ros__parameters."
      },{
        "/localization_core","localization",{
          "gnss_antenna_x_m","gnss_antenna_y_m","enable_global_gnss_velocity_fusion","enable_global_gnss_cog_fusion","gnss_yaw_rate_min_speed_mps","gnss_yaw_rate_max_abs_rps","gnss_yaw_rate_filter_alpha","anchor_init_requires_strict"
        },"localization","localization_core.ros__parameters."
      },{
        "/data_imu_node","imu",{
          "gyro_packet_timeout_sec","accel_packet_timeout_sec","require_fresh_gyro_for_imu_publish"
        },"imu","data_imu_node.ros__parameters."
      }
    };
    for(const auto&x:a){
      QVariantMap exp;
      for(const QString&n:x.names)exp[n]=stores_.value(x.file)->get(x.prefix+n);
      runtimeExpected_[x.tag]=exp;
      ros_->getParameters(x.node,x.names,"audit:"+x.tag);
    }
    saveLabel_->setText("VERIFY RUNTIME...");
  }
  void handleRuntimeAudit(const QString&tag,bool /*ok*/,const QVariantMap&v){
    if(!tag.startsWith("audit:"))return;
    QString k=tag.mid(6);
    runtimeResults_[k]=v;
    if(runtimeResults_.size()<runtimeExpected_.size())return;
    QStringList lines;
    bool all=true;
    for(auto it=runtimeExpected_.cbegin();
    it!=runtimeExpected_.cend();
    ++it){
      QVariantMap got=runtimeResults_.value(it.key());
      for(auto p=it.value().cbegin();
      p!=it.value().cend();
      ++p){
        bool eq=got.contains(p.key())&&same(p.value(),got[p.key()]);
        lines<<QString("%1.%2: YAML=%3 runtime=%4 %5").arg(it.key(),p.key(),variantText(p.value(),6),variantText(got.value(p.key()),6),eq?"✓":"MISMATCH");
        all&=eq;
      }
    }
    QMessageBox::information(this,"Config vs Runtime",lines.join('\n'));
    saveLabel_->setText(all?"CONFIG == RUNTIME ✓":"RUNTIME MISMATCH • restart/live-apply sebelum merekam tuning");
  }
  void applyStyle(){
    setStyleSheet(QStringLiteral(R"CSS(
QMainWindow,QWidget{background:#101215;color:#f5f5f5;font-family:'DejaVu Sans';font-size:12px;}
QFrame#leftPanel{background:#0b0d10;border-right:1px solid #2a2f37;}QFrame#rightPanel{background:#101215;}QFrame#brandHeader{background:#070809;border-bottom:2px solid #d8b033;}
QLabel#brandTitle{font-size:15px;font-weight:800;color:white;}QLabel#currentTabLabel,QLabel#settingsTitle,QLabel#pageTitle{font-size:15px;font-weight:800;color:white;}QLabel#pageDescription{color:#aab0ba;}
QGroupBox{border:1px solid #343a44;border-radius:7px;margin-top:10px;padding-top:10px;font-weight:700;color:#d8b033;}QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 4px;}
QPushButton{background:#252a32;border:1px solid #3b424d;border-radius:6px;padding:7px 10px;color:white;font-weight:600;}QPushButton:hover{border-color:#d8b033;}QPushButton#primaryButton{background:#d8b033;color:#101215;border:0;font-weight:800;}
QLineEdit,QSpinBox,QDoubleSpinBox,QComboBox,QPlainTextEdit,QTableWidget,QListWidget{background:#171a1f;border:1px solid #343a44;border-radius:4px;color:#f5f5f5;padding:5px;}QScrollArea{border:0;}QHeaderView::section{background:#20242b;color:#d8b033;padding:5px;border:0;}QLabel#metricCard{background:#171a1f;border:1px solid #343a44;border-radius:8px;padding:10px;font-weight:700;}
QFrame#menuPopup{background:#171a1f;border:1px solid #d8b033;border-radius:9px;}QLabel#floatingMenuTitle{font-size:18px;font-weight:800;color:white;}QLabel#floatingMenuHelp{color:#aab0ba;}QPushButton#floatingMenuButton{font-size:18px;font-weight:900;background:#d8b033;color:#101215;border:0;}QTabWidget#bab4MenuTabs::pane{border:1px solid #343a44;border-radius:6px;background:#101215;}QTabWidget#bab4MenuTabs QTabBar::tab{background:#20242b;color:#d8dbe0;padding:9px 18px;border:1px solid #343a44;min-width:120px;}QTabWidget#bab4MenuTabs QTabBar::tab:selected{background:#d8b033;color:#101215;font-weight:800;}QTreeWidget#bab4MenuTree_navigation,QTreeWidget#bab4MenuTree_perception,QTreeWidget#bab4MenuTree_steering{background:#101215;border:0;color:#f5f5f5;padding:6px;}QTreeWidget::item{padding:5px 4px;}QTreeWidget::item:selected{background:#4a4020;color:white;}
)CSS"));
  }
};
