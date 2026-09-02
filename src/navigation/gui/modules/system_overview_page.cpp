// System Overview / Vehicle System Health page.
//
// READ-ONLY monitoring. Reuses the EXISTING RosBridge telemetry channels already
// subscribed by the GUI — NO new ROS subscriber, NO new executor timer. The whole
// right pane is painted by a single low-rate QTimer (~7 Hz) reading the thread-safe
// TelemetryStore.
//
// SINGLE SOURCE OF TRUTH: health is derived from the SAME telemetry channel aliases
// the Navigation / Perception pages already read (e.g. "gnss_fix", "imu",
// "esc_odom", "ekf_local", "ekf_global", "camera_health_state", "raw_detections",
// "esc_status", "foc_telemetry", "esc_steer_actual", "system.nav2_ready", ...).
// The previous version used raw ROS topic paths as channel keys, which never match
// the aliases RosBridge emits — that is why every sensor showed OFFLINE.
//
// No Q_OBJECT / signals / slots -> stays in the implementation-partition TU without
// the AUTOMOC vtable pitfall.
#include <QDateTime>
#include <QColor>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QScrollArea>
#include <QTimer>
#include <QVBoxLayout>
#include <deque>
#include <QStringList>

class SystemOverviewPage:public QWidget{
  public:
  enum Health{Unknown,Operational,Degraded,Offline,Disabled};
  struct Mon{
    QString id,name,group,channel,commChannel,kind;
    double expectedHz=0;     // 0 => presence/state based (not rate-checked)
    bool streaming=false;    // true => continuous stream; false => state/latched
    bool critical=false;
    bool disabled=false;
    // runtime
    bool received=false;
    double lastMsgT=0;
    double lastSeenT=-1;
    bool firstRxLogged=false;
    std::deque<double>stamps;
    Health health=Unknown;
    QString reason;
    // communication
    bool commOnline=false;
    QString commText=QStringLiteral("UNKNOWN");
    // display
    QString m3label,m3value;
    QString bottom;
  };
  struct CardUI{
    QFrame*frame=nullptr;
    QLabel*accent=nullptr;
    QLabel*badge=nullptr;
    QLabel*mRate=nullptr;
    QLabel*mAge=nullptr;
    QLabel*m3=nullptr;
    QLabel*bottom=nullptr;
  };
  SystemOverviewPage(TelemetryStore*t,QWidget*p=nullptr):QWidget(p),t_(t){
    auto*root=new QVBoxLayout(this);
    root->setContentsMargins(12,12,12,12);
    root->setSpacing(10);
    auto*hdr=new QVBoxLayout();
    auto*title=new QLabel(QStringLiteral("SYSTEM OVERVIEW"));
    title->setObjectName(QStringLiteral("pageTitle"));
    hdr->addWidget(title);
    auto*sub=new QLabel(QStringLiteral("Autonomous Vehicle — System Health"));
    sub->setObjectName(QStringLiteral("pageDescription"));
    hdr->addWidget(sub);
    root->addLayout(hdr);
    // ---- Overall banner ----
    banner_=new QFrame();
    banner_->setObjectName(QStringLiteral("metricCard"));
    auto*bL=new QHBoxLayout(banner_);
    bL->setContentsMargins(14,12,14,12);
    bDot_=new QLabel(QStringLiteral("●"));
    bDot_->setObjectName(QStringLiteral("overviewDot"));
    QFont dotFont=bDot_->font();
    dotFont.setPointSize(18);
    bDot_->setFont(dotFont);
    bText_=new QLabel(QStringLiteral("UNKNOWN"));
    bText_->setObjectName(QStringLiteral("pageTitle"));
    auto*vb=new QVBoxLayout();
    vb->setSpacing(4);
    bReady_=new QLabel(QStringLiteral("0 / 0 Required Subsystems Ready"));
    bReady_->setObjectName(QStringLiteral("pageDescription"));
    bCounts_=new QLabel(QStringLiteral("● 0 Operational   ● 0 Degraded   ● 0 Offline"));
    bCounts_->setObjectName(QStringLiteral("pageDescription"));
    vb->addWidget(bReady_);
    vb->addWidget(bCounts_);
    bL->addWidget(bDot_);
    bL->addWidget(bText_);
    bL->addLayout(vb,1);
    root->addWidget(banner_);
    // ---- Scroll area with grouped cards ----
    auto*scroll=new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    auto*body=new QWidget();
    bodyL_=new QVBoxLayout(body);
    bodyL_->setSpacing(14);
    bodyL_->setContentsMargins(0,0,0,0);
    scroll->setWidget(body);
    root->addWidget(scroll,1);

    registerMonitors();
    buildCards();

    timer_=new QTimer(this);
    timer_->setInterval(150);
    connect(timer_,&QTimer::timeout,this,[this](){refresh();});
    timer_->start();
    shownAt_=QDateTime::currentMSecsSinceEpoch()/1000.0;
  }
  void refreshTargets(){
    shownAt_=QDateTime::currentMSecsSinceEpoch()/1000.0;
    for(auto&m:mon_){
      m.stamps.clear();
      m.lastSeenT=-1.0;
      m.received=false;
      m.lastMsgT=0.0;
      m.health=Unknown;
    }
  }
  private:
  TelemetryStore*t_;
  QTimer*timer_=nullptr;
  double shownAt_=0.0;
  QVector<Mon>mon_;
  QVBoxLayout*bodyL_=nullptr;
  QFrame*banner_=nullptr;
  QLabel*bDot_,*bText_,*bReady_,*bCounts_;
  QMap<QString,CardUI>cardUI_;
  static constexpr double kGraceSec=4.0;

  static QString healthText(Health h){
    return h==Operational?QStringLiteral("OPERATIONAL"):h==Degraded?QStringLiteral("DEGRADED"):h==Offline?QStringLiteral("OFFLINE"):h==Disabled?QStringLiteral("DISABLED"):QStringLiteral("UNKNOWN");
  }
  static QString healthColor(Health h){
    return h==Operational?kGreen:h==Degraded?kOrange:h==Offline?kRed:h==Disabled?kMuted:kMuted;
  }
  static QString fixName(int ft){
    // u-blox fix types
    switch(ft){
      case 0:return QStringLiteral("NO FIX");
      case 1:return QStringLiteral("DR");
      case 2:return QStringLiteral("2D");
      case 3:return QStringLiteral("3D");
      case 4:return QStringLiteral("GPS+DR");
      case 5:return QStringLiteral("TIME");
      case 6:return QStringLiteral("FLOAT");
      case 7:return QStringLiteral("RTK FIX");
      default:return QStringLiteral("TYPE %1").arg(ft);
    }
  }
  void registerMonitors(){
    auto add=[this](const QString&id,const QString&name,const QString&group,const QString&channel,const QString&commChannel,const QString&kind,double expectedHz,bool critical,bool disabled=false){
      Mon m;
      m.id=id;
      m.name=name;
      m.group=group;
      m.channel=channel;
      m.commChannel=commChannel;
      m.kind=kind;
      m.expectedHz=expectedHz;
      m.streaming=expectedHz>0;
      m.critical=critical;
      m.disabled=disabled;
      mon_.append(m);
    };
    // NAVIGATION & LOCALIZATION
    add(QStringLiteral("gnss"),QStringLiteral("GNSS"),QStringLiteral("NAVIGATION & LOCALIZATION"),QStringLiteral("gnss_fix"),QString(),QStringLiteral("gnss"),10.0,true);
    add(QStringLiteral("imu"),QStringLiteral("IMU"),QStringLiteral("NAVIGATION & LOCALIZATION"),QStringLiteral("imu"),QString(),QStringLiteral("imu"),5.0,true);
    add(QStringLiteral("wheel_odom"),QStringLiteral("Wheel Odometry / Encoder"),QStringLiteral("NAVIGATION & LOCALIZATION"),QStringLiteral("esc_odom"),QString(),QStringLiteral("wheel_odom"),50.0,false);
    add(QStringLiteral("ekf_local"),QStringLiteral("EKF Local"),QStringLiteral("NAVIGATION & LOCALIZATION"),QStringLiteral("ekf_local"),QString(),QStringLiteral("ekf"),20.0,true);
    add(QStringLiteral("ekf_global"),QStringLiteral("EKF Global"),QStringLiteral("NAVIGATION & LOCALIZATION"),QStringLiteral("ekf_global"),QString(),QStringLiteral("ekf"),10.0,false);
    // PERCEPTION
    add(QStringLiteral("rgb_cam"),QStringLiteral("RGB Camera"),QStringLiteral("PERCEPTION"),QStringLiteral("camera_health_state"),QStringLiteral("connected.camera"),QStringLiteral("rgb_cam"),0.0,false);
    add(QStringLiteral("depth_cam"),QStringLiteral("Depth Camera"),QStringLiteral("PERCEPTION"),QString(),QString(),QStringLiteral("depth_cam"),0.0,false,true);
    add(QStringLiteral("yolo"),QStringLiteral("Perception / YOLO"),QStringLiteral("PERCEPTION"),QStringLiteral("raw_detections"),QString(),QStringLiteral("yolo"),0.0,false);
    // DRIVE SYSTEM
    add(QStringLiteral("esc"),QStringLiteral("ESC"),QStringLiteral("DRIVE SYSTEM"),QStringLiteral("esc_status"),QStringLiteral("connected.esc_ready"),QStringLiteral("esc"),2.0,true);
    add(QStringLiteral("steering"),QStringLiteral("Steering"),QStringLiteral("DRIVE SYSTEM"),QStringLiteral("esc_steer_actual"),QString(),QStringLiteral("steering"),50.0,false);
    add(QStringLiteral("motor_enc"),QStringLiteral("Motor Encoder"),QStringLiteral("DRIVE SYSTEM"),QStringLiteral("foc_telemetry"),QString(),QStringLiteral("motor_enc"),50.0,false);
    // ROS / PLATFORM
    add(QStringLiteral("nav2"),QStringLiteral("Nav2"),QStringLiteral("ROS / PLATFORM"),QStringLiteral("system.nav2_ready"),QStringLiteral("system.nav2_ready"),QStringLiteral("nav2"),0.0,false);
    add(QStringLiteral("tf"),QStringLiteral("TF"),QStringLiteral("ROS / PLATFORM"),QString(),QString(),QStringLiteral("tf"),0.0,false);
    add(QStringLiteral("roscomm"),QStringLiteral("ROS 2 Communication"),QStringLiteral("ROS / PLATFORM"),QString(),QString(),QStringLiteral("roscomm"),0.0,true);
  }
  void buildCards(){
    QString curGroup;
    QGridLayout*group=nullptr;
    int col=0;
    const int columns=3;
    for(auto&m:mon_){
      if(m.group!=curGroup){
        curGroup=m.group;
        auto*g=new QLabel(m.group);
        g->setObjectName(QStringLiteral("settingsTitle"));
        g->setStyleSheet(QStringLiteral("color:#d8b033;font-weight:800;font-size:13px;"));
        bodyL_->addWidget(g);
        auto*sep=new QFrame();
        sep->setFrameShape(QFrame::HLine);
        sep->setFrameShadow(QFrame::Plain);
        sep->setStyleSheet(QStringLiteral("color:#343a44;background:#343a44;max-height:1px;"));
        bodyL_->addWidget(sep);
        group=new QGridLayout();
        group->setSpacing(10);
        group->setContentsMargins(0,0,0,0);
        auto*wrap=new QWidget();
        wrap->setLayout(group);
        bodyL_->addWidget(wrap);
        col=0;
      }
      CardUI ui;
      auto*f=new QFrame();
      f->setObjectName(QStringLiteral("metricCard"));
      f->setMinimumWidth(250);
      f->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Preferred);
      auto*hl=new QHBoxLayout(f);
      hl->setContentsMargins(0,0,0,0);
      hl->setSpacing(0);
      auto*accent=new QLabel();
      accent->setFixedWidth(4);
      accent->setStyleSheet(QStringLiteral("background:%1;").arg(kMuted));
      hl->addWidget(accent);
      auto*cl=new QVBoxLayout();
      cl->setContentsMargins(10,8,10,8);
      cl->setSpacing(4);
      auto*top=new QHBoxLayout();
      auto*name=new QLabel(m.name);
      name->setStyleSheet(QStringLiteral("font-weight:800;color:white;font-size:12px;"));
      auto*badge=new QLabel(QStringLiteral("UNKNOWN"));
      badge->setStyleSheet(QStringLiteral("font-weight:800;font-size:11px;color:%1;").arg(kMuted));
      top->addWidget(name,1);
      top->addWidget(badge);
      auto*topic=new QLabel(m.channel.isEmpty()?QStringLiteral("-"):QStringLiteral("/")+m.channel);
      topic->setStyleSheet(QStringLiteral("color:#aab0ba;font-size:10px;"));
      auto*metrics=new QHBoxLayout();
      metrics->setSpacing(12);
      auto*rate=new QLabel(QStringLiteral("RATE\n--"));
      auto*age=new QLabel(QStringLiteral("AGE\n--"));
      auto*m3=new QLabel(QStringLiteral("--"));
      for(auto*w:{rate,age,m3}){
        w->setStyleSheet(QStringLiteral("color:#aab0ba;font-size:10px;font-weight:600;"));
      }
      metrics->addWidget(rate);
      metrics->addWidget(age);
      metrics->addWidget(m3,1);
      auto*bottom=new QLabel(QStringLiteral("-"));
      bottom->setStyleSheet(QStringLiteral("color:#aab0ba;font-size:10px;"));
      bottom->setWordWrap(true);
      cl->addLayout(top);
      cl->addWidget(topic);
      cl->addLayout(metrics);
      cl->addWidget(bottom);
      hl->addLayout(cl,1);
      group->addWidget(f,col/columns,col%columns);
      col++;
      ui.frame=f;
      ui.accent=accent;
      ui.badge=badge;
      ui.mRate=rate;
      ui.mAge=age;
      ui.m3=m3;
      ui.bottom=bottom;
      cardUI_[m.id]=ui;
    }
    bodyL_->addStretch();
  }
  double nowSec()const{
    return QDateTime::currentMSecsSinceEpoch()/1000.0;
  }
  double actualRate(Mon&m)const{
    if(m.stamps.size()<2)return std::numeric_limits<double>::quiet_NaN();
    return (m.stamps.size()-1)/(m.stamps.back()-m.stamps.front());
  }
  void sampleChannel(Mon&m,double now){
    if(m.channel.isEmpty())return;
    double ts=t_->timestamp(m.channel);
    if(ts>0&&ts>m.lastSeenT){
      if(!m.firstRxLogged){
        m.firstRxLogged=true;
        qDebug().nospace()<<"[OVERVIEW] RX "<<m.id<<" (ch="<<m.channel<<")";
      }
      m.received=true;
      m.lastMsgT=now;
      m.stamps.push_back(now);
      if(m.stamps.size()>30)m.stamps.pop_front();
      m.lastSeenT=ts;
    }
  }
  void computeHealth(Mon&m,double now){
    if(m.disabled){
      m.health=Disabled;
      m.reason=QStringLiteral("Disabled by configuration");
      m.commOnline=false;
      m.commText=QStringLiteral("DISABLED");
      return;
    }
    if(!m.received){
      if(now-shownAt_<kGraceSec){
        m.health=Unknown;
        m.reason=QStringLiteral("Waiting for data");
      }else{
        m.health=Offline;
        m.reason=QStringLiteral("No telemetry received");
      }
      m.commOnline=false;
      m.commText=QStringLiteral("OFFLINE");
      return;
    }
    double ageMs=(now-m.lastMsgT)*1000.0;
    m.commOnline=true;
    m.commText=QStringLiteral("ONLINE");
    if(m.streaming){
      double timeoutMs=m.expectedHz>=40?1500:(m.expectedHz>=15?2500:3500);
      double freshMs=m.expectedHz>=40?700:(m.expectedHz>=15?1200:2000);
      if(ageMs>timeoutMs){
        m.health=Offline;
        m.reason=QStringLiteral("Telemetry timeout");
        return;
      }
      double rate=actualRate(m);
      bool rateOk=std::isnan(rate)||rate>=0.6*m.expectedHz;
      if(ageMs>freshMs||!rateOk){
        m.health=Degraded;
        m.reason=!rateOk?QStringLiteral("Low message rate"):QStringLiteral("Data slightly stale");
        return;
      }
      m.health=Operational;
      m.reason=QStringLiteral("");
    }else{
      // presence/state based: default operational; value-specific downgrade in postQuality
      m.health=Operational;
      m.reason=QStringLiteral("");
    }
  }
  void postQuality(Mon&m){
    if(m.kind==QStringLiteral("gnss")){
      int ft=t_->get(QStringLiteral("gnss_quality.fix_type"),0).toInt();
      m.m3label=QStringLiteral("FIX");
      m.m3value=fixName(ft);
      bool fixOk=t_->get(QStringLiteral("gnss_quality.gnss_fix_ok"),false).toBool();
      if(ft<3){
        m.health=Degraded;
        m.reason=QStringLiteral("No position fix");
      }else if(ft==6||!fixOk){
        m.health=Degraded;
        m.reason=QStringLiteral("Position quality below nominal (FLOAT)");
      }
    }else if(m.kind==QStringLiteral("imu")){
      m.m3label=QStringLiteral("STATUS");
      m.m3value=QStringLiteral("OK");
    }else if(m.kind==QStringLiteral("wheel_odom")){
      m.m3label=QStringLiteral("SPEED");
      m.m3value=QStringLiteral("%1 m/s").arg(number(t_->get(QStringLiteral("esc_odom.v"))),0,'f',2);
    }else if(m.kind==QStringLiteral("ekf")){
      m.m3label=QStringLiteral("POSE");
      m.m3value=QStringLiteral("x %1").arg(number(t_->get(m.channel+".x")),0,'f',1);
    }else if(m.kind==QStringLiteral("rgb_cam")){
      bool healthy=t_->get(QStringLiteral("camera_healthy"),false).toBool();
      m.m3label=QStringLiteral("LINK");
      m.m3value=m.commText;
      if(m.received&&!healthy){
        m.health=Degraded;
        m.reason=QStringLiteral("Camera stream up but not healthy");
      }
    }else if(m.kind==QStringLiteral("yolo")){
      int c=t_->get(QStringLiteral("raw_detections.count"),-1).toInt();
      m.m3label=QStringLiteral("DETS");
      m.m3value=c>=0?QString::number(c):QStringLiteral("-");
    }else if(m.kind==QStringLiteral("esc")){
      bool ready=t_->get(QStringLiteral("connected.esc_ready"),false).toBool();
      bool armed=t_->get(QStringLiteral("connected.esc_armed"),false).toBool();
      m.m3label=QStringLiteral("STATE");
      m.m3value=ready?(armed?QStringLiteral("READY+ARMED"):QStringLiteral("READY")):QStringLiteral("STANDBY");
      if(m.received&&!ready)m.health=Degraded,m.reason=QStringLiteral("ESC not ready");
    }else if(m.kind==QStringLiteral("steering")){
      double deg=number(t_->get(QStringLiteral("esc_steer_actual")))*180.0/kPi;
      m.m3label=QStringLiteral("ANGLE");
      m.m3value=QStringLiteral("%1°").arg(deg,0,'f',1);
    }else if(m.kind==QStringLiteral("motor_enc")){
      m.m3label=QStringLiteral("IQ");
      m.m3value=QStringLiteral("%1 A").arg(number(t_->get(QStringLiteral("foc_telemetry.iq_a"))),0,'f',2);
    }else if(m.kind==QStringLiteral("nav2")){
      bool ready=t_->get(QStringLiteral("system.nav2_ready"),false).toBool();
      m.m3label=QStringLiteral("MODE");
      m.m3value=ready?QStringLiteral("READY"):QStringLiteral("STANDBY");
      if(m.received&&!ready){
        m.health=Degraded;
        m.reason=QStringLiteral("Nav2 not ready");
      }
    }else if(m.kind==QStringLiteral("tf")){
      m.m3label=QStringLiteral("CHAIN");
      m.m3value=QStringLiteral("map→odom");
    }else if(m.kind==QStringLiteral("roscomm")){
      m.m3label=QStringLiteral("BRIDGE");
      m.m3value=QStringLiteral("agv_gui");
    }
  }
  void refresh(){
    double now=nowSec();
    for(auto&m:mon_){
      if(m.kind==QStringLiteral("tf")||m.kind==QStringLiteral("roscomm"))continue;
      sampleChannel(m,now);
      computeHealth(m,now);
      postQuality(m);
    }
    // derived: TF
    for(auto&m:mon_)if(m.kind==QStringLiteral("tf")){
      Health a=healthOf(QStringLiteral("ekf_local")),b=healthOf(QStringLiteral("ekf_global"));
      m.commOnline=a!=Offline&&b!=Offline;
      m.commText=m.commOnline?QStringLiteral("ONLINE"):QStringLiteral("OFFLINE");
      if(a==Offline||b==Offline){m.health=Offline;m.reason=QStringLiteral("EKF chain stale");}
      else if(a==Unknown||b==Unknown){m.health=Unknown;m.reason=QStringLiteral("Waiting for data");}
      else{m.health=Operational;m.reason=QStringLiteral("");}
    }
    // derived: ROS Communication
    for(auto&m:mon_)if(m.kind==QStringLiteral("roscomm")){
      bool anyAlive=false;
      for(const auto&o:mon_){
        if(o.disabled||o.channel.isEmpty()||o.kind==QStringLiteral("tf")||o.kind==QStringLiteral("roscomm"))continue;
        if(o.received&&(now-o.lastMsgT)<2.0){anyAlive=true;break;}
      }
      m.commOnline=anyAlive;
      m.commText=anyAlive?QStringLiteral("ONLINE"):QStringLiteral("OFFLINE");
      m.health=anyAlive?Operational:Offline;
      m.reason=anyAlive?QStringLiteral(""):QStringLiteral("No telemetry");
    }
    // paint cards
    int nOp=0,nDeg=0,nOff=0,nUnk=0,nReq=0,nReqReady=0;
    for(auto&m:mon_){
      QString col=healthColor(m.health);
      if(m.health==Operational)++nOp;
      else if(m.health==Degraded)++nDeg;
      else if(m.health==Offline)++nOff;
      else ++nUnk;
      if(!m.disabled){
        ++nReq;
        if(m.health==Operational||m.health==Degraded)++nReqReady;
      }
      auto it=cardUI_.find(m.id);
      if(it==cardUI_.end())continue;
      CardUI&ui=it.value();
      ui.accent->setStyleSheet(QStringLiteral("background:%1;").arg(col));
      ui.badge->setText(healthText(m.health));
      ui.badge->setStyleSheet(QStringLiteral("font-weight:800;font-size:11px;color:%1;").arg(col));
      ui.frame->setStyleSheet(QStringLiteral("border:1px solid #343a44;border-radius:8px;background:#171a1f;padding:0;"));
      double ageMs=(now-m.lastMsgT)*1000.0;
      QString rateStr=m.streaming?QStringLiteral("%1 Hz").arg(std::isfinite(actualRate(m))?QString::number(actualRate(m),'f',1):QStringLiteral("-")):QStringLiteral("--");
      QString ageStr=m.received?(m.streaming?QString::number(ageMs,'f',0)+" ms":QStringLiteral("recv")):QStringLiteral("--");
      ui.mRate->setText(QStringLiteral("RATE\n")+rateStr);
      ui.mAge->setText(QStringLiteral("AGE\n")+ageStr);
      ui.m3->setText(m.m3label+QStringLiteral("\n")+m.m3value);
      QString b=m.reason.isEmpty()?QStringLiteral("-"):m.reason;
      if(m.disabled)b=QStringLiteral("Disabled by configuration");
      ui.bottom->setText(b);
    }
    // overall banner
    bool critOff=false;
    for(const auto&m:mon_)if(m.critical&&m.health==Offline)critOff=true;
    Health overall;
    QString oReason;
    if(critOff){
      overall=Offline;
      oReason=QStringLiteral("Critical subsystem offline");
    }else if(nOff>0||nDeg>0){
      overall=Degraded;
      oReason=QStringLiteral("Subsystem degraded / offline");
    }else if(nOp>0){
      overall=Operational;
      oReason=QStringLiteral("");
    }else{
      overall=Unknown;
      oReason=QStringLiteral("No data yet");
    }
    QString oc=healthColor(overall);
    bDot_->setStyleSheet(QStringLiteral("color:%1;").arg(oc));
    bText_->setText(healthText(overall)+(oReason.isEmpty()?QStringLiteral(""):QStringLiteral("  —  ")+oReason));
    bText_->setStyleSheet(QStringLiteral("font-size:15px;font-weight:800;color:%1;").arg(oc));
    bReady_->setText(QStringLiteral("%1 / %2 Required Subsystems Ready").arg(nReqReady).arg(nReq));
    bCounts_->setText(QStringLiteral("● %1 Operational   ● %2 Degraded   ● %3 Offline").arg(nOp).arg(nDeg).arg(nOff));
    bCounts_->setStyleSheet(QStringLiteral("color:#aab0ba;"));
  }
  Health healthOf(const QString&id)const{
    for(const auto&m:mon_)if(m.id==id)return m.health;
    return Unknown;
  }
};
