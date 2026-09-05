#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include <functional>
#include <memory>
#include <tuple>
#include <signal.h>
#include <sys/types.h>
#include <QAbstractItemView>
#include <QByteArray>
#include <QChar>
#include <QTreeWidget>
#include <QFont>
#include <QIODevice>
#include <QLineF>
#include <QListWidgetItem>
#include <QObject>
#include <QPaintEvent>
#include <QPair>
#include <QPoint>
#include <QRect>
#include <QRectF>
#include <QString>
#include <QVector>
#include <QApplication>
#include <QGraphicsSceneMouseEvent>
#include <QInputDialog>
#include <QListWidget>
#include <QMetaType>
#include <QMutex>
#include <QMutexLocker>
#include <QNetworkRequest>
#include <QRegularExpression>
#include <QResizeEvent>
#include <QCheckBox>
#include <QCloseEvent>
#include <QColor>
#include <QComboBox>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDirIterator>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QGraphicsEllipseItem>
#include <QGraphicsItem>
#include <QGraphicsLineItem>
#include <QGraphicsPathItem>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsView>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QHash>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMap>
#include <QMessageBox>
#include <QMetaObject>
#include <QMouseEvent>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPlainTextEdit>
#include <QPointF>
#include <QPixmap>
#include <QProcess>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollArea>
#include <QSet>
#include <QScreen>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSpinBox>
#include <QSplitter>
#include <QStackedWidget>
#include <QStandardPaths>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTabWidget>
#include <QTreeWidgetItemIterator>
#include <QTextStream>
#include <QTimer>
#include <QToolButton>
#include <QTransform>
#include <QUrl>
#include <QUrlQuery>
#include <QVariant>
#include <QVariantList>
#include <QVariantMap>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QWidget>
#include <QXmlStreamReader>
#include <yaml-cpp/yaml.h>
#include <action_msgs/srv/cancel_goal.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yolo_obstacle_detection_ros2/msg/obstacle_array.hpp>
#include <yolo_obstacle_detection_ros2/msg/alignment_state.hpp>
#include "agv_gui_specs.hpp"
#include "agv_experiment_catalog.hpp"
namespace fs = std::filesystem;
using namespace std::chrono_literals;
namespace {
  #include "modules/gui_core.cpp"
  class SettingsForm:public QWidget{
    Q_OBJECT
    public:
    SettingsForm(const QString &title,const QVector<SettingSpec>&specs,const QMap<QString,std::shared_ptr<YamlStore>>&stores,QWidget*parent=nullptr):QWidget(parent),specs_(specs),stores_(stores){
      auto *lay=new QVBoxLayout(this);
      lay->setContentsMargins(10,10,10,10);
      auto *h=new QLabel(title);
      h->setObjectName("settingsTitle");
      lay->addWidget(h);
      QString current;
      QFormLayout *form=nullptr;
      for(const auto&s:specs_){
        if(s.group!=current){
          current=s.group;
          auto*g=new QGroupBox(current);
          form=new QFormLayout(g);
          form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
          lay->addWidget(g);
        }
        QWidget*w=createWidget(s);
        widgets_[s.fileKey+"|"+s.path]=w;
        form->addRow(s.label+s.suffix,w);
      }
      lay->addStretch();
      reloadValues();
    }
    QSet<QString> settingKeys()const{
      QSet<QString>s;
      for(const auto&x:specs_)s.insert(x.fileKey+"|"+x.path);
      return s;
    }
    void reloadValues(){
      for(const auto&s:specs_)loadOne(s);
    }
    void reloadKeys(const QSet<QString>&keys){
      for(const auto&s:specs_)if(keys.contains(s.fileKey+"|"+s.path))loadOne(s);
    }
    signals:void valueEdited(QString fileKey,QString path,QVariant value);
    private:
    QVector<SettingSpec> specs_;
    QMap<QString,std::shared_ptr<YamlStore>> stores_;
    QHash<QString,QWidget*>widgets_;
    QWidget*createWidget(const SettingSpec&s){
      QWidget*w=nullptr;
      if(s.kind=="bool"){
        auto*c=new QCheckBox();
        connect(c,&QCheckBox::toggled,this,[this,s](bool v){
          emit valueEdited(s.fileKey,s.path,v);
        });
        w=c;
      }
      else if(s.kind=="int"){
        auto*x=new NoWheelSpinBox();
        x->setRange(static_cast<int>(std::max(-2147483647.0,s.min)),static_cast<int>(std::min(2147483647.0,s.max)));
        x->setSingleStep(std::max(1,static_cast<int>(s.step)));
        connect(x,qOverload<int>(&QSpinBox::valueChanged),this,[this,s](int v){
          emit valueEdited(s.fileKey,s.path,v);
        });
        w=x;
      }
      else if(s.kind=="choice"){
        auto*x=new NoWheelComboBox();
        x->addItems(s.choices);
        connect(x,&QComboBox::currentTextChanged,this,[this,s](const QString&v){
          emit valueEdited(s.fileKey,s.path,v);
        });
        w=x;
      }
      else if(s.kind=="float"){
        auto*x=new NoWheelDoubleSpinBox();
        x->setDecimals(s.decimals);
        x->setRange(s.min,s.max);
        x->setSingleStep(s.step);
        connect(x,qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this,s](double v){
          emit valueEdited(s.fileKey,s.path,v);
        });
        w=x;
      }
      else {
        auto*x=new QLineEdit();
        connect(x,&QLineEdit::editingFinished,this,[this,s,x](){
          emit valueEdited(s.fileKey,s.path,s.kind=="list"?parseEditorList(x->text()):QVariant(x->text()));
        });
        w=x;
      }
      w->setToolTip(s.tip);
      return w;
    }
    void loadOne(const SettingSpec&s){
      auto it=widgets_.find(s.fileKey+"|"+s.path);
      if(it==widgets_.end()||!stores_.contains(s.fileKey))return;
      const QVariant v=stores_[s.fileKey]->get(s.path);
      QSignalBlocker b(it.value());
      if(auto*x=qobject_cast<QCheckBox*>(it.value()))x->setChecked(v.toBool());
      else if(auto*x=qobject_cast<QSpinBox*>(it.value()))x->setValue(v.toInt());
      else if(auto*x=qobject_cast<QDoubleSpinBox*>(it.value()))x->setValue(v.toDouble());
      else if(auto*x=qobject_cast<QComboBox*>(it.value())){
        int i=x->findText(v.toString());
        if(i>=0)x->setCurrentIndex(i);
        else{
          x->addItem(v.toString());
          x->setCurrentText(v.toString());
        }
      }
      else if(auto*x=qobject_cast<QLineEdit*>(it.value())){
        if(v.userType()==QMetaType::QVariantList)x->setText(QString::fromUtf8(QJsonDocument(QJsonArray::fromVariantList(v.toList())).toJson(QJsonDocument::Compact)));
        else x->setText(v.toString());
      }
    }
  };
  #include "modules/reporting_widgets.cpp"
  class RosBridge:public QObject{
    Q_OBJECT
    public:explicit RosBridge(QObject*p=nullptr):QObject(p){
    }
    ~RosBridge()override{
      shutdown();
    }
    void start(){
      if(thread_.joinable())return;
      stop_=false;
      thread_=std::thread([this](){
        run();
      });
    }
    void shutdown(){
      stop_=true;
      if(executor_)executor_->cancel();
      if(thread_.joinable())thread_.join();
      std::lock_guard<std::mutex> lock(subscriptionsMutex_);
      subscriptions_.clear();
    }
    bool publishGoal(double x,double y,double yaw){
      std::lock_guard<std::mutex>lk(mu_);
      if(!node_||!goalPub_)return false;
      geometry_msgs::msg::PoseStamped m;
      m.header.stamp=node_->now();
      m.header.frame_id="map";
      m.pose.position.x=x;
      m.pose.position.y=y;
      m.pose.orientation.z=std::sin(yaw/2.0);
      m.pose.orientation.w=std::cos(yaw/2.0);
      goalPub_->publish(m);
      {
        std::lock_guard<std::mutex>goalLock(goalMutex_);
        goalStarted_=std::chrono::steady_clock::now();
        goalActive_=true;
        firstPlanSeen_=false;
        lastPlanningLatencyMs_=std::numeric_limits<double>::quiet_NaN();
      }
      emitMap("goal_pose",{
        {
          "x",x
        },{
          "y",y
        },{
          "yaw",yaw
        }
      });
      return true;
    }
    bool publishGroundTruth(double x,double y,double yaw){
      std::lock_guard<std::mutex>lk(mu_);
      if(!node_||!initialPub_)return false;
      geometry_msgs::msg::PoseWithCovarianceStamped m;
      m.header.stamp=node_->now();
      m.header.frame_id="map";
      m.pose.pose.position.x=x;
      m.pose.pose.position.y=y;
      m.pose.pose.orientation.z=std::sin(yaw/2.0);
      m.pose.pose.orientation.w=std::cos(yaw/2.0);
      m.pose.covariance[0]=0.01;
      m.pose.covariance[7]=0.01;
      m.pose.covariance[35]=std::pow(kPi/180.0,2);
      initialPub_->publish(m);
      return true;
    }
    void callTrigger(const QString&service,const QString&tag={
    }){
      auto n=nodeCopy();
      if(!n){
        emit serviceResult(tag.isEmpty()?service:tag,false,"ROS node belum aktif");
        return;
      }
      auto c=n->create_client<std_srvs::srv::Trigger>(service.toStdString());
      if(!c->wait_for_service(300ms)){
        emit serviceResult(tag.isEmpty()?service:tag,false,"Service belum tersedia");
        return;
      }
      auto req=std::make_shared<std_srvs::srv::Trigger::Request>();
      c->async_send_request(req,[this,c,service,tag](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f){
        try{
          auto r=f.get();
          emit serviceResult(tag.isEmpty()?service:tag,r->success,QString::fromStdString(r->message));
        }
        catch(const std::exception&e){
          emit serviceResult(tag.isEmpty()?service:tag,false,e.what());
        }
      });
    }
    void setParametersAtomically(const QString&nodeName,const QVariantMap&params,const QString&tag){
      auto n=nodeCopy();
      if(!n){
        emit serviceResult(tag,false,"ROS node belum aktif");
        return;
      }
      const QString service="/"+nodeName.trimmed().remove(QRegularExpression("^/"))+"/set_parameters_atomically";
      auto c=n->create_client<rcl_interfaces::srv::SetParametersAtomically>(service.toStdString());
      if(!c->wait_for_service(600ms)){
        emit serviceResult(tag,false,"Parameter service belum tersedia");
        return;
      }
      auto req=std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
      for(auto it=params.cbegin();
      it!=params.cend();
      ++it){
        rcl_interfaces::msg::Parameter p;
        p.name=it.key().toStdString();
        auto&v=p.value;
        const QVariant x=it.value();
        if(x.userType()==QMetaType::Bool){
          v.type=rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
          v.bool_value=x.toBool();
        }
        else if(x.userType()==QMetaType::Int||x.userType()==QMetaType::LongLong){
          v.type=rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
          v.integer_value=x.toLongLong();
        }
        else if(x.type()==QVariant::List){
          const QVariantList list=x.toList();
          bool numeric=true;
          for(const QVariant&item:list){
            if(!item.canConvert<double>()||item.userType()==QMetaType::QString){
              numeric=false;
              break;
            }
          }
          if(numeric){
            v.type=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
            for(const QVariant&item:list)v.double_array_value.push_back(item.toDouble());
          }
          else{
            v.type=rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
            for(const QVariant&item:list)v.string_array_value.push_back(item.toString().toStdString());
          }
        }
        else if(x.canConvert<double>()&&x.userType()!=QMetaType::QString){
          v.type=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
          v.double_value=x.toDouble();
        }
        else{
          v.type=rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
          v.string_value=x.toString().toStdString();
        }
        req->parameters.push_back(p);
      }
      c->async_send_request(req,[this,c,tag](rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture f){
        try{
          auto r=f.get();
          emit serviceResult(tag,r->result.successful,QString::fromStdString(r->result.reason));
        }
        catch(const std::exception&e){
          emit serviceResult(tag,false,e.what());
        }
      });
    }
    void getParameters(const QString&nodeName,const QStringList&names,const QString&tag){
      auto n=nodeCopy();
      if(!n){
        emit parametersResult(tag,false,QVariantMap{
          {
            "error","ROS node belum aktif"
          }
        });
        return;
      }
      const QString service="/"+QString(nodeName).remove(QRegularExpression("^/"))+"/get_parameters";
      auto c=n->create_client<rcl_interfaces::srv::GetParameters>(service.toStdString());
      if(!c->wait_for_service(600ms)){
        emit parametersResult(tag,false,QVariantMap{
          {
            "error","Parameter service belum tersedia"
          }
        });
        return;
      }
      auto req=std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
      for(const auto&s:names)req->names.push_back(s.toStdString());
      c->async_send_request(req,[this,c,tag,names](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture f){
        try{
          auto r=f.get();
          QVariantMap m;
          for(int i=0;
          i<std::min<int>(names.size(),r->values.size());
          ++i)m[names[i]]=parameterValue(r->values[i]);
          emit parametersResult(tag,true,m);
        }
        catch(const std::exception&e){
          emit parametersResult(tag,false,QVariantMap{
            {
              "error",e.what()
            }
          });
        }
      });
    }
    void cancelNavigation(){
      auto n=nodeCopy();
      if(!n){
        emit serviceResult("Cancel navigation",false,"ROS node belum aktif");
        return;
      }
      auto c=n->create_client<action_msgs::srv::CancelGoal>("/navigate_to_pose/_action/cancel_goal");
      if(!c->wait_for_service(400ms)){
        emit serviceResult("Cancel navigation",false,"Service cancel Nav2 belum tersedia");
        return;
      }
      auto req=std::make_shared<action_msgs::srv::CancelGoal::Request>();
      c->async_send_request(req,[this,c](rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture f){
        try{
          auto r=f.get();
          emit serviceResult("Cancel navigation",r->return_code==0||!r->goals_canceling.empty(),QString("return_code=%1, goals=%2").arg(r->return_code).arg(r->goals_canceling.size()));
        }
        catch(const std::exception&e){
          emit serviceResult("Cancel navigation",false,e.what());
        }
      });
    }
    signals:void telemetry(QString channel,QVariant data);
    void image(QImage image);
    void ready(bool ok,QString message);
    void serviceResult(QString tag,bool ok,QString message);
    void parametersResult(QString tag,bool ok,QVariantMap values);
    private:std::thread thread_;
    std::atomic_bool stop_{
      false
    };
    std::mutex mu_;
    std::mutex subscriptionsMutex_;
    std::mutex goalMutex_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>executor_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialPub_;
    std::chrono::steady_clock::time_point goalStarted_{
    };
    bool goalActive_{
      false
    };
    bool firstPlanSeen_{
      false
    };
    double lastPlanningLatencyMs_{
      std::numeric_limits<double>::quiet_NaN()
    };
    std::atomic<long long> lastCameraMetricNs_{0};
    rclcpp::Node::SharedPtr nodeCopy(){
      std::lock_guard<std::mutex>lk(mu_);
      return node_;
    }
    static QVariant parameterValue(const rcl_interfaces::msg::ParameterValue&v){
      using PT=rcl_interfaces::msg::ParameterType;
      switch(v.type){
        case PT::PARAMETER_BOOL:return v.bool_value;
        case PT::PARAMETER_INTEGER:return QVariant::fromValue<qlonglong>(v.integer_value);
        case PT::PARAMETER_DOUBLE:return v.double_value;
        case PT::PARAMETER_STRING:return QString::fromStdString(v.string_value);
        case PT::PARAMETER_DOUBLE_ARRAY:{
          QVariantList l;
          for(double x:v.double_array_value)l<<x;
          return l;
        }
        case PT::PARAMETER_INTEGER_ARRAY:{
          QVariantList l;
          for(auto x:v.integer_array_value)l<<QVariant::fromValue<qlonglong>(x);
          return l;
        }
        case PT::PARAMETER_STRING_ARRAY:{
          QVariantList l;
          for(auto&x:v.string_array_value)l<<QString::fromStdString(x);
          return l;
        }
        default:return {
        };
      }
    }
    struct MessageTimingState {
      std::chrono::steady_clock::time_point previous{};
      std::deque<double> periodsMs;
      quint64 messageCount{0};
    };
    template<class Stamp>static QVariantMap messageTimingFields(MessageTimingState &state,const Stamp &stamp){
      const auto nowSteady=std::chrono::steady_clock::now();
      double periodMs=std::numeric_limits<double>::quiet_NaN();
      if(state.previous.time_since_epoch().count()!=0){
        periodMs=std::chrono::duration<double,std::milli>(nowSteady-state.previous).count();
        if(std::isfinite(periodMs)&&periodMs>0.0&&periodMs<5000.0){
          state.periodsMs.push_back(periodMs);
          while(state.periodsMs.size()>120)state.periodsMs.pop_front();
        }
      }
      state.previous=nowSteady;
      ++state.messageCount;
      double meanPeriod=std::numeric_limits<double>::quiet_NaN();
      double jitter=std::numeric_limits<double>::quiet_NaN();
      if(!state.periodsMs.empty()){
        meanPeriod=std::accumulate(state.periodsMs.begin(),state.periodsMs.end(),0.0)/state.periodsMs.size();
        double sum=0.0;
        for(double value:state.periodsMs)sum+=(value-meanPeriod)*(value-meanPeriod);
        jitter=std::sqrt(sum/state.periodsMs.size());
      }
      const double stampSec=double(stamp.sec)+double(stamp.nanosec)*1e-9;
      const double wallSec=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
      const double latencyMs=stampSec>0.0?(wallSec-stampSec)*1000.0:std::numeric_limits<double>::quiet_NaN();
      return QVariantMap{
        {"rate_hz",std::isfinite(meanPeriod)&&meanPeriod>0.0?1000.0/meanPeriod:std::numeric_limits<double>::quiet_NaN()},
        {"period_ms",periodMs},
        {"timestamp_jitter_ms",jitter},
        {"latency_ms",latencyMs},
        {"message_count",QVariant::fromValue<qulonglong>(state.messageCount)}
      };
    }
    static void mergeMap(QVariantMap &dst,const QVariantMap &src){
      for(auto it=src.cbegin();it!=src.cend();++it)dst[it.key()]=it.value();
    }
    template<class Msg,class Callback>void sub(const rclcpp::Node::SharedPtr&n,const std::string&topic,const rclcpp::QoS&q,Callback cb){
      auto subscription=n->create_subscription<Msg>(topic,q,cb);
      std::lock_guard<std::mutex> lock(subscriptionsMutex_);
      subscriptions_.push_back(subscription);
    }
    void emitMap(const QString&ch,const QVariantMap&m){
      emit telemetry(ch,m);
    }
    void run(){
      try{
        auto n=std::make_shared<rclcpp::Node>("agv_gui");
        auto ex=std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(),2);
        ex->add_node(n);
        {
          std::lock_guard<std::mutex>lk(mu_);
          node_=n;
          executor_=ex;
        }
        auto state=rclcpp::QoS(1).reliable();
        auto latched=rclcpp::QoS(1).reliable().transient_local();
        auto sensor=rclcpp::QoS(rclcpp::KeepLast(3)).best_effort();
        auto boolSub=[&](const char*t,const char*c,const rclcpp::QoS&q){
          sub<std_msgs::msg::Bool>(n,t,q,[this,c](std_msgs::msg::Bool::ConstSharedPtr m){
            emit telemetry(c,m->data);
          });
        };
        const std::vector<std::pair<const char*,const char*>> bools={
          {
            "/gnss/connected","connected.gnss"
          },{
            "/imu/connected","connected.imu"
          },{
            "/perception/camera_connected","connected.camera"
          },{
            "/esc/ready","connected.esc_ready"
          },{
            "/esc/armed","connected.esc_armed"
          },{
            "/esc/feedback_valid","connected.esc_feedback"
          },{
            "/esc/steering_calibration/controller_ready","esc_calibration_controller_ready"
          },{
            "/system/autonomy_ready","system.autonomy_ready"
          },{
            "/system/motion_ready","system.motion_ready"
          },{
            "/system/nav2_ready","system.nav2_ready"
          },{
            "/navigation/mppi_closed_loop/ready","mppi_closed_loop_ready"
          },{
            "/navigation/velocity_smoother/closed_loop_eligible","smoother_closed_loop_eligible"
          },{
            "/gnss/velocity_qualified","gnss_velocity_qualified"
          },{
            "/gnss/cog_qualified","gnss_cog_qualified"
          },{
            "/gnss/velocity_fusion_active","gnss_velocity_fusion_active"
          },{
            "/gnss/cog_fusion_active","gnss_cog_fusion_active"
          },{
            "/perception/camera_healthy","camera_healthy"
          },{
            "/perception/emergency_stop","perception_emergency"
          },{
            "/lidar/safety_healthy","lidar_safety_healthy"
          }
        };
        for(auto&p:bools)boolSub(p.first,p.second,latched);
        boolSub("/safety/estop","system.estop",state);
        const std::vector<std::pair<const char*,const char*>> strings={
          {
            "/system/localization_state","localization_state"
          },{
            "/system/gnss_status","gnss_status"
          },{
            "/gnss/state","gnss_driver_state"
          },{
            "/gnss/motion_diagnostics","gnss_motion"
          },{
            "/gnss/motion_validation","gnss_motion_validation"
          },{
            "/gnss/fusion_status","gnss_fusion_status"
          },{
            "/system/imu_status","imu_status"
          },{
            "/system/ekf_local_status","ekf_local_status"
          },{
            "/system/ekf_global_status","ekf_global_status"
          },{
            "/system/pose_estimator","pose_estimator"
          },{
            "/system/sensor_status","sensor_status"
          },{
            "/imu/status","imu_driver_status"
          },{
            "/lidar/status","lidar_driver_status"
          },{
            "/lidar/safety_health","lidar_safety_health"
          },{
            "/navigation/goal_state","goal_state"
          },{
            "/navigation/mppi_closed_loop/status","mppi_status"
          },{
            "/navigation/velocity_smoother/qualification","smoother_qualification"
          },{
            "/perception/lane_safety_state","lane_state"
          },{
            "/perception/lane_control_state","lane_control"
          },{
            "/yolop/lane_metrics","lane_metrics"
          },{
            "/perception/drivable_space","drivable_space"
          },{
            "/perception/camera_health_state","camera_health_state"
          },{
            "/perception/near_field_state","near_field_state"
          },{
            "/perception/obstacle_metrics","obstacle_metrics"
          },{
            "/perception/raw_detections","raw_detections"
          },{
            "/perception/performance","perception_performance"
          },{
            "/navigation/trajectory_safety_state","trajectory_safety_state"
          },{
            "/collision_monitor/state","collision_monitor_state"
          },{
            "/esc/status","esc_status"
          },{
            "/esc/foc/telemetry","foc_telemetry"
          },{
            "/esc/mux/active_source","esc_mux"
          }
        };
        for(auto&p:strings)sub<std_msgs::msg::String>(n,p.first,state,[this,p](std_msgs::msg::String::ConstSharedPtr m){
          const QString raw=QString::fromStdString(m->data);
          const QString channel=QString::fromLatin1(p.second);
          QVariantMap x=parseJsonOrKv(raw);
          if(channel=="raw_detections")x=parseRawDetectionSummary(raw);
          else if(channel=="perception_performance")x=normalizePerceptionPerformance(x);
          else if(channel=="obstacle_metrics")x=enrichObstacleMetrics(x);
          else if(channel=="goal_state"){
            x=QVariantMap{
              {
                "state",raw.trimmed().toUpper()
              },{
                "raw",raw
              }
            };
            std::lock_guard<std::mutex>goalLock(goalMutex_);
            if(goalActive_){
              x["duration_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-goalStarted_).count();
              if(raw.contains("SUCCEEDED",Qt::CaseInsensitive)||raw.contains("CANCELED",Qt::CaseInsensitive)||raw.contains("FAILED",Qt::CaseInsensitive)||raw.contains("ABORT",Qt::CaseInsensitive))goalActive_=false;
            }
          }
          emitMap(channel,x);
        });

        // GUI/PERCEPTION transport compatibility. gui.launch.py starts
        // yolo_obstacle_detection_ros2/perception_all.launch.py, whose production
        // interface is /camera/color/* + /obstacle_detection/*. The experiment
        // pages intentionally keep their stable TelemetryStore keys
        // (raw_detections.*, perception_performance.*, camera_health_state.*).
        // Bridge the production topics here so live graphs and the recorder read
        // exactly the same values; no duplicate perception node is introduced.
        sub<std_msgs::msg::String>(n,"/camera/color/status",latched,[this](std_msgs::msg::String::ConstSharedPtr m){
          const QString raw=QString::fromStdString(m->data);
          const QVariantMap status=parseJsonOrKv(raw);
          emitMap("camera_status",status);
          emit telemetry("connected.camera",true);
          const double fps=number(status.value(QStringLiteral("measured_fps")));
          emit telemetry("camera_healthy",std::isfinite(fps)&&fps>0.5);
        });
        sub<std_msgs::msg::String>(n,"/obstacle_detection/status",latched,[this](std_msgs::msg::String::ConstSharedPtr m){
          const QString raw=QString::fromStdString(m->data);
          QVariantMap status=parseJsonOrKv(raw);
          if(!status.contains(QStringLiteral("raw")))status[QStringLiteral("raw")]=raw;
          emitMap("yolo_status",status);
        });
        // Publisher is BEST_EFFORT; use sensor QoS or DDS will reject the link.
        sub<std_msgs::msg::String>(n,"/obstacle_detection/performance",sensor,[this](std_msgs::msg::String::ConstSharedPtr m){
          QVariantMap perf=normalizePerceptionPerformance(parseJsonOrKv(QString::fromStdString(m->data)));
          emitMap("perception_performance",perf);
        });
        // Real detection stream is a typed ObstacleArray, not std_msgs/String.
        // Convert it once into the stable keys consumed by both plot and CSV code.
        sub<yolo_obstacle_detection_ros2::msg::ObstacleArray>(
          n,"/obstacle_detection/obstacles",rclcpp::QoS(1).reliable(),
          [this](yolo_obstacle_detection_ros2::msg::ObstacleArray::ConstSharedPtr m){
            QVariantMap raw;
            raw[QStringLiteral("count")]=static_cast<int>(m->obstacles.size());
            raw[QStringLiteral("dynamic_count")]=m->dynamic_count;
            raw[QStringLiteral("static_count")]=m->static_count;
            raw[QStringLiteral("floor_count")]=m->floor_count;
            raw[QStringLiteral("pallet_count")]=m->pallet_count;
            raw[QStringLiteral("other_count")]=m->other_count;
            raw[QStringLiteral("warning_active")]=m->warning_active;
            raw[QStringLiteral("path_obstacle_count")]=static_cast<int>(m->path_obstacles.size());
            double sum=0.0,maxConf=std::numeric_limits<double>::quiet_NaN();
            int valid=0;
            const yolo_obstacle_detection_ros2::msg::Obstacle *best=nullptr;
            const yolo_obstacle_detection_ros2::msg::Obstacle *bestPallet=nullptr;
            for(const auto &o:m->obstacles){
              const double c=static_cast<double>(o.confidence);
              if(std::isfinite(c)){
                sum+=c; ++valid;
                if(!std::isfinite(maxConf)||c>maxConf)maxConf=c;
                if(!best || c>static_cast<double>(best->confidence))best=&o;
                const QString cls=QString::fromStdString(o.class_name).trimmed().toLower();
                if((cls==QStringLiteral("pallet")||cls==QStringLiteral("hole pallet")) &&
                   (!bestPallet || c>static_cast<double>(bestPallet->confidence)))bestPallet=&o;
              }
            }
            if(valid>0){
              raw[QStringLiteral("mean_confidence")]=sum/valid;
              raw[QStringLiteral("max_confidence")]=maxConf;
            }
            auto appendDetection=[&raw](const QString &prefix,const yolo_obstacle_detection_ros2::msg::Obstacle *o){
              if(!o)return;
              raw[prefix+QStringLiteral("class_name")]=QString::fromStdString(o->class_name);
              raw[prefix+QStringLiteral("category")]=QString::fromStdString(o->category);
              raw[prefix+QStringLiteral("confidence")]=static_cast<double>(o->confidence);
              raw[prefix+QStringLiteral("center_x_px")]=static_cast<double>(o->center_x);
              raw[prefix+QStringLiteral("center_y_px")]=static_cast<double>(o->center_y);
              raw[prefix+QStringLiteral("x1_px")]=static_cast<double>(o->x1);
              raw[prefix+QStringLiteral("y1_px")]=static_cast<double>(o->y1);
              raw[prefix+QStringLiteral("x2_px")]=static_cast<double>(o->x2);
              raw[prefix+QStringLiteral("y2_px")]=static_cast<double>(o->y2);
              raw[prefix+QStringLiteral("on_path")]=o->on_path;
              raw[prefix+QStringLiteral("in_danger_zone")]=o->in_danger_zone;
            };
            appendDetection(QStringLiteral("best_"),best);
            appendDetection(QStringLiteral("pallet_best_"),bestPallet);
            emitMap("raw_detections",raw);

            QVariantMap metrics;
            metrics[QStringLiteral("count")]=raw.value(QStringLiteral("count"));
            metrics[QStringLiteral("mean_confidence")]=raw.value(QStringLiteral("mean_confidence"));
            metrics[QStringLiteral("warning_active")]=m->warning_active;
            emitMap("obstacle_metrics",metrics);
          });

        // GUI-only bridge for the already-published pallet alignment state.
        // This does not alter perception behavior; it only exposes the runtime
        // values to BAB-IV Perception preview/CSV widgets.
        sub<yolo_obstacle_detection_ros2::msg::AlignmentState>(
          n,"/fork_alignment/state",latched,
          [this](yolo_obstacle_detection_ros2::msg::AlignmentState::ConstSharedPtr m){
            emitMap("alignment_state",{
              {"state",static_cast<int>(m->state)},
              {"state_text",QString::fromStdString(m->state_text)},
              {"pallet_detected",m->pallet_detected},
              {"detection_stable",m->detection_stable},
              {"confidence",static_cast<double>(m->confidence)},
              {"depth_quality",static_cast<double>(m->depth_quality)},
              {"error_lateral_m",static_cast<double>(m->error_lateral_m)},
              {"error_yaw_deg",static_cast<double>(m->error_yaw_deg)},
              {"pid_lateral_output",static_cast<double>(m->pid_lateral_output)},
              {"pid_yaw_output",static_cast<double>(m->pid_yaw_output)},
              {"desired_yaw_rate",static_cast<double>(m->desired_yaw_rate)},
              {"estimated_steering_deg",static_cast<double>(m->estimated_steering_deg)},
              {"linear_velocity_cmd",static_cast<double>(m->linear_velocity_cmd)},
              {"angular_velocity_cmd",static_cast<double>(m->angular_velocity_cmd)},
              {"steering_limit_active",m->steering_limit_active},
              {"safety_stop_active",m->safety_stop_active},
              {"data_valid",m->data_valid},
              {"lateral_within_tolerance",m->lateral_within_tolerance},
              {"yaw_within_tolerance",m->yaw_within_tolerance},
              {"steering_centered",m->steering_centered},
              {"ready_for_insertion",m->ready_for_insertion}
            });
          });

        sub<sensor_msgs::msg::NavSatFix>(n,"/gnss/fix_raw",sensor,[this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m){
          emitMap("gnss_fix",{
            {
              "lat",m->latitude
            },{
              "lon",m->longitude
            },{
              "alt",m->altitude
            },{
              "status",m->status.status
            },{
              "cov_x",m->position_covariance[0]
            },{
              "cov_y",m->position_covariance[4]
            },{
              "measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9
            }
          });
        });
        sub<std_msgs::msg::Float64MultiArray>(n,"/gnss/quality",sensor,[this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m){
          std::vector<double>d=m->data;
          d.resize(std::max<size_t>(45,d.size()),std::numeric_limits<double>::quiet_NaN());
          const char*keys[]={
            "sat","dop","hacc_m","fix_type","source_id","sacc_mps","ground_speed_mps","course_enu_rad","course_accuracy_rad","itow_ms","vacc_m","vel_e_mps","vel_n_mps","vel_d_mps","gdop","hdop","vdop","ndop","edop","tdop","nav_cov_pos_valid","nav_cov_vel_valid","pvt_rate_hz","measurement_age_sec","timestamp_source_code","flags2","flags3","nav_dop_itow_ms","nav_dop_exact_epoch","nav_cov_itow_ms","nav_cov_exact_epoch","utc_valid_flags","tacc_ns","height_ellipsoid_m","head_vehicle_enu_rad","mag_declination_rad","mag_accuracy_rad","diff_solution","carrier_solution","invalid_llh","last_correction_age_code","auth_time","head_vehicle_valid","mag_valid","gnss_fix_ok"
          };
          QVariantMap x;
          for(int i=0;
          i<45;
          ++i)x[keys[i]]=d[i];
          for(const char*k:{
            "nav_cov_pos_valid","nav_cov_vel_valid","nav_dop_exact_epoch","nav_cov_exact_epoch","diff_solution","invalid_llh","auth_time","head_vehicle_valid","mag_valid","gnss_fix_ok"
          })x[k]=number(x[k],0)>0.5;
          emitMap("gnss_quality",x);
        });
        auto velSub=[&](const char*topic,const char*ch){
          sub<geometry_msgs::msg::TwistWithCovarianceStamped>(n,topic,sensor,[this,ch](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr m){
            double vx=m->twist.twist.linear.x,vy=m->twist.twist.linear.y;
            emitMap(ch,{
              {
                "vx",vx
              },{
                "vy",vy
              },{
                "vz",m->twist.twist.linear.z
              },{
                "speed",std::hypot(vx,vy)
              },{
                "course_enu_rad",std::hypot(vx,vy)>1e-9?std::atan2(vy,vx):std::numeric_limits<double>::quiet_NaN()
              },{
                "cov_x",m->twist.covariance[0]
              },{
                "cov_y",m->twist.covariance[7]
              },{
                "measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9
              }
            });
          });
        };
        velSub("/gnss/vel","gnss_vel");
        velSub("/gnss/velocity_position_fit","gnss_vel_fit");
        velSub("/gnss/vel_map","gnss_vel_map");
        velSub("/gnss/base_velocity","gnss_base_vel");
        velSub("/gnss/base_velocity_fusion","gnss_base_vel_fusion");
        sub<geometry_msgs::msg::PoseWithCovarianceStamped>(n,"/gnss/cog_heading_fusion",sensor,[this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
          auto&q=m->pose.pose.orientation;
          emitMap("gnss_cog_fusion",{
            {
              "yaw_rad",yawFromQuat(q.x,q.y,q.z,q.w)
            },{
              "yaw_variance",m->pose.covariance[35]
            },{
              "measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9
            }
          });
        });
        sub<sensor_msgs::msg::Imu>(n,"/imu/data",sensor,[this,n](sensor_msgs::msg::Imu::ConstSharedPtr m){
          static MessageTimingState timing;
          auto&q=m->orientation;
          double sinr=2*(q.w*q.x+q.y*q.z),cosr=1-2*(q.x*q.x+q.y*q.y),roll=std::atan2(sinr,cosr),sinp=2*(q.w*q.y-q.z*q.x),pitch=std::abs(sinp)>=1?std::copysign(kPi/2,sinp):std::asin(sinp);
          QVariantMap x{
            {"roll_rad",roll},{"pitch_rad",pitch},{"yaw_rad",yawFromQuat(q.x,q.y,q.z,q.w)},
            {"qx",q.x},{"qy",q.y},{"qz",q.z},{"qw",q.w},
            {"gx",m->angular_velocity.x},{"gy",m->angular_velocity.y},{"gz",m->angular_velocity.z},
            {"ax",m->linear_acceleration.x},{"ay",m->linear_acceleration.y},{"az",m->linear_acceleration.z},
            {"orientation_valid",m->orientation_covariance[0]>=0.0},
            {"var_roll",m->orientation_covariance[0]},{"var_pitch",m->orientation_covariance[4]},{"var_yaw",m->orientation_covariance[8]},
            {"var_gx",m->angular_velocity_covariance[0]},{"var_gy",m->angular_velocity_covariance[4]},{"var_gz",m->angular_velocity_covariance[8]},
            {"var_ax",m->linear_acceleration_covariance[0]},{"var_ay",m->linear_acceleration_covariance[4]},{"var_az",m->linear_acceleration_covariance[8]},
            {"frame_id",QString::fromStdString(m->header.frame_id)},
            {"publisher_count",QVariant::fromValue<qulonglong>(n->count_publishers("/imu/data"))},
            {"measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9}
          };
          mergeMap(x,messageTimingFields(timing,m->header.stamp));
          emitMap("imu",x);
        });
        // Capture every IMU component stream already published by imu_node.
        // Separate channels avoid overwriting fused /imu/data telemetry.
        auto imuComponentSub=[&](const char*topic,const char*channel,bool gyroOnly){
          const std::string topicName(topic);
          const QString channelName=QString::fromUtf8(channel);
          auto timing=std::make_shared<MessageTimingState>();
          sub<sensor_msgs::msg::Imu>(n,topic,sensor,[this,n,topicName,channelName,gyroOnly,timing](sensor_msgs::msg::Imu::ConstSharedPtr m){
            QVariantMap x{{"frame_id",QString::fromStdString(m->header.frame_id)},
              {"publisher_count",QVariant::fromValue<qulonglong>(n->count_publishers(topicName))},
              {"measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9}};
            if(gyroOnly){
              x["gx"]=m->angular_velocity.x; x["gy"]=m->angular_velocity.y; x["gz"]=m->angular_velocity.z;
              x["var_gx"]=m->angular_velocity_covariance[0]; x["var_gy"]=m->angular_velocity_covariance[4]; x["var_gz"]=m->angular_velocity_covariance[8];
            }else{
              x["ax"]=m->linear_acceleration.x; x["ay"]=m->linear_acceleration.y; x["az"]=m->linear_acceleration.z;
              x["var_ax"]=m->linear_acceleration_covariance[0]; x["var_ay"]=m->linear_acceleration_covariance[4]; x["var_az"]=m->linear_acceleration_covariance[8];
            }
            mergeMap(x,messageTimingFields(*timing,m->header.stamp));
            emitMap(channelName,x);
          });
        };
        imuComponentSub("/imu/gyro","imu_gyro",true);
        imuComponentSub("/imu/accel","imu_accel",false);
        auto imuVectorSub=[&](const char*topic,const char*channel,const char*xKey,const char*yKey,const char*zKey){
          const std::string topicName(topic);
          const QString channelName=QString::fromUtf8(channel);
          const QString kx=QString::fromUtf8(xKey),ky=QString::fromUtf8(yKey),kz=QString::fromUtf8(zKey);
          auto timing=std::make_shared<MessageTimingState>();
          sub<geometry_msgs::msg::Vector3Stamped>(n,topic,sensor,[this,n,topicName,channelName,kx,ky,kz,timing](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr m){
            QVariantMap x{{kx,m->vector.x},{ky,m->vector.y},{kz,m->vector.z},
              {"frame_id",QString::fromStdString(m->header.frame_id)},
              {"publisher_count",QVariant::fromValue<qulonglong>(n->count_publishers(topicName))},
              {"measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9}};
            mergeMap(x,messageTimingFields(*timing,m->header.stamp));
            emitMap(channelName,x);
          });
        };
        imuVectorSub("/imu/mag","imu_mag","mx","my","mz");
        imuVectorSub("/imu/euler","imu_euler","roll_rad","pitch_rad","yaw_rad");
        {
          const std::string topicName("/imu/mag_field");
          auto timing=std::make_shared<MessageTimingState>();
          sub<sensor_msgs::msg::MagneticField>(n,topicName,sensor,[this,n,topicName,timing](sensor_msgs::msg::MagneticField::ConstSharedPtr m){
            QVariantMap x{{"mx",m->magnetic_field.x},{"my",m->magnetic_field.y},{"mz",m->magnetic_field.z},
              {"var_mx",m->magnetic_field_covariance[0]},{"var_my",m->magnetic_field_covariance[4]},{"var_mz",m->magnetic_field_covariance[8]},
              {"frame_id",QString::fromStdString(m->header.frame_id)},
              {"publisher_count",QVariant::fromValue<qulonglong>(n->count_publishers(topicName))},
              {"measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9}};
            mergeMap(x,messageTimingFields(*timing,m->header.stamp));
            emitMap("imu_mag_field",x);
          });
        }
        sub<sensor_msgs::msg::LaserScan>(n,"/scan",sensor,[this,n](sensor_msgs::msg::LaserScan::ConstSharedPtr m){
          static std::chrono::steady_clock::time_point previous;
          static std::deque<double> periodsMs;
          static quint64 messageCount=0;
          ++messageCount;
          const auto now=std::chrono::steady_clock::now();
          double latestPeriodMs=std::numeric_limits<double>::quiet_NaN();
          if(previous.time_since_epoch().count()!=0){
            latestPeriodMs=std::chrono::duration<double,std::milli>(now-previous).count();
            if(std::isfinite(latestPeriodMs)&&latestPeriodMs>0.0&&latestPeriodMs<5000.0){
              periodsMs.push_back(latestPeriodMs);
              while(periodsMs.size()>60)periodsMs.pop_front();
            }
          }
          previous=now;
          size_t valid=0;
          for(float range:m->ranges)
            if(std::isfinite(range)&&range>=m->range_min&&range<=m->range_max)++valid;
          QVector<double> center;
          int sampleIndex=-1;
          double sampleAngleDeg=std::numeric_limits<double>::quiet_NaN();
          double sampleRange=std::numeric_limits<double>::quiet_NaN();
          double sampleIntensity=std::numeric_limits<double>::quiet_NaN();
          bool sampleValid=false;
          if(!m->ranges.empty()&&std::abs(m->angle_increment)>1e-12){
            sampleIndex=int(std::lround((0.0-m->angle_min)/m->angle_increment));
            sampleIndex=std::clamp(sampleIndex,0,int(m->ranges.size())-1);
            sampleAngleDeg=(m->angle_min+double(sampleIndex)*m->angle_increment)*180.0/kPi;
            sampleRange=m->ranges[size_t(sampleIndex)];
            sampleValid=std::isfinite(sampleRange)&&sampleRange>=m->range_min&&sampleRange<=m->range_max;
            if(size_t(sampleIndex)<m->intensities.size())sampleIntensity=m->intensities[size_t(sampleIndex)];
            for(int offset=-2;offset<=2;++offset){
              const int index=sampleIndex+offset;
              if(index<0||index>=int(m->ranges.size()))continue;
              const double range=m->ranges[size_t(index)];
              if(std::isfinite(range)&&range>=m->range_min&&range<=m->range_max)center<<range;
            }
          }
          double centerMean=std::numeric_limits<double>::quiet_NaN();
          if(!center.isEmpty())centerMean=std::accumulate(center.begin(),center.end(),0.0)/center.size();
          double meanPeriod=std::numeric_limits<double>::quiet_NaN(),jitter=std::numeric_limits<double>::quiet_NaN();
          if(!periodsMs.empty()){
            meanPeriod=std::accumulate(periodsMs.begin(),periodsMs.end(),0.0)/periodsMs.size();
            double sum=0.0;
            for(double value:periodsMs)sum+=(value-meanPeriod)*(value-meanPeriod);
            jitter=std::sqrt(sum/periodsMs.size());
          }
          const double validRatio=m->ranges.empty()?0.0:100.0*double(valid)/double(m->ranges.size());
          const double stampSec=double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9;
          const double wallSec=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
          const double latencyMs=stampSec>0.0?(wallSec-stampSec)*1000.0:std::numeric_limits<double>::quiet_NaN();
          emitMap("lidar",{
            {"range_center_m",centerMean},
            {"sample_angle_deg",sampleAngleDeg},{"sample_range_m",sampleRange},{"sample_intensity",sampleIntensity},{"sample_valid",sampleValid},
            {"valid_ratio_pct",validRatio},{"dropout_pct",100.0-validRatio},
            {"scan_rate_hz",std::isfinite(meanPeriod)&&meanPeriod>0.0?1000.0/meanPeriod:std::numeric_limits<double>::quiet_NaN()},
            {"period_ms",latestPeriodMs},{"timestamp_jitter_ms",jitter},{"latency_ms",latencyMs},
            {"beam_count",qlonglong(m->ranges.size())},{"valid_beam_count",qlonglong(valid)},
            {"message_count",QVariant::fromValue<qulonglong>(messageCount)},
            {"publisher_count",QVariant::fromValue<qulonglong>(n->count_publishers("/scan"))},
            {"frame_id",QString::fromStdString(m->header.frame_id)},
            {"measurement_stamp_sec",stampSec}
          });
        });
        // Capture every LiDAR stream that already exists in GUI mode.
        // Raw streams remain visible even before TF-dependent self filters emit.
        auto lidarAuxSub=[&](const char*topic,const char*channel){
          const std::string topicName(topic);
          const QString channelName=QString::fromUtf8(channel);
          auto timing=std::make_shared<MessageTimingState>();
          sub<sensor_msgs::msg::LaserScan>(n,topic,sensor,[this,n,topicName,channelName,timing](sensor_msgs::msg::LaserScan::ConstSharedPtr m){
            size_t valid=0;
            for(float range:m->ranges)if(std::isfinite(range)&&range>=m->range_min&&range<=m->range_max)++valid;
            int sampleIndex=-1; double sampleAngleDeg=std::numeric_limits<double>::quiet_NaN();
            double sampleRange=std::numeric_limits<double>::quiet_NaN(),sampleIntensity=std::numeric_limits<double>::quiet_NaN();
            bool sampleValid=false; QVector<double> center;
            if(!m->ranges.empty()&&std::abs(m->angle_increment)>1e-12){
              sampleIndex=int(std::lround((0.0-m->angle_min)/m->angle_increment));
              sampleIndex=std::clamp(sampleIndex,0,int(m->ranges.size())-1);
              sampleAngleDeg=(m->angle_min+double(sampleIndex)*m->angle_increment)*180.0/kPi;
              sampleRange=m->ranges[size_t(sampleIndex)];
              sampleValid=std::isfinite(sampleRange)&&sampleRange>=m->range_min&&sampleRange<=m->range_max;
              if(size_t(sampleIndex)<m->intensities.size())sampleIntensity=m->intensities[size_t(sampleIndex)];
              for(int offset=-2;offset<=2;++offset){
                const int index=sampleIndex+offset; if(index<0||index>=int(m->ranges.size()))continue;
                const double range=m->ranges[size_t(index)];
                if(std::isfinite(range)&&range>=m->range_min&&range<=m->range_max)center<<range;
              }
            }
            double centerMean=std::numeric_limits<double>::quiet_NaN();
            if(!center.isEmpty())centerMean=std::accumulate(center.begin(),center.end(),0.0)/center.size();
            QVariantMap x{{"range_center_m",centerMean},{"sample_angle_deg",sampleAngleDeg},{"sample_range_m",sampleRange},
              {"sample_intensity",sampleIntensity},{"sample_valid",sampleValid},
              {"valid_ratio_pct",m->ranges.empty()?0.0:100.0*double(valid)/double(m->ranges.size())},
              {"dropout_pct",m->ranges.empty()?100.0:100.0-100.0*double(valid)/double(m->ranges.size())},
              {"beam_count",qlonglong(m->ranges.size())},{"valid_beam_count",qlonglong(valid)},
              {"angle_min_rad",m->angle_min},{"angle_max_rad",m->angle_max},{"angle_increment_rad",m->angle_increment},
              {"range_min_m",m->range_min},{"range_max_m",m->range_max},{"scan_time_s",m->scan_time},{"time_increment_s",m->time_increment},
              {"frame_id",QString::fromStdString(m->header.frame_id)},{"source_topic",QString::fromStdString(topicName)},
              {"publisher_count",QVariant::fromValue<qulonglong>(n->count_publishers(topicName))},
              {"measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9}};
            mergeMap(x,messageTimingFields(*timing,m->header.stamp));
            emitMap(channelName,x);
          });
        };
        lidarAuxSub("/scan_safety_raw","lidar_raw");
        lidarAuxSub("/scan_nav_raw","lidar_nav_raw");
        lidarAuxSub("/scan_safety","lidar_safety");
        lidarAuxSub("/scan_nav","lidar_nav");
        sub<nav_msgs::msg::OccupancyGrid>(n,"/map",latched,[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
          static std::chrono::steady_clock::time_point previous;
          const auto now=std::chrono::steady_clock::now();
          double updateHz=std::numeric_limits<double>::quiet_NaN();
          if(previous.time_since_epoch().count()!=0){
            const double dt=std::chrono::duration<double>(now-previous).count();
            if(dt>1e-6)updateHz=1.0/dt;
          }
          previous=now;
          size_t unknown=0,occupied=0;
          for(const int8_t value:m->data){
            if(value<0)++unknown;
            else if(value>=65)++occupied;
          }
          const double total=double(std::max<size_t>(1,m->data.size()));
          emitMap("slam",{
            {"width_cells",qlonglong(m->info.width)},
            {"height_cells",qlonglong(m->info.height)},
            {"resolution_m",m->info.resolution},
            {"width_m",m->info.width*m->info.resolution},
            {"height_m",m->info.height*m->info.resolution},
            {"unknown_pct",100.0*double(unknown)/total},
            {"occupied_pct",100.0*double(occupied)/total},
            {"map_update_hz",updateHz},
            {"measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9}
          });
        });
        sub<geometry_msgs::msg::PoseWithCovarianceStamped>(n,"/amcl_pose",sensor,[this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
          const auto &position=m->pose.pose.position;
          const auto &orientation=m->pose.pose.orientation;
          emitMap("amcl",{
            {"x",position.x},{"y",position.y},
            {"yaw",yawFromQuat(orientation.x,orientation.y,orientation.z,orientation.w)},
            {"var_x",m->pose.covariance[0]},
            {"var_y",m->pose.covariance[7]},
            {"var_yaw",m->pose.covariance[35]},
            {"measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9}
          });
        });
        auto odomSub=[&](const char*topic,const char*ch){
          const std::string topicName(topic);
          const QString channel=QString::fromUtf8(ch);
          auto timing=std::make_shared<MessageTimingState>();
          sub<nav_msgs::msg::Odometry>(n,topic,sensor,[this,n,topicName,channel,timing](nav_msgs::msg::Odometry::ConstSharedPtr m){
            auto&p=m->pose.pose.position;
            auto&q=m->pose.pose.orientation;
            QVariantMap x{
              {"x",p.x},{"y",p.y},{"yaw",yawFromQuat(q.x,q.y,q.z,q.w)},
              {"v",m->twist.twist.linear.x},{"vy",m->twist.twist.linear.y},{"w",m->twist.twist.angular.z},
              {"var_x",m->pose.covariance[0]},{"var_y",m->pose.covariance[7]},{"var_yaw",m->pose.covariance[35]},
              {"frame_id",QString::fromStdString(m->header.frame_id)},
              {"child_frame_id",QString::fromStdString(m->child_frame_id)},
              {"publisher_count",QVariant::fromValue<qulonglong>(n->count_publishers(topicName))},
              {"measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9}
            };
            mergeMap(x,messageTimingFields(*timing,m->header.stamp));
            emitMap(channel,x);
          });
        };
        odomSub("/esc/odom","esc_odom");
        odomSub("/odom","odom");
        odomSub("/odometry/filtered","ekf_local");
        odomSub("/odometry/filtered_map","ekf_global");
        auto pathSub=[&](const char*topic,const char*ch){
          sub<nav_msgs::msg::Path>(n,topic,sensor,[this,ch](nav_msgs::msg::Path::ConstSharedPtr m){
            QVariantList points;
            points.reserve(int(m->poses.size()));
            double length=0.0;
            double headingVariation=0.0;
            double previousHeading=0.0;
            bool havePrevious=false;
            for(size_t i=0;
            i<m->poses.size();
            ++i){
              const auto&p=m->poses[i].pose.position;
              const auto&q=m->poses[i].pose.orientation;
              double yaw=yawFromQuat(q.x,q.y,q.z,q.w);
              if(i>0){
                const auto&prev=m->poses[i-1].pose.position;
                length+=std::hypot(p.x-prev.x,p.y-prev.y);
              }
              if(havePrevious)headingVariation+=std::abs(normalizeAngle(yaw-previousHeading));
              previousHeading=yaw;
              havePrevious=true;
              points<<QVariantList{
                p.x,p.y,yaw
              };
            }
            double latency=std::numeric_limits<double>::quiet_NaN();
            {
              std::lock_guard<std::mutex>goalLock(goalMutex_);
              if(goalActive_&&!firstPlanSeen_){
                lastPlanningLatencyMs_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-goalStarted_).count();
                firstPlanSeen_=true;
              }
              latency=lastPlanningLatencyMs_;
            }
            emitMap(ch,{
              {
                "count",qlonglong(m->poses.size())
              },{
                "length_m",length
              },{
                "heading_variation_rad",headingVariation
              },{
                "planning_latency_ms",latency
              },{
                "points",points
              },{
                "measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9
              }
            });
          });
        };
        pathSub("/plan","nav_path");
        pathSub("/controller_server/transformed_global_plan","local_path");
        pathSub("/local_plan","local_path");
        auto goalSub=[&](const char*topic){
          sub<geometry_msgs::msg::PoseStamped>(n,topic,state,[this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){
            const auto&q=m->pose.orientation;
            {
              std::lock_guard<std::mutex>goalLock(goalMutex_);
              goalStarted_=std::chrono::steady_clock::now();
              goalActive_=true;
              firstPlanSeen_=false;
              lastPlanningLatencyMs_=std::numeric_limits<double>::quiet_NaN();
            }
            emitMap("goal_pose",{
              {
                "x",m->pose.position.x
              },{
                "y",m->pose.position.y
              },{
                "yaw",yawFromQuat(q.x,q.y,q.z,q.w)
              }
            });
          });
        };
        goalSub("/navigation/goal_request");
        goalSub("/goal_pose");
        auto twSub=[&](const char*topic,const char*ch){
          sub<geometry_msgs::msg::Twist>(n,topic,sensor,[this,ch](geometry_msgs::msg::Twist::ConstSharedPtr m){
            emitMap(ch,{
              {
                "linear_x",m->linear.x
              },{
                "angular_z",m->angular.z
              }
            });
          });
        };
        twSub("/cmd_vel_nav_smoothed","cmd_nav");
        twSub("/cmd_vel/perception_advisory","cmd_perception_advisory");
        twSub("/cmd_vel/autonomy_integrated","cmd_autonomy_integrated");
        twSub("/cmd_vel/nav2_pre_collision","cmd_pre_collision");
        twSub("/cmd_vel/collision_preview","cmd_collision_preview");
        twSub("/cmd_vel","cmd_final");
        twSub("/cmd_vel/actuator","cmd_actuator");
        const std::vector<std::pair<const char*,const char*>> floats={
          {
            "/esc/drive_target_mps","esc_drive_target"
          },{
            "/esc/drive_actual_mps","esc_drive_actual"
          },{
            "/esc/steering_target_rad","esc_steer_target"
          },{
            "/esc/steering_command_uncalibrated_rad","esc_steer_uncal_target"
          },{
            "/esc/steering_uncalibrated_rad","esc_steer_uncalibrated"
          },{
            "/esc/steering_protocol_command_rad","esc_steer_protocol_cmd"
          },{
            "/esc/steering_feedback_raw_rad","esc_steer_feedback_raw"
          },{
            "/esc/steering_actual_rad","esc_steer_actual"
          },{
            "/esc/yaw_rate_actual_rps","esc_yaw_rate"
          },{
            "/esc/kinematic_yaw_rate_rps","esc_kinematic_yaw_rate"
          },{
            "/navigation/mppi_closed_loop/velocity_error_mps","mppi_velocity_error"
          },{
            "/navigation/mppi_closed_loop/steering_error_rad","mppi_steering_error"
          },{
            "/navigation/mppi_closed_loop/yaw_rate_error_rps","mppi_yaw_error"
          }
        };
        for(auto&p:floats)sub<std_msgs::msg::Float64>(n,p.first,sensor,[this,p](std_msgs::msg::Float64::ConstSharedPtr m){
          emit telemetry(p.second,m->data);
        });
        auto annotatedImageSub=[&](const char *topic){
          sub<sensor_msgs::msg::Image>(n,topic,sensor,[this](sensor_msgs::msg::Image::ConstSharedPtr m){
            if(m->width==0||m->height==0)return;
            QImage img;
            if(m->encoding=="rgb8")img=QImage(m->data.data(),m->width,m->height,m->step,QImage::Format_RGB888).copy();
            else if(m->encoding=="bgr8")img=QImage(m->data.data(),m->width,m->height,m->step,QImage::Format_RGB888).rgbSwapped().copy();
            else if(m->encoding=="mono8"||m->encoding=="8UC1")img=QImage(m->data.data(),m->width,m->height,m->step,QImage::Format_Grayscale8).copy();
            if(!img.isNull())emit image(img);
          });
        };
        // Current production visualization + backwards-compatible old YOLOP topic.
        annotatedImageSub("/obstacle_detection/visualization");
        annotatedImageSub("/camera/yolop/image_annotated");

        // Lightweight camera brightness statistics for BAB-IV plots. Sample the
        // raw BGR/RGB frame sparsely; this avoids OpenCV and does not copy frames.
        sub<sensor_msgs::msg::Image>(n,"/camera/color/image_raw",sensor,[this](sensor_msgs::msg::Image::ConstSharedPtr m){
          if(m->width==0||m->height==0||m->data.empty())return;
          const long long nowNs=std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
          long long previous=lastCameraMetricNs_.load(std::memory_order_relaxed);
          if(nowNs-previous<200000000LL)return;
          if(!lastCameraMetricNs_.compare_exchange_strong(previous,nowNs,std::memory_order_relaxed))return;
          const bool mono=(m->encoding=="mono8"||m->encoding=="8UC1");
          const bool rgb=(m->encoding=="rgb8");
          const bool bgr=(m->encoding=="bgr8");
          if(!mono&&!rgb&&!bgr)return;
          const int channels=mono?1:3;
          const int sx=std::max<int>(1,static_cast<int>(m->width/64));
          const int sy=std::max<int>(1,static_cast<int>(m->height/36));
          double sum=0.0,sum2=0.0,grad=0.0;
          quint64 count=0,gradCount=0;
          for(int y=0;y<static_cast<int>(m->height);y+=sy){
            const size_t row=static_cast<size_t>(y)*m->step;
            double prev=std::numeric_limits<double>::quiet_NaN();
            for(int x=0;x<static_cast<int>(m->width);x+=sx){
              const size_t i=row+static_cast<size_t>(x)*channels;
              if(i+channels>m->data.size())break;
              double l=0.0;
              if(mono)l=m->data[i];
              else{
                const double c0=m->data[i],c1=m->data[i+1],c2=m->data[i+2];
                const double r=rgb?c0:c2,g=c1,b=rgb?c2:c0;
                l=0.299*r+0.587*g+0.114*b;
              }
              sum+=l; sum2+=l*l; ++count;
              if(std::isfinite(prev)){grad+=std::abs(l-prev);++gradCount;}
              prev=l;
            }
          }
          if(count==0)return;
          const double mean=sum/count;
          const double variance=std::max(0.0,sum2/count-mean*mean);
          emitMap("camera_health_state",{
            {"mean_luma",mean},
            {"stddev_luma",std::sqrt(variance)},
            {"mean_gradient",gradCount?grad/gradCount:0.0},
            {"width",static_cast<int>(m->width)},
            {"height",static_cast<int>(m->height)}
          });
          emit telemetry("connected.camera",true);
        });
        auto cloud=[&](const char*t,const char*c){
          sub<sensor_msgs::msg::PointCloud2>(n,t,sensor,[this,c](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){
            emitMap(c,{
              {
                "count",qlonglong(m->width)*m->height
              },{
                "frame_id",QString::fromStdString(m->header.frame_id)
              }
            });
          });
        };
        cloud("/perception/object_points","object_points");
        cloud("/perception/path_relevant_points","path_relevant_points");
        cloud("/perception/planning_relevant_points","planning_relevant_points");
        cloud("/perception/drivable_boundary_points","drivable_boundary_points");

        // GUI-only ROS graph health. This does not start, stop, configure, or
        // modify Nav2/TF. It only reports nodes/publishers that already exist so
        // System Overview does not infer TF from EKF or Nav2 from a missing
        // /system/nav2_ready publisher.
        auto graphHealthTimer=n->create_wall_timer(1000ms,[this,n](){
          bool planner=false,controller=false,bt=false,behavior=false,smoother=false;
          const auto names=n->get_node_names();
          for(const auto &rawName:names){
            const QString name=QString::fromStdString(rawName);
            if(name.endsWith(QStringLiteral("/planner_server"))||name==QStringLiteral("planner_server"))planner=true;
            else if(name.endsWith(QStringLiteral("/controller_server"))||name==QStringLiteral("controller_server"))controller=true;
            else if(name.endsWith(QStringLiteral("/bt_navigator"))||name==QStringLiteral("bt_navigator"))bt=true;
            else if(name.endsWith(QStringLiteral("/behavior_server"))||name==QStringLiteral("behavior_server"))behavior=true;
            else if(name.endsWith(QStringLiteral("/velocity_smoother"))||name==QStringLiteral("velocity_smoother"))smoother=true;
          }
          const int coreCount=(planner?1:0)+(controller?1:0)+(bt?1:0);
          emitMap("nav2_runtime",{
            {"planner_server",planner},
            {"controller_server",controller},
            {"bt_navigator",bt},
            {"behavior_server",behavior},
            {"velocity_smoother",smoother},
            {"core_count",coreCount},
            {"core_present",coreCount>=3},
            {"node_count",static_cast<int>(names.size())}
          });
          const auto dynamicPublishers=n->count_publishers("/tf");
          const auto staticPublishers=n->count_publishers("/tf_static");
          emitMap("tf_runtime",{
            {"dynamic_publishers",QVariant::fromValue<qulonglong>(dynamicPublishers)},
            {"static_publishers",QVariant::fromValue<qulonglong>(staticPublishers)},
            {"available",(dynamicPublishers+staticPublishers)>0}
          });
        });
        (void)graphHealthTimer;

        goalPub_=n->create_publisher<geometry_msgs::msg::PoseStamped>("/navigation/goal_request",10);
        initialPub_=n->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",10);
        emit ready(true,"ROS 2 C++ bridge aktif");
        while(rclcpp::ok()&&!stop_)ex->spin_once(100ms);
      }
      catch(const std::exception&e){
        emit ready(false,QStringLiteral("ROS bridge gagal: ")+e.what());
      }
      std::lock_guard<std::mutex>lk(mu_);
      if(executor_&&node_)executor_->remove_node(node_);
      node_.reset();
      executor_.reset();
      goalPub_.reset();
      initialPub_.reset();
    }
  };
  #include "modules/experiment_components.hpp"
  #include "modules/experiment_components.cpp"
  #include "modules/system_and_gnss_pages.cpp"
  #include "modules/steering_calibration_page.cpp"
  #include "modules/esc_map_overview_pages.cpp"
  class CameraCalibrationCanvas:public QWidget{
    Q_OBJECT
    public:CameraCalibrationCanvas(const QMap<QString,std::shared_ptr<YamlStore>>&s,QWidget*p=nullptr):QWidget(p),s_(s){
      per_=s_.value("perception");
      gui_=s_.value("gui");
      calW_=per_?per_->get("perception.ros__parameters.ground_calibration_width",1280).toInt():1280;
      calH_=per_?per_->get("perception.ros__parameters.ground_calibration_height",720).toInt():720;
      auto*l=new QVBoxLayout(this);
      l->setContentsMargins(0,0,0,0);
      auto*bar=new QHBoxLayout();
      auto*fit=new QPushButton("Fit Camera");
      auto*reloadBtn=new QPushButton("Reload YAML");
      coord_=new QLabel("Drag titik overlay; metric tampil di sini");
      bar->addWidget(fit);
      bar->addWidget(reloadBtn);
      bar->addWidget(coord_,1);
      l->addLayout(bar);
      scene_=new QGraphicsScene(this);
      scene_->setSceneRect(0,0,calW_,calH_);
      view_=new ZoomGraphicsView();
      view_->setScene(scene_);
      l->addWidget(view_,1);
      pix_=scene_->addPixmap(QPixmap());
      pix_->setZValue(-100);
      connect(fit,&QPushButton::clicked,this,[this](){
        fitView();
      });
      connect(reloadBtn,&QPushButton::clicked,this,[this](){
        this->reload();
      });
      placeholder();
      this->reload();
    }
    void setImage(const QImage&i){
      last_=i.copy();
      pix_->setPixmap(QPixmap::fromImage(last_).scaled(calW_,calH_,Qt::IgnoreAspectRatio,Qt::SmoothTransformation));
    }
    ZoomGraphicsView*view()const{
      return view_;
    }
    void setVisibleLayer(const QString&layer,bool on){
      visible_[layer]=on;
      for(auto*h:handles_[layer])h->setVisible(on);
      for(auto*l:lines_[layer])l->setVisible(on);
    }
    QVector<QPointF> metricPoints(const QString&layer)const{
      QVector<QPointF>out;
      for(int i=0;
      i<4;
      ++i){
        bool ok=false;
        QPointF p=projectPixel(number(points_.value(layer).value(2*i)),number(points_.value(layer).value(2*i+1)),&ok);
        if(!ok)return{
        };
        out<<p;
      }
      return out;
    }
    signals:void pointsChanged(QString layer,QVariantList points);
    protected:void resizeEvent(QResizeEvent*e)override{
      QWidget::resizeEvent(e);
      QTimer::singleShot(0,this,[this](){
        fitView();
      });
    }
    private:QMap<QString,std::shared_ptr<YamlStore>>s_;
    std::shared_ptr<YamlStore>per_,gui_;
    int calW_,calH_;
    QGraphicsScene*scene_;
    ZoomGraphicsView*view_;
    QGraphicsPixmapItem*pix_;
    QLabel*coord_;
    QImage last_;
    QMap<QString,QVariantList>points_;
    QMap<QString,QVector<DragHandle*>>handles_;
    QMap<QString,QVector<QGraphicsLineItem*>>lines_;
    QMap<QString,bool>visible_{
      {
        "ground",true
      },{
        "obstacle",true
      },{
        "lane",true
      }
    };
    void placeholder(){
      QImage i(calW_,calH_,QImage::Format_RGB32);
      i.fill(QColor("#11151a"));
      QPainter p(&i);
      p.setPen(QColor(kMuted));
      p.setFont(QFont("DejaVu Sans",22,QFont::Bold));
      p.drawText(i.rect(),Qt::AlignCenter,"Menunggu /camera/yolop/image_annotated");
      pix_->setPixmap(QPixmap::fromImage(i));
    }
    QVariantList valid(const QVariant&v,const QVariantList&fallback){
      QVariantList x=v.toList();
      return x.size()==8?x:fallback;
    }
    void reload(){
      if(per_)per_->reload();
      if(gui_)gui_->reload();
      points_["ground"]=valid(per_->get("perception.ros__parameters.ground_src_points"),{
        40.,680.,1240.,680.,760.,350.,520.,350.
      });
      points_["obstacle"]=valid(gui_->get("camera_overlay.obstacle_roi_points_px"),{
        180.,700.,1100.,700.,780.,380.,500.,380.
      });
      points_["lane"]=valid(gui_->get("camera_overlay.lane_safety_points_px"),{
        340.,700.,940.,700.,740.,420.,540.,420.
      });
      redraw();
      QTimer::singleShot(0,this,[this](){
        fitView();
      });
    }
    QColor color(const QString&l)const{
      return l=="ground"?QColor(kGold):l=="obstacle"?QColor(kRed):QColor(kGreen);
    }
    void redraw(){
      for(auto hs:handles_)for(auto*h:hs)scene_->removeItem(h);
      for(auto ls:lines_)for(auto*x:ls)scene_->removeItem(x);
      handles_.clear();
      lines_.clear();
      for(const QString&layer:{
        QString("ground"),QString("obstacle"),QString("lane")
      }){
        QVariantList pts=points_[layer];
        for(int i=0;
        i<4;
        ++i){
          auto*h=new DragHandle({
            pts[2*i].toDouble(),pts[2*i+1].toDouble()
          },color(layer));
          h->setVisible(visible_.value(layer,true));
          h->moved=[this,layer,i](QPointF p){
            auto pts=points_[layer];
            pts[2*i]=p.x();
            pts[2*i+1]=p.y();
            points_[layer]=pts;
            updateLines(layer);
            bool ok=false;
            QPointF m=projectPixel(p.x(),p.y(),&ok);
            coord_->setText(ok?QString("%1 P%2 px=(%3,%4) → forward=%5 m, left=%6 m").arg(layer.toUpper()).arg(i+1).arg(p.x(),0,'f',1).arg(p.y(),0,'f',1).arg(m.x(),0,'f',3).arg(m.y(),0,'f',3):QString("%1 P%2 px=(%3,%4)").arg(layer).arg(i+1).arg(p.x()).arg(p.y()));
          };
          h->released=[this,layer](){
            emit pointsChanged(layer,points_[layer]);
          };
          scene_->addItem(h);
          handles_[layer]<<h;
        }
        for(int i=0;
        i<4;
        ++i){
          auto*line=scene_->addLine(0,0,0,0,QPen(color(layer),2.5));
          line->setZValue(110);
          line->setVisible(visible_.value(layer,true));
          lines_[layer]<<line;
        }
        updateLines(layer);
      }
    }
    void updateLines(const QString&layer){
      auto hs=handles_[layer];
      auto ls=lines_[layer];
      if(hs.size()!=4||ls.size()!=4)return;
      for(int i=0;
      i<4;
      ++i){
        QPointF a=hs[i]->pos(),b=hs[(i+1)%4]->pos();
        ls[i]->setLine(QLineF(a,b));
      }
    }
    QPointF projectPixel(double px,double py,bool*ok)const{
      double H[9];
      QVariantList src=points_.value("ground"),dst=per_->get("perception.ros__parameters.ground_dst_points",QVariantList{
      }).toList();
      if(!solveHomography8(src,dst,H)){
        if(ok)*ok=false;
        return{
        };
      }
      bool hOk=false;
      QPointF b=projectH(H,px,py,&hOk);
      if(!hOk){
        if(ok)*ok=false;
        return{
        };
      }
      double ox=number(per_->get("perception.ros__parameters.ground_origin_x_px",0),0),oy=number(per_->get("perception.ros__parameters.ground_origin_y_px",0),0),sx=number(per_->get("perception.ros__parameters.ground_meters_per_pixel_x",1),1),sy=number(per_->get("perception.ros__parameters.ground_meters_per_pixel_y",1),1),fo=number(per_->get("perception.ros__parameters.metric_forward_offset_m",0),0),lo=number(per_->get("perception.ros__parameters.metric_lateral_offset_m",0),0);
      if(ok)*ok=true;
      return{
        (oy-b.y())*sy+fo,(ox-b.x())*sx+lo
      };
    }
    void fitView(){
      view_->fitInView(scene_->sceneRect(),Qt::KeepAspectRatio);
    }
  };
  #include "modules/camera_reports_pages.cpp"
  #include "modules/system_overview_page.cpp"
  #include "modules/main_window.cpp"
  int main(int argc,char**argv){
    rclcpp::init(argc,argv);
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling,true);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps,true);
    int qargc=1;
    char* qargv[]={
      argv[0],nullptr
    };
    QApplication app(qargc,qargv);
    app.setApplicationName(kAppTitle);
    qRegisterMetaType<QImage>("QImage");
    qRegisterMetaType<QVariantMap>("QVariantMap");
    MainWindow w;
    w.showMaximized();
    int rc=app.exec();
    if(rclcpp::ok())rclcpp::shutdown();
    return rc;
  }
  #include "agv_gui.moc"
