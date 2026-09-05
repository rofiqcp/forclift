#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <future>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <utility>

#include <QApplication>
#include <QDateTime>
#include <QCloseEvent>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QPixmap>
#include <QMessageBox>
#include <QProcess>
#include <QStringList>
#include <QPushButton>
#include <QTabWidget>
#include <QTextStream>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <sys/types.h>
#include <unistd.h>

namespace navigation
{

static constexpr const char * kWorkspace = "/home/otomasi2/ros";
static constexpr const char * kMapDir = "/home/otomasi2/ros/src/navigation/maps";
static constexpr const char * kPointerDir = "/home/otomasi2/ros/maps";
static constexpr const char * kLatestPointer = "/home/otomasi2/ros/maps/latest_map.txt";
static constexpr const char * kPidFile = "/tmp/agv_mapping_runtime.pid";
static constexpr double kPi = 3.14159265358979323846;

static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

class MappingRosNode final : public rclcpp::Node
{
public:
  MappingRosNode()
  : Node("mapping_gui_cpp_telemetry"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_nav", sensor_qos, [this](sensor_msgs::msg::LaserScan::SharedPtr) { last_scan_ = steadyNow(); });
    lidar_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/lidar/odom", 20, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        last_lidar_ = steadyNow();
        lidar_values_ = {msg->pose.pose.position.x, msg->pose.pose.position.y,
          yawFromQuaternion(msg->pose.pose.orientation) * 180.0 / kPi};
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", sensor_qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        last_imu_ = steadyNow();
        accel_values_ = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
        gyro_values_ = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
      });
    mag_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/imu/mag", sensor_qos, [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        last_mag_ = steadyNow();
        mag_values_ = {msg->vector.x * 1e6, msg->vector.y * 1e6, msg->vector.z * 1e6};
      });
    ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 20, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        last_ekf_ = steadyNow();
        ekf_values_ = {msg->pose.pose.position.x, msg->pose.pose.position.y,
          yawFromQuaternion(msg->pose.pose.orientation) * 180.0 / kPi};
      });
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        last_map_ = steadyNow();
        latest_map_ = *msg;
        map_dirty_ = true;
      });
    stop_lidar_client_ = create_client<std_srvs::srv::Trigger>("/lidar/stop_motor");
  }

  static double steadyNow()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  bool fresh(double stamp, double max_age = 1.5) const
  {
    return stamp > 0.0 && steadyNow() - stamp < max_age;
  }

  bool requestLidarStop(double timeout_sec = 1.2)
  {
    if (!stop_lidar_client_->wait_for_service(std::chrono::milliseconds(250))) {
      return false;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = stop_lidar_client_->async_send_request(req);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(shared_from_this());
      if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
        try {
          return future.get()->success;
        } catch (...) {
          return false;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  std::optional<std::array<double, 3>> currentMapPose()
  {
    try {
      const auto tf = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
      return std::array<double, 3>{
        tf.transform.translation.x,
        tf.transform.translation.y,
        yawFromQuaternion(tf.transform.rotation)};
    } catch (...) {
      return std::nullopt;
    }
  }

  double last_scan_{0.0};
  double last_lidar_{0.0};
  double last_imu_{0.0};
  double last_mag_{0.0};
  double last_ekf_{0.0};
  double last_map_{0.0};
  std::optional<std::array<double, 3>> lidar_values_;
  std::optional<std::array<double, 3>> accel_values_;
  std::optional<std::array<double, 3>> gyro_values_;
  std::optional<std::array<double, 3>> mag_values_;
  std::optional<std::array<double, 3>> ekf_values_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_map_;
  bool map_dirty_{false};

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_lidar_client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

class MappingWindow final : public QMainWindow
{
public:
  explicit MappingWindow(std::shared_ptr<MappingRosNode> node)
  : node_(std::move(node))
  {
    setWindowTitle("AGV Mapping Control — C++");
    resize(1000, 700);
    buildUi();
    cleanupStaleRuntime();
    loadSavedSlots();
    refreshDataset();

    spin_timer_ = new QTimer(this);
    connect(spin_timer_, &QTimer::timeout, this, [this]() {
      if (!rclcpp::ok()) {       // context already shut down by the SIGINT handler
        QApplication::quit();    // stop the event loop, exit cleanly
        return;
      }
      rclcpp::spin_some(node_);
    });
    spin_timer_->start(20);

    ui_timer_ = new QTimer(this);
    connect(ui_timer_, &QTimer::timeout, this, [this]() { refreshUi(); });
    ui_timer_->start(200);
  }

  ~MappingWindow() override { shutdownRuntime(false); }

protected:
  void closeEvent(QCloseEvent * event) override
  {
    if (runtime_ && runtime_->state() != QProcess::NotRunning) {
      const auto answer = QMessageBox::question(
        this, "Mapping masih aktif",
        "Mapping masih aktif. Gunakan STOP + SAVE MAP agar map tersimpan.\nKeluar sekarang akan menghentikan sensor tanpa menyimpan map terbaru.\n\nTetap keluar?",
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
      if (answer != QMessageBox::Yes) {
        event->ignore();
        return;
      }
    }
    shutdownRuntime(false);
    event->accept();
  }

private:
  static QLabel * vectorValueLabel()
  {
    auto * label = new QLabel("0.000");
    label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    label->setStyleSheet("font-family: monospace; font-weight: 600;");
    return label;
  }

  std::array<QLabel *, 3> addVectorGroup(QVBoxLayout * parent, const QString & title, const QStringList & names)
  {
    auto * box = new QGroupBox(title);
    auto * grid = new QGridLayout(box);
    std::array<QLabel *, 3> values{};
    for (int i = 0; i < 3; ++i) {
      grid->addWidget(new QLabel(names.at(i)), i, 0);
      values[static_cast<size_t>(i)] = vectorValueLabel();
      grid->addWidget(values[static_cast<size_t>(i)], i, 1);
    }
    parent->addWidget(box);
    return values;
  }

  void buildUi()
  {
    auto * central = new QWidget(this);
    setCentralWidget(central);
    auto * root = new QHBoxLayout(central);
    root->setContentsMargins(14, 14, 14, 14);
    root->setSpacing(14);

    auto * left = new QVBoxLayout();
    auto * right = new QVBoxLayout();
    root->addLayout(left, 0);
    root->addLayout(right, 1);

    auto * title = new QLabel("AGV MAPPING — C++");
    title->setStyleSheet("font-size: 22px; font-weight: 700;");
    left->addWidget(title);

    state_label_ = new QLabel("READY — sensor OFF");
    state_label_->setStyleSheet("font-size: 15px; font-weight: 600; padding: 6px;");
    left->addWidget(state_label_);

    auto * buttons = new QHBoxLayout();
    start_btn_ = new QPushButton("START");
    stop_btn_ = new QPushButton("STOP + SAVE MAP");
    reset_btn_ = new QPushButton("RESET 3 MAP");
    for (auto * b : {start_btn_, stop_btn_, reset_btn_}) {
      b->setMinimumHeight(46);
    }
    stop_btn_->setEnabled(false);
    connect(start_btn_, &QPushButton::clicked, this, [this]() { startMapping(); });
    connect(stop_btn_, &QPushButton::clicked, this, [this]() { stopAndSave(); });
    connect(reset_btn_, &QPushButton::clicked, this, [this]() { resetDataset(); });
    buttons->addWidget(start_btn_);
    buttons->addWidget(stop_btn_);
    buttons->addWidget(reset_btn_);
    left->addLayout(buttons);

    dataset_label_ = new QLabel("Dataset SLAM: 0/3 map tersimpan");
    dataset_label_->setStyleSheet("font-weight: 600; padding: 4px;");
    left->addWidget(dataset_label_);

    auto * status_box = new QGroupBox("Status");
    auto * status_grid = new QGridLayout(status_box);
    status_lidar_ = new QLabel("OFF");
    status_imu_ = new QLabel("OFF");
    status_ekf_ = new QLabel("OFF");
    status_slam_ = new QLabel("OFF");
    const std::vector<std::pair<QString, QLabel *>> rows = {
      {"LiDAR /scan_nav", status_lidar_}, {"IMU /imu/data", status_imu_},
      {"EKF /odometry/filtered", status_ekf_}, {"SLAM /map", status_slam_}};
    for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
      status_grid->addWidget(new QLabel(rows[static_cast<size_t>(i)].first), i, 0);
      status_grid->addWidget(rows[static_cast<size_t>(i)].second, i, 1);
    }
    left->addWidget(status_box);

    lidar_labels_ = addVectorGroup(left, "LiDAR Odometry", {"X [m]", "Y [m]", "Theta [deg]"});
    accel_labels_ = addVectorGroup(left, "IMU Accelerometer", {"Ax [m/s²]", "Ay [m/s²]", "Az [m/s²]"});
    gyro_labels_ = addVectorGroup(left, "IMU Gyroscope", {"Gx [rad/s]", "Gy [rad/s]", "Gz [rad/s]"});
    mag_labels_ = addVectorGroup(left, "IMU Magnetometer", {"Mx [µT]", "My [µT]", "Mz [µT]"});
    ekf_labels_ = addVectorGroup(left, "EKF", {"X [m]", "Y [m]", "Theta [deg]"});
    left->addStretch(1);

    auto * map_title = new QLabel("SLAM Dataset — 3 Variasi Map");
    map_title->setStyleSheet("font-size: 18px; font-weight: 700;");
    right->addWidget(map_title);

    map_tabs_ = new QTabWidget();
    for (int slot = 1; slot <= 3; ++slot) {
      auto * page = new QWidget();
      auto * layout = new QVBoxLayout(page);
      auto * preview = new QLabel("Map belum tersimpan");
      preview->setAlignment(Qt::AlignCenter);
      preview->setMinimumSize(500, 480);
      preview->setStyleSheet("background:#202124; color:#ddd;");
      auto * path = new QLabel(QString("Map %1: belum tersimpan").arg(slot));
      path->setWordWrap(true);
      layout->addWidget(preview, 1);
      layout->addWidget(path);
      map_tabs_->addTab(page, QString("Map %1").arg(slot));
      previews_[static_cast<size_t>(slot - 1)] = preview;
      paths_[static_cast<size_t>(slot - 1)] = path;
    }
    right->addWidget(map_tabs_, 1);

    log_label_ = new QLabel("Runtime log: -");
    log_label_->setWordWrap(true);
    right->addWidget(log_label_);
  }

  void setStatus(QLabel * label, bool on)
  {
    label->setText(on ? "ON" : "OFF");
    label->setStyleSheet(on ? "color:#119933;font-weight:700;" : "color:#bb3333;font-weight:700;");
  }

  void setVector(const std::array<QLabel *, 3> & labels, const std::optional<std::array<double, 3>> & values)
  {
    for (int i = 0; i < 3; ++i) {
      labels[static_cast<size_t>(i)]->setText(
        values ? QString::number((*values)[static_cast<size_t>(i)], 'f', 3) : "0.000");
    }
  }

  void refreshUi()
  {
    setStatus(status_lidar_, node_->fresh(node_->last_scan_));
    setStatus(status_imu_, node_->fresh(node_->last_imu_));
    setStatus(status_ekf_, node_->fresh(node_->last_ekf_));
    setStatus(status_slam_, node_->fresh(node_->last_map_));
    setVector(lidar_labels_, node_->lidar_values_);
    setVector(accel_labels_, node_->accel_values_);
    setVector(gyro_labels_, node_->gyro_values_);
    setVector(mag_labels_, node_->mag_values_);
    setVector(ekf_labels_, node_->ekf_values_);

    if (node_->map_dirty_ && node_->latest_map_) {
      renderMap(*node_->latest_map_, active_slot_.value_or(map_tabs_->currentIndex() + 1));
      node_->map_dirty_ = false;
    }

    if (runtime_ && runtime_->state() == QProcess::NotRunning && runtime_started_) {
      const int code = runtime_->exitCode();
      runtime_started_ = false;
      if (!stopping_) {
        state_label_->setText(QString("RUNTIME STOPPED (exit=%1)").arg(code));
      }
      finishRuntimeState();
    } else if (runtime_ && runtime_->state() != QProcess::NotRunning && !stopping_) {
      if (node_->fresh(node_->last_map_)) {
        state_label_->setText("MAPPING ACTIVE");
      } else if (node_->fresh(node_->last_scan_) && node_->fresh(node_->last_imu_)) {
        state_label_->setText("SENSORS ACTIVE — menunggu SLAM /map");
      } else {
        state_label_->setText("STARTING — menunggu LiDAR + IMU + SLAM");
      }
    }
  }

  void renderMap(const nav_msgs::msg::OccupancyGrid & map, int slot)
  {
    if (slot < 1 || slot > 3 || map.info.width == 0 || map.info.height == 0) {
      return;
    }
    QImage image(static_cast<int>(map.info.width), static_cast<int>(map.info.height), QImage::Format_RGB32);
    for (uint32_t y = 0; y < map.info.height; ++y) {
      for (uint32_t x = 0; x < map.info.width; ++x) {
        const int8_t value = map.data[static_cast<size_t>(y) * map.info.width + x];
        int gray = 205;
        if (value >= 65) gray = 0;
        else if (value >= 0 && value <= 25) gray = 255;
        else if (value >= 0) gray = std::clamp(255 - static_cast<int>(value) * 255 / 100, 0, 255);
        image.setPixel(static_cast<int>(x), static_cast<int>(map.info.height - 1 - y), qRgb(gray, gray, gray));
      }
    }
    const auto pix = QPixmap::fromImage(image).scaled(
      previews_[static_cast<size_t>(slot - 1)]->size(), Qt::KeepAspectRatio, Qt::FastTransformation);
    previews_[static_cast<size_t>(slot - 1)]->setPixmap(pix);
  }

  QString slotPrefix(int slot) const { return QString("%1/map_%2").arg(kMapDir).arg(slot); }

  bool slotExists(int slot) const
  {
    return QFileInfo::exists(slotPrefix(slot) + ".yaml") && QFileInfo::exists(slotPrefix(slot) + ".pgm");
  }

  int existingCount() const
  {
    int n = 0;
    for (int i = 1; i <= 3; ++i) if (slotExists(i)) ++n;
    return n;
  }

  void loadSavedSlots()
  {
    for (int slot = 1; slot <= 3; ++slot) {
      const QString prefix = slotPrefix(slot);
      if (slotExists(slot)) {
        paths_[static_cast<size_t>(slot - 1)]->setText(QString("Map %1: %2.yaml").arg(slot).arg(prefix));
        QPixmap pix(prefix + ".pgm");
        if (!pix.isNull()) {
          previews_[static_cast<size_t>(slot - 1)]->setPixmap(
            pix.scaled(previews_[static_cast<size_t>(slot - 1)]->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
        }
      } else {
        paths_[static_cast<size_t>(slot - 1)]->setText(QString("Map %1: belum tersimpan").arg(slot));
        previews_[static_cast<size_t>(slot - 1)]->clear();
        previews_[static_cast<size_t>(slot - 1)]->setText("Map belum tersimpan");
      }
    }
  }

  void refreshDataset()
  {
    const int count = existingCount();
    dataset_label_->setText(QString("Dataset SLAM: %1/3 map tersimpan").arg(count));
    start_btn_->setEnabled(!runtime_started_);
    reset_btn_->setEnabled(!runtime_started_);
    if (!runtime_started_ && count >= 3) {
      state_label_->setText("DATASET SLAM LENGKAP — 3/3 map tersimpan");
    }
  }

  void cleanupStaleRuntime()
  {
    QFile f(kPidFile);
    if (f.open(QIODevice::ReadOnly | QIODevice::Text)) {
      bool ok = false;
      const qint64 pid = QString::fromUtf8(f.readAll()).trimmed().toLongLong(&ok);
      f.close();
      if (ok && pid > 1) {
        waitProcessGroupGone(static_cast<pid_t>(pid));
      }
      QFile::remove(kPidFile);
    }

    // Belt-and-braces: a previous session may have left sensor-owner children or
    // an in-flight CP210x recovery helper behind even when the pidfile is gone.
    // Only the narrow sensor/runtime executables and the exact recovery helper
    // command are targeted; unrelated processes are never touched.
    killPattern("ros2 launch navigation mapping_runtime.launch.py");
    killPattern("/navigation/lib/navigation/imu_node");
    killPattern("/navigation/lib/navigation/lidar_node");
    killPattern("/navigation/lib/navigation/scan_self_filter");
    killPattern("/navigation/lib/navigation/lidar_safety_health.py");
    killPattern("/navigation/lib/navigation/resolve_usb_roles.py");
    killPattern("/navigation/lib/navigation/serial_transport_ready_gate.py");
    killPattern("^sudo -n /usr/local/sbin/agv-sensor-recover");
    killPattern("^/usr/local/sbin/agv-sensor-recover");
  }

  void killPattern(const char * pattern)
  {
    const QString pat = QString::fromLatin1(pattern);
    QProcess::execute(QStringLiteral("pkill"), QStringList{}
      << QStringLiteral("-TERM") << QStringLiteral("-f")
      << QStringLiteral("--") << pat);

    // LiDAR shutdown sends A5 65 and powers the motor down before closing the
    // CP210x descriptor. Give stale owners time to complete that sequence;
    // immediate TERM->KILL can leave a visible ttyUSB node in EIO state.
    for (int i = 0; i < 30; ++i) {
      QProcess probe;
      probe.start(QStringLiteral("pgrep"), QStringList{}
        << QStringLiteral("-f") << QStringLiteral("--") << pat);
      if (!probe.waitForFinished(500) || probe.exitStatus() != QProcess::NormalExit) {
        break;
      }
      if (probe.exitCode() != 0) {
        return;
      }
      QThread::msleep(100);
    }

    QProcess::execute(QStringLiteral("pkill"), QStringList{}
      << QStringLiteral("-KILL") << QStringLiteral("-f")
      << QStringLiteral("--") << pat);
  }

  void waitProcessGroupGone(pid_t leader)
  {
    if (leader <= 1 || ::kill(-leader, 0) != 0) {
      return;
    }
    ::kill(-leader, SIGINT);
    for (int i = 0; i < 35; ++i) {
      if (::kill(-leader, 0) != 0) {
        return;
      }
      QThread::msleep(100);
    }
    ::kill(-leader, SIGTERM);
    for (int i = 0; i < 15; ++i) {
      if (::kill(-leader, 0) != 0) {
        return;
      }
      QThread::msleep(100);
    }
    ::kill(-leader, SIGKILL);
    for (int i = 0; i < 10; ++i) {
      if (::kill(-leader, 0) != 0) {
        return;
      }
      QThread::msleep(100);
    }
  }

  void startMapping()
  {
    // Auto-recover from a stale runtime_started_ flag: if the flag says a
    // runtime is running but the process is actually gone, clear it so START
    // (and RESET) are never blocked by a previous session that didn't exit
    // cleanly.
    if (runtime_started_ && (!runtime_ || runtime_->state() == QProcess::NotRunning)) {
      runtime_started_ = false;
      stopping_ = false;
      active_slot_.reset();
    }
    if (runtime_started_) return;
    // START records into the slot of the currently selected tab (Map 1/2/3),
    // not the first empty slot. Overwriting an existing map on that tab is
    // allowed; use RESET 3 MAP to clear all three first if a clean dataset
    // is desired.
    const int slot = map_tabs_->currentIndex() + 1;
    if (slot < 1 || slot > 3) return;
    active_slot_ = slot;
    QDir().mkpath(kMapDir);
    QDir().mkpath(QString("%1/log").arg(kWorkspace));

    runtime_ = std::make_unique<QProcess>();
    runtime_->setProcessChannelMode(QProcess::MergedChannels);
    const QString log_path = QString("%1/log/mapping_gui_cpp_%2.txt").arg(kWorkspace).arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    runtime_->setStandardOutputFile(log_path, QIODevice::Append);
    runtime_->setWorkingDirectory(kWorkspace);
    runtime_->start("/usr/bin/setsid", QStringList{"ros2", "launch", "navigation", "mapping_runtime.launch.py", "enable_rviz:=false"});
    if (!runtime_->waitForStarted(4000)) {
      QMessageBox::critical(this, "START gagal", runtime_->errorString());
      runtime_.reset();
      active_slot_.reset();
      return;
    }
    runtime_started_ = true;
    stopping_ = false;
    QFile pidfile(kPidFile);
    if (pidfile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
      pidfile.write(QByteArray::number(runtime_->processId()));
      pidfile.write("\n");
    }
    log_label_->setText("Runtime log: " + log_path);
    start_btn_->setEnabled(false);
    stop_btn_->setEnabled(true);
    reset_btn_->setEnabled(false);
    state_label_->setText(QString("STARTING MAP %1/3 — menunggu LiDAR + IMU + SLAM").arg(slot));
  }

  bool writeMapFiles(int slot, QString & error)
  {
    if (!node_->latest_map_) {
      error = "Belum ada pesan /map yang bisa disimpan.";
      return false;
    }
    const auto & map = *node_->latest_map_;
    if (map.info.width == 0 || map.info.height == 0 || map.data.size() != static_cast<size_t>(map.info.width) * map.info.height) {
      error = "Pesan /map memiliki ukuran/data tidak valid.";
      return false;
    }

    QDir().mkpath(kMapDir);
    const QString prefix = slotPrefix(slot);
    const QString pgm_final = prefix + ".pgm";
    const QString yaml_final = prefix + ".yaml";
    const QString pgm_tmp = pgm_final + ".tmp";
    const QString yaml_tmp = yaml_final + ".tmp";
    QFile::remove(pgm_tmp); QFile::remove(yaml_tmp);

    QFile pgm(pgm_tmp);
    if (!pgm.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
      error = "Tidak dapat menulis " + pgm_tmp;
      return false;
    }
    QByteArray header = QByteArray("P5\n# CREATOR: navigation mapping_gui_cpp\n") +
      QByteArray::number(map.info.width) + " " + QByteArray::number(map.info.height) + "\n255\n";
    pgm.write(header);
    QByteArray row;
    row.resize(static_cast<int>(map.info.width));
    for (int y = static_cast<int>(map.info.height) - 1; y >= 0; --y) {
      for (uint32_t x = 0; x < map.info.width; ++x) {
        const int8_t occ = map.data[static_cast<size_t>(y) * map.info.width + x];
        unsigned char pixel = 205;
        if (occ >= 65) pixel = 0;
        else if (occ >= 0 && occ <= 25) pixel = 254;
        row[static_cast<int>(x)] = static_cast<char>(pixel);
      }
      pgm.write(row);
    }
    pgm.flush(); pgm.close();

    QFile yaml(yaml_tmp);
    if (!yaml.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
      QFile::remove(pgm_tmp);
      error = "Tidak dapat menulis " + yaml_tmp;
      return false;
    }
    const double yaw = yawFromQuaternion(map.info.origin.orientation);
    QTextStream ys(&yaml);
    ys.setRealNumberNotation(QTextStream::FixedNotation);
    ys.setRealNumberPrecision(6);
    ys << "image: map_" << slot << ".pgm\n";
    ys << "mode: trinary\n";
    ys << "resolution: " << map.info.resolution << "\n";
    ys << "origin: [" << map.info.origin.position.x << ", " << map.info.origin.position.y << ", " << yaw << "]\n";
    ys << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
    yaml.flush(); yaml.close();

    QFile::remove(pgm_final); QFile::remove(yaml_final);
    if (!QFile::rename(pgm_tmp, pgm_final) || !QFile::rename(yaml_tmp, yaml_final)) {
      error = "Atomic rename hasil map gagal.";
      return false;
    }

    QDir().mkpath(kPointerDir);
    const QString pointer_tmp = QString(kLatestPointer) + ".tmp";
    QFile pointer(pointer_tmp);
    if (!pointer.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
      error = "Map tersimpan tetapi latest_map.txt gagal ditulis.";
      return false;
    }
    pointer.write(QFileInfo(yaml_final).absoluteFilePath().toUtf8());
    pointer.write("\n");
    pointer.flush(); pointer.close();
    QFile::remove(kLatestPointer);
    if (!QFile::rename(pointer_tmp, kLatestPointer)) {
      error = "Map tersimpan tetapi latest_map.txt gagal di-commit.";
      return false;
    }

    writeAutoPose(prefix, yaml_final, pgm_final);
    return true;
  }

  void writeAutoPose(const QString & prefix, const QString & yaml_path, const QString & pgm_path)
  {
    const auto pose = node_->currentMapPose();
    if (!pose) return;
    QCryptographicHash hash(QCryptographicHash::Sha256);
    for (const QString & path : {yaml_path, pgm_path}) {
      QFile f(path);
      if (!f.open(QIODevice::ReadOnly)) return;
      while (!f.atEnd()) hash.addData(f.read(1024 * 1024));
    }
    const QByteArray digest = hash.result().toHex();
    QFile sidecar(prefix + ".autopose.json.tmp");
    if (!sidecar.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) return;
    QTextStream s(&sidecar);
    s.setRealNumberNotation(QTextStream::FixedNotation);
    s.setRealNumberPrecision(6);
    s << "{\n"
      << "  \"schema_version\": 1,\n"
      << "  \"map_yaml\": \"" << QFileInfo(yaml_path).absoluteFilePath() << "\",\n"
      << "  \"map_image\": \"" << QFileInfo(pgm_path).absoluteFilePath() << "\",\n"
      << "  \"map_sha256\": \"" << digest << "\",\n"
      << "  \"frame_id\": \"map\",\n"
      << "  \"child_frame_id\": \"base_footprint\",\n"
      << "  \"x\": " << (*pose)[0] << ",\n"
      << "  \"y\": " << (*pose)[1] << ",\n"
      << "  \"yaw\": " << (*pose)[2] << "\n"
      << "}\n";
    sidecar.flush(); sidecar.close();
    QFile::remove(prefix + ".autopose.json");
    QFile::rename(prefix + ".autopose.json.tmp", prefix + ".autopose.json");
  }

  void stopAndSave()
  {
    if (!runtime_started_ || !active_slot_) return;
    stop_btn_->setEnabled(false);
    state_label_->setText("SAVING MAP — runtime tetap hidup sampai commit");
    QString error;
    if (!writeMapFiles(*active_slot_, error)) {
      stop_btn_->setEnabled(true);
      state_label_->setText("SAVE FAILED — mapping tetap aktif");
      QMessageBox::critical(this, "Map gagal disimpan", error);
      return;
    }
    state_label_->setText(QString("MAP %1 COMMITTED — stopping LiDAR + IMU").arg(*active_slot_));
    renderMap(*node_->latest_map_, *active_slot_);
    stopping_ = true;
    node_->requestLidarStop(1.2);
    shutdownRuntime(true);
    loadSavedSlots();
    refreshDataset();
  }

  void shutdownRuntime(bool graceful)
  {
    if (!runtime_ || runtime_->state() == QProcess::NotRunning) {
      finishRuntimeState();
      return;
    }
    const qint64 pid = runtime_->processId();
    if (pid > 1) {
      ::kill(-static_cast<pid_t>(pid), graceful ? SIGINT : SIGTERM);
    }
    if (!runtime_->waitForFinished(graceful ? 3500 : 1500)) {
      if (pid > 1) ::kill(-static_cast<pid_t>(pid), SIGTERM);
      if (!runtime_->waitForFinished(1500) && pid > 1) {
        ::kill(-static_cast<pid_t>(pid), SIGKILL);
        runtime_->waitForFinished(800);
      }
    }
    finishRuntimeState();
  }

  void finishRuntimeState()
  {
    QFile::remove(kPidFile);
    runtime_started_ = false;
    stopping_ = false;
    runtime_.reset();
    active_slot_.reset();
    stop_btn_->setEnabled(false);
    refreshDataset();
    if (existingCount() < 3) {
      state_label_->setText("STOPPED — sensor OFF");
    }
  }

  void resetDataset()
  {
    if (QMessageBox::question(this, "Reset dataset", "Hapus Map 1, Map 2, dan Map 3?", QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
      return;
    }
    // If a mapping runtime is (or appears to be) still active, stop it cleanly
    // first so the dataset can be reset and a fresh mapping started afterwards.
    if (runtime_started_ || (runtime_ && runtime_->state() != QProcess::NotRunning)) {
      state_label_->setText("RESET — menghentikan mapping aktif...");
      stopping_ = true;
      if (node_) node_->requestLidarStop(1.2);
      shutdownRuntime(true);
    }
    // Force-clear any orphaned runtime/sensor processes left behind by a
    // previous session so START is never blocked by a stale runtime_started_ flag.
    cleanupStaleRuntime();
    runtime_started_ = false;
    stopping_ = false;
    runtime_.reset();
    active_slot_.reset();
    if (node_) {
      node_->latest_map_.reset();
      node_->map_dirty_ = false;
      node_->last_map_ = 0.0;
      node_->last_scan_ = 0.0;
      node_->last_imu_ = 0.0;
      node_->last_ekf_ = 0.0;
    }
    static const QStringList extensions{
      QStringLiteral(".yaml"),
      QStringLiteral(".yml"),
      QStringLiteral(".pgm"),
      QStringLiteral(".png"),
      QStringLiteral(".autopose.json")
    };
    for (int slot = 1; slot <= 3; ++slot) {
      const QString p = slotPrefix(slot);
      for (const QString & ext : extensions) {
        QFile::remove(p + ext);
      }
    }
    QFile::remove(kLatestPointer);
    loadSavedSlots();
    refreshDataset();
    state_label_->setText("READY — dataset SLAM di-reset | sensor OFF");
  }

  std::shared_ptr<MappingRosNode> node_;
  QTimer * spin_timer_{nullptr};
  QTimer * ui_timer_{nullptr};
  std::unique_ptr<QProcess> runtime_;
  bool runtime_started_{false};
  bool stopping_{false};
  std::optional<int> active_slot_;

  QLabel * state_label_{nullptr};
  QLabel * dataset_label_{nullptr};
  QLabel * status_lidar_{nullptr};
  QLabel * status_imu_{nullptr};
  QLabel * status_ekf_{nullptr};
  QLabel * status_slam_{nullptr};
  QLabel * log_label_{nullptr};
  QPushButton * start_btn_{nullptr};
  QPushButton * stop_btn_{nullptr};
  QPushButton * reset_btn_{nullptr};
  QTabWidget * map_tabs_{nullptr};
  std::array<QLabel *, 3> previews_{};
  std::array<QLabel *, 3> paths_{};
  std::array<QLabel *, 3> lidar_labels_{};
  std::array<QLabel *, 3> accel_labels_{};
  std::array<QLabel *, 3> gyro_labels_{};
  std::array<QLabel *, 3> mag_labels_{};
  std::array<QLabel *, 3> ekf_labels_{};
};

}  // namespace navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int qt_argc = 1;
  char * qt_argv[] = {argv[0], nullptr};
  QApplication app(qt_argc, qt_argv);
  auto node = std::make_shared<navigation::MappingRosNode>();
  navigation::MappingWindow window(node);
  window.show();
  const int rc = app.exec();
  rclcpp::shutdown();
  return rc;
}
