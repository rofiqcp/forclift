#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <QBuffer>
#include <QByteArray>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHostAddress>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QList>
#include <QMap>
#include <QPointer>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSet>
#include <QString>
#include <QStringList>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include <QUrl>
#include <QUrlQuery>

#include <action_msgs/srv/cancel_goal.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

#include "agv_experiment_catalog.hpp"

using namespace std::chrono_literals;

namespace {
constexpr double kPi = 3.14159265358979323846;

double nowMs() { return static_cast<double>(QDateTime::currentMSecsSinceEpoch()); }

double yawFromQuat(double x, double y, double z, double w) {
  const double siny = 2.0 * (w * z + x * y);
  const double cosy = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny, cosy);
}

QJsonValue scalarFromString(const QString &value) {
  const QString s = value.trimmed();
  const QString lower = s.toLower();
  if (lower == "true") return true;
  if (lower == "false") return false;
  bool okInt = false;
  const qlonglong integer = s.toLongLong(&okInt);
  if (okInt && !s.contains('.') && !s.contains('e', Qt::CaseInsensitive)) return static_cast<double>(integer);
  bool okDouble = false;
  const double number = s.toDouble(&okDouble);
  if (okDouble && std::isfinite(number)) return number;
  return s;
}

QJsonValue yamlToJson(const YAML::Node &node) {
  if (!node || node.IsNull()) return QJsonValue();
  if (node.IsScalar()) return scalarFromString(QString::fromStdString(node.Scalar()));
  if (node.IsSequence()) {
    QJsonArray array;
    for (const auto &item : node) array.append(yamlToJson(item));
    return array;
  }
  if (node.IsMap()) {
    QJsonObject object;
    for (auto it = node.begin(); it != node.end(); ++it) {
      object.insert(QString::fromStdString(it->first.as<std::string>()), yamlToJson(it->second));
    }
    return object;
  }
  return QJsonValue();
}

QString packageConfigDir(const QString &packageName, const char *envName) {
  const QByteArray envValue = qgetenv(envName);
  if (!envValue.trimmed().isEmpty()) {
    const QString path = QDir::cleanPath(QDir::home().absoluteFilePath(QString::fromUtf8(envValue)));
    if (QDir(path).exists()) return path;
    const QString expanded = QString::fromUtf8(envValue).replace("~", QDir::homePath());
    if (QDir(expanded).exists()) return QDir::cleanPath(expanded);
  }
  try {
    const QString share = QString::fromStdString(ament_index_cpp::get_package_share_directory(packageName.toStdString()));
    const QString marker = "/install/";
    const int idx = share.indexOf(marker);
    if (idx > 0) {
      const QString workspace = share.left(idx);
      const QString sourceConfig = workspace + "/src/" + packageName + "/config";
      if (QDir(sourceConfig).exists()) return sourceConfig;
    }
    const QString installed = share + "/config";
    if (QDir(installed).exists()) return installed;
  } catch (...) {
  }
  return QString();
}

QMap<QString, QString> configCandidates() {
  const QString nav = packageConfigDir("navigation", "AGV_CONFIG_DIR");
  const QString esc = packageConfigDir("esc", "AGV_ESC_CONFIG_DIR");
  const QString per = packageConfigDir("yolo_obstacle_detection_ros2", "AGV_PERCEPTION_CONFIG_DIR");
  return {
      {"vehicle", nav + "/vehicle_geometry.yaml"},
      {"navigation_core", nav + "/autonomy_health.yaml"},
      {"nav2", nav + "/nav2_ackermann.yaml"},
      {"ekf", nav + "/ekf_autonomous.yaml"},
      {"localization", nav + "/localization_startup.yaml"},
      {"localization_timing", nav + "/localization_timing.yaml"},
      {"imu", nav + "/imu.yaml"},
      {"lidar", nav + "/lidar.yaml"},
      {"lidar_safety", nav + "/lidar_safety.yaml"},
      {"slam", nav + "/slam_toolbox.yaml"},
      {"hector", nav + "/hector_autonomous.yaml"},
      {"runtime_schema", nav + "/runtime_schema.yaml"},
      {"manual_motion", nav + "/manual_motion_health.yaml"},
      {"gui", nav + "/gui/interface.yaml"},
      {"esc", esc + "/ackermann_2_board.yaml"},
      {"esc_mux", esc + "/esc_mux.yaml"},
      {"teleop", esc + "/keyboard_teleop.yaml"},
      {"winch", esc + "/winch.yaml"},
      {"perception", per + "/yolo_detection.yaml"},
      {"camera", per + "/camera_v4l2.yaml"},
      {"alignment", per + "/alignment_realtime.yaml"},
  };
}

QJsonObject loadConfigSnapshot() {
  QJsonObject root;
  QJsonObject paths;
  QJsonObject files;
  paths["navigation"] = packageConfigDir("navigation", "AGV_CONFIG_DIR");
  paths["esc"] = packageConfigDir("esc", "AGV_ESC_CONFIG_DIR");
  paths["perception"] = packageConfigDir("perception", "AGV_PERCEPTION_CONFIG_DIR");

  const QMap<QString, QString> candidates = configCandidates();
  for (auto it = candidates.cbegin(); it != candidates.cend(); ++it) {
    if (it.value().startsWith('/') && QFileInfo::exists(it.value())) {
      try {
        QJsonObject entry;
        entry["path"] = it.value();
        entry["data"] = yamlToJson(YAML::LoadFile(it.value().toStdString()));
        files[it.key()] = entry;
      } catch (const std::exception &e) {
        files[it.key()] = QJsonObject{{"path", it.value()}, {"error", QString::fromUtf8(e.what())}};
      }
    }
  }
  root["paths"] = paths;
  root["files"] = files;
  root["model_expected"] = "/home/sirobo/ros/models/yolopv2.pt";
  root["generated_at_ms"] = nowMs();
  return root;
}


QString jsonScalarInlineYaml(const QJsonValue &value) {
  if (value.isNull() || value.isUndefined()) return QStringLiteral("~");
  if (value.isBool()) return value.toBool() ? QStringLiteral("true") : QStringLiteral("false");
  if (value.isDouble()) return QString::number(value.toDouble(), 'g', 16);
  if (value.isString()) {
    QJsonArray wrapper{value};
    QByteArray encoded = QJsonDocument(wrapper).toJson(QJsonDocument::Compact);
    return QString::fromUtf8(encoded.mid(1, encoded.size() - 2));
  }
  if (value.isArray()) return QString::fromUtf8(QJsonDocument(value.toArray()).toJson(QJsonDocument::Compact));
  return QString::fromUtf8(QJsonDocument(value.toObject()).toJson(QJsonDocument::Compact));
}

bool patchExistingYamlScalar(const QString &filePath, const QString &yamlPath, const QJsonValue &value, QString *error) {
  QFile in(filePath);
  if (!in.open(QIODevice::ReadOnly | QIODevice::Text)) {
    if (error) *error = QStringLiteral("Tidak dapat membaca YAML");
    return false;
  }
  QStringList lines = QString::fromUtf8(in.readAll()).split('\n');
  in.close();
  const QStringList wanted = yamlPath.split('.', Qt::SkipEmptyParts);
  QVector<QPair<int, QString>> stack;
  int target = -1;
  for (int i = 0; i < lines.size(); ++i) {
    const QString raw = lines[i];
    const QString stripped = raw.trimmed();
    if (stripped.isEmpty() || stripped.startsWith('#') || stripped.startsWith('-') || !stripped.contains(':')) continue;
    const int indent = raw.size() - raw.trimmed().size();
    QString key = stripped.section(':', 0, 0).trimmed();
    key.remove('"'); key.remove('\'');
    while (!stack.isEmpty() && indent <= stack.last().first) stack.removeLast();
    QStringList full;
    for (const auto &entry : stack) full << entry.second;
    full << key;
    if (full == wanted) { target = i; break; }
    stack.push_back({indent, key});
  }
  if (target < 0) {
    if (error) *error = QStringLiteral("Path YAML tidak ditemukan untuk patch preserving-comment");
    return false;
  }
  const QString raw = lines[target];
  const int colon = raw.indexOf(':');
  if (colon < 0) return false;
  const QString prefix = raw.left(colon + 1);
  const QString rest = raw.mid(colon + 1);
  QString comment;
  bool quote = false;
  int hash = -1;
  for (int j = 0; j < rest.size(); ++j) {
    if (rest[j] == '"' && (j == 0 || rest[j - 1] != '\\')) quote = !quote;
    if (rest[j] == '#' && !quote) { hash = j; break; }
  }
  if (hash >= 0) comment = QStringLiteral(" ") + rest.mid(hash).trimmed();
  lines[target] = prefix + QStringLiteral(" ") + jsonScalarInlineYaml(value) + comment;
  const QByteArray candidate = lines.join('\n').toUtf8();
  try { YAML::Load(candidate.constData()); }
  catch (const std::exception &e) {
    if (error) *error = QStringLiteral("Patch menghasilkan YAML invalid: ") + QString::fromUtf8(e.what());
    return false;
  }
  QSaveFile out(filePath);
  if (!out.open(QIODevice::WriteOnly | QIODevice::Text)) {
    if (error) *error = QStringLiteral("Tidak dapat membuka YAML untuk atomic save");
    return false;
  }
  out.write(candidate);
  if (!out.commit()) {
    if (error) *error = QStringLiteral("Atomic YAML commit gagal");
    return false;
  }
  return true;
}

QJsonValue yamlPathValue(YAML::Node node, const QStringList &parts, int index = 0) {
  if (index >= parts.size() || !node) return yamlToJson(node);
  bool numeric = false;
  const int seqIndex = parts[index].toInt(&numeric);
  if (numeric && node.IsSequence()) {
    if (seqIndex < 0 || static_cast<size_t>(seqIndex) >= node.size()) return QJsonValue();
    return yamlPathValue(node[static_cast<size_t>(seqIndex)], parts, index + 1);
  }
  if (!node.IsMap()) return QJsonValue();
  return yamlPathValue(node[parts[index].toStdString()], parts, index + 1);
}

bool setYamlValueAtomic(const QString &fileKey, const QString &yamlPath, const QJsonValue &input,
                        QString *message, QJsonValue *savedValue = nullptr) {
  const QMap<QString, QString> candidates = configCandidates();
  if (!candidates.contains(fileKey) || yamlPath.trimmed().isEmpty()) {
    if (message) *message = QStringLiteral("file_key/path YAML tidak diizinkan");
    return false;
  }
  const QString filePath = candidates.value(fileKey);
  if (!QFileInfo(filePath).isFile()) {
    if (message) *message = QStringLiteral("File YAML tidak ditemukan: ") + filePath;
    return false;
  }
  const QStringList parts = yamlPath.split('.', Qt::SkipEmptyParts);
  if (parts.isEmpty()) return false;
  try {
    YAML::Node root = YAML::LoadFile(filePath.toStdString());
    YAML::Node current = root;
    for (int i = 0; i < parts.size() - 1; ++i) {
      bool numeric = false;
      const int seqIndex = parts[i].toInt(&numeric);
      if (numeric && current.IsSequence()) {
        if (seqIndex < 0 || static_cast<size_t>(seqIndex) >= current.size()) {
          if (message) *message = QStringLiteral("Index YAML di luar batas");
          return false;
        }
        current = current[static_cast<size_t>(seqIndex)];
      } else {
        if (!current.IsMap() || !current[parts[i].toStdString()]) {
          if (message) *message = QStringLiteral("Path YAML tidak ditemukan: ") + yamlPath;
          return false;
        }
        current = current[parts[i].toStdString()];
      }
    }
    bool lastNumeric = false;
    const int lastIndex = parts.last().toInt(&lastNumeric);
    QString patchPath = yamlPath;
    QJsonValue patchValue = input;
    if (lastNumeric && current.IsSequence()) {
      if (lastIndex < 0 || static_cast<size_t>(lastIndex) >= current.size()) {
        if (message) *message = QStringLiteral("Index YAML di luar batas");
        return false;
      }
      YAML::Node replacement = YAML::Load(jsonScalarInlineYaml(input).toStdString());
      current[static_cast<size_t>(lastIndex)] = replacement;
      patchPath = parts.mid(0, parts.size() - 1).join('.');
      patchValue = yamlToJson(current);
    } else {
      if (!current.IsMap() || !current[parts.last().toStdString()]) {
        if (message) *message = QStringLiteral("Leaf YAML tidak ditemukan: ") + yamlPath;
        return false;
      }
      YAML::Node replacement = YAML::Load(jsonScalarInlineYaml(input).toStdString());
      current[parts.last().toStdString()] = replacement;
    }
    const QString backup = filePath + QStringLiteral(".web.bak.") + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
    if (!QFile::copy(filePath, backup)) {
      if (message) *message = QStringLiteral("Gagal membuat backup YAML");
      return false;
    }
    QString patchError;
    if (!patchExistingYamlScalar(filePath, patchPath, patchValue, &patchError)) {
      QFile::remove(filePath);
      QFile::copy(backup, filePath);
      if (message) *message = patchError;
      return false;
    }
    YAML::Node verified = YAML::LoadFile(filePath.toStdString());
    const QJsonValue actual = yamlPathValue(verified, parts);
    if (savedValue) *savedValue = actual;
    if (message) *message = QStringLiteral("YAML tersimpan atomik; backup: ") + backup +
                            QStringLiteral(". Restart/lifecycle reload mungkin diperlukan.");
    return true;
  } catch (const std::exception &e) {
    if (message) *message = QString::fromUtf8(e.what());
    return false;
  }
}

QString csvEscape(QString value) {
  value.replace('"', QStringLiteral("\"\""));
  return QStringLiteral("\"") + value + QStringLiteral("\"");
}

void flattenJson(const QString &prefix, const QJsonValue &value, QMap<QString, QString> &out, int depth = 0) {
  if (depth > 4) {
    if (value.isObject()) out[prefix] = QString::fromUtf8(QJsonDocument(value.toObject()).toJson(QJsonDocument::Compact));
    else if (value.isArray()) out[prefix] = QString::fromUtf8(QJsonDocument(value.toArray()).toJson(QJsonDocument::Compact));
    return;
  }
  if (value.isObject()) {
    const QJsonObject object = value.toObject();
    for (auto it = object.constBegin(); it != object.constEnd(); ++it) {
      const QString key = prefix.isEmpty() ? it.key() : prefix + QStringLiteral(".") + it.key();
      flattenJson(key, it.value(), out, depth + 1);
    }
    return;
  }
  if (value.isArray()) {
    out[prefix] = QString::fromUtf8(QJsonDocument(value.toArray()).toJson(QJsonDocument::Compact));
  } else if (value.isBool()) out[prefix] = value.toBool() ? QStringLiteral("true") : QStringLiteral("false");
  else if (value.isDouble()) out[prefix] = QString::number(value.toDouble(), 'g', 16);
  else if (value.isString()) out[prefix] = value.toString();
  else out[prefix] = QString();
}

QString safeStem(QString value) {
  value = value.trimmed().toLower();
  value.replace(QRegularExpression(QStringLiteral("[^a-z0-9._-]+")), QStringLiteral("_"));
  while (value.contains(QStringLiteral("__"))) value.replace(QStringLiteral("__"), QStringLiteral("_"));
  return value.left(100).trimmed();
}

QJsonObject parseJsonOrKv(const QString &raw) {
  QJsonParseError error{};
  const QJsonDocument doc = QJsonDocument::fromJson(raw.toUtf8(), &error);
  if (error.error == QJsonParseError::NoError) {
    if (doc.isObject()) return doc.object();
    if (doc.isArray()) return QJsonObject{{"items", doc.array()}, {"raw", raw}};
  }
  QJsonObject out;
  const QStringList parts = raw.split(QRegularExpression("[;,\\n]+"), Qt::SkipEmptyParts);
  for (const QString &part : parts) {
    int sep = part.indexOf('=');
    if (sep <= 0) sep = part.indexOf(':');
    if (sep <= 0) continue;
    const QString key = part.left(sep).trimmed();
    const QString value = part.mid(sep + 1).trimmed();
    if (!key.isEmpty()) out[key] = scalarFromString(value);
  }
  // Runtime camera/YOLO status is intentionally compact and uses whitespace
  // between key=value tokens, e.g. "backend=... fps=28.4 total_ms=...".
  // Preserve the legacy delimiter parser above, then add token extraction so
  // performance fields (especially fps/measured_fps) remain individually live.
  const QRegularExpression token(
      QStringLiteral("(?:^|\\s)([A-Za-z_][A-Za-z0-9_-]*)=([^\\s;,]+)"));
  auto matches = token.globalMatch(raw);
  while (matches.hasNext()) {
    const auto match = matches.next();
    out[match.captured(1)] = scalarFromString(match.captured(2));
  }
  out["raw"] = raw;
  return out;
}

QByteArray mimeTypeForPath(const QString &path) {
  if (path.endsWith(".html")) return "text/html; charset=utf-8";
  if (path.endsWith(".css")) return "text/css; charset=utf-8";
  if (path.endsWith(".js")) return "application/javascript; charset=utf-8";
  if (path.endsWith(".json")) return "application/json; charset=utf-8";
  if (path.endsWith(".svg")) return "image/svg+xml";
  if (path.endsWith(".png")) return "image/png";
  if (path.endsWith(".jpg") || path.endsWith(".jpeg")) return "image/jpeg";
  return "application/octet-stream";
}

QJsonObject experimentCatalogJson() {
  QJsonObject root;
  for (const QString & subsystem : {QStringLiteral("navigation"), QStringLiteral("perception"), QStringLiteral("steering")}) {
    QJsonArray entries;
    for (const ExperimentSpec &spec : buildExperimentCatalog(subsystem)) {
      QJsonObject item;
      item["subsystem"] = spec.subsystem;
      item["groupId"] = spec.groupId;
      item["groupTitle"] = spec.groupTitle;
      item["id"] = spec.id;
      item["section"] = spec.section;
      QJsonArray tableNames;
      for (const QString &name : spec.tableNames) tableNames.append(name);
      item["tableNames"] = tableNames;
      QJsonArray tableColumns;
      for (const QStringList &columns : spec.tableColumns) {
        QJsonArray row;
        for (const QString &column : columns) row.append(column);
        tableColumns.append(row);
      }
      item["tableColumns"] = tableColumns;
      QJsonArray graphCaptions;
      for (const QString &caption : spec.graphCaptions) graphCaptions.append(caption);
      item["graphCaptions"] = graphCaptions;
      QJsonObject liveSeries;
      for (auto it = spec.liveSeries.cbegin(); it != spec.liveSeries.cend(); ++it) liveSeries[it.key()] = it.value();
      item["liveSeries"] = liveSeries;
      QJsonArray parameters;
      for (const ExperimentParameterField &field : spec.parameterFields) {
        parameters.append(QJsonObject{
            {"key", field.key}, {"label", field.label}, {"kind", field.kind},
            {"yamlFileKey", field.yamlFileKey}, {"yamlPath", field.yamlPath},
            {"placeholder", field.placeholder}, {"lockedValue", field.lockedValue},
            {"isGroundTruth", field.isGroundTruth}, {"locked", field.locked}});
      }
      item["parameterFields"] = parameters;
      QJsonArray graphs;
      for (const ExperimentGraphSpec &graph : spec.graphs) {
        QJsonArray series;
        for (const QString &s : graph.series) series.append(s);
        graphs.append(QJsonObject{{"type", graph.type}, {"series", series},
                                  {"xSeries", graph.xSeries}, {"ySeries", graph.ySeries}});
      }
      item["graphs"] = graphs;
      entries.append(item);
    }
    root[subsystem] = entries;
  }
  return root;
}

}  // namespace

class WebRosBridge {
 public:
  WebRosBridge() {
    node_ = std::make_shared<rclcpp::Node>("agv_web_gui");
    bindAddress_ = QString::fromStdString(node_->declare_parameter<std::string>("bind_address", "127.0.0.1"));
    const auto configuredPort = node_->declare_parameter<std::int64_t>("port", 5005);
    if (configuredPort < 1 || configuredPort > 65535) {
      throw std::invalid_argument("web port must be within 1..65535");
    }
    port_ = static_cast<int>(configuredPort);
    readOnly_ = node_->declare_parameter<bool>("read_only", false);
    cameraJpegFps_ = std::clamp(node_->declare_parameter<double>("camera_jpeg_fps", 5.0), 0.5, 12.0);
    setupRos();
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
    executor_->add_node(node_);
    thread_ = std::thread([this]() {
      while (rclcpp::ok() && !stop_.load()) executor_->spin_once(100ms);
    });
    update("server", QJsonObject{{"ros", true}, {"read_only", readOnly_}, {"port", port_},
                                  {"bind_address", bindAddress_}, {"started_at_ms", nowMs()}});
  }

  ~WebRosBridge() {
    stop_.store(true);
    if (executor_) executor_->cancel();
    if (thread_.joinable()) thread_.join();
    if (executor_ && node_) executor_->remove_node(node_);
  }

  QString bindAddress() const { return bindAddress_; }
  int port() const { return port_; }
  bool readOnly() const { return readOnly_; }

  QJsonObject snapshot() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    QJsonObject copy = state_;
    copy["__updated"] = updated_;
    copy["__server_time_ms"] = nowMs();
    return copy;
  }

  QJsonObject takeDelta() {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (dirty_.isEmpty()) return {};
    QJsonObject delta;
    QJsonObject timestamps;
    for (const QString &key : dirty_) {
      delta[key] = state_.value(key);
      timestamps[key] = updated_.value(key);
    }
    dirty_.clear();
    delta["__updated"] = timestamps;
    delta["__server_time_ms"] = nowMs();
    return delta;
  }

  QByteArray cameraJpeg() const {
    std::lock_guard<std::mutex> lock(mediaMutex_);
    return cameraJpeg_;
  }

  QByteArray mapPng() const {
    std::lock_guard<std::mutex> lock(mediaMutex_);
    return mapPng_;
  }

  bool publishGoal(double x, double y, double yawRad, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yawRad)) {
      if (message) *message = "Koordinat goal tidak valid";
      return false;
    }
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = node_->now();
    goal.header.frame_id = "map";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation.z = std::sin(yawRad / 2.0);
    goal.pose.orientation.w = std::cos(yawRad / 2.0);
    goalPub_->publish(goal);
    update("goal_pose", QJsonObject{{"x", x}, {"y", y}, {"yaw", yawRad}, {"source", "web"}});
    update("web_action", QJsonObject{{"ok", true}, {"action", "publish_goal"}, {"at_ms", nowMs()}});
    if (message) *message = "Goal diterbitkan ke /goal_pose";
    return true;
  }

  bool publishInitialPose(double x, double y, double yawRad, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yawRad)) {
      if (message) *message = "Initial pose tidak valid";
      return false;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = node_->now();
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.orientation.z = std::sin(yawRad / 2.0);
    pose.pose.pose.orientation.w = std::cos(yawRad / 2.0);
    pose.pose.covariance[0] = 0.01;
    pose.pose.covariance[7] = 0.01;
    pose.pose.covariance[35] = std::pow(kPi / 180.0, 2);
    initialPosePub_->publish(pose);
    update("web_action", QJsonObject{{"ok", true}, {"action", "publish_initial_pose"}, {"at_ms", nowMs()}});
    if (message) *message = "Initial pose diterbitkan ke /initialpose";
    return true;
  }

  bool cancelNavigation(QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    auto client = node_->create_client<action_msgs::srv::CancelGoal>("/navigate_to_pose/_action/cancel_goal");
    if (!client->wait_for_service(350ms)) {
      if (message) *message = "Service cancel Nav2 belum tersedia";
      return false;
    }
    auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
    client->async_send_request(request, [this, client](rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture future) {
      try {
        const auto result = future.get();
        const bool ok = result->return_code == 0 || !result->goals_canceling.empty();
        update("web_action", QJsonObject{{"ok", ok}, {"action", "cancel_navigation"},
                                         {"return_code", result->return_code},
                                         {"goals", static_cast<double>(result->goals_canceling.size())},
                                         {"at_ms", nowMs()}});
      } catch (const std::exception &e) {
        update("web_action", QJsonObject{{"ok", false}, {"action", "cancel_navigation"},
                                         {"message", QString::fromUtf8(e.what())}, {"at_ms", nowMs()}});
      }
    });
    if (message) *message = "Permintaan cancel dikirim ke Nav2";
    return true;
  }

  bool triggerService(const QString &service, const QString &tag, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    auto client = node_->create_client<std_srvs::srv::Trigger>(service.toStdString());
    if (!client->wait_for_service(350ms)) {
      if (message) *message = "Service belum tersedia: " + service;
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(request, [this, client, service, tag](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      try {
        const auto result = future.get();
        update("web_action", QJsonObject{{"ok", result->success}, {"action", tag}, {"service", service},
                                         {"message", QString::fromStdString(result->message)}, {"at_ms", nowMs()}});
      } catch (const std::exception &e) {
        update("web_action", QJsonObject{{"ok", false}, {"action", tag}, {"service", service},
                                         {"message", QString::fromUtf8(e.what())}, {"at_ms", nowMs()}});
      }
    });
    if (message) *message = "Trigger dikirim ke " + service;
    return true;
  }

  bool setSteeringCalibrationMode(bool enabled, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    if (enabled) {
      const double speed = scalarState("esc_drive_actual").value_or(0.0);
      if (std::abs(speed) > 0.05) {
        if (message) *message = QString("Ditolak: kendaraan masih bergerak (%1 m/s)").arg(speed, 0, 'f', 3);
        return false;
      }
    }
    auto client = node_->create_client<rcl_interfaces::srv::SetParametersAtomically>(
        "/esc_ackermann/set_parameters_atomically");
    if (!client->wait_for_service(500ms)) {
      if (message) *message = "Parameter service /esc_ackermann belum tersedia";
      return false;
    }
    auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = "steering_calibration_mode_enabled";
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    parameter.value.bool_value = enabled;
    request->parameters.push_back(parameter);
    client->async_send_request(
        request, [this, client, enabled](rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture future) {
          try {
            const auto result = future.get();
            update("web_action", QJsonObject{{"ok", result->result.successful},
                                             {"action", enabled ? "steering_cal_mode_on" : "steering_cal_mode_off"},
                                             {"message", QString::fromStdString(result->result.reason)}, {"at_ms", nowMs()}});
          } catch (const std::exception &e) {
            update("web_action", QJsonObject{{"ok", false}, {"action", "steering_calibration_mode"},
                                             {"message", QString::fromUtf8(e.what())}, {"at_ms", nowMs()}});
          }
        });
    if (message) *message = enabled ? "Mode kalibrasi steering diminta ON" : "Mode kalibrasi steering diminta OFF";
    return true;
  }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread thread_;
  std::atomic_bool stop_{false};
  QString bindAddress_;
  int port_{5005};
  bool readOnly_{false};
  double cameraJpegFps_{5.0};
  mutable std::mutex stateMutex_;
  QJsonObject state_;
  QJsonObject updated_;
  QSet<QString> dirty_;
  mutable std::mutex mediaMutex_;
  std::mutex cameraEncodeMutex_;
  QByteArray cameraJpeg_;
  QByteArray mapPng_;
  std::chrono::steady_clock::time_point lastCameraEncode_{};
  std::chrono::steady_clock::time_point lastLidarScan_{};
  rclcpp::TimerBase::SharedPtr graphTimer_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialPosePub_;

  bool rejectReadOnly(QString *message) const {
    if (message) *message = "Web GUI berjalan dalam read-only mode";
    return false;
  }

  std::optional<double> scalarState(const QString &key) const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    const QJsonValue value = state_.value(key);
    if (!value.isDouble()) return std::nullopt;
    return value.toDouble();
  }

  void update(const QString &channel, const QJsonValue &value) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    state_[channel] = value;
    updated_[channel] = nowMs();
    dirty_.insert(channel);
  }

  template <typename Msg, typename Callback>
  void subscribe(const std::string &topic, const rclcpp::QoS &qos, Callback callback) {
    subscriptions_.push_back(node_->create_subscription<Msg>(topic, qos, callback));
  }

  void setupRos() {
    const auto stateQos = rclcpp::QoS(1).reliable();
    const auto latchedQos = rclcpp::QoS(1).reliable().transient_local();
    const auto sensorQos = rclcpp::QoS(rclcpp::KeepLast(3)).best_effort();

    const std::vector<std::pair<const char *, const char *>> bools = {
        {"/gnss/connected", "connected.gnss"}, {"/imu/connected", "connected.imu"},
        {"/perception/camera_connected", "connected.camera"}, {"/esc/ready", "connected.esc_ready"},
        {"/esc/armed", "connected.esc_armed"}, {"/esc/feedback_valid", "connected.esc_feedback"},
        {"/esc/drive/connected", "connected.esc_drive"}, {"/esc/steer/connected", "connected.esc_steer"},
        {"/esc/steering_calibration/controller_ready", "esc_calibration_controller_ready"},
        {"/system/autonomy_ready", "system.autonomy_ready"}, {"/system/motion_ready", "system.motion_ready"},
        {"/system/planning_localization_ready", "system.planning_localization_ready"},
        {"/system/motion_localization_ready", "system.motion_localization_ready"},
        {"/system/nav2_ready", "system.nav2_ready"},
        {"/navigation/mppi_closed_loop/ready", "mppi_closed_loop_ready"},
        {"/navigation/velocity_smoother/closed_loop_eligible", "smoother_closed_loop_eligible"},
        {"/gnss/velocity_qualified", "gnss_velocity_qualified"}, {"/gnss/cog_qualified", "gnss_cog_qualified"},
        {"/gnss/velocity_fusion_active", "gnss_velocity_fusion_active"},
        {"/gnss/cog_fusion_active", "gnss_cog_fusion_active"},
        {"/perception/camera_healthy", "camera_healthy"},
        {"/perception/emergency_stop", "perception_emergency"},
        {"/lidar/safety_healthy", "connected.lidar"},
        {"/sensor_guard/healthy", "sensor_guard_healthy"},
        {"/mapping/map_valid", "mapping.map_valid"},
        {"/system/autonomy_motion_allowed", "system.autonomy_motion_allowed"},
        {"/system/manual_motion_allowed", "system.manual_motion_allowed"}};
    for (const auto &entry : bools) {
      const QString channel = QString::fromLatin1(entry.second);
      subscribe<std_msgs::msg::Bool>(entry.first, latchedQos, [this, channel](std_msgs::msg::Bool::ConstSharedPtr msg) {
        update(channel, msg->data);
      });
    }
    subscribe<std_msgs::msg::Bool>("/safety/estop", stateQos, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      update("system.estop", msg->data);
    });
    subscribe<std_msgs::msg::Bool>("/system/autonomy_motion_allowed", stateQos,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          update("system.motion_ready", msg->data);
        });

    const std::vector<std::pair<const char *, const char *>> strings = {
        {"/system/localization_state", "localization_state"}, {"/system/gnss_status", "gnss_status"},
        {"/gnss/state", "gnss_driver_state"}, {"/gnss/motion_diagnostics", "gnss_motion"},
        {"/gnss/motion_validation", "gnss_motion_validation"}, {"/gnss/fusion_status", "gnss_fusion_status"},
        {"/system/imu_status", "imu_status"}, {"/system/ekf_local_status", "ekf_local_status"},
        {"/system/ekf_global_status", "ekf_global_status"}, {"/system/pose_estimator", "pose_estimator"},
        {"/system/sensor_status", "sensor_status"}, {"/navigation/goal_state", "goal_state"},
        {"/navigation/mppi_closed_loop/status", "mppi_status"},
        {"/navigation/velocity_smoother/qualification", "smoother_qualification"},
        {"/perception/lane_safety_state", "lane_state"}, {"/perception/lane_control_state", "lane_control"},
        {"/yolop/lane_metrics", "lane_metrics"}, {"/perception/drivable_space", "drivable_space"},
        {"/perception/camera_health_state", "camera_health_state"}, {"/perception/near_field_state", "near_field_state"},
        {"/perception/obstacle_metrics", "obstacle_metrics"}, {"/perception/raw_detections", "raw_detections"},
        {"/perception/performance", "perception_performance"},
        {"/navigation/trajectory_safety_state", "trajectory_safety_state"},
        {"/collision_monitor/state", "collision_monitor_state"}, {"/esc/status", "esc_status"},
        {"/esc/foc/telemetry", "foc_telemetry"}, {"/esc/mux/active_source", "esc_mux"},
        {"/imu/status", "imu_driver_status"}, {"/lidar/status", "lidar_driver_status"},
        {"/lidar/safety_health", "lidar_safety_health"},
        {"/camera/color/status", "camera_status"},
        {"/obstacle_detection/status", "yolo_status"},
        {"/navigation/planner_status", "planner_status"},
        {"/mapping/map_stats", "mapping_stats"}};
    for (const auto &entry : strings) {
      const QString channel = QString::fromLatin1(entry.second);
      subscribe<std_msgs::msg::String>(entry.first, stateQos, [this, channel](std_msgs::msg::String::ConstSharedPtr msg) {
        const QString raw = QString::fromStdString(msg->data);
        if (channel == "goal_state") update(channel, QJsonObject{{"state", raw.trimmed().toUpper()}, {"raw", raw}});
        else update(channel, parseJsonOrKv(raw));
      });
    }

    subscribe<sensor_msgs::msg::NavSatFix>("/gnss/fix_raw", sensorQos, [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
      update("gnss_fix", QJsonObject{{"lat", msg->latitude}, {"lon", msg->longitude}, {"alt", msg->altitude},
                                     {"status", msg->status.status}, {"cov_x", msg->position_covariance[0]},
                                     {"cov_y", msg->position_covariance[4]},
                                     {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
    });

    subscribe<std_msgs::msg::Float64MultiArray>("/gnss/quality", sensorQos,
        [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
          std::vector<double> data = msg->data;
          data.resize(std::max<size_t>(45, data.size()), std::numeric_limits<double>::quiet_NaN());
          const char *keys[] = {"sat", "dop", "hacc_m", "fix_type", "source_id", "sacc_mps", "ground_speed_mps",
                                "course_enu_rad", "course_accuracy_rad", "itow_ms", "vacc_m", "vel_e_mps", "vel_n_mps",
                                "vel_d_mps", "gdop", "hdop", "vdop", "ndop", "edop", "tdop", "nav_cov_pos_valid",
                                "nav_cov_vel_valid", "pvt_rate_hz", "measurement_age_sec", "timestamp_source_code", "flags2",
                                "flags3", "nav_dop_itow_ms", "nav_dop_exact_epoch", "nav_cov_itow_ms", "nav_cov_exact_epoch",
                                "utc_valid_flags", "tacc_ns", "height_ellipsoid_m", "head_vehicle_enu_rad", "mag_declination_rad",
                                "mag_accuracy_rad", "diff_solution", "carrier_solution", "invalid_llh", "last_correction_age_code",
                                "auth_time", "head_vehicle_valid", "mag_valid", "gnss_fix_ok"};
          QJsonObject object;
          for (int i = 0; i < 45; ++i) {
            if (std::isfinite(data[static_cast<size_t>(i)])) object[keys[i]] = data[static_cast<size_t>(i)];
            else object[keys[i]] = QJsonValue();
          }
          for (const char *key : {"nav_cov_pos_valid", "nav_cov_vel_valid", "nav_dop_exact_epoch", "nav_cov_exact_epoch",
                                  "diff_solution", "invalid_llh", "auth_time", "head_vehicle_valid", "mag_valid", "gnss_fix_ok"}) {
            const QJsonValue value = object.value(key);
            object[key] = value.isDouble() && value.toDouble() > 0.5;
          }
          update("gnss_quality", object);
        });

    const auto velocitySubscribe = [this, sensorQos](const char *topic, const char *channel) {
      const QString ch = QString::fromLatin1(channel);
      subscribe<geometry_msgs::msg::TwistWithCovarianceStamped>(
          topic, sensorQos, [this, ch](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg) {
            const double vx = msg->twist.twist.linear.x;
            const double vy = msg->twist.twist.linear.y;
            update(ch, QJsonObject{{"vx", vx}, {"vy", vy}, {"vz", msg->twist.twist.linear.z},
                                   {"speed", std::hypot(vx, vy)},
                                   {"course_enu_rad", std::hypot(vx, vy) > 1e-9 ? std::atan2(vy, vx) : 0.0},
                                   {"cov_x", msg->twist.covariance[0]}, {"cov_y", msg->twist.covariance[7]},
                                   {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
          });
    };
    velocitySubscribe("/gnss/vel", "gnss_vel");
    velocitySubscribe("/gnss/velocity_position_fit", "gnss_vel_fit");
    velocitySubscribe("/gnss/vel_map", "gnss_vel_map");
    velocitySubscribe("/gnss/base_velocity", "gnss_base_vel");
    velocitySubscribe("/gnss/base_velocity_fusion", "gnss_base_vel_fusion");

    subscribe<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/gnss/cog_heading_fusion", sensorQos,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
          const auto &q = msg->pose.pose.orientation;
          update("gnss_cog_fusion", QJsonObject{{"yaw_rad", yawFromQuat(q.x, q.y, q.z, q.w)},
                                                 {"yaw_variance", msg->pose.covariance[35]},
                                                 {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
        });

    subscribe<sensor_msgs::msg::Imu>("/imu/data", sensorQos, [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
      const auto &q = msg->orientation;
      const double sinr = 2 * (q.w * q.x + q.y * q.z);
      const double cosr = 1 - 2 * (q.x * q.x + q.y * q.y);
      const double roll = std::atan2(sinr, cosr);
      const double sinp = 2 * (q.w * q.y - q.z * q.x);
      const double pitch = std::abs(sinp) >= 1 ? std::copysign(kPi / 2, sinp) : std::asin(sinp);
      update("imu", QJsonObject{{"roll_rad", roll}, {"pitch_rad", pitch}, {"yaw_rad", yawFromQuat(q.x, q.y, q.z, q.w)},
                                {"gx", msg->angular_velocity.x}, {"gy", msg->angular_velocity.y}, {"gz", msg->angular_velocity.z},
                                {"ax", msg->linear_acceleration.x}, {"ay", msg->linear_acceleration.y}, {"az", msg->linear_acceleration.z},
                                {"var_gx", msg->angular_velocity_covariance[0]}, {"var_gy", msg->angular_velocity_covariance[4]},
                                {"var_gz", msg->angular_velocity_covariance[8]},
                                {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
      update("connected.imu", true);
    });

    subscribe<sensor_msgs::msg::LaserScan>("/scan_nav", sensorQos,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          const auto now = std::chrono::steady_clock::now();
          double hz = 0.0;
          if (lastLidarScan_.time_since_epoch().count() != 0) {
            const double dt = std::chrono::duration<double>(now - lastLidarScan_).count();
            if (dt > 1e-6) hz = 1.0 / dt;
          }
          lastLidarScan_ = now;
          std::size_t valid = 0;
          double sum = 0.0;
          double minRange = std::numeric_limits<double>::infinity();
          double maxRange = 0.0;
          double center = std::numeric_limits<double>::quiet_NaN();
          const std::size_t mid = msg->ranges.empty() ? 0 : msg->ranges.size() / 2;
          for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
            const double r = msg->ranges[i];
            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
            ++valid;
            sum += r;
            minRange = std::min(minRange, r);
            maxRange = std::max(maxRange, r);
            if (i == mid) center = r;
          }
          if (!std::isfinite(center) && !msg->ranges.empty()) {
            for (std::size_t d = 0; d <= mid; ++d) {
              const std::size_t left = mid >= d ? mid - d : 0;
              const std::size_t right = std::min(mid + d, msg->ranges.size() - 1);
              for (std::size_t i : {left, right}) {
                const double r = msg->ranges[i];
                if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
                  center = r;
                  d = mid + 1;
                  break;
                }
              }
            }
          }
          const double total = static_cast<double>(msg->ranges.size());
          const double validPct = total > 0.0 ? 100.0 * static_cast<double>(valid) / total : 0.0;
          update("lidar", QJsonObject{
              {"frame_id", QString::fromStdString(msg->header.frame_id)},
              {"samples", total}, {"valid_samples", static_cast<double>(valid)},
              {"valid_ratio_pct", validPct}, {"dropout_pct", 100.0 - validPct},
              {"scan_rate_hz", hz}, {"range_center_m", std::isfinite(center) ? center : QJsonValue()},
              {"mean_range_m", valid ? sum / static_cast<double>(valid) : QJsonValue()},
              {"range_min_observed_m", std::isfinite(minRange) ? minRange : QJsonValue()},
              {"range_max_observed_m", valid ? maxRange : QJsonValue()},
              {"angle_min_rad", msg->angle_min}, {"angle_max_rad", msg->angle_max},
              {"angle_increment_rad", msg->angle_increment},
              {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
          update("connected.lidar", true);
        });

    subscribe<sensor_msgs::msg::LaserScan>("/scan_safety", sensorQos,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          std::size_t valid = 0;
          for (double r : msg->ranges) {
            if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) ++valid;
          }
          const double total = static_cast<double>(msg->ranges.size());
          update("lidar_safety", QJsonObject{
              {"frame_id", QString::fromStdString(msg->header.frame_id)},
              {"samples", total}, {"valid_ratio_pct", total > 0.0 ? 100.0 * valid / total : 0.0}});
        });

    subscribe<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", latchedQos,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
          const auto &p = msg->pose.pose.position;
          const auto &q = msg->pose.pose.orientation;
          update("amcl", QJsonObject{
              {"x", p.x}, {"y", p.y}, {"yaw", yawFromQuat(q.x, q.y, q.z, q.w)},
              {"var_x", msg->pose.covariance[0]}, {"var_y", msg->pose.covariance[7]},
              {"var_yaw", msg->pose.covariance[35]},
              {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
        });

    const auto odomSubscribe = [this, sensorQos](const char *topic, const char *channel) {
      const QString ch = QString::fromLatin1(channel);
      subscribe<nav_msgs::msg::Odometry>(topic, sensorQos, [this, ch](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto &p = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;
        update(ch, QJsonObject{{"x", p.x}, {"y", p.y}, {"yaw", yawFromQuat(q.x, q.y, q.z, q.w)},
                               {"v", msg->twist.twist.linear.x}, {"w", msg->twist.twist.angular.z},
                               {"var_x", msg->pose.covariance[0]}, {"var_y", msg->pose.covariance[7]},
                               {"var_yaw", msg->pose.covariance[35]},
                               {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
      });
    };
    odomSubscribe("/esc/odom", "esc_odom");
    odomSubscribe("/odom", "odom");
    odomSubscribe("/lidar/odom", "lidar_odom");
    odomSubscribe("/odometry/filtered", "ekf_local");
    odomSubscribe("/odometry/filtered_map", "ekf_global");

    const auto pathSubscribe = [this, sensorQos](const char *topic, const char *channel) {
      const QString ch = QString::fromLatin1(channel);
      subscribe<nav_msgs::msg::Path>(topic, sensorQos, [this, ch](nav_msgs::msg::Path::ConstSharedPtr msg) {
        QJsonArray points;
        double length = 0.0;
        double headingVariation = 0.0;
        double previousYaw = 0.0;
        bool havePrevious = false;
        const size_t stride = std::max<size_t>(1, msg->poses.size() / 800 + 1);
        for (size_t i = 0; i < msg->poses.size(); ++i) {
          const auto &p = msg->poses[i].pose.position;
          const auto &q = msg->poses[i].pose.orientation;
          const double yaw = yawFromQuat(q.x, q.y, q.z, q.w);
          if (i > 0) {
            const auto &prev = msg->poses[i - 1].pose.position;
            length += std::hypot(p.x - prev.x, p.y - prev.y);
          }
          if (havePrevious) {
            double d = yaw - previousYaw;
            while (d > kPi) d -= 2 * kPi;
            while (d < -kPi) d += 2 * kPi;
            headingVariation += std::abs(d);
          }
          previousYaw = yaw;
          havePrevious = true;
          if (i % stride == 0 || i + 1 == msg->poses.size()) points.append(QJsonArray{p.x, p.y, yaw});
        }
        update(ch, QJsonObject{{"count", static_cast<double>(msg->poses.size())}, {"length_m", length},
                               {"heading_variation_rad", headingVariation}, {"points", points},
                               {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
      });
    };
    pathSubscribe("/smac_plan", "nav_path");
    pathSubscribe("/plan", "nav_path_legacy");
    pathSubscribe("/transformed_global_plan", "local_path");
    pathSubscribe("/controller_server/transformed_global_plan", "local_path_legacy");
    pathSubscribe("/local_plan", "local_path_legacy");

    const auto goalSubscribe = [this, stateQos](const char *topic) {
      subscribe<geometry_msgs::msg::PoseStamped>(topic, stateQos, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        const auto &q = msg->pose.orientation;
        update("goal_pose", QJsonObject{{"x", msg->pose.position.x}, {"y", msg->pose.position.y},
                                        {"yaw", yawFromQuat(q.x, q.y, q.z, q.w)}});
      });
    };
    goalSubscribe("/navigation/goal_request");
    goalSubscribe("/goal_pose");

    const auto twistSubscribe = [this, sensorQos](const char *topic, const char *channel) {
      const QString ch = QString::fromLatin1(channel);
      subscribe<geometry_msgs::msg::Twist>(topic, sensorQos, [this, ch](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        update(ch, QJsonObject{{"linear_x", msg->linear.x}, {"angular_z", msg->angular.z}});
      });
    };
    twistSubscribe("/cmd_vel_nav_raw", "cmd_nav");
    twistSubscribe("/cmd_vel/perception_advisory", "cmd_perception_advisory");
    twistSubscribe("/cmd_vel/autonomy_integrated", "cmd_autonomy_integrated");
    twistSubscribe("/cmd_vel/nav2_pre_collision", "cmd_pre_collision");
    twistSubscribe("/cmd_vel/collision_preview", "cmd_collision_preview");
    twistSubscribe("/cmd_vel", "cmd_final");
    twistSubscribe("/cmd_vel/actuator", "cmd_actuator");

    const std::vector<std::pair<const char *, const char *>> floats = {
        {"/esc/drive_target_mps", "esc_drive_target"}, {"/esc/drive_actual_mps", "esc_drive_actual"},
        {"/esc/steering_target_rad", "esc_steer_target"},
        {"/esc/steering_command_uncalibrated_rad", "esc_steer_uncal_target"},
        {"/esc/steering_uncalibrated_rad", "esc_steer_uncalibrated"},
        {"/esc/steering_protocol_command_rad", "esc_steer_protocol_cmd"},
        {"/esc/steering_feedback_raw_rad", "esc_steer_feedback_raw"},
        {"/esc/steering_actual_rad", "esc_steer_actual"}, {"/esc/yaw_rate_actual_rps", "esc_yaw_rate"},
        {"/esc/kinematic_yaw_rate_rps", "esc_kinematic_yaw_rate"},
        {"/navigation/mppi_closed_loop/velocity_error_mps", "mppi_velocity_error"},
        {"/navigation/mppi_closed_loop/steering_error_rad", "mppi_steering_error"},
        {"/navigation/mppi_closed_loop/yaw_rate_error_rps", "mppi_yaw_error"}};
    for (const auto &entry : floats) {
      const QString channel = QString::fromLatin1(entry.second);
      subscribe<std_msgs::msg::Float64>(entry.first, sensorQos, [this, channel](std_msgs::msg::Float64::ConstSharedPtr msg) {
        update(channel, msg->data);
      });
    }

    auto cacheCamera = [this](sensor_msgs::msg::Image::ConstSharedPtr msg, const QString &source) {
      // Raw and annotated image subscriptions may execute concurrently in the
      // MultiThreadedExecutor. Serialize throttle/encode state to avoid racing
      // lastCameraEncode_ and producing duplicate JPEG work on the mini-PC.
      std::lock_guard<std::mutex> encodeLock(cameraEncodeMutex_);
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - lastCameraEncode_).count();
      if (lastCameraEncode_.time_since_epoch().count() != 0 && elapsed < 1.0 / cameraJpegFps_) return;
      if (msg->width == 0 || msg->height == 0 || msg->step == 0) return;
      const std::uint64_t requiredBytes = static_cast<std::uint64_t>(msg->step) * static_cast<std::uint64_t>(msg->height);
      if (requiredBytes > msg->data.size()) return;
      QImage image;
      if (msg->encoding == "rgb8" && msg->step >= msg->width * 3U) {
        image = QImage(msg->data.data(), static_cast<int>(msg->width), static_cast<int>(msg->height),
                       static_cast<int>(msg->step), QImage::Format_RGB888).copy();
      } else if (msg->encoding == "bgr8" && msg->step >= msg->width * 3U) {
        image = QImage(msg->data.data(), static_cast<int>(msg->width), static_cast<int>(msg->height),
                       static_cast<int>(msg->step), QImage::Format_RGB888).rgbSwapped().copy();
      } else if ((msg->encoding == "mono8" || msg->encoding == "8UC1") && msg->step >= msg->width) {
        image = QImage(msg->data.data(), static_cast<int>(msg->width), static_cast<int>(msg->height),
                       static_cast<int>(msg->step), QImage::Format_Grayscale8).copy();
      }
      if (image.isNull()) return;
      QByteArray encoded;
      QBuffer buffer(&encoded);
      if (!buffer.open(QIODevice::WriteOnly) || !image.save(&buffer, "JPG", 78) || encoded.isEmpty()) return;
      {
        std::lock_guard<std::mutex> lock(mediaMutex_);
        cameraJpeg_ = encoded;
      }
      lastCameraEncode_ = now;
      update("camera_frame", QJsonObject{{"width", static_cast<int>(msg->width)}, {"height", static_cast<int>(msg->height)},
                                         {"encoding", QString::fromStdString(msg->encoding)}, {"source", source}, {"at_ms", nowMs()}});
      update("connected.camera", true);
      update("camera_healthy", true);
    };
    subscribe<sensor_msgs::msg::Image>("/camera/color/image_raw", sensorQos,
                                      [cacheCamera](sensor_msgs::msg::Image::ConstSharedPtr msg) { cacheCamera(msg, "raw"); });
    subscribe<sensor_msgs::msg::Image>("/obstacle_detection/visualization", sensorQos,
                                      [cacheCamera](sensor_msgs::msg::Image::ConstSharedPtr msg) { cacheCamera(msg, "annotated"); });
    subscribe<sensor_msgs::msg::Image>("/camera/yolop/image_annotated", sensorQos,
                                      [cacheCamera](sensor_msgs::msg::Image::ConstSharedPtr msg) { cacheCamera(msg, "annotated-legacy"); });
    subscribe<std_msgs::msg::String>("/obstacle_detection/performance", sensorQos,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          update("perception_performance", parseJsonOrKv(QString::fromStdString(msg->data)));
        });

    subscribe<nav_msgs::msg::OccupancyGrid>("/map", latchedQos, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      if (msg->info.width == 0 || msg->info.height == 0 || msg->data.empty()) return;
      const std::uint64_t cellCount = static_cast<std::uint64_t>(msg->info.width) * static_cast<std::uint64_t>(msg->info.height);
      if (cellCount > msg->data.size() || cellCount > 100000000ULL ||
          msg->info.width > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
          msg->info.height > static_cast<unsigned int>(std::numeric_limits<int>::max())) return;
      QImage image(static_cast<int>(msg->info.width), static_cast<int>(msg->info.height), QImage::Format_Grayscale8);
      if (image.isNull()) return;
      for (unsigned int y = 0; y < msg->info.height; ++y) {
        uchar *row = image.scanLine(static_cast<int>(msg->info.height - 1 - y));
        for (unsigned int x = 0; x < msg->info.width; ++x) {
          const int8_t occ = msg->data[static_cast<size_t>(y) * msg->info.width + x];
          int shade = 118;
          if (occ >= 0) shade = std::clamp(245 - static_cast<int>(occ) * 2, 35, 245);
          row[x] = static_cast<uchar>(shade);
        }
      }
      QByteArray encoded;
      QBuffer buffer(&encoded);
      if (!buffer.open(QIODevice::WriteOnly) || !image.save(&buffer, "PNG") || encoded.isEmpty()) return;
      {
        std::lock_guard<std::mutex> lock(mediaMutex_);
        mapPng_ = encoded;
      }
      const auto &origin = msg->info.origin.position;
      update("map_meta", QJsonObject{{"width", static_cast<int>(msg->info.width)}, {"height", static_cast<int>(msg->info.height)},
                                     {"resolution", msg->info.resolution}, {"origin_x", origin.x}, {"origin_y", origin.y},
                                     {"frame_id", QString::fromStdString(msg->header.frame_id)}, {"at_ms", nowMs()}});
    });

    const auto gridStatsSubscribe = [this, latchedQos](const char *topic, const char *channel) {
      const QString ch = QString::fromLatin1(channel);
      subscribe<nav_msgs::msg::OccupancyGrid>(topic, latchedQos,
          [this, ch](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
            if (msg->data.empty()) return;
            std::size_t unknown = 0, occupied = 0;
            for (const auto cell : msg->data) {
              if (cell < 0) ++unknown;
              else if (cell >= 50) ++occupied;
            }
            const double n = static_cast<double>(msg->data.size());
            update(ch, QJsonObject{
                {"width", static_cast<int>(msg->info.width)}, {"height", static_cast<int>(msg->info.height)},
                {"resolution", msg->info.resolution},
                {"unknown_pct", n > 0 ? 100.0 * unknown / n : 0.0},
                {"occupied_pct", n > 0 ? 100.0 * occupied / n : 0.0},
                {"frame_id", QString::fromStdString(msg->header.frame_id)}, {"at_ms", nowMs()}});
          });
    };
    gridStatsSubscribe("/nav_map", "nav_map_stats");
    gridStatsSubscribe("/global_costmap/costmap", "global_costmap");
    gridStatsSubscribe("/local_costmap/costmap", "local_costmap");

    const auto cloudSubscribe = [this, sensorQos](const char *topic, const char *channel) {
      const QString ch = QString::fromLatin1(channel);
      subscribe<sensor_msgs::msg::PointCloud2>(topic, sensorQos, [this, ch](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        update(ch, QJsonObject{{"count", static_cast<double>(msg->width) * static_cast<double>(msg->height)},
                               {"frame_id", QString::fromStdString(msg->header.frame_id)}});
      });
    };
    cloudSubscribe("/perception/object_points", "object_points");
    cloudSubscribe("/perception/path_relevant_points", "path_relevant_points");
    cloudSubscribe("/perception/planning_relevant_points", "planning_relevant_points");
    cloudSubscribe("/perception/drivable_boundary_points", "drivable_boundary_points");

    goalPub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    initialPosePub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    graphTimer_ = node_->create_wall_timer(1s, [this]() {
      bool planner = false, controller = false, bt = false, behavior = false, smoother = false, amcl = false;
      for (const auto &raw : node_->get_node_names()) {
        const QString name = QString::fromStdString(raw);
        planner = planner || name.endsWith("/planner_server") || name == "planner_server";
        controller = controller || name.endsWith("/controller_server") || name == "controller_server";
        bt = bt || name.endsWith("/bt_navigator") || name == "bt_navigator";
        behavior = behavior || name.endsWith("/behavior_server") || name == "behavior_server";
        smoother = smoother || name.endsWith("/velocity_smoother") || name == "velocity_smoother";
        amcl = amcl || name.endsWith("/amcl") || name == "amcl";
      }
      update("nav2_runtime", QJsonObject{{"planner", planner}, {"controller", controller}, {"bt_navigator", bt},
                                         {"behavior", behavior}, {"smoother", smoother}, {"amcl", amcl}});
      update("system.nav2_ready", planner && controller && bt);
    });
  }
};

class LocalHttpServer : public QObject {
 public:
  LocalHttpServer(WebRosBridge *bridge, QObject *parent = nullptr) : QObject(parent), bridge_(bridge) {
    connect(&server_, &QTcpServer::newConnection, this, [this]() { acceptConnections(); });
    eventTimer_.setInterval(200);
    connect(&eventTimer_, &QTimer::timeout, this, [this]() { broadcastEvents(); });
    recordingTimer_.setInterval(200);
    connect(&recordingTimer_, &QTimer::timeout, this, [this]() { captureRecordingSample(); });
  }

  bool start(const QString &bindAddress, int port) {
    QHostAddress address;
    if (!address.setAddress(bindAddress)) {
      if (bindAddress == "localhost") address = QHostAddress::LocalHost;
      else return false;
    }
    if (!server_.listen(address, static_cast<quint16>(port))) return false;
    try {
      staticRoot_ = QString::fromStdString(ament_index_cpp::get_package_share_directory("navigation")) + "/web/static";
    } catch (...) {
      staticRoot_.clear();
    }
    const QString envRoot = QString::fromUtf8(qgetenv("AGV_WEB_STATIC_DIR"));
    if (!envRoot.isEmpty() && QDir(envRoot).exists()) staticRoot_ = envRoot;
    const QString canonicalRoot = QFileInfo(staticRoot_).canonicalFilePath();
    if (canonicalRoot.isEmpty() || !QFileInfo(canonicalRoot + "/index.html").isFile()) {
      server_.close();
      staticRoot_.clear();
      return false;
    }
    staticRoot_ = canonicalRoot;
    eventTimer_.start();
    return true;
  }

 private:
  struct Request {
    QByteArray method;
    QString path;
    QMap<QByteArray, QByteArray> headers;
    QByteArray body;
  };

  WebRosBridge *bridge_{nullptr};
  QTcpServer server_;
  QTimer eventTimer_;
  QTimer recordingTimer_;
  QList<QPointer<QTcpSocket>> sseClients_;
  QString staticRoot_;
  int heartbeatTicks_{0};
  bool recording_{false};
  QString recordingSubsystem_;
  QString recordingId_;
  QString recordingVariation_;
  QString recordingCondition_;
  QString recordingStartedIso_;
  double recordingRateHz_{5.0};
  QVector<QMap<QString, QString>> recordingRows_;

  void acceptConnections() {
    while (server_.hasPendingConnections()) {
      QTcpSocket *socket = server_.nextPendingConnection();
      socket->setProperty("request_buffer", QByteArray());
      connect(socket, &QTcpSocket::readyRead, this, [this, socket]() { readRequest(socket); });
      connect(socket, &QTcpSocket::disconnected, socket, &QObject::deleteLater);
    }
  }

  void readRequest(QTcpSocket *socket) {
    if (!socket || socket->property("sse").toBool()) return;
    QByteArray buffer = socket->property("request_buffer").toByteArray();
    buffer += socket->readAll();
    const int headerEnd = buffer.indexOf("\r\n\r\n");
    if (headerEnd < 0) {
      if (buffer.size() > 64 * 1024) {
        sendJson(socket, 413, QJsonObject{{"ok", false}, {"message", "Request headers too large"}});
      } else {
        socket->setProperty("request_buffer", buffer);
      }
      return;
    }
    if (headerEnd > 64 * 1024) {
      return sendJson(socket, 413, QJsonObject{{"ok", false}, {"message", "Request headers too large"}});
    }
    const QByteArray headersPart = buffer.left(headerEnd);
    const QList<QByteArray> lines = headersPart.split('\n');
    if (lines.isEmpty()) return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "Bad request"}});
    const QList<QByteArray> first = lines.first().trimmed().split(' ');
    if (first.size() < 2) return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "Bad request line"}});
    Request request;
    request.method = first[0].trimmed();
    const QUrl url = QUrl::fromEncoded(first[1]);
    request.path = url.path();
    for (int i = 1; i < lines.size(); ++i) {
      const QByteArray line = lines[i].trimmed();
      const int colon = line.indexOf(':');
      if (colon > 0) request.headers[line.left(colon).trimmed().toLower()] = line.mid(colon + 1).trimmed();
    }
    bool ok = false;
    const int contentLength = request.headers.value("content-length", "0").toInt(&ok);
    const int bodyStart = headerEnd + 4;
    if (!ok || contentLength < 0 || contentLength > 256 * 1024) {
      return sendJson(socket, 413, QJsonObject{{"ok", false}, {"message", "Request body too large or invalid"}});
    }
    if (contentLength > 0 && buffer.size() < bodyStart + contentLength) {
      socket->setProperty("request_buffer", buffer);
      return;
    }
    if (contentLength > 0) request.body = buffer.mid(bodyStart, contentLength);
    handle(socket, request);
  }

  void handle(QTcpSocket *socket, const Request &request) {
    if (request.method == "GET" && request.path == "/api/events") return openSse(socket);
    if (request.method == "GET" && request.path == "/api/state") return sendJson(socket, 200, bridge_->snapshot());
    if (request.method == "GET" && request.path == "/api/health") {
      return sendJson(socket, 200, QJsonObject{{"ok", true}, {"ros", true}, {"read_only", bridge_->readOnly()},
                                               {"server_time_ms", nowMs()}});
    }
    if (request.method == "GET" && request.path == "/api/experiments") return sendJson(socket, 200, experimentCatalogJson());
    if (request.method == "GET" && request.path == "/api/config") return sendJson(socket, 200, loadConfigSnapshot());
    if (request.method == "GET" && request.path == "/api/experiment/record/status") {
      return sendJson(socket, 200, recordingStatus());
    }
    if (request.method == "GET" && request.path == "/api/camera.jpg") {
      const QByteArray bytes = bridge_->cameraJpeg();
      if (bytes.isEmpty()) return sendText(socket, 503, "text/plain; charset=utf-8", "Camera frame belum tersedia");
      return sendBytes(socket, 200, "image/jpeg", bytes, {{"Cache-Control", "no-store, max-age=0"}});
    }
    if (request.method == "GET" && request.path == "/api/map.png") {
      const QByteArray bytes = bridge_->mapPng();
      if (bytes.isEmpty()) return sendText(socket, 503, "text/plain; charset=utf-8", "Map belum tersedia");
      return sendBytes(socket, 200, "image/png", bytes, {{"Cache-Control", "no-store, max-age=0"}});
    }
    if (request.method == "POST") return handlePost(socket, request);
    if (request.method == "GET") return serveStatic(socket, request.path);
    sendJson(socket, 405, QJsonObject{{"ok", false}, {"message", "Method not allowed"}});
  }

  void handlePost(QTcpSocket *socket, const Request &request) {
    QJsonParseError error{};
    const QJsonDocument doc = QJsonDocument::fromJson(request.body, &error);
    if (!request.body.isEmpty() && (error.error != QJsonParseError::NoError || !doc.isObject())) {
      return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "JSON body tidak valid"}});
    }
    const QJsonObject json = doc.isObject() ? doc.object() : QJsonObject();
    QString message;
    bool ok = false;
    if (request.path == "/api/config/set") {
      if (bridge_->readOnly()) {
        return sendJson(socket, 403, QJsonObject{{"ok", false}, {"message", "Web GUI read-only; perubahan YAML ditolak"}});
      }
      QJsonValue saved;
      ok = setYamlValueAtomic(json.value("file_key").toString(), json.value("path").toString(),
                              json.value("value"), &message, &saved);
      return sendJson(socket, ok ? 200 : 409, QJsonObject{{"ok", ok}, {"message", message},
                       {"file_key", json.value("file_key")}, {"path", json.value("path")}, {"saved_value", saved},
                       {"config", loadConfigSnapshot()}, {"at_ms", nowMs()}});
    } else if (request.path == "/api/experiment/record/start") {
      ok = startRecording(json, &message);
      return sendJson(socket, ok ? 200 : 409, QJsonObject{{"ok", ok}, {"message", message},
                       {"recording", recordingStatus()}, {"at_ms", nowMs()}});
    } else if (request.path == "/api/experiment/record/stop") {
      QJsonObject result;
      ok = stopRecording(&message, &result);
      result["ok"] = ok; result["message"] = message; result["at_ms"] = nowMs();
      return sendJson(socket, ok ? 200 : 409, result);
    } else if (request.path == "/api/navigation/goal") {
      ok = bridge_->publishGoal(json.value("x").toDouble(std::numeric_limits<double>::quiet_NaN()),
                                json.value("y").toDouble(std::numeric_limits<double>::quiet_NaN()),
                                json.contains("yaw_rad") ? json.value("yaw_rad").toDouble() : json.value("yaw_deg").toDouble() * kPi / 180.0,
                                &message);
    } else if (request.path == "/api/navigation/cancel") {
      ok = bridge_->cancelNavigation(&message);
    } else if (request.path == "/api/localization/initial-pose") {
      ok = bridge_->publishInitialPose(json.value("x").toDouble(std::numeric_limits<double>::quiet_NaN()),
                                       json.value("y").toDouble(std::numeric_limits<double>::quiet_NaN()),
                                       json.contains("yaw_rad") ? json.value("yaw_rad").toDouble() : json.value("yaw_deg").toDouble() * kPi / 180.0,
                                       &message);
    } else if (request.path == "/api/localization/reset-calibration") {
      ok = bridge_->triggerService("/localization/reset_calibration_samples", "reset_localization_calibration", &message);
    } else if (request.path == "/api/steering/calibration-mode") {
      ok = bridge_->setSteeringCalibrationMode(json.value("enabled").toBool(false), &message);
    } else {
      return sendJson(socket, 404, QJsonObject{{"ok", false}, {"message", "Unknown API endpoint"}});
    }
    sendJson(socket, ok ? 200 : 409, QJsonObject{{"ok", ok}, {"message", message}, {"at_ms", nowMs()}});
  }

  QJsonObject recordingStatus() const {
    return QJsonObject{{"active", recording_}, {"subsystem", recordingSubsystem_}, {"id", recordingId_},
                       {"variation", recordingVariation_}, {"condition", recordingCondition_},
                       {"started_at", recordingStartedIso_}, {"sample_rate_hz", recordingRateHz_},
                       {"samples", recordingRows_.size()},
                       {"report_root", QDir::home().filePath(QStringLiteral(".ros/agv_web_reports"))}};
  }

  bool startRecording(const QJsonObject &json, QString *message) {
    if (recording_) {
      if (message) *message = QStringLiteral("Recording lain masih aktif; Stop CSV lebih dulu");
      return false;
    }
    const QString subsystem = json.value("subsystem").toString().trimmed();
    const QString id = json.value("id").toString().trimmed();
    if (!QStringList{QStringLiteral("navigation"), QStringLiteral("perception"), QStringLiteral("steering")}.contains(subsystem) || id.isEmpty()) {
      if (message) *message = QStringLiteral("subsystem/id recording tidak valid");
      return false;
    }
    recordingSubsystem_ = subsystem;
    recordingId_ = id;
    recordingVariation_ = json.value("variation").toString().trimmed();
    recordingCondition_ = json.value("condition").toString().trimmed();
    recordingRateHz_ = std::clamp(json.value("sample_rate_hz").toDouble(5.0), 1.0, 20.0);
    recordingStartedIso_ = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    recordingRows_.clear();
    recording_ = true;
    recordingTimer_.start(std::max(50, static_cast<int>(std::lround(1000.0 / recordingRateHz_))));
    captureRecordingSample();
    if (message) *message = QStringLiteral("CSV recording dimulai: ") + subsystem + QStringLiteral("/") + id;
    return true;
  }

  void captureRecordingSample() {
    if (!recording_ || !bridge_) return;
    QMap<QString, QString> row;
    row["time_iso"] = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    row["subsystem"] = recordingSubsystem_;
    row["section_id"] = recordingId_;
    row["variation"] = recordingVariation_;
    row["condition"] = recordingCondition_;
    const QJsonObject snap = bridge_->snapshot();
    for (auto it = snap.constBegin(); it != snap.constEnd(); ++it) {
      if (it.key().startsWith(QStringLiteral("__"))) continue;
      flattenJson(it.key(), it.value(), row);
    }
    recordingRows_.push_back(row);
    if (recordingRows_.size() > 250000) {
      recording_ = false;
      recordingTimer_.stop();
    }
  }

  bool saveRecordingFiles(QJsonObject *result, QString *message) {
    if (recordingRows_.isEmpty()) {
      if (message) *message = QStringLiteral("Tidak ada sampel untuk disimpan");
      return false;
    }
    const QString root = QDir::home().filePath(QStringLiteral(".ros/agv_web_reports"));
    if (!QDir().mkpath(root)) {
      if (message) *message = QStringLiteral("Gagal membuat folder report: ") + root;
      return false;
    }
    const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString stem = safeStem(recordingSubsystem_ + QStringLiteral("_") + recordingId_ + QStringLiteral("_") + recordingVariation_);
    if (stem.isEmpty()) stem = QStringLiteral("experiment");
    stem += QStringLiteral("_") + stamp;
    QSet<QString> all;
    for (const auto &row : recordingRows_) for (auto it = row.cbegin(); it != row.cend(); ++it) all.insert(it.key());
    QStringList columns = all.values();
    std::sort(columns.begin(), columns.end());
    for (const QString &preferred : {QStringLiteral("condition"), QStringLiteral("variation"), QStringLiteral("section_id"),
                                      QStringLiteral("subsystem"), QStringLiteral("time_iso")}) columns.removeAll(preferred);
    columns.prepend(QStringLiteral("condition")); columns.prepend(QStringLiteral("variation"));
    columns.prepend(QStringLiteral("section_id")); columns.prepend(QStringLiteral("subsystem")); columns.prepend(QStringLiteral("time_iso"));
    const QString rawPath = QDir(root).filePath(stem + QStringLiteral("_raw.csv"));
    QSaveFile rawFile(rawPath);
    if (!rawFile.open(QIODevice::WriteOnly | QIODevice::Text)) return false;
    QByteArray raw;
    QStringList escapedHeader; for (const QString &c : columns) escapedHeader << csvEscape(c);
    raw += escapedHeader.join(',').toUtf8() + '\n';
    for (const auto &row : recordingRows_) {
      QStringList values; for (const QString &c : columns) values << csvEscape(row.value(c));
      raw += values.join(',').toUtf8() + '\n';
    }
    rawFile.write(raw);
    if (!rawFile.commit()) return false;

    const QString summaryPath = QDir(root).filePath(stem + QStringLiteral("_summary.csv"));
    QSaveFile summaryFile(summaryPath);
    if (!summaryFile.open(QIODevice::WriteOnly | QIODevice::Text)) return false;
    QByteArray summary = "\"metric\",\"last\",\"mean\",\"min\",\"max\",\"count\"\n";
    for (const QString &column : columns) {
      QVector<double> numbers;
      QString last;
      for (const auto &row : recordingRows_) {
        const QString text = row.value(column);
        if (!text.isEmpty()) last = text;
        bool okNum = false; const double number = text.toDouble(&okNum);
        if (okNum && std::isfinite(number)) numbers.push_back(number);
      }
      if (numbers.isEmpty()) continue;
      const double sum = std::accumulate(numbers.cbegin(), numbers.cend(), 0.0);
      const auto mm = std::minmax_element(numbers.cbegin(), numbers.cend());
      const QStringList cells = {csvEscape(column), csvEscape(last), csvEscape(QString::number(sum / numbers.size(), 'g', 12)),
                                 csvEscape(QString::number(*mm.first, 'g', 12)), csvEscape(QString::number(*mm.second, 'g', 12)),
                                 csvEscape(QString::number(numbers.size()))};
      summary += cells.join(',').toUtf8() + '\n';
    }
    summaryFile.write(summary);
    if (!summaryFile.commit()) return false;

    const QString configPath = QDir(root).filePath(stem + QStringLiteral("_config.json"));
    QSaveFile configFile(configPath);
    if (configFile.open(QIODevice::WriteOnly)) {
      configFile.write(QJsonDocument(loadConfigSnapshot()).toJson(QJsonDocument::Indented));
      configFile.commit();
    }
    const QString manifestPath = QDir(root).filePath(stem + QStringLiteral("_manifest.json"));
    QJsonObject manifest{{"subsystem", recordingSubsystem_}, {"section_id", recordingId_},
                         {"variation", recordingVariation_}, {"condition", recordingCondition_},
                         {"started_at", recordingStartedIso_},
                         {"stopped_at", QDateTime::currentDateTime().toString(Qt::ISODateWithMs)},
                         {"sample_rate_hz", recordingRateHz_}, {"sample_count", recordingRows_.size()},
                         {"raw_csv", rawPath}, {"summary_csv", summaryPath}, {"config_snapshot", configPath}};
    QSaveFile manifestFile(manifestPath);
    if (!manifestFile.open(QIODevice::WriteOnly)) return false;
    manifestFile.write(QJsonDocument(manifest).toJson(QJsonDocument::Indented));
    if (!manifestFile.commit()) return false;
    manifest["manifest"] = manifestPath;
    manifest["report_root"] = root;
    if (result) *result = manifest;
    if (message) *message = QStringLiteral("CSV + summary + config + manifest otomatis tersimpan di ") + root;
    return true;
  }

  bool stopRecording(QString *message, QJsonObject *result) {
    if (!recording_) {
      if (message) *message = QStringLiteral("Tidak ada CSV recording aktif");
      return false;
    }
    captureRecordingSample();
    recording_ = false;
    recordingTimer_.stop();
    const bool saved = saveRecordingFiles(result, message);
    recordingRows_.clear();
    return saved;
  }

  void openSse(QTcpSocket *socket) {
    QByteArray headers;
    headers += "HTTP/1.1 200 OK\r\n";
    headers += "Content-Type: text/event-stream\r\n";
    headers += "Cache-Control: no-cache, no-transform\r\n";
    headers += "Connection: keep-alive\r\n";
    headers += "X-Accel-Buffering: no\r\n\r\n";
    socket->write(headers);
    socket->setProperty("sse", true);
    sseClients_.append(QPointer<QTcpSocket>(socket));
    const QByteArray snapshot = QJsonDocument(bridge_->snapshot()).toJson(QJsonDocument::Compact);
    socket->write("event: snapshot\ndata: " + snapshot + "\n\n");
    socket->flush();
  }

  void broadcastEvents() {
    for (int i = sseClients_.size() - 1; i >= 0; --i) {
      if (sseClients_[i].isNull() || sseClients_[i]->state() != QAbstractSocket::ConnectedState) sseClients_.removeAt(i);
    }
    const QJsonObject delta = bridge_->takeDelta();
    QByteArray payload;
    if (!delta.isEmpty()) payload = "data: " + QJsonDocument(delta).toJson(QJsonDocument::Compact) + "\n\n";
    ++heartbeatTicks_;
    const bool heartbeat = heartbeatTicks_ >= 50;
    if (heartbeat) heartbeatTicks_ = 0;
    for (const auto &client : sseClients_) {
      if (client.isNull()) continue;
      if (client->bytesToWrite() > 2 * 1024 * 1024) continue;
      if (!payload.isEmpty()) client->write(payload);
      if (heartbeat) client->write(": heartbeat\n\n");
    }
  }

  void serveStatic(QTcpSocket *socket, QString path) {
    if (path == "/") path = "/index.html";
    if (path.contains("..")) return sendJson(socket, 403, QJsonObject{{"ok", false}, {"message", "Forbidden"}});
    if (staticRoot_.isEmpty()) return sendJson(socket, 500, QJsonObject{{"ok", false}, {"message", "Static web root tidak ditemukan"}});
    // Use lexical containment instead of canonicalFilePath() for the requested
    // file.  With colcon --symlink-install, static assets are symlinks into src/;
    // canonicalizing the file resolves outside install/share and incorrectly
    // rejects valid files as 404.  '..' is already rejected above and the
    // cleaned absolute path must remain inside the trusted static root.
    const QString cleanRoot = QDir::cleanPath(QDir(staticRoot_).absolutePath());
    const QString cleanFile = QDir::cleanPath(QDir(cleanRoot).absoluteFilePath(path.mid(1)));
    const QString rootPrefix = cleanRoot.endsWith(QDir::separator()) ? cleanRoot : cleanRoot + QDir::separator();
    if (!cleanFile.startsWith(rootPrefix) || !QFileInfo(cleanFile).isFile()) {
      return sendJson(socket, 404, QJsonObject{{"ok", false}, {"message", "Not found"}});
    }
    QFile file(cleanFile);
    if (!file.open(QIODevice::ReadOnly)) return sendJson(socket, 404, QJsonObject{{"ok", false}, {"message", "Not found"}});
    sendBytes(socket, 200, mimeTypeForPath(cleanFile), file.readAll(), {{"Cache-Control", "no-cache"}});
  }

  static QByteArray statusText(int status) {
    switch (status) {
      case 200: return "OK";
      case 400: return "Bad Request";
      case 403: return "Forbidden";
      case 404: return "Not Found";
      case 405: return "Method Not Allowed";
      case 409: return "Conflict";
      case 413: return "Payload Too Large";
      case 500: return "Internal Server Error";
      case 503: return "Service Unavailable";
      default: return "Response";
    }
  }

  void sendJson(QTcpSocket *socket, int status, const QJsonObject &json) {
    sendBytes(socket, status, "application/json; charset=utf-8", QJsonDocument(json).toJson(QJsonDocument::Compact),
              {{"Cache-Control", "no-store"}});
  }

  void sendText(QTcpSocket *socket, int status, const QByteArray &contentType, const QByteArray &text) {
    sendBytes(socket, status, contentType, text, {{"Cache-Control", "no-store"}});
  }

  void sendBytes(QTcpSocket *socket, int status, const QByteArray &contentType, const QByteArray &body,
                 const QMap<QByteArray, QByteArray> &extra = {}) {
    QByteArray headers = "HTTP/1.1 " + QByteArray::number(status) + " " + statusText(status) + "\r\n";
    headers += "Content-Type: " + contentType + "\r\n";
    headers += "Content-Length: " + QByteArray::number(body.size()) + "\r\n";
    headers += "Connection: close\r\n";
    headers += "X-Content-Type-Options: nosniff\r\n";
    headers += "Referrer-Policy: no-referrer\r\n";
    headers += "Content-Security-Policy: default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; connect-src 'self'\r\n";
    for (auto it = extra.cbegin(); it != extra.cend(); ++it) headers += it.key() + ": " + it.value() + "\r\n";
    headers += "\r\n";
    socket->write(headers);
    socket->write(body);
    socket->disconnectFromHost();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  QCoreApplication app(argc, argv);
  QCoreApplication::setApplicationName("AGV Web GUI");
  QCoreApplication::setApplicationVersion("1.0");
  int result = 1;
  try {
    WebRosBridge bridge;
    LocalHttpServer server(&bridge);
    if (!server.start(bridge.bindAddress(), bridge.port())) {
      RCLCPP_ERROR(rclcpp::get_logger("agv_web_gui"), "Gagal listen web GUI pada %s:%d",
                   bridge.bindAddress().toUtf8().constData(), bridge.port());
    } else {
      RCLCPP_INFO(rclcpp::get_logger("agv_web_gui"), "Web GUI aktif: http://%s:%d%s",
                  bridge.bindAddress().toUtf8().constData(), bridge.port(), bridge.readOnly() ? " [READ ONLY]" : "");
      if (bridge.bindAddress() != "127.0.0.1" && bridge.bindAddress() != "::1" && bridge.bindAddress() != "localhost") {
        RCLCPP_WARN(rclcpp::get_logger("agv_web_gui"),
                    "Web GUI di-bind non-loopback. Gunakan jaringan tepercaya; endpoint kontrol tidak memakai autentikasi eksternal.");
      }
      QTimer rosShutdownGuard;
      rosShutdownGuard.setInterval(200);
      QObject::connect(&rosShutdownGuard, &QTimer::timeout, &app, [&app]() {
        if (!rclcpp::ok()) app.quit();
      });
      rosShutdownGuard.start();
      result = app.exec();
    }
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("agv_web_gui"), "Web GUI fatal: %s", e.what());
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
