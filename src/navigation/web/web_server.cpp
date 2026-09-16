#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <deque>
#include <future>
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
#include <QCryptographicHash>
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
#include <QProcess>
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
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
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
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>
#include <yolo_obstacle_detection_ros2/msg/obstacle_array.hpp>
#include <yolo_obstacle_detection_ros2/msg/alignment_state.hpp>

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
      const QString runtimeConfig = workspace + "/config/runtime/" + packageName;
      if (QDir(runtimeConfig).exists()) return runtimeConfig;
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
      {"esc", esc + "/ackermann_1_board.yaml"},
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
  const QString previousScalar = (hash >= 0 ? rest.left(hash) : rest).trimmed();
  QString replacementScalar = jsonScalarInlineYaml(value);
  // JSON numbers do not retain integer-vs-floating lexical type. Preserve an
  // existing YAML float (e.g. 30.0) so ROS2 parameter type does not silently
  // change to integer after a GUI tuning write.
  if (value.isDouble() && (previousScalar.contains('.') || previousScalar.contains('e', Qt::CaseInsensitive))) {
    const double numeric = value.toDouble();
    if (std::isfinite(numeric) && std::floor(numeric) == numeric &&
        !replacementScalar.contains('.') && !replacementScalar.contains('e', Qt::CaseInsensitive)) {
      replacementScalar += QStringLiteral(".0");
    }
  }
  lines[target] = prefix + QStringLiteral(" ") + replacementScalar + comment;
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

QJsonValue currentYamlValue(const QString &fileKey, const QString &yamlPath) {
  const auto candidates = configCandidates();
  if (!candidates.contains(fileKey) || yamlPath.trimmed().isEmpty()) return QJsonValue();
  try {
    YAML::Node root = YAML::LoadFile(candidates.value(fileKey).toStdString());
    return yamlPathValue(root, yamlPath.split('.', Qt::SkipEmptyParts));
  } catch (...) {
    return QJsonValue();
  }
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


struct RuntimeParameterTarget {
  QString node;
  QString parameter;
};

std::optional<RuntimeParameterTarget> runtimeParameterTarget(const QString &fileKey, const QString &yamlPath) {
  if (fileKey == QStringLiteral("alignment") && !yamlPath.trimmed().isEmpty()) {
    return RuntimeParameterTarget{QStringLiteral("/hole_block_alignment_node"),
                                  QStringLiteral("cfg.") + yamlPath};
  }
  const QString token = QStringLiteral(".ros__parameters.");
  const int idx = yamlPath.indexOf(token);
  if (idx < 0) return std::nullopt;
  QString root = yamlPath.left(idx);
  const QString parameter = yamlPath.mid(idx + token.size());
  if (parameter.isEmpty()) return std::nullopt;
  if (root == QStringLiteral("/**")) {
    static const QMap<QString, QString> wildcardNodes{
      {"lidar", "/lidar_node"}, {"camera", "/astra_rgb_v4l2_node"},
      {"perception", "/obstacle_detector_node"}
    };
    if (!wildcardNodes.contains(fileKey)) return std::nullopt;
    return RuntimeParameterTarget{wildcardNodes.value(fileKey), parameter};
  }
  root.replace('.', '/');
  return RuntimeParameterTarget{QStringLiteral("/") + root, parameter};
}

QString runtimeValueArgument(const QJsonValue &value) {
  if (value.isBool()) return value.toBool() ? QStringLiteral("true") : QStringLiteral("false");
  if (value.isDouble()) return QString::number(value.toDouble(), 'g', 16);
  if (value.isString()) return value.toString();
  if (value.isArray()) return QString::fromUtf8(QJsonDocument(value.toArray()).toJson(QJsonDocument::Compact));
  if (value.isObject()) return QString::fromUtf8(QJsonDocument(value.toObject()).toJson(QJsonDocument::Compact));
  return QStringLiteral("null");
}

std::optional<QString> runtimeRestartPattern(const QString &fileKey) {
  static const QMap<QString, QString> patterns{
    {"lidar", "/install/navigation/lib/navigation/lidar_node"},
    {"imu", "/install/navigation/lib/navigation/imu_node"},
    {"ekf", "/robot_localization/ekf_node"},
    {"navigation_core", "/install/navigation/lib/navigation/autonomy_health_manager.py"},
    {"camera", "/install/yolo_obstacle_detection_ros2/lib/yolo_obstacle_detection_ros2/astra_rgb_v4l2_node"},
    {"perception", "/install/yolo_obstacle_detection_ros2/lib/yolo_obstacle_detection_ros2/obstacle_detector_node"},
    {"alignment", "/install/yolo_obstacle_detection_ros2/lib/yolo_obstacle_detection_ros2/hole_block_alignment_node.py"},
    {"esc", "/install/esc/lib/esc/esc_driver"},
    {"esc_mux", "/install/esc/lib/esc/esc_command_mux"},
    {"teleop", "/install/esc/lib/esc/keyboard_teleop"},
    {"winch", "/install/esc/lib/esc/winch_serial_node"},
  };
  if (!patterns.contains(fileKey)) return std::nullopt;
  return patterns.value(fileKey);
}

QVector<qint64> matchingOwnerPids(const QString &pattern) {
  QVector<qint64> out;
  QProcess proc;
  proc.setProcessChannelMode(QProcess::MergedChannels);
  proc.start(QStringLiteral("pgrep"), {QStringLiteral("-f"), pattern});
  if (!proc.waitForStarted(800) || !proc.waitForFinished(1800) || proc.exitCode() != 0) return out;
  const QStringList lines = QString::fromUtf8(proc.readAll()).split('\n', Qt::SkipEmptyParts);
  for (const QString &line : lines) {
    bool ok = false;
    const qint64 pid = line.trimmed().toLongLong(&ok);
    if (ok && pid > 1) out.push_back(pid);
  }
  return out;
}

bool restartRuntimeOwner(const QString &fileKey, QString *detail) {
  const auto pattern = runtimeRestartPattern(fileKey);
  if (!pattern) return false;
  const QVector<qint64> oldPids = matchingOwnerPids(*pattern);
  if (oldPids.isEmpty()) {
    if (detail) *detail = QStringLiteral("owner process tidak aktif; restart tidak dapat diverifikasi");
    return false;
  }
  QProcess proc;
  proc.setProcessChannelMode(QProcess::MergedChannels);
  proc.start(QStringLiteral("pkill"), {QStringLiteral("-TERM"), QStringLiteral("-f"), *pattern});
  if (!proc.waitForStarted(800) || !proc.waitForFinished(2500) || proc.exitCode() != 0) {
    if (detail) *detail = QStringLiteral("restart command gagal/timeout");
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (std::chrono::steady_clock::now() < deadline) {
    const QVector<qint64> nowPids = matchingOwnerPids(*pattern);
    for (qint64 pid : nowPids) {
      if (!oldPids.contains(pid)) {
        if (detail) *detail = QStringLiteral("owner process respawn terverifikasi PID baru=%1").arg(pid);
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  if (detail) *detail = QStringLiteral("owner process tidak respawn dalam 15 s");
  return false;
}

std::optional<QString> sha256FileHex(const QString &path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) return std::nullopt;
  QCryptographicHash hash(QCryptographicHash::Sha256);
  if (!hash.addData(&file)) return std::nullopt;
  return QString::fromLatin1(hash.result().toHex());
}

std::optional<QString> alignmentRuntimeConfigSha256() {
  QFile file(QStringLiteral("/dev/shm/agv_alignment_status.json"));
  if (!file.open(QIODevice::ReadOnly)) return std::nullopt;
  QJsonParseError error{};
  const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &error);
  if (error.error != QJsonParseError::NoError || !doc.isObject()) return std::nullopt;
  const QString value = doc.object().value(QStringLiteral("config_sha256")).toString().trimmed();
  if (value.size() != 64) return std::nullopt;
  return value;
}

bool waitAlignmentConfigSha256(const QString &expected, QString *actual) {
  for (int attempt = 0; attempt < 24; ++attempt) {
    const auto got = alignmentRuntimeConfigSha256();
    if (got) {
      if (actual) *actual = *got;
      if (got->compare(expected, Qt::CaseInsensitive) == 0) return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  return false;
}

std::optional<QString> readRuntimeParameter(const RuntimeParameterTarget &target) {
  for (int attempt = 0; attempt < 3; ++attempt) {
    QProcess proc;
    proc.setProcessChannelMode(QProcess::MergedChannels);
    proc.start(QStringLiteral("ros2"), {QStringLiteral("param"), QStringLiteral("get"),
      QStringLiteral("--no-daemon"), QStringLiteral("--spin-time"), QStringLiteral("1.0"),
      QStringLiteral("--hide-type"), target.node, target.parameter});
    if (proc.waitForStarted(1000) && proc.waitForFinished(5500) &&
        proc.exitStatus() == QProcess::NormalExit && proc.exitCode() == 0) {
      return QString::fromUtf8(proc.readAll()).trimmed();
    }
    proc.kill(); proc.waitForFinished(200);
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
  }
  return std::nullopt;
}

bool runtimeValueMatches(const QJsonValue &value, QString actual) {
  actual = actual.trimmed();
  if (value.isBool()) return actual.compare(value.toBool() ? "true" : "false", Qt::CaseInsensitive) == 0;
  if (value.isDouble()) {
    bool ok = false; const double got = actual.toDouble(&ok);
    return ok && std::abs(got - value.toDouble()) <= std::max(1e-9, std::abs(value.toDouble()) * 1e-9);
  }
  if (value.isString()) return actual == value.toString();
  const QString expected = runtimeValueArgument(value).simplified();
  return actual.simplified() == expected;
}

QJsonValue parameterValueToJson(const rcl_interfaces::msg::ParameterValue &value) {
  using rcl_interfaces::msg::ParameterType;
  switch (value.type) {
    case ParameterType::PARAMETER_BOOL: return value.bool_value;
    case ParameterType::PARAMETER_INTEGER: return static_cast<double>(value.integer_value);
    case ParameterType::PARAMETER_DOUBLE: return value.double_value;
    case ParameterType::PARAMETER_STRING: return QString::fromStdString(value.string_value);
    case ParameterType::PARAMETER_BYTE_ARRAY: {
      QJsonArray out; for (const auto v : value.byte_array_value) out.append(static_cast<int>(v)); return out;
    }
    case ParameterType::PARAMETER_BOOL_ARRAY: {
      QJsonArray out; for (const auto v : value.bool_array_value) out.append(v); return out;
    }
    case ParameterType::PARAMETER_INTEGER_ARRAY: {
      QJsonArray out; for (const auto v : value.integer_array_value) out.append(static_cast<double>(v)); return out;
    }
    case ParameterType::PARAMETER_DOUBLE_ARRAY: {
      QJsonArray out; for (const auto v : value.double_array_value) out.append(v); return out;
    }
    case ParameterType::PARAMETER_STRING_ARRAY: {
      QJsonArray out; for (const auto &v : value.string_array_value) out.append(QString::fromStdString(v)); return out;
    }
    default: return QJsonValue();
  }
}

bool jsonValueMatches(const QJsonValue &expected, const QJsonValue &actual) {
  if (expected.isDouble() && actual.isDouble()) {
    const double a = expected.toDouble(), b = actual.toDouble();
    return std::isfinite(a) && std::isfinite(b) &&
           std::abs(a - b) <= std::max(1e-9, std::max(std::abs(a), std::abs(b)) * 1e-9);
  }
  if (expected.isArray() && actual.isArray()) {
    const auto a = expected.toArray(), b = actual.toArray();
    if (a.size() != b.size()) return false;
    for (int i = 0; i < a.size(); ++i) if (!jsonValueMatches(a[i], b[i])) return false;
    return true;
  }
  return expected == actual;
}

bool jsonToParameterValueLike(const QJsonValue &input, std::uint8_t type,
                              rcl_interfaces::msg::ParameterValue *out, QString *error) {
  using rcl_interfaces::msg::ParameterType;
  if (!out) return false;
  out->type = type;
  if (type == ParameterType::PARAMETER_BOOL && input.isBool()) { out->bool_value = input.toBool(); return true; }
  if (type == ParameterType::PARAMETER_INTEGER && input.isDouble()) {
    const double v = input.toDouble();
    if (!std::isfinite(v) || std::abs(v - std::round(v)) > 1e-9) {
      if (error) *error = QStringLiteral("Parameter integer membutuhkan bilangan bulat");
      return false;
    }
    out->integer_value = static_cast<std::int64_t>(std::llround(v)); return true;
  }
  if (type == ParameterType::PARAMETER_DOUBLE && input.isDouble()) {
    const double v = input.toDouble(); if (!std::isfinite(v)) { if (error) *error = "Nilai double tidak finite"; return false; }
    out->double_value = v; return true;
  }
  if (type == ParameterType::PARAMETER_STRING && input.isString()) {
    out->string_value = input.toString().toStdString(); return true;
  }
  if (!input.isArray()) {
    if (error) *error = QStringLiteral("Tipe nilai GUI tidak sesuai dengan tipe parameter runtime");
    return false;
  }
  const QJsonArray array = input.toArray();
  if (type == ParameterType::PARAMETER_BYTE_ARRAY) {
    for (const auto &x : array) { if (!x.isDouble() || x.toDouble() < 0 || x.toDouble() > 255) return false; out->byte_array_value.push_back(static_cast<std::uint8_t>(x.toInt())); }
    return true;
  }
  if (type == ParameterType::PARAMETER_BOOL_ARRAY) {
    for (const auto &x : array) { if (!x.isBool()) return false; out->bool_array_value.push_back(x.toBool()); } return true;
  }
  if (type == ParameterType::PARAMETER_INTEGER_ARRAY) {
    for (const auto &x : array) { if (!x.isDouble() || std::abs(x.toDouble()-std::round(x.toDouble()))>1e-9) return false; out->integer_array_value.push_back(static_cast<std::int64_t>(std::llround(x.toDouble()))); } return true;
  }
  if (type == ParameterType::PARAMETER_DOUBLE_ARRAY) {
    for (const auto &x : array) { if (!x.isDouble() || !std::isfinite(x.toDouble())) return false; out->double_array_value.push_back(x.toDouble()); } return true;
  }
  if (type == ParameterType::PARAMETER_STRING_ARRAY) {
    for (const auto &x : array) { if (!x.isString()) return false; out->string_array_value.push_back(x.toString().toStdString()); } return true;
  }
  if (error) *error = QStringLiteral("Tipe parameter runtime tidak didukung oleh GUI");
  return false;
}

bool nodeVisibleFrom(const rclcpp::Node::SharedPtr &node, const QString &fullName) {
  if (!node) return false;
  const std::string wanted = fullName.toStdString();
  for (const auto &name : node->get_node_names()) if (name == wanted) return true;
  return false;
}

std::optional<rcl_interfaces::msg::ParameterValue> nativeReadParameter(
    const rclcpp::Node::SharedPtr &node, const RuntimeParameterTarget &target, QString *detail) {
  if (!node) { if (detail) *detail = "ROS node GUI tidak tersedia"; return std::nullopt; }
  const QString serviceName = target.node + QStringLiteral("/get_parameters");
  auto client = node->create_client<rcl_interfaces::srv::GetParameters>(serviceName.toStdString());
  if (!client->wait_for_service(2200ms)) {
    if (detail) *detail = QStringLiteral("Service tidak tersedia: ") + serviceName;
    return std::nullopt;
  }
  QString lastError;
  for (int attempt = 1; attempt <= 3; ++attempt) {
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back(target.parameter.toStdString());
    auto future = client->async_send_request(request);
    if (future.wait_for(4200ms) == std::future_status::ready) {
      try {
        const auto response = future.get();
        if (response && !response->values.empty()) return response->values.front();
        lastError = QStringLiteral("Parameter tidak ditemukan pada runtime: ") + target.parameter;
      } catch (const std::exception &e) {
        lastError = QStringLiteral("Native get exception: ") + QString::fromUtf8(e.what());
      }
    } else {
      lastError = QStringLiteral("Timeout native get_parameters attempt %1/3: ").arg(attempt) + target.node;
    }
    if (attempt < 3) std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  if (detail) *detail = lastError;
  return std::nullopt;
}

QJsonObject applyNav2NativeParameter(const rclcpp::Node::SharedPtr &node,
                                     const RuntimeParameterTarget &target,
                                     const QJsonValue &value) {
  QString detail;
  const bool visible = nodeVisibleFrom(node, target.node);
  const auto before = nativeReadParameter(node, target, &detail);
  if (!before) {
    // PlannerServer can be intentionally held in the unconfigured lifecycle
    // state by the autonomous safety gates. In that state its parameter
    // services may exist in the graph but not service requests promptly. Do
    // not label this as a failed edit: persist it for the next planner process
    // start. Other visible Nav2 nodes still report a real reload-required error.
    const bool plannerDeferred = target.node == QStringLiteral("/planner_server");
    const bool deferred = !visible || plannerDeferred;
    const QString msg = plannerDeferred
        ? QStringLiteral("Planner runtime belum siap merespons parameter; nilai disimpan untuk start planner berikutnya")
        : detail;
    return QJsonObject{{"attempted", visible && !plannerDeferred}, {"applied", false}, {"verified", false},
      {"strategy", "native_parameter_service"}, {"state", deferred ? "NEXT_START" : "RELOAD_REQUIRED"},
      {"node", target.node}, {"parameter", target.parameter}, {"message", msg}};
  }

  rcl_interfaces::msg::Parameter parameter;
  parameter.name = target.parameter.toStdString();
  QString typeError;
  if (!jsonToParameterValueLike(value, before->type, &parameter.value, &typeError)) {
    return QJsonObject{{"attempted", true}, {"applied", false}, {"verified", false},
      {"strategy", "native_parameter_service"}, {"state", "VERIFY_FAILED"},
      {"node", target.node}, {"parameter", target.parameter}, {"message", typeError},
      {"previous_value", parameterValueToJson(*before)}};
  }

  const QString serviceName = target.node + QStringLiteral("/set_parameters_atomically");
  auto client = node->create_client<rcl_interfaces::srv::SetParametersAtomically>(serviceName.toStdString());
  if (!client->wait_for_service(1800ms)) {
    return QJsonObject{{"attempted", true}, {"applied", false}, {"verified", false},
      {"strategy", "native_parameter_service"}, {"state", "RELOAD_REQUIRED"},
      {"node", target.node}, {"parameter", target.parameter},
      {"previous_value", parameterValueToJson(*before)},
      {"message", QStringLiteral("Native set service tidak tersedia: ") + serviceName}};
  }
  bool setSucceeded = false;
  QString setError;
  for (int attempt = 1; attempt <= 2 && !setSucceeded; ++attempt) {
    auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    request->parameters.push_back(parameter);
    auto future = client->async_send_request(request);
    if (future.wait_for(4500ms) != std::future_status::ready) {
      setError = QStringLiteral("Timeout native set_parameters_atomically attempt %1/2").arg(attempt);
    } else {
      try {
        const auto response = future.get();
        if (response && response->result.successful) setSucceeded = true;
        else setError = QStringLiteral("Nav2 menolak parameter: ") +
                        (response ? QString::fromStdString(response->result.reason) : QStringLiteral("no response"));
      } catch (const std::exception &e) {
        setError = QStringLiteral("Native set exception: ") + QString::fromUtf8(e.what());
      }
    }
    if (!setSucceeded && attempt < 2) std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  if (!setSucceeded) {
    return QJsonObject{{"attempted", true}, {"applied", false}, {"verified", false},
      {"strategy", "native_parameter_service"}, {"state", "RELOAD_REQUIRED"},
      {"node", target.node}, {"parameter", target.parameter},
      {"previous_value", parameterValueToJson(*before)},
      {"message", setError + QStringLiteral("; YAML belum diubah")}};
  }

  QString readDetail;
  const auto after = nativeReadParameter(node, target, &readDetail);
  const QJsonValue actual = after ? parameterValueToJson(*after) : QJsonValue();
  const bool verified = after.has_value() && jsonValueMatches(value, actual);
  return QJsonObject{{"attempted", true}, {"applied", verified}, {"verified", verified},
    {"strategy", "native_parameter_service"}, {"state", verified ? "APPLIED" : "VERIFY_FAILED"},
    {"node", target.node}, {"parameter", target.parameter},
    {"previous_value", parameterValueToJson(*before)}, {"actual_value", actual},
    {"message", verified ? "Parameter Nav2 diterapkan via native ROS2 service dan read-back terverifikasi" :
                            QStringLiteral("Native set diterima tetapi read-back gagal: ") + readDetail}};
}

bool nav2PlannerOwnedTarget(const RuntimeParameterTarget &target) {
  // SmacPlannerHybrid and Costmap2D expose native parameter callbacks while
  // configured/active. Prefer verified hot-apply; the restart helper remains
  // available only as an explicit fallback, never as the default SAVE path.
  Q_UNUSED(target);
  return false;
}

QString compactJsonValue(const QJsonValue &value) {
  QJsonArray wrapped;
  wrapped.append(value);
  QByteArray bytes = QJsonDocument(wrapped).toJson(QJsonDocument::Compact);
  if (bytes.size() >= 2 && bytes.front() == '[' && bytes.back() == ']')
    bytes = bytes.mid(1, bytes.size() - 2);
  return QString::fromUtf8(bytes);
}

QJsonObject applyPlannerReloadVerified(const RuntimeParameterTarget &target, const QJsonValue &value) {
  const QString helper = QStringLiteral("/home/otomasi2/ros/src/navigation/scripts/nav2_param_reload_apply.py");
  if (!QFileInfo::exists(helper)) {
    return QJsonObject{{"attempted", true}, {"applied", false}, {"verified", false},
      {"strategy", "planner_restart_configure_verify"}, {"state", "HELPER_MISSING"},
      {"node", target.node}, {"parameter", target.parameter},
      {"message", QStringLiteral("Helper reload Nav2 tidak ditemukan: ") + helper}};
  }
  QProcess proc;
  proc.setProcessChannelMode(QProcess::SeparateChannels);
  proc.start(QStringLiteral("/usr/bin/python3"), {helper,
    QStringLiteral("--target-node"), target.node,
    QStringLiteral("--parameter"), target.parameter,
    QStringLiteral("--expected-json"), compactJsonValue(value)});
  if (!proc.waitForStarted(1500)) {
    return QJsonObject{{"attempted", true}, {"applied", false}, {"verified", false},
      {"strategy", "planner_restart_configure_verify"}, {"state", "HELPER_START_FAILED"},
      {"node", target.node}, {"parameter", target.parameter},
      {"message", "Gagal menjalankan helper reload planner"}};
  }
  if (!proc.waitForFinished(115000)) {
    proc.kill(); proc.waitForFinished(1000);
    return QJsonObject{{"attempted", true}, {"applied", false}, {"verified", false},
      {"strategy", "planner_restart_configure_verify"}, {"state", "HELPER_TIMEOUT"},
      {"node", target.node}, {"parameter", target.parameter},
      {"message", "Timeout reload/configure/verify planner; transaksi akan di-rollback"}};
  }
  const QByteArray stdoutBytes = proc.readAllStandardOutput().trimmed();
  const QString stderrText = QString::fromUtf8(proc.readAllStandardError()).trimmed();
  const QList<QByteArray> lines = stdoutBytes.split('\n');
  QJsonObject result;
  for (int i = lines.size() - 1; i >= 0; --i) {
    QJsonParseError error{};
    const QJsonDocument doc = QJsonDocument::fromJson(lines[i].trimmed(), &error);
    if (error.error == QJsonParseError::NoError && doc.isObject()) { result = doc.object(); break; }
  }
  if (result.isEmpty()) {
    return QJsonObject{{"attempted", true}, {"applied", false}, {"verified", false},
      {"strategy", "planner_restart_configure_verify"}, {"state", "HELPER_INVALID_RESPONSE"},
      {"node", target.node}, {"parameter", target.parameter}, {"stderr", stderrText},
      {"message", "Helper reload planner tidak memberikan JSON hasil verifikasi"}};
  }
  result["attempted"] = true;
  result["applied"] = result.value("verified").toBool(false);
  result["node"] = target.node;
  result["parameter"] = target.parameter;
  if (!stderrText.isEmpty()) result["stderr"] = stderrText;
  return result;
}

QJsonObject applyRuntimeParameter(const QString &fileKey, const QString &yamlPath, const QJsonValue &value,
                                  const rclcpp::Node::SharedPtr &rosNode = nullptr) {
  // SLAM parameters are intentionally applied on the next mapping start. Restarting
  // slam_toolbox inside an active mapping session would destroy the current pose graph.
  if (fileKey == QStringLiteral("slam")) {
    return QJsonObject{{"attempted", false}, {"applied", false}, {"verified", false},
      {"strategy", "next_mapping"}, {"state", "NEXT_MAPPING"},
      {"message", "YAML tersimpan; parameter akan dipakai otomatis pada START MAPPING berikutnya"}};
  }

  if (fileKey == QStringLiteral("alignment")) {
    QString restartDetail;
    const bool restarted = restartRuntimeOwner(fileKey, &restartDetail);
    if (!restarted) return QJsonObject{{"attempted", true}, {"applied", false}, {"verified", false},
      {"strategy", "node_restart_sha256"}, {"state", "NEXT_START"}, {"message", restartDetail}};
    const QString filePath = configCandidates().value(fileKey);
    const auto expected = sha256FileHex(filePath);
    QString actualHash;
    const bool verified = expected.has_value() && waitAlignmentConfigSha256(*expected, &actualHash);
    return QJsonObject{{"attempted", true}, {"applied", verified}, {"verified", verified},
      {"strategy", "node_restart_sha256"}, {"state", verified ? "APPLIED" : "VERIFY_FAILED"},
      {"actual_value", verified ? value : QJsonValue()},
      {"runtime_config_sha256", actualHash},
      {"message", verified ? "Alignment node direstart dan SHA-256 config runtime cocok; nilai algoritma terverifikasi aktif" :
                              "Alignment node direstart tetapi SHA-256 config runtime belum cocok; jangan anggap nilai aktif"}};
  }

  const auto target = runtimeParameterTarget(fileKey, yamlPath);

  // EKF and autonomy-health parameters are backed by live ROS2 parameter
  // services. Never kill these owners from the tuning GUI: EKF/health are not
  // launch-respawned in the current stack. Apply atomically and read back.
  if ((fileKey == QStringLiteral("ekf") || fileKey == QStringLiteral("navigation_core")) &&
      target && rosNode) {
    return applyNav2NativeParameter(rosNode, *target, value);
  }

  const auto restartPattern = runtimeRestartPattern(fileKey);
  if (!target && restartPattern) {
    QString restartDetail;
    const bool restarted = restartRuntimeOwner(fileKey, &restartDetail);
    return QJsonObject{{"attempted", true}, {"applied", restarted}, {"verified", restarted},
      {"strategy", "node_restart"}, {"state", restarted ? "APPLIED_RESTART" : "NEXT_START"},
      {"message", restarted ? "Config aktif diterapkan melalui restart node terisolasi" : restartDetail}};
  }
  if (!target) return QJsonObject{{"attempted", false}, {"applied", false}, {"verified", false},
      {"strategy", "next_start"}, {"state", "NEXT_START"},
      {"message", "Runtime target tidak ada; nilai akan dipakai pada start/reload berikutnya"}};

  // Custom nodes cache parameters in member variables at startup. Respawn them so
  // the algorithm, not only the ROS parameter server, consumes the new YAML value.
  if (restartPattern) {
    QString restartDetail;
    const bool restarted = restartRuntimeOwner(fileKey, &restartDetail);
    if (!restarted) return QJsonObject{{"attempted", true}, {"applied", false}, {"verified", false},
      {"strategy", "node_restart"}, {"state", "NEXT_START"}, {"node", target->node},
      {"parameter", target->parameter}, {"message", restartDetail}};
    if ((fileKey == QStringLiteral("lidar") || fileKey == QStringLiteral("imu")) && rosNode) {
      // A new driver PID appears before its ROS parameter services are ready.
      // Give the respawned owner time to initialise, then retry native read-back.
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      QString readDetail;
      std::optional<rcl_interfaces::msg::ParameterValue> actualNative;
      for (int attempt = 0; attempt < 4 && !actualNative; ++attempt) {
        actualNative = nativeReadParameter(rosNode, *target, &readDetail);
        if (!actualNative && attempt < 3)
          std::this_thread::sleep_for(std::chrono::milliseconds(700));
      }
      const QJsonValue actual = actualNative ? parameterValueToJson(*actualNative) : QJsonValue();
      const bool verified = actualNative.has_value() && jsonValueMatches(value, actual);
      const QString sensorName = fileKey == QStringLiteral("lidar") ? QStringLiteral("LiDAR") : QStringLiteral("IMU");
      return QJsonObject{{"attempted", true}, {"applied", verified}, {"verified", verified},
        {"strategy", "sensor_restart_native_verify"}, {"state", verified ? "APPLIED_RESTART" : "VERIFY_FAILED"},
        {"node", target->node}, {"parameter", target->parameter}, {"actual_value", actual},
        {"message", verified ? restartDetail + QStringLiteral("; parameter ") + sensorName + QStringLiteral(" read-back cocok") :
                                restartDetail + QStringLiteral("; read-back ") + sensorName + QStringLiteral(" gagal: ") + readDetail}};
    }
    const auto actual = readRuntimeParameter(*target);
    const bool verified = actual.has_value() && runtimeValueMatches(value, *actual);
    return QJsonObject{{"attempted", true}, {"applied", verified}, {"verified", verified},
      {"strategy", "node_restart"}, {"state", verified ? "APPLIED" : "VERIFY_FAILED"},
      {"node", target->node}, {"parameter", target->parameter},
      {"actual_value", actual ? QJsonValue(*actual) : QJsonValue()},
      {"message", verified ? "Node direstart terisolasi dan nilai runtime terverifikasi" :
                              "Node sudah direstart tetapi runtime read-back belum sama; jangan anggap nilai aktif"}};
  }

  // Navigation-only path: use the already-running GUI ROS node and native
  // parameter services. Avoid short-lived `ros2 param` CLI discovery on Jetson.
  if (fileKey == QStringLiteral("nav2") && rosNode) {
    return applyNav2NativeParameter(rosNode, *target, value);
  }

  // Legacy path is retained for non-navigation owners so working perception/
  // steering behavior is not changed by this navigation-only fix.
  QProcess proc;
  proc.setProcessChannelMode(QProcess::MergedChannels);
  proc.start(QStringLiteral("ros2"), {QStringLiteral("param"), QStringLiteral("set"),
    QStringLiteral("--no-daemon"), QStringLiteral("--spin-time"), QStringLiteral("1.0"),
    target->node, target->parameter, runtimeValueArgument(value)});
  if (!proc.waitForStarted(1000) || !proc.waitForFinished(5500)) {
    proc.kill(); proc.waitForFinished(300);
    return QJsonObject{{"attempted", true}, {"applied", false}, {"verified", false},
      {"strategy", "dynamic"}, {"state", "RELOAD_REQUIRED"}, {"node", target->node},
      {"parameter", target->parameter}, {"message", "Dynamic apply timeout; YAML tersimpan, lifecycle reload diperlukan"}};
  }
  const QString output = QString::fromUtf8(proc.readAll()).trimmed();
  const bool setOk = proc.exitStatus() == QProcess::NormalExit && proc.exitCode() == 0 &&
                     output.contains(QStringLiteral("Successful"), Qt::CaseInsensitive);
  if (!setOk) return QJsonObject{{"attempted", true}, {"applied", false}, {"verified", false},
    {"strategy", "dynamic"}, {"state", "RELOAD_REQUIRED"}, {"node", target->node},
    {"parameter", target->parameter}, {"output", output},
    {"message", "Node menolak dynamic update; YAML tersimpan dan lifecycle reload diperlukan"}};

  const auto actual = readRuntimeParameter(*target);
  const bool verified = actual.has_value() && runtimeValueMatches(value, *actual);
  return QJsonObject{{"attempted", true}, {"applied", verified}, {"verified", verified},
    {"strategy", "dynamic"}, {"state", verified ? "APPLIED" : "VERIFY_FAILED"},
    {"node", target->node}, {"parameter", target->parameter}, {"output", output},
    {"actual_value", actual ? QJsonValue(*actual) : QJsonValue()},
    {"message", verified ? "Parameter runtime berhasil diterapkan dan read-back terverifikasi" :
                            "Set diterima tetapi read-back tidak sama; nilai belum dianggap aktif"}};
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
    // 8-direction calibration 2026-09-09. /imu/mag is already hard/soft-iron corrected by imu_node.
    magHeadingOffsetRad_ = node_->declare_parameter<double>("mag_heading_offset_rad", 0.013525733461806364);
    magHeadingSign_ = node_->declare_parameter<double>("mag_heading_sign", 1.0) >= 0.0 ? 1.0 : -1.0;
    magHeadingRmseDeg_ = node_->declare_parameter<double>("mag_heading_validation_rmse_deg", 3.0407979290635105);
    setupRos();
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
    executor_->add_node(node_);
    thread_ = std::thread([this]() {
      while (rclcpp::ok() && !stop_.load()) executor_->spin_once(100ms);
    });
    // Parameter transactions use a dedicated ROS node/executor so camera, map,
    // scan, costmap and SSE telemetry callbacks cannot starve Nav2 read-back.
    rclcpp::NodeOptions parameterOptions;
    parameterOptions.use_global_arguments(false);
    parameterNode_ = std::make_shared<rclcpp::Node>("agv_web_gui_parameter_client", parameterOptions);
    parameterExecutor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    parameterExecutor_->add_node(parameterNode_);
    parameterThread_ = std::thread([this]() { parameterExecutor_->spin(); });
    update("server", QJsonObject{{"ros", true}, {"read_only", readOnly_}, {"port", port_},
                                  {"bind_address", bindAddress_}, {"started_at_ms", nowMs()}});
  }

  ~WebRosBridge() {
    stop_.store(true);
    if (executor_) executor_->cancel();
    if (parameterExecutor_) parameterExecutor_->cancel();
    if (thread_.joinable()) thread_.join();
    if (parameterThread_.joinable()) parameterThread_.join();
    if (executor_ && node_) executor_->remove_node(node_);
    if (parameterExecutor_ && parameterNode_) parameterExecutor_->remove_node(parameterNode_);
  }

  QString bindAddress() const { return bindAddress_; }
  int port() const { return port_; }
  bool readOnly() const { return readOnly_; }
  rclcpp::Node::SharedPtr rosNode() const { return node_; }
  rclcpp::Node::SharedPtr parameterNode() const { return parameterNode_; }

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

  QByteArray globalCostmapPng() const {
    std::lock_guard<std::mutex> lock(mediaMutex_);
    return globalCostmapPng_;
  }

  QByteArray localCostmapPng() const {
    std::lock_guard<std::mutex> lock(mediaMutex_);
    return localCostmapPng_;
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

  bool triggerServiceSync(const QString &service, const QString &tag, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    auto client = node_->create_client<std_srvs::srv::Trigger>(service.toStdString());
    if (!client->wait_for_service(800ms)) {
      if (message) *message = "Service belum tersedia: " + service;
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(3000ms) != std::future_status::ready) {
      if (message) *message = "Timeout menunggu hasil service: " + service;
      update("web_action", QJsonObject{{"ok", false}, {"action", tag}, {"service", service},
                                       {"message", *message}, {"at_ms", nowMs()}});
      return false;
    }
    try {
      const auto result = future.get();
      const QString resultMessage = QString::fromStdString(result->message);
      if (message) *message = resultMessage;
      update("web_action", QJsonObject{{"ok", result->success}, {"action", tag}, {"service", service},
                                       {"message", resultMessage}, {"at_ms", nowMs()}});
      return result->success;
    } catch (const std::exception &e) {
      if (message) *message = QString::fromUtf8(e.what());
      update("web_action", QJsonObject{{"ok", false}, {"action", tag}, {"service", service},
                                       {"message", QString::fromUtf8(e.what())}, {"at_ms", nowMs()}});
      return false;
    }
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
  rclcpp::Node::SharedPtr parameterNode_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> parameterExecutor_;
  std::thread parameterThread_;
  std::atomic_bool stop_{false};
  QString bindAddress_;
  int port_{5005};
  bool readOnly_{false};
  double cameraJpegFps_{5.0};
  double magHeadingOffsetRad_{0.0};
  double magHeadingSign_{1.0};
  double magHeadingRmseDeg_{0.0};
  mutable std::mutex stateMutex_;
  QJsonObject state_;
  QJsonObject updated_;
  QSet<QString> dirty_;
  mutable std::mutex mediaMutex_;
  std::mutex cameraEncodeMutex_;
  QByteArray cameraJpeg_;
  QByteArray mapPng_;
  QByteArray globalCostmapPng_;
  QByteArray localCostmapPng_;
  std::chrono::steady_clock::time_point lastCameraEncode_{};
  std::chrono::steady_clock::time_point lastAnnotatedCameraFrame_{};
  std::mutex lidarRateMutex_;
  std::deque<double> lidarScanPeriods_;
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

    // Perception obstacle-corridor geometry. The footprint comes from the same
    // vehicle_geometry.yaml authority used to synchronize URDF/Nav2. Speed is
    // intentionally NOT used here: this is a static geometry/measurement test
    // until measured ESC odometry vx is available.
    double obstacleHalfLengthM = 0.65;
    double obstacleHalfWidthM = 0.40;
    constexpr double obstacleLateralMarginM = 0.10;
    constexpr double obstacleFrontMarginM = 0.05;
    constexpr double obstacleTestHorizonM = 5.0;
    constexpr int obstacleMinClusterHits = 2;
    QString obstacleGeometrySource = QStringLiteral("vehicle_geometry.yaml default");
    try {
      const QString geometryPath = configCandidates().value(QStringLiteral("vehicle"));
      if (!geometryPath.isEmpty() && QFileInfo::exists(geometryPath)) {
        const YAML::Node root = YAML::LoadFile(geometryPath.toStdString());
        const YAML::Node vehicle = root["vehicle"];
        if (vehicle && vehicle["footprint_half_length_m"])
          obstacleHalfLengthM = vehicle["footprint_half_length_m"].as<double>();
        if (vehicle && vehicle["footprint_half_width_m"])
          obstacleHalfWidthM = vehicle["footprint_half_width_m"].as<double>();
        obstacleGeometrySource = geometryPath;
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(node_->get_logger(), "Obstacle corridor geometry fallback: %s", e.what());
    }
    const double obstacleCorridorHalfWidthM = obstacleHalfWidthM + obstacleLateralMarginM;
    const double obstacleFrontBoundaryM = obstacleHalfLengthM + obstacleFrontMarginM;

    // Lightweight static/dynamic behavior classifier. Geometry stays LiDAR-first:
    // a fresh on-path person semantic makes the blocking return DYNAMIC, while a
    // spatially stable corridor cluster becomes STATIC (box/object). No extra CNN
    // is added for cardboard boxes, keeping GPU/RAM load bounded.
    struct CorridorBehaviorState {
      std::mutex mutex;
      bool anchorValid{false};
      double anchorX{0.0};
      double anchorY{0.0};
      double stableSinceMs{0.0};
      int stableScans{0};
    };
    const auto corridorBehaviorState = std::make_shared<CorridorBehaviorState>();
    const auto lastDynamicPersonOnPathMs = std::make_shared<std::atomic<double>>(0.0);
    constexpr double staticPositionToleranceM = 0.22;
    constexpr double staticPersistenceSec = 1.50;
    constexpr int staticMinStableScans = 5;
    constexpr double dynamicPersonFreshSec = 1.60;

    RCLCPP_INFO(node_->get_logger(),
      "Obstacle corridor TEST geometry: footprint %.2fx%.2f m, corridor width %.2f m, front boundary %.2f m",
      2.0 * obstacleHalfLengthM, 2.0 * obstacleHalfWidthM,
      2.0 * obstacleCorridorHalfWidthM, obstacleFrontBoundaryM);

    // Dedicated BAB 4.1 raw-stream timing.  Keep this independent from the
    // operational navigation telemetry so raw acquisition never changes the
    // /scan_nav or /imu/data paths used by SLAM/AMCL/EKF/Nav2.
    struct RawTimingState {
      std::mutex mutex;
      double previousStampSec{std::numeric_limits<double>::quiet_NaN()};
      std::deque<double> periodsSec;
      std::uint64_t messageCount{0};
    };
    const auto rawTimingFields = [](const std::shared_ptr<RawTimingState> &state, const auto &stamp) {
      QJsonObject out;
      const double stampSec = double(stamp.sec) + double(stamp.nanosec) * 1e-9;
      const double wallSec = std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      double periodSec = std::numeric_limits<double>::quiet_NaN();
      double rateHz = std::numeric_limits<double>::quiet_NaN();
      double jitterMs = std::numeric_limits<double>::quiet_NaN();
      std::uint64_t count = 0;
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (std::isfinite(state->previousStampSec) && stampSec > state->previousStampSec) {
          periodSec = stampSec - state->previousStampSec;
          if (periodSec > 0.001 && periodSec < 5.0) {
            state->periodsSec.push_back(periodSec);
            while (state->periodsSec.size() > 120U) state->periodsSec.pop_front();
          }
        }
        state->previousStampSec = stampSec;
        ++state->messageCount;
        count = state->messageCount;
        if (!state->periodsSec.empty()) {
          const double mean = std::accumulate(state->periodsSec.begin(), state->periodsSec.end(), 0.0) /
                              static_cast<double>(state->periodsSec.size());
          if (mean > 0.0) rateHz = 1.0 / mean;
          double sumSq = 0.0;
          for (double value : state->periodsSec) sumSq += (value - mean) * (value - mean);
          jitterMs = 1000.0 * std::sqrt(sumSq / static_cast<double>(state->periodsSec.size()));
        }
      }
      out["rate_hz"] = std::isfinite(rateHz) ? QJsonValue(rateHz) : QJsonValue();
      out["period_ms"] = std::isfinite(periodSec) ? QJsonValue(periodSec * 1000.0) : QJsonValue();
      out["timestamp_jitter_ms"] = std::isfinite(jitterMs) ? QJsonValue(jitterMs) : QJsonValue();
      out["latency_ms"] = stampSec > 0.0 ? QJsonValue((wallSec - stampSec) * 1000.0) : QJsonValue();
      out["message_count"] = static_cast<double>(count);
      out["measurement_stamp_sec"] = stampSec;
      return out;
    };

    const std::vector<std::pair<const char *, const char *>> bools = {
        {"/gnss/connected", "connected.gnss"}, {"/imu/connected", "connected.imu"},
        {"/winch/connected", "connected.winch"},
        {"/winch/top_limit", "winch.top_limit"}, {"/winch/bottom_limit", "winch.bottom_limit"},
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
        {"/winch/port", "winch.port"}, {"/winch/state", "winch.state"}, {"/winch/raw", "winch.raw"},
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

    // BAB 4.1 raw IMU channel: decoded engineering units BEFORE bias /
    // hard-soft-iron calibration.  Do not feed this channel into navigation.
    const auto rawImuTiming = std::make_shared<RawTimingState>();
    subscribe<sensor_msgs::msg::Imu>("/imu/raw/data", sensorQos,
        [this, rawImuTiming, rawTimingFields](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          const auto &q = msg->orientation;
          const double sinr = 2 * (q.w * q.x + q.y * q.z);
          const double cosr = 1 - 2 * (q.x * q.x + q.y * q.y);
          const double roll = std::atan2(sinr, cosr);
          const double sinp = 2 * (q.w * q.y - q.z * q.x);
          const double pitch = std::abs(sinp) >= 1 ? std::copysign(kPi / 2, sinp) : std::asin(sinp);
          QJsonObject raw{{"roll_rad", roll}, {"pitch_rad", pitch},
                          {"yaw_rad", yawFromQuat(q.x, q.y, q.z, q.w)},
                          {"qx", q.x}, {"qy", q.y}, {"qz", q.z}, {"qw", q.w},
                          {"gx", msg->angular_velocity.x}, {"gy", msg->angular_velocity.y}, {"gz", msg->angular_velocity.z},
                          {"ax", msg->linear_acceleration.x}, {"ay", msg->linear_acceleration.y}, {"az", msg->linear_acceleration.z},
                          {"frame_id", QString::fromStdString(msg->header.frame_id)},
                          {"publisher_count", static_cast<double>(node_->count_publishers("/imu/raw/data"))},
                          {"source_topic", "/imu/raw/data"}, {"raw_ros_level", true}};
          const QJsonObject timing = rawTimingFields(rawImuTiming, msg->header.stamp);
          for (auto it = timing.constBegin(); it != timing.constEnd(); ++it) raw[it.key()] = it.value();
          update("imu_raw", raw);
        });

    const auto rawImuEulerTiming = std::make_shared<RawTimingState>();
    subscribe<geometry_msgs::msg::Vector3Stamped>("/imu/raw/euler", sensorQos,
        [this, rawImuEulerTiming, rawTimingFields](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
          QJsonObject raw{{"roll_rad", msg->vector.x}, {"pitch_rad", msg->vector.y}, {"yaw_rad", msg->vector.z},
                          {"frame_id", QString::fromStdString(msg->header.frame_id)},
                          {"publisher_count", static_cast<double>(node_->count_publishers("/imu/raw/euler"))},
                          {"source_topic", "/imu/raw/euler"}, {"raw_ros_level", true}};
          const QJsonObject timing = rawTimingFields(rawImuEulerTiming, msg->header.stamp);
          for (auto it = timing.constBegin(); it != timing.constEnd(); ++it) raw[it.key()] = it.value();
          update("imu_raw_euler", raw);
        });

    const auto rawImuMagTiming = std::make_shared<RawTimingState>();
    subscribe<geometry_msgs::msg::Vector3Stamped>("/imu/raw/mag", sensorQos,
        [this, rawImuMagTiming, rawTimingFields](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
          QJsonObject raw{{"mx", msg->vector.x}, {"my", msg->vector.y}, {"mz", msg->vector.z},
                          {"frame_id", QString::fromStdString(msg->header.frame_id)},
                          {"publisher_count", static_cast<double>(node_->count_publishers("/imu/raw/mag"))},
                          {"source_topic", "/imu/raw/mag"}, {"raw_ros_level", true}};
          const QJsonObject timing = rawTimingFields(rawImuMagTiming, msg->header.stamp);
          for (auto it = timing.constBegin(); it != timing.constEnd(); ++it) raw[it.key()] = it.value();
          update("imu_raw_mag", raw);
        });

    // /imu/mag is already calibrated by imu_node. Compute compass heading only once here.
    subscribe<geometry_msgs::msg::Vector3Stamped>(
        "/imu/mag", sensorQos, [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
          const double mx = msg->vector.x, my = msg->vector.y, mz = msg->vector.z;
          const double field = std::sqrt(mx * mx + my * my + mz * mz);
          double heading = magHeadingSign_ * std::atan2(my, mx) + magHeadingOffsetRad_;
          heading = std::fmod(heading, 2.0 * kPi);
          if (heading < 0.0) heading += 2.0 * kPi;
          const double signedHeading = heading > kPi ? heading - 2.0 * kPi : heading;
          update("imu_mag", QJsonObject{{"mx_t", mx}, {"my_t", my}, {"mz_t", mz}, {"field_t", field},
              {"heading_compass_rad", heading}, {"heading_signed_rad", signedHeading},
              {"heading_compass_deg", heading * 180.0 / kPi}, {"heading_offset_rad", magHeadingOffsetRad_},
              {"heading_offset_deg", magHeadingOffsetRad_ * 180.0 / kPi}, {"heading_sign", magHeadingSign_},
              {"validation_rmse_deg", magHeadingRmseDeg_}, {"calibrated", true},
              {"source", "/imu/mag (hard/soft-iron corrected)"},
              {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
        });

    subscribe<sensor_msgs::msg::LaserScan>("/scan_nav", sensorQos,
        [this, obstacleHalfLengthM, obstacleHalfWidthM, obstacleCorridorHalfWidthM,
         obstacleFrontBoundaryM, obstacleGeometrySource, corridorBehaviorState,
         lastDynamicPersonOnPathMs](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          double hz = 0.0;
          {
            // The driver stamps scan_time from consecutive physical revolutions.
            // Use that acquisition period instead of DDS callback arrival time:
            // a busy web executor may drop one best-effort message, but that must
            // not make the displayed LiDAR motor frequency appear to slow down.
            const double period = static_cast<double>(msg->scan_time);
            std::lock_guard<std::mutex> rateLock(lidarRateMutex_);
            if (std::isfinite(period) && period > 0.02 && period < 1.0) {
              lidarScanPeriods_.push_back(period);
              while (lidarScanPeriods_.size() > 30U) lidarScanPeriods_.pop_front();
            }
            if (!lidarScanPeriods_.empty()) {
              const double totalPeriod = std::accumulate(
                lidarScanPeriods_.begin(), lidarScanPeriods_.end(), 0.0);
              if (totalPeriod > 1e-6) {
                hz = static_cast<double>(lidarScanPeriods_.size()) / totalPeriod;
              }
            }
          }
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

          // Lightweight LaserScan geometry for the Navigation Workspace only.
          // Keep at most ~90 points so the live web stream stays cheap on the
          // Jetson.  Coordinates remain in lidar_link; the frontend applies the
          // verified static base_footprint->lidar_link transform and AMCL pose.
          // Static obstacle-in-path test in base_footprint coordinates.
          // base_footprint +X is forward and +Y is left. Ignore returns inside
          // the robot envelope, then test the URDF/Nav2-width corridor ahead.
          constexpr double baseToLidarX = 0.350;
          constexpr double baseToLidarY = 0.0;
          constexpr double baseToLidarYaw = 0.0;
          std::size_t corridorHitCount = 0;
          int currentClusterHits = 0;
          int maxClusterHits = 0;
          double nearestClearanceM = std::numeric_limits<double>::infinity();
          double nearestRangeM = std::numeric_limits<double>::infinity();
          double nearestBaseX = std::numeric_limits<double>::quiet_NaN();
          double nearestBaseY = std::numeric_limits<double>::quiet_NaN();
          for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
            const double r = msg->ranges[i];
            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
              currentClusterHits = 0;
              continue;
            }
            const double angle = static_cast<double>(msg->angle_min) +
                                 static_cast<double>(i) * static_cast<double>(msg->angle_increment);
            const double lx = r * std::cos(angle);
            const double ly = r * std::sin(angle);
            const double bx = baseToLidarX + lx;
            const double by = baseToLidarY + ly;
            const bool inCorridor = bx >= obstacleFrontBoundaryM &&
                                    bx <= obstacleFrontBoundaryM + obstacleTestHorizonM &&
                                    std::abs(by) <= obstacleCorridorHalfWidthM;
            if (!inCorridor) {
              currentClusterHits = 0;
              continue;
            }
            ++corridorHitCount;
            ++currentClusterHits;
            maxClusterHits = std::max(maxClusterHits, currentClusterHits);
            const double clearance = bx - obstacleHalfLengthM;
            if (clearance < nearestClearanceM) {
              nearestClearanceM = clearance;
              nearestRangeM = r;
              nearestBaseX = bx;
              nearestBaseY = by;
            }
          }
          const bool corridorCandidate = corridorHitCount > 0U;
          const bool obstacleInPath = maxClusterHits >= obstacleMinClusterHits;
          const QString corridorStatus = obstacleInPath ? QStringLiteral("OBSTACLE IN PATH")
              : (corridorCandidate ? QStringLiteral("CANDIDATE") : QStringLiteral("CLEAR"));

          const double behaviorNowMs = nowMs();
          const double personAgeMs = behaviorNowMs - lastDynamicPersonOnPathMs->load(std::memory_order_relaxed);
          const bool dynamicPersonOnPath = obstacleInPath && personAgeMs >= 0.0 &&
              personAgeMs <= dynamicPersonFreshSec * 1000.0;
          bool staticPersistent = false;
          double staticDurationSec = 0.0;
          int staticStableScans = 0;
          {
            std::lock_guard<std::mutex> behaviorLock(corridorBehaviorState->mutex);
            if (obstacleInPath && std::isfinite(nearestBaseX) && std::isfinite(nearestBaseY)) {
              const double shift = corridorBehaviorState->anchorValid
                  ? std::hypot(nearestBaseX - corridorBehaviorState->anchorX,
                               nearestBaseY - corridorBehaviorState->anchorY)
                  : std::numeric_limits<double>::infinity();
              if (!corridorBehaviorState->anchorValid || shift > staticPositionToleranceM) {
                corridorBehaviorState->anchorValid = true;
                corridorBehaviorState->anchorX = nearestBaseX;
                corridorBehaviorState->anchorY = nearestBaseY;
                corridorBehaviorState->stableSinceMs = behaviorNowMs;
                corridorBehaviorState->stableScans = 1;
              } else {
                // Small EMA follows LiDAR noise without allowing a moving obstacle
                // to accumulate a false static persistence timer.
                corridorBehaviorState->anchorX = 0.85 * corridorBehaviorState->anchorX + 0.15 * nearestBaseX;
                corridorBehaviorState->anchorY = 0.85 * corridorBehaviorState->anchorY + 0.15 * nearestBaseY;
                ++corridorBehaviorState->stableScans;
              }
              staticDurationSec = std::max(0.0, (behaviorNowMs - corridorBehaviorState->stableSinceMs) / 1000.0);
              staticStableScans = corridorBehaviorState->stableScans;
              staticPersistent = !dynamicPersonOnPath &&
                  staticDurationSec >= staticPersistenceSec &&
                  staticStableScans >= staticMinStableScans;
            } else {
              corridorBehaviorState->anchorValid = false;
              corridorBehaviorState->stableSinceMs = 0.0;
              corridorBehaviorState->stableScans = 0;
            }
          }

          QString behaviorClass = QStringLiteral("CLEAR");
          QString operatorIndicator = QStringLiteral("LINTASAN BEBAS");
          bool removeRequired = false;
          bool waitUntilClear = false;
          if (obstacleInPath && dynamicPersonOnPath) {
            behaviorClass = QStringLiteral("DYNAMIC_PERSON");
            operatorIndicator = QStringLiteral("TUNGGU - OBSTACLE DINAMIS");
            waitUntilClear = true;
          } else if (obstacleInPath && staticPersistent) {
            behaviorClass = QStringLiteral("STATIC_BOX_OR_OBJECT");
            operatorIndicator = QStringLiteral("SEGERA SINGKIRKAN");
            removeRequired = true;
          } else if (obstacleInPath) {
            behaviorClass = QStringLiteral("VERIFYING");
            operatorIndicator = QStringLiteral("VALIDASI STATIC / DYNAMIC");
          } else if (corridorCandidate) {
            behaviorClass = QStringLiteral("CANDIDATE");
            operatorIndicator = QStringLiteral("PANTAU");
          }

          update("obstacle_corridor", QJsonObject{
              {"available", true}, {"status", corridorStatus},
              {"obstacle_in_path", obstacleInPath}, {"candidate", corridorCandidate},
              {"hit_count", static_cast<double>(corridorHitCount)},
              {"max_cluster_hits", maxClusterHits}, {"min_cluster_hits", obstacleMinClusterHits},
              {"nearest_clearance_m", std::isfinite(nearestClearanceM) ? QJsonValue(nearestClearanceM) : QJsonValue()},
              {"nearest_lidar_range_m", std::isfinite(nearestRangeM) ? QJsonValue(nearestRangeM) : QJsonValue()},
              {"nearest_base_x_m", std::isfinite(nearestBaseX) ? QJsonValue(nearestBaseX) : QJsonValue()},
              {"nearest_base_y_m", std::isfinite(nearestBaseY) ? QJsonValue(nearestBaseY) : QJsonValue()},
              {"footprint_length_m", 2.0 * obstacleHalfLengthM},
              {"footprint_width_m", 2.0 * obstacleHalfWidthM},
              {"front_extent_m", obstacleHalfLengthM},
              {"corridor_half_width_m", obstacleCorridorHalfWidthM},
              {"corridor_width_m", 2.0 * obstacleCorridorHalfWidthM},
              {"lateral_margin_m", obstacleLateralMarginM},
              {"front_margin_m", obstacleFrontMarginM},
              {"test_horizon_m", obstacleTestHorizonM},
              {"dynamic_stopping_enabled", false},
              {"behavior_class", behaviorClass}, {"operator_indicator", operatorIndicator},
              {"dynamic_person_on_path", dynamicPersonOnPath},
              {"static_persistent", staticPersistent}, {"static_duration_s", staticDurationSec},
              {"static_stable_scans", staticStableScans},
              {"static_position_tolerance_m", staticPositionToleranceM},
              {"static_persistence_threshold_s", staticPersistenceSec},
              {"remove_required", removeRequired}, {"wait_until_clear", waitUntilClear},
              {"box_policy", QStringLiteral("persistent LiDAR obstacle -> static box/object")},
              {"geometry_source", obstacleGeometrySource},
              {"source_topic", QStringLiteral("/scan_nav")},
              {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});

          QJsonArray scanPoints;
          constexpr std::size_t kMaxWebScanPoints = 90;
          const std::size_t scanStride = std::max<std::size_t>(
              1, msg->ranges.size() / kMaxWebScanPoints + 1);
          for (std::size_t i = 0; i < msg->ranges.size(); i += scanStride) {
            const double r = msg->ranges[i];
            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
            const double angle = static_cast<double>(msg->angle_min) +
                                 static_cast<double>(i) * static_cast<double>(msg->angle_increment);
            scanPoints.append(QJsonArray{r * std::cos(angle), r * std::sin(angle)});
          }

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
              {"scan_points_lidar", scanPoints},
              // Verified live TF: base_footprint -> lidar_link = [0.350, 0.000, yaw 0].
              {"base_to_lidar_x_m", baseToLidarX}, {"base_to_lidar_y_m", baseToLidarY},
              {"base_to_lidar_yaw_rad", baseToLidarYaw},
              {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
          update("connected.lidar", true);
        });

    // BAB 4.1 raw LiDAR: direct driver LaserScan before navigation/self filtering.
    // Statistics below are descriptive only; no smoothing/median/outlier filtering
    // is applied. BAB 4.1.2 uses exactly one raw beam whose reported LaserScan
    // angle is closest to 0 rad. If that beam is invalid, keep it INVALID; never
    // substitute a neighboring beam. This affects reporting only, not SLAM/Nav2.
    const auto rawLidarTiming = std::make_shared<RawTimingState>();
    subscribe<sensor_msgs::msg::LaserScan>("/scan_safety_raw", sensorQos,
        [this, rawLidarTiming, rawTimingFields](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          std::size_t valid = 0;
          double sum = 0.0;
          double minRange = std::numeric_limits<double>::infinity();
          double maxRange = 0.0;
          for (double r : msg->ranges) {
            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
            ++valid; sum += r; minRange = std::min(minRange, r); maxRange = std::max(maxRange, r);
          }
          const double total = static_cast<double>(msg->ranges.size());
          const double validPct = total > 0.0 ? 100.0 * static_cast<double>(valid) / total : 0.0;
          int sampleIndex = -1;
          double sampleAngleDeg = std::numeric_limits<double>::quiet_NaN();
          double sampleRange = std::numeric_limits<double>::quiet_NaN();
          double sampleIntensity = std::numeric_limits<double>::quiet_NaN();
          bool sampleValid = false;
          if (!msg->ranges.empty() && std::abs(msg->angle_increment) > 1e-12) {
            auto isValidRawBeam = [&](int idx) {
              if (idx < 0 || idx >= static_cast<int>(msg->ranges.size())) return false;
              const double r = msg->ranges[static_cast<std::size_t>(idx)];
              return std::isfinite(r) && r >= msg->range_min && r <= msg->range_max;
            };
            const double indexAtZero = (0.0 - static_cast<double>(msg->angle_min)) /
                                       static_cast<double>(msg->angle_increment);
            sampleIndex = std::clamp(static_cast<int>(std::lround(indexAtZero)),
                                     0, static_cast<int>(msg->ranges.size()) - 1);
            sampleAngleDeg = (msg->angle_min + static_cast<double>(sampleIndex) * msg->angle_increment) * 180.0 / kPi;
            sampleRange = msg->ranges[static_cast<std::size_t>(sampleIndex)];
            sampleValid = isValidRawBeam(sampleIndex);
            if (static_cast<std::size_t>(sampleIndex) < msg->intensities.size())
              sampleIntensity = msg->intensities[static_cast<std::size_t>(sampleIndex)];
          }
          QJsonObject raw{{"frame_id", QString::fromStdString(msg->header.frame_id)},
                          {"samples", total}, {"beam_count", total},
                          {"valid_samples", static_cast<double>(valid)}, {"valid_beam_count", static_cast<double>(valid)},
                          {"valid_ratio_pct", validPct}, {"dropout_pct", 100.0 - validPct},
                          {"range_center_m", sampleValid ? QJsonValue(sampleRange) : QJsonValue()},
                          {"sample_angle_deg", std::isfinite(sampleAngleDeg) ? QJsonValue(sampleAngleDeg) : QJsonValue()},
                          {"sample_range_m", std::isfinite(sampleRange) ? QJsonValue(sampleRange) : QJsonValue()},
                          {"sample_intensity", std::isfinite(sampleIntensity) ? QJsonValue(sampleIntensity) : QJsonValue()},
                          {"sample_valid", sampleValid},
                          {"mean_range_m", valid ? QJsonValue(sum / static_cast<double>(valid)) : QJsonValue()},
                          {"range_min_observed_m", std::isfinite(minRange) ? QJsonValue(minRange) : QJsonValue()},
                          {"range_max_observed_m", valid ? QJsonValue(maxRange) : QJsonValue()},
                          {"angle_min_rad", msg->angle_min}, {"angle_max_rad", msg->angle_max},
                          {"angle_increment_rad", msg->angle_increment}, {"range_min_m", msg->range_min},
                          {"range_max_m", msg->range_max}, {"scan_time_s", msg->scan_time},
                          {"time_increment_s", msg->time_increment},
                          {"publisher_count", static_cast<double>(node_->count_publishers("/scan_safety_raw"))},
                          {"source_topic", "/scan_safety_raw"}, {"raw_ros_level", true}};
          const QJsonObject timing = rawTimingFields(rawLidarTiming, msg->header.stamp);
          for (auto it = timing.constBegin(); it != timing.constEnd(); ++it) raw[it.key()] = it.value();
          raw["scan_rate_hz"] = raw.value("rate_hz");
          update("lidar_raw", raw);
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

    const auto pathSubscribe = [this](const char *topic, const char *channel, const rclcpp::QoS &qos) {
      const QString ch = QString::fromLatin1(channel);
      subscribe<nav_msgs::msg::Path>(topic, qos, [this, ch](nav_msgs::msg::Path::ConstSharedPtr msg) {
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
    // Global paths are one-shot control products, not high-rate sensor data.
    // Keep the canonical /smac_plan durable so the web GUI recovers the last
    // valid path after a web-only restart; /plan and controller paths remain
    // reliable volatile to match their Nav2 publishers.
    const auto pathLatchedQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto pathReliableQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    pathSubscribe("/smac_plan", "nav_path", pathLatchedQos);
    pathSubscribe("/plan", "nav_path_legacy", pathReliableQos);
    pathSubscribe("/transformed_global_plan", "local_path", pathReliableQos);
    pathSubscribe("/controller_server/transformed_global_plan", "local_path_legacy", pathReliableQos);
    pathSubscribe("/local_plan", "local_path_legacy", pathReliableQos);

    // RViz-compatible MPPI MarkerArray bridge. Telemetry-only: preserves
    // marker semantics without changing MPPI/controller/lifecycle/cmd_vel.
    const auto mppiMarkerQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    subscribe<visualization_msgs::msg::MarkerArray>("/trajectories", mppiMarkerQos,
        [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
          QJsonArray markers;
          std::size_t pointCount = 0;
          std::size_t candidateCount = 0;
          std::size_t optimalCount = 0;
          constexpr std::size_t kMaxMarkers = 1200;
          constexpr std::size_t kMaxPointsTotal = 12000;
          constexpr std::size_t kMaxPointsPerMarker = 256;

          for (std::size_t mi = 0; mi < msg->markers.size() && mi < kMaxMarkers; ++mi) {
            const auto &m = msg->markers[mi];
            const QString ns = QString::fromStdString(m.ns);
            const QString nsLower = ns.toLower();
            if (nsLower.contains("candidate")) ++candidateCount;
            if (nsLower.contains("optimal")) ++optimalCount;

            QJsonObject out{{"ns", ns}, {"id", m.id}, {"type", m.type}, {"action", m.action},
                            {"frame_id", QString::fromStdString(m.header.frame_id)},
                            {"frame_locked", m.frame_locked},
                            {"scale_x", m.scale.x}, {"scale_y", m.scale.y}, {"scale_z", m.scale.z},
                            {"r", m.color.r}, {"g", m.color.g}, {"b", m.color.b}, {"a", m.color.a},
                            {"lifetime_ms", static_cast<double>(m.lifetime.sec) * 1000.0 +
                                                static_cast<double>(m.lifetime.nanosec) * 1e-6}};

            // DELETE / DELETEALL intentionally carry no geometry: frontend marker
            // cache applies action by namespace+id exactly like RViz.
            if (m.action == visualization_msgs::msg::Marker::ADD && pointCount < kMaxPointsTotal) {
              const auto &pq = m.pose.orientation;
              const double pyaw = yawFromQuat(pq.x, pq.y, pq.z, pq.w);
              const double c = std::cos(pyaw), si = std::sin(pyaw);
              const double tx = m.pose.position.x, ty = m.pose.position.y;
              QJsonArray pts;
              QJsonArray colors;
              auto appendPoint = [&](double x, double y, const std_msgs::msg::ColorRGBA *col) {
                if (pointCount >= kMaxPointsTotal) return;
                pts.append(QJsonArray{x, y});
                if (col) colors.append(QJsonArray{col->r, col->g, col->b, col->a});
                ++pointCount;
              };
              if (m.points.empty()) {
                appendPoint(tx, ty, nullptr);
              } else {
                const std::size_t stride = std::max<std::size_t>(1, m.points.size() / kMaxPointsPerMarker + 1);
                for (std::size_t pi = 0; pi < m.points.size() && pointCount < kMaxPointsTotal; pi += stride) {
                  const auto &pt = m.points[pi];
                  const std_msgs::msg::ColorRGBA *col = m.colors.size() == m.points.size() ? &m.colors[pi] : nullptr;
                  appendPoint(tx + c * pt.x - si * pt.y, ty + si * pt.x + c * pt.y, col);
                }
              }
              out["points"] = pts;
              if (!colors.isEmpty()) out["colors"] = colors;
            }
            markers.append(out);
          }

          update("mppi_trajectories", QJsonObject{
              {"markers", markers}, {"source_marker_count", static_cast<double>(msg->markers.size())},
              {"serialized_marker_count", markers.size()}, {"point_count", static_cast<double>(pointCount)},
              {"candidate_markers", static_cast<double>(candidateCount)},
              {"optimal_markers", static_cast<double>(optimalCount)}, {"at_ms", nowMs()}});
        });

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
        {"/navigation/mppi_closed_loop/yaw_rate_error_rps", "mppi_yaw_error"},
        {"/winch/pwm_pct", "winch.pwm_pct"}, {"/winch/servo_deg", "winch.servo_deg"}};
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
      const bool annotatedSource = source.startsWith(QStringLiteral("annotated"));
      if (annotatedSource) {
        lastAnnotatedCameraFrame_ = now;
      } else if (lastAnnotatedCameraFrame_.time_since_epoch().count() != 0 &&
                 std::chrono::duration<double>(now - lastAnnotatedCameraFrame_).count() < 3.0) {
        // Keep YOLO boxes stable on screen. Raw camera is only a fallback when
        // the annotated detector stream has actually gone stale.
        return;
      }
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
    // Localhost preview intentionally encodes RAW only. YOLO annotated ROS topics stay
    // untouched for RViz/debug, while browser overlays are drawn from ObstacleArray JSON.
    subscribe<sensor_msgs::msg::Image>("/camera/color/image_raw", sensorQos,
                                      [cacheCamera](sensor_msgs::msg::Image::ConstSharedPtr msg) { cacheCamera(msg, "raw"); });
    subscribe<std_msgs::msg::String>("/obstacle_detection/performance", sensorQos,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          update("perception_performance", parseJsonOrKv(QString::fromStdString(msg->data)));
        });
    subscribe<std_msgs::msg::String>("/warehouse_obstacle/performance", sensorQos,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          update("warehouse_person_performance", parseJsonOrKv(QString::fromStdString(msg->data)));
        });
    subscribe<yolo_obstacle_detection_ros2::msg::ObstacleArray>("/obstacle_detection/obstacles", sensorQos,
        [this](yolo_obstacle_detection_ros2::msg::ObstacleArray::ConstSharedPtr msg) {
          QJsonArray items;
          double confidenceSum = 0.0;
          double maxConfidence = -1.0;
          int dangerCount = 0, pathCount = 0;
          QJsonObject bestDetection, bestPalletDetection;
          double bestConfidence = -1.0, bestPalletConfidence = -1.0;
          for (const auto &o : msg->obstacles) {
            const QString className = QString::fromStdString(o.class_name);
            const QString category = QString::fromStdString(o.category);
            QJsonObject item{{"class_id", o.class_id}, {"class_name", className},
                             {"category", category}, {"confidence", o.confidence},
                             {"x1", o.x1}, {"y1", o.y1}, {"x2", o.x2}, {"y2", o.y2},
                             {"center_x", o.center_x}, {"center_y", o.center_y},
                             {"on_path", o.on_path}, {"in_danger_zone", o.in_danger_zone}};
            items.append(item);
            confidenceSum += o.confidence;
            maxConfidence = std::max(maxConfidence, static_cast<double>(o.confidence));
            if (o.confidence > bestConfidence) { bestConfidence = o.confidence; bestDetection = item; }
            const QString cls = className.trimmed().toLower();
            const QString cat = category.trimmed().toLower();
            if ((cls == QStringLiteral("pallet") || cat == QStringLiteral("pallet")) && o.confidence > bestPalletConfidence) {
              bestPalletConfidence = o.confidence; bestPalletDetection = item;
            }
            if (o.in_danger_zone) ++dangerCount;
            if (o.on_path) ++pathCount;
          }
          const int count = static_cast<int>(msg->obstacles.size());
          QJsonObject raw{{"count", count},
                          {"dynamic_count", msg->dynamic_count}, {"static_count", msg->static_count},
                          {"floor_count", msg->floor_count}, {"pallet_count", msg->pallet_count},
                          {"other_count", msg->other_count}, {"warning_active", msg->warning_active},
                          {"path_obstacle_count", static_cast<int>(msg->path_obstacles.size())},
                          {"mean_confidence", count ? confidenceSum / count : 0.0},
                          {"max_confidence", count ? maxConfidence : 0.0},
                          {"items", items}, {"at_ms", nowMs()}};
          if (!bestDetection.isEmpty()) {
            raw["best_class_name"] = bestDetection.value("class_name"); raw["best_category"] = bestDetection.value("category");
            raw["best_confidence"] = bestDetection.value("confidence"); raw["best_center_x_px"] = bestDetection.value("center_x"); raw["best_center_y_px"] = bestDetection.value("center_y");
          }
          if (!bestPalletDetection.isEmpty()) {
            raw["pallet_best_class_name"] = bestPalletDetection.value("class_name"); raw["pallet_best_category"] = bestPalletDetection.value("category");
            raw["pallet_best_confidence"] = bestPalletDetection.value("confidence"); raw["pallet_best_center_x_px"] = bestPalletDetection.value("center_x"); raw["pallet_best_center_y_px"] = bestPalletDetection.value("center_y");
          }
          update("raw_detections", raw);
          update("obstacle_metrics", QJsonObject{{"count", count}, {"dynamic", msg->dynamic_count},
                                                  {"static", msg->static_count}, {"floor", msg->floor_count},
                                                  {"pallet", msg->pallet_count}, {"other", msg->other_count},
                                                  {"danger", dangerCount}, {"on_path", pathCount},
                                                  {"warning_active", msg->warning_active}, {"at_ms", nowMs()}});
        });
    subscribe<yolo_obstacle_detection_ros2::msg::ObstacleArray>("/warehouse_obstacle/persons", sensorQos,
        [this, lastDynamicPersonOnPathMs](yolo_obstacle_detection_ros2::msg::ObstacleArray::ConstSharedPtr msg) {
          QJsonArray items;
          double confidenceSum = 0.0;
          bool anyPersonOnPath = false;
          for (const auto &o : msg->obstacles) {
            if (QString::fromStdString(o.class_name).trimmed().toLower() != QStringLiteral("person")) continue;
            items.append(QJsonObject{{"class_id", o.class_id}, {"class_name", "person"},
                                     {"category", "dynamic"}, {"confidence", o.confidence},
                                     {"x1", o.x1}, {"y1", o.y1}, {"x2", o.x2}, {"y2", o.y2},
                                     {"center_x", o.center_x}, {"center_y", o.center_y},
                                     {"on_path", o.on_path}, {"in_danger_zone", o.in_danger_zone}});
            confidenceSum += o.confidence;
            anyPersonOnPath = anyPersonOnPath || o.on_path;
          }
          if (anyPersonOnPath) lastDynamicPersonOnPathMs->store(nowMs(), std::memory_order_relaxed);
          const int count = items.size();
          update("warehouse_person_detections", QJsonObject{{"count", count}, {"dynamic_count", count},
              {"mean_confidence", count ? confidenceSum / count : 0.0}, {"items", items}, {"at_ms", nowMs()}});
        });
    subscribe<yolo_obstacle_detection_ros2::msg::AlignmentState>("/fork_alignment/state", rclcpp::QoS(1).reliable().transient_local(),
        [this](yolo_obstacle_detection_ros2::msg::AlignmentState::ConstSharedPtr msg) {
          update("alignment_state", QJsonObject{
            {"state", msg->state}, {"state_text", QString::fromStdString(msg->state_text)},
            {"pallet_detected", msg->pallet_detected}, {"detection_stable", msg->detection_stable},
            {"confidence", msg->confidence}, {"error_lateral_m", msg->error_lateral_m},
            {"error_yaw_deg", msg->error_yaw_deg}, {"pid_lateral_output", msg->pid_lateral_output},
            {"pid_yaw_output", msg->pid_yaw_output}, {"desired_yaw_rate", msg->desired_yaw_rate},
            {"estimated_steering_deg", msg->estimated_steering_deg},
            {"linear_velocity_cmd", msg->linear_velocity_cmd},
            {"angular_velocity_cmd", msg->angular_velocity_cmd},
            {"steering_limit_active", msg->steering_limit_active},
            {"safety_stop_active", msg->safety_stop_active}, {"data_valid", msg->data_valid},
            {"lateral_within_tolerance", msg->lateral_within_tolerance},
            {"yaw_within_tolerance", msg->yaw_within_tolerance},
            {"ready_for_insertion", msg->ready_for_insertion}, {"at_ms", nowMs()}});
        });

    subscribe<std_msgs::msg::String>("/mapping/web_status", rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        update("mapping_status", parseJsonOrKv(QString::fromStdString(msg->data)));
      });

    subscribe<std_msgs::msg::String>("/navigation/map_switch_status", rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        update("nav_map_switch_status", parseJsonOrKv(QString::fromStdString(msg->data)));
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
          int shade = 205;  // unknown, matching ROS map_server trinary view
          if (occ >= 65) shade = 0;
          else if (occ >= 0 && occ <= 25) shade = 254;
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

    const auto gridStatsSubscribe = [this, latchedQos](const char *topic, const char *channel,
                                                       const char *overlayKind = nullptr) {
      const QString ch = QString::fromLatin1(channel);
      const QString overlay = overlayKind ? QString::fromLatin1(overlayKind) : QString();
      subscribe<nav_msgs::msg::OccupancyGrid>(topic, latchedQos,
          [this, ch, overlay](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
            if (msg->info.width == 0 || msg->info.height == 0 || msg->data.empty()) return;
            const std::uint64_t cellCount = static_cast<std::uint64_t>(msg->info.width) * static_cast<std::uint64_t>(msg->info.height);
            if (cellCount > msg->data.size() || cellCount > 100000000ULL ||
                msg->info.width > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
                msg->info.height > static_cast<unsigned int>(std::numeric_limits<int>::max())) return;
            std::size_t unknown = 0, occupied = 0, inflated = 0;
            QImage image;
            if (!overlay.isEmpty()) {
              image = QImage(static_cast<int>(msg->info.width), static_cast<int>(msg->info.height), QImage::Format_ARGB32);
              if (image.isNull()) return;
              image.fill(Qt::transparent);
            }
            for (unsigned int y = 0; y < msg->info.height; ++y) {
              QRgb *row = overlay.isEmpty() ? nullptr : reinterpret_cast<QRgb *>(image.scanLine(static_cast<int>(msg->info.height - 1 - y)));
              for (unsigned int x = 0; x < msg->info.width; ++x) {
                const int8_t cell = msg->data[static_cast<size_t>(y) * msg->info.width + x];
                if (cell < 0) { ++unknown; continue; }
                if (cell >= 50) ++occupied;
                if (cell > 0 && cell < 99) ++inflated;
                if (!row || cell <= 0) continue;
                if (cell >= 99) {
                  row[x] = qRgba(255, 72, 84, 185);
                } else {
                  const int alpha = std::clamp(20 + static_cast<int>(cell) * 13 / 10, 22, 150);
                  if (overlay == "local") row[x] = qRgba(85, 214, 232, alpha);
                  else row[x] = qRgba(255, 189, 89, alpha);
                }
              }
            }
            const double n = static_cast<double>(cellCount);
            const auto &origin = msg->info.origin.position;
            const auto &oq = msg->info.origin.orientation;
            QJsonObject meta{
                {"width", static_cast<int>(msg->info.width)}, {"height", static_cast<int>(msg->info.height)},
                {"resolution", msg->info.resolution}, {"origin_x", origin.x}, {"origin_y", origin.y},
                {"origin_yaw", yawFromQuat(oq.x, oq.y, oq.z, oq.w)},
                {"unknown_pct", n > 0 ? 100.0 * unknown / n : 0.0},
                {"occupied_pct", n > 0 ? 100.0 * occupied / n : 0.0},
                {"inflated_pct", n > 0 ? 100.0 * inflated / n : 0.0},
                {"frame_id", QString::fromStdString(msg->header.frame_id)}, {"at_ms", nowMs()}};
            if (!overlay.isEmpty()) {
              QByteArray encoded;
              QBuffer buffer(&encoded);
              if (buffer.open(QIODevice::WriteOnly) && image.save(&buffer, "PNG") && !encoded.isEmpty()) {
                std::lock_guard<std::mutex> lock(mediaMutex_);
                if (overlay == "local") localCostmapPng_ = encoded;
                else globalCostmapPng_ = encoded;
                meta["overlay_ready"] = true;
              }
            }
            update(ch, meta);
          });
    };
    gridStatsSubscribe("/nav_map", "nav_map_stats");
    gridStatsSubscribe("/global_costmap/costmap", "global_costmap", "global");
    gridStatsSubscribe("/local_costmap/costmap", "local_costmap", "local");

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

  // BAB 4.2.4 recorder follows Mapping 1/2/3 without changing the mapping engine.
  int loopRecordingSlot_{0};
  bool loopReferenceValid_{false};
  double loopReferenceX_{0.0};
  double loopReferenceY_{0.0};
  double loopReferenceYaw_{0.0};

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
    if (request.method == "GET" && request.path == "/api/global_costmap.png") {
      const QByteArray bytes = bridge_->globalCostmapPng();
      if (bytes.isEmpty()) return sendText(socket, 503, "text/plain; charset=utf-8", "Global costmap belum tersedia");
      return sendBytes(socket, 200, "image/png", bytes, {{"Cache-Control", "no-store, max-age=0"}});
    }
    if (request.method == "GET" && request.path == "/api/local_costmap.png") {
      const QByteArray bytes = bridge_->localCostmapPng();
      if (bytes.isEmpty()) return sendText(socket, 503, "text/plain; charset=utf-8", "Local costmap belum tersedia");
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
      const QString fileKey = json.value("file_key").toString();
      const QString yamlPath = json.value("path").toString();
      QJsonValue saved;
      QJsonObject runtimeApply;
      const QJsonValue requestedValue = json.value("value");
      if (fileKey == QStringLiteral("nav2") || fileKey == QStringLiteral("ekf") ||
          fileKey == QStringLiteral("navigation_core")) {
        const auto target = runtimeParameterTarget(fileKey, yamlPath);
        if (target && nav2PlannerOwnedTarget(*target)) {
          // Smac planner and global-costmap parameters are declared during the
          // planner lifecycle CONFIGURE transition. Persist first, respawn only
          // planner_server, configure temporarily, verify the real parameter,
          // then restore its safe lifecycle state. Roll back both disk and owner
          // runtime if any step fails.
          const QJsonValue previousYaml = currentYamlValue(fileKey, yamlPath);
          ok = setYamlValueAtomic(fileKey, yamlPath, requestedValue, &message, &saved);
          if (ok) {
            runtimeApply = applyPlannerReloadVerified(*target, requestedValue);
            if (!runtimeApply.value("verified").toBool(false)) {
              const QString failedState = runtimeApply.value("state").toString();
              QString rollbackMessage;
              QJsonValue rollbackSaved;
              QJsonObject rollbackApply;
              const bool canRollback = !previousYaml.isUndefined() && !previousYaml.isNull();
              const bool yamlRolledBack = canRollback &&
                setYamlValueAtomic(fileKey, yamlPath, previousYaml, &rollbackMessage, &rollbackSaved);
              if (yamlRolledBack) {
                saved = rollbackSaved;
                if (failedState == QStringLiteral("BUSY_ACTIVE_NAV")) {
                  rollbackApply = QJsonObject{{"verified", true}, {"applied", false},
                    {"state", "YAML_ROLLED_BACK_RUNTIME_UNCHANGED"},
                    {"message", "Planner ACTIVE tidak direstart; YAML dikembalikan ke nilai sebelumnya"}};
                } else {
                  rollbackApply = applyPlannerReloadVerified(*target, previousYaml);
                }
              }
              const bool rollbackVerified = yamlRolledBack &&
                rollbackApply.value("verified").toBool(false);
              runtimeApply["rollback"] = rollbackApply;
              runtimeApply["applied"] = false;
              runtimeApply["verified"] = false;
              runtimeApply["state"] = rollbackVerified ? "FAILED_ROLLED_BACK" : "ROLLBACK_FAILED";
              message = QStringLiteral("Parameter planner/global-costmap belum diterapkan; ") +
                        runtimeApply.value("message").toString();
              if (yamlRolledBack) message += QStringLiteral(" • YAML dikembalikan: ") + rollbackMessage;
              ok = false;
            }
          } else {
            runtimeApply = QJsonObject{{"attempted", false}, {"applied", false}, {"verified", false},
              {"strategy", "planner_restart_configure_verify"}, {"state", "YAML_SAVE_FAILED"},
              {"node", target->node}, {"parameter", target->parameter},
              {"message", "YAML gagal disimpan; planner tidak direstart"}};
          }
        } else {
          const bool navigationCoreNextStartOnly =
            fileKey == QStringLiteral("navigation_core") && yamlPath.endsWith(QStringLiteral(".publish_rate_hz"));
          if (navigationCoreNextStartOnly) {
            ok = setYamlValueAtomic(fileKey, yamlPath, requestedValue, &message, &saved);
            runtimeApply = QJsonObject{{"attempted", false}, {"applied", false}, {"verified", false},
              {"strategy", "next_start"}, {"state", "NEXT_START"},
              {"message", "YAML tersimpan; publish-rate health manager diterapkan pada start manager berikutnya"}};
          } else {
            // AMCL, planner/controller/costmap, EKF and live health parameters:
            // apply to runtime first and require read-back before persisting.
            runtimeApply = applyRuntimeParameter(fileKey, yamlPath, requestedValue, bridge_->parameterNode());
            const bool runtimeVerified = runtimeApply.value("verified").toBool(false);
            if (runtimeVerified) {
              ok = setYamlValueAtomic(fileKey, yamlPath, requestedValue, &message, &saved);
              if (!ok) {
                const QJsonValue previous = runtimeApply.value("previous_value");
                QJsonObject rollback;
                if (!previous.isUndefined() && !previous.isNull())
                  rollback = applyRuntimeParameter(fileKey, yamlPath, previous, bridge_->parameterNode());
                runtimeApply["rollback"] = rollback;
                runtimeApply["applied"] = false;
                runtimeApply["verified"] = false;
                runtimeApply["state"] = rollback.value("verified").toBool(false) ?
                  "YAML_SAVE_FAILED_ROLLED_BACK" : "ROLLBACK_FAILED";
              }
            } else if (fileKey == QStringLiteral("navigation_core")) {
              // Health-manager parameter service can be busy polling lifecycle
              // nodes under high CPU load. Persist safely without killing it.
              ok = setYamlValueAtomic(fileKey, yamlPath, requestedValue, &message, &saved);
              if (ok) {
                runtimeApply["state"] = "NEXT_START";
                runtimeApply["applied"] = false;
                runtimeApply["verified"] = false;
                runtimeApply["message"] = QStringLiteral("Runtime health manager sedang sibuk; YAML tersimpan untuk start berikutnya");
              }
            } else {
              ok = false;
              message = QStringLiteral("Runtime tidak berubah; YAML dipertahankan. ") +
                        runtimeApply.value("message").toString();
            }
          }
        }
      } else if (fileKey == QStringLiteral("lidar")) {
        // LiDAR parameters are consumed by the custom driver at startup. Apply
        // transactionally: write YAML, respawn only lidar_node, require native
        // parameter read-back, and roll back disk + driver if verification fails.
        const QJsonValue previousYaml = currentYamlValue(fileKey, yamlPath);
        ok = setYamlValueAtomic(fileKey, yamlPath, requestedValue, &message, &saved);
        if (ok) {
          runtimeApply = applyRuntimeParameter(fileKey, yamlPath, saved, bridge_->parameterNode());
          if (!runtimeApply.value("verified").toBool(false)) {
            QString rollbackMessage;
            QJsonValue rollbackSaved;
            QJsonObject rollbackApply;
            const bool canRollback = !previousYaml.isUndefined() && !previousYaml.isNull();
            const bool yamlRolledBack = canRollback &&
              setYamlValueAtomic(fileKey, yamlPath, previousYaml, &rollbackMessage, &rollbackSaved);
            if (yamlRolledBack) {
              saved = rollbackSaved;
              rollbackApply = applyRuntimeParameter(fileKey, yamlPath, previousYaml, bridge_->parameterNode());
            }
            const bool rollbackVerified = yamlRolledBack && rollbackApply.value("verified").toBool(false);
            runtimeApply["rollback"] = rollbackApply;
            runtimeApply["applied"] = false;
            runtimeApply["verified"] = false;
            runtimeApply["state"] = rollbackVerified ? "FAILED_ROLLED_BACK" : "ROLLBACK_FAILED";
            message = QStringLiteral("Parameter LiDAR belum diterapkan; ") + runtimeApply.value("message").toString();
            if (yamlRolledBack) message += QStringLiteral(" • YAML dikembalikan: ") + rollbackMessage;
            ok = false;
          }
        }
      } else {
        // Preserve the existing behavior for perception, steering, sensor and SLAM.
        ok = setYamlValueAtomic(fileKey, yamlPath, requestedValue, &message, &saved);
        if (ok) runtimeApply = applyRuntimeParameter(fileKey, yamlPath, saved, bridge_->rosNode());
      }
      const QString runtimeMessage = runtimeApply.value("message").toString();
      if (ok && !runtimeMessage.isEmpty()) message += QStringLiteral(" • ") + runtimeMessage;
      return sendJson(socket, ok ? 200 : 409, QJsonObject{{"ok", ok}, {"message", message},
                       {"file_key", fileKey}, {"path", yamlPath}, {"saved_value", saved},
                       {"runtime_apply", runtimeApply}, {"config", loadConfigSnapshot()}, {"at_ms", nowMs()}});
    } else if (request.path == "/api/experiment/record/start") {
      ok = startRecording(json, &message);
      return sendJson(socket, ok ? 200 : 409, QJsonObject{{"ok", ok}, {"message", message},
                       {"recording", recordingStatus()}, {"at_ms", nowMs()}});
    } else if (request.path == "/api/experiment/record/stop") {
      QJsonObject result;
      ok = stopRecording(&message, &result);
      result["ok"] = ok; result["message"] = message; result["at_ms"] = nowMs();
      return sendJson(socket, ok ? 200 : 409, result);
    } else if (request.path == "/api/mapping/start") {
      const int slot = json.value("slot").toInt(0);
      if (slot < 1 || slot > 3) {
        return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "Slot mapping harus 1, 2, atau 3"}});
      }
      ok = bridge_->triggerService(QString("/mapping/start_%1").arg(slot), "mapping_start", &message);
      if (ok && !recording_) {
        QJsonObject autoRec{{"subsystem", "navigation"}, {"id", "4.2.4"},
                            {"variation", QString("Map %1").arg(slot)}, {"condition", "SYNC MAPPING"},
                            {"sample_rate_hz", 1.0}, {"mapping_slot", slot}};
        QString recMessage;
        if (startRecording(autoRec, &recMessage)) message += QStringLiteral(" • CSV 4.2.4 AUTO START");
        else message += QStringLiteral(" • WARNING CSV 4.2.4: ") + recMessage;
      } else if (ok && recording_) {
        message += QStringLiteral(" • recorder lain aktif; CSV 4.2.4 tidak diambil alih");
      }
    } else if (request.path == "/api/mapping/stop-save") {
      const bool autoLoopRecording = recording_ && recordingSubsystem_ == QStringLiteral("navigation") &&
                                     recordingId_ == QStringLiteral("4.2.4");
      if (autoLoopRecording) captureRecordingSample();
      ok = bridge_->triggerService("/mapping/stop_save", "mapping_stop_save", &message);
      if (ok && autoLoopRecording) {
        QJsonObject recResult; QString recMessage;
        if (stopRecording(&recMessage, &recResult))
          message += QStringLiteral(" • CSV 4.2.4 SAVED: ") + recResult.value("raw_csv").toString();
        else message += QStringLiteral(" • WARNING CSV 4.2.4: ") + recMessage;
      }
    } else if (request.path == "/api/mapping/stop-without-save") {
      ok = bridge_->triggerService("/mapping/stop_without_save", "mapping_stop_without_save", &message);
    } else if (request.path == "/api/mapping/delete") {
      const int slot = json.value("slot").toInt(0);
      if (slot < 1 || slot > 3) {
        return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "Slot map harus 1, 2, atau 3"}});
      }
      ok = bridge_->triggerServiceSync(QString("/mapping/delete_%1").arg(slot), "mapping_delete", &message);
    } else if (request.path == "/api/navigation/map-start") {
      const int slot = json.value("slot").toInt(0);
      if (slot < 1 || slot > 4) {
        return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "Slot Nav2 harus 1..4 (4 = Navigation Map)"}});
      }
      ok = bridge_->triggerService(QString("/navigation/map/start_%1").arg(slot), "nav_map_start", &message);
    } else if (request.path == "/api/navigation/map-stop") {
      ok = bridge_->triggerService("/navigation/map/stop", "nav_map_stop", &message);
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
    if (recordingSubsystem_ == QStringLiteral("navigation") && recordingId_ == QStringLiteral("4.2.4")) {
      loopRecordingSlot_ = json.value("mapping_slot").toInt(0);
      loopReferenceValid_ = false;
    }
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
    if (recordingSubsystem_ == QStringLiteral("navigation") && recordingId_ == QStringLiteral("4.2.4")) {
      const QJsonObject mapping = snap.value("mapping_status").toObject();
      const QJsonObject pose = mapping.value("mapping_odom").toObject();
      if (loopRecordingSlot_ <= 0) loopRecordingSlot_ = mapping.value("slot").toInt(0);
      row["mapping_slot"] = QString::number(loopRecordingSlot_);
      const bool poseValid = mapping.value("active").toBool(false) && mapping.value("map_fresh").toBool(false) &&
                             pose.value("ready").toBool(false) && pose.value("pose_source").toString() == QStringLiteral("slam_tf") &&
                             pose.contains("map_x") && pose.contains("map_y") && pose.contains("map_yaw") &&
                             std::isfinite(pose.value("map_x").toDouble()) && std::isfinite(pose.value("map_y").toDouble()) &&
                             std::isfinite(pose.value("map_yaw").toDouble());
      row["loop_pose_valid"] = poseValid ? QStringLiteral("1") : QStringLiteral("0");
      row["loop_state"] = poseValid ? QStringLiteral("CAPTURING") : QStringLiteral("WAITING_POSE");
      if (poseValid) {
        const double x = pose.value("map_x").toDouble(), y = pose.value("map_y").toDouble(), yaw = pose.value("map_yaw").toDouble();
        if (!loopReferenceValid_) {
          loopReferenceValid_ = true; loopReferenceX_ = x; loopReferenceY_ = y; loopReferenceYaw_ = yaw;
        }
        const double dx = x - loopReferenceX_, dy = y - loopReferenceY_;
        const double dyaw = std::atan2(std::sin(yaw - loopReferenceYaw_), std::cos(yaw - loopReferenceYaw_));
        row["loop_reference_x_m"] = QString::number(loopReferenceX_, 'f', 6);
        row["loop_reference_y_m"] = QString::number(loopReferenceY_, 'f', 6);
        row["loop_reference_yaw_rad"] = QString::number(loopReferenceYaw_, 'f', 8);
        row["loop_current_x_m"] = QString::number(x, 'f', 6);
        row["loop_current_y_m"] = QString::number(y, 'f', 6);
        row["loop_current_yaw_rad"] = QString::number(yaw, 'f', 8);
        row["loop_error_m"] = QString::number(std::hypot(dx, dy), 'f', 6);
        row["loop_yaw_error_deg"] = QString::number(std::abs(dyaw) * 180.0 / kPi, 'f', 6);
      }
    }
    const bool navBab41 = recordingSubsystem_ == QStringLiteral("navigation") &&
                          recordingId_.startsWith(QStringLiteral("4.1."));
    const auto allowRecordingChannel = [this, navBab41](const QString &key) {
      if (!navBab41) return true;
      if (recordingId_ == QStringLiteral("4.1.2")) {
        return key == QStringLiteral("lidar_raw") || key == QStringLiteral("connected.lidar");
      }
      if (recordingId_ == QStringLiteral("4.1.3")) {
        return key == QStringLiteral("imu_raw") || key == QStringLiteral("imu_raw_euler") ||
               key == QStringLiteral("imu_raw_mag") || key == QStringLiteral("connected.imu");
      }
      if (recordingId_ == QStringLiteral("4.1.1") || recordingId_ == QStringLiteral("4.1.5")) {
        return key == QStringLiteral("lidar_raw") || key == QStringLiteral("imu_raw") ||
               key == QStringLiteral("imu_raw_euler") || key == QStringLiteral("imu_raw_mag") ||
               key == QStringLiteral("odom") || key == QStringLiteral("connected.lidar") ||
               key == QStringLiteral("connected.imu");
      }
      if (recordingId_ == QStringLiteral("4.1.4")) {
        return key == QStringLiteral("odom");
      }
      return true;  // 4.1.6 is a TF validation section, not a raw sensor-value section.
    };
    for (auto it = snap.constBegin(); it != snap.constEnd(); ++it) {
      if (it.key().startsWith(QStringLiteral("__"))) continue;
      if (!allowRecordingChannel(it.key())) continue;
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
