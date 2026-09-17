#pragma once
#include <QMap>
#include <QString>
#include <QStringList>
#include <QVector>
#include <QVariant>
#include <initializer_list>
#include <utility>
// Each leaf = one report subsection (subjudul). A single leaf may carry multiple
// report tables (Tabel 1..N) and multiple graphs (Grafik 1..N) — the GUI exposes
// them through small selectors in the right panel. No artificial `B`/`C` suffixes
// are emitted: "4.1.1" is one leaf with two tables / two graphs.
//
// Graph titles render as "Format mengacu Gambar 4.xx — Data Aktual GUI" and never
// as estimated-data labels. The catalog only carries the live ROS topic bindings
// used to build the actual GUI panels; no estimated values are stored here.

// Parameter field metadata for dynamic ExperimentParameterPanel.
struct ExperimentParameterField {
  QString key;                // "sample_rate", "frequency", "gt_x", etc
  QString label;              // "Sample Rate (Hz)", "Ground Truth X", etc
  QString kind;               // "float", "int", "string", "yaml_readonly"
  QString yamlFileKey;        // "gnss", "ekf_local", "nav2" (maps to YamlStore)
  QString yamlPath;           // "data_cuav_node.ros__parameters.navigation_rate_hz"
  QString placeholder;        // hint text
  QString lockedValue;        // for locked fields
  bool isGroundTruth = false; // yellow styling
  bool locked = false;        // read-only + locked label
  // Compact constructor for initializer-list init (trailing defaults)
  ExperimentParameterField(QString k = {}, QString lbl = {}, QString knd = {},
    QString yf = {}, QString yp = {}, QString ph = {}, QString lv = {},
    bool gt = false, bool lck = false)
  : key(k), label(lbl), kind(knd), yamlFileKey(yf), yamlPath(yp),
    placeholder(ph), lockedValue(lv), isGroundTruth(gt), locked(lck) {}
};

// Optional per-graph rendering spec (for multi-series or scatter graphs).
struct ExperimentGraphSpec {
  QString type;               // "time_series" | "scatter"
  QStringList series;         // for time_series: keys into liveSeries
  QString xSeries;            // for scatter: telemetry key for X
  QString ySeries;            // for scatter: telemetry key for Y
  ExperimentGraphSpec(QString t = {}, QStringList s = {}, QString x = {}, QString y = {})
  : type(t), series(s), xSeries(x), ySeries(y) {}
};

struct ExperimentSpec {
  QString subsystem;
  // navigation | perception | steering
  QString groupId;
  // "4.1"
  QString groupTitle;
  // "4.1 Pengujian Sensor"
  QString id;
  // "4.1.1"  (no fake suffix)
  QString section;
  // "4.1.1 Pengujian GNSS"
  QStringList tableNames;
  // "Tabel 1", "Tabel 2", ...
  QVector<QStringList> tableColumns;
  // columns per table, report order preserved
  QStringList graphCaptions;
  // "Grafik N" captions (0..k)
  QMap<QString, QString> liveSeries;
  // Parameter fields for dynamic panel (empty = auto-generate from liveSeries)
  QVector<ExperimentParameterField> parameterFields;
  // Optional graph render specs (empty = default time-series of all liveSeries)
  QVector<ExperimentGraphSpec> graphs;
  // Optional source-defined rows for report-only sections (e.g. BAB IV 4.1 summary).
  QVector<QVariantMap> defaultRows;
  // False for report/overview leaves that are not acquisition runs.
  bool recordable = true;
};
inline QVector<ExperimentSpec> buildExperimentCatalog(const QString &subsystem) {
  QVector<ExperimentSpec> out;
  auto add = [&](const char *subsys, const QString &groupId, const QString &groupTitle,
  const char *id, const QString &section,
  std::initializer_list<const char *> graphs,
  std::initializer_list<std::initializer_list<const char *>> tblColumns,
  std::initializer_list<std::pair<const char *, const char *>> series,
  QVector<ExperimentParameterField> params = QVector<ExperimentParameterField>(),
  QVector<ExperimentGraphSpec> graphSpecs = QVector<ExperimentGraphSpec>()) {
    if (subsystem != QString::fromLatin1(subsys)) return;
    ExperimentSpec spec;
    spec.subsystem = subsystem;
    spec.groupId = groupId;
    spec.groupTitle = groupTitle;
    spec.id = QString::fromUtf8(id);
    spec.section = section;
    for (const char *g : graphs) spec.graphCaptions << QString::fromUtf8(g);
    for (const auto &cols : tblColumns) {
      spec.tableNames << QStringLiteral("Tabel %1").arg(spec.tableColumns.size() + 1);
      QStringList list;
      for (const char *c : cols) list << QString::fromUtf8(c);
      spec.tableColumns << list;
    }
    for (const auto &item : series)
    spec.liveSeries[QString::fromUtf8(item.first)] = QString::fromUtf8(item.second);
    for (const ExperimentParameterField &p : params) spec.parameterFields << p;
    for (const ExperimentGraphSpec &g : graphSpecs) spec.graphs << g;
    out << spec;
  };
  // clang-format off
  /* ------------------------- NAVIGASI ------------------------- */
  if (subsystem == QStringLiteral("navigation")) {
    #include "agv_navigation_bab4_catalog.inc"
    return out;
  }
  add("navigation", QStringLiteral("4.1"), QStringLiteral("4.1 Pengujian Sensor"), "4.1.1",
  QStringLiteral("4.1.1 Pengujian GNSS"),
  {
    "Sebaran posisi GNSS statis", "Kualitas satelit, pDOP, dan hAcc GNSS"
  },
  {
    {
      "Parameter", "Nilai YAML Aktual", "Fungsi"
    },
    {
      "Metrik", "Hasil"
    }
  },
  {
    {
      "Satelit", "gnss_quality.sat"
    }, {
      "DOP", "gnss_quality.dop"
    }, {
      "hAcc", "gnss_quality.hacc_m"
    }
  },
  {
    { "sample_rate", "Sample Rate (Hz)", "float", "", "", "10.0" },
    { "duration", "Durasi (s)", "float", "", "", "30.0" },
    { "variation", "Variasi / Run", "string", "", "", "variasi-1" },
    { "condition", "Kondisi", "string", "", "", "statis" },
    { "gt_x", "Ground Truth X", "float", "", "", "0.0", "", true },
    { "gt_y", "Ground Truth Y", "float", "", "", "0.0", "", true },
    { "min_satellites", "Min Satellites", "yaml_readonly", "gnss", "data_cuav_node.ros__parameters.min_satellites" },
    { "max_dop", "Max DOP", "yaml_readonly", "gnss", "data_cuav_node.ros__parameters.max_dop" },
    { "max_hacc_m", "Max hAcc (m)", "yaml_readonly", "gnss", "data_cuav_node.ros__parameters.max_hacc_m" },
    { "nav_rate", "navigation_rate_hz", "float", "gnss", "data_cuav_node.ros__parameters.navigation_rate_hz", "10.0" },
    { "nav_model", "Dynamic Model", "yaml_readonly", "gnss", "data_cuav_node.ros__parameters.dynamic_model" }
  },
  {
    { "scatter", {}, "gnss_fix.lat", "gnss_fix.lon" },
    { "time_series", { "Satelit", "DOP", "hAcc" } }
  });
  add("navigation", QStringLiteral("4.1"), QStringLiteral("4.1 Pengujian Sensor"), "4.1.2",
  QStringLiteral("4.1.2 Pengujian IMU"),
  {
    "Noise gyro-Z IMU ketika kendaraan diam", "Linearitas heading IMU"
  },
  {
    {
      "Sumbu", "Mean gyro saat diam (rad/s)", "Std (rad/s)"
    },
    {
      "Heading referensi", "Heading IMU", "Error"
    }
  },
  {
    {
      "Gyro Z", "imu.gz"
    }, {
      "Yaw", "imu.yaw_rad"
    }, {
      "Yaw residual", "imu_status.yaw_residual"
    }
  });
  add("navigation", QStringLiteral("4.1"), QStringLiteral("4.1 Pengujian Sensor"), "4.1.3",
  QStringLiteral("4.1.3 Pengujian Encoder Steering dan Feedback RPM"),
  {
    "Linearitas feedback RPM", "Linearitas encoder steering"
  },
  {
    {
      "RPM perintah", "RPM feedback", "Error", "Error relatif"
    },
    {
      "Steering perintah", "Feedback", "Error"
    }
  },
  {
    {
      "Steer target", "esc_steer_target"
    }, {
      "Steer actual", "esc_steer_actual"
    },
    {
      "Drive target", "esc_drive_target"
    }, {
      "Drive actual", "esc_drive_actual"
    }
  });
  add("navigation", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian dan Tuning Extended Kalman Filter Lokal"), "4.2.1",
  QStringLiteral("4.2.1 Pengujian Parameter frequency"),
  {
    "Pengaruh frequency terhadap RMSE dan latency EKF lokal"
  },
  {
    {
      "frequency", "RMSE v", "RMSE yaw-rate", "Latency median", "CPU EKF", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  },
  {
    { "sample_rate", "Sample Rate (Hz)", "float", "", "", "20.0" },
    { "duration", "Durasi (s)", "float", "", "", "30.0" },
    { "variation", "Variasi / Run", "string", "", "", "frequency-1" },
    { "condition", "Kondisi", "string", "", "", "lintasan lurus" },
    { "frequency", "EKF Lokal frequency (Hz)", "float", "ekf", "ekf_filter_node_odom.ros__parameters.frequency", "20.0" },
    { "sensor_timeout", "Sensor timeout (s)", "yaml_readonly", "ekf", "ekf_filter_node_odom.ros__parameters.sensor_timeout" }
  });
  add("navigation", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian dan Tuning Extended Kalman Filter Lokal"), "4.2.2",
  QStringLiteral("4.2.2 Pengujian Parameter sensor_timeout"),
  {
    "Pengaruh sensor timeout EKF lokal"
  },
  {
    {
      "sensor timeout", "Episode predict-only/menit", "Error akhir 10 m", "Respons terhadap stale", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian dan Tuning Extended Kalman Filter Lokal"), "4.2.3",
  QStringLiteral("4.2.3 Pengujian odom0_twist_rejection_threshold"),
  {
    "Pengaruh twist rejection threshold EKF lokal"
  },
  {
    {
      "Threshold", "Outlier tertolak", "Measurement valid ikut tertolak", "RMSE posisi lokal", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian dan Tuning Extended Kalman Filter Lokal"), "4.2.4",
  QStringLiteral("4.2.4 Pengujian process_noise_covariance"),
  {
    "Pengaruh process noise EKF lokal"
  },
  {
    {
      "Set", "Q(vx)", "Q(vyaw)", "Karakter"
    },
    {
      "Set Q", "RMSE v", "RMSE yaw-rate", "Waktu respons perubahan", "Noise output v", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian dan Tuning Extended Kalman Filter Lokal"), "4.2.5",
  QStringLiteral("4.2.5 Pengujian predict_to_current_time"),
  {
    "Pengaruh predict_to_current_time EKF lokal"
  },
  {
    {
      "Mode", "RMSE v", "Median latency", "Age state saat publish", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian dan Tuning Extended Kalman Filter Lokal"), "4.2.6",
  QStringLiteral("4.2.6 Validasi Konfigurasi Akhir EKF Lokal"),
  {
    "Perbandingan odometri ESC dan EKF lokal pada lintasan lurus"
  },
  {
    {
      "Metrik validasi", "Baseline 30 Hz", "Konfigurasi tuning"
    }
  },
  {
    {
      "ESC v", "esc_odom.v"
    }, {
      "EKF v", "ekf_local.v"
    }, {
      "ESC w", "esc_odom.w"
    }, {
      "EKF w", "ekf_local.w"
    }
  });
  add("navigation", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian dan Tuning Extended Kalman Filter Global"), "4.3.1",
  QStringLiteral("4.3.1 Pengujian Parameter frequency"),
  {
    "Pengaruh frequency EKF global"
  },
  {
    {
      "frequency", "RMSE posisi statis", "Respons dinamis", "CPU", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  },
  {
    { "sample_rate", "Sample Rate (Hz)", "float", "", "", "10.0" },
    { "duration", "Durasi (s)", "float", "", "", "60.0" },
    { "variation", "Variasi / Run", "string", "", "", "frequency-1" },
    { "condition", "Kondisi", "string", "", "", "statis" },
    { "frequency", "EKF Global frequency (Hz)", "float", "ekf", "ekf_filter_node_map.ros__parameters.frequency", "10.0" },
    { "sensor_timeout", "Sensor timeout (s)", "yaml_readonly", "ekf", "ekf_filter_node_map.ros__parameters.sensor_timeout" },
    { "pose_threshold", "odom0_pose_rejection_threshold", "yaml_readonly", "ekf", "ekf_filter_node_map.ros__parameters.odom0_pose_rejection_threshold" }
  });
  add("navigation", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian dan Tuning Extended Kalman Filter Global"), "4.3.2",
  QStringLiteral("4.3.2 Pengujian Parameter sensor_timeout"),
  {
    "Pengaruh sensor timeout EKF global"
  },
  {
    {
      "sensor timeout", "Kontinuitas output", "Peak error setelah gap", "Keterangan"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian dan Tuning Extended Kalman Filter Global"), "4.3.3",
  QStringLiteral("4.3.3 Pengujian odom0_pose_rejection_threshold"),
  {
    "Pengaruh pose rejection threshold EKF global"
  },
  {
    {
      "Threshold", "Spike tertolak", "Measurement valid tertolak", "RMSE posisi", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian dan Tuning Extended Kalman Filter Global"), "4.3.4",
  QStringLiteral("4.3.4 Pengujian process_noise_covariance Posisi X-Y"),
  {
    "Pengaruh process noise posisi EKF global"
  },
  {
    {
      "Qx=Qy", "Std statis", "RMSE dinamis", "Waktu respons", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian dan Tuning Extended Kalman Filter Global"), "4.3.5",
  QStringLiteral("4.3.5 Pengujian predict_to_current_time"),
  {
    "Pengaruh predict_to_current_time EKF global"
  },
  {
    {
      "Mode", "RMSE dinamis", "Peak error", "Karakter output", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian dan Tuning Extended Kalman Filter Global"), "4.3.6",
  QStringLiteral("4.3.6 Validasi Konfigurasi Akhir EKF Global"),
  {
    "Perbandingan scatter GNSS map dan EKF global akhir"
  },
  {
    {
      "Metrik", "GNSS map raw", "EKF global tuning"
    }
  },
  {
    {
      "Map X", "localization_state.map_x"
    }, {
      "EKF X", "ekf_global.x"
    },
    {
      "Map Y", "localization_state.map_y"
    }, {
      "EKF Y", "ekf_global.y"
    }
  });
  add("navigation", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian dan Tuning Koreksi Global LocalizationCore"), "4.4.1",
  QStringLiteral("4.4.1 Pengujian Kalibrasi Koordinat ENU terhadap Map"),
  {
    "Kalibrasi multi-titik ENU terhadap map"
  },
  {
    {
      "Titik", "Error sebelum", "Error sesudah", "ΔX sebelum", "ΔY sebelum", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian dan Tuning Koreksi Global LocalizationCore"), "4.4.2",
  QStringLiteral("4.4.2 Pengujian startup_gnss_samples"),
  {
    "Pengaruh jumlah sampel GNSS startup"
  },
  {
    {
      "Sampel", "Waktu anchor", "Std posisi awal", "Error awal", "Anchor ulang", "Status"
    }
  },
  {
    {
      "Satelit", "gnss_quality.sat"
    }, {
      "DOP", "gnss_quality.dop"
    }, {
      "hAcc", "gnss_quality.hacc_m"
    }
  });
  add("navigation", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian dan Tuning Koreksi Global LocalizationCore"), "4.4.3",
  QStringLiteral("4.4.3 Pengujian strict_correction_alpha"),
  {
    "Pengaruh strict_correction_alpha pada kondisi diam"
  },
  {
    {
      "Alpha diam", "Error posisi", "Settling time", "Max Δ map→odom", "Std posisi akhir", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian dan Tuning Koreksi Global LocalizationCore"), "4.4.4",
  QStringLiteral("4.4.4 Pengujian strict_moving_correction_alpha"),
  {
    "Pengaruh strict_moving_correction_alpha"
  },
  {
    {
      "Alpha", "RMSE global", "Max Δ map→odom", "Settling time", "Tracking RMSE", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian dan Tuning Koreksi Global LocalizationCore"), "4.4.5",
  QStringLiteral("4.4.5 Pengujian strict_max_correction_m"),
  {
    "Trade-off batas koreksi dan tracking"
  },
  {
    {
      "Batas", "Peak TF step", "Konvergensi", "Tracking RMSE", "Peak tracking", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian dan Tuning Koreksi Global LocalizationCore"), "4.4.6",
  QStringLiteral("4.4.6 Validasi Konfigurasi Akhir LocalizationCore"),
  {
    "Gambar 4.24 Perbandingan error koreksi global baseline dan hasil tuning"
  },
  {
    {
      "Metrik", "Baseline", "Hasil tuning", "Perubahan", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian dan Tuning Global Costmap"), "4.5.1",
  QStringLiteral("4.5.1 Pengujian footprint_padding"),
  {
    "Pengaruh footprint_padding terhadap path"
  },
  {
    {
      "Padding", "Min clearance", "Path length", "Koridor sukses", "Tracking RMSE", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian dan Tuning Global Costmap"), "4.5.2",
  QStringLiteral("4.5.2 Pengujian inflation_radius"),
  {
    "Trade-off inflation_radius dan clearance"
  },
  {
    {
      "Radius", "Min clearance", "Path length", "Planning time", "Koridor sukses", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian dan Tuning Global Costmap"), "4.5.3",
  QStringLiteral("4.5.3 Pengujian cost_scaling_factor"),
  {
    "Pengaruh cost_scaling_factor pada clearance path"
  },
  {
    {
      "Faktor", "Min clearance", "Path length", "Planning time", "Tracking RMSE", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian dan Tuning Global Costmap"), "4.5.4",
  QStringLiteral("4.5.4 Validasi Konfigurasi Akhir Global Costmap"),
  {
    "Ringkasan konfigurasi akhir global costmap"
  },
  {
    {
      "Konfigurasi", "Path length", "Min clearance", "Planning time", "Tracking RMSE", "Goal success"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Heading error", "derived.path_heading_error_rad"
    },
    {
      "Path length", "nav_path.length_m"
    }, {
      "Plan latency", "nav_path.planning_latency_ms"
    }
  });
  add("navigation", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian dan Tuning Smac Hybrid-A* Planner"), "4.6.1",
  QStringLiteral("4.6.1 Pengujian minimum_turning_radius"),
  {
    "Pengaruh minimum_turning_radius"
  },
  {
    {
      "Rmin", "Planning time", "Path length", "Tracking RMSE", "Steering saturasi", "Status"
    }
  },
  {
    {
      "Gyro Z", "imu.gz"
    }, {
      "Yaw", "imu.yaw_rad"
    }, {
      "Yaw residual", "imu_status.yaw_residual"
    }
  });
  add("navigation", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian dan Tuning Smac Hybrid-A* Planner"), "4.6.2",
  QStringLiteral("4.6.2 Pengujian downsampling_factor"),
  {
    "Trade-off downsampling dan tracking"
  },
  {
    {
      "Downsampling", "Planning time", "Min clearance", "Tracking RMSE", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian dan Tuning Smac Hybrid-A* Planner"), "4.6.3",
  QStringLiteral("4.6.3 Pengujian angle_quantization_bins"),
  {
    "Pengaruh angle_quantization_bins"
  },
  {
    {
      "Bins", "Resolusi heading", "Planning time", "Tracking RMSE", "Variasi steering", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian dan Tuning Smac Hybrid-A* Planner"), "4.6.4",
  QStringLiteral("4.6.4 Pengujian cost_penalty"),
  {
    "Pengaruh cost_penalty terhadap clearance dan panjang path"
  },
  {
    {
      "cost penalty", "Min clearance", "Path length", "Planning time", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian dan Tuning Smac Hybrid-A* Planner"), "4.6.5",
  QStringLiteral("4.6.5 Pengujian analytic_expansion_max_length"),
  {
    "Pengaruh analytic_expansion_max_length"
  },
  {
    {
      "Max length", "Planning time", "Clearance dekat goal", "Final tracking RMSE", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian dan Tuning Smac Hybrid-A* Planner"), "4.6.6",
  QStringLiteral("4.6.6 Pengujian non_straight_penalty"),
  {
    "Pengaruh non_straight_penalty"
  },
  {
    {
      "Penalty", "Perubahan arah steering", "Path length", "Tracking RMSE", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian dan Tuning Smac Hybrid-A* Planner"), "4.6.7",
  QStringLiteral("4.6.7 Validasi Konfigurasi Akhir Smac Hybrid-A*"),
  {
    "Perbandingan bentuk path baseline dan hasil tuning"
  },
  {
    {
      "Parameter", "Baseline source", "Hasil tuning aktual", "Perubahan utama"
    },
    {
      "Metrik skenario gabungan", "Baseline", "Hasil tuning aktual", "Perubahan"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Heading error", "derived.path_heading_error_rad"
    },
    {
      "Path length", "nav_path.length_m"
    }, {
      "Plan latency", "nav_path.planning_latency_ms"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.1",
  QStringLiteral("4.7.1 Pengujian controller_frequency dan model_dt"),
  {
    "Pengaruh controller_frequency terhadap tracking dan CPU"
  },
  {
    {
      "Pasangan", "Tracking RMSE", "Latency", "CPU", "Miss cycle", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.2",
  QStringLiteral("4.7.2 Pengujian time_steps atau Prediction Horizon"),
  {
    "Pengaruh prediction horizon MPPI"
  },
  {
    {
      "time_steps", "CTE RMSE", "Max CTE", "Compute time", "Std steering", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.3",
  QStringLiteral("4.7.3 Pengujian PathAlignCritic cost_weight"),
  {
    "Pengaruh PathAlignCritic terhadap tracking"
  },
  {
    {
      "Weight", "CTE RMSE", "Heading RMSE", "Std steering", "Time-to-goal", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.4",
  QStringLiteral("4.7.4 Pengujian PathFollowCritic cost_weight"),
  {
    "Pengaruh PathFollowCritic terhadap cross-track error"
  },
  {
    {
      "Weight", "CTE RMSE", "Max CTE", "Recovery time", "Std steering", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.5",
  QStringLiteral("4.7.5 Pengujian PathAngleCritic cost_weight"),
  {
    "Pengaruh PathAngleCritic terhadap heading error"
  },
  {
    {
      "Weight", "CTE RMSE", "Heading RMSE", "Std steering", "Time-to-goal", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.6",
  QStringLiteral("4.7.6 Pengujian vx_max"),
  {
    "Trade-off vx_max terhadap waktu tempuh dan CTE"
  },
  {
    {
      "vx_max", "CTE RMSE", "Max CTE", "Waktu tempuh", "Steering saturasi", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.7",
  QStringLiteral("4.7.7 Konsistensi minimum_turning_r MPPI dengan Planner"),
  {
    "Validasi minimum_turning_radius MPPI"
  },
  {
    {
      "Rmin MPPI", "CTE RMSE", "Saturasi steering", "Radius aktual minimum", "Status"
    }
  },
  {
    {
      "Gyro Z", "imu.gz"
    }, {
      "Yaw", "imu.yaw_rad"
    }, {
      "Yaw residual", "imu_status.yaw_residual"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.8",
  QStringLiteral("4.7.8 Validasi Konfigurasi Akhir MPPI"),
  {
    "Ringkasan konfigurasi akhir MPPI"
  },
  {
    {
      "Konfigurasi", "CTE RMSE", "Max CTE", "Heading RMSE", "Waktu", "Steering saturasi"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "v error", "derived.velocity_error_mps"
    },
    {
      "steer error", "derived.steering_error_rad"
    }, {
      "yaw error", "derived.yaw_error_rps"
    }
  });
  add("navigation", QStringLiteral("4.8"), QStringLiteral("4.8 Pengujian dan Tuning Pipeline Command dan Velocity Smoother"), "4.8.1",
  QStringLiteral("4.8.1 Pengujian Kecepatan Minimum Stabil Kendaraan"),
  {
    "Penentuan kecepatan minimum stabil"
  },
  {
    {
      "Command", "Actual speed", "Kontinu", "Gejala", "Status"
    }
  },
  {
    {
      "Gyro Z", "imu.gz"
    }, {
      "Yaw", "imu.yaw_rad"
    }, {
      "Yaw residual", "imu_status.yaw_residual"
    }
  });
  add("navigation", QStringLiteral("4.8"), QStringLiteral("4.8 Pengujian dan Tuning Pipeline Command dan Velocity Smoother"), "4.8.2",
  QStringLiteral("4.8.2 Pengujian Harmonisasi Low-Speed Deadband"),
  {
    "Pengaruh low-speed deadband terhadap final approach"
  },
  {
    {
      "Threshold set", "Endpoint error", "Start-stop/run", "Command dipotong", "Time-to-goal", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.8"), QStringLiteral("4.8 Pengujian dan Tuning Pipeline Command dan Velocity Smoother"), "4.8.3",
  QStringLiteral("4.8.3 Pengujian smoothing_frequency"),
  {
    "Pengaruh smoothing_frequency pada command"
  },
  {
    {
      "Frekuensi", "Mean Δ command", "Latency", "Speed oscillation", "CTE RMSE", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.8"), QStringLiteral("4.8 Pengujian dan Tuning Pipeline Command dan Velocity Smoother"), "4.8.4",
  QStringLiteral("4.8.4 Pengujian max_accel dan max_decel"),
  {
    "Pengaruh acceleration/deceleration limit"
  },
  {
    {
      "Accel/Decel", "Overshoot speed", "Stop error", "Waktu", "Peak jerk", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.8"), QStringLiteral("4.8 Pengujian dan Tuning Pipeline Command dan Velocity Smoother"), "4.8.5",
  QStringLiteral("4.8.5 Validasi Konfigurasi Akhir Pipeline Command"),
  {
    "Ringkasan konfigurasi akhir pipeline command"
  },
  {
    {
      "Parameter", "Baseline", "Hasil tuning"
    }
  },
  {
    {
      "Nav v", "cmd_nav.linear_x"
    }, {
      "Integrated v", "cmd_autonomy_integrated.linear_x"
    },
    {
      "Final v", "cmd_final.linear_x"
    }, {
      "Actual v", "esc_drive_actual"
    }
  });
  add("navigation", QStringLiteral("4.9"), QStringLiteral("4.9 Pengujian Goal Checker dan Validasi Akurasi Navigasi"), "4.9.1",
  QStringLiteral("4.9.1 Pengujian xy_goal_tolerance"),
  {
    "Hubungan xy_goal_tolerance dan error posisi akhir"
  },
  {
    {
      "Tolerance", "Nav2 success", "Endpoint RMSE", "Max error", "Time-to-goal", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.9"), QStringLiteral("4.9 Pengujian Goal Checker dan Validasi Akurasi Navigasi"), "4.9.2",
  QStringLiteral("4.9.2 Pengujian yaw_goal_tolerance"),
  {
    "Pengaruh yaw_goal_tolerance"
  },
  {
    {
      "Yaw tolerance", "Success", "Mean yaw error", "Time-to-goal", "Retry/reversal", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.9"), QStringLiteral("4.9 Pengujian Goal Checker dan Validasi Akurasi Navigasi"), "4.9.3",
  QStringLiteral("4.9.3 Validasi Navigasi End-to-End"),
  {
    "Sebaran posisi akhir pengujian berulang"
  },
  {
    {
      "Skenario", "CTE RMSE", "Heading RMSE", "Mean endpoint error", "Std endpoint", "Success"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.9"), QStringLiteral("4.9 Pengujian Goal Checker dan Validasi Akurasi Navigasi"), "4.9.4",
  QStringLiteral("4.9.4 Rekapitulasi Parameter Navigasi Akhir"),
  {
    "Ringkasan parameter navigasi akhir"
  },
  {
    {
      "Lapisan", "Parameter", "Kandidat hasil tuning"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.10"), QStringLiteral("4.10 Ringkasan Hubungan Hasil Pengujian"), "4.10",
  QStringLiteral("4.10 Ringkasan Hubungan Hasil Pengujian"),
  {
  },
  {
    {
      "Lapisan", "Ringkasan hasil", "Keterkaitan"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  auto perceptionCommonFields = []() {
    return QVector<ExperimentParameterField>{
      { "sample_rate", "Sample Rate (Hz)", "float", "", "", "10.0" },
      { "duration", "Durasi (s)", "float", "", "", "30.0" },
      { "variation", "Variasi / Run", "string", "", "", "run-1" },
      { "condition", "Kondisi", "string", "", "", "statis" }
    };
  };
  auto perceptionLateralCalibrationFields = [&]() {
    auto v = perceptionCommonFields();
    v << ExperimentParameterField("gt_lateral_cm", "Ground Truth Lateral (cm)", "float", "", "", "0.0", "", true)
      << ExperimentParameterField("calib_mpp", "Kalibrasi meter / pixel", "float", "bbox_calib", "calibration.meter_per_pixel_at_working_distance")
      << ExperimentParameterField("calib_setpoint_x_px", "Setpoint X (px)", "float", "bbox_calib", "setpoint.x_px")
      << ExperimentParameterField("calib_setpoint_y_px", "Setpoint Y (px)", "float", "bbox_calib", "setpoint.y_px")
      << ExperimentParameterField("calib_canvas_w", "Canvas width (px)", "yaml_readonly", "bbox_calib", "video.processing_width")
      << ExperimentParameterField("calib_canvas_h", "Canvas height (px)", "yaml_readonly", "bbox_calib", "video.processing_height");
    return v;
  };
  auto perceptionYawCalibrationFields = [&]() {
    auto v = perceptionCommonFields();
    v << ExperimentParameterField("gt_yaw_deg", "Ground Truth Yaw (deg)", "float", "", "", "0.0", "", true)
      << ExperimentParameterField("calib_fx_px", "Focal length fx (px)", "float", "bbox_calib", "setpoint.focal_length_x_px")
      << ExperimentParameterField("calib_cx_px", "Principal point cx (px)", "float", "bbox_calib", "setpoint.principal_x_px")
      << ExperimentParameterField("calib_yaw_offset_deg", "Yaw zero offset (deg)", "float", "bbox_calib", "yaw.setpoint_yaw_offset_deg")
      << ExperimentParameterField("calib_geometry_sign", "Yaw geometry sign", "yaml_readonly", "bbox_calib", "yaw.geometry_sign")
      << ExperimentParameterField("calib_canvas_w", "Canvas width (px)", "yaml_readonly", "bbox_calib", "video.processing_width")
      << ExperimentParameterField("calib_canvas_h", "Canvas height (px)", "yaml_readonly", "bbox_calib", "video.processing_height");
    return v;
  };
  auto perceptionCombinedCalibrationFields = [&]() {
    auto v = perceptionCommonFields();
    v << ExperimentParameterField("gt_lateral_cm", "Ground Truth Lateral (cm)", "float", "", "", "0.0", "", true)
      << ExperimentParameterField("gt_yaw_deg", "Ground Truth Yaw (deg)", "float", "", "", "0.0", "", true)
      << ExperimentParameterField("calib_mpp", "Kalibrasi meter / pixel", "float", "bbox_calib", "calibration.meter_per_pixel_at_working_distance")
      << ExperimentParameterField("calib_setpoint_x_px", "Setpoint X (px)", "float", "bbox_calib", "setpoint.x_px")
      << ExperimentParameterField("calib_setpoint_y_px", "Setpoint Y (px)", "float", "bbox_calib", "setpoint.y_px")
      << ExperimentParameterField("calib_fx_px", "Focal length fx (px)", "float", "bbox_calib", "setpoint.focal_length_x_px")
      << ExperimentParameterField("calib_cx_px", "Principal point cx (px)", "float", "bbox_calib", "setpoint.principal_x_px")
      << ExperimentParameterField("calib_yaw_offset_deg", "Yaw zero offset (deg)", "float", "bbox_calib", "yaw.setpoint_yaw_offset_deg")
      << ExperimentParameterField("calib_geometry_sign", "Yaw geometry sign", "yaml_readonly", "bbox_calib", "yaw.geometry_sign");
    return v;
  };
  auto perceptionDockingCalibrationFields = [&](bool lateral, bool yaw) {
    auto v = perceptionCombinedCalibrationFields();
    v << ExperimentParameterField("calib_lateral_tol_m", "Toleransi lateral (m)", "float", "bbox_calib", "alignment.lateral_tolerance_m")
      << ExperimentParameterField("calib_yaw_tol_deg", "Toleransi yaw (deg)", "float", "bbox_calib", "alignment.yaw_tolerance_deg");
    if (lateral) v << ExperimentParameterField("kp_lateral", "Kp lateral", "float", "bbox_calib", "control.kp_lateral");
    if (yaw) v << ExperimentParameterField("kp_yaw", "Kp yaw", "float", "bbox_calib", "control.kp_yaw");
    return v;
  };
  /* ------------------------- PERSEPSI ------------------------- */
  // Struktur BAB IV Persepsi mengikuti dokumen sempro pengguna.
  // Perubahan di blok ini hanya mengatur menu/subbab, format tabel, grafik,
  // dan binding telemetry yang sudah tersedia. Runtime perception tidak diubah.
  add("perception", QStringLiteral("4.1"), QStringLiteral("4.1 Dataset YOLOv8"), "4.1.1",
  QStringLiteral("4.1.1 Hasil Implementasi Sistem"),
  {
  },
  {
    {
      "Tahap Sistem", "Input", "Proses", "Output", "Topic / Bukti", "Status"
    }
  },
  {
    {
      "Deteksi", "raw_detections.count"
    }, {
      "Confidence", "raw_detections.mean_confidence"
    }, {
      "FPS", "perception_performance.fps"
    }, {
      "Mean process", "perception_performance.mean_ms"
    }
  });
  add("perception", QStringLiteral("4.1"), QStringLiteral("4.1 Dataset YOLOv8"), "4.1.2",
  QStringLiteral("4.1.2 Implementasi AGV dan Kamera RGB"),
  {
  },
  {
    {
      "Parameter", "Kondisi Aktual", "Satuan", "Keterangan"
    }
  },
  {
    {
      "FPS", "perception_performance.fps"
    }, {
      "Capture drop", "perception_performance.capture_dropped"
    }, {
      "Mean luma", "camera_health_state.mean_luma"
    }, {
      "Camera healthy", "camera_healthy"
    }
  });
  add("perception", QStringLiteral("4.1"), QStringLiteral("4.1 Dataset YOLOv8"), "4.1.3",
  QStringLiteral("4.1.3 Implementasi Posisi Visual dan Error"),
  {
  },
  {
    {
      "Parameter Visual", "Nilai Aktual", "Satuan", "Keterangan"
    }
  },
  {
    {
      "Deteksi", "raw_detections.count"
    }, {
      "Confidence", "raw_detections.mean_confidence"
    }, {
      "FPS", "perception_performance.fps"
    }, {
      "Mean process", "perception_performance.mean_ms"
    }
  });
  add("perception", QStringLiteral("4.1"), QStringLiteral("4.1 Dataset YOLOv8"), "4.1.4",
  QStringLiteral("4.1.4 Implementasi Kontrol dan ROS 2"),
  {
  },
  {
    {
      "Data", "Topic / Interface", "Nilai / Status", "Keterangan"
    }
  },
  {
    {
      "Deteksi", "raw_detections.count"
    }, {
      "Confidence", "raw_detections.mean_confidence"
    }, {
      "FPS", "perception_performance.fps"
    }, {
      "Capture drop", "perception_performance.capture_dropped"
    }
  });
  add("perception", QStringLiteral("4.1"), QStringLiteral("4.1 Dataset YOLOv8"), "4.1.5",
  QStringLiteral("4.1.5 Hasil Training Model YOLOv8"),
  {
    "Training loss dan validation loss terhadap epoch",
    "Precision, recall, F1-score dan mAP model YOLOv8"
  },
  {
    {
      "Metrik", "Nilai Aktual", "Sumber Data", "Keterangan"
    },
    {
      "Kelas", "Precision", "Recall", "F1-Score", "AP / mAP", "Catatan"
    }
  },
  {
    {
      "Deteksi runtime", "raw_detections.count"
    }, {
      "Confidence runtime", "raw_detections.mean_confidence"
    }
  });

  add("perception", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian Deteksi Halangan"), "4.2.1",
  QStringLiteral("4.2.1 Pengujian Variasi Jarak Halangan"),
  {
    "Hubungan jarak aktual halangan terhadap confidence rata-rata",
    "Hubungan jarak aktual halangan terhadap detection rate"
  },
  {
    {
      "Jarak Aktual (m)", "Confidence Rata-rata", "Berhasil / Total", "Detection Rate (%)"
    }
  },
  {
    {
      "Deteksi", "raw_detections.count"
    }, {
      "Confidence", "raw_detections.mean_confidence"
    }, {
      "FPS", "perception_performance.fps"
    }
  });
  add("perception", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian Deteksi Halangan"), "4.2.2",
  QStringLiteral("4.2.2 Pengujian Variasi Posisi Halangan"),
  {
    "Confidence rata-rata pada posisi kiri, tengah, dan kanan",
    "Detection rate pada posisi kiri, tengah, dan kanan"
  },
  {
    {
      "Posisi", "Confidence Rata-rata", "Detection Rate (%)"
    }
  },
  {
    {
      "Deteksi", "raw_detections.count"
    }, {
      "Confidence", "raw_detections.mean_confidence"
    }, {
      "FPS", "perception_performance.fps"
    }
  });
  add("perception", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian Deteksi Halangan"), "4.2.3",
  QStringLiteral("4.2.3 Pengujian Variasi Pencahayaan Halangan"),
  {
    "Hubungan kondisi pencahayaan terhadap confidence rata-rata",
    "Hubungan kondisi pencahayaan terhadap detection rate"
  },
  {
    {
      "Kondisi", "Lux*", "Confidence Rata-rata", "Detection Rate (%)"
    }
  },
  {
    {
      "Confidence", "raw_detections.mean_confidence"
    }, {
      "Mean luma", "camera_health_state.mean_luma"
    }, {
      "Std luma", "camera_health_state.stddev_luma"
    }, {
      "FPS", "perception_performance.fps"
    }
  });

  add("perception", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian Deteksi Pallet"), "4.3.1",
  QStringLiteral("4.3.1 Pengujian Variasi Jarak Pallet"),
  {
    "Hubungan jarak aktual pallet terhadap confidence rata-rata",
    "Hubungan jarak aktual pallet terhadap detection rate"
  },
  {
    {
      "Jarak Aktual (m)", "Confidence Rata-rata", "Detection Rate (%)"
    }
  },
  {
    {
      "Pallet count", "raw_detections.pallet_count"
    }, {
      "Pallet confidence", "raw_detections.pallet_best_confidence"
    }, {
      "Pallet center X", "raw_detections.pallet_best_center_x_px"
    }, {
      "FPS", "perception_performance.fps"
    }
  });
  add("perception", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian Deteksi Pallet"), "4.3.2",
  QStringLiteral("4.3.2 Pengujian Variasi Orientasi Pallet"),
  {
    "Hubungan orientasi pallet terhadap confidence rata-rata",
    "Hubungan orientasi pallet terhadap detection rate"
  },
  {
    {
      "Sudut Pallet (°)", "Confidence Rata-rata", "Detection Rate (%)"
    }
  },
  {
    {
      "Pallet confidence", "raw_detections.pallet_best_confidence"
    }, {
      "Yaw visual", "alignment_state.error_yaw_deg"
    }, {
      "Detection stable", "alignment_state.detection_stable"
    }, {
      "FPS", "perception_performance.fps"
    }
  }, perceptionYawCalibrationFields());
  add("perception", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian Deteksi Pallet"), "4.3.3",
  QStringLiteral("4.3.3 Pengujian Variasi Pencahayaan Pallet"),
  {
    "Hubungan kondisi pencahayaan terhadap confidence rata-rata",
    "Hubungan kondisi pencahayaan terhadap detection rate"
  },
  {
    {
      "Kondisi", "Lux*", "Confidence Rata-rata", "Detection Rate (%)"
    }
  },
  {
    {
      "Confidence", "raw_detections.mean_confidence"
    }, {
      "Mean luma", "camera_health_state.mean_luma"
    }, {
      "Std luma", "camera_health_state.stddev_luma"
    }, {
      "FPS", "perception_performance.fps"
    }
  });

  add("perception", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian Akurasi Error Visual"), "4.4.1",
  QStringLiteral("4.4.1 Pengujian Error Lateral Visual"),
  {
    "Hubungan offset lateral aktual terhadap error pixel",
    "Hubungan offset lateral aktual terhadap error ternormalisasi"
  },
  {
    {
      "Offset Lateral Aktual (cm)", "Target Center X (pixel)", "Image Center X (pixel)", "Error Pixel (pixel)", "Error Ternormalisasi", "Keterangan"
    }
  },
  {
    {
      "Visual error (px)", "derived.visual_error_px"
    }, {
      "Visual error normalized", "derived.visual_error_normalized"
    }, {
      "Lateral alignment (cm)", "derived.alignment_lateral_error_cm"
    }, {
      "Pallet confidence", "raw_detections.pallet_best_confidence"
    }, {
      "FPS", "perception_performance.fps"
    }
  }, perceptionLateralCalibrationFields());
  add("perception", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian Akurasi Error Visual"), "4.4.2",
  QStringLiteral("4.4.2 Pengujian Akurasi Error Yaw"),
  {
    "Hubungan yaw aktual terhadap yaw visual"
  },
  {
    {
      "Yaw Aktual (°)", "Yaw Visual (°)", "Error Absolut (°)"
    }
  },
  {
    {
      "Yaw visual (deg)", "alignment_state.error_yaw_deg"
    }, {
      "Lateral alignment (cm)", "derived.alignment_lateral_error_cm"
    }, {
      "Alignment confidence", "alignment_state.confidence"
    }, {
      "Data valid", "alignment_state.data_valid"
    }, {
      "FPS", "perception_performance.fps"
    }
  }, perceptionYawCalibrationFields());
  add("perception", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian Akurasi Error Visual"), "4.4.3",
  QStringLiteral("4.4.3 Pengujian Error Forward"),
  {
    "Error forward visual terhadap waktu"
  },
  {
    {
      "Offset/Jarak Forward Aktual (cm)", "yB (px)", "yS (px)", "Error Forward (px)", "Status Deteksi"
    }
  },
  {
    {
      "Error forward (px)", "derived.forward_error_px"
    }, {
      "Expected block center Y (px)", "derived.target_center_y_px"
    }, {
      "Forward setpoint Y (px)", "derived.forward_setpoint_y_px"
    }, {
      "Data valid", "alignment_state.data_valid"
    }
  }, {});

  add("perception", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian Kontrol Proporsional Docking"), "4.5.1",
  QStringLiteral("4.5.1 Pengujian Kontrol terhadap Error Lateral Awal"),
  {
    "Respons error lateral terhadap waktu selama docking"
  },
  {
    {
      "Offset Awal (cm)", "Error Visual Awal", "Kp lateral", "Error Visual Akhir", "Settling Time (s)", "Overshoot", "Status"
    }
  },
  {
    {
      "Lateral error (cm)", "derived.alignment_lateral_error_cm"
    }, {
      "PID lateral", "alignment_state.pid_lateral_output"
    }, {
      "Steering estimate", "alignment_state.estimated_steering_deg"
    }, {
      "Velocity cmd", "alignment_state.linear_velocity_cmd"
    }, {
      "FPS", "perception_performance.fps"
    }
  }, perceptionDockingCalibrationFields(true, false));
  add("perception", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian Kontrol Proporsional Docking"), "4.5.2",
  QStringLiteral("4.5.2 Pengujian Kontrol terhadap Error Yaw Awal"),
  {
    "Respons error visual terhadap waktu selama docking"
  },
  {
    {
      "Yaw Awal (deg)", "Error Yaw Awal", "Kp yaw", "Error Yaw Akhir", "Settling Time (s)", "Overshoot", "Status"
    }
  },
  {
    {
      "Yaw error (deg)", "alignment_state.error_yaw_deg"
    }, {
      "PID yaw", "alignment_state.pid_yaw_output"
    }, {
      "Desired yaw rate", "alignment_state.desired_yaw_rate"
    }, {
      "Steering estimate", "alignment_state.estimated_steering_deg"
    }, {
      "FPS", "perception_performance.fps"
    }
  }, perceptionDockingCalibrationFields(false, true));
  add("perception", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian Kontrol Proporsional Docking"), "4.5.3",
  QStringLiteral("4.5.3 Pengujian Kontrol terhadap Error Forward"),
  {
    "Respons error forward terhadap waktu selama pendekatan"
  },
  {
    {
      "No.", "Jarak Awal Aktual (cm)", "Error Forward Awal (px)", "Kp,fwd", "Final Error Forward (px)", "Stopping Error (cm)", "Settling Time (s)", "Overshoot (px)", "Status"
    }
  },
  {
    {
      "Error forward (px)", "derived.forward_error_px"
    }, {
      "Expected block center Y (px)", "derived.target_center_y_px"
    }, {
      "Forward setpoint Y (px)", "derived.forward_setpoint_y_px"
    }, {
      "Velocity preview", "alignment_state.linear_velocity_cmd"
    }, {
      "Data valid", "alignment_state.data_valid"
    }
  }, {});
  /* ------------------------- ESC / BLDC FOC ------------------------- */
  add("steering", QStringLiteral("4.1"), QStringLiteral("4.1 Pengujian Controller Hoverboard STM32F103RCT6"), "4.1",
  QStringLiteral("4.1 Pengujian Controller Hoverboard STM32F103RCT6"),
  {
    "Tegangan DC bus controller", "Telemetry arus dan tegangan controller"
  },
  {
    {
      "Parameter", "Hasil Pengukuran", "Satuan"
    }
  },
  {
    {
      "Vbus", "foc_telemetry.vbus_v"
    }, {
      "Ia", "foc_telemetry.ia_a"
    }, {
      "Ib", "foc_telemetry.ib_a"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }
  });
  add("steering", QStringLiteral("4.1"), QStringLiteral("4.1 Pengujian Controller Hoverboard STM32F103RCT6"), "4.1.1",
  QStringLiteral("4.1.1 Pengujian Pembacaan Hall Sensor"),
  {
    "RPM Hall terhadap setpoint", "Feedback encoder/Hall selama pengujian"
  },
  {
    {
      "Setpoint RPM", "RPM Hall", "RPM Tachometer", "Error (%)"
    }
  },
  {
    {
      "RPM Setpoint", "derived.target_rpm"
    }, {
      "RPM Hall", "derived.actual_rpm"
    }, {
      "Encoder", "foc_telemetry.encoder_count"
    }, {
      "Sektor Elektrik", "foc_telemetry.electrical_sector"
    }
  });
  add("steering", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian Catu Daya"), "4.2",
  QStringLiteral("4.2 Pengujian Catu Daya"),
  {
    "Tegangan catu daya controller selama pengujian"
  },
  {
    {
      "Jalur", "Tegangan Desain", "Tegangan Terukur", "Deviasi (%)"
    }
  },
  {
    {
      "Vbus", "foc_telemetry.vbus_v"
    }
  });
  add("steering", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian Respons Kecepatan Tanpa Beban"), "4.3",
  QStringLiteral("4.3 Pengujian Respons Kecepatan Tanpa Beban"),
  {
    "Respons kecepatan 100 RPM", "Respons kecepatan 150 RPM", "Respons kecepatan 200 RPM"
  },
  {
    {
      "Setpoint", "Steady State", "Rise Time", "Overshoot", "Settling Time", "Error"
    }
  },
  {
    {
      "RPM Setpoint", "derived.target_rpm"
    }, {
      "RPM Aktual", "derived.actual_rpm"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }
  });
  add("steering", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian Respons Kecepatan Tanpa Beban"), "4.3.1",
  QStringLiteral("4.3.1 Perbandingan Ketiga Setpoint"),
  {
    "Perbandingan respons kecepatan 100, 150, dan 200 RPM"
  },
  {
    {
      "Setpoint", "Rise Time", "Overshoot", "Settling Time", "Error Keadaan Tunak"
    }
  },
  {
    {
      "RPM Setpoint", "derived.target_rpm"
    }, {
      "RPM Aktual", "derived.actual_rpm"
    }
  });
  add("steering", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian Respons terhadap Variasi Beban"), "4.4",
  QStringLiteral("4.4 Pengujian Respons terhadap Variasi Beban"),
  {
    "Respons kecepatan terhadap variasi beban", "Arus motor terhadap variasi beban"
  },
  {
    {
      "Beban", "Setpoint", "RPM Awal", "RPM Minimum", "Speed Drop (%)", "Recovery Time", "Arus"
    }
  },
  {
    {
      "RPM Setpoint", "derived.target_rpm"
    }, {
      "RPM Aktual", "derived.actual_rpm"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }, {
      "Ia", "foc_telemetry.ia_a"
    }, {
      "Ib", "foc_telemetry.ib_a"
    }
  });
  add("steering", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian Karakteristik Id dan Iq pada FOC"), "4.5",
  QStringLiteral("4.5 Pengujian Karakteristik Id dan Iq pada FOC"),
  {
    "Karakteristik Id pada FOC", "Karakteristik Iq pada FOC", "Perbandingan Iq pada 100, 150, dan 200 RPM"
  },
  {
    {
      "Setpoint", "Id Ref", "Id Aktual", "Iq Rata-rata", "Arus Motor"
    }
  },
  {
    {
      "RPM Setpoint", "derived.target_rpm"
    }, {
      "Id Aktual", "foc_telemetry.id_a"
    }, {
      "Iq Ref", "foc_telemetry.iq_ref_a"
    }, {
      "Iq Aktual", "foc_telemetry.iq_a"
    }, {
      "Ia", "foc_telemetry.ia_a"
    }, {
      "Ib", "foc_telemetry.ib_a"
    }
  });
  add("steering", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian Kecepatan Motor Kiri dan Kanan"), "4.6",
  QStringLiteral("4.6 Pengujian Kecepatan Motor Kiri dan Kanan"),
  {
    "Perbandingan kecepatan motor kiri dan kanan", "Error sinkronisasi motor kiri-kanan"
  },
  {
    {
      "Setpoint", "Motor Kiri", "Motor Kanan", "Selisih", "Error (%)"
    }
  },
  {
    {
      "RPM Setpoint", "derived.target_rpm"
    }, {
      "RPM Feedback", "derived.actual_rpm"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }
  });
  add("steering", QStringLiteral("4.7"), QStringLiteral("4.7 Analisis Kinerja Keseluruhan"), "4.7",
  QStringLiteral("4.7 Analisis Kinerja Keseluruhan"),
  {
    "Ringkasan respons sistem penggerak BLDC berbasis FOC"
  },
  {
    {
      "Aspek", "Parameter Utama", "Hasil", "Status"
    }
  },
  {
    {
      "RPM Setpoint", "derived.target_rpm"
    }, {
      "RPM Aktual", "derived.actual_rpm"
    }, {
      "Id", "foc_telemetry.id_a"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }, {
      "Vbus", "foc_telemetry.vbus_v"
    }
  });
  add("steering", QStringLiteral("4.8"), QStringLiteral("4.8 Estimasi Grafik BAB IV yang Disarankan"), "4.8",
  QStringLiteral("4.8 Estimasi Grafik BAB IV yang Disarankan"),
  {
    "RPM aktual dan setpoint terhadap waktu untuk 100 RPM, 150 RPM, dan 200 RPM",
    "Perbandingan ketiga respons kecepatan pada satu grafik",
    "RPM terhadap waktu saat beban diberikan pada setpoint 150 RPM",
    "Arus motor terhadap waktu pada variasi beban",
    "Id terhadap waktu",
    "Iq terhadap waktu",
    "Perbandingan Iq pada 100, 150, dan 200 RPM",
    "RPM motor kiri dan kanan terhadap waktu",
    "Error sinkronisasi motor kiri-kanan terhadap waktu atau setpoint"
  },
  {
    {
      "No.", "Grafik yang Disarankan", "Sumber Data", "Status"
    }
  },
  {
    {
      "RPM Setpoint", "derived.target_rpm"
    }, {
      "RPM Aktual", "derived.actual_rpm"
    }, {
      "Id", "foc_telemetry.id_a"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }, {
      "Vbus", "foc_telemetry.vbus_v"
    }
  });
  // clang-format on
  return out;
}
