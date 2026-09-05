// Katalog Navigasi disusun langsung dari dokumen
// BAB_II_III_IV_NAVIGASI_AGV_OFAT_3_MAP_ESTIMASI.docx.
// Satu leaf = satu tabel pengambilan data aktual. Tidak ada angka estimasi
// yang ditanamkan sebagai hasil eksperimen.
auto navField = [](const char *key, const char *label, const char *kind,
                   const char *fileKey = "", const char *yamlPath = "",
                   const char *placeholder = "", bool groundTruth = false) {
  return ExperimentParameterField(
      QString::fromUtf8(key), QString::fromUtf8(label), QString::fromUtf8(kind),
      QString::fromUtf8(fileKey), QString::fromUtf8(yamlPath),
      QString::fromUtf8(placeholder), QString(), groundTruth, false);
};

auto commonNavFields = [&]() {
  return QVector<ExperimentParameterField>{
      navField("sample_rate", "Sample Rate (Hz)", "float", "", "", "5.0"),
      navField("duration", "Durasi (s)", "float", "", "", "30.0"),
      navField("variation", "Variasi / Run ID", "string", "", "", "R1"),
      navField("condition", "Kondisi / Skenario", "string", "", "", "normal")};
};

auto navAdd = [&](const char *groupId, const char *groupTitle, const char *id,
                  const char *section, const char *tableName,
                  std::initializer_list<const char *> columns,
                  std::initializer_list<const char *> graphCaptions,
                  std::initializer_list<std::pair<const char *, const char *>> series,
                  QVector<ExperimentParameterField> extra = {}) {
  ExperimentSpec spec;
  spec.subsystem = QStringLiteral("navigation");
  spec.groupId = QString::fromUtf8(groupId);
  spec.groupTitle = QString::fromUtf8(groupTitle);
  spec.id = QString::fromUtf8(id);
  spec.section = QString::fromUtf8(section);
  spec.tableNames << QString::fromUtf8(tableName);
  QStringList cols;
  for (const char *column : columns) cols << QString::fromUtf8(column);
  spec.tableColumns << cols;
  for (const char *caption : graphCaptions)
    spec.graphCaptions << QString::fromUtf8(caption);
  for (const auto &item : series)
    spec.liveSeries[QString::fromUtf8(item.first)] = QString::fromUtf8(item.second);
  spec.parameterFields = commonNavFields();
  for (const auto &field : extra) spec.parameterFields << field;
  out << spec;
};

const std::initializer_list<std::pair<const char *, const char *>> lidarSeries = {
    {"Range", "lidar.range_center_m"}, {"Valid ratio", "lidar.valid_ratio_pct"},
    {"Dropout", "lidar.dropout_pct"}, {"Scan rate", "lidar.scan_rate_hz"},
    {"Jitter", "lidar.timestamp_jitter_ms"}};
const std::initializer_list<std::pair<const char *, const char *>> imuSeries = {
    {"Yaw", "imu.yaw_rad"}, {"Gyro Z", "imu.gz"}};
const std::initializer_list<std::pair<const char *, const char *>> odomSeries = {
    {"vx odom", "odom.v"}, {"vx EKF", "ekf_local.v"}};
const std::initializer_list<std::pair<const char *, const char *>> slamSeries = {
    {"Map update", "slam.map_update_hz"}, {"CPU", "host.cpu_percent"},
    {"RAM", "host.ram_used_gb"}, {"Unknown", "slam.unknown_pct"}};
const std::initializer_list<std::pair<const char *, const char *>> amclSeries = {
    {"Error posisi", "derived.amcl_pos_error_m"},
    {"Error yaw", "derived.amcl_yaw_error_rad"}, {"CPU", "host.cpu_percent"}};
const std::initializer_list<std::pair<const char *, const char *>> plannerSeries = {
    {"Planning", "nav_path.planning_latency_ms"}, {"Path", "nav_path.length_m"},
    {"CTE", "derived.cte_m"}, {"CPU", "host.cpu_percent"}};

const auto gtDistance = [&]() {
  return QVector<ExperimentParameterField>{
      navField("gt_distance_m", "Jarak Referensi (m)", "float", "", "", "2.0", true)};
};
const auto gtYaw = [&]() {
  return QVector<ExperimentParameterField>{
      navField("gt_yaw_deg", "Yaw Referensi (deg)", "float", "", "", "0.0", true)};
};
const auto gtVx = [&]() {
  return QVector<ExperimentParameterField>{
      navField("gt_vx_mps", "Kecepatan Referensi (m/s)", "float", "", "", "0.20", true)};
};
const auto gtPose = [&]() {
  return QVector<ExperimentParameterField>{
      navField("gt_x", "Ground Truth X (m)", "float", "", "", "0.0", true),
      navField("gt_y", "Ground Truth Y (m)", "float", "", "", "0.0", true),
      navField("gt_yaw_deg", "Ground Truth Yaw (deg)", "float", "", "", "0.0", true)};
};

/* 4.1 — Pengujian Komunikasi dan Data Raw Sensor (DOCX terbaru, Tabel 4.1–4.16) */
const std::initializer_list<std::pair<const char *, const char *>> communicationSeries = {
    {"LiDAR rate", "lidar.scan_rate_hz"}, {"IMU rate", "imu.rate_hz"},
    {"Odom rate", "odom.rate_hz"}, {"LiDAR jitter", "lidar.timestamp_jitter_ms"},
    {"IMU jitter", "imu.timestamp_jitter_ms"}, {"Odom jitter", "odom.timestamp_jitter_ms"}};
const auto communicationTargets = [&]() {
  return QVector<ExperimentParameterField>{
      navField("target_lidar_rate_hz", "Target Rate LiDAR (Hz)", "float", "", "", "10.0", true),
      navField("target_imu_rate_hz", "Target Rate IMU (Hz)", "float", "", "", "100.0", true),
      navField("target_odom_rate_hz", "Target Rate Odom (Hz)", "float", "", "", "50.0", true)};
};

navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.1",
       "4.1.1 Pengujian Komunikasi Sensor",
       "Tabel 4.1 Pengujian komunikasi sensor saat AGV diam",
       {"Sensor", "Topic", "Frame", "Target Rate", "Actual Rate", "Jitter", "Dropout", "Latency", "Status"},
       {}, communicationSeries, communicationTargets());
navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.1b",
       "4.1.1 Pengujian Komunikasi Sensor — Stabilitas pada kondisi aktuator berbeda",
       "Tabel 4.2 Stabilitas komunikasi pada kondisi aktuator berbeda",
       {"Kondisi", "LiDAR Rate", "IMU Rate", "Odom Rate", "LiDAR Dropout", "IMU Dropout", "Odom Dropout", "Catatan"},
       {}, communicationSeries, communicationTargets());

navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.2",
       "4.1.2 Pengujian Raw Data LiDAR",
       "Tabel 4.3 Contoh raw LaserScan",
       {"Sampel", "Sudut (deg)", "Range (m)", "Intensity", "Valid/Invalid", "Timestamp"},
       {}, lidarSeries, {});
navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.2b",
       "4.1.2 Pengujian Raw Data LiDAR — Akurasi terhadap jarak referensi",
       "Tabel 4.4 Akurasi LiDAR terhadap jarak referensi",
       {"Jarak Ref (m)", "Mean LiDAR (m)", "Min", "Max", "Error Mean", "MAE", "RMSE", "Std Dev", "Valid Ratio"},
       {"Reference vs pembacaan LiDAR", "RMSE terhadap jarak", "Distribusi error"}, lidarSeries,
       gtDistance());
navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.2c",
       "4.1.2 Pengujian Raw Data LiDAR — Pengaruh getaran dan aktuator",
       "Tabel 4.5 Pengaruh getaran dan aktuator terhadap LiDAR",
       {"Kondisi", "RMSE Range", "Std Dev", "Valid Ratio", "Dropout", "Scan Rate", "Status"},
       {"Valid ratio pada kondisi aktuator berbeda"}, lidarSeries,
       {navField("gt_distance_m", "Jarak Referensi (m)", "float", "", "", "2.0", true),
        navField("target_lidar_rate_hz", "Target Rate LiDAR (Hz)", "float", "", "", "10.0", true)});
navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.2d",
       "4.1.2 Pengujian Raw Data LiDAR — Kualitas LaserScan saat AGV bergerak",
       "Tabel 4.6 Kualitas LaserScan saat AGV bergerak",
       {"vx AGV (m/s)", "Effective Scan Rate", "Dropout", "Valid Ratio", "Timestamp Jitter", "RMSE Range", "Status"},
       {"Scan rate terhadap kecepatan"}, lidarSeries,
       {navField("gt_distance_m", "Jarak Referensi (m)", "float", "", "", "2.0", true),
        navField("gt_vx_mps", "Kecepatan Referensi (m/s)", "float", "", "", "0.20", true),
        navField("target_lidar_rate_hz", "Target Rate LiDAR (Hz)", "float", "", "", "10.0", true)});

navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.3",
       "4.1.3 Pengujian Raw Data IMU",
       "Tabel 4.7 Contoh raw data IMU",
       {"Sampel", "qx", "qy", "qz", "qw", "Yaw (deg)", "Angular z", "Timestamp"},
       {}, imuSeries, {});
navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.3b",
       "4.1.3 Pengujian Raw Data IMU — Akurasi yaw pada delapan orientasi",
       "Tabel 4.8 Akurasi yaw IMU pada delapan orientasi",
       {"Yaw Ref", "Yaw Mean IMU", "Error", "|Error|", "Std Dev", "RMSE", "Status"},
       {}, imuSeries, gtYaw());
navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.3c",
       "4.1.3 Pengujian Raw Data IMU — Drift yaw saat IMU diam",
       "Tabel 4.9 Drift yaw saat IMU diam",
       {"Durasi", "Yaw Awal", "Yaw Akhir", "Drift", "Drift/min", "Std Dev"},
       {}, imuSeries, {});
navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.3d",
       "4.1.3 Pengujian Raw Data IMU — Pengaruh operasi aktuator",
       "Tabel 4.10 Pengaruh operasi aktuator terhadap yaw IMU",
       {"Kondisi", "Mean Shift Yaw", "Std Dev Yaw", "Spike Maksimum", "Status"},
       {}, imuSeries, {});

navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.4",
       "4.1.4 Pengujian Raw Data Odometri",
       "Tabel 4.11 Contoh raw data odometri",
       {"Sampel", "Timestamp", "vx", "vy", "wz", "Frame", "Catatan"},
       {}, odomSeries, {});
navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.4b",
       "4.1.4 Pengujian Raw Data Odometri — Akurasi vx",
       "Tabel 4.12 Akurasi vx odometri",
       {"v_ref (m/s)", "vx Mean", "Error", "Error Relatif", "RMSE", "Std Dev", "Status"},
       {}, odomSeries, gtVx());
navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.4c",
       "4.1.4 Pengujian Raw Data Odometri — Respons perubahan kecepatan",
       "Tabel 4.13 Respons perubahan kecepatan odometri",
       {"Step Kecepatan", "Rise Time", "Overshoot", "Settling Time", "Steady-State Error", "Status"},
       {}, odomSeries,
       {navField("gt_vx_mps", "Kecepatan Akhir Referensi (m/s)", "float", "", "", "0.20", true)});

navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.5",
       "4.1.5 Analisis Sinkronisasi LiDAR, IMU, dan Odometri",
       "Tabel 4.14 Sinkronisasi data sensor",
       {"Kondisi", "Sensor", "Frequency", "Mean Timestamp Gap", "Max Gap", "Jitter", "Dropout", "Status"},
       {}, communicationSeries, communicationTargets());

/* Tabel 4.15 menggunakan hasil validasi TF. Baris frame dibuat persis seperti template DOCX;
 * nilainya diisi dari hasil tf2/ukur fisik agar tidak mengarang transformasi. */
{
  ExperimentSpec spec;
  spec.subsystem = QStringLiteral("navigation");
  spec.groupId = QStringLiteral("4.1");
  spec.groupTitle = QStringLiteral("4.1 Pengujian Komunikasi dan Data Raw Sensor");
  spec.id = QStringLiteral("4.1.6");
  spec.section = QStringLiteral("4.1.6 Validasi Transformasi TF");
  spec.tableNames << QStringLiteral("Tabel 4.15 Validasi transformasi TF");
  spec.tableColumns << QStringList{QStringLiteral("Parent Frame"), QStringLiteral("Child Frame"),
      QStringLiteral("x"), QStringLiteral("y"), QStringLiteral("z"), QStringLiteral("Roll"),
      QStringLiteral("Pitch"), QStringLiteral("Yaw"), QStringLiteral("Static/Dynamic"), QStringLiteral("Status")};
  spec.recordable = false;
  auto tfRow = [&](const char *parent, const char *child, const char *kind) {
    QVariantMap row;
    row[QStringLiteral("Parent Frame")] = QString::fromUtf8(parent);
    row[QStringLiteral("Child Frame")] = QString::fromUtf8(child);
    row[QStringLiteral("Static/Dynamic")] = QString::fromUtf8(kind);
    spec.defaultRows << row;
  };
  tfRow("map", "odom", "Dynamic");
  tfRow("odom", "base_link", "Dynamic");
  tfRow("base_link", "laser_frame", "Static");
  tfRow("base_link", "imu_link", "Static");
  out << spec;
}

/* 4.1.7 adalah rekap keputusan setelah tabel 4.1–4.15 selesai. */
{
  ExperimentSpec spec;
  spec.subsystem = QStringLiteral("navigation");
  spec.groupId = QStringLiteral("4.1");
  spec.groupTitle = QStringLiteral("4.1 Pengujian Komunikasi dan Data Raw Sensor");
  spec.id = QStringLiteral("4.1.7");
  spec.section = QStringLiteral("4.1.7 Rekapitulasi Kelayakan Data Sensor");
  spec.tableNames << QStringLiteral("Tabel 4.16 Rekapitulasi kelayakan input navigasi");
  spec.tableColumns << QStringList{QStringLiteral("Kelompok"), QStringLiteral("Metrik Kunci"),
      QStringLiteral("Hasil"), QStringLiteral("Kriteria"), QStringLiteral("Keputusan")};
  spec.recordable = false;
  auto recapRow = [&](const char *group, const char *metric, const char *criteria) {
    QVariantMap row;
    row[QStringLiteral("Kelompok")] = QString::fromUtf8(group);
    row[QStringLiteral("Metrik Kunci")] = QString::fromUtf8(metric);
    row[QStringLiteral("Kriteria")] = QString::fromUtf8(criteria);
    spec.defaultRows << row;
  };
  recapRow("Komunikasi", "Rate, jitter, dropout", "Stabil");
  recapRow("LiDAR", "RMSE, valid ratio", "Layak");
  recapRow("IMU yaw", "RMSE, drift", "Layak");
  recapRow("Odometri vx", "RMSE, repeatability", "Layak");
  recapRow("Sinkronisasi", "Gap/jitter", "Konsisten");
  recapRow("TF", "Frame chain", "Valid");
  out << spec;
}

/* 4.2 — LiDAR 2D, Tabel 4.2–4.7 */
navAdd("4.2", "4.2 Pengujian LiDAR 2D", "4.2.1",
       "4.2.1 Pengaruh Jarak terhadap Error Pengukuran",
       "Tabel 4.2 Akurasi LiDAR terhadap jarak",
       {"Jarak ref (m)", "RMSE (m)", "σ (m)", "Valid ratio (%)", "Status"},
       {"Gambar 4.1 RMSE dan simpangan baku LiDAR terhadap jarak"},
       lidarSeries, gtDistance());
navAdd("4.2", "4.2 Pengujian LiDAR 2D", "4.2.2",
       "4.2.2 Pengaruh Getaran dan Aktuator terhadap LaserScan",
       "Tabel 4.3 Pengaruh aktuator/getaran",
       {"Kondisi", "RMSE (m)", "Valid ratio (%)", "Dropout/invalid (%)", "Status"},
       {"Gambar 4.2 Kenaikan RMSE LiDAR akibat aktuator",
        "Gambar 4.3 Valid ratio LaserScan pada kondisi aktuator"},
       lidarSeries, gtDistance());
navAdd("4.2", "4.2 Pengujian LiDAR 2D", "4.2.3",
       "4.2.3 Tuning Scan Frequency", "Tabel 4.4 Variasi scan frequency",
       {"Frekuensi (Hz)", "Timestamp jitter (ms)", "Dropout (%)", "CPU driver (%)", "RMSE range (m)", "Status"},
       {}, lidarSeries,
       {navField("scan_frequency", "Scan Frequency (Hz)", "float", "lidar",
                 "/**.ros__parameters.scan_frequency", "10.0"),
        navField("gt_distance_m", "Jarak Referensi (m)", "float", "", "", "2.0", true)});
navAdd("4.2", "4.2 Pengujian LiDAR 2D", "4.2.4",
       "4.2.4 Pengaruh Sudut Datang Target", "Tabel 4.5 Pengaruh sudut target",
       {"Sudut target", "RMSE (m)", "σ (m)", "Valid ratio (%)", "Status"},
       {}, lidarSeries,
       {navField("gt_distance_m", "Jarak Referensi (m)", "float", "", "", "2.0", true),
        navField("gt_target_angle_deg", "Sudut Target (deg)", "float", "", "", "0.0", true)});
navAdd("4.2", "4.2 Pengujian LiDAR 2D", "4.2.5",
       "4.2.5 Kualitas /scan saat AGV Bergerak", "Tabel 4.6 LaserScan saat bergerak",
       {"vx (m/s)", "Scan rate efektif (Hz)", "Dropout (%)", "Valid ratio (%)", "Timestamp jitter (ms)", "Status"},
       {}, lidarSeries, gtVx());
navAdd("4.2", "4.2 Pengujian LiDAR 2D", "4.2.6",
       "4.2.6 Validasi LiDAR Akhir", "Tabel 4.7 Validasi konfigurasi LiDAR final",
       {"Run", "RMSE 2 m (m)", "Valid ratio (%)", "Dropout (%)", "Scan rate (Hz)", "Status"},
       {}, lidarSeries, gtDistance());

/* 4.3 — IMU yaw, Tabel 4.8–4.12 */
navAdd("4.3", "4.3 Pengujian IMU: Komponen Yaw", "4.3.1",
       "4.3.1 Akurasi Yaw pada Delapan Orientasi", "Tabel 4.8 Akurasi yaw IMU",
       {"Yaw ref", "Yaw IMU", "Error", "|Error|", "Status"},
       {"Gambar 4.4 Error yaw IMU pada delapan orientasi"}, imuSeries, gtYaw());
navAdd("4.3", "4.3 Pengujian IMU: Komponen Yaw", "4.3.2",
       "4.3.2 Drift Yaw saat IMU Diam", "Tabel 4.9 Drift yaw statis",
       {"Durasi", "Yaw awal", "Yaw akhir", "Drift", "σ yaw"}, {}, imuSeries, {});
navAdd("4.3", "4.3 Pengujian IMU: Komponen Yaw", "4.3.3",
       "4.3.3 Repeatability Yaw", "Tabel 4.10 Repeatability yaw pada orientasi 90°",
       {"Run", "Yaw mean", "Error", "σ run", "Status"}, {}, imuSeries, gtYaw());
navAdd("4.3", "4.3 Pengujian IMU: Komponen Yaw", "4.3.4",
       "4.3.4 Respons Yaw pada Rotasi Dinamis", "Tabel 4.11 Respons dinamis yaw",
       {"Kecepatan rotasi", "RMSE yaw", "Overshoot", "Latency", "Status"}, {}, imuSeries,
       {navField("gt_yaw_rate_dps", "Kecepatan Rotasi Referensi (deg/s)", "float", "", "", "10.0", true)});
navAdd("4.3", "4.3 Pengujian IMU: Komponen Yaw", "4.3.5",
       "4.3.5 Pengaruh Operasi Aktuator terhadap Yaw", "Tabel 4.12 Gangguan yaw akibat aktuator",
       {"Kondisi", "Mean shift yaw", "σ yaw", "Spike maksimum", "Status"}, {}, imuSeries, {});

/* 4.4 — Odometri vx, Tabel 4.13–4.17 */
navAdd("4.4", "4.4 Pengujian Odometri Kecepatan Longitudinal vx", "4.4.1",
       "4.4.1 Akurasi vx pada Beberapa Kecepatan", "Tabel 4.13 Akurasi vx odometri",
       {"v_ref (m/s)", "vx odom (m/s)", "Error", "Error relatif", "Status"},
       {"Gambar 4.5 Perbandingan vx odometri terhadap referensi"}, odomSeries, gtVx());
navAdd("4.4", "4.4 Pengujian Odometri Kecepatan Longitudinal vx", "4.4.2",
       "4.4.2 Repeatability vx pada Kecepatan 0,20 m/s", "Tabel 4.14 Repeatability vx",
       {"Run", "vx mean", "σ vx", "RMSE vx", "Status"}, {}, odomSeries, gtVx());
navAdd("4.4", "4.4 Pengujian Odometri Kecepatan Longitudinal vx", "4.4.3",
       "4.4.3 Respons Perubahan Kecepatan", "Tabel 4.15 Respons step vx",
       {"Step command", "Rise time", "Overshoot vx", "Settling time", "Status"}, {}, odomSeries, gtVx());
navAdd("4.4", "4.4 Pengujian Odometri Kecepatan Longitudinal vx", "4.4.4",
       "4.4.4 Konsistensi vx Maju dan Mundur", "Tabel 4.16 Konsistensi arah",
       {"Arah", "|v_ref|", "|vx odom|", "Error", "Status"}, {}, odomSeries, gtVx());
navAdd("4.4", "4.4 Pengujian Odometri Kecepatan Longitudinal vx", "4.4.5",
       "4.4.5 Validasi vx Final", "Tabel 4.17 Validasi vx final",
       {"Run", "v_ref", "vx mean", "RMSE", "σ", "Status"}, {}, odomSeries, gtVx());

/* 4.5 — SLAM tiga map, Tabel 4.18–4.25 */
navAdd("4.5", "4.5 Pengujian SLAM dengan Tiga Hasil Map Berbeda", "4.5.1",
       "4.5.1 Identitas Tiga Hasil Map", "Tabel 4.18 Identitas tiga hasil map",
       {"Parameter/kondisi", "Map 1", "Map 2", "Map 3", "Kontrol"}, {}, slamSeries, {});
navAdd("4.5", "4.5 Pengujian SLAM dengan Tiga Hasil Map Berbeda", "4.5.2",
       "4.5.2 Ringkasan Kinerja Tiap Hasil Map", "Tabel 4.19 Kinerja tiga hasil map",
       {"Metrik", "Map 1", "Map 2", "Map 3", "Terbaik"},
       {"Gambar 4.9 RMSE geometri dan loop closure tiga hasil map"}, slamSeries, {});
navAdd("4.5", "4.5 Pengujian SLAM dengan Tiga Hasil Map Berbeda", "4.5.3",
       "4.5.3 Akurasi Geometri pada 10 Referensi", "Tabel 4.20 Dimensi fisik vs tiga map",
       {"Ref", "Fisik (m)", "Map 1 (m)", "Map 2 (m)", "Map 3 (m)", "Terbaik"},
       {"Gambar 4.10 Error geometri pada 10 referensi"}, slamSeries,
       {navField("gt_map_dimension_m", "Dimensi Fisik Referensi (m)", "float", "", "", "1.0", true)});
navAdd("4.5", "4.5 Pengujian SLAM dengan Tiga Hasil Map Berbeda", "4.5.4",
       "4.5.4 Loop Closure dan Drift", "Tabel 4.21 Loop closure tiga map",
       {"Map", "Run 1 e_loop", "Run 2", "Run 3", "Mean", "Yaw closure mean"}, {}, slamSeries, {});
navAdd("4.5", "4.5 Pengujian SLAM dengan Tiga Hasil Map Berbeda", "4.5.5",
       "4.5.5 Kualitas Struktur Peta", "Tabel 4.22 Kualitas struktur tiga peta",
       {"Metrik", "Map 1", "Map 2", "Map 3", "Kriteria"}, {}, slamSeries, {});
navAdd("4.5", "4.5 Pengujian SLAM dengan Tiga Hasil Map Berbeda", "4.5.6",
       "4.5.6 Kinerja Komputasi Mapping", "Tabel 4.23 Beban komputasi mapping",
       {"Metrik", "Map 1", "Map 2", "Map 3", "Terbaik"}, {}, slamSeries, {});
navAdd("4.5", "4.5 Pengujian SLAM dengan Tiga Hasil Map Berbeda", "4.5.7",
       "4.5.7 Repeatability dan Pemilihan Map Terbaik", "Tabel 4.24 Repeatability tiga hasil map",
       {"Metrik", "Map 1", "Map 2", "Map 3", "Terbaik"}, {}, slamSeries, {});
navAdd("4.5", "4.5 Pengujian SLAM dengan Tiga Hasil Map Berbeda", "4.5.8",
       "4.5.8 Validasi Map Terpilih", "Tabel 4.25 Validasi map terpilih sebagai peta final",
       {"Run", "RMSE_map", "Loop error", "Yaw closure", "Double wall", "Status"}, {}, slamSeries, {});

/* 4.6 — AMCL, Tabel 4.26–4.34 */
navAdd("4.6", "4.6 Pengujian dan Tuning AMCL", "4.6.1",
       "4.6.1 Tuning Rentang min_particles/max_particles", "Tabel 4.26 Tuning rentang partikel AMCL",
       {"min/max", "RMSE pos", "RMSE yaw", "Conv. time", "CPU", "Status"},
       {"Gambar 4.11 RMSE posisi terhadap rentang partikel", "Gambar 4.12 Beban CPU terhadap rentang partikel"}, amclSeries,
       {navField("min_particles", "min_particles", "int", "nav2", "amcl.ros__parameters.min_particles", "500"),
        navField("max_particles", "max_particles", "int", "nav2", "amcl.ros__parameters.max_particles", "2500"),
        navField("gt_x", "Ground Truth X (m)", "float", "", "", "0.0", true),
        navField("gt_y", "Ground Truth Y (m)", "float", "", "", "0.0", true),
        navField("gt_yaw_deg", "Ground Truth Yaw (deg)", "float", "", "", "0.0", true)});
navAdd("4.6", "4.6 Pengujian dan Tuning AMCL", "4.6.2", "4.6.2 Tuning update_min_d",
       "Tabel 4.27 Tuning update_min_d", {"update_min_d", "RMSE pos", "Conv. time", "CPU", "Update rate", "Status"},
       {"Gambar 4.13 Tuning update_min_d AMCL"}, amclSeries,
       {navField("update_min_d", "update_min_d (m)", "float", "nav2", "amcl.ros__parameters.update_min_d", "0.10"),
        navField("gt_x", "Ground Truth X (m)", "float", "", "", "0.0", true),
        navField("gt_y", "Ground Truth Y (m)", "float", "", "", "0.0", true)});
navAdd("4.6", "4.6 Pengujian dan Tuning AMCL", "4.6.3", "4.6.3 Tuning update_min_a",
       "Tabel 4.28 Tuning update_min_a", {"update_min_a", "RMSE yaw", "Conv. time", "CPU", "Update belokan", "Status"},
       {"Gambar 4.14 Tuning update_min_a AMCL"}, amclSeries,
       {navField("update_min_a", "update_min_a (rad)", "float", "nav2", "amcl.ros__parameters.update_min_a", "0.10"),
        navField("gt_yaw_deg", "Ground Truth Yaw (deg)", "float", "", "", "0.0", true)});
navAdd("4.6", "4.6 Pengujian dan Tuning AMCL", "4.6.4", "4.6.4 Tuning pf_err",
       "Tabel 4.29 Tuning pf_err", {"pf_err", "RMSE pos", "Particle mean", "Particle peak", "CPU", "Status"}, {}, amclSeries,
       {navField("pf_err", "pf_err", "float", "nav2", "amcl.ros__parameters.pf_err", "0.05"),
        navField("gt_x", "Ground Truth X (m)", "float", "", "", "0.0", true),
        navField("gt_y", "Ground Truth Y (m)", "float", "", "", "0.0", true)});
navAdd("4.6", "4.6 Pengujian dan Tuning AMCL", "4.6.5", "4.6.5 Tuning pf_z",
       "Tabel 4.30 Tuning pf_z", {"pf_z", "RMSE pos", "Conv. time", "Particle mean", "CPU", "Status"}, {}, amclSeries,
       {navField("pf_z", "pf_z", "float", "nav2", "amcl.ros__parameters.pf_z", "0.99"),
        navField("gt_x", "Ground Truth X (m)", "float", "", "", "0.0", true),
        navField("gt_y", "Ground Truth Y (m)", "float", "", "", "0.0", true)});
navAdd("4.6", "4.6 Pengujian dan Tuning AMCL", "4.6.6", "4.6.6 Tuning sigma_hit",
       "Tabel 4.31 Tuning sigma_hit", {"sigma_hit", "RMSE pos", "RMSE yaw", "Recovery", "CPU", "Status"}, {}, amclSeries,
       {navField("sigma_hit", "sigma_hit (m)", "float", "nav2", "amcl.ros__parameters.sigma_hit", "0.20"),
        navField("gt_x", "Ground Truth X (m)", "float", "", "", "0.0", true),
        navField("gt_y", "Ground Truth Y (m)", "float", "", "", "0.0", true),
        navField("gt_yaw_deg", "Ground Truth Yaw (deg)", "float", "", "", "0.0", true)});
navAdd("4.6", "4.6 Pengujian dan Tuning AMCL", "4.6.7", "4.6.7 Tuning max_beams",
       "Tabel 4.32 Tuning max_beams", {"max_beams", "RMSE pos", "Conv. time", "CPU", "Scan likelihood", "Status"}, {}, amclSeries,
       {navField("max_beams", "max_beams", "int", "nav2", "amcl.ros__parameters.max_beams", "60"),
        navField("gt_x", "Ground Truth X (m)", "float", "", "", "0.0", true),
        navField("gt_y", "Ground Truth Y (m)", "float", "", "", "0.0", true)});
navAdd("4.6", "4.6 Pengujian dan Tuning AMCL", "4.6.8", "4.6.8 Tuning laser_likelihood_max_dist",
       "Tabel 4.33 Tuning likelihood max distance", {"Max dist", "RMSE pos", "RMSE yaw", "Koridor simetris", "Recovery", "Status"}, {}, amclSeries,
       {navField("laser_likelihood_max_dist", "laser_likelihood_max_dist (m)", "float", "nav2", "amcl.ros__parameters.laser_likelihood_max_dist", "2.0"),
        navField("gt_x", "Ground Truth X (m)", "float", "", "", "0.0", true),
        navField("gt_y", "Ground Truth Y (m)", "float", "", "", "0.0", true),
        navField("gt_yaw_deg", "Ground Truth Yaw (deg)", "float", "", "", "0.0", true)});
navAdd("4.6", "4.6 Pengujian dan Tuning AMCL", "4.6.9", "4.6.9 Validasi AMCL Akhir",
       "Tabel 4.34 Validasi AMCL akhir", {"Skenario", "RMSE pos", "RMSE yaw", "Conv. time", "Lost count", "Success"}, {}, amclSeries, gtPose());

/* 4.7 — Global Costmap, Tabel 4.35–4.39 */
navAdd("4.7", "4.7 Pengujian dan Tuning Global Costmap", "4.7.1", "4.7.1 Tuning Resolution",
       "Tabel 4.35 Tuning costmap resolution", {"Resolution", "Planning time", "Min clearance", "Path length", "CPU", "Status"}, {}, plannerSeries,
       {navField("costmap_resolution", "Global Costmap Resolution (m)", "float", "nav2", "global_costmap.global_costmap.ros__parameters.resolution", "0.05")});
navAdd("4.7", "4.7 Pengujian dan Tuning Global Costmap", "4.7.2", "4.7.2 Tuning footprint_padding",
       "Tabel 4.36 Tuning footprint_padding", {"Padding", "Min clearance", "Path length", "Koridor success", "Tracking RMSE", "Status"}, {}, plannerSeries,
       {navField("footprint_padding", "footprint_padding (m)", "float", "nav2", "global_costmap.global_costmap.ros__parameters.footprint_padding", "0.0")});
navAdd("4.7", "4.7 Pengujian dan Tuning Global Costmap", "4.7.3", "4.7.3 Tuning inflation_radius",
       "Tabel 4.37 Tuning inflation_radius", {"Radius", "Min clearance", "Path length", "Planning time", "Koridor success", "Status"},
       {"Gambar 4.15 Hubungan inflation radius dan minimum clearance"}, plannerSeries,
       {navField("inflation_radius", "inflation_radius (m)", "float", "nav2", "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius", "0.45")});
navAdd("4.7", "4.7 Pengujian dan Tuning Global Costmap", "4.7.4", "4.7.4 Tuning cost_scaling_factor",
       "Tabel 4.38 Tuning cost_scaling_factor", {"Scaling", "Min clearance", "Path length", "Planning time", "Tracking RMSE", "Status"}, {}, plannerSeries,
       {navField("cost_scaling_factor", "cost_scaling_factor", "float", "nav2", "global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor", "12.0")});
navAdd("4.7", "4.7 Pengujian dan Tuning Global Costmap", "4.7.5", "4.7.5 Validasi Costmap Akhir",
       "Tabel 4.39 Validasi costmap final", {"Skenario", "Path length", "Min clearance", "Collision/near miss", "Path available", "Status"}, {}, plannerSeries, {});

/* 4.8 — Smac Hybrid-A*, Tabel 4.40–4.48 */
navAdd("4.8", "4.8 Pengujian dan Tuning Smac Hybrid-A*", "4.8.1", "4.8.1 Tuning minimum_turning_radius",
       "Tabel 4.40 Tuning minimum_turning_radius", {"Rmin", "Planning time", "Path length", "Tracking RMSE", "Steering saturation", "Status"},
       {"Gambar 4.16 Tuning minimum turning radius Smac"}, plannerSeries,
       {navField("minimum_turning_radius", "minimum_turning_radius (m)", "float", "nav2", "planner_server.ros__parameters.GridBased.minimum_turning_radius", "2.0")});
navAdd("4.8", "4.8 Pengujian dan Tuning Smac Hybrid-A*", "4.8.2", "4.8.2 Tuning Downsampling Costmap",
       "Tabel 4.41 Tuning downsampling", {"Mode", "Planning time", "Path length", "Tracking RMSE", "Min clearance", "Status"}, {}, plannerSeries,
       {navField("downsample_costmap", "downsample_costmap (true/false)", "bool", "nav2", "planner_server.ros__parameters.GridBased.downsample_costmap", "false"),
        navField("downsampling_factor", "downsampling_factor", "int", "nav2", "planner_server.ros__parameters.GridBased.downsampling_factor", "1")});
navAdd("4.8", "4.8 Pengujian dan Tuning Smac Hybrid-A*", "4.8.3", "4.8.3 Tuning angle_quantization_bins",
       "Tabel 4.42 Tuning angle_quantization_bins", {"Bins", "Planning time", "Path length", "Tracking RMSE", "Steering changes", "Status"},
       {"Gambar 4.17 Planning time terhadap angle quantization bins"}, plannerSeries,
       {navField("angle_quantization_bins", "angle_quantization_bins", "int", "nav2", "planner_server.ros__parameters.GridBased.angle_quantization_bins", "72")});
navAdd("4.8", "4.8 Pengujian dan Tuning Smac Hybrid-A*", "4.8.4", "4.8.4 Tuning cost_penalty",
       "Tabel 4.43 Tuning cost_penalty", {"Penalty", "Planning time", "Path length", "Min clearance", "Tracking RMSE", "Status"},
       {"Gambar 4.18 cost_penalty terhadap clearance"}, plannerSeries,
       {navField("cost_penalty", "cost_penalty", "float", "nav2", "planner_server.ros__parameters.GridBased.cost_penalty", "2.0")});
navAdd("4.8", "4.8 Pengujian dan Tuning Smac Hybrid-A*", "4.8.5", "4.8.5 Tuning analytic_expansion_max_length",
       "Tabel 4.44 Tuning analytic_expansion_max_length", {"Max length", "Planning time", "Path length", "Clearance dekat goal", "Failure", "Status"}, {}, plannerSeries,
       {navField("analytic_expansion_max_length", "analytic_expansion_max_length (m)", "float", "nav2", "planner_server.ros__parameters.GridBased.analytic_expansion_max_length", "10.0")});
navAdd("4.8", "4.8 Pengujian dan Tuning Smac Hybrid-A*", "4.8.6", "4.8.6 Tuning non_straight_penalty",
       "Tabel 4.45 Tuning non_straight_penalty", {"Penalty", "Path length", "Jumlah belokan", "Tracking RMSE", "Planning time", "Status"}, {}, plannerSeries,
       {navField("non_straight_penalty", "non_straight_penalty", "float", "nav2", "planner_server.ros__parameters.GridBased.non_straight_penalty", "1.2")});
navAdd("4.8", "4.8 Pengujian dan Tuning Smac Hybrid-A*", "4.8.7", "4.8.7 Tuning retrospective_penalty",
       "Tabel 4.46 Tuning retrospective_penalty", {"Penalty", "Planning time", "Path length", "Node expanded relatif", "Tracking RMSE", "Status"}, {}, plannerSeries,
       {navField("retrospective_penalty", "retrospective_penalty", "float", "nav2", "planner_server.ros__parameters.GridBased.retrospective_penalty", "0.015")});
navAdd("4.8", "4.8 Pengujian dan Tuning Smac Hybrid-A*", "4.8.8", "4.8.8 Tuning Smoother w_smooth",
       "Tabel 4.47 Tuning smoother w_smooth", {"w_smooth", "Path length", "Curvature peak", "Tracking RMSE", "Clearance min", "Status"}, {}, plannerSeries,
       {navField("w_smooth", "smoother.w_smooth", "float", "nav2", "planner_server.ros__parameters.GridBased.smoother.w_smooth", "0.30")});
navAdd("4.8", "4.8 Pengujian dan Tuning Smac Hybrid-A*", "4.8.9", "4.8.9 Validasi Smac Hybrid-A* Akhir",
       "Tabel 4.48 Validasi planner akhir", {"Skenario", "Planning time", "Path length", "Min clearance", "CTE", "Success"},
       {"Gambar 4.19 Ringkasan baseline dan hasil tuning Smac"}, plannerSeries, {});

/* 4.9 — End-to-end, Tabel 4.49–4.53 */
navAdd("4.9", "4.9 Validasi Navigasi End-to-End", "4.9.1", "4.9.1 Skenario Lurus",
       "Tabel 4.49 Validasi Lurus", {"Run", "Waktu (s)", "Final e_pos", "Final e_yaw", "CTE", "Status"}, {}, plannerSeries, {});
navAdd("4.9", "4.9 Validasi Navigasi End-to-End", "4.9.2", "4.9.2 Skenario Belok 90°",
       "Tabel 4.50 Validasi Belok 90°", {"Run", "Waktu (s)", "Final e_pos", "Final e_yaw", "CTE", "Status"}, {}, plannerSeries, {});
navAdd("4.9", "4.9 Validasi Navigasi End-to-End", "4.9.3", "4.9.3 Skenario Koridor",
       "Tabel 4.51 Validasi Koridor", {"Run", "Waktu (s)", "Final e_pos", "Final e_yaw", "CTE", "Status"}, {}, plannerSeries, {});
navAdd("4.9", "4.9 Validasi Navigasi End-to-End", "4.9.4", "4.9.4 Skenario U-turn",
       "Tabel 4.52 Validasi U-turn", {"Run", "Waktu (s)", "Final e_pos", "Final e_yaw", "CTE", "Status"}, {}, plannerSeries, {});
navAdd("4.9", "4.9 Validasi Navigasi End-to-End", "4.9.5", "4.9.5 Skenario Kompleks",
       "Tabel 4.53 Validasi Kompleks", {"Run", "Waktu (s)", "Final e_pos", "Final e_yaw", "CTE", "Status"},
       {"Gambar 4.20 Final position error pada lima skenario"}, plannerSeries, {});

/* Rekap Tabel 4.54–4.55 */
navAdd("4.10", "4.10 Rekapitulasi Konfigurasi Kandidat Akhir", "4.10.1",
       "4.10 Rekapitulasi Konfigurasi Kandidat Akhir", "Tabel 4.54 Rekap kandidat hasil cumulative OFAT",
       {"Lapisan", "Parameter", "Baseline/source", "Kandidat akhir", "Dasar"}, {}, plannerSeries, {});
navAdd("4.11", "4.11 Rekap Estimasi Peningkatan Performa", "4.11.1",
       "4.11 Rekap Performa Aktual Setelah Tuning", "Tabel 4.55 Baseline vs sistem setelah tuning",
       {"Metrik", "Baseline", "Setelah tuning", "Perubahan"}, {}, plannerSeries, {});
