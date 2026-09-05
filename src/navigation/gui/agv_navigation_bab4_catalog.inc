// Katalog NAVIGATION GUI — disusun dari BAB IV TA_Ernanta_Revisi_Final.docx.
// Struktur leaf mengikuti dokumen terbaru secara langsung:
// 4.1 raw sensor -> 4.2 SLAM -> 4.3 AMCL -> 4.4 Smac Hybrid-A* -> 4.5 end-to-end.
// Tidak ada nilai estimasi/simulasi yang ditanamkan sebagai hasil eksperimen.

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

const std::initializer_list<std::pair<const char *, const char *>> communicationSeries = {
    {"LiDAR rate", "lidar.scan_rate_hz"}, {"IMU rate", "imu.rate_hz"},
    {"Odom rate", "odom.rate_hz"}, {"LiDAR jitter", "lidar.timestamp_jitter_ms"},
    {"IMU jitter", "imu.timestamp_jitter_ms"}, {"Odom jitter", "odom.timestamp_jitter_ms"}};
const std::initializer_list<std::pair<const char *, const char *>> lidarSeries = {
    {"Range", "lidar.range_center_m"}, {"Valid ratio", "lidar.valid_ratio_pct"},
    {"Dropout", "lidar.dropout_pct"}, {"Scan rate", "lidar.scan_rate_hz"},
    {"Jitter", "lidar.timestamp_jitter_ms"}};
const std::initializer_list<std::pair<const char *, const char *>> imuSeries = {
    {"Yaw", "imu.yaw_rad"}, {"Gyro Z", "imu.gz"}, {"Accel X", "imu.ax"},
    {"Accel Y", "imu.ay"}, {"Accel Z", "imu.az"}};
const std::initializer_list<std::pair<const char *, const char *>> odomSeries = {
    {"vx", "odom.v"}, {"wz", "odom.w"}, {"x", "odom.x"}, {"y", "odom.y"}};
const std::initializer_list<std::pair<const char *, const char *>> slamSeries = {
    {"Map update", "slam.map_update_hz"}, {"Unknown", "slam.unknown_pct"},
    {"Occupied", "slam.occupied_pct"}, {"CPU", "host.cpu_percent"},
    {"RAM", "host.ram_used_gb"}};
const std::initializer_list<std::pair<const char *, const char *>> amclSeries = {
    {"AMCL X", "amcl.x"}, {"AMCL Y", "amcl.y"}, {"AMCL Yaw", "amcl.yaw"},
    {"Error posisi", "derived.amcl_pos_error_m"},
    {"Error yaw", "derived.amcl_yaw_error_rad"}, {"CPU", "host.cpu_percent"}};
const std::initializer_list<std::pair<const char *, const char *>> plannerSeries = {
    {"Planning time", "nav_path.planning_latency_ms"}, {"Path length", "nav_path.length_m"},
    {"CTE", "derived.cte_m"}, {"Heading error", "derived.path_heading_error_rad"},
    {"Endpoint error", "derived.endpoint_error_m"}};
const std::initializer_list<std::pair<const char *, const char *>> navEndSeries = {
    {"Path length", "nav_path.length_m"}, {"CTE", "derived.cte_m"},
    {"Endpoint error", "derived.endpoint_error_m"},
    {"Goal yaw error", "derived.goal_yaw_error_rad"},
    {"vx", "odom.v"}};

const auto gtDistance = [&]() {
  return QVector<ExperimentParameterField>{
      navField("gt_distance_m", "Jarak Referensi (m)", "float", "", "", "1.0", true)};
};
const auto gtPose = [&]() {
  return QVector<ExperimentParameterField>{
      navField("gt_x", "Ground Truth X (m)", "float", "", "", "0.0", true),
      navField("gt_y", "Ground Truth Y (m)", "float", "", "", "0.0", true),
      navField("gt_yaw_deg", "Ground Truth Yaw (deg)", "float", "", "", "0.0", true)};
};
const auto sensorTargets = [&]() {
  return QVector<ExperimentParameterField>{
      navField("target_lidar_rate_hz", "Target Rate LiDAR (Hz)", "float", "", "", "10.0", true),
      navField("target_imu_rate_hz", "Target Rate IMU (Hz)", "float", "", "", "50.0", true),
      navField("target_odom_rate_hz", "Target Rate Odom (Hz)", "float", "", "", "50.0", true)};
};

/* ========================================================================== */
/* 4.1 Pengujian Komunikasi dan Data Raw Sensor                              */
/* ========================================================================== */
navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.1",
       "4.1.1 Pengujian Komunikasi Sensor",
       "Tabel 4.1 Validasi komunikasi dan data sensor",
       {"Sensor", "Topic", "Frame", "Target Rate", "Actual Rate", "Jitter",
        "Dropout", "Latency", "Status"},
       {}, communicationSeries, sensorTargets());

navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.2",
       "4.1.2 Pengujian Data Raw LiDAR",
       "Tabel 4.2 Pengujian akurasi data raw LiDAR",
       {"Jarak Ref (m)", "Mean LiDAR (m)", "Error Mean (m)", "MAE (m)",
        "RMSE (m)", "Std Dev (m)", "Valid Ratio (%)", "Status"},
       {"Gambar 4.1 Error pengukuran LiDAR terhadap jarak referensi"},
       lidarSeries, gtDistance());

navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.3",
       "4.1.3 Pengujian Data Raw IMU",
       "Data raw IMU",
       {"Sampel", "qx", "qy", "qz", "qw", "Yaw (deg)", "Angular z", "Timestamp"},
       {}, imuSeries, {});

navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.4",
       "4.1.4 Pengujian Data Raw Odometri",
       "Data raw odometri",
       {"Sampel", "Timestamp", "vx", "vy", "wz", "Frame", "Catatan"},
       {}, odomSeries, {});

navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.5",
       "4.1.5 Analisis Sinkronisasi LiDAR, IMU, dan Odometri",
       "Analisis sinkronisasi data sensor",
       {"Kondisi", "Sensor", "Frequency", "Mean Timestamp Gap", "Max Gap",
        "Jitter", "Dropout", "Status"},
       {"Sinkronisasi rate LiDAR, IMU, dan odometri"},
       communicationSeries, sensorTargets());

{
  ExperimentSpec spec;
  spec.subsystem = QStringLiteral("navigation");
  spec.groupId = QStringLiteral("4.1");
  spec.groupTitle = QStringLiteral("4.1 Pengujian Komunikasi dan Data Raw Sensor");
  spec.id = QStringLiteral("4.1.6");
  spec.section = QStringLiteral("4.1.6 Validasi Transformasi TF");
  spec.tableNames << QStringLiteral("Validasi rantai transformasi TF");
  spec.tableColumns << QStringList{
      QStringLiteral("Parent Frame"), QStringLiteral("Child Frame"),
      QStringLiteral("x"), QStringLiteral("y"), QStringLiteral("z"),
      QStringLiteral("Roll"), QStringLiteral("Pitch"), QStringLiteral("Yaw"),
      QStringLiteral("Static/Dynamic"), QStringLiteral("Status")};
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
  tfRow("base_link", "lidar_link", "Static");
  tfRow("base_link", "imu_link", "Static");
  out << spec;
}

/* ========================================================================== */
/* 4.2 Pengujian SLAM                                                        */
/* ========================================================================== */
auto slamMapFields = [&](const char *mapName) {
  return QVector<ExperimentParameterField>{
      navField("map_name", "Identitas Map", "string", "", "", mapName),
      navField("reference_dimension_m", "Dimensi Referensi Fisik (m)", "float", "", "", "", true)};
};

navAdd("4.2", "4.2 Pengujian SLAM", "4.2.1",
       "4.2.1 Hasil Mapping 1",
       "Hasil Mapping 1",
       {"Map", "Resolution (m/cell)", "Width (m)", "Height (m)", "Unknown (%)",
        "Occupied (%)", "Map Update (Hz)", "CPU Mean (%)", "RAM Mean (GB)", "Catatan"},
       {}, slamSeries, slamMapFields("Map 1"));

navAdd("4.2", "4.2 Pengujian SLAM", "4.2.2",
       "4.2.2 Hasil Mapping 2",
       "Hasil Mapping 2",
       {"Map", "Resolution (m/cell)", "Width (m)", "Height (m)", "Unknown (%)",
        "Occupied (%)", "Map Update (Hz)", "CPU Mean (%)", "RAM Mean (GB)", "Catatan"},
       {}, slamSeries, slamMapFields("Map 2"));

navAdd("4.2", "4.2 Pengujian SLAM", "4.2.3",
       "4.2.3 Hasil Mapping 3",
       "Hasil Mapping 3",
       {"Map", "Resolution (m/cell)", "Width (m)", "Height (m)", "Unknown (%)",
        "Occupied (%)", "Map Update (Hz)", "CPU Mean (%)", "RAM Mean (GB)", "Catatan"},
       {}, slamSeries, slamMapFields("Map 3"));

navAdd("4.2", "4.2 Pengujian SLAM", "4.2.4",
       "4.2.4 Loop Closure dan Drift",
       "Evaluasi loop closure dan drift",
       {"Map", "Run", "Error Loop Closure (m)", "Error Yaw Closure (deg)",
        "Indikasi Double Wall", "Pergeseran Koridor", "Status"},
       {}, slamSeries,
       {navField("map_name", "Map", "string", "", "", "Map 1"),
        navField("loop_error_m", "Error Loop Closure Aktual (m)", "float", "", "", "", true),
        navField("loop_yaw_error_deg", "Error Yaw Closure Aktual (deg)", "float", "", "", "", true)});

navAdd("4.2", "4.2 Pengujian SLAM", "4.2.5",
       "4.2.5 Analisis Kualitas Struktur Map",
       "Evaluasi kualitas struktur map",
       {"Map", "Titik/Dimensi Referensi", "Fisik (m)", "Map (m)", "Error (m)",
        "MAE (m)", "RMSE Struktur (m)", "Catatan"},
       {}, slamSeries,
       {navField("map_name", "Map", "string", "", "", "Map 1"),
        navField("reference_name", "Titik/Dimensi Referensi", "string", "", "", "R1"),
        navField("reference_dimension_m", "Jarak Fisik (m)", "float", "", "", "", true),
        navField("map_dimension_m", "Jarak pada Map (m)", "float", "", "", "", true)});

navAdd("4.2", "4.2 Pengujian SLAM", "4.2.6",
       "4.2.6 Perbandingan dan Pemilihan Map Terbaik",
       "Tabel 4.3 Perbandingan tiga hasil pemetaan",
       {"Metrik", "Map 1", "Map 2", "Map 3", "Terbaik", "Keputusan"},
       {"Gambar 4.2 Perbandingan RMSE struktur tiga map"},
       slamSeries, {});

navAdd("4.2", "4.2 Pengujian SLAM", "4.2.7",
       "4.2.7 Validasi Map Terpilih",
       "Validasi map terpilih",
       {"Map Terpilih", "Resolution", "Origin", "Frame Map", "Coverage",
        "Scan Selaras dengan Map", "Offset", "Status"},
       {}, slamSeries,
       {navField("selected_map", "Map Terpilih", "string", "", "", "Map 3")});

/* ========================================================================== */
/* 4.3 Pengujian dan Tuning AMCL                                             */
/* ========================================================================== */
navAdd("4.3", "4.3 Pengujian dan Tuning AMCL", "4.3.1",
       "4.3.1 Akurasi Pose Statis AMCL",
       "Tabel 4.4 Akurasi pose statis AMCL",
       {"Pose Uji", "GT X (m)", "GT Y (m)", "GT Yaw (deg)",
        "AMCL X (m)", "AMCL Y (m)", "AMCL Yaw (deg)",
        "Error Posisi (cm)", "Error Yaw (deg)", "Status"},
       {"Error posisi dan yaw AMCL pada pose statis"},
       amclSeries, gtPose());

navAdd("4.3", "4.3 Pengujian dan Tuning AMCL", "4.3.2",
       "4.3.2 Repeatability Pose AMCL",
       "Repeatability pose AMCL",
       {"Run", "Pose Uji", "AMCL X (m)", "AMCL Y (m)", "AMCL Yaw (deg)",
        "Error Posisi (cm)", "Error Yaw (deg)", "Var X", "Var Y", "Var Yaw", "Status"},
       {}, amclSeries, gtPose());

navAdd("4.3", "4.3 Pengujian dan Tuning AMCL", "4.3.3",
       "4.3.3 Waktu Konvergensi Initial Pose",
       "Waktu konvergensi initial pose AMCL",
       {"Run", "Pose Awal", "Waktu Konvergensi (s)", "Final Error Posisi (cm)",
        "Final Error Yaw (deg)", "Var X Akhir", "Var Y Akhir", "Var Yaw Akhir", "Status"},
       {"Waktu konvergensi AMCL"},
       amclSeries, gtPose());

navAdd("4.3", "4.3 Pengujian dan Tuning AMCL", "4.3.4",
       "4.3.4 Tuning min_particles / max_particles",
       "Tabel 4.5 Tuning jumlah partikel AMCL",
       {"min_particles", "max_particles", "Waktu Konvergensi (s)",
        "Error Posisi (cm)", "Error Yaw (deg)", "CPU Mean (%)", "Status"},
       {"Pengaruh jumlah partikel terhadap error dan beban CPU"},
       amclSeries,
       {navField("min_particles", "min_particles", "int", "nav2",
                 "amcl.ros__parameters.min_particles", "500"),
        navField("max_particles", "max_particles", "int", "nav2",
                 "amcl.ros__parameters.max_particles", "2000"),
        navField("gt_x", "Ground Truth X (m)", "float", "", "", "0.0", true),
        navField("gt_y", "Ground Truth Y (m)", "float", "", "", "0.0", true),
        navField("gt_yaw_deg", "Ground Truth Yaw (deg)", "float", "", "", "0.0", true)});

navAdd("4.3", "4.3 Pengujian dan Tuning AMCL", "4.3.5",
       "4.3.5 Pengujian Relocalization",
       "Pengujian relocalization AMCL",
       {"Run", "Gangguan/Pose Baru", "Waktu Recovery (s)", "Error Posisi Akhir (cm)",
        "Error Yaw Akhir (deg)", "Var Posisi Akhir", "Success", "Catatan"},
       {}, amclSeries, gtPose());

navAdd("4.3", "4.3 Pengujian dan Tuning AMCL", "4.3.6",
       "4.3.6 Pengujian Localization Dinamis",
       "Pengujian localization dinamis",
       {"Run", "Skenario", "RMSE Posisi (cm)", "RMSE Yaw (deg)",
        "Var X Maks", "Var Y Maks", "Var Yaw Maks", "Lonjakan Pose", "Status"},
       {"Stabilitas pose AMCL selama gerak"},
       amclSeries, gtPose());

navAdd("4.3", "4.3 Pengujian dan Tuning AMCL", "4.3.7",
       "4.3.7 Validasi AMCL Final",
       "Validasi AMCL final",
       {"Metrik", "Hasil", "Kriteria", "Keputusan"},
       {}, amclSeries, gtPose());

/* ========================================================================== */
/* 4.4 Pengujian dan Tuning Smac Hybrid-A*                                  */
/* ========================================================================== */
navAdd("4.4", "4.4 Pengujian dan Tuning Smac Hybrid-A*", "4.4.1",
       "4.4.1 Validasi Input Global Costmap",
       "Validasi input global costmap",
       {"Item", "Nilai Aktual", "Kriteria", "Status"},
       {}, plannerSeries,
       {navField("costmap_resolution", "Global Costmap Resolution (m)", "yaml_readonly",
                 "nav2", "global_costmap.global_costmap.ros__parameters.resolution"),
        navField("footprint", "Footprint", "yaml_readonly",
                 "nav2", "global_costmap.global_costmap.ros__parameters.footprint"),
        navField("inflation_radius", "Inflation Radius (m)", "yaml_readonly",
                 "nav2", "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius"),
        navField("motion_model", "Motion Model", "yaml_readonly",
                 "nav2", "planner_server.ros__parameters.GridBased.motion_model_for_search")});

navAdd("4.4", "4.4 Pengujian dan Tuning Smac Hybrid-A*", "4.4.2",
       "4.4.2 Tuning minimum_turning_radius",
       "Tuning minimum_turning_radius",
       {"minimum_turning_radius (m)", "Planning Time (ms)", "Path Length (m)",
        "Min Clearance (m)", "Tracking Error/CTE (m)", "Feasible Fisik", "Status"},
       {"Pengaruh minimum_turning_radius terhadap jalur"},
       plannerSeries,
       {navField("minimum_turning_radius", "minimum_turning_radius (m)", "float", "nav2",
                 "planner_server.ros__parameters.GridBased.minimum_turning_radius", "2.0")});

navAdd("4.4", "4.4 Pengujian dan Tuning Smac Hybrid-A*", "4.4.3",
       "4.4.3 Pengujian Beberapa Start-Goal",
       "Tabel 4.6 Kinerja SmacPlannerHybrid pada beberapa start-goal",
       {"Skenario", "Start", "Goal", "Path Length (m)", "Planning Time (ms)",
        "Reverse/Cusp", "Min Clearance (m)", "Status"},
       {"Planning time dan panjang jalur beberapa start-goal"},
       plannerSeries, {});

navAdd("4.4", "4.4 Pengujian dan Tuning Smac Hybrid-A*", "4.4.4",
       "4.4.4 Repeatability Planner",
       "Repeatability planner",
       {"Run", "Skenario", "Path Length (m)", "Planning Time (ms)",
        "Heading Variation (rad)", "Min Clearance (m)", "Status"},
       {}, plannerSeries, {});

navAdd("4.4", "4.4 Pengujian dan Tuning Smac Hybrid-A*", "4.4.5",
       "4.4.5 Analisis Feasibility Jalur terhadap Kinematika AGV",
       "Analisis feasibility jalur terhadap kinematika AGV",
       {"Skenario", "Radius Minimum Path (m)", "Radius Fisik AGV (m)",
        "Reverse/Cusp", "Min Clearance (m)", "Steering Saturation",
        "Feasible", "Catatan"},
       {}, plannerSeries,
       {navField("minimum_turning_radius", "Radius Belok Fisik AGV (m)", "float", "nav2",
                 "planner_server.ros__parameters.GridBased.minimum_turning_radius", "2.0")});

navAdd("4.4", "4.4 Pengujian dan Tuning Smac Hybrid-A*", "4.4.6",
       "4.4.6 Validasi Hybrid-A* Final",
       "Validasi Hybrid-A* final",
       {"Skenario", "Planning Time (ms)", "Path Length (m)", "Min Clearance (m)",
        "Tracking Error/CTE (m)", "Collision Free", "Kinematik Valid", "Success"},
       {}, plannerSeries, {});

/* ========================================================================== */
/* 4.5 Validasi Sistem Navigasi Keseluruhan                                  */
/* ========================================================================== */
auto endToEndFields = [&]() {
  return QVector<ExperimentParameterField>{
      navField("gt_x", "Goal Ground Truth X (m)", "float", "", "", "0.0", true),
      navField("gt_y", "Goal Ground Truth Y (m)", "float", "", "", "0.0", true),
      navField("gt_yaw_deg", "Goal Ground Truth Yaw (deg)", "float", "", "", "0.0", true)};
};

auto endColumns = std::initializer_list<const char *>{
    "Run", "Waktu (s)", "Path Length (m)", "Error Posisi Akhir (m)",
    "Error Yaw Akhir (deg)", "CTE RMSE (m)", "Recovery", "Collision Monitor Event", "Status"};

navAdd("4.5", "4.5 Validasi Sistem Navigasi Keseluruhan", "4.5.1",
       "4.5.1 Lintasan Lurus", "Validasi lintasan lurus",
       endColumns, {"Trajectory aktual terhadap global path — lintasan lurus"},
       navEndSeries, endToEndFields());

navAdd("4.5", "4.5 Validasi Sistem Navigasi Keseluruhan", "4.5.2",
       "4.5.2 Belok 90°", "Validasi belok 90 derajat",
       endColumns, {"Trajectory aktual terhadap global path — belok 90°"},
       navEndSeries, endToEndFields());

navAdd("4.5", "4.5 Validasi Sistem Navigasi Keseluruhan", "4.5.3",
       "4.5.3 Koridor", "Validasi koridor",
       endColumns, {"Trajectory aktual terhadap global path — koridor"},
       navEndSeries, endToEndFields());

navAdd("4.5", "4.5 Validasi Sistem Navigasi Keseluruhan", "4.5.4",
       "4.5.4 U-Turn", "Validasi U-Turn",
       endColumns, {"Trajectory aktual terhadap global path — U-Turn"},
       navEndSeries, endToEndFields());

navAdd("4.5", "4.5 Validasi Sistem Navigasi Keseluruhan", "4.5.5",
       "4.5.5 Skenario Kompleks", "Validasi skenario kompleks",
       endColumns, {"Trajectory aktual terhadap global path — skenario kompleks"},
       navEndSeries, endToEndFields());

navAdd("4.5", "4.5 Validasi Sistem Navigasi Keseluruhan", "4.5.6",
       "4.5.6 Rekapitulasi Performa Sistem",
       "Tabel 4.7 Rekapitulasi performa navigasi end-to-end",
       {"Skenario", "Percobaan", "Berhasil", "Success Rate (%)",
        "Waktu Rata-rata (s)", "Error Akhir Rata-rata (m)",
        "Error Yaw Rata-rata (deg)", "Catatan"},
       {"Gambar 4.3 Error posisi akhir pada lima skenario navigasi"},
       navEndSeries, {});
