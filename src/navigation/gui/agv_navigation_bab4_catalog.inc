// Katalog NAVIGATION GUI — disusun dari BAB IV TA_Ernanta_Revisi_Final.docx.
// Struktur leaf mengikuti dokumen terbaru secara langsung:
// 4.1 raw sensor -> 4.2 SLAM -> 4.3 AMCL -> 4.4 navigasi global AGV.
// Validasi MPPI/end-to-end digabung ke 4.4.3-4.4.4 sesuai struktur laporan; tidak ada subbab navigasi 4.5.
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
  // Every navigation experiment must expose a graph backed by its actual live
  // telemetry. Existing report-specific captions are preserved; only sections
  // that previously had no graph receive this generic actual-data caption.
  if (spec.graphCaptions.isEmpty() && !spec.liveSeries.isEmpty())
    spec.graphCaptions << QStringLiteral("Grafik data aktual — ") + spec.section;
  spec.parameterFields = commonNavFields();
  for (const auto &field : extra) spec.parameterFields << field;
  out << spec;
};

// BAB 4.1 must use raw ROS-level measurements only.  Processed navigation
// channels (lidar.*, imu.*) remain untouched for SLAM/AMCL/Nav2 sections.
const std::initializer_list<std::pair<const char *, const char *>> communicationSeries = {
    {"LiDAR rate", "lidar_raw.rate_hz"}, {"IMU rate", "imu_raw.rate_hz"},
    {"Odom rate", "odom.rate_hz"}, {"LiDAR jitter", "lidar_raw.timestamp_jitter_ms"},
    {"IMU jitter", "imu_raw.timestamp_jitter_ms"}, {"Odom jitter", "odom.timestamp_jitter_ms"}};
const std::initializer_list<std::pair<const char *, const char *>> lidarSeries = {
    {"Range", "lidar_raw.range_center_m"}, {"Valid ratio", "lidar_raw.valid_ratio_pct"},
    {"Dropout", "lidar_raw.dropout_pct"}, {"Scan rate", "lidar_raw.rate_hz"},
    {"Jitter", "lidar_raw.timestamp_jitter_ms"}};
const std::initializer_list<std::pair<const char *, const char *>> imuSeries = {
    // BAB 4.1.3 penelitian memakai IMU sebagai yaw-only. Quaternion tetap
    // direkam di raw CSV oleh commonPaths() sebagai sumber audit Persamaan (2.8),
    // tetapi grafik/tabel utama hanya menggunakan yaw.
    {"Yaw", "imu_raw.yaw_rad"}};
const std::initializer_list<std::pair<const char *, const char *>> odomSeries = {
    {"vx", "odom.v"}};
const std::initializer_list<std::pair<const char *, const char *>> slamSeries = {
    {"Map update", "slam.map_update_hz"}, {"Sel Belum Terobservasi", "slam.unknown_pct"},
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
const auto gtDistance = [&]() {
  return QVector<ExperimentParameterField>{
      navField("gt_distance_m", "Jarak Referensi (m)", "float", "", "", "2.0", true)};
};
const auto gtYaw = [&]() {
  return QVector<ExperimentParameterField>{
      navField("gt_yaw_deg", "Sudut Referensi Yaw (deg)", "float", "", "", "0.0", true)};
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
       "Tabel 4.3 Pengujian akurasi yaw IMU",
       {"Pengujian", "Sudut Referensi (deg)", "Yaw IMU (deg)",
        "Error Yaw (deg)", "MAE Yaw (deg)", "RMSE Yaw (deg)", "Status"},
       {"Perbandingan yaw IMU terhadap sudut referensi",
        "Error yaw terhadap sudut referensi",
        "MAE dan RMSE yaw terhadap sudut referensi"},
       imuSeries, gtYaw());

navAdd("4.1", "4.1 Pengujian Komunikasi dan Data Raw Sensor", "4.1.4",
       "4.1.4 Pengujian Data Raw Odometri",
       "Data raw odometri",
       {"Sampel", "Timestamp", "vx", "Frame", "Catatan"},
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
  // TF validation keeps its structural table and additionally plots the live
  // localization pose produced through the map->odom->base chain. No dummy TF
  // samples are introduced.
  spec.graphCaptions << QStringLiteral("Grafik konsistensi pose hasil rantai TF terhadap waktu");
  spec.liveSeries[QStringLiteral("Pose X")] = QStringLiteral("amcl.x");
  spec.liveSeries[QStringLiteral("Pose Y")] = QStringLiteral("amcl.y");
  spec.liveSeries[QStringLiteral("Pose Yaw")] = QStringLiteral("amcl.yaw");
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
       {"Map", "Resolution (m/cell)", "Width (m)", "Height (m)", "Sel Belum Terobservasi (%)",
        "Occupied (%)", "Map Update (Hz)", "CPU Mean (%)", "RAM Mean (GB)", "Catatan"},
       {}, slamSeries, slamMapFields("Map 1"));

navAdd("4.2", "4.2 Pengujian SLAM", "4.2.2",
       "4.2.2 Hasil Mapping 2",
       "Hasil Mapping 2",
       {"Map", "Resolution (m/cell)", "Width (m)", "Height (m)", "Sel Belum Terobservasi (%)",
        "Occupied (%)", "Map Update (Hz)", "CPU Mean (%)", "RAM Mean (GB)", "Catatan"},
       {}, slamSeries, slamMapFields("Map 2"));

navAdd("4.2", "4.2 Pengujian SLAM", "4.2.3",
       "4.2.3 Hasil Mapping 3",
       "Hasil Mapping 3",
       {"Map", "Resolution (m/cell)", "Width (m)", "Height (m)", "Sel Belum Terobservasi (%)",
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
       {}, {{"Map structure error", "derived.map_structure_error_m"},
            {"Sel Belum Terobservasi", "slam.unknown_pct"}, {"Occupied", "slam.occupied_pct"}},
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
       "4.3.6 Pengujian Lokalisasi Dinamis",
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
/* 4.4 Pengujian dan Analisis Navigasi Global AGV                            */
/* ========================================================================== */
navAdd("4.4", "4.4 Pengujian dan Analisis Navigasi Global AGV", "4.4.1",
       "4.4.1 Validasi Global Costmap dan Hybrid A*",
       "Tabel 4.6 Validasi global costmap dan Hybrid A*",
       {"Skenario", "Global Costmap", "Motion Model", "Path Terbentuk",
        "Path Length (m)", "Planning Time (ms)", "Collision Free", "Status"},
       {"Gambar 4.3 Planning time dan panjang jalur hasil Hybrid A*"},
       plannerSeries,
       {navField("costmap_resolution", "Global Costmap Resolution (m)", "yaml_readonly",
                 "nav2", "global_costmap.global_costmap.ros__parameters.resolution"),
        navField("footprint", "Footprint", "yaml_readonly",
                 "nav2", "global_costmap.global_costmap.ros__parameters.footprint"),
        navField("inflation_radius", "Inflation Radius (m)", "yaml_readonly",
                 "nav2", "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius"),
        navField("motion_model", "Motion Model", "yaml_readonly",
                 "nav2", "planner_server.ros__parameters.GridBased.motion_model_for_search"),
        navField("minimum_turning_radius", "Minimum Turning Radius (m)", "yaml_readonly",
                 "nav2", "planner_server.ros__parameters.GridBased.minimum_turning_radius")});

navAdd("4.4", "4.4 Pengujian dan Analisis Navigasi Global AGV", "4.4.2",
       "4.4.2 Tuning dan Pengujian Smac Planner Hybrid",
       "Tabel 4.7 Tuning SmacPlannerHybrid",
       {"Run", "minimum_turning_radius (m)", "Tolerance (m)", "Max Planning Time (s)",
        "Angle Bins", "Reverse Penalty", "Cost Penalty", "Path Length (m)",
        "Planning Time (ms)", "CTE RMSE (m)", "Status"},
       {"Gambar 4.4 Pengaruh tuning SmacPlannerHybrid terhadap performa jalur"},
       plannerSeries,
       {navField("minimum_turning_radius", "minimum_turning_radius (m)", "float", "nav2",
                 "planner_server.ros__parameters.GridBased.minimum_turning_radius", "2.0"),
        navField("tolerance", "Planner tolerance (m)", "float", "nav2",
                 "planner_server.ros__parameters.GridBased.tolerance"),
        navField("max_planning_time", "Max planning time (s)", "float", "nav2",
                 "planner_server.ros__parameters.GridBased.max_planning_time"),
        navField("angle_quantization_bins", "Angle quantization bins", "int", "nav2",
                 "planner_server.ros__parameters.GridBased.angle_quantization_bins"),
        navField("reverse_penalty", "Reverse penalty", "float", "nav2",
                 "planner_server.ros__parameters.GridBased.reverse_penalty"),
        navField("non_straight_penalty", "Non-straight penalty", "float", "nav2",
                 "planner_server.ros__parameters.GridBased.non_straight_penalty"),
        navField("cost_penalty", "Cost penalty", "float", "nav2",
                 "planner_server.ros__parameters.GridBased.cost_penalty"),
        navField("analytic_expansion_ratio", "Analytic expansion ratio", "float", "nav2",
                 "planner_server.ros__parameters.GridBased.analytic_expansion_ratio"),
        navField("analytic_expansion_max_length", "Analytic expansion max length (m)", "float", "nav2",
                 "planner_server.ros__parameters.GridBased.analytic_expansion_max_length"),
        navField("max_iterations", "Maximum iterations", "int", "nav2",
                 "planner_server.ros__parameters.GridBased.max_iterations"),
        navField("retrospective_penalty", "Retrospective penalty", "float", "nav2",
                 "planner_server.ros__parameters.GridBased.retrospective_penalty")});

navAdd("4.4", "4.4 Pengujian dan Analisis Navigasi Global AGV", "4.4.3",
       "4.4.3 Pengujian Navigasi pada Variasi Lintasan",
       "Tabel 4.8 Pengujian navigasi pada variasi lintasan",
       {"Skenario", "Run", "Goal Status", "Waktu Navigasi (s)", "Path Length (m)",
        "CTE RMSE (m)", "Endpoint Error (m)", "Goal Yaw Error (deg)",
        "Collision Monitor Event", "Status"},
       {"Gambar 4.5 Trajectory aktual terhadap global path",
        "Gambar 4.6 Perbandingan performa lima variasi lintasan"},
       {{"Planning time", "nav_path.planning_latency_ms"},
        {"Path length", "nav_path.length_m"}, {"CTE", "derived.cte_m"},
        {"Endpoint error", "derived.endpoint_error_m"},
        {"Goal yaw error", "derived.goal_yaw_error_rad"},
        {"Navigation time", "goal_state.duration_s"},
        {"Success rate", "derived.nav_success_pct"}},
       {navField("gt_x", "Goal Ground Truth X (m)", "float", "", "", "0.0", true),
        navField("gt_y", "Goal Ground Truth Y (m)", "float", "", "", "0.0", true),
        navField("gt_yaw_deg", "Goal Ground Truth Yaw (deg)", "float", "", "", "0.0", true),
        navField("mppi_vx_max", "MPPI vx max (m/s)", "yaml_readonly", "nav2",
                 "controller_server.ros__parameters.FollowPath.vx_max"),
        navField("mppi_wz_max", "MPPI wz max (rad/s)", "yaml_readonly", "nav2",
                 "controller_server.ros__parameters.FollowPath.wz_max"),
        navField("mppi_turning_radius", "MPPI Ackermann min turning radius (m)", "yaml_readonly", "nav2",
                 "controller_server.ros__parameters.FollowPath.AckermannConstraints.min_turning_r")});

navAdd("4.4", "4.4 Pengujian dan Analisis Navigasi Global AGV", "4.4.4",
       "4.4.4 Rekapitulasi Performa Navigasi",
       "Tabel 4.9 Rekapitulasi performa navigasi",
       {"Skenario", "Percobaan", "Berhasil", "Success Rate (%)", "Waktu Rata-rata (s)",
        "Path Rata-rata (m)", "CTE RMSE (m)", "Error Akhir Rata-rata (m)",
        "Error Yaw Rata-rata (deg)", "Catatan"},
       {"Gambar 4.7 Success rate dan waktu navigasi tiap skenario",
        "Gambar 4.8 Error akhir navigasi tiap skenario"},
       {{"Navigation time", "goal_state.duration_s"},
        {"Path length", "nav_path.length_m"}, {"CTE", "derived.cte_m"},
        {"Endpoint error", "derived.endpoint_error_m"},
        {"Goal yaw error", "derived.goal_yaw_error_rad"},
        {"Success rate", "derived.nav_success_pct"}}, {});
