#pragma once
#include <QString>
#include <QStringList>
#include <QVector>
struct SettingSpec {
  QString group,label,fileKey,path,kind;
  double min=-1e9,max=1e9,step=0.01;
  int decimals=4;
  QStringList choices;
  QString suffix,tip;
};
struct TabDef {
  QString key,icon,name,subtitle,pageKind;
  QVector<SettingSpec> specs;
};
inline QVector<TabDef> buildTabs() {
  QVector<TabDef> tabs;
  {
    TabDef t;
    t.key=QStringLiteral("connection");
    t.icon=QStringLiteral("⛓");
    t.name=QStringLiteral("Koneksi");
    t.subtitle=QStringLiteral("Serial, kamera, kesiapan ROS");
    t.pageKind=QStringLiteral("connection");
    t.specs.push_back(SettingSpec{
      QStringLiteral("GNSS"),QStringLiteral("Port GNSS"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.port"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("GNSS"),QStringLiteral("Receiver rate"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.navigation_rate_hz"),QStringLiteral("float"),1.0,25.0,1.0,1,QStringList{},QStringLiteral(" Hz"),QStringLiteral("UBX NAV-PVT measurement rate; 10 Hz recommended for AGV.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("GNSS"),QStringLiteral("Dynamic model"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.dynamic_model"),QStringLiteral("choice"),0.0,0.0,0.0,0,QStringList{QStringLiteral("automotive"),QStringLiteral("portable"),QStringLiteral("stationary"),QStringLiteral("pedestrian"),QStringLiteral("bike")},QStringLiteral(""),QStringLiteral("u-blox M9 platform model; automotive is the default for this AGV.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("GNSS"),QStringLiteral("Baudrate GNSS"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.baudrate"),QStringLiteral("int"),1200.0,921600.0,1200.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("IMU"),QStringLiteral("Port IMU"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.port"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("IMU"),QStringLiteral("Baudrate IMU"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.baudrate"),QStringLiteral("int"),1200.0,921600.0,1200.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("ESC"),QStringLiteral("Aktifkan serial ESC"),QStringLiteral("esc"),QStringLiteral("esc_ackermann.ros__parameters.serial_enabled"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("OFF = node ROS tetap hidup, UART tidak dibuka, /esc/ready tetap false.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("ESC"),QStringLiteral("Serial ESC"),QStringLiteral("esc"),QStringLiteral("esc_ackermann.ros__parameters.serial_device"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("ESC"),QStringLiteral("Baud ESC"),QStringLiteral("esc"),QStringLiteral("esc_ackermann.ros__parameters.serial_baud"),QStringLiteral("int"),9600.0,2000000.0,9600.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera"),QStringLiteral("RGB device"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.rgb_device"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera / Backend"),QStringLiteral("Perception mode"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.perception_mode"),QStringLiteral("choice"),-1000000000.0,1000000000.0,0.01,4,QStringList{
        QStringLiteral("off"),QStringLiteral("cpu"),QStringLiteral("gpu")
      },QStringLiteral(""),QStringLiteral("off = kamera saja tanpa model; cpu = YOLOPv2 .pt langsung via LibTorch; gpu = TensorRT engine. Berlaku pada launch berikutnya.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera / Backend"),QStringLiteral("TensorRT engine (.engine)"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.engine_path"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("Dipakai saat launch perception_mode:=gpu; berlaku pada launch berikutnya.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera / Backend"),QStringLiteral("YOLOPv2 checkpoint (.pt)"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.pt_model_path"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("Dipakai saat perception_mode:=cpu; dibaca langsung oleh LibTorch tanpa ONNX. Gunakan auto untuk mencari YOLOPV2_PT_PATH, <workspace>/models/yolopv2.pt, ~/ros/models/yolopv2.pt, atau ~/models/yolopv2.pt.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera / Backend"),QStringLiteral("CPU inference FPS"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.cpu_inference_fps"),QStringLiteral("float"),0.5,30.0,0.5,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("Berlaku saat node perception CPU dijalankan ulang.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera / Backend"),QStringLiteral("CPU threads"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.cpu_threads"),QStringLiteral("int"),0.0,32.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("0 = auto; node menyisakan kapasitas CPU untuk ROS2, Nav2, GUI, GNSS/IMU, dan ESC. Berlaku setelah perception CPU dijalankan ulang.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera"),QStringLiteral("FPS"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.fps"),QStringLiteral("int"),1.0,120.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("GUI / YAML"),QStringLiteral("Live apply parameter setelah autosave"),QStringLiteral("gui"),QStringLiteral("runtime.live_apply_yaml"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("YAML tetap sumber utama. Parameter node yang mendukung runtime update akan langsung disinkronkan.")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("overview");
    t.icon=QStringLiteral("◉");
    t.name=QStringLiteral("Ringkasan Sistem");
    t.subtitle=QStringLiteral("Ringkasan semua subsistem");
    t.pageKind=QStringLiteral("overview");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Logging"),QStringLiteral("Folder laporan"),QStringLiteral("gui"),QStringLiteral("reporting.output_directory"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Logging"),QStringLiteral("Sample rate CSV"),QStringLiteral("gui"),QStringLiteral("reporting.sample_rate_hz"),QStringLiteral("float"),0.5,200.0,0.5,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    tabs.push_back(t);
  }
  {
    // System Overview / Vehicle System Health (read-only monitoring page).
    // No tuning fields — the whole right pane is the health dashboard.
    TabDef t;
    t.key=QStringLiteral("system_overview");
    t.icon=QStringLiteral("◎");
    t.name=QStringLiteral("System Overview");
    t.subtitle=QStringLiteral("Vehicle system health & subsystem monitoring");
    t.pageKind=QStringLiteral("system_overview");
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("chapter_nav");
    t.icon=QStringLiteral("N");
    t.name=QStringLiteral("Akuisisi BAB IV — Navigasi");
    t.subtitle=QStringLiteral("Tabel, raw samples, dan grafik sesuai laporan navigasi");
    t.pageKind=QStringLiteral("experiment_navigation");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Akuisisi"),QStringLiteral("Sample rate"),QStringLiteral("gui"),QStringLiteral("reporting.sample_rate_hz"),QStringLiteral("float"),0.5,50.0,0.5,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("Frekuensi rekam raw sample GUI; bukan frekuensi node ROS.")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("chapter_perception");
    t.icon=QStringLiteral("P");
    t.name=QStringLiteral("Akuisisi BAB IV — Persepsi");
    t.subtitle=QStringLiteral("Deteksi, homography, lane, safety, dan performa");
    t.pageKind=QStringLiteral("experiment_perception");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Akuisisi"),QStringLiteral("Sample rate"),QStringLiteral("gui"),QStringLiteral("reporting.sample_rate_hz"),QStringLiteral("float"),0.5,50.0,0.5,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("Frekuensi rekam raw sample GUI; tidak mengubah FPS inference.")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("chapter_steering");
    t.icon=QStringLiteral("S");
    t.name=QStringLiteral("Akuisisi BAB IV — ESC / FOC");
    t.subtitle=QStringLiteral("Encoder, respons posisi, tracking, dan telemetry FOC bila tersedia");
    t.pageKind=QStringLiteral("experiment_steering");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Akuisisi"),QStringLiteral("Sample rate"),QStringLiteral("gui"),QStringLiteral("reporting.sample_rate_hz"),QStringLiteral("float"),0.5,50.0,0.5,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("Telemetry FOC internal hanya dapat direkam bila firmware/source mempublish /esc/foc/telemetry.")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("foc");
    t.icon=QStringLiteral("⚙");
    t.name=QStringLiteral("FOC / Steering BAB IV");
    t.subtitle=QStringLiteral("Kalibrasi, tuning PI, respons step, tracking, beban, dan fault");
    t.pageKind=QStringLiteral("foc");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Sesi 4.1"),QStringLiteral("Revision ID"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.session.revision_id"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Sesi 4.1"),QStringLiteral("Firmware version"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.session.firmware_version"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Sesi 4.1"),QStringLiteral("Parameter CRC"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.session.parameter_crc"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Sesi 4.1"),QStringLiteral("Operator"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.session.operator"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Sesi 4.1"),QStringLiteral("Kondisi / catatan"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.session.condition"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Runtime"),QStringLiteral("PWM frequency"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.runtime.pwm_frequency_hz"),QStringLiteral("int"),1000.0,50000.0,100.0,0,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Runtime"),QStringLiteral("Telemetry rate"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.runtime.telemetry_rate_hz"),QStringLiteral("int"),1.0,1000.0,10.0,0,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Runtime"),QStringLiteral("FOC telemetry topic"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.runtime.telemetry_topic"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("GUI membaca JSON/KV dari topic ini; source ESC saat ini belum mengirim arus d/q secara native.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Runtime"),QStringLiteral("Command timeout"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.runtime.command_timeout_ms"),QStringLiteral("int"),10.0,5000.0,10.0,0,QStringList{
      },QStringLiteral(" ms"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Runtime"),QStringLiteral("Burst current enabled"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.runtime.current_burst_enabled"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Runtime"),QStringLiteral("Burst current rate"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.runtime.current_burst_rate_hz"),QStringLiteral("int"),100.0,50000.0,100.0,0,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.1 Arus & Vbus"),QStringLiteral("Calibration samples"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_vbus_calibration.sample_count"),QStringLiteral("int"),64.0,65536.0,64.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.1 Arus & Vbus"),QStringLiteral("Ia offset ADC"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_vbus_calibration.ia_offset_adc"),QStringLiteral("float"),-10000.0,10000.0,0.1,3,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.1 Arus & Vbus"),QStringLiteral("Ib offset ADC"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_vbus_calibration.ib_offset_adc"),QStringLiteral("float"),-10000.0,10000.0,0.1,3,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.1 Arus & Vbus"),QStringLiteral("Ia noise std"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_vbus_calibration.ia_noise_std_adc"),QStringLiteral("float"),0.0,1000.0,0.1,3,QStringList{
      },QStringLiteral(" count"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.1 Arus & Vbus"),QStringLiteral("Ib noise std"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_vbus_calibration.ib_noise_std_adc"),QStringLiteral("float"),0.0,1000.0,0.1,3,QStringList{
      },QStringLiteral(" count"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.1 Arus & Vbus"),QStringLiteral("Current gain"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_vbus_calibration.current_gain_a_per_count"),QStringLiteral("float"),-10.0,10.0,0.0001,6,QStringList{
      },QStringLiteral(" A/count"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.1 Arus & Vbus"),QStringLiteral("Ia polarity"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_vbus_calibration.ia_polarity"),QStringLiteral("int"),-1.0,1.0,2.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.1 Arus & Vbus"),QStringLiteral("Ib polarity"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_vbus_calibration.ib_polarity"),QStringLiteral("int"),-1.0,1.0,2.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.1 Arus & Vbus"),QStringLiteral("Vbus scale"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_vbus_calibration.vbus_scale_v_per_count"),QStringLiteral("float"),0.0,10.0,0.0001,6,QStringList{
      },QStringLiteral(" V/count"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.2 Encoder"),QStringLiteral("PPR"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.encoder_mechanical.ppr"),QStringLiteral("int"),1.0,1000000.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.2 Encoder"),QStringLiteral("CPR"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.encoder_mechanical.cpr"),QStringLiteral("int"),1.0,4000000.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.2 Encoder"),QStringLiteral("Direction"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.encoder_mechanical.direction"),QStringLiteral("choice"),-1000000000.0,1000000000.0,0.01,4,QStringList{
        QStringLiteral("UNVERIFIED"),QStringLiteral("CW"),QStringLiteral("CCW")
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.2 Encoder"),QStringLiteral("Center count C0"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.encoder_mechanical.center_count"),QStringLiteral("int"),-2147483647.0,2147483647.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.2 Encoder"),QStringLiteral("Mechanical ratio G"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.encoder_mechanical.gear_ratio"),QStringLiteral("float"),0.0001,10000.0,0.001,5,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.2 Encoder"),QStringLiteral("Hard limit left"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.encoder_mechanical.hard_limit_left_deg"),QStringLiteral("float"),-180.0,0.0,0.5,2,QStringList{
      },QStringLiteral("°"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.2 Encoder"),QStringLiteral("Hard limit right"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.encoder_mechanical.hard_limit_right_deg"),QStringLiteral("float"),0.0,180.0,0.5,2,QStringList{
      },QStringLiteral("°"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.2 Encoder"),QStringLiteral("Soft limit left"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.encoder_mechanical.soft_limit_left_deg"),QStringLiteral("float"),-180.0,0.0,0.5,2,QStringList{
      },QStringLiteral("°"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.2 Encoder"),QStringLiteral("Soft limit right"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.encoder_mechanical.soft_limit_right_deg"),QStringLiteral("float"),0.0,180.0,0.5,2,QStringList{
      },QStringLiteral("°"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.2 Encoder"),QStringLiteral("Invalid transition limit"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.encoder_mechanical.invalid_transition_limit"),QStringLiteral("int"),0.0,1000000.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.3 Sudut Elektrik"),QStringLiteral("Phase order"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.electrical_alignment.phase_order"),QStringLiteral("choice"),-1000000000.0,1000000000.0,0.01,4,QStringList{
        QStringLiteral("UNVERIFIED"),QStringLiteral("UVW"),QStringLiteral("UWV"),QStringLiteral("VUW"),QStringLiteral("VWU"),QStringLiteral("WUV"),QStringLiteral("WVU")
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.3 Sudut Elektrik"),QStringLiteral("Pole pairs"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.electrical_alignment.pole_pairs"),QStringLiteral("int"),1.0,100.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.3 Sudut Elektrik"),QStringLiteral("Theta offset"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.electrical_alignment.theta_offset_rad"),QStringLiteral("float"),-6.2832,6.2832,0.001,6,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.3 Sudut Elektrik"),QStringLiteral("Id reference"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.electrical_alignment.id_ref_a"),QStringLiteral("float"),-50.0,50.0,0.1,3,QStringList{
      },QStringLiteral(" A"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.3 Sudut Elektrik"),QStringLiteral("Alignment Iq"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.electrical_alignment.alignment_iq_a"),QStringLiteral("float"),0.0,50.0,0.1,3,QStringList{
      },QStringLiteral(" A"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.2.3 Sudut Elektrik"),QStringLiteral("Sweep runs / direction"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.electrical_alignment.sweep_runs_each_direction"),QStringLiteral("int"),1.0,20.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.3 PI Arus"),QStringLiteral("Kp_i aktif"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_loop.kp_i"),QStringLiteral("float"),0.0,1000.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.3 PI Arus"),QStringLiteral("Ki_i aktif"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_loop.ki_i"),QStringLiteral("float"),0.0,100000.0,0.1,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.3 PI Arus"),QStringLiteral("Iq step test"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_loop.iq_step_a"),QStringLiteral("float"),0.01,100.0,0.1,3,QStringList{
      },QStringLiteral(" A"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.3 PI Arus"),QStringLiteral("Iq limit"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_loop.iq_limit_a"),QStringLiteral("float"),0.1,200.0,0.1,2,QStringList{
      },QStringLiteral(" A"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.3 PI Arus C1"),QStringLiteral("C1 Kp / Ki"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_loop.c1_kp_i"),QStringLiteral("float"),0.0,1000.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.3 PI Arus C1"),QStringLiteral("C1 Ki"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_loop.c1_ki_i"),QStringLiteral("float"),0.0,100000.0,0.1,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.3 PI Arus C2"),QStringLiteral("C2 Kp"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_loop.c2_kp_i"),QStringLiteral("float"),0.0,1000.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.3 PI Arus C2"),QStringLiteral("C2 Ki"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_loop.c2_ki_i"),QStringLiteral("float"),0.0,100000.0,0.1,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.3 PI Arus C3"),QStringLiteral("C3 Kp"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_loop.c3_kp_i"),QStringLiteral("float"),0.0,1000.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.3 PI Arus C3"),QStringLiteral("C3 Ki"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_loop.c3_ki_i"),QStringLiteral("float"),0.0,100000.0,0.1,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.3 PI Arus"),QStringLiteral("Selected set"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.current_loop.selected_set"),QStringLiteral("choice"),-1000000000.0,1000000000.0,0.01,4,QStringList{
        QStringLiteral("UNSET"),QStringLiteral("C1"),QStringLiteral("C2"),QStringLiteral("C3")
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.4 PI Posisi"),QStringLiteral("Kp_pos aktif"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.position_loop.kp_pos"),QStringLiteral("float"),0.0,1000.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.4 PI Posisi"),QStringLiteral("Ki_pos aktif"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.position_loop.ki_pos"),QStringLiteral("float"),0.0,1000.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.4 PI Posisi"),QStringLiteral("Kd_pos opsional"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.position_loop.kd_pos"),QStringLiteral("float"),0.0,1000.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.4 PI Posisi"),QStringLiteral("Iq max"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.position_loop.iq_max_a"),QStringLiteral("float"),0.1,200.0,0.1,2,QStringList{
      },QStringLiteral(" A"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.4 PI Posisi"),QStringLiteral("Iq slew rate"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.position_loop.iq_slew_rate_a_per_s"),QStringLiteral("float"),0.0,10000.0,1.0,2,QStringList{
      },QStringLiteral(" A/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.4 PI Posisi"),QStringLiteral("Tuning target"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.position_loop.tuning_target_deg"),QStringLiteral("float"),-90.0,90.0,1.0,1,QStringList{
      },QStringLiteral("°"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.4 PI Posisi P1"),QStringLiteral("P1 Kp"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.position_loop.p1_kp_pos"),QStringLiteral("float"),0.0,1000.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.4 PI Posisi P1"),QStringLiteral("P1 Ki"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.position_loop.p1_ki_pos"),QStringLiteral("float"),0.0,1000.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.4 PI Posisi P2"),QStringLiteral("P2 Kp"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.position_loop.p2_kp_pos"),QStringLiteral("float"),0.0,1000.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.4 PI Posisi P2"),QStringLiteral("P2 Ki"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.position_loop.p2_ki_pos"),QStringLiteral("float"),0.0,1000.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.4 PI Posisi P3"),QStringLiteral("P3 Kp"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.position_loop.p3_kp_pos"),QStringLiteral("float"),0.0,1000.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.4 PI Posisi P3"),QStringLiteral("P3 Ki"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.position_loop.p3_ki_pos"),QStringLiteral("float"),0.0,1000.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.4 PI Posisi"),QStringLiteral("Selected set"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.position_loop.selected_set"),QStringLiteral("choice"),-1000000000.0,1000000000.0,0.01,4,QStringList{
        QStringLiteral("UNSET"),QStringLiteral("P1"),QStringLiteral("P2"),QStringLiteral("P3")
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.5 Step"),QStringLiteral("Target step [deg]"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.step_test.targets_deg"),QStringLiteral("list"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.5 Step"),QStringLiteral("Runs per direction"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.step_test.runs_per_direction"),QStringLiteral("int"),1.0,100.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.5 Step / Ackermann"),QStringLiteral("Measured inner wheel"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.step_test.measured_inner_wheel_deg"),QStringLiteral("float"),-90.0,90.0,0.1,2,QStringList{
      },QStringLiteral("°"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.5 Step / Ackermann"),QStringLiteral("Measured outer wheel"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.step_test.measured_outer_wheel_deg"),QStringLiteral("float"),-90.0,90.0,0.1,2,QStringList{
      },QStringLiteral("°"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.7 Tracking"),QStringLiteral("Amplitude"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.tracking_test.amplitude_deg"),QStringLiteral("float"),0.1,90.0,0.5,2,QStringList{
      },QStringLiteral("°"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.7 Tracking"),QStringLiteral("Frequencies [Hz]"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.tracking_test.frequencies_hz"),QStringLiteral("list"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.8 Beban"),QStringLiteral("Target"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.load_test.target_deg"),QStringLiteral("float"),-90.0,90.0,1.0,1,QStringList{
      },QStringLiteral("°"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.8 Beban"),QStringLiteral("Level aktif"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.load_test.selected_level"),QStringLiteral("choice"),-1000000000.0,1000000000.0,0.01,4,QStringList{
        QStringLiteral("tanpa_beban"),QStringLiteral("sedang"),QStringLiteral("maksimum")
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.9 Fault"),QStringLiteral("Overcurrent"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.safety_limits.overcurrent_a"),QStringLiteral("float"),0.0,300.0,0.1,2,QStringList{
      },QStringLiteral(" A"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.9 Fault"),QStringLiteral("Undervoltage"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.safety_limits.undervoltage_v"),QStringLiteral("float"),0.0,200.0,0.1,2,QStringList{
      },QStringLiteral(" V"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.9 Fault"),QStringLiteral("Overvoltage"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.safety_limits.overvoltage_v"),QStringLiteral("float"),0.0,200.0,0.1,2,QStringList{
      },QStringLiteral(" V"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.9 Fault"),QStringLiteral("Overtemperature"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.safety_limits.overtemperature_c"),QStringLiteral("float"),0.0,200.0,1.0,1,QStringList{
      },QStringLiteral(" °C"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.9 Fault"),QStringLiteral("ISR budget"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.safety_limits.isr_budget_us"),QStringLiteral("float"),1.0,1000.0,0.1,2,QStringList{
      },QStringLiteral(" µs"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Akuisisi Bab IV"),QStringLiteral("CSV columns"),QStringLiteral("foc_thesis"),QStringLiteral("foc_thesis.acquisition.csv_columns"),QStringLiteral("list"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("Kolom minimum yang harus tersedia pada telemetry/log FOC.")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("steer_cal");
    t.icon=QStringLiteral("↔");
    t.name=QStringLiteral("Kalibrasi Steering Fisik");
    t.subtitle=QStringLiteral("Petakan protocol ESC ke sudut roda nyata KIRI / LURUS / KANAN");
    t.pageKind=QStringLiteral("steering_calibration");
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("odom");
    t.icon=QStringLiteral("↦");
    t.name=QStringLiteral("Kalibrasi ESC");
    t.subtitle=QStringLiteral("Kecepatan, jarak, encoder, dan steering");
    t.pageKind=QStringLiteral("odom");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Vehicle Authority"),QStringLiteral("Physical wheelbase"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.wheelbase_m"),QStringLiteral("float"),0.1,5.0,0.001,4,QStringList{
      },QStringLiteral(" m"),QStringLiteral("Jarak axle fisik untuk URDF/dokumentasi. Tidak dioverwrite oleh circle calibration.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Vehicle Authority"),QStringLiteral("Effective kinematic wheelbase"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.effective_wheelbase_m"),QStringLiteral("float"),0.1,5.0,0.001,4,QStringList{
      },QStringLiteral(" m"),QStringLiteral("Hasil circle-test Part 3; disinkronkan ke ESC, NavigationCore dan model kinematik perception.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Vehicle Authority"),QStringLiteral("Track width"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.track_width_m"),QStringLiteral("float"),0.1,5.0,0.001,4,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Vehicle Authority"),QStringLiteral("Total physical width"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.total_width_m"),QStringLiteral("float"),0.1,5.0,0.001,4,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Vehicle Authority"),QStringLiteral("Vehicle length"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.vehicle_length_m"),QStringLiteral("float"),0.1,10.0,0.001,4,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Vehicle Authority"),QStringLiteral("Wheel radius"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.wheel_radius_m"),QStringLiteral("float"),0.01,1.0,0.0005,4,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Vehicle Authority"),QStringLiteral("Mechanical steering common max"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.max_steering_angle_rad"),QStringLiteral("float"),0.05,1.55,0.001,5,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("Diisi otomatis dari sisi fisik terkecil hasil kalibrasi steering.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Vehicle Authority"),QStringLiteral("Operational steering limit"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.operational_steering_angle_rad"),QStringLiteral("float"),0.05,1.55,0.001,5,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("Batas simetris Teleop/Nav2; otomatis lebih kecil dari endpoint mekanik hasil ukur.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Vehicle Authority"),QStringLiteral("Minimum turning radius"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.minimum_turning_radius_m"),QStringLiteral("float"),0.1,20.0,0.001,4,QStringList{
      },QStringLiteral(" m"),QStringLiteral("Setelah Part 3 valid, berasal dari radius circle-test kiri/kanan + safety margin dan dipropagasi ke Smac/MPPI.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Certified Ceiling"),QStringLiteral("Max forward"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.max_forward_speed_mps"),QStringLiteral("float"),0.05,5.0,0.01,3,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Certified Ceiling"),QStringLiteral("Max reverse"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.max_reverse_speed_mps"),QStringLiteral("float"),0.0,5.0,0.01,3,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Certified Ceiling"),QStringLiteral("Max yaw rate"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.max_yaw_rate_rps"),QStringLiteral("float"),0.05,10.0,0.01,3,QStringList{
      },QStringLiteral(" rad/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Certified Ceiling"),QStringLiteral("Max accel"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.max_accel_mps2"),QStringLiteral("float"),0.01,10.0,0.01,3,QStringList{
      },QStringLiteral(" m/s²"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Certified Ceiling"),QStringLiteral("Max decel"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.max_decel_mps2"),QStringLiteral("float"),-10.0,-0.01,0.01,3,QStringList{
      },QStringLiteral(" m/s²"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Certified Ceiling"),QStringLiteral("Max yaw accel"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.max_yaw_accel_rps2"),QStringLiteral("float"),0.01,20.0,0.01,3,QStringList{
      },QStringLiteral(" rad/s²"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Vehicle Authority"),QStringLiteral("Nav2 footprint"),QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.footprint"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("AUTHORITATIVE footprint string, disinkronkan ke local/global costmap.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Actuator"),QStringLiteral("Invert steering"),QStringLiteral("esc"),QStringLiteral("esc_ackermann.ros__parameters.invert_steering"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Actuator"),QStringLiteral("Invert drive"),QStringLiteral("esc"),QStringLiteral("esc_ackermann.ros__parameters.invert_drive"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Actuator"),QStringLiteral("Serial TX rate"),QStringLiteral("esc"),QStringLiteral("esc_ackermann.ros__parameters.serial_tx_rate_hz"),QStringLiteral("float"),1.0,200.0,1.0,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Limits"),QStringLiteral("Speed max"),QStringLiteral("esc"),QStringLiteral("esc_ackermann.ros__parameters.speed_max"),QStringLiteral("float"),0.05,5.0,0.05,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Legacy / Protocol"),QStringLiteral("Legacy fallback steering scale"),QStringLiteral("esc"),QStringLiteral("esc_ackermann.ros__parameters.serial_left_max_deg"),QStringLiteral("float"),1.0,90.0,1.0,1,QStringList{
      },QStringLiteral("°"),QStringLiteral("Bukan sudut roda fisik. Hanya fallback sebelum kalibrasi fisik Part 1 aktif.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Limits"),QStringLiteral("Drive RPM max"),QStringLiteral("esc"),QStringLiteral("esc_ackermann.ros__parameters.serial_right_max_rpm"),QStringLiteral("float"),10.0,2000.0,10.0,1,QStringList{
      },QStringLiteral(" RPM"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Adaptive Covariance"),QStringLiteral("Base vx variance"),QStringLiteral("esc"),QStringLiteral("esc_ackermann.ros__parameters.odom_v_variance_base"),QStringLiteral("float"),1e-06,5.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Adaptive Covariance"),QStringLiteral("RPM tracking error gain"),QStringLiteral("esc"),QStringLiteral("esc_ackermann.ros__parameters.odom_v_variance_rpm_error_gain"),QStringLiteral("float"),0.0,10.0,0.01,3,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Adaptive Covariance"),QStringLiteral("Base yaw variance"),QStringLiteral("esc"),QStringLiteral("esc_ackermann.ros__parameters.odom_yaw_variance_base"),QStringLiteral("float"),1e-06,5.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Adaptive Covariance"),QStringLiteral("Steering variance gain"),QStringLiteral("esc"),QStringLiteral("esc_ackermann.ros__parameters.odom_yaw_variance_steer_gain"),QStringLiteral("float"),0.0,10.0,0.01,3,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Adaptive Covariance"),QStringLiteral("Base yaw-rate variance"),QStringLiteral("esc"),QStringLiteral("esc_ackermann.ros__parameters.odom_yaw_rate_variance_base"),QStringLiteral("float"),1e-06,5.0,0.001,6,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("map_gt");
    t.icon=QStringLiteral("⌖");
    t.name=QStringLiteral("Peta & Ground Truth");
    t.subtitle=QStringLiteral("PGM, OSM, ground truth, dan goal pose");
    t.pageKind=QStringLiteral("map");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Nav2 Map YAML"),QStringLiteral("Resolution"),QStringLiteral("map"),QStringLiteral("resolution"),QStringLiteral("float"),0.01,1.0,0.001,5,QStringList{
      },QStringLiteral(" m/px"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Nav2 Map YAML"),QStringLiteral("Origin X"),QStringLiteral("map"),QStringLiteral("origin.0"),QStringLiteral("float"),-10000000.0,10000000.0,0.1,4,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Nav2 Map YAML"),QStringLiteral("Origin Y"),QStringLiteral("map"),QStringLiteral("origin.1"),QStringLiteral("float"),-10000000.0,10000000.0,0.1,4,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Nav2 Map YAML"),QStringLiteral("Origin Yaw"),QStringLiteral("map"),QStringLiteral("origin.2"),QStringLiteral("float"),-1000000000.0,1000000000.0,0.001,5,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("OSM Overlay"),QStringLiteral("PGM opacity"),QStringLiteral("gui"),QStringLiteral("map_overlay.pgm_opacity"),QStringLiteral("float"),0.05,1.0,0.05,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("OSM Overlay"),QStringLiteral("OSM opacity"),QStringLiteral("gui"),QStringLiteral("map_overlay.osm_opacity"),QStringLiteral("float"),0.05,1.0,0.05,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("OSM Overlay"),QStringLiteral("OSM X offset"),QStringLiteral("gui"),QStringLiteral("map_overlay.offset_x_m"),QStringLiteral("float"),-100.0,100.0,0.05,3,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("OSM Overlay"),QStringLiteral("OSM Y offset"),QStringLiteral("gui"),QStringLiteral("map_overlay.offset_y_m"),QStringLiteral("float"),-100.0,100.0,0.05,3,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("OSM Overlay"),QStringLiteral("OSM yaw"),QStringLiteral("gui"),QStringLiteral("map_overlay.yaw_deg"),QStringLiteral("float"),-180.0,180.0,0.05,3,QStringList{
      },QStringLiteral("°"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("OSM Overlay"),QStringLiteral("OSM scale (diagnostic)"),QStringLiteral("gui"),QStringLiteral("map_overlay.scale"),QStringLiteral("float"),0.8,1.2,0.0001,5,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Localization datum"),QStringLiteral("Reference latitude"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.reference_latitude"),QStringLiteral("float"),-90.0,90.0,1e-07,9,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Localization datum"),QStringLiteral("Reference longitude"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.reference_longitude"),QStringLiteral("float"),-180.0,180.0,1e-07,9,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Localization datum"),QStringLiteral("Reference map X"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.reference_map_x_m"),QStringLiteral("float"),-1000.0,10000.0,0.01,4,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Localization datum"),QStringLiteral("Reference map Y"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.reference_map_y_m"),QStringLiteral("float"),-1000.0,10000.0,0.01,4,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Localization datum"),QStringLiteral("Map yaw from ENU"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.map_yaw_from_enu_rad"),QStringLiteral("float"),-1000000000.0,1000000000.0,0.001,6,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("gnss");
    t.icon=QStringLiteral("⌁");
    t.name=QStringLiteral("Akurasi GNSS");
    t.subtitle=QStringLiteral("Kualitas fix, DOP, hAcc, drift, dan motion validation");
    t.pageKind=QStringLiteral("gnss");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Quality Gate"),QStringLiteral("Minimum satellites"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.min_satellites"),QStringLiteral("int"),4.0,40.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Quality Gate"),QStringLiteral("Maximum DOP"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.max_dop"),QStringLiteral("float"),0.1,20.0,0.1,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Quality Gate"),QStringLiteral("Maximum hAcc"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.max_hacc_m"),QStringLiteral("float"),0.1,50.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Quality Gate"),QStringLiteral("Maximum sAcc"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.max_sacc_mps"),QStringLiteral("float"),0.05,10.0,0.05,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Driver V2"),QStringLiteral("Configure receiver nav rate"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.configure_navigation_rate"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Driver V2"),QStringLiteral("NAV-PVT target rate"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.navigation_rate_hz"),QStringLiteral("float"),1.0,25.0,1.0,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Driver V2"),QStringLiteral("Poll NAV-COV"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.poll_nav_cov"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Driver V2"),QStringLiteral("NAV-COV poll rate"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.nav_cov_poll_rate_hz"),QStringLiteral("float"),0.2,25.0,0.2,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Driver V2"),QStringLiteral("Poll NAV-DOP"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.poll_nav_dop"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Driver V2"),QStringLiteral("NAV-DOP poll rate"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.nav_dop_poll_rate_hz"),QStringLiteral("float"),0.1,10.0,0.1,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Driver V2"),QStringLiteral("NAV-COV wait timeout"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.cov_wait_timeout_sec"),QStringLiteral("float"),0.0,0.5,0.01,3,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Driver V2"),QStringLiteral("Timestamp mode"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.timestamp_mode"),QStringLiteral("choice"),-1000000000.0,1000000000.0,0.01,4,QStringList{
        QStringLiteral("auto"),QStringLiteral("utc"),QStringLiteral("itow"),QStringLiteral("arrival")
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Driver V2"),QStringLiteral("UTC/host max offset"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.utc_stamp_max_offset_sec"),QStringLiteral("float"),1.0,60.0,1.0,1,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Position Fit"),QStringLiteral("Regression window"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.position_fit_window_sec"),QStringLiteral("float"),0.5,30.0,0.5,1,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Position Fit"),QStringLiteral("Minimum samples"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.position_fit_min_samples"),QStringLiteral("int"),3.0,200.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Position Fit"),QStringLiteral("Minimum baseline"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.position_fit_min_baseline_m"),QStringLiteral("float"),0.1,20.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Position Fit"),QStringLiteral("hAcc baseline multiplier"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.position_fit_hacc_multiplier"),QStringLiteral("float"),0.0,10.0,0.1,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Fallback NMEA"),QStringLiteral("Enable validated fallback"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.allow_validated_nmea_fallback"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Fallback NMEA"),QStringLiteral("Fallback min satellites"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.nmea_fallback_min_satellites"),QStringLiteral("int"),4.0,40.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Fallback NMEA"),QStringLiteral("Fallback max HDOP"),QStringLiteral("gnss"),QStringLiteral("data_cuav_node.ros__parameters.nmea_fallback_max_hdop"),QStringLiteral("float"),0.1,10.0,0.1,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Localization Display"),QStringLiteral("Provisional map display"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.allow_provisional_map_display"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{},QStringLiteral(""),QStringLiteral("Menampilkan x/y + map->odom dari horizontal fix selama GNSS konvergen; tidak membuka motor autonomy.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Localization Display"),QStringLiteral("Provisional min satellites"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.provisional_min_satellites"),QStringLiteral("int"),1.0,20.0,1.0,0,QStringList{},QStringLiteral(""),QStringLiteral("Khusus tampilan/TF provisional.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Localization Display"),QStringLiteral("Provisional max DOP"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.provisional_max_dop"),QStringLiteral("float"),1.0,99.0,0.5,1,QStringList{},QStringLiteral(""),QStringLiteral("Khusus tampilan/TF provisional.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Localization Display"),QStringLiteral("Provisional max hAcc"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.provisional_max_hacc_m"),QStringLiteral("float"),5.0,500.0,5.0,1,QStringList{},QStringLiteral(" m"),QStringLiteral("Khusus tampilan/TF provisional; strict motion gate tetap jauh lebih ketat.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Localization Gate"),QStringLiteral("Anchor wajib strict"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.anchor_init_requires_strict"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("ON mewajibkan strict untuk anchor. OFF mengizinkan degraded/provisional display; motor autonomy tetap butuh STRICT.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Localization Gate"),QStringLiteral("Startup samples"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.startup_gnss_samples"),QStringLiteral("int"),2.0,100.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Localization Gate"),QStringLiteral("Startup max spread"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.startup_max_spread_m"),QStringLiteral("float"),0.1,20.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Lever Arm"),QStringLiteral("GNSS antenna X from base"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_antenna_x_m"),QStringLiteral("float"),-5.0,5.0,0.005,3,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Lever Arm"),QStringLiteral("GNSS antenna Y from base"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_antenna_y_m"),QStringLiteral("float"),-5.0,5.0,0.005,3,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Motion Sync"),QStringLiteral("Odom history"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_motion_history_sec"),QStringLiteral("float"),0.5,30.0,0.5,1,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Motion Sync"),QStringLiteral("Maximum GNSS↔odom sync gap"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_sync_max_gap_sec"),QStringLiteral("float"),0.02,2.0,0.01,2,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Motion Validation"),QStringLiteral("GNSS velocity timeout"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_velocity_timeout_sec"),QStringLiteral("float"),0.05,5.0,0.05,2,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Motion Validation"),QStringLiteral("Position-fit timeout"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_fit_timeout_sec"),QStringLiteral("float"),0.2,30.0,0.2,1,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Motion Validation"),QStringLiteral("Minimum validation speed"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_velocity_min_validation_speed_mps"),QStringLiteral("float"),0.01,3.0,0.01,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Motion Validation"),QStringLiteral("Doppler vector speed tolerance"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_speed_consistency_max_mps"),QStringLiteral("float"),0.01,3.0,0.01,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Motion Validation"),QStringLiteral("Position-fit speed residual max"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_fit_speed_residual_max_mps"),QStringLiteral("float"),0.01,3.0,0.01,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Motion Validation"),QStringLiteral("COG vs velocity course max"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_cog_velocity_course_max_rad"),QStringLiteral("float"),0.01,3.14,0.01,3,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Motion Validation"),QStringLiteral("COG vs position-fit max"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_cog_fit_course_max_rad"),QStringLiteral("float"),0.01,3.14,0.01,3,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Motion Validation"),QStringLiteral("Lateral body velocity warn"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_lateral_velocity_warn_mps"),QStringLiteral("float"),0.01,2.0,0.01,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Motion Validation"),QStringLiteral("Wheel-GNSS slip residual"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.wheel_gnss_slip_residual_mps"),QStringLiteral("float"),0.01,3.0,0.01,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("COG Qualification"),QStringLiteral("Valid hold time"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.cog_valid_hold_sec"),QStringLiteral("float"),0.0,10.0,0.1,1,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("COG Qualification"),QStringLiteral("Invalid hold time"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.cog_invalid_hold_sec"),QStringLiteral("float"),0.0,10.0,0.1,1,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("GNSS vyaw"),QStringLiteral("Minimum speed for COG derivative"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_yaw_rate_min_speed_mps"),QStringLiteral("float"),0.05,3.0,0.01,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("Di bawah nilai ini vyaw GNSS diberi covariance sangat besar.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("GNSS vyaw"),QStringLiteral("Maximum |vyaw|"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_yaw_rate_max_abs_rps"),QStringLiteral("float"),0.1,5.0,0.05,2,QStringList{
      },QStringLiteral(" rad/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("GNSS vyaw"),QStringLiteral("Low-pass alpha"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_yaw_rate_filter_alpha"),QStringLiteral("float"),0.01,1.0,0.01,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Global EKF Fusion"),QStringLiteral("Velocity min variance"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_velocity_fusion_min_variance"),QStringLiteral("float"),1e-06,10.0,0.001,6,QStringList{
      },QStringLiteral(" (m/s)^2"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Global EKF Fusion"),QStringLiteral("Velocity max variance"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_velocity_fusion_max_variance"),QStringLiteral("float"),1e-06,100.0,0.01,6,QStringList{
      },QStringLiteral(" (m/s)^2"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Global EKF Fusion"),QStringLiteral("COG min variance"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_cog_fusion_min_variance_rad2"),QStringLiteral("float"),1e-06,10.0,0.001,6,QStringList{
      },QStringLiteral(" rad²"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Global EKF Fusion"),QStringLiteral("COG max variance"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_cog_fusion_max_variance_rad2"),QStringLiteral("float"),1e-06,10.0,0.01,6,QStringList{
      },QStringLiteral(" rad²"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Global EKF Fusion"),QStringLiteral("Measurement timeout"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.gnss_fusion_measurement_timeout_sec"),QStringLiteral("float"),0.05,10.0,0.05,2,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Global EKF Yaw"),QStringLiteral("Use global EKF yaw for map correction"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.use_global_ekf_yaw_for_map_correction"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Global EKF Yaw"),QStringLiteral("Correction alpha"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.global_ekf_yaw_correction_alpha"),QStringLiteral("float"),0.0,1.0,0.005,3,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Global EKF Yaw"),QStringLiteral("Maximum yaw step"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.global_ekf_yaw_max_step_rad"),QStringLiteral("float"),1e-05,0.25,0.001,6,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Global EKF Yaw"),QStringLiteral("Maximum innovation"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.global_ekf_yaw_max_innovation_rad"),QStringLiteral("float"),0.05,3.14,0.01,3,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("COG Yaw (gated)"),QStringLiteral("Minimum forward speed"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.cog_min_forward_speed_mps"),QStringLiteral("float"),0.05,5.0,0.01,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("COG Yaw (gated)"),QStringLiteral("Maximum sAcc"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.cog_max_sacc_mps"),QStringLiteral("float"),0.01,5.0,0.01,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("COG Yaw (gated)"),QStringLiteral("Maximum heading accuracy"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.cog_max_heading_accuracy_rad"),QStringLiteral("float"),0.01,3.14,0.01,3,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("COG Yaw (gated)"),QStringLiteral("Maximum local yaw rate"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.cog_max_local_yaw_rate_rps"),QStringLiteral("float"),0.01,3.0,0.01,3,QStringList{
      },QStringLiteral(" rad/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("COG Yaw (gated)"),QStringLiteral("Maximum yaw innovation"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.cog_max_innovation_rad"),QStringLiteral("float"),0.05,3.14,0.01,3,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("COG Yaw (gated)"),QStringLiteral("Yaw correction alpha"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.cog_yaw_alpha"),QStringLiteral("float"),0.0,1.0,0.005,3,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("COG Yaw (gated)"),QStringLiteral("Maximum yaw step"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.cog_max_yaw_step_rad"),QStringLiteral("float"),1e-05,0.25,0.001,6,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Model vs IMU"),QStringLiteral("Ackermann yaw freshness"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.esc_kinematic_yaw_timeout_sec"),QStringLiteral("float"),0.05,2.0,0.05,2,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("imu");
    t.icon=QStringLiteral("⇄");
    t.name=QStringLiteral("Kalibrasi IMU");
    t.subtitle=QStringLiteral("Bias, heading, covariance");
    t.pageKind=QStringLiteral("imu");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Orientation"),QStringLiteral("Invert roll"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.invert_roll"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Orientation"),QStringLiteral("Invert pitch"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.invert_pitch"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Orientation"),QStringLiteral("Invert yaw"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.invert_yaw"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Orientation"),QStringLiteral("Yaw sign"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.yaw_sign"),QStringLiteral("float"),-1.0,1.0,2.0,1,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Orientation"),QStringLiteral("Roll offset"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.roll_offset_rad"),QStringLiteral("float"),-1000000000.0,1000000000.0,0.001,6,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Orientation"),QStringLiteral("Pitch offset"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.pitch_offset_rad"),QStringLiteral("float"),-1000000000.0,1000000000.0,0.001,6,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Orientation"),QStringLiteral("Yaw offset"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.yaw_offset_rad"),QStringLiteral("float"),-1000000000.0,1000000000.0,0.001,6,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Orientation"),QStringLiteral("Magnetic declination"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.magnetic_declination_radians"),QStringLiteral("float"),-1000000000.0,1000000000.0,0.0001,7,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Bias"),QStringLiteral("Accel bias [x,y,z]"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.accel_bias"),QStringLiteral("list"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Bias"),QStringLiteral("Gyro bias [x,y,z]"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.gyro_bias"),QStringLiteral("list"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Covariance"),QStringLiteral("Orientation variance"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.orientation_covariance"),QStringLiteral("list"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Covariance"),QStringLiteral("Angular velocity variance"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.angular_velocity_covariance"),QStringLiteral("list"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Covariance"),QStringLiteral("Linear acceleration variance"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.linear_acceleration_covariance"),QStringLiteral("list"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Freshness / EKF"),QStringLiteral("Gyro packet timeout"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.gyro_packet_timeout_sec"),QStringLiteral("float"),0.05,2.0,0.05,2,QStringList{
      },QStringLiteral(" s"),QStringLiteral("Jika gyro lebih lama dari ini, /imu/data ditahan agar EKF tidak menerima nilai gyro lama dengan timestamp baru.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Freshness / EKF"),QStringLiteral("Accel packet timeout"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.accel_packet_timeout_sec"),QStringLiteral("float"),0.05,2.0,0.05,2,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Freshness / EKF"),QStringLiteral("Require fresh gyro"),QStringLiteral("imu"),QStringLiteral("data_imu_node.ros__parameters.require_fresh_gyro_for_imu_publish"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("Tidak wajib untuk EKF baru: yaw absolut dari orientation IMU; vyaw dari GNSS.")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("ekf");
    t.icon=QStringLiteral("∑");
    t.name=QStringLiteral("Validasi EKF");
    t.subtitle=QStringLiteral("Filter lokal/global dan covariance");
    t.pageKind=QStringLiteral("ekf");
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Local"),QStringLiteral("Frequency"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_odom.ros__parameters.frequency"),QStringLiteral("float"),1.0,100.0,1.0,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Local"),QStringLiteral("Sensor timeout"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_odom.ros__parameters.sensor_timeout"),QStringLiteral("float"),0.05,5.0,0.05,2,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Local"),QStringLiteral("Predict to current time"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_odom.ros__parameters.predict_to_current_time"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Local GNSS"),QStringLiteral("GNSS vx + vyaw topic"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_odom.ros__parameters.twist0"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("twist0_config mengaktifkan vx dan vyaw.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Local GNSS"),QStringLiteral("GNSS twist rejection"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_odom.ros__parameters.twist0_rejection_threshold"),QStringLiteral("float"),0.1,100.0,0.1,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Local IMU"),QStringLiteral("IMU absolute yaw topic"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_odom.ros__parameters.imu0"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("imu0_config mengaktifkan yaw orientation saja.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Local IMU"),QStringLiteral("IMU queue size"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_odom.ros__parameters.imu0_queue_size"),QStringLiteral("int"),1.0,200.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Local"),QStringLiteral("Process noise Q(vx)"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_odom.ros__parameters.process_noise_covariance.96"),QStringLiteral("float"),0.0001,1.0,0.005,4,QStringList{
      },QStringLiteral(""),QStringLiteral("Baseline source Q(vx)=0.025; uji fisik tetap menentukan hasil final.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Local"),QStringLiteral("Process noise Q(vyaw)"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_odom.ros__parameters.process_noise_covariance.176"),QStringLiteral("float"),0.0001,1.0,0.005,4,QStringList{
      },QStringLiteral(""),QStringLiteral("Baseline source Q(vyaw)=0.020; uji fisik tetap menentukan hasil final.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Global"),QStringLiteral("Frequency"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_map.ros__parameters.frequency"),QStringLiteral("float"),1.0,100.0,1.0,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Global"),QStringLiteral("Sensor timeout"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_map.ros__parameters.sensor_timeout"),QStringLiteral("float"),0.05,10.0,0.05,2,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Global"),QStringLiteral("Predict to current time"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_map.ros__parameters.predict_to_current_time"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Global"),QStringLiteral("Pose rejection threshold"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_map.ros__parameters.odom0_pose_rejection_threshold"),QStringLiteral("float"),0.1,100.0,0.1,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Global GNSS"),QStringLiteral("GNSS vx + vyaw topic"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_map.ros__parameters.twist0"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("twist0_config mengaktifkan vx dan vyaw.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Global GNSS"),QStringLiteral("Mahalanobis rejection"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_map.ros__parameters.twist0_rejection_threshold"),QStringLiteral("float"),0.1,100.0,0.1,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Global IMU"),QStringLiteral("IMU absolute yaw topic"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_map.ros__parameters.imu0"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("Absolute yaw hanya IMU; COG absolute-yaw tidak difuse.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Global"),QStringLiteral("Process noise Q(x)"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_map.ros__parameters.process_noise_covariance.0"),QStringLiteral("float"),0.0001,1.0,0.005,4,QStringList{
      },QStringLiteral(""),QStringLiteral("Baseline source Q(x)=0.050; uji fisik tetap menentukan hasil final.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Global"),QStringLiteral("Process noise Q(y)"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_map.ros__parameters.process_noise_covariance.16"),QStringLiteral("float"),0.0001,1.0,0.005,4,QStringList{
      },QStringLiteral(""),QStringLiteral("Baseline source Q(y)=0.050; uji fisik tetap menentukan hasil final.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Global"),QStringLiteral("Process noise Q(yaw)"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_map.ros__parameters.process_noise_covariance.80"),QStringLiteral("float"),0.0001,1.0,0.005,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Global"),QStringLiteral("Process noise Q(vx)"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_map.ros__parameters.process_noise_covariance.96"),QStringLiteral("float"),0.0001,1.0,0.005,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("EKF Global"),QStringLiteral("Process noise Q(vyaw)"),QStringLiteral("ekf"),QStringLiteral("ekf_filter_node_map.ros__parameters.process_noise_covariance.176"),QStringLiteral("float"),0.0001,1.0,0.005,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("localization");
    t.icon=QStringLiteral("✣");
    t.name=QStringLiteral("Lokalisasi Multi-Point");
    t.subtitle=QStringLiteral("Anchor map↔odom dan solver ground truth");
    t.pageKind=QStringLiteral("localization");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Map Calibration"),QStringLiteral("Calibration X"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.map_calibration_x_m"),QStringLiteral("float"),-100.0,100.0,0.01,4,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Map Calibration"),QStringLiteral("Calibration Y"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.map_calibration_y_m"),QStringLiteral("float"),-100.0,100.0,0.01,4,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Map Calibration"),QStringLiteral("Calibration Yaw"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.map_calibration_yaw_rad"),QStringLiteral("float"),-1000000000.0,1000000000.0,0.001,6,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Multi-Point Solver"),QStringLiteral("Enable multi-point"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.multi_point_map_calibration"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Multi-Point Solver"),QStringLiteral("Minimum unique points"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.map_calibration_min_unique_points"),QStringLiteral("int"),2.0,30.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Multi-Point Solver"),QStringLiteral("Unique distance"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.map_calibration_unique_distance_m"),QStringLiteral("float"),0.2,20.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Multi-Point Solver"),QStringLiteral("Capture window"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.calibration_capture_window_sec"),QStringLiteral("float"),0.5,20.0,0.5,1,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Multi-Point Solver"),QStringLiteral("Minimum GNSS samples"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.calibration_capture_min_samples"),QStringLiteral("int"),1.0,100.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Geometry Quality"),QStringLiteral("Minimum baseline"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.map_calibration_min_baseline_m"),QStringLiteral("float"),0.1,100.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Geometry Quality"),QStringLiteral("Minimum 2D geometry score"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.map_calibration_min_geometry_score"),QStringLiteral("float"),0.0,1.0,0.01,3,QStringList{
      },QStringLiteral(""),QStringLiteral("0=segaris/buruk, mendekati 1=sebaran 2D baik.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Geometry Quality"),QStringLiteral("Maximum solver RMSE"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.map_calibration_max_rmse_m"),QStringLiteral("float"),0.05,20.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Motion Gate"),QStringLiteral("Strict max hAcc"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.strict_max_hacc_m"),QStringLiteral("float"),0.1,20.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Motion Gate"),QStringLiteral("Strict max DOP"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.strict_max_dop"),QStringLiteral("float"),0.1,10.0,0.1,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Correction 4.4"),QStringLiteral("Strict correction alpha"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.strict_correction_alpha"),QStringLiteral("float"),0.0,1.0,0.01,3,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Correction 4.4"),QStringLiteral("Moving correction alpha"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.strict_moving_correction_alpha"),QStringLiteral("float"),0.0,1.0,0.005,3,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Correction 4.4"),QStringLiteral("Max correction"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.strict_max_correction_m"),QStringLiteral("float"),0.01,2.0,0.01,3,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Correction 4.4"),QStringLiteral("TF publish rate"),QStringLiteral("localization"),QStringLiteral("localization_core.ros__parameters.tf_publish_rate_hz"),QStringLiteral("float"),1.0,100.0,1.0,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("navigation");
    t.icon=QStringLiteral("➤");
    t.name=QStringLiteral("Navigasi & MPPI");
    t.subtitle=QStringLiteral("Bab IV 4.5–4.9: costmap, Smac, MPPI, command & goal");
    t.pageKind=QStringLiteral("navigation");
    t.specs.push_back(SettingSpec{
      QStringLiteral("MPPI Runtime"),QStringLiteral("Visualization (tuning only)"),QStringLiteral("nav2"),QStringLiteral("controller_server.ros__parameters.FollowPath.visualize"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("OFF untuk deployment agar CPU lebih ringan.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Velocity Smoother"),QStringLiteral("Feedback mode"),QStringLiteral("nav2"),QStringLiteral("velocity_smoother.ros__parameters.feedback"),QStringLiteral("choice"),-1000000000.0,1000000000.0,0.01,4,QStringList{
        QStringLiteral("OPEN_LOOP"),QStringLiteral("CLOSED_LOOP")
      },QStringLiteral(""),QStringLiteral("Gunakan CLOSED_LOOP hanya jika /odometry/filtered rate/latency sudah tervalidasi.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Navigation Core"),QStringLiteral("Max forward speed"),QStringLiteral("navigation_core"),QStringLiteral("navigation_core.ros__parameters.max_forward_speed_mps"),QStringLiteral("float"),0.05,5.0,0.05,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Navigation Core"),QStringLiteral("Max reverse speed"),QStringLiteral("navigation_core"),QStringLiteral("navigation_core.ros__parameters.max_reverse_speed_mps"),QStringLiteral("float"),0.05,5.0,0.05,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Navigation Core"),QStringLiteral("Max yaw rate"),QStringLiteral("navigation_core"),QStringLiteral("navigation_core.ros__parameters.max_yaw_rate_rps"),QStringLiteral("float"),0.01,5.0,0.01,3,QStringList{
      },QStringLiteral(" rad/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Navigation Core"),QStringLiteral("Linear deadband"),QStringLiteral("navigation_core"),QStringLiteral("navigation_core.ros__parameters.linear_deadband_mps"),QStringLiteral("float"),0.0,1.0,0.01,3,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Calibration Gate"),QStringLiteral("Require camera metric calibration"),QStringLiteral("navigation_core"),QStringLiteral("navigation_core.ros__parameters.require_camera_metric_calibration"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Calibration Gate"),QStringLiteral("Camera metric validated"),QStringLiteral("navigation_core"),QStringLiteral("navigation_core.ros__parameters.camera_metric_calibration_validated"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("Aktifkan hanya setelah pengujian 4.3 lulus.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Global Costmap 4.5"),QStringLiteral("Footprint padding"),QStringLiteral("nav2"),QStringLiteral("global_costmap.global_costmap.ros__parameters.footprint_padding"),QStringLiteral("float"),0.0,0.3,0.01,3,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Global Costmap 4.5"),QStringLiteral("Inflation radius"),QStringLiteral("nav2"),QStringLiteral("global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius"),QStringLiteral("float"),0.1,3.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Global Costmap 4.5"),QStringLiteral("Cost scaling factor"),QStringLiteral("nav2"),QStringLiteral("global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor"),QStringLiteral("float"),0.1,20.0,0.1,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Smac Hybrid-A* 4.6"),QStringLiteral("Downsample costmap"),QStringLiteral("nav2"),QStringLiteral("planner_server.ros__parameters.GridBased.downsample_costmap"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Smac Hybrid-A* 4.6"),QStringLiteral("Downsampling factor"),QStringLiteral("nav2"),QStringLiteral("planner_server.ros__parameters.GridBased.downsampling_factor"),QStringLiteral("int"),1.0,8.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Smac Hybrid-A* 4.6"),QStringLiteral("Angle quantization bins"),QStringLiteral("nav2"),QStringLiteral("planner_server.ros__parameters.GridBased.angle_quantization_bins"),QStringLiteral("int"),16.0,256.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Smac Hybrid-A* 4.6"),QStringLiteral("Cost penalty"),QStringLiteral("nav2"),QStringLiteral("planner_server.ros__parameters.GridBased.cost_penalty"),QStringLiteral("float"),0.1,10.0,0.1,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Smac Hybrid-A* 4.6"),QStringLiteral("Analytic expansion max"),QStringLiteral("nav2"),QStringLiteral("planner_server.ros__parameters.GridBased.analytic_expansion_max_length"),QStringLiteral("float"),0.1,20.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Smac Hybrid-A* 4.6"),QStringLiteral("Non-straight penalty"),QStringLiteral("nav2"),QStringLiteral("planner_server.ros__parameters.GridBased.non_straight_penalty"),QStringLiteral("float"),1.0,10.0,0.1,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("MPPI 4.7"),QStringLiteral("Controller frequency"),QStringLiteral("nav2"),QStringLiteral("controller_server.ros__parameters.controller_frequency"),QStringLiteral("float"),1.0,50.0,1.0,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("MPPI 4.7"),QStringLiteral("Model dt"),QStringLiteral("nav2"),QStringLiteral("controller_server.ros__parameters.FollowPath.model_dt"),QStringLiteral("float"),0.02,0.5,0.005,3,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("MPPI 4.7"),QStringLiteral("Time steps"),QStringLiteral("nav2"),QStringLiteral("controller_server.ros__parameters.FollowPath.time_steps"),QStringLiteral("int"),5.0,100.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("MPPI 4.7"),QStringLiteral("vx max"),QStringLiteral("nav2"),QStringLiteral("controller_server.ros__parameters.FollowPath.vx_max"),QStringLiteral("float"),0.05,3.0,0.01,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("MPPI 4.7"),QStringLiteral("PathAlign weight"),QStringLiteral("nav2"),QStringLiteral("controller_server.ros__parameters.FollowPath.PathAlignCritic.cost_weight"),QStringLiteral("float"),0.0,100.0,0.5,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("MPPI 4.7"),QStringLiteral("PathFollow weight"),QStringLiteral("nav2"),QStringLiteral("controller_server.ros__parameters.FollowPath.PathFollowCritic.cost_weight"),QStringLiteral("float"),0.0,100.0,0.5,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("MPPI 4.7"),QStringLiteral("PathAngle weight"),QStringLiteral("nav2"),QStringLiteral("controller_server.ros__parameters.FollowPath.PathAngleCritic.cost_weight"),QStringLiteral("float"),0.0,100.0,0.5,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Command Pipeline 4.8"),QStringLiteral("Velocity smoother frequency"),QStringLiteral("nav2"),QStringLiteral("velocity_smoother.ros__parameters.smoothing_frequency"),QStringLiteral("float"),1.0,100.0,1.0,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Command Pipeline 4.8"),QStringLiteral("Controller min X velocity threshold"),QStringLiteral("nav2"),QStringLiteral("controller_server.ros__parameters.min_x_velocity_threshold"),QStringLiteral("float"),0.0,1.0,0.01,3,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Command Pipeline 4.8"),QStringLiteral("VelocityDeadbandCritic linear"),QStringLiteral("nav2"),QStringLiteral("controller_server.ros__parameters.FollowPath.VelocityDeadbandCritic.deadband_velocities.0"),QStringLiteral("float"),0.0,1.0,0.01,3,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Command Pipeline 4.8"),QStringLiteral("Max linear accel"),QStringLiteral("nav2"),QStringLiteral("velocity_smoother.ros__parameters.max_accel.0"),QStringLiteral("float"),0.01,5.0,0.01,3,QStringList{
      },QStringLiteral(" m/s²"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Command Pipeline 4.8"),QStringLiteral("Max linear decel"),QStringLiteral("nav2"),QStringLiteral("velocity_smoother.ros__parameters.max_decel.0"),QStringLiteral("float"),-5.0,-0.01,0.01,3,QStringList{
      },QStringLiteral(" m/s²"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Command Pipeline 4.8"),QStringLiteral("Smoother linear deadband"),QStringLiteral("nav2"),QStringLiteral("velocity_smoother.ros__parameters.deadband_velocity.0"),QStringLiteral("float"),0.0,1.0,0.01,3,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Command Pipeline 4.8"),QStringLiteral("NavigationCore linear deadband"),QStringLiteral("navigation_core"),QStringLiteral("navigation_core.ros__parameters.linear_deadband_mps"),QStringLiteral("float"),0.0,1.0,0.01,3,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Command Pipeline 4.8"),QStringLiteral("Minimum speed for yaw"),QStringLiteral("navigation_core"),QStringLiteral("navigation_core.ros__parameters.min_speed_for_yaw_mps"),QStringLiteral("float"),0.0,1.0,0.01,3,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Goal Checker 4.9"),QStringLiteral("XY tolerance"),QStringLiteral("nav2"),QStringLiteral("controller_server.ros__parameters.goal_checker.xy_goal_tolerance"),QStringLiteral("float"),0.05,5.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Goal Checker 4.9"),QStringLiteral("Yaw tolerance"),QStringLiteral("nav2"),QStringLiteral("controller_server.ros__parameters.goal_checker.yaw_goal_tolerance"),QStringLiteral("float"),0.01,3.141592654,0.01,3,QStringList{
      },QStringLiteral(" rad"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("MPPI Supervisor"),QStringLiteral("Status rate"),QStringLiteral("mppi"),QStringLiteral("mppi_closed_loop_supervisor.ros__parameters.status_rate_hz"),QStringLiteral("float"),0.5,100.0,0.5,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Smoother Qualification"),QStringLiteral("Window"),QStringLiteral("mppi"),QStringLiteral("mppi_closed_loop_supervisor.ros__parameters.qualification_window_sec"),QStringLiteral("float"),3.0,120.0,1.0,1,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Smoother Qualification"),QStringLiteral("Min odom rate"),QStringLiteral("mppi"),QStringLiteral("mppi_closed_loop_supervisor.ros__parameters.min_odom_rate_hz"),QStringLiteral("float"),1.0,200.0,1.0,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Smoother Qualification"),QStringLiteral("Max odom jitter"),QStringLiteral("mppi"),QStringLiteral("mppi_closed_loop_supervisor.ros__parameters.max_odom_jitter_sec"),QStringLiteral("float"),0.001,1.0,0.001,3,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Smoother Qualification"),QStringLiteral("Max v RMSE"),QStringLiteral("mppi"),QStringLiteral("mppi_closed_loop_supervisor.ros__parameters.max_velocity_rmse_mps"),QStringLiteral("float"),0.001,2.0,0.005,3,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Smoother Qualification"),QStringLiteral("Max yaw RMSE"),QStringLiteral("mppi"),QStringLiteral("mppi_closed_loop_supervisor.ros__parameters.max_yaw_rate_rmse_rps"),QStringLiteral("float"),0.001,5.0,0.01,3,QStringList{
      },QStringLiteral(" rad/s"),QStringLiteral("")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("camera");
    t.icon=QStringLiteral("▣");
    t.name=QStringLiteral("Kamera Persepsi");
    t.subtitle=QStringLiteral("Citra YOLOPv2 dan deteksi");
    t.pageKind=QStringLiteral("camera");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera"),QStringLiteral("Width"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.rgb_width"),QStringLiteral("int"),160.0,4096.0,16.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera"),QStringLiteral("Height"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.rgb_height"),QStringLiteral("int"),120.0,2160.0,8.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera"),QStringLiteral("FPS target"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.fps"),QStringLiteral("int"),1.0,120.0,1.0,0,QStringList{
      },QStringLiteral(" FPS"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera Capture 4.1"),QStringLiteral("Pixel format"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.v4l2_pixel_format"),QStringLiteral("choice"),-1000000000.0,1000000000.0,0.01,4,QStringList{
        QStringLiteral("MJPEG"),QStringLiteral("YUYV")
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera Capture 4.1"),QStringLiteral("USERPTR zero-copy"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.use_v4l2_userptr_zero_copy"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera Runtime 4.1"),QStringLiteral("Async RViz publish"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.async_rviz_publish"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera Runtime 4.1"),QStringLiteral("RViz max publish rate"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.rviz_max_publish_rate_hz"),QStringLiteral("float"),1.0,60.0,1.0,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera"),QStringLiteral("Confidence threshold"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.confidence_threshold"),QStringLiteral("float"),0.01,0.99,0.01,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera"),QStringLiteral("NMS IoU threshold"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.iou_threshold"),QStringLiteral("float"),0.01,0.99,0.01,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera"),QStringLiteral("Lane threshold"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.lane_threshold"),QStringLiteral("float"),0.01,0.99,0.01,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera Intrinsic"),QStringLiteral("fx"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.camera_fx"),QStringLiteral("float"),1.0,5000.0,0.1,3,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera Intrinsic"),QStringLiteral("fy"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.camera_fy"),QStringLiteral("float"),1.0,5000.0,0.1,3,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera Intrinsic"),QStringLiteral("cx"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.camera_cx"),QStringLiteral("float"),0.0,5000.0,0.1,3,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera Intrinsic"),QStringLiteral("cy"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.camera_cy"),QStringLiteral("float"),0.0,5000.0,0.1,3,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Camera Intrinsic"),QStringLiteral("Distortion [k1,k2,p1,p2,k3]"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.camera_distortion"),QStringLiteral("list"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("ground_plane");
    t.icon=QStringLiteral("▱");
    t.name=QStringLiteral("Ground Plane Persepsi");
    t.subtitle=QStringLiteral("Homography pixel↔meter");
    t.pageKind=QStringLiteral("ground_plane");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Calibration Image"),QStringLiteral("Calibration width"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.ground_calibration_width"),QStringLiteral("int"),100.0,5000.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Calibration Image"),QStringLiteral("Calibration height"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.ground_calibration_height"),QStringLiteral("int"),100.0,5000.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Homography"),QStringLiteral("Source points [8]"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.ground_src_points"),QStringLiteral("list"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Homography"),QStringLiteral("Destination points [8]"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.ground_dst_points"),QStringLiteral("list"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Ground Canvas"),QStringLiteral("Origin X px"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.ground_origin_x_px"),QStringLiteral("float"),-10000.0,10000.0,0.1,4,QStringList{
      },QStringLiteral(" px"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Ground Canvas"),QStringLiteral("Origin Y px"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.ground_origin_y_px"),QStringLiteral("float"),-10000.0,10000.0,0.1,4,QStringList{
      },QStringLiteral(" px"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Ground Canvas"),QStringLiteral("Meters / pixel X"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.ground_meters_per_pixel_x"),QStringLiteral("float"),1e-05,1.0,1e-05,8,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Ground Canvas"),QStringLiteral("Meters / pixel Y"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.ground_meters_per_pixel_y"),QStringLiteral("float"),1e-05,1.0,1e-05,8,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Metric ROI"),QStringLiteral("Min forward"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.metric_minimum_forward_m"),QStringLiteral("float"),0.0,20.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Metric ROI"),QStringLiteral("Max forward"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.metric_maximum_forward_m"),QStringLiteral("float"),0.1,100.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Metric ROI"),QStringLiteral("Max abs lateral"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.metric_maximum_abs_left_m"),QStringLiteral("float"),0.1,20.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Metric Calibration 4.3"),QStringLiteral("Forward offset"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.metric_forward_offset_m"),QStringLiteral("float"),-5.0,5.0,0.01,4,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Metric Calibration 4.3"),QStringLiteral("Lateral offset"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.metric_lateral_offset_m"),QStringLiteral("float"),-5.0,5.0,0.01,4,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.3 Calibration Gate"),QStringLiteral("Require calibrated metric"),QStringLiteral("navigation_core"),QStringLiteral("navigation_core.ros__parameters.require_camera_metric_calibration"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("4.3 Calibration Gate"),QStringLiteral("Metric calibration validated"),QStringLiteral("navigation_core"),QStringLiteral("navigation_core.ros__parameters.camera_metric_calibration_validated"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("Set TRUE hanya setelah MAE/RMSE 4.3 dinyatakan lulus.")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("obstacle");
    t.icon=QStringLiteral("▲");
    t.name=QStringLiteral("Persepsi Obstacle");
    t.subtitle=QStringLiteral("Kelas, kontak drivable, dan jarak");
    t.pageKind=QStringLiteral("obstacle");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Detection"),QStringLiteral("Minimum obstacle confidence"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.minimum_obstacle_confidence"),QStringLiteral("float"),0.01,0.99,0.01,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Detection"),QStringLiteral("Accept all detected classes"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.accept_all_detected_classes_as_obstacles"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("Matikan bila filter class spesifik diaktifkan.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Detection"),QStringLiteral("Safety obstacle class IDs"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.safety_obstacle_class_ids"),QStringLiteral("list"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("Bab IV meminta verifikasi raw class sebelum mapping ini dinyatakan final.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Drivable Contact"),QStringLiteral("Require drivable contact"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.require_drivable_contact"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Drivable Contact"),QStringLiteral("Contact radius"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.drivable_contact_radius_px"),QStringLiteral("int"),0.0,100.0,1.0,0,QStringList{
      },QStringLiteral(" px"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Drivable Contact"),QStringLiteral("Vertical tolerance"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.drivable_contact_vertical_tolerance_px"),QStringLiteral("int"),0.0,100.0,1.0,0,QStringList{
      },QStringLiteral(" px"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Obstacle ROI"),QStringLiteral("Forward min"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.obstacle_forward_min_m"),QStringLiteral("float"),0.0,20.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Obstacle ROI"),QStringLiteral("Forward max"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.obstacle_forward_max_m"),QStringLiteral("float"),0.1,50.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Obstacle ROI"),QStringLiteral("Lateral margin"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.obstacle_lateral_margin_m"),QStringLiteral("float"),0.0,5.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Drivable Contact"),QStringLiteral("Minimum samples"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.drivable_contact_min_samples"),QStringLiteral("int"),1.0,50.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Drivable Contact"),QStringLiteral("Minimum fraction"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.drivable_contact_min_fraction"),QStringLiteral("float"),0.0,1.0,0.01,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("PointCloud"),QStringLiteral("Points per box"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.points_per_box"),QStringLiteral("int"),1.0,20.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("PointCloud"),QStringLiteral("Max publish rate"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.obstacle_cloud_max_publish_rate_hz"),QStringLiteral("float"),1.0,60.0,1.0,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Tracking"),QStringLiteral("Match distance"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.track_match_distance_m"),QStringLiteral("float"),0.05,5.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Tracking"),QStringLiteral("EMA alpha"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.track_ema_alpha"),QStringLiteral("float"),0.01,1.0,0.01,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Tracking"),QStringLiteral("Confirm hits"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.track_confirm_hits"),QStringLiteral("int"),1.0,30.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Tracking"),QStringLiteral("Max missed frames"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.track_max_missed_frames"),QStringLiteral("int"),0.0,100.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Tracking"),QStringLiteral("Clearing margin"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.clearing_obstacle_margin_m"),QStringLiteral("float"),0.0,2.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Obstacle State"),QStringLiteral("Minimum width"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.minimum_obstacle_width_m"),QStringLiteral("float"),0.0,3.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Obstacle State"),QStringLiteral("Blocked confirm frames"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.blocked_confirm_frames"),QStringLiteral("int"),1.0,60.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Obstacle State"),QStringLiteral("Clear confirm frames"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.clear_confirm_frames"),QStringLiteral("int"),1.0,60.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("lane");
    t.icon=QStringLiteral("║");
    t.name=QStringLiteral("Safety Jalur");
    t.subtitle=QStringLiteral("Area bahaya, centering, dan corridor");
    t.pageKind=QStringLiteral("lane");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Road Geometry"),QStringLiteral("Nominal road width"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.nominal_road_width_m"),QStringLiteral("float"),1.0,20.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Road Geometry"),QStringLiteral("Vehicle width"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.lane_vehicle_width_m"),QStringLiteral("float"),0.1,5.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Danger Area"),QStringLiteral("Edge warning clearance"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.edge_warning_clearance_m"),QStringLiteral("float"),0.05,5.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Danger Area"),QStringLiteral("Edge critical clearance"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.edge_critical_clearance_m"),QStringLiteral("float"),0.05,5.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Danger Area"),QStringLiteral("Edge release clearance"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.edge_release_clearance_m"),QStringLiteral("float"),0.05,5.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Centering"),QStringLiteral("Center deadband"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.center_deadband_m"),QStringLiteral("float"),0.0,3.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Centering"),QStringLiteral("Center gain"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.center_gain"),QStringLiteral("float"),0.0,5.0,0.05,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Centering"),QStringLiteral("Heading gain"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.heading_gain"),QStringLiteral("float"),0.0,5.0,0.05,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("State Filter"),QStringLiteral("Confirm frames"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.state_confirm_frames"),QStringLiteral("int"),1.0,60.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("State Filter"),QStringLiteral("Release frames"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.release_confirm_frames"),QStringLiteral("int"),1.0,60.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("State Filter"),QStringLiteral("Lost confirm frames"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.lost_confirm_frames"),QStringLiteral("int"),1.0,60.0,1.0,0,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Lane Metric"),QStringLiteral("Minimum road width"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.lane_minimum_road_width_m"),QStringLiteral("float"),1.0,20.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Lane Metric"),QStringLiteral("Maximum road width"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.lane_maximum_road_width_m"),QStringLiteral("float"),1.0,20.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Lane Metric"),QStringLiteral("Near lookahead"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.lane_lookahead_near_m"),QStringLiteral("float"),0.1,10.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Lane Metric"),QStringLiteral("Far lookahead"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.lane_lookahead_far_m"),QStringLiteral("float"),0.2,20.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Lane Metric"),QStringLiteral("Control lookahead"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.lane_control_lookahead_m"),QStringLiteral("float"),0.1,20.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Lane Metric"),QStringLiteral("EMA alpha"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.lane_ema_alpha"),QStringLiteral("float"),0.01,1.0,0.01,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Lane Metric"),QStringLiteral("Minimum metric confidence"),QStringLiteral("perception"),QStringLiteral("perception.ros__parameters.minimum_metric_confidence"),QStringLiteral("float"),0.0,1.0,0.01,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("safety");
    t.icon=QStringLiteral("⬢");
    t.name=QStringLiteral("Safety & Collision Monitor");
    t.subtitle=QStringLiteral("Trajectory Safety + certified Collision Monitor production");
    t.pageKind=QStringLiteral("safety");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Trajectory Safety 4.5"),QStringLiteral("Control rate"),QStringLiteral("trajectory_safety"),QStringLiteral("trajectory_safety_supervisor.ros__parameters.control_rate_hz"),QStringLiteral("float"),1.0,100.0,1.0,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Local Costmap 4.5"),QStringLiteral("Update frequency"),QStringLiteral("nav2"),QStringLiteral("local_costmap.local_costmap.ros__parameters.update_frequency"),QStringLiteral("float"),0.1,50.0,0.1,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Local Costmap 4.5"),QStringLiteral("Publish frequency"),QStringLiteral("nav2"),QStringLiteral("local_costmap.local_costmap.ros__parameters.publish_frequency"),QStringLiteral("float"),0.1,50.0,0.1,1,QStringList{
      },QStringLiteral(" Hz"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Local Costmap 4.5"),QStringLiteral("Resolution"),QStringLiteral("nav2"),QStringLiteral("local_costmap.local_costmap.ros__parameters.resolution"),QStringLiteral("float"),0.01,1.0,0.01,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Trajectory Safety 4.5"),QStringLiteral("Trajectory lateral margin"),QStringLiteral("trajectory_safety"),QStringLiteral("trajectory_safety_supervisor.ros__parameters.trajectory_lateral_margin_m"),QStringLiteral("float"),0.0,2.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Trajectory Safety 4.5"),QStringLiteral("Planning lateral margin"),QStringLiteral("trajectory_safety"),QStringLiteral("trajectory_safety_supervisor.ros__parameters.planning_lateral_margin_m"),QStringLiteral("float"),0.0,3.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Trajectory Safety 4.5"),QStringLiteral("Path horizon"),QStringLiteral("trajectory_safety"),QStringLiteral("trajectory_safety_supervisor.ros__parameters.path_horizon_m"),QStringLiteral("float"),0.5,10.0,0.1,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Trajectory Safety 4.5"),QStringLiteral("Command collision horizon"),QStringLiteral("trajectory_safety"),QStringLiteral("trajectory_safety_supervisor.ros__parameters.command_collision_horizon_m"),QStringLiteral("float"),0.1,5.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Trajectory Safety 4.5"),QStringLiteral("Immediate hard stop"),QStringLiteral("trajectory_safety"),QStringLiteral("trajectory_safety_supervisor.ros__parameters.immediate_command_hard_stop_m"),QStringLiteral("float"),0.1,5.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Trajectory Safety 4.5"),QStringLiteral("Hard-stop path distance"),QStringLiteral("trajectory_safety"),QStringLiteral("trajectory_safety_supervisor.ros__parameters.hard_stop_path_distance_m"),QStringLiteral("float"),0.1,5.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Trajectory Safety 4.5"),QStringLiteral("Slow path distance"),QStringLiteral("trajectory_safety"),QStringLiteral("trajectory_safety_supervisor.ros__parameters.slow_path_distance_m"),QStringLiteral("float"),0.1,10.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Trajectory Safety 4.5"),QStringLiteral("Minimum slow scale"),QStringLiteral("trajectory_safety"),QStringLiteral("trajectory_safety_supervisor.ros__parameters.minimum_slow_speed_scale"),QStringLiteral("float"),0.0,1.0,0.05,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Trajectory Safety 4.5"),QStringLiteral("Avoidance side margin"),QStringLiteral("trajectory_safety"),QStringLiteral("trajectory_safety_supervisor.ros__parameters.avoidance_side_margin_m"),QStringLiteral("float"),0.0,2.0,0.05,2,QStringList{
      },QStringLiteral(" m"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Trajectory Safety 4.5"),QStringLiteral("Avoidance speed"),QStringLiteral("trajectory_safety"),QStringLiteral("trajectory_safety_supervisor.ros__parameters.avoidance_speed_mps"),QStringLiteral("float"),0.0,2.0,0.01,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Trajectory Safety 4.5"),QStringLiteral("Warning lane speed"),QStringLiteral("trajectory_safety"),QStringLiteral("trajectory_safety_supervisor.ros__parameters.warning_lane_speed_mps"),QStringLiteral("float"),0.0,2.0,0.01,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Trajectory Safety 4.5"),QStringLiteral("Critical lane speed"),QStringLiteral("trajectory_safety"),QStringLiteral("trajectory_safety_supervisor.ros__parameters.critical_lane_speed_mps"),QStringLiteral("float"),0.0,2.0,0.01,2,QStringList{
      },QStringLiteral(" m/s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Collision Production"),QStringLiteral("Transform tolerance"),QStringLiteral("collision"),QStringLiteral("collision_monitor.ros__parameters.transform_tolerance"),QStringLiteral("float"),0.01,5.0,0.01,2,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Collision Production"),QStringLiteral("Source timeout"),QStringLiteral("collision"),QStringLiteral("collision_monitor.ros__parameters.source_timeout"),QStringLiteral("float"),0.05,10.0,0.05,2,QStringList{
      },QStringLiteral(" s"),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Collision Production"),QStringLiteral("Stop polygon enabled"),QStringLiteral("collision"),QStringLiteral("collision_monitor.ros__parameters.PolygonStop.enabled"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Collision Production"),QStringLiteral("Stop points [x1,y1,...]"),QStringLiteral("collision"),QStringLiteral("collision_monitor.ros__parameters.PolygonStop.points"),QStringLiteral("list"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Collision Production"),QStringLiteral("Slow polygon enabled"),QStringLiteral("collision"),QStringLiteral("collision_monitor.ros__parameters.PolygonSlow.enabled"),QStringLiteral("bool"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Collision Production"),QStringLiteral("Slowdown ratio"),QStringLiteral("collision"),QStringLiteral("collision_monitor.ros__parameters.PolygonSlow.slowdown_ratio"),QStringLiteral("float"),0.0,1.0,0.05,2,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    tabs.push_back(t);
  }
  {
    TabDef t;
    t.key=QStringLiteral("reports");
    t.icon=QStringLiteral("▤");
    t.name=QStringLiteral("Pelaporan");
    t.subtitle=QStringLiteral("CSV, PNG, dan bukti pengujian");
    t.pageKind=QStringLiteral("reports");
    t.specs.push_back(SettingSpec{
      QStringLiteral("Output"),QStringLiteral("Folder output"),QStringLiteral("gui"),QStringLiteral("reporting.output_directory"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Output"),QStringLiteral("Nama proyek"),QStringLiteral("gui"),QStringLiteral("reporting.project_name"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Output"),QStringLiteral("Operator"),QStringLiteral("gui"),QStringLiteral("reporting.operator"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Output"),QStringLiteral("Kode sesi / batch"),QStringLiteral("gui"),QStringLiteral("reporting.session_id"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("Masukkan ID pengujian BAB IV agar CSV mudah ditelusuri.")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Experiment"),QStringLiteral("Kategori"),QStringLiteral("gui"),QStringLiteral("reporting.experiment_category"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Experiment"),QStringLiteral("Varian"),QStringLiteral("gui"),QStringLiteral("reporting.experiment_variant"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    t.specs.push_back(SettingSpec{
      QStringLiteral("Experiment"),QStringLiteral("Catatan"),QStringLiteral("gui"),QStringLiteral("reporting.experiment_notes"),QStringLiteral("text"),-1000000000.0,1000000000.0,0.01,4,QStringList{
      },QStringLiteral(""),QStringLiteral("")
    });
    tabs.push_back(t);
  }
  return tabs;
}
