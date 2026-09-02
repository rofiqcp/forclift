# Native Qt GUI modules

File GUI dibagi menjadi modul implementasi C++ agar `agv_gui.cpp` tidak menjadi satu file raksasa. Ekstensi yang dipakai `.cpp` (bukan `.c`) karena kode memakai Qt, ROS 2 C++, STL, class, template, dan lambda.

- `gui_core.cpp` — common utilities, workspace/YAML/telemetry helpers.
- `reporting_widgets.cpp` — plots dan report manager.
- `system_and_gnss_pages.cpp` — diagnostic, connection, tuning, experiment, IMU/GNSS pages.
- `steering_calibration_page.cpp` — physical steering calibration dan LUT workflow.
- `esc_map_overview_pages.cpp` — ESC calibration, OSM/map, overview widgets.
- `camera_reports_pages.cpp` — camera/perception calibration dan report pages.
- `main_window.cpp` — main window, satu floating BAB IV menu dengan tiga tab (Navigasi, Persepsi, ESC), autosave/runtime controls.

Menu pengujian tidak membuat ulang workspace. Setiap leaf 4.x merutekan ke `ExperimentWorkspacePage` yang sama sehingga ROS subscription, akuisisi CSV/PNG/raw, dan state YAML tetap satu sumber.

Modul ini di-include sebagai implementation partitions oleh `agv_gui.cpp` supaya class Qt yang saling bergantung tetap berada pada satu translation unit dan `AUTOMOC` tetap stabil pada ROS 2 Humble. Semua source sudah diformat menjadi baris normal; tidak ada lagi baris 4000+ kolom.
