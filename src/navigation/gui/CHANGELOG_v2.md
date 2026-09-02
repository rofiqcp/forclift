# GUI Changelog — C++/Qt Migration

## v3.7.0

- GUI operator dikonversi penuh dari PyQt5/rclpy menjadi C++17 + Qt5 + rclcpp.
- 17 halaman dan 377 parameter tuning tetap tersedia.
- ROS callback berjalan pada `MultiThreadedExecutor` terpisah dari event-loop Qt.
- YAML autosave atomik, Config-vs-Runtime audit, kalibrasi steering/ESC/IMU/GNSS/camera, MPPI sweep, map/OSM, reporting, rosbag, dan profiling dipertahankan.
- Pelaporan C++ menambahkan numeric summary serta RMSE untuk kolom error/residual.
- Dependency `rclpy` dan `python3-pyqt5` dihapus dari GUI runtime.
