#!/usr/bin/env python3
"""Qt5 GUI untuk tuning ESC FOC/PID melalui USART3.

Tujuan desain:
- panel kiri untuk koneksi, kontrol, PID, telemetry, EEPROM, dan CSV;
- panel kanan untuk tabel data live dan grafik real-time;
- hot-plug aman tanpa reconnect-loop palsu;
- PID dapat dibaca dari STM32, di-apply ke RAM, dan disimpan + diverifikasi EEPROM;
- logging CSV buffered agar tidak membebani GUI pada telemetry rate tinggi.
"""
from __future__ import annotations

import csv
import glob
import os
import sys
import time
from collections import defaultdict, deque
from datetime import datetime
from pathlib import Path

import pyqtgraph as pg
import serial
from serial.tools import list_ports
from PyQt5 import QtCore, QtGui, QtWidgets

from protocol import *

MODE_NAMES = {
    "OPEN": MODE_OPEN,
    "VLT": MODE_VLT,
    "TRQ": MODE_TRQ,
    "SPD": MODE_SPD,
    "POS": MODE_POS,
}
LOOP_NAMES = {
    "Id / Current D": PID_CURRENT_D,
    "Iq / Torque": PID_TORQUE_Q,
    "Speed": PID_SPEED,
    "Position": PID_POSITION,
}
PAGE_NAMES = {
    "Basic": TELEM_BASIC,
    "FOC": TELEM_FOC,
    "PID": TELEM_PID,
    "Raw / Debug": TELEM_RAW,
}
PAGE_VARIABLES = {
    TELEM_BASIC: [
        "position_left", "position_right", "setpoint_left", "setpoint_right",
        "speed_left", "speed_right", "battery_centi_volt", "temperature_deci_c",
        "dc_current_left_centi_amp", "dc_current_right_centi_amp",
        "command_left", "command_right", "link_age_ms", "telemetry_rate_hz",
    ],
    TELEM_FOC: [
        "id_left", "iq_left", "id_right", "iq_right",
        "phase_a_left", "phase_b_left", "phase_b_right", "phase_c_right",
        "dc_link_left", "dc_link_right",
        "duty_u_left", "duty_v_left", "duty_w_left",
        "duty_u_right", "duty_v_right", "duty_w_right",
        "electrical_angle_left", "electrical_angle_right",
        "speed_left", "speed_right", "hall_bits",
    ],
    TELEM_PID: [
        "setpoint_left", "setpoint_right", "measured_left", "measured_right",
        "error_left_pid", "error_right_pid",
        "p_left", "i_left", "d_left", "output_left",
        "p_right", "i_right", "d_right", "output_right", "antiwindup_flags",
    ],
    TELEM_RAW: [
        "adc_phase_a_left", "adc_phase_b_left", "adc_phase_b_right", "adc_phase_c_right",
        "adc_dc_left", "adc_dc_right", "adc_battery", "adc_temperature",
        "hall_left", "hall_right", "pwm_period", "main_loop_counter",
        "valid_frames", "bad_frames", "reconnect_counter",
        "core_speed_left", "core_speed_right", "eeprom_crc",
        "eeprom_generation", "eeprom_verify_failures",
    ],
}

BASE_FRAME_FIELDS = [
    "sequence", "page", "status", "uptime_ms", "mode_left", "mode_right",
    "error_left", "error_right",
]
CSV_FIELDS = ["host_time_iso", "host_monotonic_s"] + BASE_FRAME_FIELDS
for _page_vars in PAGE_VARIABLES.values():
    for _name in _page_vars:
        if _name not in CSV_FIELDS:
            CSV_FIELDS.append(_name)
CSV_FIELDS += [
    "config_motor", "config_loop", "config_kp", "config_ki", "config_kd",
    "config_i_limit", "config_output_min", "config_output_max",
    "config_position_min", "config_position_max", "config_position_deadband",
    "config_eeprom_crc", "config_eeprom_generation",
    "config_eeprom_verify_failures", "config_eeprom_verified", "config_settings_dirty",
    # Snapshot state GUI pada setiap baris agar hasil tuning dapat direproduksi.
    "gui_port", "gui_mode_left", "gui_mode_right", "gui_setpoint_left", "gui_setpoint_right",
    "gui_pid_motor", "gui_pid_loop", "gui_kp", "gui_ki", "gui_kd",
    "gui_i_limit", "gui_output_min", "gui_output_max",
    "gui_position_min", "gui_position_max", "gui_position_deadband",
]


class MainWindow(QtWidgets.QMainWindow):
    """Window utama tuner ESC."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ESC FOC/PID Tuner - USART3 - Qt5")
        self.resize(1550, 900)

        # ---------------- Serial/link state ----------------
        self.serial_port: serial.Serial | None = None
        self.parser = FeedbackParser()
        self.sequence = 0
        self.latest: dict = {}
        self.latest_page_data: dict[int, dict] = {}
        self.last_connect_try = 0.0
        self.last_keepalive = 0.0
        self.last_feedback_time = 0.0
        self.last_handshake_try = 0.0
        self.last_port_refresh = 0.0
        self.handshake_retry_count = 0
        self.connected_path = ""
        self.preferred_port_identity: dict | None = None
        # Tidak auto-connect pada startup. Setelah user menekan Connect, hot-plug
        # boleh menjaga intent koneksi sampai user menekan Disconnect.
        self.connection_requested = False
        self.user_arm_requested = False
        self.arm_handshake_pending = False

        # ---------------- Plot state ----------------
        self.series_x = defaultdict(lambda: deque(maxlen=5000))
        self.series_y = defaultdict(lambda: deque(maxlen=5000))
        self.curves: dict[str, object] = {}
        self.graph_start_time = time.monotonic()
        self.last_plot_refresh = 0.0
        self.last_table_refresh = 0.0

        # ---------------- EEPROM/PID verification ----------------
        self.pending_pid_verify: dict | None = None
        self.last_eeprom_status = "Belum diverifikasi"

        # ---------------- CSV state ----------------
        self.csv_handle = None
        self.csv_writer = None
        self.csv_rows = 0
        self.csv_last_flush = 0.0
        self.csv_last_ui_update = 0.0

        self._build_ui()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(20)

    # ======================================================================
    # UI
    # ======================================================================
    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)

        self.status_bar_label = QtWidgets.QLabel("DISCONNECTED")
        self.status_bar_label.setMinimumHeight(28)
        root.addWidget(self.status_bar_label)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        root.addWidget(splitter, 1)

        # Left control panel.
        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 4, 0)
        self.tabs = QtWidgets.QTabWidget()
        left_layout.addWidget(self.tabs)
        splitter.addWidget(left)

        self._create_connection_tab()
        self._create_control_tab()
        self._create_pid_tab()
        self._create_telemetry_tab()
        self._create_eeprom_tab()
        self._create_csv_tab()

        # Right live-data + graph panel.
        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        right_layout.setContentsMargins(4, 0, 0, 0)

        live_group = QtWidgets.QGroupBox("Live Data - halaman telemetry aktif")
        live_layout = QtWidgets.QVBoxLayout(live_group)
        self.live_table = QtWidgets.QTableWidget(0, 3)
        self.live_table.setHorizontalHeaderLabels(["Variable", "Raw", "Engineering"])
        self.live_table.horizontalHeader().setStretchLastSection(True)
        self.live_table.verticalHeader().setVisible(False)
        self.live_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.live_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.live_table.setMaximumHeight(260)
        live_layout.addWidget(self.live_table)
        right_layout.addWidget(live_group)

        plot_toolbar = QtWidgets.QHBoxLayout()
        self.pause_plot = QtWidgets.QCheckBox("Pause graph")
        self.auto_range = QtWidgets.QCheckBox("Auto range Y")
        self.auto_range.setChecked(True)
        self.plot_window = QtWidgets.QDoubleSpinBox()
        self.plot_window.setRange(1.0, 300.0)
        self.plot_window.setValue(20.0)
        self.plot_window.setSuffix(" s")
        clear_plot = QtWidgets.QPushButton("Clear graph")
        clear_plot.clicked.connect(self.clear_graph)
        plot_toolbar.addWidget(self.pause_plot)
        plot_toolbar.addWidget(self.auto_range)
        plot_toolbar.addWidget(QtWidgets.QLabel("Window"))
        plot_toolbar.addWidget(self.plot_window)
        plot_toolbar.addWidget(clear_plot)
        plot_toolbar.addStretch()
        right_layout.addLayout(plot_toolbar)

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel("bottom", "Time", units="s")
        self.plot.addLegend()
        right_layout.addWidget(self.plot, 1)
        splitter.addWidget(right)
        splitter.setSizes([470, 1080])

    def _create_connection_tab(self) -> None:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        form = QtWidgets.QFormLayout()

        port_row = QtWidgets.QHBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setEditable(True)
        refresh = QtWidgets.QPushButton("Refresh")
        refresh.clicked.connect(self.refresh_ports)
        port_row.addWidget(self.port_combo, 1)
        port_row.addWidget(refresh)
        form.addRow("Serial port", port_row)

        self.auto_reconnect = QtWidgets.QCheckBox("Hot-plug / reconnect otomatis setelah koneksi pertama")
        self.auto_reconnect.setChecked(True)
        form.addRow(self.auto_reconnect)

        self.connect_button = QtWidgets.QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        form.addRow(self.connect_button)

        self.link_info = QtWidgets.QLabel("Protocol v4 | 115200 8N1 | frame 64 byte CRC16")
        self.link_info.setWordWrap(True)
        form.addRow(self.link_info)

        self.link_detail = QtWidgets.QLabel("Port belum dibuka")
        self.link_detail.setWordWrap(True)
        form.addRow("Link", self.link_detail)

        layout.addLayout(form)
        layout.addStretch()
        self.tabs.addTab(widget, "Connection")
        self.refresh_ports()

    def _create_control_tab(self) -> None:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        form = QtWidgets.QFormLayout()

        self.mode_left = QtWidgets.QComboBox()
        self.mode_right = QtWidgets.QComboBox()
        self.mode_left.addItems(MODE_NAMES.keys())
        self.mode_right.addItems(MODE_NAMES.keys())
        self.mode_left.setCurrentText("SPD")
        self.mode_right.setCurrentText("SPD")

        self.setpoint_left = QtWidgets.QSpinBox()
        self.setpoint_right = QtWidgets.QSpinBox()
        for spin in (self.setpoint_left, self.setpoint_right):
            spin.setRange(-1000, 1000)

        self.mode_left.currentTextChanged.connect(
            lambda _: self.update_setpoint_range(self.mode_left, self.setpoint_left)
        )
        self.mode_right.currentTextChanged.connect(
            lambda _: self.update_setpoint_range(self.mode_right, self.setpoint_right)
        )

        form.addRow("Mode LEFT", self.mode_left)
        form.addRow("Setpoint LEFT", self.setpoint_left)
        form.addRow("Mode RIGHT", self.mode_right)
        form.addRow("Setpoint RIGHT", self.setpoint_right)

        current_row = QtWidgets.QHBoxLayout()
        use_left = QtWidgets.QPushButton("POS L = current")
        use_right = QtWidgets.QPushButton("POS R = current")
        use_left.clicked.connect(lambda: self.setpoint_left.setValue(int(self.latest.get("position_left", 0))))
        use_right.clicked.connect(lambda: self.setpoint_right.setValue(int(self.latest.get("position_right", 0))))
        current_row.addWidget(use_left)
        current_row.addWidget(use_right)
        form.addRow(current_row)

        arm_row = QtWidgets.QHBoxLayout()
        arm = QtWidgets.QPushButton("ARM")
        arm.setStyleSheet("font-weight: bold;")
        disarm = QtWidgets.QPushButton("DISARM")
        disarm.setStyleSheet("font-weight: bold;")
        arm.clicked.connect(self.request_arm)
        disarm.clicked.connect(self.send_disarm)
        arm_row.addWidget(arm)
        arm_row.addWidget(disarm)
        form.addRow(arm_row)

        note = QtWidgets.QLabel(
            "Setpoint host memakai koordinat mekanik yang sama untuk kiri/kanan. "
            "Firmware membalik command core motor kanan seperti firmware asli. "
            "POS memakai signed Hall position + soft-limit + deadband."
        )
        note.setWordWrap(True)
        form.addRow(note)
        layout.addLayout(form)
        layout.addStretch()
        self.tabs.addTab(widget, "Control")

    def _create_pid_tab(self) -> None:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        form = QtWidgets.QFormLayout()

        self.pid_motor = QtWidgets.QComboBox()
        self.pid_motor.addItems(["Left", "Right", "Both"])
        self.pid_loop = QtWidgets.QComboBox()
        self.pid_loop.addItems(LOOP_NAMES.keys())

        self.kp = QtWidgets.QDoubleSpinBox()
        self.ki = QtWidgets.QDoubleSpinBox()
        self.kd = QtWidgets.QDoubleSpinBox()
        for spin in (self.kp, self.ki, self.kd):
            spin.setRange(0.0, 32767.0)
            spin.setKeyboardTracking(False)

        self.integral_limit = QtWidgets.QDoubleSpinBox()
        self.integral_limit.setRange(0.0, 32767.0)
        self.integral_limit.setDecimals(3)
        self.integral_limit.setValue(200.0)

        self.output_min = QtWidgets.QSpinBox()
        self.output_max = QtWidgets.QSpinBox()
        self.output_min.setRange(-1000, 0)
        self.output_min.setValue(-300)
        self.output_max.setRange(0, 1000)
        self.output_max.setValue(300)

        self.position_min = QtWidgets.QSpinBox()
        self.position_max = QtWidgets.QSpinBox()
        for spin in (self.position_min, self.position_max):
            spin.setRange(-2_000_000_000, 2_000_000_000)
        self.position_min.setValue(-200000)
        self.position_max.setValue(200000)

        self.position_deadband = QtWidgets.QSpinBox()
        self.position_deadband.setRange(0, 1000)
        self.position_deadband.setValue(2)
        self.position_deadband.setSuffix(" ticks")

        rows = [
            ("Motor", self.pid_motor), ("Loop", self.pid_loop),
            ("Kp", self.kp), ("Ki", self.ki), ("Kd", self.kd),
            ("Integral limit (POS)", self.integral_limit),
            ("Output min (POS)", self.output_min), ("Output max (POS)", self.output_max),
            ("Position min", self.position_min), ("Position max", self.position_max),
            ("Deadband POS", self.position_deadband),
        ]
        for label, control in rows:
            form.addRow(label, control)

        button_row1 = QtWidgets.QHBoxLayout()
        read = QtWidgets.QPushButton("Read from STM32")
        apply = QtWidgets.QPushButton("Apply RAM (DISARM)")
        read.clicked.connect(self.request_pid_config)
        apply.clicked.connect(lambda: self.send_pid(save=False))
        button_row1.addWidget(read)
        button_row1.addWidget(apply)
        form.addRow(button_row1)

        button_row2 = QtWidgets.QHBoxLayout()
        save = QtWidgets.QPushButton("Apply + Save + Verify (DISARM)")
        load = QtWidgets.QPushButton("Load EEPROM")
        save.clicked.connect(lambda: self.send_pid(save=True))
        load.clicked.connect(self.load_eeprom)
        button_row2.addWidget(save)
        button_row2.addWidget(load)
        form.addRow(button_row2)

        self.pid_verify_label = QtWidgets.QLabel("PID belum dibaca dari STM32")
        self.pid_verify_label.setWordWrap(True)
        form.addRow("Verify", self.pid_verify_label)

        self.pid_note = QtWidgets.QLabel()
        self.pid_note.setWordWrap(True)
        form.addRow(self.pid_note)

        layout.addLayout(form)
        layout.addStretch()
        self.pid_loop.currentIndexChanged.connect(self.update_pid_field_state)
        self.update_pid_field_state()
        self.tabs.addTab(widget, "PID Tuning")

    def _create_telemetry_tab(self) -> None:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        form = QtWidgets.QFormLayout()

        self.telemetry_page = QtWidgets.QComboBox()
        self.telemetry_page.addItems(PAGE_NAMES.keys())
        self.telemetry_rate = QtWidgets.QSpinBox()
        self.telemetry_rate.setRange(1, 100)
        self.telemetry_rate.setValue(50)
        self.telemetry_rate.setSuffix(" Hz")
        self.observe_loop = QtWidgets.QComboBox()
        self.observe_loop.addItems(LOOP_NAMES.keys())
        form.addRow("Page", self.telemetry_page)
        form.addRow("Rate", self.telemetry_rate)
        form.addRow("PID loop observed", self.observe_loop)
        layout.addLayout(form)

        self.variable_list = QtWidgets.QListWidget()
        self.variable_list.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
        layout.addWidget(self.variable_list, 1)

        select_row = QtWidgets.QHBoxLayout()
        all_button = QtWidgets.QPushButton("Select all")
        none_button = QtWidgets.QPushButton("Clear selection")
        all_button.clicked.connect(self.variable_list.selectAll)
        none_button.clicked.connect(self.variable_list.clearSelection)
        select_row.addWidget(all_button)
        select_row.addWidget(none_button)
        layout.addLayout(select_row)

        apply = QtWidgets.QPushButton("Apply telemetry + graph")
        apply.clicked.connect(self.apply_telemetry)
        layout.addWidget(apply)

        tip = QtWidgets.QLabel(
            "STM32 hanya mengisi field yang dipilih untuk page aktif. Field status/EEPROM debug tertentu tetap dikirim agar diagnosis link tetap tersedia."
        )
        tip.setWordWrap(True)
        layout.addWidget(tip)

        self.telemetry_page.currentIndexChanged.connect(self.refresh_variables)
        self.refresh_variables()
        self.tabs.addTab(widget, "Telemetry")

    def _create_eeprom_tab(self) -> None:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        form = QtWidgets.QFormLayout()

        self.eeprom_verified_label = QtWidgets.QLabel("False")
        self.eeprom_dirty_label = QtWidgets.QLabel("False")
        self.eeprom_generation_label = QtWidgets.QLabel("0")
        self.eeprom_crc_label = QtWidgets.QLabel("0x0000")
        self.eeprom_fail_label = QtWidgets.QLabel("0")
        self.eeprom_action_label = QtWidgets.QLabel("Belum ada operasi EEPROM")
        self.eeprom_action_label.setWordWrap(True)
        form.addRow("Flash verified", self.eeprom_verified_label)
        form.addRow("RAM unsaved / dirty", self.eeprom_dirty_label)
        form.addRow("Generation", self.eeprom_generation_label)
        form.addRow("Stored CRC", self.eeprom_crc_label)
        form.addRow("Verify failures", self.eeprom_fail_label)
        form.addRow("Last action", self.eeprom_action_label)
        layout.addLayout(form)

        save = QtWidgets.QPushButton("Save ALL current settings + read-back verify")
        load = QtWidgets.QPushButton("Load EEPROM to RAM")
        save.clicked.connect(self.save_eeprom)
        load.clicked.connect(self.load_eeprom)
        layout.addWidget(save)
        layout.addWidget(load)

        zero_group = QtWidgets.QGroupBox("Signed position")
        zero_layout = QtWidgets.QHBoxLayout(zero_group)
        for text, motor in [("Zero LEFT", MOTOR_LEFT), ("Zero RIGHT", MOTOR_RIGHT), ("Zero BOTH", MOTOR_BOTH)]:
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(lambda _checked=False, m=motor: self.zero_position(m))
            zero_layout.addWidget(button)
        layout.addWidget(zero_group)

        note = QtWidgets.QLabel(
            "SAVE dianggap berhasil hanya setelah firmware membaca kembali seluruh 64 word, membandingkannya byte-for-byte, dan memverifikasi CRC. Generation bertambah pada save baru."
        )
        note.setWordWrap(True)
        layout.addWidget(note)
        layout.addStretch()
        self.tabs.addTab(widget, "EEPROM")

    def _create_csv_tab(self) -> None:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        form = QtWidgets.QFormLayout()

        self.csv_path = QtWidgets.QLineEdit(str(Path.cwd() / self._default_csv_name()))
        browse = QtWidgets.QPushButton("Browse")
        browse.clicked.connect(self.choose_csv_path)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.csv_path, 1)
        row.addWidget(browse)
        form.addRow("CSV file", row)

        self.csv_button = QtWidgets.QPushButton("Start CSV recording")
        self.csv_button.clicked.connect(self.toggle_csv)
        form.addRow(self.csv_button)

        self.csv_status = QtWidgets.QLabel("STOPPED | rows=0")
        form.addRow("Status", self.csv_status)
        layout.addLayout(form)

        note = QtWidgets.QLabel(
            "CSV menyimpan setiap frame telemetry valid yang diterima, lengkap dengan timestamp host. "
            "Kolom yang tidak termasuk page saat itu dibiarkan kosong. File di-buffer dan di-flush periodik agar plotting tetap ringan."
        )
        note.setWordWrap(True)
        layout.addWidget(note)
        layout.addStretch()
        self.tabs.addTab(widget, "CSV Logger")

    # ======================================================================
    # Serial / hot-plug
    # ======================================================================
    @staticmethod
    def _port_infos():
        """Ambil daftar port dari pyserial; fallback glob tetap dipakai untuk device khusus."""
        try:
            return list(list_ports.comports())
        except Exception:
            return []

    @staticmethod
    def _identity_from_port_info(info) -> dict:
        return {
            "serial_number": getattr(info, "serial_number", None),
            "vid": getattr(info, "vid", None),
            "pid": getattr(info, "pid", None),
            "location": getattr(info, "location", None),
            "hwid": getattr(info, "hwid", None),
        }

    def _remember_port_identity(self, path: str) -> None:
        for info in self._port_infos():
            if getattr(info, "device", None) == path:
                self.preferred_port_identity = self._identity_from_port_info(info)
                return
        # Port manual/non-USB tetap dapat dipakai, hanya tanpa identity matching.
        self.preferred_port_identity = None

    def _resolve_reconnect_path(self, requested: str) -> str:
        """Cari kembali board yang sama walaupun ttyUSB berubah setelah hot-plug."""
        infos = self._port_infos()
        identity = self.preferred_port_identity
        if identity:
            serial_number = identity.get("serial_number")
            vid = identity.get("vid")
            pid = identity.get("pid")
            location = identity.get("location")

            # Serial-number adalah identitas terkuat bila USB-UART menyediakannya.
            if serial_number:
                for info in infos:
                    if (getattr(info, "serial_number", None) == serial_number and
                        getattr(info, "vid", None) == vid and getattr(info, "pid", None) == pid):
                        return info.device

            # Banyak USB-UART murah tidak punya serial-number; physical USB location
            # biasanya tetap stabil selama perangkat dipasang ke port USB yang sama.
            if location:
                for info in infos:
                    if (getattr(info, "location", None) == location and
                        getattr(info, "vid", None) == vid and getattr(info, "pid", None) == pid):
                        return info.device

        devices = {getattr(info, "device", "") for info in infos}
        if requested in devices:
            return requested
        if requested and (not requested.startswith("/dev/") or os.path.exists(requested)):
            return requested
        return requested

    def refresh_ports(self) -> None:
        current = self.port_combo.currentText().strip() if hasattr(self, "port_combo") else ""
        infos = self._port_infos()
        ports = {getattr(info, "device", "") for info in infos if getattr(info, "device", "")}
        ports.update(glob.glob("/dev/serial/by-id/*"))
        ports.update(glob.glob("/dev/ttyUSB*"))
        ports.update(glob.glob("/dev/ttyACM*"))
        ports = sorted(ports)
        if not hasattr(self, "port_combo"):
            return

        resolved = self._resolve_reconnect_path(current) if self.connection_requested else current
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if resolved and (resolved in ports or not resolved.startswith("/dev/")):
            self.port_combo.setEditText(resolved)
        elif current and not self.connection_requested:
            self.port_combo.setEditText(current)
        elif ports:
            self.port_combo.setCurrentText(ports[0])
        self.port_combo.blockSignals(False)

    def toggle_connection(self) -> None:
        if self.serial_port:
            self.connection_requested = False
            self.disconnect(send_disarm=True, keep_reconnect_request=False)
        else:
            self.connection_requested = True
            self.try_connect(force=True)

    def try_connect(self, force: bool = False) -> None:
        if self.serial_port or not self.connection_requested:
            return
        now = time.monotonic()
        if not force and now - self.last_connect_try < 1.0:
            return
        self.last_connect_try = now
        self.refresh_ports()
        requested_path = self.port_combo.currentText().strip()
        path = self._resolve_reconnect_path(requested_path)
        if path and path != requested_path:
            self.port_combo.setEditText(path)
        if not path:
            self._set_status("WAITING HOT-PLUG: pilih/masukkan port", warning=True)
            return
        try:
            port = serial.Serial(
                path, 115200, timeout=0, write_timeout=0.1,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, rtscts=False, dsrdtr=False, xonxoff=False,
            )
            port.reset_input_buffer()
            port.reset_output_buffer()
            self.serial_port = port
            self.connected_path = path
            self._remember_port_identity(path)
            self.parser = FeedbackParser()
            self.latest.clear()
            self.latest_page_data.clear()
            self.user_arm_requested = False
            self.arm_handshake_pending = False
            self.last_feedback_time = 0.0
            self.last_handshake_try = 0.0
            self.handshake_retry_count = 0
            self.connect_button.setText("Disconnect")
            self.link_detail.setText(f"PORT OPEN: {path} | waiting CRC-valid feedback")
            self._set_status("PORT OPEN - WAITING VALID FEEDBACK", warning=True)
            self.send_handshake(force=True)
        except Exception as error:
            self.serial_port = None
            self.connected_path = ""
            self._set_status(f"WAITING HOT-PLUG: {error}", warning=True)

    def disconnect(self, send_disarm: bool = False, keep_reconnect_request: bool = False) -> None:
        port = self.serial_port
        self.serial_port = None
        self.user_arm_requested = False
        self.arm_handshake_pending = False
        self.last_feedback_time = 0.0
        self.last_handshake_try = 0.0
        self.connected_path = ""
        if not keep_reconnect_request:
            self.connection_requested = False
        if port is not None:
            if send_disarm:
                try:
                    self.sequence = (self.sequence + 1) & 0xFFFF
                    cmd = Command(MSG_DISARM, sequence=self.sequence)
                    port.write(cmd.pack())
                except Exception:
                    pass
            try:
                port.close()
            except Exception:
                pass
        self.connect_button.setText("Connect")
        self.link_detail.setText("Port closed")
        self._set_status("DISCONNECTED")

    def send(self, command: Command) -> bool:
        if not self.serial_port:
            return False
        self.sequence = (self.sequence + 1) & 0xFFFF
        command.sequence = self.sequence
        try:
            packet = command.pack()
            written = self.serial_port.write(packet)
            if written != len(packet):
                raise serial.SerialTimeoutException(f"partial write {written}/{len(packet)}")
            return True
        except Exception as error:
            self.disconnect(send_disarm=False, keep_reconnect_request=True)
            self._set_status(f"SERIAL ERROR: {error} | waiting hot-plug", error=True)
            return False

    def send_handshake(self, force: bool = False) -> None:
        if not self.serial_port:
            return
        now = time.monotonic()
        if not force and now - self.last_handshake_try < 0.5:
            return
        self.last_handshake_try = now
        self.handshake_retry_count += 1
        if not self.send(Command(MSG_HELLO)):
            return
        self.send(Command(
            MSG_TELEMETRY_CONFIG,
            telemetry_page=TELEM_BASIC,
            telemetry_rate_hz=max(1, self.telemetry_rate.value()),
            loop=LOOP_NAMES[self.observe_loop.currentText()],
            telemetry_mask=0xFFFFFFFF,
        ))

    # ======================================================================
    # Runtime control
    # ======================================================================
    def update_setpoint_range(self, combo: QtWidgets.QComboBox, spin: QtWidgets.QSpinBox) -> None:
        if MODE_NAMES[combo.currentText()] == MODE_POS:
            spin.setRange(-2_000_000_000, 2_000_000_000)
        else:
            spin.setRange(-1000, 1000)

    def build_control_command(self, armed: bool, safe: bool = False) -> Command:
        mode_left = MODE_NAMES[self.mode_left.currentText()]
        mode_right = MODE_NAMES[self.mode_right.currentText()]
        left = self.setpoint_left.value()
        right = self.setpoint_right.value()
        if safe:
            left = int(self.latest.get("position_left", 0)) if mode_left == MODE_POS else 0
            right = int(self.latest.get("position_right", 0)) if mode_right == MODE_POS else 0
        return Command(
            MSG_CONTROL,
            flags=FLAG_ARM if armed else 0,
            mode_left=mode_left,
            mode_right=mode_right,
            setpoint_left=left,
            setpoint_right=right,
        )

    def request_arm(self) -> None:
        if not self.serial_port or self.last_feedback_time <= 0.0:
            self._set_status("ARM ditolak GUI: belum ada feedback valid", warning=True)
            return
        self.user_arm_requested = True
        self.arm_handshake_pending = True
        self.send(self.build_control_command(armed=True, safe=True))

    def send_disarm(self) -> None:
        self.user_arm_requested = False
        self.arm_handshake_pending = False
        self.send(Command(MSG_DISARM))

    # ======================================================================
    # PID / EEPROM
    # ======================================================================
    def update_pid_field_state(self) -> None:
        position = LOOP_NAMES[self.pid_loop.currentText()] == PID_POSITION
        for field in (self.integral_limit, self.output_min, self.output_max,
                      self.position_min, self.position_max, self.position_deadband):
            field.setEnabled(position)
        for gain in (self.kp, self.ki, self.kd):
            if position:
                gain.setDecimals(6)
                gain.setSingleStep(0.01)
            else:
                # Core FOC memang menyimpan Kp/Ki/Kd sebagai uint16 raw; jangan
                # menampilkan presisi desimal palsu yang akan dibulatkan firmware.
                gain.setDecimals(0)
                gain.setSingleStep(1.0)
        if position:
            self.pid_note.setText(
                "Position: Kp/Ki/Kd Q16.16, anti-windup conditional, integral clamp, output clamp, soft-limit, dan deadband STOP."
            )
        else:
            self.pid_note.setText(
                "Id/Iq/Speed: gain adalah nilai raw uint16 controller FOC. Integral dan anti-windup memakai jalur fixed-point asli; Kd=0 mempertahankan perilaku PI."
            )

    def _pid_expected(self, motor: int, loop: int) -> dict:
        position = loop == PID_POSITION
        return {
            "motor": motor,
            "loop": loop,
            "remaining": [MOTOR_LEFT, MOTOR_RIGHT] if motor == MOTOR_BOTH else [motor],
            "matched": [],
            "kp": float(self.kp.value()),
            "ki": float(self.ki.value()),
            "kd": float(self.kd.value()),
            "i_limit": float(self.integral_limit.value()) if position else None,
            "output_min": self.output_min.value() if position else None,
            "output_max": self.output_max.value() if position else None,
            "position_min": self.position_min.value() if position else None,
            "position_max": self.position_max.value() if position else None,
            "deadband": self.position_deadband.value() if position else None,
        }

    def send_pid(self, save: bool) -> None:
        motor = [MOTOR_LEFT, MOTOR_RIGHT, MOTOR_BOTH][self.pid_motor.currentIndex()]
        loop = LOOP_NAMES[self.pid_loop.currentText()]
        if self.position_min.value() > self.position_max.value() and loop == PID_POSITION:
            self.pid_verify_label.setText("ERROR: position_min > position_max")
            return
        self.pending_pid_verify = self._pid_expected(motor, loop)
        ok = self.send(Command(
            MSG_PID_CONFIG,
            flags=FLAG_SAVE_AFTER_APPLY if save else 0,
            motor=motor,
            loop=loop,
            kp=self.kp.value(), ki=self.ki.value(), kd=self.kd.value(),
            i_limit=self.integral_limit.value(),
            output_min=self.output_min.value(), output_max=self.output_max.value(),
            position_min=self.position_min.value(), position_max=self.position_max.value(),
            position_deadband=self.position_deadband.value(),
        ))
        if ok:
            if save:
                self.last_eeprom_status = "PID apply + EEPROM save requested; waiting STM32 read-back verification"
                self.eeprom_action_label.setText(self.last_eeprom_status)
            self.pid_verify_label.setText("Command sent; waiting read-back...")
            # Both harus diverifikasi satu motor pada satu waktu. Mulai LEFT.
            verify_motor = MOTOR_LEFT if motor == MOTOR_BOTH else motor
            QtCore.QTimer.singleShot(120, lambda m=verify_motor, l=loop: self.send(Command(MSG_REQUEST_CONFIG, motor=m, loop=l)))

    def request_pid_config(self) -> None:
        if self.pid_motor.currentIndex() == 2:
            self.pid_verify_label.setText("Read current: pilih Left atau Right.")
            return
        motor = MOTOR_LEFT if self.pid_motor.currentIndex() == 0 else MOTOR_RIGHT
        loop = LOOP_NAMES[self.pid_loop.currentText()]
        self.send(Command(MSG_REQUEST_CONFIG, motor=motor, loop=loop))

    def save_eeprom(self) -> None:
        if self.send(Command(MSG_SAVE_EEPROM)):
            self.last_eeprom_status = "SAVE requested - waiting read-back + CRC verification"
            self.eeprom_action_label.setText(self.last_eeprom_status)
            QtCore.QTimer.singleShot(150, self.request_pid_config_if_single)

    def request_pid_config_if_single(self) -> None:
        if self.pid_motor.currentIndex() != 2 and self.serial_port:
            self.request_pid_config()

    def load_eeprom(self) -> None:
        self.send_disarm()
        if self.send(Command(MSG_LOAD_EEPROM)):
            self.last_eeprom_status = "LOAD requested - waiting STM32 status"
            self.eeprom_action_label.setText(self.last_eeprom_status)
            QtCore.QTimer.singleShot(150, self.request_pid_config_if_single)

    def zero_position(self, motor: int) -> None:
        self.send_disarm()
        self.send(Command(MSG_ZERO_POSITION, motor=motor))

    def _verify_pid_readback(self, data: dict) -> None:
        """Bandingkan config STM32 dengan nilai GUI; BOTH diverifikasi LEFT lalu RIGHT."""
        if not self.pending_pid_verify:
            return
        expected = self.pending_pid_verify
        motor = data.get("config_motor")
        if motor not in expected.get("remaining", []):
            return
        if expected["loop"] != data.get("config_loop"):
            return

        position = expected["loop"] == PID_POSITION
        tol = 2.0 / 65536.0 if position else 0.1
        checks = [
            abs(data.get("config_kp", 0.0) - expected["kp"]) <= tol,
            abs(data.get("config_ki", 0.0) - expected["ki"]) <= tol,
            abs(data.get("config_kd", 0.0) - expected["kd"]) <= tol,
        ]
        if position:
            checks += [
                abs(data.get("config_i_limit", 0.0) - expected["i_limit"]) <= 0.01,
                data.get("config_output_min") == expected["output_min"],
                data.get("config_output_max") == expected["output_max"],
                data.get("config_position_min") == expected["position_min"],
                data.get("config_position_max") == expected["position_max"],
                data.get("config_position_deadband") == expected["deadband"],
            ]

        motor_name = "LEFT" if motor == MOTOR_LEFT else "RIGHT"
        if not all(checks):
            self.pid_verify_label.setText(f"READ-BACK MISMATCH {motor_name} - jangan gunakan tuning")
            self.pending_pid_verify = None
            return

        expected["remaining"].remove(motor)
        expected["matched"].append(motor_name)
        if expected["remaining"]:
            next_motor = expected["remaining"][0]
            self.pid_verify_label.setText(f"{motor_name} MATCH; checking {'LEFT' if next_motor == MOTOR_LEFT else 'RIGHT'}...")
            QtCore.QTimer.singleShot(60, lambda m=next_motor, l=expected["loop"]: self.send(Command(MSG_REQUEST_CONFIG, motor=m, loop=l)))
            return

        eeprom_ok = bool(data.get("config_eeprom_verified", 0))
        dirty = bool(data.get("config_settings_dirty", 0))
        matched = "+".join(expected["matched"])
        suffix = " | EEPROM VERIFIED" if eeprom_ok and not dirty else " | RAM MATCH; EEPROM UNSAVED/DIRTY"
        self.pid_verify_label.setText(f"READ-BACK MATCH {matched}{suffix}")
        self.pending_pid_verify = None

    # ======================================================================
    # Telemetry / plot
    # ======================================================================
    def selected_telemetry_mask(self) -> int:
        page = PAGE_NAMES[self.telemetry_page.currentText()]
        selected = {item.text() for item in self.variable_list.selectedItems()}
        mask = 0
        for index, name in enumerate(PAGE_VARIABLES[page]):
            if name in selected and index < 32:
                mask |= 1 << index
        return mask

    def send_telemetry_config(self, page: int | None = None, mask: int | None = None) -> None:
        if page is None:
            page = PAGE_NAMES[self.telemetry_page.currentText()]
        if mask is None:
            mask = self.selected_telemetry_mask()
        self.send(Command(
            MSG_TELEMETRY_CONFIG,
            telemetry_page=page,
            telemetry_rate_hz=self.telemetry_rate.value(),
            loop=LOOP_NAMES[self.observe_loop.currentText()],
            telemetry_mask=mask,
        ))

    def refresh_variables(self) -> None:
        if not hasattr(self, "variable_list"):
            return
        page = PAGE_NAMES[self.telemetry_page.currentText()]
        self.variable_list.clear()
        for name in PAGE_VARIABLES[page]:
            item = QtWidgets.QListWidgetItem(name)
            self.variable_list.addItem(item)
        for index in range(min(6, self.variable_list.count())):
            self.variable_list.item(index).setSelected(True)

    def apply_telemetry(self) -> None:
        if not self.variable_list.selectedItems() and self.variable_list.count():
            self.variable_list.item(0).setSelected(True)
        self.send_telemetry_config()
        self.clear_graph()

    def clear_graph(self) -> None:
        self.plot.clear()
        if self.plot.plotItem.legend is None:
            self.plot.addLegend()
        self.series_x.clear()
        self.series_y.clear()
        self.curves.clear()
        self.graph_start_time = time.monotonic()
        if hasattr(self, "variable_list"):
            selected_items = self.variable_list.selectedItems()
            hue_count = max(1, len(selected_items))
            for index, item in enumerate(selected_items):
                pen = pg.mkPen(pg.intColor(index, hues=hue_count), width=2)
                self.curves[item.text()] = self.plot.plot(name=item.text(), pen=pen)

    def _append_graph_samples(self, data: dict) -> None:
        requested_page = PAGE_NAMES[self.telemetry_page.currentText()]
        if data.get("page") != requested_page:
            return
        t = time.monotonic() - self.graph_start_time
        for name in self.curves:
            if name in data:
                self.series_x[name].append(t)
                self.series_y[name].append(float(data[name]))

    def _refresh_plot(self, now: float) -> None:
        if self.pause_plot.isChecked() or now - self.last_plot_refresh < 0.05:
            return
        self.last_plot_refresh = now
        window = self.plot_window.value()
        for name, curve in self.curves.items():
            xs = self.series_x[name]
            ys = self.series_y[name]
            if not xs:
                continue
            cutoff = xs[-1] - window
            start = 0
            for idx, value in enumerate(xs):
                if value >= cutoff:
                    start = idx
                    break
            curve.setData(list(xs)[start:], list(ys)[start:])
        if self.auto_range.isChecked():
            self.plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)
        else:
            self.plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=False)

    @staticmethod
    def _engineering_text(name: str, value) -> str:
        """Mengubah field ber-unit fixed integer ke tampilan manusia tanpa mengubah data raw."""
        if name == "battery_centi_volt":
            return f"{float(value) / 100.0:.2f} V"
        if name == "temperature_deci_c":
            return f"{float(value) / 10.0:.1f} °C"
        if name in ("dc_current_left_centi_amp", "dc_current_right_centi_amp"):
            return f"{float(value) / 100.0:.2f} A"
        if name == "link_age_ms":
            return f"{int(value)} ms"
        if name == "telemetry_rate_hz":
            return f"{int(value)} Hz"
        if name in ("eeprom_crc",):
            return f"0x{int(value) & 0xFFFF:04X}"
        if name in ("hall_left", "hall_right"):
            return f"0b{int(value) & 0x7:03b}"
        if "position" in name:
            return f"{int(value)} ticks"
        return ""

    def _refresh_live_table(self, now: float) -> None:
        if now - self.last_table_refresh < 0.10:
            return
        self.last_table_refresh = now
        page = PAGE_NAMES[self.telemetry_page.currentText()]
        data = self.latest_page_data.get(page, {})
        selected = {item.text() for item in self.variable_list.selectedItems()} if hasattr(self, "variable_list") else set()
        names = [name for name in PAGE_VARIABLES[page] if name in data and name in selected]
        self.live_table.setRowCount(len(names))
        for row, name in enumerate(names):
            self.live_table.setItem(row, 0, QtWidgets.QTableWidgetItem(name))
            value = data[name]
            raw_text = f"{value:.6f}" if isinstance(value, float) else str(value)
            self.live_table.setItem(row, 1, QtWidgets.QTableWidgetItem(raw_text))
            self.live_table.setItem(row, 2, QtWidgets.QTableWidgetItem(self._engineering_text(name, value)))

    # ======================================================================
    # CSV
    # ======================================================================
    @staticmethod
    def _default_csv_name() -> str:
        return datetime.now().strftime("esc_%Y%m%d_%H%M%S.csv")

    def choose_csv_path(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save ESC CSV", self.csv_path.text(), "CSV (*.csv)")
        if path:
            if not path.lower().endswith(".csv"):
                path += ".csv"
            self.csv_path.setText(path)

    def toggle_csv(self) -> None:
        if self.csv_handle is not None:
            self.stop_csv()
        else:
            self.start_csv()

    def start_csv(self) -> None:
        path = Path(self.csv_path.text()).expanduser()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            self.csv_handle = path.open("w", newline="", encoding="utf-8")
            self.csv_writer = csv.DictWriter(self.csv_handle, fieldnames=CSV_FIELDS, extrasaction="ignore")
            self.csv_writer.writeheader()
            self.csv_rows = 0
            self.csv_last_flush = time.monotonic()
            self.csv_last_ui_update = 0.0
            self.csv_button.setText("Stop CSV recording")
            self.csv_status.setText(f"RECORDING: {path}")
        except Exception as error:
            self.csv_handle = None
            self.csv_writer = None
            self.csv_status.setText(f"CSV ERROR: {error}")

    def stop_csv(self) -> None:
        if self.csv_handle is not None:
            try:
                self.csv_handle.flush()
                self.csv_handle.close()
            except Exception:
                pass
        self.csv_handle = None
        self.csv_writer = None
        self.csv_button.setText("Start CSV recording")
        self.csv_status.setText(f"STOPPED | rows={self.csv_rows}")

    def _write_csv(self, data: dict) -> None:
        if self.csv_writer is None:
            return
        row = {name: "" for name in CSV_FIELDS}
        row.update(data)
        row["host_time_iso"] = datetime.now().isoformat(timespec="milliseconds")
        row["host_monotonic_s"] = f"{time.monotonic():.6f}"
        # Simpan snapshot setting GUI pada tiap frame agar file CSV dapat dipakai
        # untuk membandingkan respons sebelum/sesudah tuning secara reproducible.
        row["gui_port"] = self.connected_path
        row["gui_mode_left"] = self.mode_left.currentText()
        row["gui_mode_right"] = self.mode_right.currentText()
        row["gui_setpoint_left"] = self.setpoint_left.value()
        row["gui_setpoint_right"] = self.setpoint_right.value()
        row["gui_pid_motor"] = self.pid_motor.currentText()
        row["gui_pid_loop"] = self.pid_loop.currentText()
        row["gui_kp"] = self.kp.value()
        row["gui_ki"] = self.ki.value()
        row["gui_kd"] = self.kd.value()
        row["gui_i_limit"] = self.integral_limit.value()
        row["gui_output_min"] = self.output_min.value()
        row["gui_output_max"] = self.output_max.value()
        row["gui_position_min"] = self.position_min.value()
        row["gui_position_max"] = self.position_max.value()
        row["gui_position_deadband"] = self.position_deadband.value()
        try:
            self.csv_writer.writerow(row)
            self.csv_rows += 1
            now = time.monotonic()
            if now - self.csv_last_flush >= 1.0:
                self.csv_handle.flush()
                self.csv_last_flush = now
            # Jangan memaksa repaint QLabel pada setiap frame 50-100 Hz. Data CSV
            # tetap ditulis setiap frame; status UI cukup diperbarui 4 Hz.
            if now - self.csv_last_ui_update >= 0.25:
                self.csv_last_ui_update = now
                self.csv_status.setText(f"RECORDING | rows={self.csv_rows} | {self.csv_path.text()}")
        except Exception as error:
            self.csv_status.setText(f"CSV ERROR: {error}")
            self.stop_csv()

    # ======================================================================
    # Main GUI service
    # ======================================================================
    def tick(self) -> None:
        now = time.monotonic()
        if now - self.last_port_refresh > 1.0 and not self.serial_port:
            self.last_port_refresh = now
            self.refresh_ports()

        if not self.serial_port:
            if self.connection_requested and self.auto_reconnect.isChecked():
                self.try_connect()
            self._refresh_plot(now)
            return

        if self.connected_path.startswith("/dev/") and not os.path.exists(self.connected_path):
            self.disconnect(send_disarm=False, keep_reconnect_request=True)
            self._set_status("HOT-UNPLUG - waiting device", warning=True)
            return

        try:
            available = self.serial_port.in_waiting
            if available:
                raw = self.serial_port.read(available)
                for data in self.parser.feed(raw):
                    self.on_feedback(data)
        except Exception as error:
            self.disconnect(send_disarm=False, keep_reconnect_request=True)
            self._set_status(f"SERIAL LOST: {error}", error=True)
            return

        if self.last_feedback_time <= 0.0:
            self.send_handshake()
            self._set_status(
                f"PORT OPEN - waiting feedback | hello #{self.handshake_retry_count} | "
                f"RX={self.parser.total_bytes} B valid={self.parser.valid_frames} invalid={self.parser.invalid_frames}",
                warning=True,
            )
            return

        silence = now - self.last_feedback_time
        if silence > 0.75:
            self.user_arm_requested = False
            self.arm_handshake_pending = False
            self.send_handshake()
            self._set_status(
                f"LINK SILENT {silence:.2f}s - resync same port | "
                f"RX={self.parser.total_bytes} B valid={self.parser.valid_frames} invalid={self.parser.invalid_frames}",
                warning=True,
            )
            return

        if now - self.last_keepalive >= 0.10:
            self.last_keepalive = now
            armed_by_stm = bool(self.latest.get("status", 0) & STATUS_ARMED)
            if self.user_arm_requested:
                if self.arm_handshake_pending and armed_by_stm:
                    self.arm_handshake_pending = False
                self.send(self.build_control_command(armed=True, safe=self.arm_handshake_pending))
            else:
                self.send(self.build_control_command(armed=False, safe=True))

        self._refresh_plot(now)
        self._refresh_live_table(now)

    def on_feedback(self, data: dict) -> None:
        self.latest.update(data)
        if data.get("page") in PAGE_VARIABLES:
            self.latest_page_data[data["page"]] = dict(data)
        self.last_feedback_time = time.monotonic()
        self.handshake_retry_count = 0

        status = data.get("status", 0)
        verified = bool(status & STATUS_EEPROM_VERIFIED)
        dirty = bool(status & STATUS_SETTINGS_DIRTY)
        self.eeprom_verified_label.setText(str(verified))
        self.eeprom_dirty_label.setText(str(dirty))
        if verified and not dirty:
            self.last_eeprom_status = "EEPROM VERIFIED: read-back dan CRC valid; RAM sama dengan flash"
            self.eeprom_action_label.setText(self.last_eeprom_status)
        elif dirty:
            self.last_eeprom_status = "RAM DIRTY: ada parameter aktif yang belum disimpan"
            self.eeprom_action_label.setText(self.last_eeprom_status)
        if "eeprom_generation" in data:
            self.eeprom_generation_label.setText(str(data["eeprom_generation"]))
            self.eeprom_crc_label.setText(f"0x{int(data.get('eeprom_crc', 0)):04X}")
            self.eeprom_fail_label.setText(str(data.get("eeprom_verify_failures", 0)))
        if "config_eeprom_generation" in data:
            self.eeprom_generation_label.setText(str(data["config_eeprom_generation"]))
            self.eeprom_crc_label.setText(f"0x{int(data.get('config_eeprom_crc', 0)):04X}")
            self.eeprom_fail_label.setText(str(data.get("config_eeprom_verify_failures", 0)))
            self.eeprom_verified_label.setText(str(bool(data.get("config_eeprom_verified", 0))))
            self.eeprom_dirty_label.setText(str(bool(data.get("config_settings_dirty", 0))))

        self.link_detail.setText(
            f"{self.connected_path} | uptime={data.get('uptime_ms', 0)} ms | "
            f"seq={data.get('sequence', 0)} | page={data.get('page', 0)} | "
            f"RX={self.parser.total_bytes} B valid={self.parser.valid_frames} invalid={self.parser.invalid_frames}"
        )
        self._set_status(
            f"LINK=OK | ARMED={bool(status & STATUS_ARMED)} | "
            f"EEPROM_OK={bool(status & STATUS_EEPROM_OK)} | EEPROM_VERIFIED={verified} | RAM_DIRTY={dirty} | "
            f"errL={data.get('error_left', 0)} errR={data.get('error_right', 0)}"
        )

        if data.get("page") == TELEM_CONFIG:
            self.kp.setValue(data.get("config_kp", 0.0))
            self.ki.setValue(data.get("config_ki", 0.0))
            self.kd.setValue(data.get("config_kd", 0.0))
            if data.get("config_loop") == PID_POSITION:
                self.integral_limit.setValue(max(0.0, data.get("config_i_limit", 0.0)))
                self.output_min.setValue(max(-1000, min(0, data.get("config_output_min", -300))))
                self.output_max.setValue(min(1000, max(0, data.get("config_output_max", 300))))
                self.position_min.setValue(data.get("config_position_min", -200000))
                self.position_max.setValue(data.get("config_position_max", 200000))
                self.position_deadband.setValue(data.get("config_position_deadband", 2))
            self._verify_pid_readback(data)

        self._append_graph_samples(data)
        self._write_csv(data)

    def _set_status(self, text: str, warning: bool = False, error: bool = False) -> None:
        self.status_bar_label.setText(text)
        palette = self.status_bar_label.palette()
        if error:
            palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor("#b00020"))
        elif warning:
            palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor("#9a6700"))
        else:
            palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor("#116611"))
        self.status_bar_label.setPalette(palette)

    def closeEvent(self, event) -> None:  # noqa: N802
        self.connection_requested = False
        self.stop_csv()
        self.disconnect(send_disarm=True, keep_reconnect_request=False)
        event.accept()


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("ESC FOC PID Tuner")
    pg.setConfigOptions(antialias=False)
    window = MainWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
