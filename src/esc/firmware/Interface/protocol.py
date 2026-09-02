#!/usr/bin/env python3
"""Protokol biner 64-byte ESC STM32 melalui USART3.

File ini sengaja tidak bergantung pada PyQt sehingga dapat digunakan juga untuk
script pengujian otomatis atau logger serial sederhana.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

START = 0xABCD
VERSION = 4
FRAME_SIZE = 64

# Jenis pesan host -> STM32.
MSG_HELLO = 1
MSG_CONTROL = 2
MSG_PID_CONFIG = 3
MSG_TELEMETRY_CONFIG = 4
MSG_SAVE_EEPROM = 5
MSG_LOAD_EEPROM = 6
MSG_ZERO_POSITION = 7
MSG_DISARM = 8
MSG_REQUEST_CONFIG = 9
MSG_TELEMETRY = 0x80

# Pemilihan motor.
MOTOR_LEFT = 0
MOTOR_RIGHT = 1
MOTOR_BOTH = 2

# Mode kontrol runtime.
MODE_OPEN = 0
MODE_VLT = 1
MODE_SPD = 2
MODE_TRQ = 3
MODE_POS = 4

# Loop PID. TRQ dan Iq adalah loop yang sama pada algoritma FOC asli.
PID_CURRENT_D = 0
PID_TORQUE_Q = 1
PID_SPEED = 2
PID_POSITION = 3

# Halaman telemetry.
TELEM_BASIC = 0
TELEM_FOC = 1
TELEM_PID = 2
TELEM_RAW = 3
TELEM_CONFIG = 4

FLAG_ARM = 1
FLAG_SAVE_AFTER_APPLY = 2

STATUS_LINK_OK = 1 << 0
STATUS_ARMED = 1 << 1
STATUS_EEPROM_OK = 1 << 2
STATUS_TIMEOUT = 1 << 3
STATUS_EEPROM_VERIFIED = 1 << 4
STATUS_SETTINGS_DIRTY = 1 << 5

COMMAND_FORMAT = "<HBBHBBBBBBHiiiiiiiiiiIHHH"
FEEDBACK_FORMAT = "<HBBHBBIBBBB46sH"
assert struct.calcsize(COMMAND_FORMAT) == FRAME_SIZE
assert struct.calcsize(FEEDBACK_FORMAT) == FRAME_SIZE


def crc16(data: bytes) -> int:
    """Menghitung CRC16-CCITT-FALSE yang sama dengan firmware STM32."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def q16(value: float) -> int:
    """Mengubah float host menjadi signed Q16.16 untuk frame command."""
    raw = int(round(value * 65536.0))
    return max(-0x80000000, min(0x7FFFFFFF, raw))


def from_q16(value: int) -> float:
    """Mengubah signed Q16.16 dari STM32 menjadi float host."""
    return value / 65536.0


@dataclass
class Command:
    """Representasi satu frame command 64 byte."""

    type: int
    sequence: int = 0
    flags: int = 0
    motor: int = MOTOR_BOTH
    mode_left: int = MODE_OPEN
    mode_right: int = MODE_OPEN
    loop: int = PID_SPEED
    telemetry_page: int = TELEM_BASIC
    telemetry_rate_hz: int = 50
    setpoint_left: int = 0
    setpoint_right: int = 0
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    i_limit: float = 0.0
    output_min: int = -1000
    output_max: int = 1000
    position_min: int = -200000
    position_max: int = 200000
    telemetry_mask: int = 0xFFFFFFFF
    position_deadband: int = 2

    def pack(self) -> bytes:
        """Mengemas field command dan menambahkan CRC di dua byte terakhir."""
        values = [
            START,
            VERSION,
            self.type,
            self.sequence & 0xFFFF,
            self.flags,
            self.motor,
            self.mode_left,
            self.mode_right,
            self.loop,
            self.telemetry_page,
            self.telemetry_rate_hz,
            self.setpoint_left,
            self.setpoint_right,
            q16(self.kp),
            q16(self.ki),
            q16(self.kd),
            q16(self.i_limit),
            self.output_min,
            self.output_max,
            self.position_min,
            self.position_max,
            self.telemetry_mask & 0xFFFFFFFF,
            0,
            max(0, min(1000, int(self.position_deadband))),
            0,
        ]
        raw = bytearray(struct.pack(COMMAND_FORMAT, *values))
        struct.pack_into("<H", raw, 62, crc16(raw[:-2]))
        return bytes(raw)


def unpack_feedback(raw: bytes) -> dict | None:
    """Memvalidasi CRC lalu mengubah feedback menjadi dictionary mudah dipakai GUI."""
    if len(raw) != FRAME_SIZE:
        return None
    if crc16(raw[:-2]) != struct.unpack_from("<H", raw, 62)[0]:
        return None

    (
        start,
        version,
        msg_type,
        sequence,
        page,
        status,
        uptime_ms,
        mode_left,
        mode_right,
        error_left,
        error_right,
        payload,
        _checksum,
    ) = struct.unpack(FEEDBACK_FORMAT, raw)

    if start != START or version != VERSION or msg_type != MSG_TELEMETRY:
        return None

    result = {
        "sequence": sequence,
        "page": page,
        "status": status,
        "uptime_ms": uptime_ms,
        "mode_left": mode_left,
        "mode_right": mode_right,
        "error_left": error_left,
        "error_right": error_right,
    }

    if page == TELEM_BASIC:
        values = struct.unpack_from("<iiiihhhhhhhhHH", payload)
        keys = [
            "position_left", "position_right", "setpoint_left", "setpoint_right",
            "speed_left", "speed_right", "battery_centi_volt", "temperature_deci_c",
            "dc_current_left_centi_amp", "dc_current_right_centi_amp",
            "command_left", "command_right", "link_age_ms", "telemetry_rate_hz",
        ]
        result.update(dict(zip(keys, values)))

    elif page == TELEM_FOC:
        values = struct.unpack_from("<hhhhhhhhhhhhhhhhhhhhH", payload)
        keys = [
            "id_left", "iq_left", "id_right", "iq_right",
            "phase_a_left", "phase_b_left", "phase_b_right", "phase_c_right",
            "dc_link_left", "dc_link_right",
            "duty_u_left", "duty_v_left", "duty_w_left",
            "duty_u_right", "duty_v_right", "duty_w_right",
            "electrical_angle_left", "electrical_angle_right",
            "speed_left", "speed_right", "hall_bits",
        ]
        result.update(dict(zip(keys, values)))

    elif page == TELEM_PID:
        values = struct.unpack_from("<BBiiiiiihhhhhhhhH", payload)
        keys = [
            "pid_loop", "reserved", "setpoint_left", "setpoint_right",
            "measured_left", "measured_right", "error_left_pid", "error_right_pid",
            "p_left", "i_left", "d_left", "output_left",
            "p_right", "i_right", "d_right", "output_right", "antiwindup_flags",
        ]
        result.update(dict(zip(keys, values)))

    elif page == TELEM_RAW:
        values = struct.unpack_from("<hhhhhhhhBBHIIIIhhHHH", payload)
        keys = [
            "adc_phase_a_left", "adc_phase_b_left", "adc_phase_b_right", "adc_phase_c_right",
            "adc_dc_left", "adc_dc_right", "adc_battery", "adc_temperature",
            "hall_left", "hall_right", "pwm_period", "main_loop_counter",
            "valid_frames", "bad_frames", "reconnect_counter",
            "core_speed_left", "core_speed_right",
            "eeprom_crc", "eeprom_generation", "eeprom_verify_failures",
        ]
        result.update(dict(zip(keys, values)))

    elif page == TELEM_CONFIG:
        values = struct.unpack_from("<BBiiiiiiiiHHHHBBBB", payload)
        keys = [
            "config_motor", "config_loop", "config_kp_q16", "config_ki_q16",
            "config_kd_q16", "config_i_limit_q16", "config_output_min",
            "config_output_max", "config_position_min", "config_position_max",
            "config_position_deadband", "config_eeprom_crc", "config_eeprom_generation",
            "config_eeprom_verify_failures", "config_eeprom_verified", "config_settings_dirty",
            "config_reserved2", "config_reserved3",
        ]
        result.update(dict(zip(keys, values)))
        result["config_kp"] = from_q16(result["config_kp_q16"])
        result["config_ki"] = from_q16(result["config_ki_q16"])
        result["config_kd"] = from_q16(result["config_kd_q16"])
        result["config_i_limit"] = from_q16(result["config_i_limit_q16"])

    return result


class FeedbackParser:
    """Parser streaming yang dapat pulih kembali setelah byte hilang/corrupt."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.total_bytes = 0
        self.valid_frames = 0
        self.invalid_frames = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[dict]:
        """Memasukkan byte baru dan mengembalikan semua frame valid yang ditemukan."""
        self.total_bytes += len(data)
        self.buffer.extend(data)
        parsed_frames: list[dict] = []
        marker = b"\xCD\xAB"

        while len(self.buffer) >= 2:
            index = self.buffer.find(marker)
            if index < 0:
                discarded = max(0, len(self.buffer) - 1)
                self.discarded_bytes += discarded
                self.buffer[:] = self.buffer[-1:]
                break

            if index > 0:
                self.discarded_bytes += index
                del self.buffer[:index]

            if len(self.buffer) < FRAME_SIZE:
                break

            frame = bytes(self.buffer[:FRAME_SIZE])
            parsed = unpack_feedback(frame)
            if parsed is not None:
                self.valid_frames += 1
                parsed_frames.append(parsed)
                del self.buffer[:FRAME_SIZE]
            else:
                # CRC/header salah: geser satu byte agar START berikutnya tetap dapat ditemukan.
                self.invalid_frames += 1
                self.discarded_bytes += 1
                del self.buffer[0]

        return parsed_frames

