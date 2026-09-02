#!/usr/bin/env python3
"""
=====================================================================
 ESC USART3 TESTER - VLT / TRQ / SPD / POS
=====================================================================

Program test terminal untuk firmware ESC board 0 + USART3.

Tujuan:
1. Membuka /dev/ttyUSBx atau /dev/serial/by-id/... pada 115200 baud.
2. Mengirim HELLO dan meminta telemetry BASIC.
3. Memastikan frame 64-byte + CRC16 dari STM32 benar-benar diterima.
4. Menguji mode runtime VLT / TRQ / SPD / POS tanpa GUI.
5. Menampilkan signed wheel position, speed, battery, temperature, current,
   command, link age, status ARMED, dan error motor.
6. Mengirim DISARM beberapa kali ketika Ctrl+C atau program berakhir.

Keselamatan:
- Default program TIDAK ARM motor.
- Tambahkan --arm jika memang ingin mengaktifkan output motor.
- Untuk mode POS, program terlebih dahulu menggunakan posisi aktual sebagai
  safe setpoint sebelum mengirim target pengguna.
- Lakukan pengujian awal dengan roda terangkat dari lantai.

Contoh:
    python3 esc_serial_test.py --port /dev/ttyUSB0
    python3 esc_serial_test.py --port /dev/ttyUSB0 --mode SPD --left 100 --right 100 --arm
    python3 esc_serial_test.py --port /dev/ttyUSB0 --mode SPD --triangle 100 --arm
    python3 esc_serial_test.py --port /dev/ttyUSB0 --left-mode POS --left 1200 --right-mode OPEN --arm
=====================================================================
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial

# protocol.py berada di folder Interface yang sama.
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from protocol import (  # noqa: E402
    Command,
    FeedbackParser,
    FLAG_ARM,
    MODE_OPEN,
    MODE_POS,
    MODE_SPD,
    MODE_TRQ,
    MODE_VLT,
    MSG_CONTROL,
    MSG_DISARM,
    MSG_HELLO,
    MSG_TELEMETRY_CONFIG,
    STATUS_ARMED,
    STATUS_EEPROM_OK,
    STATUS_EEPROM_VERIFIED,
    STATUS_SETTINGS_DIRTY,
    STATUS_LINK_OK,
    TELEM_BASIC,
)


# =====================================================================
# KONFIGURASI DEFAULT
# =====================================================================

SERIAL_BAUD = 115200
SEND_INTERVAL_S = 0.100
HANDSHAKE_INTERVAL_S = 0.500
FEEDBACK_WAIT_S = 5.0

MODE_BY_NAME = {
    "OPEN": MODE_OPEN,
    "VLT": MODE_VLT,
    "TRQ": MODE_TRQ,
    "SPD": MODE_SPD,
    "POS": MODE_POS,
}


# =====================================================================
# HELPER ARGUMENT
# =====================================================================

def parse_args() -> argparse.Namespace:
    """Membaca opsi command-line agar satu tester dapat dipakai untuk semua mode."""
    parser = argparse.ArgumentParser(
        description="Tester terminal ESC USART3 64-byte CRC16"
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Port serial ESC")
    parser.add_argument("--mode", choices=MODE_BY_NAME, help="Mode yang diterapkan ke kedua motor")
    parser.add_argument("--left-mode", choices=MODE_BY_NAME, default="SPD")
    parser.add_argument("--right-mode", choices=MODE_BY_NAME, default="SPD")
    parser.add_argument("--left", type=int, default=0, help="Setpoint motor kiri")
    parser.add_argument("--right", type=int, default=0, help="Setpoint motor kanan")
    parser.add_argument(
        "--triangle",
        type=int,
        default=0,
        metavar="AMPLITUDE",
        help="Buat test triangular -AMPLITUDE..+AMPLITUDE untuk VLT/TRQ/SPD",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=2,
        help="Kenaikan tiap 100 ms pada triangular test (default 2)",
    )
    parser.add_argument("--arm", action="store_true", help="Izinkan motor ARM")
    parser.add_argument("--raw-rx", action="store_true", help="Cetak byte RX mentah dalam hex")
    parser.add_argument("--rate", type=int, default=20, help="Telemetry rate 1..100 Hz")
    return parser.parse_args()


# =====================================================================
# CLASS SERIAL TEST
# =====================================================================

class EscSerialTester:
    """Menangani open, TX command, parser RX, safe arm, dan safe disarm."""

    def __init__(self, port: str, baud: int = SERIAL_BAUD, raw_rx: bool = False) -> None:
        self.port_name = port
        self.baud = baud
        self.raw_rx = raw_rx
        self.port: serial.Serial | None = None
        self.parser = FeedbackParser()
        self.sequence = 0
        self.latest: dict = {}
        self.valid_feedback_count = 0
        self.total_rx_bytes = 0

    def open(self) -> None:
        """Membuka serial port 8N1 tanpa flow control dan membersihkan buffer lama."""
        print("=" * 78)
        print("ESC USART3 TESTER")
        print("=" * 78)
        print(f"Port     : {self.port_name}")
        print(f"Baudrate : {self.baud}")

        self.port = serial.Serial(
            self.port_name,
            self.baud,
            timeout=0,
            write_timeout=0.1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()
        print("Port     : OPEN")
        print("=" * 78)

    def close(self) -> None:
        """Menutup serial port bila masih terbuka."""
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
            self.port = None

    def send(self, command: Command) -> None:
        """Memberi sequence baru lalu mengirim tepat satu frame 64 byte."""
        if self.port is None:
            raise RuntimeError("serial belum dibuka")
        self.sequence = (self.sequence + 1) & 0xFFFF
        command.sequence = self.sequence
        packet = command.pack()
        written = self.port.write(packet)
        if written != len(packet):
            raise serial.SerialTimeoutException(
                f"partial write: {written}/{len(packet)} byte"
            )

    def receive(self) -> list[dict]:
        """Membaca seluruh byte tersedia lalu mengembalikan frame CRC-valid."""
        if self.port is None:
            return []
        waiting = self.port.in_waiting
        if waiting <= 0:
            return []

        raw = self.port.read(waiting)
        self.total_rx_bytes += len(raw)
        if self.raw_rx:
            print("RX RAW:", " ".join(f"{byte:02X}" for byte in raw))

        frames = self.parser.feed(raw)
        for frame in frames:
            self.latest.update(frame)
            self.valid_feedback_count += 1
        return frames

    def handshake(self, telemetry_rate_hz: int) -> bool:
        """Mengulang HELLO pada port yang sama sampai ada feedback valid atau timeout."""
        telemetry_rate_hz = max(1, min(100, int(telemetry_rate_hz)))
        deadline = time.monotonic() + FEEDBACK_WAIT_S
        next_hello = 0.0

        print("Menunggu feedback CRC-valid dari STM32...")
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_hello:
                self.send(Command(MSG_HELLO))
                self.send(
                    Command(
                        MSG_TELEMETRY_CONFIG,
                        telemetry_page=TELEM_BASIC,
                        telemetry_rate_hz=telemetry_rate_hz,
                        telemetry_mask=0xFFFFFFFF,
                    )
                )
                next_hello = now + HANDSHAKE_INTERVAL_S

            frames = self.receive()
            if frames:
                print(
                    f"HANDSHAKE OK | valid frame={self.valid_feedback_count} "
                    f"| RX={self.total_rx_bytes} byte"
                )
                return True
            time.sleep(0.005)

        print(
            "HANDSHAKE GAGAL: tidak ada frame telemetry CRC-valid selama "
            f"{FEEDBACK_WAIT_S:.1f} s. RX total={self.total_rx_bytes} byte."
        )
        if self.total_rx_bytes == 0:
            print("Kemungkinan: TX STM32/USART3/wiring/firmware belum mengirim data.")
        else:
            print(
                "Ada byte masuk tetapi tidak menjadi frame valid. Periksa versi protokol, "
                "CRC, atau firmware yang ter-flash."
            )
        return False

    def safe_disarm(self) -> None:
        """Mengirim DISARM tiga kali sebelum port ditutup."""
        if self.port is None:
            return
        for _ in range(3):
            try:
                self.send(Command(MSG_DISARM))
                time.sleep(0.03)
            except Exception:
                break

    def print_basic(self, frame: dict) -> None:
        """Menampilkan telemetry BASIC dalam satu baris agar mudah diamati."""
        status = frame.get("status", 0)
        print(
            f"SEQ:{frame.get('sequence', 0):5d} | "
            f"LINK:{'OK' if status & STATUS_LINK_OK else '--'} | "
            f"ARM:{'ON' if status & STATUS_ARMED else 'OFF'} | "
            f"EEP:{'VER' if status & STATUS_EEPROM_VERIFIED else ('OK' if status & STATUS_EEPROM_OK else '--')} | "
            f"DIRTY:{'YES' if status & STATUS_SETTINGS_DIRTY else 'NO'} | "
            f"Mode L/R:{frame.get('mode_left', 0)}/{frame.get('mode_right', 0)} | "
            f"SP L/R:{frame.get('setpoint_left', 0):8d}/{frame.get('setpoint_right', 0):8d} | "
            f"Speed L/R:{frame.get('speed_left', 0):6d}/{frame.get('speed_right', 0):6d} | "
            f"Pos L/R:{frame.get('position_left', 0):10d}/{frame.get('position_right', 0):10d} | "
            f"I L/R:{frame.get('dc_current_left_centi_amp', 0):6d}/{frame.get('dc_current_right_centi_amp', 0):6d} | "
            f"Bat:{frame.get('battery_centi_volt', 0):5d} | "
            f"Temp:{frame.get('temperature_deci_c', 0):5d} | "
            f"Err:{frame.get('error_left', 0)}/{frame.get('error_right', 0)}"
        )


# =====================================================================
# TEST SIGNAL
# =====================================================================

def triangle_next(value: int, direction: int, amplitude: int, step: int) -> tuple[int, int]:
    """Menghasilkan triangular signal signed tanpa overflow/wrap counter."""
    amplitude = abs(amplitude)
    step = max(1, abs(step))
    value += direction * step
    if value >= amplitude:
        value = amplitude
        direction = -1
    elif value <= -amplitude:
        value = -amplitude
        direction = 1
    return value, direction


# =====================================================================
# MAIN
# =====================================================================

def main() -> int:
    """Menjalankan handshake, optional safe-arm, keepalive, dan print telemetry."""
    args = parse_args()
    if args.mode:
        args.left_mode = args.mode
        args.right_mode = args.mode

    left_mode = MODE_BY_NAME[args.left_mode]
    right_mode = MODE_BY_NAME[args.right_mode]

    if args.triangle and (left_mode == MODE_POS or right_mode == MODE_POS):
        print("[ERROR] --triangle hanya untuk VLT/TRQ/SPD, bukan POS.")
        return 2

    tester = EscSerialTester(args.port, raw_rx=args.raw_rx)
    try:
        tester.open()
        if not tester.handshake(args.rate):
            return 3

        # Tahap safe-arm. POS memakai posisi aktual; mode lain memakai nol.
        safe_left = tester.latest.get("position_left", 0) if left_mode == MODE_POS else 0
        safe_right = tester.latest.get("position_right", 0) if right_mode == MODE_POS else 0

        if args.arm:
            print("ARM diminta: mengirim safe setpoint terlebih dahulu...")
            arm_deadline = time.monotonic() + 2.0
            while time.monotonic() < arm_deadline:
                tester.send(
                    Command(
                        MSG_CONTROL,
                        flags=FLAG_ARM,
                        mode_left=left_mode,
                        mode_right=right_mode,
                        setpoint_left=safe_left,
                        setpoint_right=safe_right,
                    )
                )
                for frame in tester.receive():
                    tester.print_basic(frame)
                if tester.latest.get("status", 0) & STATUS_ARMED:
                    print("ARMED dikonfirmasi STM32.")
                    break
                time.sleep(SEND_INTERVAL_S)
            else:
                print("[ERROR] STM32 tidak mengonfirmasi ARMED. Tetap DISARM.")
                return 4
        else:
            print("Mode monitor saja: --arm tidak diberikan, MOSFET tetap DISARM.")

        left_target = int(args.left)
        right_target = int(args.right)
        triangle_value = 0
        triangle_direction = 1
        next_send = 0.0

        while True:
            now = time.monotonic()
            if now >= next_send:
                next_send = now + SEND_INTERVAL_S

                if args.triangle:
                    triangle_value, triangle_direction = triangle_next(
                        triangle_value,
                        triangle_direction,
                        args.triangle,
                        args.step,
                    )
                    left_target = triangle_value
                    right_target = triangle_value

                tester.send(
                    Command(
                        MSG_CONTROL,
                        flags=FLAG_ARM if args.arm else 0,
                        mode_left=left_mode,
                        mode_right=right_mode,
                        setpoint_left=left_target if args.arm else safe_left,
                        setpoint_right=right_target if args.arm else safe_right,
                    )
                )

            frames = tester.receive()
            for frame in frames:
                tester.print_basic(frame)

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\nCTRL+C diterima.")
    except (serial.SerialException, OSError) as error:
        print(f"[SERIAL ERROR] {error}")
        return 5
    finally:
        print("Mengirim DISARM...")
        tester.safe_disarm()
        tester.close()
        print("Program selesai.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
