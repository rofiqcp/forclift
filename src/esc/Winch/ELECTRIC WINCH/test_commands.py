#!/usr/bin/env python3
"""Interactive terminal controller for the electric winch."""

import sys
import time
import serial
import serial.tools.list_ports

from serial_port_utils import find_best_port

BAUD = 115200


def detect_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    preferred = find_best_port(ports)
    if preferred:
        return preferred
    if len(ports) == 1:
        return ports[0].device
    print("Available ports:")
    for i, port in enumerate(ports, start=1):
        print(f"  {i}. {port.device}")
    choice = input("Select port [1]: ").strip() or "1"
    try:
        idx = int(choice) - 1
        return ports[idx].device
    except Exception:
        return ports[0].device


def read_serial_lines(ser, wait_seconds=0.5):
    end = time.time() + wait_seconds
    lines = []
    while time.time() < end:
        try:
            if ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    lines.append(line)
                    print(f"[RX] {line}")
        except Exception:
            pass
        time.sleep(0.05)
    return lines


def send_command(ser, cmd):
    print(f"[TX] {cmd}")
    ser.write((cmd + "\n").encode())
    ser.flush()
    read_serial_lines(ser, wait_seconds=0.8)


def monitor_loop(ser):
    print("\nMonitoring serial. Press Ctrl+C to exit.")
    try:
        while True:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                except Exception:
                    continue
                if not line:
                    continue
                print(f"[RX] {line}")
                if line.startswith("TOP:"):
                    value = line.split(":", 1)[1].strip()
                    if value == "1":
                        print("[INFO] LS TOP triggered: winch will move DOWN automatically.")
                        time.sleep(0.2)
                        ser.write(b"DOWN\n")
                        ser.flush()
                        print("[TX] DOWN")
                elif line.startswith("BOTTOM:"):
                    value = line.split(":", 1)[1].strip()
                    if value == "1":
                        print("[INFO] LS BOTTOM triggered.")
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[INFO] Monitoring stopped.")


def main():
    print("=" * 60)
    print("ELECTRIC WINCH - TERMINAL CONTROL")
    print("=" * 60)
    port = detect_port() or "COM3"
    print(f"Using port: {port}")

    try:
        ser = serial.Serial(port, BAUD, timeout=0.2)
        time.sleep(0.5)
        print("[INFO] Connected.")
    except Exception as e:
        print(f"[ERROR] Failed to open {port}: {e}")
        print("[INFO] Try one of the COM ports shown above or set port manually in the script.")
        sys.exit(1)

    try:
        while True:
            print("\nMenu:")
            print("  1. UP")
            print("  2. DOWN")
            print("  3. STOP")
            print("  4. STATUS")
            print("  5. MONITOR")
            print("  6. EXIT")
            choice = input("Choose: ").strip()

            if choice == "1":
                send_command(ser, "UP")
            elif choice == "2":
                send_command(ser, "DOWN")
            elif choice == "3":
                send_command(ser, "STOP")
            elif choice == "4":
                send_command(ser, "STATUS")
            elif choice == "5":
                monitor_loop(ser)
                break
            elif choice == "6":
                print("[INFO] Closing connection.")
                break
            else:
                print("[INFO] Invalid choice.")
    except KeyboardInterrupt:
        print("\n[INFO] User interrupted.")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        print("[INFO] Port closed.")


if __name__ == "__main__":
    main()
