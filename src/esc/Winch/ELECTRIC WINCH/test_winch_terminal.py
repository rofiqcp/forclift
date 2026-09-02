#!/usr/bin/env python3
"""
NADIA ELECTRIC WINCH - Terminal Test
=====================================
Simple command-line test for winch.
Press 1 = UP, 2 = DOWN, 3 = STOP, 4 = STATUS, q = quit
"""

import serial
import serial.tools.list_ports
import time
import threading
import sys

# ============================================================
# FIND USB-TTL / STM32 CDC PORT
# DO NOT skip STM32 boards; USB CDC devices use VID 0x0483 and must be detected.
# ============================================================
from serial_port_utils import find_best_port


def find_ttl_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    return find_best_port(ports)

PORT = find_ttl_port()
BAUD = 115200

if not PORT:
    print("❌ USB-TTL adapter tidak ditemukan!")
    print("   Colok USB-TTL adapter ke laptop")
    print("   Cek Device Manager > Ports (COM & LPT)")
    sys.exit(1)

print(f"✅ Menggunakan port: {PORT}")

# ============================================================
# SERIAL CONNECTION
# ============================================================
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.5)  # Stabilize
    ser.reset_input_buffer()
except Exception as e:
    print(f"❌ Gagal buka port: {e}")
    sys.exit(1)

# ============================================================
# READER THREAD
# ============================================================
stop_thread = False

def reader():
    global stop_thread
    buffer = ""
    while not stop_thread:
        try:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip('\r')
                    if line:
                        print(f"  📥 RX: {line}")
            else:
                time.sleep(0.01)
        except:
            break

thread = threading.Thread(target=reader, daemon=True)
thread.start()

# ============================================================
# COMMAND LOOP
# ============================================================
def send(cmd):
    ser.write((cmd + "\n").encode())
    ser.flush()
    print(f"  📤 TX: {cmd}")

print("\n" + "="*50)
print("  NADIA ELECTRIC WINCH - TERMINAL TEST")
print("="*50)
print(f"  Port: {PORT} @ {BAUD}")
print("  Wiring: PA9=TX → USB-TTL RX, PA10=RX → USB-TTL TX, GND=GND")
print("\n  COMMANDS:")
print("    1 / UP      : Winch NAIK")
print("    2 / DOWN    : Winch TURUN")
print("    3 / STOP    : STOP darurat")
print("    4 / STATUS  : Cek status")
print("    q / quit    : Keluar")
print("="*50)
print("  AUTO-RETURN: Saat LS ATAS (PB6) ketrigger → otomatis TURUN ke LS BAWAH")
print("="*50 + "\n")

# Initial status
send("STATUS")

try:
    while True:
        cmd = input("  > ").strip().upper()
        
        if cmd in ('1', 'UP'):
            send("UP")
        elif cmd in ('2', 'DOWN'):
            send("DOWN")
        elif cmd in ('3', 'STOP'):
            send("STOP")
        elif cmd in ('4', 'STATUS'):
            send("STATUS")
        elif cmd in ('Q', 'QUIT', 'EXIT'):
            send("STOP")  # Safety stop before quit
            time.sleep(0.2)
            break
        elif cmd == '':
            continue
        else:
            print(f"  ❓ Unknown: {cmd}")
            
except KeyboardInterrupt:
    print("\n  Ctrl+C - Stopping...")
    send("STOP")
    time.sleep(0.2)

finally:
    stop_thread = True
    ser.close()
    print("  👋 Selesai")