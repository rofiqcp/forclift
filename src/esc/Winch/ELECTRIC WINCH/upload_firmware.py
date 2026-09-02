#!/usr/bin/env python3
"""
STM32F411CEU6 Firmware Upload Helper
Handles bootloader activation and upload via stm32flash
"""

import serial
import time
import subprocess
import sys
import os

def enter_bootloader_mode(com_port="COM3", baud=115200):
    """
    Try to enter bootloader mode on STM32F411
    """
    try:
        ser = serial.Serial(com_port, baud, timeout=1)
        time.sleep(0.5)
        
        # Try to trigger bootloader by sending a specific sequence
        # This works with WeAct Black Pill - toggle DTR/RTS
        print(f"[*] Attempting to put {com_port} into bootloader mode...")
        
        # Set DTR low, RTS high (enter bootloader)
        ser.dtr = False
        ser.rts = True
        time.sleep(0.1)
        
        # Toggle reset
        ser.dtr = True
        time.sleep(0.1)
        ser.dtr = False
        time.sleep(0.5)
        
        ser.close()
        print("[+] Bootloader mode sequence sent")
        time.sleep(1)
        return True
        
    except Exception as e:
        print(f"[-] Error entering bootloader mode: {e}")
        return False

def upload_with_stm32flash(com_port="COM3", baud=115200, binary_path=".pio/build/blackpill_f411ce/firmware.bin"):
    """
    Upload firmware using stm32flash
    """
    if not os.path.exists(binary_path):
        print(f"[-] Binary file not found: {binary_path}")
        return False
    
    print(f"[*] Uploading {binary_path} to {com_port}...")
    
    # Use stm32flash via python -m (comes with stm32duino package)
    try:
        # Try using stm32flash directly
        cmd = [
            "stm32flash",
            "-b", str(baud),
            "-w", binary_path,
            com_port
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        
        if result.returncode == 0:
            print("[+] Upload successful!")
            print(result.stdout)
            return True
        else:
            print("[-] Upload failed!")
            print(result.stderr)
            print(result.stdout)
            return False
            
    except FileNotFoundError:
        print("[-] stm32flash not found in PATH")
        return False
    except subprocess.TimeoutExpired:
        print("[-] Upload timeout")
        return False
    except Exception as e:
        print(f"[-] Error: {e}")
        return False

def main():
    com_port = "COM3"
    baud = 115200
    
    print("===============================================")
    print("STM32F411CEU6 Firmware Upload Helper")
    print("===============================================\n")
    
    # Step 1: Enter bootloader
    if not enter_bootloader_mode(com_port, baud):
        print("[-] Failed to enter bootloader mode")
        sys.exit(1)
    
    # Step 2: Upload firmware
    if not upload_with_stm32flash(com_port, baud):
        print("[-] Upload failed")
        sys.exit(1)
    
    print("\n[+] Firmware upload complete!")
    print("[*] Device will restart automatically...")
    time.sleep(2)
    
    # Wait for device to restart
    time.sleep(2)
    
    print("[+] Done!")

if __name__ == "__main__":
    main()
