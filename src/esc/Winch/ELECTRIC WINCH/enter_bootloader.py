#!/usr/bin/env python3
"""
Try to enter STM32 bootloader mode via serial sequence
The WeAct Black Pill V2 can enter bootloader via DTR/RTS toggle
"""

import serial
import time

def try_bootloader_entry(port="COM3"):
    """
    Try various methods to enter bootloader mode
    """
    print("=" * 60)
    print("STM32F411 Bootloader Entry Attempts")
    print("=" * 60)
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(0.5)
        
        # Method 1: DTR/RTS toggle (WeAct Black Pill V2)
        print("\n[1] Trying DTR/RTS toggle (WeAct Black Pill V2)...")
        print("    Setting RTS=HIGH, DTR=LOW...")
        ser.rts = True
        ser.dtr = False
        time.sleep(0.1)
        print("    Toggling DTR...")
        ser.dtr = True
        time.sleep(0.1)
        ser.dtr = False
        time.sleep(0.5)
        
        # Try to sync
        print("[*] Sending bootloader sync byte (0x7F)...")
        ser.write(bytes([0x7F]))
        time.sleep(0.2)
        
        if ser.in_waiting:
            resp = ser.read()
            if resp[0] == 0x79:
                print("[+] SUCCESS! Bootloader is responding!")
                print("[+] Device is now in bootloader mode")
                ser.close()
                return True
        
        # Method 2: Send 0x7F multiple times
        print("\n[2] Trying multiple sync bytes...")
        for i in range(10):
            ser.write(bytes([0x7F]))
            time.sleep(0.05)
            
            if ser.in_waiting:
                resp = ser.read()
                if resp[0] == 0x79:
                    print(f"[+] SUCCESS after attempt {i+1}!")
                    ser.close()
                    return True
        
        # Method 3: Reset via RTS toggle
        print("\n[3] Trying RTS reset pulse...")
        ser.rts = False
        time.sleep(0.1)
        ser.rts = True
        time.sleep(0.5)
        
        ser.write(bytes([0x7F]))
        time.sleep(0.2)
        
        if ser.in_waiting:
            resp = ser.read()
            if resp[0] == 0x79:
                print("[+] SUCCESS! Device responded to bootloader")
                ser.close()
                return True
        
        # Method 4: Try lower baud rate
        ser.close()
        print("\n[4] Trying different baud rates...")
        for baud in [9600, 19200, 38400, 57600]:
            print(f"    Trying {baud} baud...")
            try:
                ser = serial.Serial(port, baud, timeout=0.1)
                time.sleep(0.3)
                
                ser.write(bytes([0x7F]))
                time.sleep(0.1)
                
                if ser.in_waiting:
                    resp = ser.read()
                    if resp[0] == 0x79:
                        print(f"[+] SUCCESS at {baud} baud!")
                        ser.close()
                        return True
                
                ser.close()
                time.sleep(0.2)
            except:
                pass
        
        print("\n[-] Could not enter bootloader mode")
        print("[!] Manual action required:")
        print("    1. Hold down BOOT0 button")
        print("    2. Press RESET button (while holding BOOT0)")
        print("    3. Release BOOT0")
        print("    4. Run stm32_upload.py again")
        
        return False
        
    except Exception as e:
        print(f"[-] Error: {e}")
        return False

if __name__ == "__main__":
    try_bootloader_entry()
