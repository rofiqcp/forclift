#!/usr/bin/env python3
"""
Test serial connection to STM32F411 and check current firmware state
"""

import serial
import time

def test_device(port="COM3", baud=115200):
    """Test device connection and get firmware info"""
    print(f"[*] Connecting to {port} at {baud} baud...")
    
    try:
        ser = serial.Serial(port, baud, timeout=2)
        time.sleep(1)  # Wait for device
        
        print("[+] Connected!")
        
        # Clear any buffered data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Send STATUS command
        print("[*] Sending STATUS command...")
        ser.write(b"STATUS\n")
        
        # Read response with timeout
        ser.timeout = 1
        responses = []
        start_time = time.time()
        
        while time.time() - start_time < 2:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[RX] {line}")
                    responses.append(line)
            time.sleep(0.05)
        
        if not responses:
            print("[-] No response from device")
            print("[*] Device might be in bootloader mode or needs upload")
        else:
            print("\n[+] Device is responding with firmware!")
            print("[+] Ready to test GUI commands")
        
        ser.close()
        return len(responses) > 0
        
    except serial.SerialException as e:
        print(f"[-] Serial error: {e}")
        return False
    except Exception as e:
        print(f"[-] Error: {e}")
        return False

if __name__ == "__main__":
    test_device()
