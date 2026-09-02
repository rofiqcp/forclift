#!/usr/bin/env python3
"""
STM32F411 Automatic Firmware Upload Helper
Waits for user to enter bootloader mode, then automatically uploads
"""

import serial
import time
import os
import sys
import subprocess

def wait_for_bootloader(port="COM3", timeout=60):
    """
    Wait for bootloader to be active on COM port
    """
    print("=" * 70)
    print("STM32F411 Automatic Firmware Upload")
    print("=" * 70)
    
    print("\n📋 INSTRUCTIONS:\n")
    print("1. Locate BOOT0 button on your Black Pill board")
    print("2. Locate RESET button on your board")
    print("3. Follow this sequence:")
    print("   ├─ HOLD DOWN the BOOT0 button")
    print("   ├─ While holding BOOT0, press RESET button briefly")
    print("   ├─ RELEASE the BOOT0 button")
    print("   └─ Device is now in bootloader mode!\n")
    
    print("⏳ Waiting for bootloader response on COM3...")
    print("   (You have {} seconds to complete the sequence)\n".format(timeout))
    
    start_time = time.time()
    attempt = 0
    
    while time.time() - start_time < timeout:
        attempt += 1
        
        try:
            ser = serial.Serial(
                port,
                115200,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            
            # Send bootloader sync byte
            ser.write(bytes([0x7F]))
            time.sleep(0.1)
            
            # Check for ACK response
            if ser.in_waiting:
                resp = ser.read(1)
                if resp and resp[0] == 0x79:  # ACK
                    print("✅ BOOTLOADER DETECTED!\n")
                    ser.close()
                    return True
            
            ser.close()
            
        except Exception as e:
            pass
        
        # Print progress indicator
        if attempt % 5 == 0:
            elapsed = int(time.time() - start_time)
            remaining = timeout - elapsed
            print("   ⏱️  Waiting... ({} seconds remaining)".format(remaining))
        
        time.sleep(0.2)
    
    print("\n❌ Timeout! Bootloader was not detected.")
    print("   Make sure you followed the sequence correctly.\n")
    return False

def upload_firmware():
    """
    Upload firmware to device
    """
    BINARY_FILE = ".pio/build/blackpill_f411ce/firmware.bin"
    
    if not os.path.exists(BINARY_FILE):
        print(f"❌ Firmware not found: {BINARY_FILE}")
        print("   Run 'pio run' to build the firmware first")
        return False
    
    print("🔧 Uploading firmware...")
    print(f"   Binary: {BINARY_FILE}\n")
    
    # Call the STM32 upload script
    try:
        result = subprocess.run(
            [sys.executable, "stm32_upload.py"],
            capture_output=True,
            text=True,
            timeout=60
        )
        
        if result.returncode == 0:
            print(result.stdout)
            print("\n✅ Firmware upload successful!\n")
            return True
        else:
            print(result.stdout)
            print(result.stderr)
            print("\n❌ Upload failed")
            return False
            
    except subprocess.TimeoutExpired:
        print("❌ Upload timeout")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def test_firmware():
    """
    Test if firmware is responding
    """
    print("🔍 Testing firmware communication...")
    time.sleep(2)  # Wait for device to restart
    
    try:
        ser = serial.Serial("COM3", 115200, timeout=1)
        time.sleep(0.5)
        
        # Send STATUS command
        ser.write(b"STATUS\n")
        
        responses = []
        for _ in range(5):
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    responses.append(line)
            time.sleep(0.1)
        
        ser.close()
        
        if responses:
            print("\n✅ Firmware is responding!\n")
            print("Response received:")
            for r in responses:
                print(f"   {r}")
            print()
            return True
        else:
            print("⚠️  No response from firmware")
            return False
            
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def show_next_steps():
    """
    Show next steps for user
    """
    print("=" * 70)
    print("✅ SETUP COMPLETE!\n")
    
    print("📖 NEXT STEPS:\n")
    
    print("1️⃣  Start the Winch Control GUI:")
    print("   python winch_gui.py\n")
    
    print("2️⃣  In the GUI:")
    print("   • Port: COM3")
    print("   • Baud: 115200")
    print("   • Click: CONNECT\n")
    
    print("3️⃣  Test the winch:")
    print("   • Click UP to raise winch")
    print("   • Click STOP to stop")
    print("   • Click DOWN to lower winch\n")
    
    print("4️⃣  Monitor:")
    print("   • Watch the Console Log for feedback")
    print("   • Check Status panel for limit switches\n")
    
    print("=" * 70)

def main():
    try:
        # Step 1: Wait for bootloader
        if not wait_for_bootloader():
            print("\n📝 Please try again:")
            print("   python upload_auto.py\n")
            return False
        
        # Step 2: Upload firmware
        if not upload_firmware():
            print("Please check the error messages above.\n")
            return False
        
        # Step 3: Test firmware
        if not test_firmware():
            print("⚠️  Firmware may not be responding correctly.")
            print("    Please check your connections.\n")
            # Continue anyway, might just need more time
        
        # Step 4: Show next steps
        show_next_steps()
        
        return True
        
    except KeyboardInterrupt:
        print("\n\n⏸️  Interrupted by user")
        return False
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
