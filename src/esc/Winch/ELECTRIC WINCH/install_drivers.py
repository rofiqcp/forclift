#!/usr/bin/env python3
"""
USB Serial Driver Installer for STM32 Development
Installs CH340, CP210x, and FTDI drivers
"""

import os
import sys
import urllib.request
import subprocess
import time
import shutil

def download_file(url, filename):
    """Download file from URL"""
    print(f"[*] Downloading {filename}...")
    try:
        urllib.request.urlretrieve(url, filename)
        print(f"[+] Downloaded: {filename}")
        return True
    except Exception as e:
        print(f"[-] Download failed: {e}")
        return False

def install_exe(filepath):
    """Install Windows executable"""
    if not os.path.exists(filepath):
        print(f"[-] File not found: {filepath}")
        return False
    
    print(f"[*] Installing {os.path.basename(filepath)}...")
    try:
        # Run installer
        result = subprocess.run(
            [filepath],
            timeout=300
        )
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        print("[-] Installation timeout")
        return False
    except Exception as e:
        print(f"[-] Installation error: {e}")
        return False

def install_driver_ch340():
    """Download and install CH340 driver"""
    print("\n" + "="*60)
    print("Installing CH340 Driver")
    print("="*60)
    
    # CH340 driver from WCH official
    url = "https://www.wch.cn/downloads/CH341SER_EXE.zip"
    filename = "CH340_Driver.zip"
    
    print("[*] CH340 is a USB-to-serial converter used by many STM32 boards")
    
    if download_file(url, filename):
        print("[*] Extracting and installing...")
        try:
            # Extract zip
            import zipfile
            with zipfile.ZipFile(filename, 'r') as z:
                z.extractall("CH340_Temp")
            
            # Find and run installer
            for root, dirs, files in os.walk("CH340_Temp"):
                for file in files:
                    if file.endswith(".exe"):
                        installer = os.path.join(root, file)
                        if install_exe(installer):
                            print("[+] CH340 driver installed successfully")
                            return True
            
            # Cleanup
            shutil.rmtree("CH340_Temp", ignore_errors=True)
            
        except Exception as e:
            print(f"[-] Extraction/installation failed: {e}")
    
    return False

def install_driver_cp210x():
    """Download and install CP210x driver"""
    print("\n" + "="*60)
    print("Installing CP210x Driver")
    print("="*60)
    
    # CP210x driver from Silicon Labs official
    url = "https://www.silabs.com/documents/public/software/CP210x_Windows_Drivers.zip"
    filename = "CP210x_Driver.zip"
    
    print("[*] CP210x is used by Silicon Labs USB-to-serial converters")
    
    if download_file(url, filename):
        print("[*] Extracting and installing...")
        try:
            import zipfile
            with zipfile.ZipFile(filename, 'r') as z:
                z.extractall("CP210x_Temp")
            
            # Find and run installer
            for root, dirs, files in os.walk("CP210x_Temp"):
                for file in files:
                    if file.endswith(".exe"):
                        installer = os.path.join(root, file)
                        if install_exe(installer):
                            print("[+] CP210x driver installed successfully")
                            return True
            
            shutil.rmtree("CP210x_Temp", ignore_errors=True)
            
        except Exception as e:
            print(f"[-] Extraction/installation failed: {e}")
    
    return False

def install_driver_ftdi():
    """Download and install FTDI driver"""
    print("\n" + "="*60)
    print("Installing FTDI Driver")
    print("="*60)
    
    # FTDI driver from FTDI official
    url = "https://ftdichip.com/wp-content/uploads/2020/08/CDM21228_Setup.zip"
    filename = "FTDI_Driver.zip"
    
    print("[*] FTDI FT232RL is a popular USB-to-serial converter")
    
    if download_file(url, filename):
        print("[*] Extracting and installing...")
        try:
            import zipfile
            with zipfile.ZipFile(filename, 'r') as z:
                z.extractall("FTDI_Temp")
            
            # Find and run installer
            for root, dirs, files in os.walk("FTDI_Temp"):
                for file in files:
                    if file.endswith(".exe"):
                        installer = os.path.join(root, file)
                        if install_exe(installer):
                            print("[+] FTDI driver installed successfully")
                            return True
            
            shutil.rmtree("FTDI_Temp", ignore_errors=True)
            
        except Exception as e:
            print(f"[-] Extraction/installation failed: {e}")
    
    return False

def main():
    print("="*60)
    print("USB Serial Driver Installer")
    print("="*60)
    print("\nThis will install drivers for:")
    print("  • CH340 (WCH USB-to-serial)")
    print("  • CP210x (Silicon Labs USB-to-serial)")
    print("  • FTDI FT232RL (FTDI USB-to-serial)")
    print("\nThese drivers are needed for STM32 development boards")
    print("to communicate with your computer via USB.\n")
    
    results = {
        "CH340": False,
        "CP210x": False,
        "FTDI": False
    }
    
    # Install drivers
    results["CH340"] = install_driver_ch340()
    time.sleep(1)
    
    results["CP210x"] = install_driver_cp210x()
    time.sleep(1)
    
    results["FTDI"] = install_driver_ftdi()
    
    # Summary
    print("\n" + "="*60)
    print("Installation Summary")
    print("="*60)
    for driver, success in results.items():
        status = "✅ SUCCESS" if success else "❌ FAILED/SKIPPED"
        print(f"{driver:15} {status}")
    
    print("\n" + "="*60)
    print("After installation:")
    print("  1. Plug in your STM32 board via USB")
    print("  2. Windows will auto-detect and install appropriate driver")
    print("  3. Check Device Manager to see COM port")
    print("  4. Your device should appear as 'COM3' or similar")
    print("="*60)
    
    return all(results.values())

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n[!] Interrupted")
        sys.exit(1)
    except Exception as e:
        print(f"\n[-] Error: {e}")
        sys.exit(1)
