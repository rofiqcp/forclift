#!/usr/bin/env python3
"""
Windows USB Driver Installation Helper
Uses Windows Device Manager to install drivers for CH340 and CP210x
"""

import os
import subprocess
import sys
import time

def find_unsigned_devices():
    """Find devices in Device Manager that need drivers"""
    print("[*] Scanning Device Manager for uninstalled devices...")
    
    try:
        result = subprocess.run(
            ["pnputil.exe", "/enum-devices", "/class", "Ports"],
            capture_output=True,
            text=True
        )
        return result.stdout
    except Exception as e:
        print(f"[-] Error: {e}")
        return None

def install_driver_inf(inf_file):
    """Install driver using .INF file"""
    if not os.path.exists(inf_file):
        print(f"[-] File not found: {inf_file}")
        return False
    
    inf_path = os.path.abspath(inf_file)
    print(f"[*] Installing driver: {inf_path}")
    
    try:
        # Use pnputil to install the driver
        result = subprocess.run(
            ["pnputil.exe", "/add-driver", inf_path, "/install"],
            capture_output=True,
            text=True,
            timeout=30
        )
        
        if result.returncode == 0:
            print("[+] Driver installed successfully")
            return True
        else:
            print(f"[-] Installation failed: {result.stderr}")
            return False
            
    except subprocess.TimeoutExpired:
        print("[-] Installation timeout")
        return False
    except Exception as e:
        print(f"[-] Error: {e}")
        return False

def main():
    print("="*70)
    print("Windows USB Driver Installer")
    print("="*70)
    print("\nThis script will install CP210x and CH340 USB drivers.\n")
    
    # Check if running as admin
    try:
        import ctypes
        is_admin = ctypes.windll.shell.IsUserAnAdmin()
        if not is_admin:
            print("⚠️  This script works best with Administrator privileges!")
            print("   Right-click prompt → Run as Administrator\n")
    except:
        pass
    
    driver_dir = "USB_Drivers"
    
    if not os.path.exists(driver_dir):
        print(f"[-] Driver directory not found: {driver_dir}")
        return False
    
    # Find .INF files
    drivers_to_install = []
    
    print("[*] Looking for driver files...\n")
    
    # CP210x driver
    cp210x_inf = os.path.join(driver_dir, "CP210x", "silabser.inf")
    if os.path.exists(cp210x_inf):
        print(f"✅ Found CP210x driver: {cp210x_inf}")
        drivers_to_install.append(("CP210x", cp210x_inf))
    else:
        print(f"⚠️  CP210x driver not found: {cp210x_inf}")
    
    # CH340 driver
    ch340_paths = [
        os.path.join(driver_dir, "CH340", "CH341SER.INF"),
        os.path.join(driver_dir, "CH340", "ch341ser.inf"),
    ]
    for path in ch340_paths:
        if os.path.exists(path):
            print(f"✅ Found CH340 driver: {path}")
            drivers_to_install.append(("CH340", path))
            break
    else:
        print(f"⚠️  CH340 driver not found (may need to extract from CH340.exe)")
    
    if not drivers_to_install:
        print("\n[-] No drivers found!")
        print("Please extract the driver ZIP files first.")
        return False
    
    print(f"\nFound {len(drivers_to_install)} driver(s) to install.\n")
    
    # Install drivers
    results = {}
    for driver_name, inf_file in drivers_to_install:
        print("-"*70)
        print(f"Installing {driver_name}...")
        print("-"*70)
        
        if install_driver_inf(inf_file):
            results[driver_name] = True
            print(f"✅ {driver_name} installed")
        else:
            results[driver_name] = False
            print(f"❌ {driver_name} installation failed")
        
        print()
        time.sleep(1)
    
    # Summary
    print("="*70)
    print("Installation Summary")
    print("="*70)
    for driver_name, success in results.items():
        status = "✅ SUCCESS" if success else "❌ FAILED"
        print(f"{driver_name:20} {status}")
    
    print("\n" + "="*70)
    print("Next Steps:")
    print("="*70)
    print("\n1. Connect your STM32 board via USB")
    print("2. Windows will detect new device and auto-install driver")
    print("3. Open Device Manager to verify COM port assignment")
    print("4. Check that device appears with correct driver")
    print("\nDevice Manager location:")
    print("  • Right-click Start → Device Manager")
    print("  • Or: Win+R → devmgmt.msc → OK\n")
    
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
