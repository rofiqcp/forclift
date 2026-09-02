#!/usr/bin/env python3
"""
USB Driver Download & Manual Installation Guide
Downloads drivers so user can install manually with elevated privileges
"""

import os
import urllib.request
import sys

def download_driver(url, filename):
    """Download driver file"""
    try:
        print(f"[*] Downloading {filename}...")
        urllib.request.urlretrieve(url, filename, reporthook=progress)
        print(f"\n[+] Downloaded: {os.path.abspath(filename)}")
        return True
    except Exception as e:
        print(f"[-] Download failed: {e}")
        return False

def progress(blocknum, blocksize, totalsize):
    """Progress bar for downloads"""
    readsofar = blocknum * blocksize
    if totalsize > 0:
        percent = readsofar * 1e2 / totalsize
        s = "\r%5.1f%%" % percent
        sys.stdout.write(s)
        if readsofar >= totalsize:
            sys.stdout.write("\n")

def main():
    print("="*70)
    print("USB Serial Driver Downloader")
    print("="*70)
    print("\nThis script will download USB-to-serial drivers.")
    print("Then you'll need to install them manually with admin privileges.\n")
    
    # Create drivers folder
    driver_dir = "USB_Drivers"
    if not os.path.exists(driver_dir):
        os.makedirs(driver_dir)
    
    drivers = {
        "CH340": {
            "name": "WCH CH340 USB-to-Serial",
            "url": "https://www.wch.cn/downloads/CH341SER_EXE.zip",
            "file": os.path.join(driver_dir, "CH340_Driver.zip"),
            "notes": "Popular on Arduino clones and some STM32 boards"
        },
        "CP210x": {
            "name": "Silicon Labs CP210x USB-to-Serial",
            "url": "https://www.silabs.com/documents/public/software/CP210x_Universal_Windows_Driver.zip",
            "file": os.path.join(driver_dir, "CP210x_Driver.zip"),
            "notes": "Used on many official development boards"
        },
        "FTDI": {
            "name": "FTDI FT232 USB-to-Serial",
            "url": "https://ftdichip.com/wp-content/uploads/2024/01/CDM21228_Setup.exe",
            "file": os.path.join(driver_dir, "FTDI_Driver.exe"),
            "notes": "High-quality USB UART devices"
        }
    }
    
    results = {}
    
    for driver_name, driver_info in drivers.items():
        print("\n" + "-"*70)
        print(f"Driver: {driver_info['name']}")
        print(f"Info:   {driver_info['notes']}")
        print("-"*70)
        
        if download_driver(driver_info['url'], driver_info['file']):
            results[driver_name] = True
        else:
            results[driver_name] = False
    
    # Summary and instructions
    print("\n" + "="*70)
    print("Installation Instructions")
    print("="*70)
    print("\nAll drivers have been downloaded to:")
    print(f"  📁 {os.path.abspath(driver_dir)}\n")
    
    print("To install the drivers:\n")
    
    for driver_name, success in results.items():
        if success:
            driver_info = drivers[driver_name]
            print(f"✅ {driver_name}:")
            print(f"   1. Open: {driver_info['file']}")
            
            if driver_info['file'].endswith('.zip'):
                print(f"   2. Extract the ZIP file")
                print(f"   3. Find and run 'SETUP.EXE' or similar installer")
            else:
                print(f"   2. Right-click → Run as Administrator")
            
            print(f"   3. Follow installation wizard")
            print()
    
    print("\nAfter installation:")
    print("  1. Connect your STM32 board via USB")
    print("  2. Windows will auto-detect and install driver")
    print("  3. Open Device Manager to confirm COM port assignment")
    print("  4. Your board should appear as 'COM3' or similar\n")
    
    print("To open Device Manager:")
    print("  • Right-click Start menu → Device Manager")
    print("  • Or type 'devmgmt.msc' in Run dialog (Win+R)\n")
    
    print("="*70)
    print("\n✅ Drivers downloaded successfully!")
    print("📝 Next: Install them manually using instructions above.\n")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[!] Interrupted")
        sys.exit(1)
    except Exception as e:
        print(f"\n[-] Error: {e}")
        sys.exit(1)
