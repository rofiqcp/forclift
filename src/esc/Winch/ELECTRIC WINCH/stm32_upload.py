#!/usr/bin/env python3
"""
STM32F411CEU6 Serial Bootloader - Upload via STM32 ROM Bootloader
Implements AN2606 - STM32 System Memory Boot Mode
"""

import serial
import time
import struct
import os
import sys

class STM32Bootloader:
    # Protocol constants
    SYNC_BYTE = 0x7F
    ACK = 0x79
    NACK = 0x1F
    
    # Commands
    GET = 0x00
    GET_VERSION = 0x01
    GET_ID = 0x02
    READ_MEMORY = 0x11
    GO = 0x21
    WRITE_MEMORY = 0x31
    ERASE = 0x43
    EXTENDED_ERASE = 0x44
    
    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.verbose = True
    
    def log(self, msg):
        if self.verbose:
            print(msg)
    
    def connect(self):
        """Open connection and sync with bootloader"""
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            time.sleep(0.5)
            self.log(f"[+] Connected to {self.port}")
            
            # Sync with bootloader
            return self.sync()
        except Exception as e:
            self.log(f"[-] Connection failed: {e}")
            return False
    
    def sync(self):
        """Send sync byte until bootloader responds"""
        self.log("[*] Syncing with bootloader...")
        
        for attempt in range(10):
            try:
                self.ser.write(bytes([self.SYNC_BYTE]))
                resp = self.ser.read(1)
                
                if resp and resp[0] == self.ACK:
                    self.log("[+] Bootloader synchronized!")
                    return True
                    
            except:
                pass
            
            time.sleep(0.1)
        
        self.log("[-] Failed to sync with bootloader")
        return False
    
    def send_command(self, cmd):
        """Send command byte and checksum"""
        checksum = cmd ^ 0xFF
        self.ser.write(bytes([cmd, checksum]))
        return self.wait_ack()
    
    def wait_ack(self, timeout=1):
        """Wait for ACK response"""
        start = time.time()
        while time.time() - start < timeout:
            try:
                resp = self.ser.read(1)
                if resp:
                    if resp[0] == self.ACK:
                        return True
                    elif resp[0] == self.NACK:
                        return False
            except:
                pass
            time.sleep(0.01)
        return False
    
    def get_bootloader_info(self):
        """Get bootloader version and chip ID"""
        # Get version
        self.log("[*] Getting bootloader info...")
        self.send_command(self.GET_VERSION)
        
        data_len = self.ser.read(1)
        if not data_len:
            return None
        
        data = self.ser.read(data_len[0] + 1)
        self.wait_ack()
        
        if data:
            self.log(f"[+] Bootloader version: {data[0]:02X}")
        
        # Get chip ID
        self.send_command(self.GET_ID)
        data_len = self.ser.read(1)
        if data_len and data_len[0] >= 1:
            data = self.ser.read(data_len[0] + 1)
            self.wait_ack()
            chip_id = (data[0] << 8) | data[1]
            self.log(f"[+] Chip ID: 0x{chip_id:04X}")
            return chip_id
        
        return None
    
    def erase_flash(self):
        """Erase entire flash memory"""
        self.log("[*] Erasing flash memory...")
        
        # Send EXTENDED_ERASE command
        self.send_command(self.EXTENDED_ERASE)
        
        # Send parameters for full erase
        # FF FF = erase all pages
        self.ser.write(bytes([0xFF, 0xFF, 0x00]))
        
        if self.wait_ack(timeout=5):
            self.log("[+] Flash erased")
            time.sleep(0.5)
            return True
        else:
            self.log("[-] Erase failed")
            return False
    
    def write_memory(self, address, data):
        """Write data to flash memory"""
        if len(data) > 256:
            self.log(f"[-] Data too large: {len(data)}")
            return False
        
        # Send WRITE_MEMORY command
        self.send_command(self.WRITE_MEMORY)
        
        # Send address (4 bytes, BE)
        addr_bytes = struct.pack('>I', address)
        addr_checksum = address >> 24 ^ (address >> 16) & 0xFF ^ (address >> 8) & 0xFF ^ address & 0xFF
        self.ser.write(addr_bytes)
        self.ser.write(bytes([addr_checksum]))
        
        if not self.wait_ack(timeout=1):
            self.log(f"[-] Address write failed for 0x{address:08X}")
            return False
        
        # Send length and data
        data_len = len(data) - 1
        checksum = data_len
        self.ser.write(bytes([data_len]))
        
        for byte in data:
            self.ser.write(bytes([byte]))
            checksum ^= byte
        
        self.ser.write(bytes([checksum]))
        
        if not self.wait_ack(timeout=1):
            self.log(f"[-] Data write failed at 0x{address:08X}")
            return False
        
        return True
    
    def write_file(self, filepath, start_address=0x08000000):
        """Write firmware file to flash"""
        if not os.path.exists(filepath):
            self.log(f"[-] File not found: {filepath}")
            return False
        
        with open(filepath, 'rb') as f:
            data = f.read()
        
        self.log(f"[+] Firmware size: {len(data)} bytes")
        self.log(f"[*] Writing to 0x{start_address:08X}...")
        
        # Write in 256-byte chunks
        chunk_size = 256
        for offset in range(0, len(data), chunk_size):
            chunk = data[offset:offset + chunk_size]
            address = start_address + offset
            
            if not self.write_memory(address, chunk):
                return False
            
            if (offset + chunk_size) % 1024 == 0:
                self.log(f"[*] Progress: {offset + chunk_size}/{len(data)} bytes")
        
        self.log("[+] Firmware written successfully!")
        return True
    
    def run_firmware(self, address=0x08000000):
        """Jump to application code"""
        self.log(f"[*] Starting firmware at 0x{address:08X}...")
        
        self.send_command(self.GO)
        
        # Send address
        addr_bytes = struct.pack('>I', address)
        addr_checksum = address >> 24 ^ (address >> 16) & 0xFF ^ (address >> 8) & 0xFF ^ address & 0xFF
        self.ser.write(addr_bytes)
        self.ser.write(bytes([addr_checksum]))
        
        if self.wait_ack():
            self.log("[+] Firmware started!")
            return True
        else:
            self.log("[-] Failed to start firmware")
            return False
    
    def close(self):
        """Close connection"""
        if self.ser:
            self.ser.close()
            self.log("[+] Connection closed")

def main():
    print("=" * 60)
    print("STM32F411CEU6 Firmware Upload Tool")
    print("=" * 60)
    
    BINARY_FILE = ".pio/build/blackpill_f411ce/firmware.bin"
    COM_PORT = "COM3"
    BAUD_RATE = 115200
    APP_ADDRESS = 0x08000000
    
    if not os.path.exists(BINARY_FILE):
        print(f"[-] Firmware file not found: {BINARY_FILE}")
        print("[*] Please build the firmware first with: pio run")
        return False
    
    bootloader = STM32Bootloader(COM_PORT, BAUD_RATE)
    
    print("\n[Step 1] Connecting to bootloader...")
    print("[INFO] Ensure BOOT0 is held HIGH during connection!")
    print("[INFO] Press RESET button while holding BOOT0...\n")
    
    if not bootloader.connect():
        print("[-] Failed to connect. Try holding BOOT0 and pressing RESET")
        return False
    
    print("\n[Step 2] Getting bootloader info...")
    bootloader.get_bootloader_info()
    
    print("\n[Step 3] Erasing flash memory...")
    if not bootloader.erase_flash():
        bootloader.close()
        return False
    
    print("\n[Step 4] Writing firmware...")
    if not bootloader.write_file(BINARY_FILE, APP_ADDRESS):
        bootloader.close()
        return False
    
    print("\n[Step 5] Starting firmware...")
    if not bootloader.run_firmware(APP_ADDRESS):
        bootloader.close()
        return False
    
    bootloader.close()
    
    print("\n" + "=" * 60)
    print("[+] Upload complete! Firmware is running.")
    print("[*] Connect to COM3 @ 115200 to test the winch control.")
    print("=" * 60)
    
    return True

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
