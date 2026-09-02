# 📦 USB Driver Installation Guide

## ✅ Status: Drivers Downloaded

Driver files telah diunduh ke folder: `USB_Drivers/`

```
✅ CH340_Driver.zip       (WCH CH340 USB-to-Serial)
✅ CP210x_Driver.zip      (Silicon Labs CP210x)
❌ FTDI_Driver.exe        (Download failed - optional)
```

---

## 🔧 Cara Install Driver

### **Metode 1: Windows Device Manager Auto-Install (RECOMMENDED)**

Ini adalah metode paling sederhana:

1. **Ekstrak driver files:**
   - Buka `USB_Drivers/CH340_Driver.zip`
   - Extract ke folder (misalnya `CH340`)
   - Buka `USB_Drivers/CP210x_Driver.zip`
   - Extract ke folder (misalnya `CP210x`)

2. **Hubungkan board STM32 ke USB:**
   - Plug in board Black Pill ke computer
   - Windows akan deteksi device baru

3. **Install driver via Device Manager:**
   - Buka **Device Manager** (devmgmt.msc)
   - Cari device yang belum terinstall (biasanya dengan warning icon)
   - Right-click → Update driver
   - Pilih "Browse my computer for drivers"
   - Arahkan ke folder driver yang sudah di-extract
   - Click "Next" dan tunggu instalasi selesai

---

### **Metode 2: Manual Installer**

Jika ada file `.EXE`:

1. Extract ZIP file → cari `SETUP.EXE`
2. Right-click `SETUP.EXE` → **Run as Administrator**
3. Follow installation wizard
4. Selesai!

---

## 📋 File Checklist

Setelah download, dalam folder `USB_Drivers/` harus ada:

```
USB_Drivers/
├── CH340_Driver.zip           ← Driver WCH CH340
├── CP210x_Driver.zip          ← Driver Silicon Labs
└── [Kalau ada] FTDI Driver
```

---

## ✅ Verifikasi Instalasi

Setelah install, cek di Device Manager:

1. **Buka Device Manager:**
   - Right-click Start menu
   - Pilih Device Manager
   - **Atau** tekan `Win+R`, ketik `devmgmt.msc`, Enter

2. **Cari di bagian "Ports (COM & LPT)":**
   - Harus muncul sesuatu seperti:
   ```
   COM3 (atau COM1, COM2, COM4, dll)
   USB Serial Device (COM3)
   Silicon Labs CP210x USB to UART Bridge (COM3)
   ```

3. **Jika berhasil:**
   - ✅ Device muncul dengan COM port
   - ✅ Tidak ada warning/error icon
   - ✅ Ready untuk upload firmware!

---

## 🆘 Troubleshooting

### ❌ Device tidak muncul di Device Manager?

1. Cek USB cable - pastikan terhubung baik
2. Coba port USB berbeda di komputer
3. Restart komputer setelah install driver
4. Device mungkin masih dalam bootloader mode - tekan RESET

### ❌ Device muncul tapi dengan warning icon?

1. Right-click device → Properties
2. Tab "Driver" → Update Driver
3. Browse to extracted driver folder
4. Let Windows install from that location

### ❌ Instalasi exe file error "Run as Administrator"?

1. Buka Command Prompt **as Administrator**
2. Navigate ke folder driver
3. Jalankan: `SETUP.EXE` atau nama file installer

---

## 🚀 Setelah Driver Installed

Device harus muncul sebagai COM port (misalnya COM3) di Device Manager.

**Kemudian lanjutkan dengan:**

1. Jalankan bootloader sequence (BOOT0 + RESET)
2. Jalankan `python upload_auto.py`
3. Firmware akan upload otomatis
4. Winch GUI siap digunakan!

---

## 📝 Quick Commands

**Buka Device Manager:**
```
Win+R → devmgmt.msc → OK
```

**Atau:**
```
Right-click Start Menu → Device Manager
```

**Cek COM port tersedia:**
```
CMD: mode
PowerShell: Get-CimInstance Win32_SerialPort
```

---

## 💡 Catatan Penting

- **CH340** driver untuk board dengan chip WCH (banyak di Arduino clone)
- **CP210x** driver untuk board resmi Silicon Labs
- **Board Anda (STM32F411)** mungkin pakai salah satu dari keduanya
- Windows biasanya auto-install driver dari Windows Update
- Kalau sudah di-detect sebagai "USB Serial Device", upgrade ke driver resmi buat stabilitas lebih baik

---

## ✅ NEXT STEPS

1. ✅ Extract driver files dari `USB_Drivers/`
2. ⏳ Install drivers (auto via Device Manager atau manual)
3. ⏳ Verify di Device Manager (harus muncul COM3)
4. ⏳ Perform bootloader sequence
5. ⏳ Run `python upload_auto.py`
6. ⏳ Test dengan GUI

**Good luck! 🚀**
