# 🔧 USB Driver Installation - Manual Approach (Recommended)

## ✅ Current Status

```
📱 Device:        STM32F411CEU6 (Black Pill)
🔌 Port:          COM3 ✅
📊 Status:        Detected and responding
🎯 Next:          Bootloader sequence → Firmware upload
```

---

## 📖 How to Manually Install/Update Driver via Device Manager

This is the **easiest and most reliable method**:

### **Step 1: Open Device Manager**

Choose ONE:
- Right-click **Start button** → Select **Device Manager**
- **OR** Press `Win+R` → Type `devmgmt.msc` → Press Enter

### **Step 2: Find Your Device**

Look under **"Ports (COM & LPT)"**:

```
✅ Should see:
- COM3 (or similar)
- USB Serial Device (COM3)
- OR Silicon Labs CP210x USB to UART Bridge (COM3)
```

### **Step 3: Update Driver (if needed)**

If you want the **Silicon Labs official driver** instead of generic:

1. Right-click your COM port device
2. Select **"Update driver"**
3. Choose **"Browse my computer for drivers"**
4. Navigate to: `USB_Drivers\CP210x\`
5. Click **"Next"** and wait for installation
6. Click **"Finish"** when done
7. Device should now show as "Silicon Labs CP210x..."

### **Step 4: Verify**

After update:
- Device should appear under Ports section
- No warning/error icons
- Should still be COM3 (or whatever port was assigned)

---

## ⚡ Quick Start (Skip Driver Update)

**Since your device is already detected as COM3, you can proceed immediately:**

```bash
1. Keep device plugged in
2. Do bootloader sequence:
   - HOLD BOOT0 button
   - PRESS RESET button (while holding BOOT0)
   - RELEASE BOOT0
   
3. Run firmware upload:
   python upload_auto.py
```

---

## 🎯 Why Update to Official Driver?

### Generic "USB Serial Device" Driver:
- ✅ Works fine
- ✅ Basic functionality
- ⚠️ May have timing issues
- ⚠️ No manufacturer support

### Official "Silicon Labs CP210x" Driver:
- ✅ Optimized for CP210x chips
- ✅ Better stability
- ✅ Professional support
- ✅ Works with all CP210x boards

**Recommendation:** Update to official driver for better reliability, especially for firmware upload.

---

## 📸 Device Manager Screenshot Locations

### **Ports Section (Where your device appears):**
```
Device Manager
├── Ports (COM & LPT)
│   ├── COM1
│   ├── COM3 ← Your STM32 board (HERE)
│   └── COM4
└── [Other categories]
```

### **After Driver Update:**
```
Device Manager
├── Ports (COM & LPT)
│   ├── COM3
│   └── Silicon Labs CP210x USB to UART Bridge (COM3) ← Now shows this
└── [Other categories]
```

---

## ✅ Checklist

- [ ] Device Manager opened
- [ ] Found COM3 in "Ports (COM & LPT)" section
- [ ] (Optional) Updated to official CP210x driver
- [ ] No warning/error icons on device
- [ ] Ready to proceed with bootloader sequence

---

## 🚀 Next Steps

After driver is verified to be working:

1. **Hold BOOT0, press RESET**
2. **Run:** `python upload_auto.py`
3. **Wait for upload to complete**
4. **GUI akan terhubung otomatis**

---

## 🆘 If Device Doesn't Appear

1. Check USB cable (try different cable or port)
2. Restart computer
3. Re-plug device
4. Check Device Manager again (may appear under "Unknown devices")

If still not appearing, you may need to install drivers manually from `USB_Drivers\CP210x\`.

---

## 📝 File Locations

```
Current Directory: c:\Users\THINKPAD\Downloads\NADIA\NADIA\ELECTRIC WINCH\

USB_Drivers/
├── CP210x/              ← Driver files here
│   ├── silabser.inf
│   ├── silabser.sys
│   ├── silabser.cat
│   └── [other files]
└── CH340/               ← CH340 driver (if needed)
```

---

## 💡 Troubleshooting

| Problem | Solution |
|---------|----------|
| Device not in Device Manager | Restart computer, re-plug device |
| Wrong driver installed | Right-click → Update driver → Browse to `USB_Drivers\CP210x\` |
| COM port keeps changing | Update to official driver (it should be stable) |
| COM3 appears with error | Device Manager → Update driver from extracted files |

---

**Status: ✅ Device detected on COM3 - Ready to proceed!**

Next action: Perform bootloader sequence and run `python upload_auto.py` 🚀
