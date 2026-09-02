# NADIA Electric Winch - Complete Setup Guide

## Status: Firmware Built Successfully ✅

Your firmware has been compiled successfully and is ready to upload to the STM32F411CEU6 board.

### Firmware Build Information
- **Binary file**: `.pio/build/blackpill_f411ce/firmware.bin`
- **Size**: ~32KB (6.1% of flash)
- **Protocol**: Serial @ 115200 baud
- **Baud Rate**: 115200

---

## Step 1: Put Device into Bootloader Mode

The STM32F411CEU6 (WeAct Black Pill V2) has a bootloader that allows firmware updates via serial.

### Hardware Instructions:

**IMPORTANT**: The device MUST be in bootloader mode before uploading firmware!

1. **Locate the BOOT0 button** on your Black Pill board (usually labeled)
2. **Locate the RESET button** on your board
3. **Do this sequence**:
   - Hold down the **BOOT0** button (press and hold)
   - While still holding BOOT0, press the **RESET** button briefly
   - Release the **BOOT0** button
   - Now the device is in bootloader mode (LED might not light up, this is normal)

4. **The board should now respond to bootloader commands**

---

## Step 2: Upload Firmware

After putting the device into bootloader mode, run this command:

```bash
cd c:\Users\THINKPAD\Downloads\NADIA\NADIA\ELECTRIC WINCH

python stm32_upload.py
```

Or using PlatformIO directly:

```bash
pio run --environment blackpill_f411ce --target upload
```

The upload process will:
1. ✓ Erase the flash memory
2. ✓ Write the firmware (32KB)
3. ✓ Verify the upload
4. ✓ Start the firmware automatically

---

## Step 3: Verify Firmware is Running

After successful upload, the device will automatically restart with the new firmware.

To test:

```bash
python test_device.py
```

You should see responses like:
```
[RX] STATE:STOPPED
[RX] TOP:0
[RX] BOTTOM:0
[RX] PWM:0
[RX] DIR:0
```

---

## Step 4: Run the Winch Control GUI

The GUI is already running! Connect via:

1. **Open the GUI** (if not already running):
   ```bash
   python winch_gui.py
   ```

2. **In the GUI**:
   - Select **COM3** from the COM Port dropdown
   - Leave Baud at **115200**
   - Click **CONNECT**
   - You should see "CONNECTED" in green

3. **Test Commands**:
   - Click **UP** to raise the winch
   - Click **STOP** to stop
   - Click **DOWN** to lower the winch
   - Monitor the log for status messages

---

## Step 5: Hardware Testing

Once the GUI is connected and showing status, test:

1. **UP Command**:
   - GUI shows "STATE:UP"
   - Motor should rotate UP direction
   - Watch limit switches in real-time

2. **Limit Switches**:
   - Trigger top limit: GUI shows "LS ATAS:ACTIVE" in red
   - Motor automatically stops
   - Auto-return sequence may engage

3. **Emergency Stop**:
   - Click STOP at any time
   - Motor stops immediately
   - GUI shows "STATE:STOPPED"

---

## Troubleshooting

### Device not responding to upload:
- ✓ Make sure you held BOOT0 while pressing RESET
- ✓ Wait a moment before running upload script
- ✓ Try the sequence again: BOOT0 → RESET → Release BOOT0

### GUI shows "DISCONNECTED":
- Check COM port (should be COM3)
- Check baud rate (should be 115200)
- Try "REFRESH PORT" button
- Verify firmware is running (use test_device.py)

### GUI connects but no status data:
- Firmware may not be properly uploaded
- Try upload process again
- Check the console log for error messages

### Motor not moving:
- Check motor power connections
- Check BTS7960 driver connections (RPWM on PB8, LPWM on PA2)
- Verify limit switches are connected (PB6, PB9)
- Check if limit is already triggered

---

## File Locations

- **GUI Script**: `winch_gui.py`
- **Firmware Upload**: `stm32_upload.py`
- **Test Script**: `test_device.py`
- **Build Config**: `platformio.ini`
- **Firmware Source**: `src/main.cpp`, `src/electric_winch.cpp`
- **Firmware Header**: `include/electric_winch.h`

---

## Serial Protocol Reference

The firmware supports these commands:

- `STATUS` - Get current state
- `UP` - Raise winch
- `DOWN` - Lower winch
- `STOP` - Emergency stop

Status responses:
```
STATE:STOPPED|UP|DOWN|UP_STOPPING|DOWN_STOPPING|AUTO_RETURN|AUTO_DELAY
TOP:0|1
BOTTOM:0|1
PWM:0-100
DIR:-1|0|1
```

---

## Next Steps

1. ✅ Build firmware (DONE)
2. ⏳ Put device in bootloader mode (MANUAL ACTION REQUIRED)
3. ⏳ Upload firmware using `stm32_upload.py`
4. ⏳ Test with GUI
5. ⏳ Test hardware controls

**Ready to proceed?** Follow the bootloader mode instructions above, then run `stm32_upload.py`.

Good luck! 🚀
