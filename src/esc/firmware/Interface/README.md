# ESC FOC/PID Tuning Interface - Qt5

GUI berkomunikasi langsung dengan STM32 melalui USART3 115200 menggunakan **ESC Protocol v4**: frame tetap 64 byte + CRC16-CCITT-FALSE. Ini adalah protocol terbaru pada paket ESC yang menjadi sumber revisi ini; header firmware, host C++, dan Python memakai versi yang sama.

## Install

```bash
cd esc/firmware/Interface
python3 -m pip install -r requirements.txt
python3 esc_tuning_gui.py
```

Gunakan `/dev/serial/by-id/...` bila tersedia. Jika user memilih `/dev/ttyUSBx`, GUI juga menyimpan identity USB dari pyserial dan dapat mengikuti perubahan `ttyUSB0 -> ttyUSB1` setelah hot-plug bila serial-number atau USB location cocok.

## Layout

### Panel kiri

1. **Connection** — pilih port, Connect/Disconnect, hot-plug.
2. **Control** — VLT/TRQ/SPD/POS per motor, setpoint, ARM/DISARM.
3. **PID Tuning** — Left/Right/Both, Id/Iq/Speed/Position, Kp/Ki/Kd, limits, deadband, read/apply/save.
4. **Telemetry** — Basic/FOC/PID/Raw, rate 1..100 Hz, pilihan variabel.
5. **EEPROM** — verified, dirty, generation, CRC, verify failures, save/load/zero.
6. **CSV Logger** — pilih file, Start/Stop recording.

### Panel kanan

- tabel **Live Data** untuk variabel yang dipilih;
- kolom Raw dan Engineering;
- grafik real-time multi-channel;
- pause, auto-range, clear, window waktu.

Plot di-refresh sekitar 20 Hz agar CPU ringan walaupun telemetry serial lebih cepat.

## Koneksi / hot-plug

GUI **tidak auto-connect saat startup**. User harus menekan Connect sekali. Setelah itu, bila opsi hot-plug aktif, intent koneksi dipertahankan saat USB benar-benar dicabut.

Port terbuka tetapi feedback belum valid **tidak menyebabkan close/open berulang**. GUI mempertahankan handle yang sama dan mengulang HELLO + TELEMETRY_CONFIG setiap sekitar 0,5 s. Reconnect fisik hanya dilakukan jika device hilang atau ada I/O error. Status Connection menampilkan `RX bytes / valid frames / invalid frames`: `RX=0` menunjukkan belum ada byte dari board, sedangkan `RX>0` dengan invalid bertambah mengarah ke mismatch firmware/protokol/CRC.

Setelah reconnect GUI selalu DISARM dan tidak mengirim setpoint lama secara otomatis.

## PID tuning

- Id / Current-D: Kp/Ki/Kd **integer raw core**.
- Iq / Torque: Kp/Ki/Kd **integer raw core**.
- Speed: Kp/Ki/Kd **integer raw core**.
- Position: Kp/Ki/Kd **Q16.16 desimal** + integral/output/position limits + deadband.

`Apply RAM` tidak menulis flash, selalu DISARM sebelum mengganti gain, dan menghasilkan status RAM DIRTY. DISARM saat apply mencegah ISR FOC membaca kombinasi Kp baru dengan Ki/Kd lama di satu control step.

`Apply + Save + Verify EEPROM` menulis permanen, lalu firmware melakukan read-back seluruh konfigurasi + CRC. Save permanen otomatis DISARM untuk menghindari flash stall saat motor aktif.

Jika Motor = Both, read-back diverifikasi **LEFT lalu RIGHT** dan MATCH baru ditampilkan jika keduanya sama.

## Fix motor kanan POS

Host selalu memakai koordinat mekanik. Firmware mengembalikan mapping hardware asli:

```c
pwml = +runtimeCommandLeft;
pwmr = -runtimeCommandRight;
```

Ini mencegah PID Position kanan menjadi positive-feedback. Deadband juga mereset integral ketika target tercapai.

## CSV

CSV menyimpan setiap frame CRC-valid yang diterima dan snapshot setting GUI pada baris yang sama, termasuk:

- timestamp;
- telemetry;
- status/error;
- mode/setpoint;
- loop dan motor PID;
- Kp/Ki/Kd;
- integral/output/position limit;
- deadband.

File di-buffer dan di-flush sekitar satu detik sekali agar logging tidak mengganggu plotting. CSV tetap ditulis pada setiap frame valid; hanya repaint label status CSV yang di-throttle sekitar 4 Hz. Grafik di-refresh ~20 Hz dan tabel ~10 Hz.

## Tester terminal

Monitor tanpa ARM:

```bash
python3 esc_serial_test.py --port /dev/ttyUSB0
```

Bench-test speed dengan roda terangkat:

```bash
python3 esc_serial_test.py --port /dev/ttyUSB0 --mode SPD --left 100 --right 100 --arm
```

Tester menampilkan signed position, speed, status EEPROM verified/dirty, battery, temperature, current, dan error.
