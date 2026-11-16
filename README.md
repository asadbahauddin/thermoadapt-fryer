# ThermoAdapt Fryer

**Kontrol suhu minyak deep fryer berbahan bakar gas dengan Adaptive PID berbasis ESP32.**
Termasuk firmware mikrokontroler, telemetri WebSocket, logger data Python, serta
identifikasi sistem (FOPDT) dan tuning PID robust (SIMC) di MATLAB.

> Status: prototipe riset. Bagian yang **belum terverifikasi di hardware** ditandai di
> komentar kode. Jangan sambungkan servo ke katup gas sebelum `SERVO_SAFETY` diaktifkan.

## Struktur

| Folder / file | Isi |
|---|---|
| `dep_frayer/dep_frayer.ino` | Firmware ESP32: WiFi AP + web + WebSocket, MAX31855, servo katup gas, motor DC Z (BTS7960), stepper X (TB6600), 4 limit switch, buzzer; sejak rev 39 mengirim data tampilan ke Arduino layar lewat UART2 |
| `lcd_mega/lcd_mega.ino` | Arduino UNO + shield TFT 2.4" (MCUFRIEND, 320x240): menampilkan data dari ESP32 (satu arah) |
| `Python/` | Logger data uji via WebSocket: grafik live (suhu vs setpoint, bukaan servo) + CSV semua sensor/aktuator, auto-simpan CSV & PNG saat berhenti |
| `Program/` | Model Simulink Adaptive PID + simulasi (parameter plant masih asumsi) |
| `Program/step_test/` | Identifikasi FOPDT dari data step test + tuning PI SIMC + cek robustness (Ms, PM, GM) |
| `pin_scan.txt` | Sketch diagnostik pin limit switch |
| `FRITZINK.fzz` | Rangkaian (Fritzing) |

## Perangkat keras

ESP32 DevKit V1 30-pin · thermocouple K + MAX31855 · servo 180° pada katup gas ·
motor DC + BTS7960 (sumbu Z) · stepper NEMA + TB6600 (sumbu X) · 4 limit switch ·
Arduino UNO + shield TFT 2.4" (layar).

### Layar di Arduino UNO (firmware rev 41)

Shield TFT 2.4" (jenis MCUFRIEND, data paralel, 5 V) ditancap langsung ke UNO. Touch dan slot SD tidak dipakai.
Library UNO: *MCUFRIEND_kbv* + *Adafruit GFX Library*. Data diterima lewat SoftwareSerial (pin 0/1 dipakai USB).

| Dari | Ke | Catatan |
|---|---|---|
| ESP32 GPIO22 (TX2) | level shifter LV1 | sisi 3,3 V (LV = 3V3 ESP32) |
| level shifter HV1 | UNO A5 | sisi 5 V (HV = 5V UNO), 19200 bps, data satu arah |
| ESP32 GND, UNO GND | GND level shifter | wajib, ground bersama (GND UNO mis. pin 6 header ICSP) |

Bingkai dari ESP32: `$DF,t,held,fault,sp,se,pe,am,ph,pu,pf,sv,uj,us,uf,ls,dc,xm,xp,ssid,ip,wc,up,pesan*CS` (CS = XOR byte di antara `$` dan `*`).

## Logger data (Python)

```
pip install -r Python/requirements.txt
python Python/logger.py            # laptop tersambung ke WiFi DeepFryer-ESP32
python Python/logger.py --demo     # coba tanpa hardware
python Python/uji_logger.py        # uji otomatis (parser, WebSocket, end-to-end)
```

Berhenti dengan menutup jendela grafik atau Ctrl+C. CSV (ditulis tiap sampel, aman bila
crash) dan PNG seluruh pengujian tersimpan di `Python/data/deepfryer_<tanggal>_<jam>.*`.
Satu baris per sampel suhu (1 Hz); waktu memakai jam ESP32 (`ms`) sehingga bebas jitter WiFi.
Kolom PID (firmware rev 16-17): `pid_aktif`, `pid_output_pct`, `setpoint_efektif_c`, `kp`, `ki`, `kd`
(gain yang SEDANG dipakai, berubah tiap detik saat Adaptive PID), `kp0`/`ki0`/`kd0` (gain dasar),
`pid_adaptif`, `lambda_p`/`lambda_i`/`lambda_d`,
`pid_umax_pct`. Grafik live: suhu vs setpoint vs setpoint efektif, dan sudut servo vs keluaran PID.
`suhu_c` adalah suhu tersaring firmware (Cascaded Filtering 3 tahap: Median-5 → Moving Average-5 → EMA, + hold saat sensor error);
`suhu_mentah_c`, `suhu_ditahan`, `sensor_error_total` mencatat bacaan mentah dan kondisi sensor.
Kolom `buzzer_aktif` sudah ada tetapi kosong karena
firmware belum mengirimnya.

## Alur tuning PID

1. Step test open-loop (PID mati, bukaan katup manual), rekam dengan logger Python.
2. `analisis_step_test('file.csv')` di MATLAB → model K, τ, L per titik kerja.
3. Gain PI SIMC yang aman di semua titik kerja (Ms terburuk ≤ 1.7).
4. Validasi di Simulink, lalu di alat.

Uji tanpa hardware: `uji_sintetis` di `Program/step_test/`.
