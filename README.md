# ThermoAdapt Fryer

**Kontrol suhu minyak deep fryer berbahan bakar gas dengan Adaptive PID berbasis ESP32.**
Termasuk firmware mikrokontroler, telemetri WebSocket, logger data Python, serta
identifikasi sistem (FOPDT) dan tuning PID robust (SIMC) di MATLAB.

> Status: prototipe riset. Bagian yang **belum terverifikasi di hardware** ditandai di
> komentar kode. Jangan sambungkan servo ke katup gas sebelum `SERVO_SAFETY` diaktifkan.

## Struktur

| Folder / file | Isi |
|---|---|
| `dep_frayer/dep_frayer.ino` | Firmware ESP32: WiFi AP + web + WebSocket, MAX31855, servo katup gas, motor DC Z (BTS7960), stepper X (TB6600), 4 limit switch, LCD ILI9488, buzzer |
| `Program/` | Model Simulink Adaptive PID + simulasi (parameter plant masih asumsi) |
| `Program/step_test/` | Identifikasi FOPDT dari data step test + tuning PI SIMC + cek robustness (Ms, PM, GM) |
| `pin_scan.txt` | Sketch diagnostik pin limit switch |
| `FRITZINK.fzz` | Rangkaian (Fritzing) |

## Perangkat keras

ESP32 DevKit V1 30-pin · thermocouple K + MAX31855 · servo 180° pada katup gas ·
motor DC + BTS7960 (sumbu Z) · stepper NEMA + TB6600 (sumbu X) · TFT ILI9488 · 4 limit switch.

## Alur tuning PID

1. Step test open-loop (PID mati, bukaan katup manual), rekam dengan logger Python.
2. `analisis_step_test('file.csv')` di MATLAB → model K, τ, L per titik kerja.
3. Gain PI SIMC yang aman di semua titik kerja (Ms terburuk ≤ 1.7).
4. Validasi di Simulink, lalu di alat.

Uji tanpa hardware: `uji_sintetis` di `Program/step_test/`.
