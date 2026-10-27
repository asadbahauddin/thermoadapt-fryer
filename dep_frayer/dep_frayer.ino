/*
 * DEEP FRYER CERDAS - MONITOR & KONTROL VIA WEB + LCD (rev 24)
 * ESP32 = WiFi Access Point (192.168.4.1) + WebSocket.
 * Isi: suhu MAX31855, 4 limit switch, Z = MOTOR DC (BTS7960), X = stepper (TB6600),
 *      SERVO katup gas 0-180 derajat (kalibrasi), BUZZER, LCD TFT mirror dari web.
 *
 * REV 24 (dari rev 23):
 *   - OVERSHOOT tidak lagi mematikan PID: suhu >= setpoint + PID_HOLD_C (10 C) -> katup ditahan
 *     TUTUP, integral 0, PID tetap ON; normal lagi saat suhu <= setpoint + 7 C. PID MATI hanya
 *     di batas akhir TEMP_CUTOFF_C (180 C), sensor error, atau STOP. JSON "temp" + "ph" (tahan).
 *   - Step test web: hasil ditandai TIDAK VALID (gain tidak ditampilkan) bila R^2 < 0.9,
 *     kenaikan < 10 C, kemiringan tidak konsisten, atau durasi < 3 x dead time.
 *   - Gain default sementara: Kp 1.46, Ki 0.0054 (Ti 270 dtk), lambda_i 0.
 *
 * REV 23 (dari rev 22):
 *   - Reset LCD otomatis tiap 30 dtk DIMATIKAN (LCD_REINIT_MS 0): pengguna menilai rev 21 lebih
 *     stabil. Diganti tombol "Reset LCD" di web (perintah "lcdr") yang menjalankan urutan sama:
 *     pin RESET -> tft.init() -> MAX31855 begin() -> gambar ulang (non-blocking, di uiTask).
 *     Pin RESET saja TIDAK cukup: setelah reset hardware ILI9488 kembali ke kondisi awal
 *     (sleep, format warna & rotasi hilang) sehingga harus diinisialisasi ulang.
 *   - Watchdog, diagnosa, dan pola non-blocking rev 22 tetap.
 *
 * REV 22 (dari rev 21): ROBUST
 *   - LCD di-reset (hardware) + diinisialisasi ulang tiap LCD_REINIT_MS (30 dtk), lalu semua
 *     elemen digambar ulang. MAX31855 di-begin() ulang (bus SPI dipakai bersama LCD).
 *     Sampling suhu tertunda ~0.4 dtk saat itu; layar berkedip sesaat.
 *   - WATCHDOG: loop() dan uiTask diawasi; macet > WDT_TIMEOUT_S (10 dtk) -> ESP restart
 *     sendiri. Setelah restart katup ke TUTUP, PID mati, motor diam (aman, tidak lanjut sendiri).
 *   - Alasan boot terakhir (POWER_ON / TASK_WDT / BROWNOUT / PANIC ...), heap minimum,
 *     sisa stack uiTask, dan jumlah reset LCD tampil di web (JSON "st": rr, hp, sk, lr).
 *   - IP di LCD diformat tanpa String (tidak memecah heap tiap 150 ms).
 *   - NON-BLOCKING / MULTITASKING KOOPERATIF / BLINK WITHOUT DELAY:
 *       core 1 loop(): web, WebSocket, stepper, step test, servo (20 ms), motor DC (10 ms),
 *                      buzzer (pola ms), PID (tiap sampel 1 Hz), status (250 ms); semua berbasis
 *                      millis(), tanpa delay().
 *       core 0 uiTask: servis LCD (state machine reset), sampel suhu (200 ms), LCD (150 ms),
 *                      reset LCD (30 dtk); vTaskDelay(10 ms) menyerahkan CPU tiap putaran.
 *     Reset LCD dulu delay(210 ms) -> sekarang state machine millis(). Sisa tunggu: delay
 *     internal tft.init() library (~295 ms, tiap 30 dtk, hanya di core 0) dan
 *     delayMicroseconds(2) waktu setup CS MAX31855 (syarat datasheet).
 *
 * REV 21 (dari rev 20):
 *   - FILTER SUHU diganti CASCADED FILTERING 3 TAHAP: Median-5 -> Moving Average-5 -> EMA
 *     (alpha 0.3), menggantikan tolak-lonjakan + Savitzky-Golay rev 15. HOLD saat fault tetap.
 *     Jeda ~1.3 dtk, std noise ~34%, lonjakan 1-2 sampel dibuang (lihat blok FILTER).
 *   - Servo default hasil kalibrasi: TUTUP -3 derajat, BUKA 55 derajat. PID Umax default 100%
 *     (seluruh rentang). Nilai di flash (bila pernah disimpan dari web) tetap diutamakan.
 *
 * REV 20 (dari rev 19): PERBAIKAN "mentok di 0 derajat"
 *   - Tombol -1/+1 dan kotak "Sudut" (sva/svad) dulu memakai setServoAngle() yang DIAM-DIAM
 *     mengunci ke TUTUP..BUKA, sehingga di 0 derajat -1 tidak berbuat apa-apa. Sekarang semua
 *     perintah manual memakai setServoAngleManual(): -30..180 derajat, dibatasi pulsa, dan
 *     memberi pesan bila ditolak. PID & step test tetap dikunci TUTUP..BUKA.
 *   - Halaman web dikirim dengan header no-cache (HP tidak memakai halaman lama).
 *   - Versi firmware (FW_REV) tampil di judul halaman web.
 *
 * REV 19 (dari rev 18): kalibrasi boleh SUDUT NEGATIF (sampai SERVO_CAL_MIN_DEG, dibatasi pulsa
 *   >= SERVO_US_ABS_MIN), mis. TUTUP = -3 derajat bila di 0 derajat katup masih bocor.
 *
 * REV 18 (dari rev 17): KALIBRASI KATUP lewat kotak angka (tanpa slider)
 *   - Web: sudut uji 0..180 per 1 derajat ("svk,<deg>", bebas dari kunci TUTUP..BUKA), simpan
 *     sebagai TUTUP/BUKA ("svset,c|o"), koreksi pulsa 0/180 derajat ("svcal,<us0>,<us180>").
 *     Tersimpan di flash dan DIMUAT saat boot (rev 13 sempat memaku 0/55).
 *     Syarat: TUTUP < BUKA, selisih >= 5 derajat. Ditolak saat PID aktif / step test.
 *   - Slider sudut dihapus; diganti kotak sudut + tombol -1/+1.
 *   - Keputusan pengguna: TUTUP = katup RAPAT (api PADAM), tidak ada sudut api minimum.
 *     [Hasil kalibrasi di alat: servo mentok di -3 derajat dan katup masih menyisakan api kecil yang
 *      tidak mampu menaikkan suhu -> TUTUP = api minimum (berfungsi seperti pilot), BUKAN padam.]
 *     PID 0%, STOP, PID OFF, akhir step test -> gas berhenti. Saat katup membuka lagi gas keluar
 *     TANPA api kecuali ada pemantik/pilot yang selalu menyala. Harus ada orang di dekat mesin.
 *
 * REV 17 (dari rev 16): ADAPTIVE PID (BELUM diuji di alat)
 *   - Hukum adaptasi sama dgn model Simulink: Kp = Kp0 + lambda_p|e|, Ki = Ki0 + lambda_i|e|,
 *     Kd = Kd0 + lambda_d|dT/dt|, dibatasi PID_KP_MAX/KI_MAX/KD_MAX (dicek Ms, lihat konfigurasi).
 *   - Web: ADAPTIF ON/OFF (OFF = PID biasa sbg pembanding), lambda, gain yang sedang dipakai.
 *     Perintah "pida,<0|1>", "pidl,<lp>,<li>,<ld>"; disimpan di flash.
 *   - JSON "temp" + kpe/kie/kde (gain DIPAKAI tiap sampel); "st" + ad, lp, li, ld.
 *     kp/ki/kd di "st" = gain DASAR (Kp0/Ki0/Kd0).
 *   - Tanpa deadband integral (sesuai keputusan pengguna).
 *
 * REV 16 (dari rev 15): PID SUHU (pertama, BELUM diuji di alat)
 *   - Kontrol PID suhu -> bukaan katup (0..Umax persen dari rentang TUTUP..BUKA), update tiap
 *     sampel suhu TERKIRIM (1 Hz, dari nilai tersaring rev 15; dt dari gTempMs). Web: kotak
 *     "PID Suhu" (ON/OFF, Kp, Ki, Kd, Umax; disimpan di flash). Setpoint: "sp,<C>".
 *     Bentuk: u = Kp*e + I + D, Ki dalam %/(C*dtk) (= Kp/Ti), D pada PENGUKURAN (terfilter).
 *     Default dari step test web: Kp 1.9, Ti ~34 dtk (Ki 0.056), Kd 0. TEBAKAN AWAL, bukan optimum.
 *   - Setpoint efektif naik bertahap (PID_SP_RAMP_C_S) mulai dari suhu saat PID dinyalakan.
 *   - Anti-windup (integrasi bersyarat + batas), start tanpa loncatan (bumpless).
 *   - Saat suhu DITAHAN (HELD, rev 15): integral & derivatif dibekukan, keluaran tetap.
 *   - PENGAMAN: PID mati + katup TUTUP bila sensor error > SENSOR_FAULT_MS, tidak ada sampel
 *     suhu baru > SENSOR_FAULT_MS (uiTask macet), suhu >= setpoint + PID_OVER_C atau
 *     >= TEMP_CUTOFF_C - 5, tombol STOP, atau step test berjalan. Servo manual ditolak saat PID
 *     aktif. [rev 18: TUTUP kini dikalibrasi RAPAT = api PADAM, lihat REV 18.] PID tidak bisa mendinginkan.
 *   - Setpoint dibatasi <= PID_SP_MAX_C (TEMP_CUTOFF_C - 15) selama PID aktif.
 *   - JSON "temp" + pe (PID aktif), pu (keluaran %), se (setpoint efektif);
 *     "st" + kp, ki, kd, pm (Umax %). ("um" TETAP pulsa servo 0 derajat; Umax sengaja "pm".)
 *   - Perintah web > 63 karakter dibalas "terlalu panjang" (dulu dibuang diam-diam di 48).
 *   - Harus ada orang di dekat mesin. Tidak ada E-stop fisik.
 *
 * REV 15 (dari rev 14):
 *   - BTS7960 sesuai wiring terbaru: GPIO27 = RPWM | GPIO26 = LPWM | GPIO14 = R_EN + L_EN
 *     (RPWM/LPWM sempat tertukar di awal rev 15: 26/27 dibalik setelah dicek pengguna).
 *     GPIO14 mengeluarkan PWM singkat saat boot; dcInit() dijalankan PERTAMA di setup() dan
 *     menahan EN LOW, tetapi selama bootloader (sebelum setup) firmware belum berjalan:
 *     WAJIB pull-down 10k ke GND di EN (14), RPWM (27), dan LPWM (26).
 *     DC_POS_USES_RPWM dikembalikan ke true. ARAH NAIK/TURUN HARUS DIUJI ULANG.
 *   - SENSOR SUHU: sampling 200 ms, tolak lonjakan > SENSOR_MAX_JUMP_C, HOLD nilai terakhir
 *     maks SENSOR_HOLD_MS saat error, lalu Savitzky-Golay kausal (11 titik, orde 2).
 *     gTemp = nilai tersaring. Pesan "temp" (tetap 1 Hz) ditambah "tr" (mentah), "held", "ec".
 *     SENSOR_FILTER_ENABLE 0 = perilaku lama (tanpa tolak lonjakan/hold/SG).
 *
 * REV 14 (dari rev 13):
 *   - STEP TEST open-loop (tampil di web saja): servo dipindah ke satu bukaan tetap dari
 *     TUTUP, suhu dicatat tiap 1 dtk oleh browser, lalu web menghitung kemiringan (R), dead
 *     time (L) dan USULAN awal Kp/Ki/Kd (Ziegler-Nichols kurva reaksi, dan PI konservatif).
 *     Perintah: "stt,<sudut>,<batasC>,<menit>" (mulai), "stx" (batal), tombol STOP juga membatalkan.
 *     Pengaman: berhenti + katup ke TUTUP bila suhu >= batas, sensor error >5 dtk, batas waktu,
 *     atau dibatalkan. Selama test, perintah servo manual ditolak. Ada api menyala TANPA PID:
 *     harus ada orang di dekat mesin. Tidak ada E-stop fisik.
 *   - Data hilang bila browser terputus/ditutup saat test (firmware tidak menyimpan sampel).
 *
 * REV 13 (dari rev 12):
 *   - LIMIT SWITCH: penamaan dipetakan ulang (hasil pengujian fisik: Z Atas/Z Bawah dan
 *     X Awal/X Akhir sebelumnya tertukar):
 *       GPIO17 = Z Atas | GPIO35 = Z Bawah | GPIO39 = X Awal | GPIO34 = X Akhir
 *     Perilaku pengaman tidak berubah (tiap pin tetap memblok arah yang sama): "naik" (+) memeriksa
 *     switch Z Atas, "+" X memeriksa switch X Akhir. Semua interlock AKTIF.
 *   - SERVO: hanya TUTUP (0) dan BUKA (55). Tombol TUTUP/BUKA, bukaan %, penyetelan halus.
 *     Kalibrasi/simpan sudut/tes pulsa dihapus dari web. Sudut dikunci 0..55 di firmware
 *     dan sudut tutup/buka dipaku (tidak diambil dari flash).
 *
 * REV 12 (dari rev 11):
 *   - SETPOINT suhu: diatur dari web (perintah "sp,<C>", 20-200 C), disimpan di flash,
 *     digambar putus-putus (oranye) di grafik suhu web. BELUM ada PID.
 *   - Pesan WebSocket "temp" ditambah "sp", "seq", "ms"; pesan "st" ditambah "sp".
 *
 * REV 11 (dari v10):
 *   - STEPPER Z DIHAPUS dari kode. Pin 26/27/14 sekarang milik BTS7960
 *       (pemetaan rev 11 di bawah KELIRU, sudah diganti di rev 15: 27 = RPWM, 26 = LPWM, 14 = EN):
 *       GPIO26 = EN (R_EN + L_EN disatukan) | GPIO27 = LPWM | GPIO14 = RPWM
 *     Satu-satunya stepper: X (STEP32, DIR33, EN25).
 *     Pull-down 10k di EN, LPWM, RPWM ke GND (GPIO14 mengeluarkan PWM sesaat saat boot;
 *     karena EN ada di GPIO26 dan ditarik LOW, motor tetap diam saat boot).
 *     [Alasan ini TIDAK berlaku lagi sejak rev 15: EN sekarang di GPIO14; lihat REV 15.]
 *   - SERVO 180 derajat, MODE KALIBRASI TANPA PEMBATAS (servo belum dipasang ke gas):
 *       * Slider/tombol sudut 0-180, tombol relatif +-1/+-5/+-10.
 *       * "Simpan sebagai TUTUP/BUKA": menyimpan sudut yang sedang dipakai ke flash.
 *       * Kalibrasi pulsa: us untuk 0 derajat dan 180 derajat (default 500/2500, BELUM
 *         terverifikasi; sesuaikan dgn servo-mu).
 *       * Tes pulsa mentah (us) untuk mencari ujung jangkauan servo.
 *       * Kunci pengaman (sensor error / suhu lewat batas) DIMATIKAN (SERVO_SAFETY false).
 *         Nyalakan lagi sebelum servo terhubung ke gas!
 *       * Tanpa batas laju (SERVO_SLEW_DEG_S = 0): servo melompat ke target. Lonjakan arus
 *         servo bisa mengganggu rel 5V / pembacaan suhu.
 *       * HATI-HATI: pulsa di luar jangkauan mekanik servo membuat servo menekan stopper
 *         (panas, bunyi dengung, bisa rusak). Cari ujung jangkauan pelan-pelan dgn tes pulsa
 *         mentah, jangan langsung ke nilai ekstrem.
 *   - Motor DC Z: duty dibatasi MOTOR_RATED_V / MOTOR_SUPPLY_V = 12/24 = 50% ("100%" di web =
 *     rata-rata 12 V; motor tetap melihat pulsa 24 V). PWM 20 kHz, naik bertahap, batas waktu
 *     gerak 8 s, berhenti seketika bila limit kena (BENCH_MODE false & limit interlock).
 *   - LCD: reset manual ke GPIO15 sebelum tft.init(). User_Setup.h: TFT_RST -1.
 *   - Buzzer aktif di GPIO2 (via BC547). Bip pendek saat boot, kotak "Cek Buzzer" di web.
 *
 * LIBRARY: TFT_eSPI (Bodmer), Adafruit MAX31855 (+BusIO), WebSockets (Markus Sattler),
 *          AccelStepper. WiFi, WebServer, Preferences bawaan core.
 * User_Setup.h: lihat User_Setup_DeepFryer.txt.
 *
 * PERINGATAN:
 *   - BENCH_MODE true: interlock limit & batas jog NONAKTIF. WAJIB false sebelum motor
 *     dipasang ke mekanik.
 *   - Belum ada E-stop fisik. Firmware ini belum dikompilasi/diuji di hardware oleh penulisnya.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <SPI.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <Adafruit_MAX31855.h>
#include <AccelStepper.h>
#include <esp_task_wdt.h>         // rev 22: watchdog (ESP restart sendiri bila macet)

#define FW_REV "rev 24"     // tampil di judul halaman web: cek apakah firmware terbaru sudah ter-upload

// ---- ROBUST (rev 22) ----
#define LCD_REINIT_MS   0UL       // rev 23: reset LCD otomatis MATI (rev 22 tiap 30 dtk terasa kurang stabil);
                                  // reset manual lewat tombol "Reset LCD" di web. >0 = reset otomatis tiap N ms
#define WDT_TIMEOUT_S   10        // loop() atau uiTask macet > 10 dtk -> ESP restart
                                  // (setelah restart: katup ke TUTUP, PID mati, motor diam)

// ---------------- PIN ----------------
#define PIN_MAX_SCK 18
#define PIN_MAX_CS  5
#define PIN_MAX_SO  19

#define PIN_X_STEP 32
#define PIN_X_DIR  33
#define PIN_X_EN   25

// BTS7960 (rev 15, wiring terbaru; dicek kontinuitas oleh pengguna).
// Pull-down 10k ke GND di ketiga pin: GPIO14 mengeluarkan PWM singkat saat boot.
#define PIN_DC_RPWM 27
#define PIN_DC_LPWM 26
#define PIN_DC_EN   14     // R_EN + L_EN BTS7960 (disatukan)

#define PIN_SERVO   13
#define PIN_TFT_RST 15     // RESET modul layar (dipulsa manual di uiTask)
#define PIN_BUZZ     2     // buzzer aktif via BC547 (GPIO2 = pin strapping)
#define BUZZER_ACTIVE_HIGH true

// ---------------- LIMIT SWITCH (TABEL KONFIGURASI) ----------------
// activeHigh = true : HIGH berarti KENA. false : LOW berarti KENA (bebas = HIGH).
// verified   = true : tampil normal. false = kuning (?)
// interlock  = true : dipakai sebagai pengaman gerak (hanya jika BENCH_MODE false)
enum { LS_ZATAS = 0, LS_ZBAWAH, LS_XAWAL, LS_XAKHIR, LS_COUNT };

struct LimitCfg { const char* name; uint8_t pin; bool activeHigh; bool verified; bool interlock; };
LimitCfg LS[LS_COUNT] = {
  // PEMETAAN (rev 13, hasil pengujian fisik): GPIO17 = Z Atas | GPIO35 = Z Bawah |
  //                                           GPIO39 = X Awal | GPIO34 = X Akhir
  // interlock = true: gerak berhenti saat limit di arah geraknya kena. Semua AKTIF.
  // Jika satu switch ternyata tidak bereaksi/terbalik, set interlock false untuk pin itu.
  { "Z Atas",  17, false, true, true },
  { "Z Bawah", 35, false, true, true },
  { "X Awal",  39, false, true, true },
  { "X Akhir", 34, false, true, true },
};

// ---------------- KONFIGURASI ----------------
const char* AP_SSID = "DeepFryer-ESP32";
const char* AP_PASS = "fryer1234";             // minimal 8 karakter
IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GW(192, 168, 4, 1);
IPAddress AP_MASK(255, 255, 255, 0);

#define BENCH_MODE             false   // false = interlock limit AKTIF (Z DC berhenti di Z Atas/Z Bawah; X jog tanpa limit terverifikasi dibatasi 200 step). true = meja uji tanpa pengaman
#define EN_HIGH_MEANS_ENABLED  false   // TB6600 (X): GPIO LOW = driver AKTIF (hasil ukur)
#define Z_PLUS_TOWARD_ATAS     true    // arah + Z (naik) menuju limit "atas"
#define X_PLUS_TOWARD_AKHIR    true    // arah + (KANAN) menuju switch GPIO34 ("X Akhir" setelah pemetaan ulang)
#define X_DIR_INVERT           false

#define MAX_JOG_STEPS        2000000000L
#define MAX_JOG_UNVERIFIED   200
#define DEFAULT_SPEED_X  3000
#define DEFAULT_ACCEL_X  3000
#define MIN_SPEED 50
#define MAX_SPEED 10000
#define MIN_ACCEL 50
#define MAX_ACCEL 12000

// ---- MOTOR DC Z (BTS7960) ----
#define MOTOR_SUPPLY_V      24.0f    // tegangan catu modul BTS7960 (ukur dgn multimeter)
#define MOTOR_RATED_V       12.0f    // tegangan motor; duty maks = RATED / SUPPLY
#define DC_POS_USES_RPWM    true     // positif (naik) memakai RPWM (GPIO27); false = LPWM (GPIO26)
                                     // Dikembalikan ke default (rev 15): koreksi "false" sebelumnya
                                     // dibuat saat pin masih keliru. ARAH BELUM TERVERIFIKASI: uji
                                     // "Naik 25%" sambil tangan di saklar catu daya; bila terbalik
                                     // ubah ke false. Interlock limit Z bergantung pada arah ini.
#define DC_PWM_FREQ         20000
#define DC_PWM_BITS         10
#define DC_LEDC_CH_R        2        // hanya core ESP32 v2 (servo memakai kanal 0)
#define DC_LEDC_CH_L        3
#define DC_RAMP_PCT_S       100.0f   // laju naik/turun keluaran (persen per detik)
#define DC_MAX_PCT          100      // batas perintah motor DC Z (persen web); belum diuji dengan beban, turunkan ke 50 bila perlu
#define DC_MAX_RUN_MS       8000UL   // batas waktu 1x gerak
#define DC_BRAKE_WHEN_IDLE  true     // true: EN HIGH saat diam (rem); false: EN LOW (bebas)

// ---- SERVO / KATUP (kalibrasi, tanpa pembatas) ----
#define SERVO_FREQ_HZ           50
#define SERVO_PWM_BITS          14
#define SERVO_LEDC_CH           0        // hanya core ESP32 v2
#define SERVO_US_ABS_MIN        300      // batas kewajaran input pulsa mentah
#define SERVO_US_ABS_MAX        3000
#define DEFAULT_SERVO_US_MIN    500      // us untuk 0 derajat   (BELUM terverifikasi)
#define DEFAULT_SERVO_US_MAX    2500     // us untuk 180 derajat (BELUM terverifikasi)
#define DEFAULT_SERVO_CLOSED_DEG -3      // rev 21: TUTUP hasil kalibrasi di alat = -3 derajat
                                         // (servo mentok; api sangat kecil yang tidak menaikkan suhu)
#define DEFAULT_SERVO_OPEN_DEG   55      // sudut katup BUKA  (hasil kalibrasi: membuka di 55 derajat)
#define SERVO_SLEW_DEG_S        0.0f     // 0 = tanpa batas laju (melompat ke target)
#define SERVO_MIN_SPAN_DEG      5        // rev 18: BUKA - TUTUP minimal (derajat)
#define SERVO_CAL_MIN_DEG       -30      // rev 19: kalibrasi boleh di bawah 0 derajat (sudut negatif =
                                         // pulsa < pulsa 0 derajat, mis. -1 = 500 - 11 = ~489 us).
                                         // Batas sebenarnya: pulsa >= SERVO_US_ABS_MIN (~ -18 derajat default)

#define SERVO_SAFETY           false    // true: paksa katup ke sudut TUTUP bila sensor error/suhu lewat batas
#define SENSOR_FAULT_MS        5000UL
#define TEMP_CUTOFF_C          180.0f   // NILAI AWAL: cek titik asap minyak

// ---- PENANGANAN SENSOR SUHU (rev 15) ----
#define SENSOR_FILTER_ENABLE   1        // 0 = perilaku lama: gTemp = bacaan mentah (tanpa filter cascade/hold)
#define SENSOR_SAMPLE_MS       200UL    // periode baca MAX31855 (konversi internal ~100 ms)
#define SENSOR_PUBLISH_EVERY   5        // kirim ke web/logger & grafik LCD tiap 5 sampel = 1 Hz (seperti sebelumnya)
                                        // (minyak fisik tidak berubah 5 C dalam 200 ms)
#define SENSOR_HOLD_MS         3000UL   // tahan nilai tersaring terakhir maks selama ini saat error
static_assert(SENSOR_HOLD_MS < SENSOR_FAULT_MS, "SENSOR_HOLD_MS harus < SENSOR_FAULT_MS (pengaman sensor)");

// ---- PID SUHU (rev 16) ----
#define PID_DEFAULT_KP     1.46f     // rev 24: PI konservatif step test 28 Okt (Ti 270 dtk). SEMENTARA:
#define PID_DEFAULT_KI     0.0054f   //   step test itu R^2 0.296 (tidak valid); ulangi step test lebih
                                     //   besar & lama. Ki lama 0.056 (Ti 34 dtk) terlalu cepat utk L ~80 dtk.
#define PID_DEFAULT_KD     0.0f      // mulai 0: derivatif memperbesar noise sensor
#define PID_DEFAULT_UMAX   100.0f    // rev 21: PID boleh memakai seluruh rentang TUTUP(-3)..BUKA(55)
                                     // (40% hanya sampai ~20 derajat). Bisa diturunkan dari web.
#define PID_SP_RAMP_C_S    0.15f     // laju maks perubahan setpoint efektif (C/dtk) = 9 C/menit
                                     // (simulasi model dugaan: ramp 0.30 -> overshoot ~7 C, 0.15 -> ~4 C)
#define PID_HOLD_C         10.0f     // rev 24: suhu >= setpoint + ini -> katup TUTUP, PID TETAP ON
#define PID_HOLD_HYST_C    3.0f      //   keluar dari tahan bila suhu <= setpoint + HOLD - HYST
                                     // PID baru MATI total bila suhu >= TEMP_CUTOFF_C (batas akhir)
#define PID_SP_MAX_C       (TEMP_CUTOFF_C - 15.0f)
#define PID_D_FILTER       0.8f      // filter derivatif orde-1 (0..1; makin besar makin halus)

// ---- ADAPTIVE PID (rev 17), hukum adaptasi SAMA dgn model Simulink (build_deep_fryer_model.m) ----
//   Kp(k) = min(Kp0 + lambda_p*|e(k)|,     max(Kp0, PID_KP_MAX))
//   Ki(k) = min(Ki0 + lambda_i*|e(k)|,     max(Ki0, PID_KI_MAX))
//   Kd(k) = min(Kd0 + lambda_d*|dT/dt|,    max(Kd0, PID_KD_MAX))   (dT/dt terfilter, C/dtk)
// e = setpoint EFEKTIF - suhu. Adaptif OFF (atau semua lambda = 0) = PID biasa (pembanding).
// Batas "max(K0, K_MAX)": adaptasi tidak pernah MENURUNKAN gain di bawah nilai dasar.
// Integral disimpan dalam satuan keluaran (I += Ki(k)*e*dt), BUKAN Ki(k)*sum(e) seperti di
// Simulink: perubahan Ki tidak menimbulkan loncatan keluaran (bentuk Simulink terbukti memicu
// umpan balik positif, lihat catatan FIX #2 di model).
// BATAS (dicek dgn Ms, model FOPDT dugaan K 1.9-4 C/%, tau 180-600 s, L 10-20 s, Ts 1 s):
//   Kp 6 / Ki 0.2 -> Ms 2.6..125 (TIDAK aman). Kp 3 / Ki 0.065 / Kd 5 -> Ms tidak lebih buruk
//   dari gain dasar (1.9/0.056). Ki paling cepat merusak kestabilan -> hampir tidak diadaptasi.
//   Catatan: gain dasar sendiri Ms 2.9-3.8 bila dead time ternyata ~20 s -> verifikasi step test.
#define PID_ADAPT_DEFAULT  true
#define PID_DEFAULT_LP     0.06f     // %/(C*C):  Kp = 1.9 -> 3.0 pada |e| ~ 18 C
#define PID_DEFAULT_LI     0.0f      // rev 24: Ki TIDAK diadaptasi (dead time ~80 dtk: Ki besar = overshoot)
#define PID_DEFAULT_LD     5.0f      // Kd naik saat suhu berubah cepat (|dT/dt| 1 C/dtk -> +5)
#define PID_KP_MAX         3.0f
#define PID_KI_MAX         0.065f
#define PID_KD_MAX         5.0f

// ---- SETPOINT SUHU (rev 12) ----
// Saat ini HANYA referensi untuk grafik web & logger; belum ada PID yang memakainya.
#define DEFAULT_SETPOINT_C     160.0f
#define SETPOINT_MIN_C          20.0f
#define SETPOINT_MAX_C         200.0f

// ---------------- OBJEK ----------------
Adafruit_MAX31855 thermocouple(PIN_MAX_CS);
TFT_eSPI tft = TFT_eSPI();
AccelStepper stepX(AccelStepper::DRIVER, PIN_X_STEP, PIN_X_DIR);
WebServer server(80);
WebSocketsServer ws(81);
Preferences prefs;

volatile float gTemp = NAN;
volatile bool  gTempFresh = false;
volatile uint32_t gLastValidMs = 0;
char gMsg[64] = "";
volatile uint8_t gFault = 0;
bool driversEnabled = false;                 // driver stepper X
int  spdX = DEFAULT_SPEED_X, accX = DEFAULT_ACCEL_X;
unsigned long lastStatus = 0;

volatile uint32_t gRaw = 0;
volatile uint32_t gTempSeq = 0;              // nomor urut sampel suhu (deteksi paket hilang)
volatile uint32_t gTempMs  = 0;              // millis() saat sampel suhu diambil
volatile float gSetpoint = DEFAULT_SETPOINT_C;
volatile float gTempRaw = NAN;               // bacaan mentah MAX31855 terakhir (NAN bila fault)
volatile bool  gTempHeld = false;            // true: gTemp = nilai ditahan (sensor error sesaat)
volatile uint32_t gErrCount = 0;             // jumlah sampel tidak valid (fault/lonjakan) sejak boot
volatile bool  pidOn = false;                // PID suhu aktif (rev 16)
volatile bool  gUiForce = false;             // rev 22: paksa gambar ulang semua elemen LCD
volatile uint32_t gLcdResets = 0;            // rev 22: jumlah reset LCD sejak boot
volatile bool  gLcdResetReq = false;         // rev 23: permintaan reset LCD manual dari web (loop -> uiTask)
TaskHandle_t   gUiTask = nullptr;            // rev 22: untuk cek sisa stack uiTask
char gResetWhy[24] = "?";                    // rev 22: alasan reset/boot terakhir

// Snapshot sampel TERKIRIM (1 Hz) untuk PID (rev 16). Ditulis uiTask (core 0), dibaca loop()
// (core 1). volatile menjamin tiap nilai 32-bit utuh, tapi TIDAK menjamin suhu/waktu/flag
// berasal dari sampel yang sama -> dibaca/ditulis bersama di dalam critical section.
portMUX_TYPE gSnapMux = portMUX_INITIALIZER_UNLOCKED;
struct TempSnap { float t; uint32_t ms; bool held; };
TempSnap gPub = { NAN, 0, false };
TempSnap readPubSnap() {
  portENTER_CRITICAL(&gSnapMux);
  TempSnap s = gPub;
  portEXIT_CRITICAL(&gSnapMux);
  return s;
}

// servo state
int   svUsMin = DEFAULT_SERVO_US_MIN;
int   svUsMax = DEFAULT_SERVO_US_MAX;
int   svClosedDeg = DEFAULT_SERVO_CLOSED_DEG;
int   svOpenDeg   = DEFAULT_SERVO_OPEN_DEG;
volatile float svTargetDeg = DEFAULT_SERVO_CLOSED_DEG;
volatile float svCurDeg    = DEFAULT_SERVO_CLOSED_DEG;
volatile int   svRawUs     = 0;              // >0: mode tes pulsa mentah
volatile uint8_t svLock    = 0;              // 0 normal, 1 sensor error, 2 suhu lewat batas

// Baca frame mentah 32-bit MAX31855 (bus SPI bersama; hanya dari uiTask)
uint32_t maxReadRaw() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_MAX_CS, LOW);
  delayMicroseconds(2);
  uint32_t v = 0;
  for (int i = 0; i < 4; i++) v = (v << 8) | SPI.transfer(0x00);
  digitalWrite(PIN_MAX_CS, HIGH);
  SPI.endTransaction();
  return v;
}

const char* rawDiag(uint32_t v) {
  if (v == 0x00000000UL) return "SO stuck LOW: cek VCC 3V3, GND, CS5, SO19";
  if (v == 0xFFFFFFFFUL) return "SO stuck HIGH/mengambang: SO19 atau CS5 tdk terhubung";
  if (v & 0x00020008UL)  return "frame tidak valid (bit17/bit3=1): noise atau wiring buruk";
  if (v & 0x00010000UL) {
    if (v & 1) return "FAULT: thermocouple LEPAS (open)";
    if (v & 2) return "FAULT: thermocouple short ke GND";
    if (v & 4) return "FAULT: thermocouple short ke VCC";
    return "FAULT (bit16=1)";
  }
  return "frame valid";
}

// ---------------- HALAMAN WEB ----------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Deep Fryer Monitor</title>
<style>
  body{font-family:system-ui,Arial,sans-serif;background:#111;color:#eee;margin:0;padding:12px;text-align:center}
  h1{font-size:18px;margin:4px 0;color:#aaa}
  h2{font-size:14px;margin:0 0 8px;color:#aaa;text-transform:uppercase;letter-spacing:1px}
  .box{background:#1b1b1b;border-radius:8px;padding:12px;margin:10px auto;max-width:640px}
  #temp{font-size:64px;font-weight:700;margin:4px 0}
  .ok{color:#4ade80}.err{color:#f87171}
  canvas{width:100%;height:160px;background:#151515;border-radius:6px}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:8px}
  .led{padding:8px;border-radius:6px;font-size:13px;background:#222}
  .led b{display:block;font-size:12px;margin-top:4px}
  .led i{display:block;font-size:11px;color:#999;font-style:normal;margin-top:2px}
  .on{background:#4a1515;color:#f87171}.off{background:#12301c;color:#4ade80}.unk{background:#3a3210;color:#fbbf24}
  button{background:#2a2a2a;color:#eee;border:1px solid #444;border-radius:6px;padding:10px 6px;font-size:14px;margin:3px;min-width:56px}
  button:active{background:#444}
  .stop{background:#7f1d1d;border-color:#ef4444;font-weight:700;width:90%;padding:14px}
  .pos{font-size:20px;margin:6px 0}
  input{width:70px;background:#111;color:#eee;border:1px solid #444;border-radius:4px;padding:6px}
  input[type=range]{width:85%;padding:0}
  #log{font-size:12px;color:#fbbf24;min-height:16px;margin-top:6px}
  small{color:#777}
</style>
</head>
<body>
  <h1>Deep Fryer - Monitor &amp; Kontrol <small id="fw" style="color:#777">(firmware ?)</small></h1>
  <div id="bench" class="err" style="font-weight:700"></div>

  <div class="box">
    <h2>Suhu Minyak</h2>
    <div id="temp">--.- &deg;C</div>
    <div id="status" class="err">Menghubungkan...</div>
    <canvas id="c" width="640" height="160"></canvas>
    <div><small><span style="color:#4ade80">&#9473; suhu</span> &nbsp;
      <span style="color:#f59e0b">&#9476; setpoint</span></small></div>
    <div style="margin-top:6px">Setpoint: <b id="spv">--</b> &deg;C &nbsp;
      <input id="spin" type="text" inputmode="decimal" placeholder="mis. 160"
             onkeydown="if(event.key==='Enter')sendSp()">
      <button onclick="sendSp()">Set</button></div>
    <div><small>Cara uji: isi setpoint &rarr; Set &rarr; klik <b>MULAI KONTROL</b> di kotak PID Suhu.</small></div>
    <small>Setpoint dipakai PID (maks 165 &deg;C saat PID aktif) dan sebagai referensi grafik.</small>
  </div>

  <div class="box">
    <h2>PID Suhu</h2>
    <div class="pos"><span id="pids" class="err">PID mati</span>
      <small>&nbsp; keluaran <b id="pidu">0.0</b>% &nbsp; setpoint efektif <b id="pide">--</b> &deg;C</small></div>
    <button onclick="send('pid,1')" style="font-size:1.1em">MULAI KONTROL (PID ON)</button>
    <button onclick="send('pid,0')" style="font-size:1.1em">BERHENTI KONTROL (PID OFF)</button>
    <button class="stop" onclick="send('stop')" style="width:auto">STOP</button>
    <div style="margin-top:8px">
      Kp <input id="pkp" type="number" step="0.1" min="0" max="50">
      Ki <input id="pki" type="number" step="0.001" min="0" max="5">
      Kd <input id="pkd" type="number" step="0.1" min="0" max="200">
      Umax <input id="pum" type="number" step="1" min="5" max="100"> %
      <button onclick="pidSet()">Simpan</button></div>
    <div style="margin-top:8px">Mode: <b id="pada">--</b>
      <button onclick="send('pida,1')">ADAPTIF ON</button>
      <button onclick="send('pida,0')">ADAPTIF OFF (PID biasa)</button></div>
    <div style="margin-top:6px">
      &lambda;p <input id="plp" type="number" step="0.01" min="0" max="1">
      &lambda;i <input id="pli" type="number" step="0.0001" min="0" max="0.01">
      &lambda;d <input id="pld" type="number" step="0.5" min="0" max="50">
      <button onclick="pidLam()">Simpan &lambda;</button></div>
    <div style="margin-top:6px">Gain dipakai: Kp <b id="kpe">--</b> &nbsp; Ki <b id="kie">--</b> &nbsp; Kd <b id="kde">--</b></div>
    <small>Adaptive PID: Kp = Kp0 + &lambda;p&middot;|e|, Ki = Ki0 + &lambda;i&middot;|e|, Kd = Kd0 + &lambda;d&middot;|dT/dt|,
    dibatasi Kp &le; 3, Ki &le; 0,065, Kd &le; 5 (atau nilai dasar bila lebih besar). Kp/Ki/Kd di atas = nilai DASAR.</small><br>
    <small>u = Kp&middot;e + &int;Ki&middot;e + D (D pada pengukuran). Satuan: % bukaan per &deg;C. Ki = Kp/Ti.
    Setpoint diatur di kotak Suhu Minyak. Setpoint efektif naik bertahap 0,15 &deg;C/dtk.
    Suhu &ge; setpoint+10 &deg;C: katup DITAHAN TUTUP, PID tetap ON (lanjut normal saat turun 3 &deg;C).
    PID MATI + katup TUTUP bila: sensor error &gt;5 dtk, suhu &ge; 180 &deg;C, atau STOP.
    TUTUP = sudut terkecil hasil kalibrasi (di alat ini masih api kecil, bukan padam). Servo BUKAN pemutus gas:
    mematikan api tetap lewat keran/solenoid. Harus ada orang di dekat mesin.</small>
  </div>

  <div class="box">
    <h2>Servo Katup Gas</h2>
    <div class="pos">Sudut: <span id="sva">0</span>&deg;
      <small>(target <span id="svg">0</span>&deg; | &asymp; <span id="svp">0</span>% bukaan <span id="svr"></span>)</small><br>
      <small>Pulsa dikirim ke servo: <b id="svus">-</b> &micro;s (bila angka ini turun tapi servo tidak bergerak,
      servo sudah mentok secara mekanis)</small></div>
    <div id="svl" class="err" style="font-weight:700;min-height:18px"></div>
    <button onclick="send('sv,0')" style="font-size:1.2em">TUTUP (<span class="lcd">0</span>&deg;)</button>
    <button onclick="send('sv,100')" style="font-size:1.2em">BUKA (<span class="lod">55</span>&deg;)</button><br>
    <div style="margin-top:8px">Bukaan:
      <button onclick="send('sv,0')">0%</button>
      <button onclick="send('sv,25')">25%</button>
      <button onclick="send('sv,50')">50%</button>
      <button onclick="send('sv,75')">75%</button>
      <button onclick="send('sv,100')">100%</button></div>
    <div style="margin-top:8px">Sudut: <input id="sva_in" type="text" inputmode="text" placeholder="derajat"
        onkeydown="if(event.key==='Enter')svAngle()"> &deg; <button onclick="svAngle()">Gerak</button>
      &nbsp; <button onclick="send('svad,-1')">-1&deg;</button><button onclick="send('svad,1')">+1&deg;</button></div>
    <small>Manual (kotak sudut, -1/+1): bebas -30..180&deg; (dibatasi pulsa servo 300-3000 &micro;s).
    PID &amp; step test dikunci antara TUTUP (<span class="lcd">0</span>&deg;) dan BUKA (<span class="lod">55</span>&deg;).</small>

    <div style="margin-top:12px;border-top:1px solid #333;padding-top:8px;text-align:left">
      <b>Kalibrasi katup</b> (PID harus OFF, step test tidak berjalan)<br>
      Sudut uji: <input id="svk_in" type="text" inputmode="text" placeholder="-30..180"
        onkeydown="if(event.key==='Enter')svCalGo(0)"> &deg;
      <button onclick="svCalGo(0)">Gerak</button>
      <button onclick="svCalGo(-1)">-1&deg;</button><button onclick="svCalGo(1)">+1&deg;</button><br>
      <button onclick="send('svset,c')">Simpan sudut ini sbg TUTUP</button>
      <button onclick="send('svset,o')">Simpan sudut ini sbg BUKA</button><br>
      Tersimpan: TUTUP <b id="svcd">-</b>&deg;, BUKA <b id="svod">-</b>&deg;<br>
      Pulsa servo: 0&deg; = <input id="sumin" type="text" inputmode="numeric"> &micro;s,
      180&deg; = <input id="sumax" type="text" inputmode="numeric"> &micro;s
      <button onclick="svCalPulse()">Simpan pulsa</button><br>
      <small>Cara: gerakkan per 1&deg; ke arah menutup sampai api sekecil mungkin, lalu "Simpan sbg TUTUP".
      Bila di 0&deg; masih besar, lanjut ke -1&deg;, -2&deg;, ... (sudut negatif = pulsa di bawah pulsa 0&deg;; batas
      sekitar -18&deg; dgn pulsa default). Berhenti di sudut terakhir yang servonya masih DIAM; bila berdengung/menekan
      stopper, naikkan 1&deg;. Api kecil di TUTUP harus tidak mampu menaikkan suhu (uji: minyak panas, TUTUP
      10 menit, suhu turun/diam). <b>PERINGATAN:</b> servo BUKAN pemutus gas; bila api di TUTUP sampai padam,
      gas keluar TANPA api saat katup membuka lagi. Harus ada orang di dekat mesin.</small>
    </div>
  </div>

  <div class="box">
    <h2>Step Test (open-loop)</h2>
    <div>Bukaan: <input id="stang" type="number" value="30" min="1" max="55" step="1"> &deg; (maks <span class="lod">55</span>)
      &nbsp; Batas suhu: <input id="stmax" type="number" value="170" min="60" max="175"> &deg;C
      &nbsp; Durasi: <input id="stdur" type="number" value="20" min="1" max="60"> menit</div>
    <button onclick="startStep()" style="font-size:1.1em">MULAI TEST</button>
    <button class="stop" onclick="send('stx')" style="width:auto">BATAL</button>
    <div id="sts" style="margin:6px 0">Siap.</div>
    <canvas id="sc" width="640" height="200"></canvas>
    <div id="sres" style="text-align:left;margin-top:6px;font-size:14px"></div>
    <small>Mulai dari minyak DINGIN dan stabil. Katup dibuka ke bukaan tetap; suhu direkam tiap 1 dtk
    oleh browser (jangan tutup/refresh halaman). Berhenti sendiri bila suhu mencapai batas atau waktu habis.
    Api menyala TANPA PID: harus ada orang di dekat mesin. Hasil Kp/Ki/Kd hanya USULAN awal.</small>
  </div>

  <div class="box" id="dcbox">
    <h2>Motor DC Z (BTS7960)</h2>
    <div class="pos">Keluaran: <span id="dcc">0</span>% <small>(target <span id="dct">0</span>%, rata-rata &asymp; <span id="dcv">0.0</span> V)</small></div>
    <input id="dcs" type="range" min="-100" max="100" value="0"
           oninput="$('dcsv').textContent=this.value" onchange="send('dcz,'+this.value)">
    <div>Slider: <span id="dcsv">0</span>% (positif = naik, negatif = turun)</div>
    <button onclick="send('dcz,-100')">Turun 100%</button>
    <button onclick="send('dcz,-50')">Turun 50%</button>
    <button onclick="send('dcz,-25')">Turun 25%</button><br>
    <button onclick="send('dcz,25')">Naik 25%</button>
    <button onclick="send('dcz,50')">Naik 50%</button>
    <button onclick="send('dcz,100')">Naik 100%</button><br>
    <button onclick="send('dcz,0')">Berhenti halus</button>
    <button class="stop" onclick="send('stop')">STOP SEKETIKA</button><br>
    <small>100% = rata-rata 12 V (duty dibatasi di firmware). Berhenti otomatis saat limit kena
    atau lewat batas waktu.<br>Pin: RPWM GPIO27, LPWM GPIO26, EN GPIO14 (arah belum diverifikasi).</small>
  </div>

  <div class="box">
    <h2>Cek Buzzer</h2>
    <button onclick="send('bz,1')">Bip pendek</button>
    <button onclick="send('bz,2')">Bip ganda</button>
    <button onclick="send('bz,3')">Bip panjang</button>
    <button onclick="send('bz,4')">Tes urutan</button>
    <button onclick="send('bz,0')">Diam</button><br>
    <small>Tes urutan: 3 bip pendek lalu 1 panjang.</small>
  </div>

  <div class="box">
    <h2>Limit Switch</h2>
    <div class="grid" id="leds"></div>
    <small>Kuning = belum terverifikasi (hanya tampil, tidak jadi pengaman).<br>
    GPIO mentah (H/L) ditampilkan untuk membantu identifikasi switch.</small>
  </div>

  <div class="box">
    <h2>Driver &amp; Kecepatan Stepper X</h2>
    <button id="enbtn" onclick="toggleEn()">Driver X: ?</button><br>
    <div>X: speed <input id="spdX" type="number" min="50" max="10000"> step/s
    accel <input id="accX" type="number" min="50" max="12000">
    <button onclick="setSA('X')">Set X</button></div>
  </div>

  <div id="cards"></div>

  <div class="box">
    <button class="stop" onclick="send('stop')">STOP SEMUA GERAK</button>
    <div id="log"></div>
  </div>
  <small id="info"></small><br>
  <small id="diag"></small><br>
  <button onclick="send('lcdr')">Reset LCD</button>
  <small>Pin RESET LCD + inisialisasi ulang + gambar ulang (~0,5 dtk). Pakai bila layar putih/membeku.</small>

<script>
  const MAXPTS = 120, data = [];
  const $ = id => document.getElementById(id);
  const cv = $('c'), g = cv.getContext('2d');
  let sock = null, enState = 0, ledsBuilt = false;

  function send(m){ if (sock && sock.readyState === 1) sock.send(m); else $('log').textContent='Belum terhubung'; }
  function setSA(ax){ send('spd,'+ax+','+$('spd'+ax).value); send('acc,'+ax+','+$('acc'+ax).value); clean(['spd'+ax,'acc'+ax]); }
  function toggleEn(){ send('en,' + (enState ? 0 : 1)); }
  function pidSet(){
    const k = ['pkp','pki','pkd','pum'].map(id => parseFloat($(id).value));
    if (k.some(x => !isFinite(x))) { $('log').textContent = 'Parameter PID tidak valid'; return; }
    // dibulatkan supaya perintah tidak melebihi panjang maksimum di firmware
    send('pidk,' + k[0].toFixed(3) + ',' + k[1].toFixed(5) + ',' + k[2].toFixed(3) + ',' + k[3].toFixed(1));
    clean(['pkp','pki','pkd','pum']);
  }
  // ---- servo & kalibrasi (rev 18) ----
  let svCd = 0, svOd = 55, svSg = 0;
  function num(id){ return parseFloat(String($(id).value).replace(',', '.')); }
  function svAngle(){
    const d = Math.round(num('sva_in'));
    if (!isFinite(d)) { $('log').textContent = 'Sudut tidak valid'; return; }
    send('sva,' + d); clean(['sva_in']);
  }
  function svCalGo(step){
    const now = Math.round(num('svk_in'));                 // dasar = isi kotak (langsung terbarui tiap klik)
    let d = step === 0 ? now : (isFinite(now) ? now : svSg) + step;
    if (!isFinite(d) || d < -30 || d > 180) { $('log').textContent = 'Sudut uji harus -30..180'; return; }
    $('svk_in').value = d; clean(['svk_in']);
    send('svk,' + d);
  }
  function svCalPulse(){
    const a = Math.round(num('sumin')), b = Math.round(num('sumax'));
    if (!isFinite(a) || !isFinite(b)) { $('log').textContent = 'Pulsa tidak valid'; return; }
    send('svcal,' + a + ',' + b); clean(['sumin','sumax']);
  }
  function pidLam(){
    const l = ['plp','pli','pld'].map(id => parseFloat($(id).value));
    if (l.some(x => !isFinite(x))) { $('log').textContent = 'Lambda tidak valid'; return; }
    send('pidl,' + l[0].toFixed(4) + ',' + l[1].toFixed(5) + ',' + l[2].toFixed(3));
    clean(['plp','pli','pld']);
  }
  function sendSp(){
    const v = parseFloat(String($('spin').value).replace(',', '.'));   // "160,5" juga diterima
    if (!isFinite(v) || v < 20 || v > 200) { $('log').textContent = 'Setpoint harus angka 20-200 C'; return; }
    send('sp,' + v.toFixed(1));
    clean(['spin']);
    $('log').textContent = 'Mengirim setpoint ' + v.toFixed(1) + ' C...';
  }

  // ---------- STEP TEST: rekam, gambar, identifikasi ----------
  let stPts = [], stRun = false, stFit = null;
  const SC = $('sc'), sg = SC.getContext('2d');

  function startStep(){
    const a = parseFloat($('stang').value), mx = parseFloat($('stmax').value), du = parseInt($('stdur').value);
    if (!isFinite(a) || !isFinite(mx) || !isFinite(du)) { $('log').textContent = 'Parameter step test tidak valid'; return; }
    stPts = []; stFit = null; $('sres').innerHTML = '';
    send('stt,' + a + ',' + mx + ',' + du);
  }

  function regress(p, i0, i1){            // regresi linear t -> T pada p[i0..i1-1]
    let n = i1 - i0, sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
    for (let i = i0; i < i1; i++) { const x = p[i][0], y = p[i][1]; sx += x; sy += y; sxx += x*x; sxy += x*y; syy += y*y; }
    const den = n*sxx - sx*sx;
    if (den === 0) return null;
    const s = (n*sxy - sx*sy) / den, a = (sy - s*sx) / n;
    const ssTot = syy - sy*sy/n;
    let ssRes = 0; for (let i = i0; i < i1; i++) { const e = p[i][1] - (a + s*p[i][0]); ssRes += e*e; }
    return {s: s, a: a, r2: ssTot > 0 ? 1 - ssRes/ssTot : 0};
  }

  function drawStep(){
    sg.clearRect(0,0,SC.width,SC.height);
    if (stPts.length < 2) return;
    const tmax = Math.max(60, stPts[stPts.length-1][0]);
    let lo = Infinity, hi = -Infinity;
    stPts.forEach(p => { lo = Math.min(lo, p[1]); hi = Math.max(hi, p[1]); });
    if (hi - lo < 5) { const m=(hi+lo)/2; lo=m-2.5; hi=m+2.5; }
    const X = t => 30 + t/tmax*(SC.width-40), Y = v => SC.height-14 - (v-lo)/(hi-lo)*(SC.height-28);
    sg.strokeStyle='#4ade80'; sg.lineWidth=2; sg.beginPath();
    stPts.forEach((p,i)=>{ if(i) sg.lineTo(X(p[0]),Y(p[1])); else sg.moveTo(X(p[0]),Y(p[1])); });
    sg.stroke();
    if (stFit){                              // garis singgung + garis dead time
      sg.strokeStyle='#f59e0b'; sg.setLineDash([6,4]); sg.beginPath();
      const t1 = Math.max(0, (lo - stFit.a)/stFit.s), t2 = Math.min(tmax, (hi - stFit.a)/stFit.s);
      sg.moveTo(X(t1), Y(stFit.a + stFit.s*t1)); sg.lineTo(X(t2), Y(stFit.a + stFit.s*t2));
      sg.stroke(); sg.setLineDash([]);
    }
    sg.fillStyle='#777'; sg.font='12px sans-serif';
    sg.fillText(hi.toFixed(1)+' C',2,12); sg.fillText(lo.toFixed(1)+' C',2,SC.height-2);
    sg.fillText(Math.round(tmax)+' s',SC.width-50,SC.height-2);
  }

  function analyzeStep(){
    const p = stPts, n = p.length;
    if (n < 30) { $('sres').textContent = 'Data terlalu sedikit (<30 detik): tidak bisa dianalisis.'; return; }
    // rev 24: jendela = 1/5 durasi (60..300 dtk). Jendela tetap 60 dtk memilih potongan curam akibat
    // noise -> kemiringan terlalu besar & dead time salah (uji: L asli 80 dtk terbaca 225 dtk).
    const W = Math.max(Math.min(60, Math.floor(n/2)), Math.min(300, Math.floor(n/5)));
    let best = null;
    for (let i = 0; i + W <= n; i++) {
      const f = regress(p, i, i + W);
      if (f && (best === null || f.s > best.s)) best = f;
    }
    if (!best || !(best.s > 0)) { $('sres').textContent = 'Suhu tidak naik: respons tidak valid (cek api/sensor).'; return; }
    const k = Math.min(5, n);
    let T0 = 0; for (let i = 0; i < k; i++) T0 += p[i][1]; T0 /= k;
    const rise = p[n-1][1] - T0;
    const L = (T0 - best.a) / best.s;                  // waktu garis singgung memotong T0 (dtk)
    const ang = parseFloat($('stang').value), u = (ang - svCd) / (svOd - svCd) * 100;   // % bukaan dari TUTUP..BUKA tersimpan
    const Kp_ = best.s / u;                            // gain proses: degC/dtk per % bukaan
    stFit = best; drawStep();
    let h = '<b>Hasil identifikasi</b><br>'
      + 'Kenaikan total: ' + rise.toFixed(1) + ' &deg;C dalam ' + p[n-1][0] + ' dtk (bukaan ' + ang + '&deg; = ' + u.toFixed(0) + '%)<br>'
      + 'Kemiringan maks R = ' + (best.s*60).toFixed(2) + ' &deg;C/menit (R&sup2; ' + best.r2.toFixed(3) + ', jendela ' + W + ' dtk)<br>';
    if (!(L > 0) || !isFinite(L)) {
      h += '<span class="err">Dead time tidak valid (L=' + L.toFixed(1) + ' dtk): kemungkinan awal data tidak bersih/api sudah menyala sebelum test.</span>';
      $('sres').innerHTML = h; return;
    }
    h += 'Dead time L = ' + L.toFixed(1) + ' dtk<br>';
    // rev 24: uji kelayakan. Hasil tidak layak -> Kp/Ki/Kd TIDAK ditampilkan.
    const dur = p[n-1][0], alasan = [];
    if (best.r2 < 0.9) alasan.push('R&sup2; ' + best.r2.toFixed(3) + ' &lt; 0,9 (garis tidak cocok dgn data)');
    if (rise < 10) alasan.push('kenaikan total ' + rise.toFixed(1) + ' &deg;C &lt; 10 &deg;C (terlalu kecil dibanding noise sensor)');
    if (best.s * W > Math.max(rise, 0) * 1.5 + 1) alasan.push('kemiringan maks tidak sesuai kenaikan total (noise/lonjakan)');
    if (dur < 3 * L) alasan.push('durasi ' + dur + ' dtk &lt; 3 &times; dead time (' + (3*L).toFixed(0) + ' dtk)');
    if (alasan.length) {
      h += '<div class="err" style="font-weight:700;margin-top:6px">HASIL TIDAK VALID - JANGAN pakai untuk tuning:<br>- '
        + alasan.join('<br>- ') + '<br>Ulangi: bukaan lebih besar (mis. 30-40%), tahan sampai suhu naik &ge; 20 &deg;C '
        + '(&ge; 20-40 menit), rekam dgn logger Python.</div>';
      $('sres').innerHTML = h; return;
    }
    const kZN = 1.2/(Kp_*L), tiZN = 2*L, tdZN = 0.5*L;
    const kPI = 0.9/(Kp_*L), tiPI = 3.33*L;
    h += '<br><b>Usulan awal (satuan: output % bukaan per &deg;C, waktu dtk)</b><br>'
      + 'PI konservatif: Kp = ' + kPI.toFixed(2) + ', Ti = ' + tiPI.toFixed(0) + ' dtk, Ki = ' + (kPI/tiPI).toFixed(4) + '/dtk<br>'
      + 'PID Z-N kurva reaksi: Kp = ' + kZN.toFixed(2) + ', Ti = ' + tiZN.toFixed(0) + ' dtk, Td = ' + tdZN.toFixed(0)
      + ' dtk, Ki = ' + (kZN/tiZN).toFixed(4) + ', Kd = ' + (kZN*tdZN).toFixed(2) + '<br>'
      + '<small>Peringatan: model integrator + dead time hanya berlaku di rentang suhu yang diuji; dead time mencakup '
      + 'lag sensor/probe; Z-N agresif (overshoot), mulai dari PI konservatif. Ulangi test bila R&sup2; &lt; 0,98 atau hasil aneh.</small>';
    $('sres').innerHTML = h;
  }

  function stepFeed(m){                    // dipanggil dari pesan 'temp'
    const names = {0:'Siap.', 1:'BERJALAN', 2:'SELESAI', 3:'BERHENTI (pengaman/batal)'};
    if (m.st === 1) {
      if (!stRun) { stRun = true; if (stPts.length && stPts[stPts.length-1][0] > m.te) stPts = []; }
      if (m.ok && m.t !== null && (stPts.length === 0 || m.te > stPts[stPts.length-1][0])) stPts.push([m.te, m.t]);
      $('sts').textContent = 'BERJALAN: ' + m.te + ' dtk' + (m.t !== null ? ', suhu ' + m.t.toFixed(1) + ' C' : '');
      drawStep();
    } else {
      if (stRun) { stRun = false; $('sts').textContent = names[m.st] || 'Selesai'; analyzeStep(); }
    }
  }

  function card(ax, name){
    const steps = [-5000,-1000,-200,-50,50,200,1000,5000];
    const btns = steps.map(s => '<button onclick="send(\'jog,'+ax+','+s+'\')">'+(s>0?'+':'')+s+'</button>').join('');
    return '<div class="box"><h2>Stepper '+name+'</h2>'
      + '<div class="pos">Posisi: <span id="pos_'+ax+'">0</span> step <small id="mv_'+ax+'"></small></div>'
      + '<div>Level pin DIR: <b id="dir_'+ax+'">?</b></div>'
      + btns + '<br><button onclick="send(\'jog,'+ax+',-1000000000\')">&#9664; Putar terus KIRI (-)</button>'
      + '<button onclick="send(\'jog,'+ax+',1000000000\')">Putar terus KANAN (+) &#9654;</button>'
      + '<br><button onclick="send(\'dir,'+ax+',0\')">DIR=LOW (tahan)</button><button onclick="send(\'dir,'+ax+',1\')">DIR=HIGH (tahan)</button>'
      + '</div>';
  }
  $('cards').innerHTML = card('X','X (Conveyor)');

  // data[i] = {t: suhu|null, sp: setpoint|null}; skala Y mencakup suhu DAN setpoint
  function draw(){
    g.clearRect(0,0,cv.width,cv.height);
    const v = [];
    data.forEach(p => { if (p.t !== null) v.push(p.t); if (p.sp !== null) v.push(p.sp); });
    if (v.length < 2) return;
    let lo = Math.min(...v), hi = Math.max(...v);
    if (hi - lo < 5) { const m=(hi+lo)/2; lo=m-2.5; hi=m+2.5; }
    const X = i => i*(cv.width/(MAXPTS-1));
    const Y = val => cv.height-10 - (val-lo)/(hi-lo)*(cv.height-20);
    function line(key, color, dash){
      g.strokeStyle=color; g.lineWidth=2; g.setLineDash(dash); g.beginPath();
      let started=false;
      data.forEach((p,i)=>{
        const val = p[key];
        if (val === null) { started=false; return; }
        if(!started){g.moveTo(X(i),Y(val));started=true;} else g.lineTo(X(i),Y(val));
      });
      g.stroke(); g.setLineDash([]);
    }
    line('sp', '#f59e0b', [6,4]);
    line('t',  '#4ade80', []);
    g.fillStyle='#777'; g.font='12px sans-serif';
    g.fillText(hi.toFixed(1)+' C',4,12); g.fillText(lo.toFixed(1)+' C',4,cv.height-2);
  }

  const NAMES = ['Z Atas','Z Bawah','X Awal','X Akhir'];
  function updLeds(m){
    if (!ledsBuilt){
      $('leds').innerHTML = NAMES.map((n,i)=>'<div class="led" id="led'+i+'">'+n+'<b></b><i></i></div>').join('');
      ledsBuilt = true;
    }
    for (let i=0;i<4;i++){
      const el = $('led'+i);
      const hit = m.ls[i], ver = m.vr[i];
      el.className = 'led ' + (!ver ? 'unk' : (hit ? 'on' : 'off'));
      el.querySelector('b').textContent = (hit ? 'KENA' : 'BEBAS') + (ver ? '' : ' (?)');
      el.querySelector('i').textContent = 'GPIO ' + m.pn[i] + ' = ' + (m.raw[i] ? 'H' : 'L') + (m.il[i] ? '' : ' | tanpa interlock');
    }
  }

  // Isian yang sudah diketik pengguna (dirty) TIDAK ditimpa status dari ESP sampai dikirim.
  // Dulu: klik tombol membuat isian kehilangan fokus sesaat, status (tiap 250 ms) menimpanya,
  // sehingga nilai LAMA yang terkirim -> setpoint/Kp "tidak bisa diubah".
  function setField(id, val){ const el=$(id); if (document.activeElement !== el && !el.dirty) el.value = val; }
  document.addEventListener('input', ev => { if (ev.target.tagName === 'INPUT' && ev.target.type !== 'range') ev.target.dirty = true; });
  function clean(ids){ ids.forEach(id => { $(id).dirty = false; }); }

  function onMsg(m){
    if (m.type === 'temp'){
      const sp = (typeof m.sp === 'number') ? m.sp : null;
      if (sp !== null) $('spv').textContent = sp.toFixed(1);
      if (m.ok && m.t !== null){
        $('temp').innerHTML = m.t.toFixed(1)+' &deg;C';
        if (m.held) { $('status').textContent='HELD: nilai ditahan (sensor error sesaat)'; $('status').className='err'; }
        else { $('status').textContent='Sensor OK'; $('status').className='ok'; }
        data.push({t: m.t, sp: sp});
      } else {
        $('temp').innerHTML='ERR';
        $('status').textContent='SENSOR ERROR: ' + ({1:'thermocouple LEPAS (open circuit)',2:'short ke GND',4:'short ke VCC'}[m.f] || 'tidak ada data (cek SCK/CS/SO & VCC 3V3)'); $('status').className='err'; data.push({t: null, sp: sp});
      }
      if (data.length > MAXPTS) data.shift();
      if (typeof m.st === 'number') stepFeed(m);
      if (typeof m.pe === 'number') {
        $('pids').textContent = m.pe ? (m.ph ? 'PID AKTIF - TAHAN (overshoot, katup TUTUP)' : 'PID AKTIF') : 'PID mati';
        $('pids').className = m.pe ? 'ok' : 'err';
        $('pidu').textContent = m.pu.toFixed(1);
        $('pide').textContent = (m.pe && typeof m.se === 'number') ? m.se.toFixed(1) : '--';
      }
      if (typeof m.kpe === 'number') {
        $('kpe').textContent = m.kpe.toFixed(3); $('kie').textContent = m.kie.toFixed(4); $('kde').textContent = m.kde.toFixed(3);
      }
      $('info').textContent = 'Uptime ' + m.up + ' s | MAX31855 raw ' + m.raw + ' | ' + m.d
        + (typeof m.ec === 'number' ? ' | error sensor: ' + m.ec : '')
        + (typeof m.tr === 'number' ? ' | mentah ' + m.tr.toFixed(2) + ' C' : (m.tr === null ? ' | mentah: -' : ''));
      draw();
    } else if (m.type === 'st'){
      updLeds(m);
      $('bench').textContent = m.bn ? 'MODE BENCH: pengaman limit & batas jog NONAKTIF' : '';
      $('pos_X').textContent = m.xp;
      $('mv_X').textContent = m.xm ? '(bergerak)' : '';
      $('dir_X').textContent = m.dx ? 'HIGH (GPIO 3.3V)' : 'LOW (GPIO 0V)';
      enState = m.en;
      $('enbtn').textContent = 'Driver X: ' + (m.en ? 'AKTIF (klik = matikan)' : 'MATI (klik = aktifkan)');
      setField('spdX', m.sx); setField('accX', m.ax);
      if (typeof m.sp === 'number') { $('spv').textContent = m.sp.toFixed(1); setField('spin', m.sp); }
      // Umax memakai key "pm"; "um" tetap pulsa servo 0 derajat (kotak tersembunyi sumin)
      if (typeof m.kp === 'number') { setField('pkp', m.kp); setField('pki', m.ki); setField('pkd', m.kd); setField('pum', m.pm); }
      if (typeof m.ad === 'number') {
        $('pada').textContent = m.ad ? 'ADAPTIVE PID' : 'PID BIASA'; $('pada').className = m.ad ? 'ok' : '';
        setField('plp', m.lp); setField('pli', m.li); setField('pld', m.ld);
      }

      // servo
      $('sva').textContent = m.sa; $('svg').textContent = m.sg;
      const span = m.od - m.cd;
      $('svp').textContent = span ? Math.round((m.sa - m.cd) / span * 100) : 0;
      $('svr').textContent = m.sr ? '| PULSA MENTAH ' + m.sr + ' us' : '';
      if (typeof m.fw === 'string') $('fw').textContent = '(firmware ' + m.fw + ')';
      if (typeof m.rr === 'string')                         // rev 22: diagnosa ketahanan
        $('diag').textContent = 'Boot terakhir: ' + m.rr + ' | heap min ' + m.hp + ' B | sisa stack LCD '
          + m.sk + ' B | reset LCD ' + m.lr + 'x';
      $('svus').textContent = m.sr ? m.sr : Math.round(m.um + (m.ux - m.um) * m.sa / 180);
      $('svcd').textContent = m.cd; $('svod').textContent = m.od;
      svCd = m.cd; svOd = m.od; svSg = m.sg;
      document.querySelectorAll('.lcd').forEach(el => el.textContent = m.cd);
      document.querySelectorAll('.lod').forEach(el => el.textContent = m.od);
      $('stang').max = m.od;
      setField('sva_in', m.sg); setField('svk_in', m.sg);
      setField('sumin', m.um); setField('sumax', m.ux);
      $('svl').textContent = m.sl == 1 ? 'TERKUNCI: sensor suhu bermasalah, katup dipaksa ke sudut TUTUP'
                           : m.sl == 2 ? 'TERKUNCI: suhu melewati batas ' + m.tc + ' C, katup dipaksa ke sudut TUTUP' : '';

      // motor DC Z
      $('dcc').textContent = m.dcc; $('dct').textContent = m.dct;
      $('dcv').textContent = (Math.abs(m.dcc) / 100 * (m.dcm / 100) * m.dcv).toFixed(1);
      if (document.activeElement !== $('dcs')) { $('dcs').value = m.dct; $('dcsv').textContent = m.dct; }
    } else if (m.type === 'msg'){
      $('log').textContent = m.text;
    }
  }

  function connect(){
    sock = new WebSocket('ws://' + location.hostname + ':81/');
    sock.onopen  = () => { $('status').textContent='Terhubung'; $('status').className='ok'; };
    sock.onclose = () => { $('status').textContent='Terputus, mencoba lagi...'; $('status').className='err'; setTimeout(connect,1500); };
    sock.onmessage = e => onMsg(JSON.parse(e.data));
  }
  connect();
</script>
</body>
</html>
)rawliteral";

// ---------------- LIMIT SWITCH ----------------
inline bool lsRaw(int i) { return digitalRead(LS[i].pin) == HIGH; }
inline bool lsHit(int i) { bool h = lsRaw(i); return LS[i].activeHigh ? h : !h; }

int limitIdx(char ax, int dir) {
  if (ax == 'Z') {
    bool toward = (dir > 0) ? Z_PLUS_TOWARD_ATAS : !Z_PLUS_TOWARD_ATAS;
    return toward ? LS_ZATAS : LS_ZBAWAH;
  } else {
    bool toward = (dir > 0) ? X_PLUS_TOWARD_AKHIR : !X_PLUS_TOWARD_AKHIR;
    return toward ? LS_XAKHIR : LS_XAWAL;
  }
}

bool limitBlocks(char ax, int dir) {
  if (BENCH_MODE) return false;
  int i = limitIdx(ax, dir);
  return LS[i].interlock && lsHit(i);
}
bool dirUnprotected(char ax, int dir) { if (BENCH_MODE) return false; return !LS[limitIdx(ax, dir)].interlock; }

// ---------------- UTIL ----------------
void sendMsg(const char* text) {
  strncpy(gMsg, text, sizeof(gMsg) - 1); gMsg[sizeof(gMsg) - 1] = 0;
  char b[160];
  snprintf(b, sizeof(b), "{\"type\":\"msg\",\"text\":\"%s\"}", text);
  ws.broadcastTXT(b);
  Serial.println(text);
}

void setDrivers(bool en) {                    // driver stepper X (TB6600)
  driversEnabled = en;
  bool level = EN_HIGH_MEANS_ENABLED ? en : !en;
  digitalWrite(PIN_X_EN, level ? HIGH : LOW);
}

void hardStop(AccelStepper& s) { s.setCurrentPosition(s.currentPosition()); }

void applySpeedAccel() {
  stepX.setMaxSpeed(spdX); stepX.setAcceleration(accX);
}

// ---------------- BUZZER (non-blocking, pola on/off bergantian dalam ms; 0 = akhir) ----------------
static const uint16_t BZ_PAT1[] = { 150, 0 };                                   // bip pendek
static const uint16_t BZ_PAT2[] = { 120, 120, 120, 0 };                         // bip ganda
static const uint16_t BZ_PAT3[] = { 800, 0 };                                   // bip panjang
static const uint16_t BZ_PAT4[] = { 100, 100, 100, 100, 100, 300, 600, 0 };     // tes: 3 pendek + 1 panjang
const uint16_t* bzPat = nullptr;
uint8_t  bzIdx = 0;
uint32_t bzNext = 0;

void buzzWrite(bool on) {
  digitalWrite(PIN_BUZZ, (on == BUZZER_ACTIVE_HIGH) ? HIGH : LOW);
}

void buzzInit() {
  pinMode(PIN_BUZZ, OUTPUT);
  buzzWrite(false);
}

void buzzPlay(int id) {
  switch (id) {
    case 1: bzPat = BZ_PAT1; break;
    case 2: bzPat = BZ_PAT2; break;
    case 3: bzPat = BZ_PAT3; break;
    case 4: bzPat = BZ_PAT4; break;
    default: bzPat = nullptr; buzzWrite(false); return;
  }
  bzIdx = 0;
  bzNext = millis();
}

void buzzerUpdate() {
  if (!bzPat) return;
  uint32_t now = millis();
  if ((int32_t)(now - bzNext) < 0) return;
  uint16_t d = bzPat[bzIdx];
  if (d == 0) { buzzWrite(false); bzPat = nullptr; return; }
  buzzWrite((bzIdx % 2) == 0);      // indeks genap = bunyi, ganjil = diam
  bzNext = now + d;
  bzIdx++;
}

// ---------------- MOTOR DC Z (BTS7960, PWM 20 kHz) ----------------
float        dcCur    = 0;        // keluaran terhaluskan, persen dari batas (-100..100)
volatile int dcTarget = 0;        // target persen (-100..100); positif = naik
uint32_t     dcRunStart = 0;

float dcCap() {                   // duty maksimum sebagai pecahan (RATED / SUPPLY, maks 1)
  float c = MOTOR_RATED_V / MOTOR_SUPPLY_V;
  return c > 1.0f ? 1.0f : c;
}

void dcPwmWrite(bool rSide, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(rSide ? PIN_DC_RPWM : PIN_DC_LPWM, duty);
#else
  ledcWrite(rSide ? DC_LEDC_CH_R : DC_LEDC_CH_L, duty);
#endif
}

void dcInit() {
  pinMode(PIN_DC_EN, OUTPUT);
  digitalWrite(PIN_DC_EN, LOW);            // EN LOW lebih dulu: motor lepas
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_DC_RPWM, DC_PWM_FREQ, DC_PWM_BITS);
  ledcAttach(PIN_DC_LPWM, DC_PWM_FREQ, DC_PWM_BITS);
#else
  ledcSetup(DC_LEDC_CH_R, DC_PWM_FREQ, DC_PWM_BITS);
  ledcSetup(DC_LEDC_CH_L, DC_PWM_FREQ, DC_PWM_BITS);
  ledcAttachPin(PIN_DC_RPWM, DC_LEDC_CH_R);
  ledcAttachPin(PIN_DC_LPWM, DC_LEDC_CH_L);
#endif
  dcPwmWrite(true, 0);
  dcPwmWrite(false, 0);
}

// tulis keluaran: hanya SATU sisi PWM yang aktif; sisi lain 0
void dcWrite(float pct) {
  if (pct == 0) { dcPwmWrite(true, 0); dcPwmWrite(false, 0); return; }
  float a = fabsf(pct) / 100.0f * dcCap();                    // 0..cap
  uint32_t duty = (uint32_t)(a * (float)((1UL << DC_PWM_BITS) - 1));
  bool useR = (pct > 0) ? DC_POS_USES_RPWM : !DC_POS_USES_RPWM;
  dcPwmWrite(!useR, 0);
  dcPwmWrite(useR, duty);
}

void dcStop() {                   // berhenti SEKETIKA
  dcTarget = 0; dcCur = 0;
  dcWrite(0);
}

void dcSetTarget(int pct) {
  pct = constrain(pct, -DC_MAX_PCT, DC_MAX_PCT);
  if (pct != 0 && limitBlocks('Z', pct > 0 ? 1 : -1)) {
    sendMsg("Ditolak: limit Z di arah itu sedang kena"); return;
  }
  dcTarget = pct;
  dcRunStart = millis();
}

// dipanggil tiap loop; kerja tiap 10 ms
void dcUpdate() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 10) return;
  float dt = (now - last) / 1000.0f;
  last = now;

  if (dcTarget != 0 && limitBlocks('Z', dcTarget > 0 ? 1 : -1)) {
    dcStop(); sendMsg("Limit Z kena: motor DC berhenti");
  }
  if (dcTarget != 0 && (now - dcRunStart) > DC_MAX_RUN_MS) {
    dcStop(); sendMsg("Batas waktu motor DC: berhenti"); buzzPlay(3);
  }

  float diff = (float)dcTarget - dcCur;
  float step = DC_RAMP_PCT_S * dt;
  float next = (fabsf(diff) <= step) ? (float)dcTarget : dcCur + (diff > 0 ? step : -step);
  if ((dcCur > 0 && next < 0) || (dcCur < 0 && next > 0)) next = 0;   // lewati 0 dulu
  dcCur = next;
  dcWrite(dcCur);

  bool active = (dcTarget != 0) || (dcCur != 0);
  digitalWrite(PIN_DC_EN, (active || DC_BRAKE_WHEN_IDLE) ? HIGH : LOW);
}

// ---------------- SERVO (LEDC langsung, 0-180 derajat) ----------------
void servoPwmBegin() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_SERVO, SERVO_FREQ_HZ, SERVO_PWM_BITS);
#else
  ledcSetup(SERVO_LEDC_CH, SERVO_FREQ_HZ, SERVO_PWM_BITS);
  ledcAttachPin(PIN_SERVO, SERVO_LEDC_CH);
#endif
}

void servoPwmWrite(uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_SERVO, duty);
#else
  ledcWrite(SERVO_LEDC_CH, duty);
#endif
}

void servoWriteUs(float us) {
  us = constrain(us, (float)SERVO_US_ABS_MIN, (float)SERVO_US_ABS_MAX);
  uint32_t maxDuty = (1UL << SERVO_PWM_BITS) - 1;
  servoPwmWrite((uint32_t)(us / (1000000.0f / SERVO_FREQ_HZ) * maxDuty));
}

float servoUsFromDeg(float deg) {
  return (float)svUsMin + (float)(svUsMax - svUsMin) * deg / 180.0f;
}

bool servoPulseOk(int a, int b) {
  return a >= SERVO_US_ABS_MIN && a <= SERVO_US_ABS_MAX && b >= SERVO_US_ABS_MIN && b <= SERVO_US_ABS_MAX && abs(a - b) >= 200;
}

bool servoAnglesOk(int c, int o) {
  return c >= SERVO_CAL_MIN_DEG && o <= 180 && o - c >= SERVO_MIN_SPAN_DEG;
}

void servoSaveCal() {
  prefs.begin("fryer", false);
  prefs.putInt("usmin", svUsMin); prefs.putInt("usmax", svUsMax);
  prefs.putInt("cdeg", svClosedDeg); prefs.putInt("odeg", svOpenDeg);
  prefs.end();
}

void setServoAngle(float deg) {
  deg = constrain(deg, (float)svClosedDeg, (float)svOpenDeg);   // dikunci tutup..buka
  if (svLock && fabsf(deg - (float)svClosedDeg) > 0.5f) {
    sendMsg("Servo terkunci pengaman: hanya boleh ke sudut TUTUP"); return;
  }
  svRawUs = 0;
  svTargetDeg = deg;
}

// rev 20: gerak MANUAL (kotak sudut, -1/+1, kalibrasi). Tidak dikunci TUTUP..BUKA: boleh
// SERVO_CAL_MIN_DEG..180, asalkan pulsa tetap SERVO_US_ABS_MIN..MAX. PID & step test tetap memakai
// setServoAngle() yang dikunci TUTUP..BUKA. Saat PID/step test aktif, perintah manual sudah ditolak.
void setServoAngleManual(float deg) {
  if (deg < SERVO_CAL_MIN_DEG || deg > 180.0f) {
    char m[48]; snprintf(m, sizeof(m), "Sudut harus %d..180 derajat", SERVO_CAL_MIN_DEG); sendMsg(m); return;
  }
  float us = servoUsFromDeg(deg);
  if (us < SERVO_US_ABS_MIN || us > SERVO_US_ABS_MAX) {
    char m[64]; snprintf(m, sizeof(m), "Ditolak: pulsa %.0f us di luar %d-%d us", us, SERVO_US_ABS_MIN, SERVO_US_ABS_MAX);
    sendMsg(m); return;
  }
  if (svLock && fabsf(deg - (float)svClosedDeg) > 0.5f) {
    sendMsg("Servo terkunci pengaman: hanya boleh ke sudut TUTUP"); return;
  }
  svRawUs = 0;
  svTargetDeg = deg;
}

void setServoPct(int pct) {
  pct = constrain(pct, 0, 100);
  setServoAngle((float)svClosedDeg + (float)(svOpenDeg - svClosedDeg) * pct / 100.0f);
}

// dipanggil tiap loop; kerja tiap 20 ms: pengaman (opsional) + gerak + tulis PWM
void servoUpdate() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 20) return;
  float dt = (now - last) / 1000.0f;
  last = now;

#if SERVO_SAFETY
  static bool overLatched = false;
  float t = gTemp;
  bool sensorBad = isnan(t) && (now - gLastValidMs > SENSOR_FAULT_MS);
  if (!isnan(t)) {
    if (t >= TEMP_CUTOFF_C) overLatched = true;
    else if (t <= TEMP_CUTOFF_C - 10.0f) overLatched = false;
  }
  uint8_t lock = sensorBad ? 1 : (overLatched ? 2 : 0);
  if (lock != svLock) {
    svLock = lock;
    if (lock) buzzPlay(3);
    if (lock == 1)      sendMsg("PENGAMAN: sensor suhu error, katup ke sudut TUTUP");
    else if (lock == 2) sendMsg("PENGAMAN: suhu melewati batas, katup ke sudut TUTUP");
    else                sendMsg("Kunci katup dilepas");
  }
  if (svLock) { svRawUs = 0; svTargetDeg = (float)svClosedDeg; }
#endif

  if (svRawUs > 0) { servoWriteUs((float)svRawUs); return; }

  float diff = svTargetDeg - svCurDeg;
  if (SERVO_SLEW_DEG_S <= 0.0f) {
    svCurDeg = svTargetDeg;
  } else {
    float step = SERVO_SLEW_DEG_S * dt;
    if (fabsf(diff) <= step) svCurDeg = svTargetDeg;
    else svCurDeg += (diff > 0 ? step : -step);
  }
  servoWriteUs(servoUsFromDeg(svCurDeg));
}

// ---------------- STEP TEST OPEN-LOOP (rev 14) ----------------
// stState: 0 siap, 1 berjalan, 2 selesai normal, 3 dihentikan pengaman/batal
#define ST_MAX_MIN       60
volatile uint8_t stState = 0;
float    stAngle = 30, stMaxC = 170;
uint32_t stDurMs = 0, stStartMs = 0;

uint32_t stElapsedS() { return (stState == 1) ? (millis() - stStartMs) / 1000UL : 0; }

void stepTestEnd(uint8_t newState, const char* why) {
  stState = newState;
  setServoAngle((float)svClosedDeg);          // katup ke TUTUP
  sendMsg(why);
  buzzPlay(newState == 2 ? 2 : 3);
}

void stepTestStart(float angle, float maxC, int durMin) {
  float t = gTemp;
  if (stState == 1) { sendMsg("Step test sudah berjalan"); return; }
  if (pidOn) { sendMsg("Step test ditolak: matikan PID dulu"); return; }
  if (isnan(t)) { sendMsg("Step test ditolak: sensor suhu tidak valid"); return; }
  if (svLock || svRawUs > 0) { sendMsg("Step test ditolak: servo terkunci/mode pulsa"); return; }
  if (!(angle > (float)svClosedDeg + 0.5f && angle <= (float)svOpenDeg)) {
    sendMsg("Step test ditolak: sudut harus di atas TUTUP dan maks BUKA"); return;
  }
  if (!(maxC >= 60.0f && maxC <= TEMP_CUTOFF_C - 5.0f)) {
    char m[64]; snprintf(m, sizeof(m), "Step test ditolak: batas suhu 60-%.0f C", TEMP_CUTOFF_C - 5.0f);
    sendMsg(m); return;
  }
  if (t >= maxC - 10.0f) { sendMsg("Step test ditolak: suhu awal terlalu dekat batas"); return; }
  if (durMin < 1 || durMin > ST_MAX_MIN) { sendMsg("Step test ditolak: durasi 1-60 menit"); return; }
  stAngle = angle; stMaxC = maxC;
  stDurMs = (uint32_t)durMin * 60000UL;
  stStartMs = millis();
  stState = 1;
  setServoAngle(angle);
  sendMsg("Step test MULAI");
}

void stepTestUpdate() {
  if (stState != 1) return;
  uint32_t now = millis();
  float t = gTemp;
  if (isnan(t) && (now - gLastValidMs) > SENSOR_FAULT_MS) { stepTestEnd(3, "Step test BERHENTI: sensor suhu error"); return; }
  if (!isnan(t) && t >= stMaxC)                           { stepTestEnd(3, "Step test BERHENTI: suhu mencapai batas"); return; }
  if ((now - stStartMs) >= stDurMs)                       { stepTestEnd(2, "Step test SELESAI"); return; }
}

// ---------------- PID SUHU (rev 16) ----------------
float pidKp = PID_DEFAULT_KP, pidKi = PID_DEFAULT_KI, pidKd = PID_DEFAULT_KD, pidUmax = PID_DEFAULT_UMAX;  // gain DASAR (Kp0, Ki0, Kd0)
bool  pidAdapt = PID_ADAPT_DEFAULT;                                       // rev 17: adaptif ON/OFF
float pidLp = PID_DEFAULT_LP, pidLi = PID_DEFAULT_LI, pidLd = PID_DEFAULT_LD;   // lambda_p, lambda_i, lambda_d
float pidKpEff = PID_DEFAULT_KP, pidKiEff = PID_DEFAULT_KI, pidKdEff = PID_DEFAULT_KD;  // gain yang SEDANG dipakai
float pidI = 0;                   // suku integral (persen bukaan)
float pidOut = 0;                 // keluaran terakhir (persen bukaan, 0..Umax)
float pidSpEff = 0;               // setpoint efektif (dengan ramp)
float pidPrevT = 0, pidDf = 0;    // untuk derivatif pada pengukuran
bool  pidPrevValid = false;       // false setelah sampel NAN: derivatif diinisialisasi ulang
bool  pidHold = false;            // rev 24: overshoot -> katup ditahan TUTUP, PID tetap ON
uint32_t pidLastMs = 0;

float servoOpeningPct() {         // bukaan servo sekarang (target) dalam persen TUTUP..BUKA
  float span = (float)(svOpenDeg - svClosedDeg);
  return span > 0 ? constrain(((float)svTargetDeg - (float)svClosedDeg) / span * 100.0f, 0.0f, 100.0f) : 0.0f;
}

void pidAdaptGains(float e, float dTdt);   // didefinisikan di bawah

void pidDisable(const char* why) {
  bool was = pidOn;
  pidOn = false; pidOut = 0; pidI = 0;
  pidAdaptGains(0.0f, 0.0f);                 // rev 17: gain dipakai kembali = gain dasar
  setServoAngle((float)svClosedDeg);         // katup ke TUTUP (api minimum hasil kalibrasi)
  if (was && why) { sendMsg(why); buzzPlay(3); }
}

// rev 17: hukum adaptasi (lihat blok ADAPTIVE PID di konfigurasi). Adaptif OFF -> gain dasar.
void pidAdaptGains(float e, float dTdt) {
  if (!pidAdapt) { pidKpEff = pidKp; pidKiEff = pidKi; pidKdEff = pidKd; return; }
  pidKpEff = fminf(pidKp + pidLp * fabsf(e),    fmaxf(pidKp, PID_KP_MAX));
  pidKiEff = fminf(pidKi + pidLi * fabsf(e),    fmaxf(pidKi, PID_KI_MAX));
  pidKdEff = fminf(pidKd + pidLd * fabsf(dTdt), fmaxf(pidKd, PID_KD_MAX));
}

void pidEnable() {
  TempSnap s = readPubSnap();
  float t = s.t;
  if (pidOn) { sendMsg("PID sudah aktif"); return; }
  if (stState == 1)       { sendMsg("PID ditolak: step test berjalan"); return; }
  if (isnan(t))           { sendMsg("PID ditolak: sensor suhu tidak valid"); return; }
  if (s.held)             { sendMsg("PID ditolak: sensor sedang error sesaat (HELD)"); return; }
  if (svLock || svRawUs > 0) { sendMsg("PID ditolak: servo terkunci/mode pulsa"); return; }
  if (gSetpoint > PID_SP_MAX_C) {
    char m[64]; snprintf(m, sizeof(m), "PID ditolak: setpoint maks %.0f C", PID_SP_MAX_C); sendMsg(m); return;
  }
  if (t >= TEMP_CUTOFF_C - 5.0f) { sendMsg("PID ditolak: suhu sudah terlalu tinggi"); return; }
  pidHold = false;
  pidSpEff = t;                               // ramp dimulai dari suhu sekarang
  pidPrevT = t; pidDf = 0; pidPrevValid = true;
  pidAdaptGains(0.0f, 0.0f);                  // gain efektif = gain dasar saat start
  pidI = constrain(servoOpeningPct(), 0.0f, pidUmax);   // start tanpa loncatan (e = 0 saat start)
  pidOut = pidI;
  pidLastMs = s.ms;
  pidOn = true;
  sendMsg("PID AKTIF");
}

// Dipanggil SEKALI per sampel suhu TERKIRIM (gTempFresh, 1 Hz). dt dari waktu sampel (gPub.ms).
void pidUpdate() {
  if (!pidOn) return;
  TempSnap s = readPubSnap();
  float T = s.t;
  uint32_t nowMs = millis();
  if (isnan(T)) {                             // sampel buruk: tahan keluaran; mati bila terlalu lama
    pidPrevValid = false;
    if (nowMs - gLastValidMs > SENSOR_FAULT_MS) pidDisable("PID MATI: sensor suhu error");
    return;
  }
  // rev 24: hanya batas AKHIR yang mematikan PID. Overshoot biasa -> tahan TUTUP (lihat di bawah).
  if (T >= TEMP_CUTOFF_C) { pidDisable("PID MATI: suhu mencapai batas akhir"); return; }
  if (gSetpoint > PID_SP_MAX_C) { pidDisable("PID MATI: setpoint melewati batas"); return; }

  float dt = (float)(s.ms - pidLastMs) / 1000.0f;
  pidLastMs = s.ms;
  if (dt < 0.05f || dt > 5.0f) dt = 1.0f;

  // ramp setpoint efektif (tetap jalan saat HELD: hanya bergantung waktu)
  float tgt = gSetpoint, stp = PID_SP_RAMP_C_S * dt;
  if (fabsf(tgt - pidSpEff) <= stp) pidSpEff = tgt; else pidSpEff += (tgt > pidSpEff ? stp : -stp);

  if (s.held) return;                         // nilai ditahan = bukan pengukuran baru: I & D beku, keluaran tetap

  // rev 24: OVERSHOOT -> katup ditahan TUTUP (api minimum), integral dinolkan, PID TETAP ON.
  // TUTUP (-3 derajat) = api sangat kecil yang tidak menaikkan suhu, jadi aman ditahan di sini.
  if (!pidHold && T >= gSetpoint + PID_HOLD_C) {
    pidHold = true;
    sendMsg("Overshoot: katup TUTUP sementara, PID tetap ON");
    buzzPlay(1);
  } else if (pidHold && T <= gSetpoint + PID_HOLD_C - PID_HOLD_HYST_C) {
    pidHold = false;
    sendMsg("Suhu turun: kontrol PID normal lagi");
  }
  if (pidHold) {
    pidI = 0; pidOut = 0;
    pidPrevT = T; pidDf = 0;
    pidAdaptGains(pidSpEff - T, 0.0f);
    setServoAngle((float)svClosedDeg);
    return;
  }

  float e = pidSpEff - T;
  if (!pidPrevValid) { pidPrevT = T; pidDf = 0; pidPrevValid = true; }   // cegah lonjakan D setelah gangguan
  float dRaw = (T - pidPrevT) / dt; pidPrevT = T;
  pidDf = PID_D_FILTER * pidDf + (1.0f - PID_D_FILTER) * dRaw;

  pidAdaptGains(e, pidDf);                    // rev 17: Kp/Ki/Kd efektif dari |e| dan |dT/dt|
  float uUnsat = pidKpEff * e + pidI - pidKdEff * pidDf;
  float u = constrain(uUnsat, 0.0f, pidUmax);
  bool windup = (uUnsat > pidUmax && e > 0) || (uUnsat < 0.0f && e < 0);     // integrasi bersyarat
  if (!windup) pidI = constrain(pidI + pidKiEff * e * dt, 0.0f, pidUmax);
  pidOut = u;
  setServoAngle((float)svClosedDeg + (float)(svOpenDeg - svClosedDeg) * u / 100.0f);
}

// Tiap loop: PID mati bila tidak ada sampel suhu terkirim > SENSOR_FAULT_MS (mis. uiTask/SPI macet).
// Tanpa ini pidUpdate() tidak pernah dipanggil lagi dan katup tetap pada bukaan terakhir.
void pidWatchdog() {
  if (!pidOn) return;
  TempSnap s = readPubSnap();
  if (millis() - s.ms > SENSOR_FAULT_MS) pidDisable("PID MATI: data suhu berhenti (uiTask macet?)");
}

void pidSave() {
  prefs.begin("fryer", false);
  prefs.putFloat("pkp", pidKp); prefs.putFloat("pki", pidKi);
  prefs.putFloat("pkd", pidKd); prefs.putFloat("pum", pidUmax);
  prefs.putFloat("plp", pidLp); prefs.putFloat("pli", pidLi); prefs.putFloat("pld", pidLd);
  prefs.putBool("pad", pidAdapt);
  prefs.end();
}

bool pidParamsOk(float kp, float ki, float kd, float um) {
  return isfinite(kp) && isfinite(ki) && isfinite(kd) && isfinite(um) &&
         kp >= 0 && kp <= 50 && ki >= 0 && ki <= 5 && kd >= 0 && kd <= 200 && um >= 5 && um <= 100;
}

bool pidLambdaOk(float lp, float li, float ld) {
  return isfinite(lp) && isfinite(li) && isfinite(ld) &&
         lp >= 0 && lp <= 1 && li >= 0 && li <= 0.01f && ld >= 0 && ld <= 50;
}

// ---------------- PENGAMAN STEPPER X (tiap loop) ----------------
void enforceSafety() {
  long dx = stepX.distanceToGo();
  if (dx != 0 && limitBlocks('X', dx > 0 ? 1 : -1)) { hardStop(stepX); sendMsg("Limit X kena: gerak dihentikan"); }
}

// ---------------- PERINTAH DARI WEB ----------------
void handleCommand(const char* cmd) {
  char ax; long steps; int v, a, b;

  if (sscanf(cmd, "jog,%c,%ld", &ax, &steps) == 2) {
    if (ax == 'Z') { sendMsg("Z = motor DC: pakai kotak Motor DC Z"); return; }
    if (ax != 'X') { sendMsg("Axis tidak dikenal"); return; }
    if (!driversEnabled) { sendMsg("Aktifkan driver X dulu"); return; }
    if (steps == 0 || labs(steps) > MAX_JOG_STEPS) { sendMsg("Jumlah step tidak valid"); return; }
    int dir = steps > 0 ? 1 : -1;
    if (limitBlocks('X', dir)) { sendMsg("Ditolak: limit di arah itu sedang kena"); return; }
    if (dirUnprotected('X', dir) && labs(steps) > MAX_JOG_UNVERIFIED) {
      sendMsg("Arah ini TANPA pengaman limit (tdk terverifikasi): jog dibatasi kecil"); return;
    }
    stepX.move(steps);
    return;
  }
  if (sscanf(cmd, "dir,%c,%d", &ax, &v) == 2) {
    if (ax != 'X') { sendMsg("Hanya axis X yang punya pin DIR"); return; }
    if (stepX.distanceToGo() != 0) { sendMsg("Axis sedang bergerak: STOP dulu"); return; }
    digitalWrite(PIN_X_DIR, v ? HIGH : LOW);
    sendMsg(v ? "DIR X ditahan HIGH" : "DIR X ditahan LOW");
    return;
  }
  if (strcmp(cmd, "stop") == 0) {
    hardStop(stepX); dcStop();
    bool hadPid = pidOn;
    if (hadPid) pidDisable("PID MATI (STOP): katup ke TUTUP");
    if (stState == 1) { stepTestEnd(3, "Step test DIBATALKAN (STOP)"); return; }
    if (!hadPid) sendMsg("Semua gerak dihentikan");   // pesan PID tidak ditimpa
    return;
  }
  // ---- rev 23: "lcdr" reset LCD manual. Hanya memasang flag; reset dikerjakan uiTask (core 0)
  // karena hanya task itu yang memakai bus SPI LCD/MAX31855 (tidak ada akses SPI dari 2 core).
  if (strcmp(cmd, "lcdr") == 0) {
    gLcdResetReq = true;
    sendMsg("Reset LCD diminta");
    return;
  }
  // ---- rev 17: "pida,<0|1>" adaptif ON/OFF (OFF = PID biasa, untuk pembanding) ----
  if (sscanf(cmd, "pida,%d", &v) == 1) {
    pidAdapt = (v != 0);
    if (!pidOn) pidAdaptGains(0.0f, 0.0f);   // saat aktif: berlaku di sampel berikutnya
    pidSave();
    sendMsg(pidAdapt ? "Mode ADAPTIVE PID" : "Mode PID BIASA (adaptif OFF)");
    return;
  }
  // ---- PID (rev 16): "pid,<0|1>" dan "pidk,<kp>,<ki>,<kd>,<umax>" ----
  if (sscanf(cmd, "pid,%d", &v) == 1) {
    if (v) pidEnable(); else { if (pidOn) pidDisable("PID dimatikan, katup ke TUTUP"); else sendMsg("PID sudah mati"); }
    return;
  }
  { float fk, fi, fd, fu;
    // rev 17: "pidl,<lambda_p>,<lambda_i>,<lambda_d>"
    if (sscanf(cmd, "pidl,%f,%f,%f", &fk, &fi, &fd) == 3) {
      if (!pidLambdaOk(fk, fi, fd)) { sendMsg("Lambda ditolak: lp 0-1, li 0-0.01, ld 0-50"); return; }
      pidLp = fk; pidLi = fi; pidLd = fd;
      pidSave();
      sendMsg("Lambda adaptif disimpan");
      return;
    }
    if (sscanf(cmd, "pidk,%f,%f,%f,%f", &fk, &fi, &fd, &fu) == 4) {
      if (!pidParamsOk(fk, fi, fd, fu)) {
        sendMsg("Parameter PID ditolak: Kp 0-50, Ki 0-5, Kd 0-200, Umax 5-100"); return;
      }
      pidKp = fk; pidKi = fi; pidKd = fd; pidUmax = fu;
      if (!pidOn) pidAdaptGains(0.0f, 0.0f);
      if (pidI > pidUmax) pidI = pidUmax;      // Ki disimpan dlm satuan keluaran: ganti Ki tanpa loncatan
      pidSave();
      sendMsg(pidOn ? "Parameter PID disimpan (berlaku sampel berikutnya)" : "Parameter PID disimpan");
      return;
    }
  }
  if (strcmp(cmd, "stx") == 0) {
    if (stState == 1) stepTestEnd(3, "Step test DIBATALKAN");
    else sendMsg("Tidak ada step test berjalan");
    return;
  }
  { float fa, fm; int dm;
    if (sscanf(cmd, "stt,%f,%f,%d", &fa, &fm, &dm) == 3) { stepTestStart(fa, fm, dm); return; } }
  if (stState == 1 && strncmp(cmd, "sv", 2) == 0) { sendMsg("Step test berjalan: servo dikunci (batal dulu)"); return; }
  if (pidOn && strncmp(cmd, "sv", 2) == 0) { sendMsg("PID aktif: servo manual ditolak (PID OFF dulu)"); return; }
  if (sscanf(cmd, "en,%d", &v) == 1) {
    if (v == 0) hardStop(stepX);
    setDrivers(v != 0);
    return;
  }
  if (sscanf(cmd, "spd,%c,%d", &ax, &v) == 2 && ax == 'X') {
    spdX = constrain(v, MIN_SPEED, MAX_SPEED);
    applySpeedAccel(); return;
  }
  if (sscanf(cmd, "acc,%c,%d", &ax, &v) == 2 && ax == 'X') {
    accX = constrain(v, MIN_ACCEL, MAX_ACCEL);
    applySpeedAccel(); return;
  }
  // ---- servo ----
  if (sscanf(cmd, "svcal,%d,%d", &a, &b) == 2) {
    if (!servoPulseOk(a, b)) {
      sendMsg("Kalibrasi pulsa ditolak: 300-3000 us dan selisih minimal 200"); return;
    }
    svUsMin = a; svUsMax = b;
    servoSaveCal();
    sendMsg("Kalibrasi pulsa disimpan");
    return;
  }
  // rev 18: kalibrasi - gerak ke sudut ABSOLUT 0..180 tanpa kunci TUTUP..BUKA.
  // (Ditolak saat PID aktif / step test lewat cek awalan "sv" di atas.)
  if (sscanf(cmd, "svk,%d", &v) == 1) { setServoAngleManual((float)v); return; }
  if (sscanf(cmd, "svset,%c", &ax) == 1 && (ax == 'c' || ax == 'o')) {
    if (svRawUs > 0) { sendMsg("Keluar dari mode pulsa mentah dulu (Kembali ke sudut)"); return; }
    int deg = (int)lroundf(svTargetDeg);   // rev 19: benar juga utk sudut negatif
    // rev 18: arah menutup = sudut mengecil (hasil kalibrasi awal: tutup 0, buka 55);
    // TUTUP harus lebih kecil dari BUKA minimal SERVO_MIN_SPAN_DEG agar % bukaan bermakna.
    if (ax == 'c' && deg > svOpenDeg - SERVO_MIN_SPAN_DEG) {
      sendMsg("Ditolak: TUTUP harus lebih kecil dari BUKA (selisih min 5 derajat)"); return;
    }
    if (ax == 'o' && deg < svClosedDeg + SERVO_MIN_SPAN_DEG) {
      sendMsg("Ditolak: BUKA harus lebih besar dari TUTUP (selisih min 5 derajat)"); return;
    }
    if (ax == 'c') svClosedDeg = deg; else svOpenDeg = deg;
    servoSaveCal();
    char m[48]; snprintf(m, sizeof(m), "%s = %d derajat disimpan", ax == 'c' ? "TUTUP" : "BUKA", deg);
    sendMsg(m);
    return;
  }
  if (sscanf(cmd, "svus,%d", &v) == 1) {
    if (v == 0) { svRawUs = 0; sendMsg("Kembali ke mode sudut"); return; }
    if (v < SERVO_US_ABS_MIN || v > SERVO_US_ABS_MAX) { sendMsg("Pulsa di luar 300-3000 us"); return; }
    svRawUs = v;
    return;
  }
  if (sscanf(cmd, "svad,%d", &v) == 1) { setServoAngleManual(lroundf(svTargetDeg) + v); return; }   // rev 20
  if (sscanf(cmd, "sva,%d", &v) == 1)  { setServoAngleManual((float)v); return; }                   // rev 20
  if (sscanf(cmd, "sv,%d", &v) == 1)   { setServoPct(v); return; }
  // ---- setpoint suhu (rev 12): "sp,<derajat C>" ----
  if (strncmp(cmd, "sp,", 3) == 0) {
    char* end = nullptr;
    float spv = strtof(cmd + 3, &end);
    if (end == cmd + 3 || !isfinite(spv)) { sendMsg("Setpoint tidak valid"); return; }
    if (spv < SETPOINT_MIN_C || spv > SETPOINT_MAX_C) {
      char m[64];
      snprintf(m, sizeof(m), "Setpoint ditolak: harus %.0f-%.0f C", SETPOINT_MIN_C, SETPOINT_MAX_C);
      sendMsg(m); return;
    }
    if (pidOn && spv > PID_SP_MAX_C) {
      char m[64];
      snprintf(m, sizeof(m), "Setpoint ditolak: PID aktif, maks %.0f C", PID_SP_MAX_C);
      sendMsg(m); return;
    }
    gSetpoint = spv;
    prefs.begin("fryer", false);
    prefs.putFloat("sp", spv);
    prefs.end();
    sendMsg("Setpoint disimpan");
    return;
  }
  // ---- motor DC Z ----
  if (sscanf(cmd, "dcz,%d", &v) == 1) { dcSetTarget(v); return; }
  // ---- buzzer ----
  if (sscanf(cmd, "bz,%d", &v) == 1) { buzzPlay(v); return; }
  sendMsg("Perintah tidak dikenal");
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type != WStype_TEXT || length == 0) return;
  if (length >= 64) { sendMsg("Perintah ditolak: terlalu panjang"); return; }   // dulu dibuang diam-diam
  char buf[64];
  memcpy(buf, payload, length);
  buf[length] = 0;
  handleCommand(buf);
}

// ---------------- BROADCAST ----------------
void broadcastStatus() {
  char b[900];
  int n = snprintf(b, sizeof(b), "{\"type\":\"st\",\"ls\":[%d,%d,%d,%d],\"raw\":[%d,%d,%d,%d],\"vr\":[%d,%d,%d,%d],\"il\":[%d,%d,%d,%d],\"pn\":[%d,%d,%d,%d],",
    lsHit(0), lsHit(1), lsHit(2), lsHit(3),
    lsRaw(0), lsRaw(1), lsRaw(2), lsRaw(3),
    LS[0].verified, LS[1].verified, LS[2].verified, LS[3].verified,
    LS[0].interlock, LS[1].interlock, LS[2].interlock, LS[3].interlock,
    LS[0].pin, LS[1].pin, LS[2].pin, LS[3].pin);
  snprintf(b + n, sizeof(b) - n,
    "\"bn\":%d,\"dx\":%d,\"xp\":%ld,\"xm\":%d,\"en\":%d,\"sx\":%d,\"ax\":%d,"
    "\"sa\":%d,\"sg\":%d,\"sr\":%d,\"sl\":%d,\"um\":%d,\"ux\":%d,\"cd\":%d,\"od\":%d,\"tc\":%d,"
    "\"dcc\":%d,\"dct\":%d,\"dcm\":%d,\"dcv\":%d,\"sp\":%.1f,"
    "\"kp\":%.3f,\"ki\":%.5f,\"kd\":%.3f,\"pm\":%.1f,"           // rev 16: pm = Umax PID (um = pulsa servo); kp/ki/kd = gain DASAR
    "\"ad\":%d,\"lp\":%.4f,\"li\":%.5f,\"ld\":%.3f,"             // rev 17: adaptif ON/OFF + lambda
    "\"fw\":\"%s\",\"rr\":\"%s\",\"hp\":%lu,\"sk\":%u,\"lr\":%lu}",  // rev 20 versi; rev 22 diagnosa
    (int)BENCH_MODE, digitalRead(PIN_X_DIR), stepX.currentPosition(), stepX.distanceToGo() != 0,
    driversEnabled, spdX, accX,
    (int)lroundf(svCurDeg), (int)lroundf(svTargetDeg), (int)svRawUs, (int)svLock, svUsMin, svUsMax, svClosedDeg, svOpenDeg, (int)TEMP_CUTOFF_C,
    (int)lroundf(dcCur), (int)dcTarget, (int)(dcCap() * 100.0f + 0.5f), (int)MOTOR_SUPPLY_V, (float)gSetpoint,
    pidKp, pidKi, pidKd, pidUmax,
    (int)pidAdapt, pidLp, pidLi, pidLd, FW_REV,
    gResetWhy, (unsigned long)ESP.getMinFreeHeap(),
    gUiTask ? (unsigned)uxTaskGetStackHighWaterMark(gUiTask) : 0u, (unsigned long)gLcdResets);
  ws.broadcastTXT(b);
}

void broadcastTemp() {
  // rev 16: t/ms/held dari snapshot terkirim yang SAMA dengan yang dipakai PID
  TempSnap s = readPubSnap();
  float t = s.t;
  uint32_t r = gRaw;
  const char* d = rawDiag(r);
  float sp = gSetpoint;
  unsigned long seq = gTempSeq, ms = s.ms;
  // rev 15: tr = mentah (null bila fault), held = nilai ditahan, ec = jumlah error sejak boot
  float tr = gTempRaw;
  char trs[16];
  if (isnan(tr)) strcpy(trs, "null"); else snprintf(trs, sizeof(trs), "%.2f", tr);
  int held = s.held ? 1 : 0;
  unsigned long ec = gErrCount;
  // rev 16: pe = PID aktif, pu = keluaran PID (%), se = setpoint efektif (null bila PID mati)
  // rev 17: kpe/kie/kde = gain yang SEDANG dipakai (adaptif); saat PID mati = gain dasar
  bool pe = pidOn;
  char ses[16];
  if (pe) snprintf(ses, sizeof(ses), "%.2f", pidSpEff); else strcpy(ses, "null");
  char b[560];
  // sp, seq, ms (rev 12): setpoint, nomor urut sampel, waktu sampel ESP32 (ms)
  if (isnan(t)) snprintf(b, sizeof(b), "{\"type\":\"temp\",\"t\":null,\"ok\":false,\"f\":%d,\"raw\":\"%08lX\",\"d\":\"%s\",\"up\":%lu,\"sp\":%.1f,\"seq\":%lu,\"ms\":%lu,\"st\":%d,\"te\":%lu,\"tr\":%s,\"held\":%d,\"ec\":%lu,\"pe\":%d,\"pu\":%.2f,\"se\":%s,\"kpe\":%.4f,\"kie\":%.5f,\"kde\":%.4f,\"ph\":%d}",
                         (int)gFault, (unsigned long)r, d, millis() / 1000, sp, seq, ms, (int)stState, (unsigned long)stElapsedS(), trs, held, ec, (int)pe, pidOut, ses, pidKpEff, pidKiEff, pidKdEff, (int)(pe && pidHold));
  else          snprintf(b, sizeof(b), "{\"type\":\"temp\",\"t\":%.2f,\"ok\":true,\"raw\":\"%08lX\",\"d\":\"%s\",\"up\":%lu,\"sp\":%.1f,\"seq\":%lu,\"ms\":%lu,\"st\":%d,\"te\":%lu,\"tr\":%s,\"held\":%d,\"ec\":%lu,\"pe\":%d,\"pu\":%.2f,\"se\":%s,\"kpe\":%.4f,\"kie\":%.5f,\"kde\":%.4f,\"ph\":%d}",
                         t, (unsigned long)r, d, millis() / 1000, sp, seq, ms, (int)stState, (unsigned long)stElapsedS(), trs, held, ec, (int)pe, pidOut, ses, pidKpEff, pidKiEff, pidKdEff, (int)(pe && pidHold));
  ws.broadcastTXT(b);
}

// ---------------- UI TFT + BACA MAX31855 (task di core 0) ----------------
struct Field { char last[96]; uint16_t color; Field() { last[0] = 1; last[1] = 0; color = 0; } };

void putText(Field& f, const char* str, int x, int y, int size, uint16_t color, int pad) {
  if (f.color == color && strcmp(f.last, str) == 0) return;
  strncpy(f.last, str, sizeof(f.last) - 1); f.last[sizeof(f.last) - 1] = 0; f.color = color;
  tft.setTextSize(size);
  tft.setTextColor(color, TFT_BLACK);
  tft.setTextPadding(pad);
  tft.drawString(str, x, y);
}

void drawLimitBox(int i, bool hit, bool raw, bool il) {
  int x = 6 + i * 118, y = 112;
  uint16_t bg = hit ? TFT_MAROON : TFT_DARKGREEN;
  tft.fillRect(x, y, 112, 62, bg);
  tft.setTextColor(TFT_WHITE, bg);
  tft.setTextPadding(0);
  tft.setTextSize(2);
  tft.drawString(LS[i].name, x + 6, y + 6);
  tft.drawString(hit ? "KENA" : "BEBAS", x + 6, y + 28);
  tft.setTextSize(1);
  char b[28];
  snprintf(b, sizeof(b), "GPIO%d=%c%s", LS[i].pin, raw ? 'H' : 'L', il ? "" : " no-IL");
  tft.drawString(b, x + 6, y + 50);
}

#define CH_N 100
float chartBuf[CH_N];
int   chartCnt = 0;

void chartPush(float v) {
  if (chartCnt < CH_N) chartBuf[chartCnt++] = v;
  else { memmove(chartBuf, chartBuf + 1, (CH_N - 1) * sizeof(float)); chartBuf[CH_N - 1] = v; }
}

void chartDraw() {
  const int X = 270, Y = 28, W = 204, H = 74;
  tft.fillRect(X, Y, W, H, TFT_BLACK);
  tft.drawRect(X, Y, W, H, TFT_DARKGREY);
  float lo = 1e9, hi = -1e9; int n = 0;
  for (int k = 0; k < chartCnt; k++) if (!isnan(chartBuf[k])) { lo = min(lo, chartBuf[k]); hi = max(hi, chartBuf[k]); n++; }
  if (n < 2) return;
  if (hi - lo < 5) { float m = (hi + lo) / 2; lo = m - 2.5; hi = m + 2.5; }
  int px = -1, py = 0;
  for (int k = 0; k < chartCnt; k++) {
    if (isnan(chartBuf[k])) { px = -1; continue; }
    int x = X + 2 + k * (W - 4) / (CH_N - 1);
    int y = Y + H - 3 - (int)((chartBuf[k] - lo) / (hi - lo) * (H - 6));
    if (px >= 0) tft.drawLine(px, py, x, y, TFT_GREEN);
    px = x; py = y;
  }
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setTextPadding(0);
  tft.setTextSize(1);
  char b[16];
  snprintf(b, sizeof(b), "%.0f", hi); tft.drawString(b, X + 4, Y + 3);
  snprintf(b, sizeof(b), "%.0f", lo); tft.drawString(b, X + 4, Y + H - 11);
}

void uiStatic() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextPadding(0);
  tft.setTextSize(2);
  tft.drawString("DEEP FRYER CERDAS", 6, 4);
  if (BENCH_MODE) {
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.drawString(" BENCH ", 380, 4);
  }
  tft.drawFastHLine(0, 24, tft.width(), TFT_DARKGREY);
}

void fieldReset(Field& f) { f.last[0] = 1; f.last[1] = 0; f.color = 0; }

void uiUpdate() {
  static Field fTemp, fSens, fZ, fX, fDrv, fSv, fMsg, fWifi;
  static int8_t lHit[LS_COUNT] = { -1, -1, -1, -1 }, lRaw[LS_COUNT] = { -1, -1, -1, -1 };
  char b[80];

  if (gUiForce) {                    // rev 22: LCD baru di-reset -> semua elemen digambar ulang
    gUiForce = false;
    Field* all[] = { &fTemp, &fSens, &fZ, &fX, &fDrv, &fSv, &fMsg, &fWifi };
    for (Field* f : all) fieldReset(*f);
    for (int i = 0; i < LS_COUNT; i++) { lHit[i] = -1; lRaw[i] = -1; }
  }

  float t = gTemp;
  if (isnan(t)) {
    putText(fTemp, "ERR", 10, 34, 6, TFT_RED, 250);
    const char* why = (gFault & 1) ? "thermocouple LEPAS" : (gFault & 2) ? "short ke GND" : (gFault & 4) ? "short ke VCC" : "tdk ada data";
    snprintf(b, sizeof(b), "ERR %08lX %s", (unsigned long)gRaw, gRaw == 0 ? "SO=LOW" : (gRaw == 0xFFFFFFFFUL ? "SO=HIGH" : why));
    putText(fSens, b, 10, 86, 2, TFT_RED, 470);
  } else {
    bool held = gTempHeld;
    snprintf(b, sizeof(b), "%.1f C", t);
    putText(fTemp, b, 10, 34, 6, held ? TFT_YELLOW : TFT_WHITE, 250);
    if (held) snprintf(b, sizeof(b), "HELD (error sesaat) raw %08lX", (unsigned long)gRaw);
    else      snprintf(b, sizeof(b), "Sensor OK  raw %08lX", (unsigned long)gRaw);
    putText(fSens, b, 10, 86, 2, held ? TFT_YELLOW : TFT_GREEN, 470);
  }

  for (int i = 0; i < LS_COUNT; i++) {
    int8_t h = lsHit(i), r = lsRaw(i);
    if (h != lHit[i] || r != lRaw[i]) { lHit[i] = h; lRaw[i] = r; drawLimitBox(i, h, r, LS[i].interlock); }
  }

  snprintf(b, sizeof(b), "Z DC: %d%% (tgt %d%%) %s", (int)lroundf(dcCur), (int)dcTarget, dcTarget > 0 ? "NAIK" : (dcTarget < 0 ? "TURUN" : "diam"));
  putText(fZ, b, 10, 180, 2, TFT_WHITE, 470);
  snprintf(b, sizeof(b), "X: %ld step %s D:%c", stepX.currentPosition(), stepX.distanceToGo() ? "[gerak]" : "", digitalRead(PIN_X_DIR) ? 'H' : 'L');
  putText(fX, b, 10, 202, 2, TFT_WHITE, 470);

  snprintf(b, sizeof(b), "Drv X:%s %d/%d", driversEnabled ? "AKTIF" : "MATI", spdX, accX);
  putText(fDrv, b, 10, 224, 2, driversEnabled ? TFT_GREEN : TFT_LIGHTGREY, 470);

  uint8_t lk = svLock;
  int raw = svRawUs;
  if (raw > 0) snprintf(b, sizeof(b), "Servo: PULSA %d us", raw);
  else         snprintf(b, sizeof(b), "Servo: %d deg (tgt %d)%s", (int)lroundf(svCurDeg), (int)lroundf(svTargetDeg),
                        lk == 1 ? " KUNCI:SENSOR" : (lk == 2 ? " KUNCI:SUHU" : ""));
  putText(fSv, b, 10, 246, 2, lk ? TFT_RED : TFT_CYAN, 470);

  char m[40];
  strncpy(m, gMsg[0] ? gMsg : "-", sizeof(m) - 1); m[sizeof(m) - 1] = 0;
  putText(fMsg, m, 10, 270, 2, TFT_YELLOW, 470);

  // rev 22: IP diformat langsung (tanpa String -> tidak membebani/memecah heap tiap 150 ms)
  snprintf(b, sizeof(b), "WiFi AP: %s  IP: %u.%u.%u.%u  klien:%d  up:%lus", AP_SSID,
           AP_IP[0], AP_IP[1], AP_IP[2], AP_IP[3], WiFi.softAPgetStationNum(), millis() / 1000);
  putText(fWifi, b, 6, 302, 1, TFT_DARKGREY, 470);
}

// rev 22: reset LCD NON-BLOCKING (pola blink-without-delay). Dulu delay(10)+delay(50)+delay(150)
// menghentikan uiTask 210 ms. Sekarang tiap tahap hanya memeriksa millis() lalu kembali, sehingga
// pembacaan suhu tetap berjalan selama pin RESET LCD ditahan.
enum LcdState : uint8_t { LCD_START, LCD_RST_HIGH, LCD_RST_LOW, LCD_RST_WAIT, LCD_READY };
LcdState lcdState = LCD_START;
uint32_t lcdT0 = 0;
bool     sensorReady = false;       // MAX31855 sudah di-begin() (setelah init LCD pertama)

// ---------------- FILTER SENSOR SUHU: CASCADED FILTERING 3 TAHAP (rev 21) ----------------
// Menggantikan Savitzky-Golay (rev 15). Per sampel valid (200 ms):
//   Tahap 1  MEDIAN-5      : median 5 sampel terakhir -> membuang lonjakan 1-2 sampel berturut
//                            (frame MAX31855 rusak/noise EMI) tanpa ambang tetap.
//   Tahap 2  MOVING AVG-5  : rata-rata 5 keluaran median -> menekan noise acak (std / ~sqrt(5)).
//   Tahap 3  EMA (alpha)   : low-pass orde-1  y += alpha*(x - y) -> menghaluskan sisa riak.
// SIFAT (simulasi Python, Ts 200 ms, alpha 0.3; lihat Python/uji_filter_sensor.py):
//   - jeda pada suhu yang naik/turun linear ~6.3 sampel = ~1.3 dtk (median 2 + MA 2 + EMA 2.3).
//     Jeda ini IKUT masuk ke dead time hasil step test dan ke loop PID (kecil dibanding dead
//     time plant ~10-20 dtk, tetapi tidak nol).
//   - std noise acak turun ke ~34% (SG rev 15: ~76%).
//   - lonjakan 1 atau 2 sampel berturut dibuang 100%; 3 sampel berturut lolos sebagian (~45%).
//   - respons loncatan: tanpa overshoot, 90% tercapai ~2 dtk.
//   - pengaman suhu-lebih (TEMP_CUTOFF, PID_OVER) memakai nilai tersaring -> ikut tertunda ~1.3 dtk.
// Awal data / setelah reset: median & MA memakai sampel yang ada (1..5), EMA mulai dari nilai
// pertama -> tidak menunggu jendela penuh.
// FAULT (NaN dari MAX31855): HOLD nilai tersaring terakhir maks SENSOR_HOLD_MS (filter dibekukan,
// tidak diisi); lewat itu gTemp = NAN (pengaman servo/step test/PID bekerja seperti semula) dan
// seluruh filter direset. gLastValidMs hanya diperbarui oleh sampel valid.
#define FILT_MED_N      5
#define FILT_MA_N       5
#define FILT_EMA_ALPHA  0.3f

struct SensorFilter {
  float    med[FILT_MED_N]; uint8_t mn = 0, mh = 0;   // tahap 1: buffer sirkular median
  float    ma[FILT_MA_N];   uint8_t an = 0, ah = 0;   // tahap 2: buffer sirkular moving average
  float    ema = NAN;                                  // tahap 3
  bool     have = false;                               // sudah ada sampel valid sejak reset
  float    lastOut = NAN;                              // keluaran terakhir (yang ditahan saat error)
  uint32_t lastGoodMs = 0;

  void reset() { mn = mh = an = ah = 0; ema = NAN; have = false; }

  float stage1Median(float v) {
    med[mh] = v; mh = (mh + 1) % FILT_MED_N; if (mn < FILT_MED_N) mn++;
    float t[FILT_MED_N];
    for (int i = 0; i < mn; i++) t[i] = med[i];          // urutan tidak penting untuk median
    for (int i = 1; i < mn; i++) {                       // insertion sort (n <= 5)
      float x = t[i]; int j = i - 1;
      while (j >= 0 && t[j] > x) { t[j + 1] = t[j]; j--; }
      t[j + 1] = x;
    }
    return (mn % 2) ? t[mn / 2] : 0.5f * (t[mn / 2 - 1] + t[mn / 2]);
  }
  float stage2MovAvg(float v) {
    ma[ah] = v; ah = (ah + 1) % FILT_MA_N; if (an < FILT_MA_N) an++;
    float sum = 0;
    for (int i = 0; i < an; i++) sum += ma[i];           // dijumlah ulang: tanpa drift pembulatan
    return sum / an;
  }
  float stage3Ema(float v) {
    ema = isnan(ema) ? v : ema + FILT_EMA_ALPHA * (v - ema);
    return ema;
  }
};
SensorFilter sf;

// Proses satu sampel. Mengembalikan nilai untuk gTemp; *held = nilai ditahan;
// *accepted = sampel valid diterima (hanya ini yang memperbarui gLastValidMs).
float sensorFilterStep(float tv, uint32_t now, bool* held, bool* accepted) {
  *held = false; *accepted = false;
  bool valid = !isnan(tv);
  if (!valid) gErrCount = gErrCount + 1;
#if SENSOR_FILTER_ENABLE
  if (valid) {
    sf.have = true; sf.lastGoodMs = now;
    sf.lastOut = sf.stage3Ema(sf.stage2MovAvg(sf.stage1Median(tv)));
    *accepted = true;
    return sf.lastOut;
  }
  if (sf.have && (now - sf.lastGoodMs) <= SENSOR_HOLD_MS) {      // HOLD: filter dibekukan
    *held = true;
    return sf.lastOut;
  }
  sf.reset();                                                     // lewat batas hold
  return NAN;
#else
  *accepted = valid;
  return tv;                                                      // perilaku lama (tanpa filter)
#endif
}

// Dipanggil tiap putaran uiTask; TIDAK pernah menunggu. Urutan (sama dgn boot):
//   RESET HIGH 10 ms -> LOW 50 ms -> HIGH 150 ms -> tft.init() -> MAX31855 begin() lagi
// LCD & MAX31855 berbagi bus SPI dan tft.init() mengatur ulang bus, jadi MAX31855 di-begin()
// sesudahnya. CATATAN JUJUR: tft.init() milik library TFT_eSPI berisi delay internal ~295 ms
// (reset 150 + sleep-out ILI9488 120 + 25) yang tidak bisa dipecah; delay itu menyerahkan CPU
// (vTaskDelay), jadi loop() di core 1 tetap jalan, tetapi sampling suhu tertahan ~0.3 dtk.
void lcdService(uint32_t now) {
  switch (lcdState) {
    case LCD_START:
      pinMode(PIN_TFT_RST, OUTPUT);
      digitalWrite(PIN_TFT_RST, HIGH);
      lcdT0 = now; lcdState = LCD_RST_HIGH;
      break;
    case LCD_RST_HIGH:
      if (now - lcdT0 >= 10)  { digitalWrite(PIN_TFT_RST, LOW);  lcdT0 = now; lcdState = LCD_RST_LOW; }
      break;
    case LCD_RST_LOW:
      if (now - lcdT0 >= 50)  { digitalWrite(PIN_TFT_RST, HIGH); lcdT0 = now; lcdState = LCD_RST_WAIT; }
      break;
    case LCD_RST_WAIT:
      if (now - lcdT0 >= 150) {
        tft.init();
        tft.setRotation(1);           // landscape 480x320
        uiStatic();
        if (!thermocouple.begin()) Serial.println(F("MAX31855 begin() GAGAL: cek wiring CS5, SO19, SCK18, VCC 3V3"));
        sensorReady = true;
        gUiForce = true;              // semua field LCD digambar ulang
        chartDraw();
        lcdState = LCD_READY;
      }
      break;
    case LCD_READY:
      break;
  }
}

// uiTask (core 0) = MULTITASKING KOOPERATIF: tiap putaran menjalankan potongan kerja pendek
// berbasis millis() (servis LCD, sampel suhu 200 ms, gambar LCD 150 ms, reset LCD 30 dtk),
// lalu vTaskDelay(10 ms) menyerahkan CPU. Tidak ada delay() di dalam putaran ini.
void uiTask(void*) {
  esp_task_wdt_add(NULL);           // rev 22: uiTask diawasi watchdog

  uint32_t lastTemp = 0, lastUi = 0;
  [[maybe_unused]] uint32_t lastLcdInit = millis();   // hanya dipakai bila LCD_REINIT_MS > 0
  uint8_t  pubCnt = SENSOR_PUBLISH_EVERY - 1;     // sampel pertama langsung dikirim
  bool first = true;
  for (;;) {
    uint32_t now = millis();
    lcdService(now);                                         // reset/init LCD bertahap
    if (sensorReady && (first || now - lastTemp >= SENSOR_SAMPLE_MS)) {
      first = false; lastTemp = now;
      gRaw = maxReadRaw();
      float tv = thermocouple.readCelsius();          // NAN kalau fault
      uint8_t f = isnan(tv) ? thermocouple.readError() : 0;
      bool held, accepted;
      float out = sensorFilterStep(tv, now, &held, &accepted);
      gFault = f;
      gTempRaw = tv;
      gTempHeld = held;
      gTemp = out;
      if (accepted) gLastValidMs = now;
      if (++pubCnt >= SENSOR_PUBLISH_EVERY) {        // 1 Hz: web, logger, grafik LCD
        pubCnt = 0;
        portENTER_CRITICAL(&gSnapMux);               // rev 16: suhu+waktu+flag sampel yang sama
        gPub.t = out; gPub.ms = now; gPub.held = held;
        portEXIT_CRITICAL(&gSnapMux);
        gTempMs = now;
        gTempSeq = gTempSeq + 1;
        gTempFresh = true;
        chartPush(out);
        if (lcdState == LCD_READY) chartDraw();          // LCD sedang di-reset -> jangan gambar
      }
    }
    if (gLcdResetReq && lcdState == LCD_READY) {         // rev 23: reset manual dari web
      gLcdResetReq = false;
      lastLcdInit = now;
      lcdState = LCD_START;
      gLcdResets = gLcdResets + 1;
    }
#if LCD_REINIT_MS > 0
    if (lcdState == LCD_READY && now - lastLcdInit >= LCD_REINIT_MS) {
      lastLcdInit = now;                                     // rev 22: reset LCD otomatis tiap N ms
      lcdState = LCD_START;
      gLcdResets = gLcdResets + 1;
    }
#endif
    if (lcdState == LCD_READY && now - lastUi >= 150) { lastUi = now; uiUpdate(); }
    esp_task_wdt_reset();                                       // rev 22: uiTask masih hidup
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  // Motor DC PERTAMA: EN (GPIO14) LOW + PWM 0. Sebelum titik ini (bootloader) hanya
  // pull-down 10k eksternal yang menahan EN/RPWM/LPWM.
  dcInit();

  // Servo: muat kalibrasi dari flash, langsung ke sudut TUTUP
  prefs.begin("fryer", true);
  svUsMin     = prefs.getInt("usmin", DEFAULT_SERVO_US_MIN);
  svUsMax     = prefs.getInt("usmax", DEFAULT_SERVO_US_MAX);
  svClosedDeg = prefs.getInt("cdeg",  DEFAULT_SERVO_CLOSED_DEG);
  svOpenDeg   = prefs.getInt("odeg",  DEFAULT_SERVO_OPEN_DEG);
  float spSaved = prefs.getFloat("sp", DEFAULT_SETPOINT_C);
  float kp = prefs.getFloat("pkp", PID_DEFAULT_KP), ki = prefs.getFloat("pki", PID_DEFAULT_KI);
  float kd = prefs.getFloat("pkd", PID_DEFAULT_KD), um = prefs.getFloat("pum", PID_DEFAULT_UMAX);
  float lp = prefs.getFloat("plp", PID_DEFAULT_LP), li = prefs.getFloat("pli", PID_DEFAULT_LI);
  float ld = prefs.getFloat("pld", PID_DEFAULT_LD);
  bool  pad = prefs.getBool("pad", PID_ADAPT_DEFAULT);
  prefs.end();
  if (pidParamsOk(kp, ki, kd, um)) { pidKp = kp; pidKi = ki; pidKd = kd; pidUmax = um; }   // flash rusak -> default
  if (pidLambdaOk(lp, li, ld)) { pidLp = lp; pidLi = li; pidLd = ld; }
  pidAdapt = pad;
  pidAdaptGains(0.0f, 0.0f);
  gSetpoint = (isfinite(spSaved) && spSaved >= SETPOINT_MIN_C && spSaved <= SETPOINT_MAX_C)
              ? spSaved : DEFAULT_SETPOINT_C;
  // rev 18: kalibrasi TUTUP/BUKA & pulsa diambil dari flash (rev 13 sempat dipaku 0/55);
  // nilai rusak/tidak wajar -> default.
  if (!servoPulseOk(svUsMin, svUsMax)) { svUsMin = DEFAULT_SERVO_US_MIN; svUsMax = DEFAULT_SERVO_US_MAX; }
  if (!servoAnglesOk(svClosedDeg, svOpenDeg)) { svClosedDeg = DEFAULT_SERVO_CLOSED_DEG; svOpenDeg = DEFAULT_SERVO_OPEN_DEG; }
  svTargetDeg = svCurDeg = (float)svClosedDeg;
  servoPwmBegin();
  servoWriteUs(servoUsFromDeg(svCurDeg));

  buzzInit();

  Serial.begin(115200);              // rev 22: tanpa delay (ESP32 tidak perlu menunggu port serial)

  // rev 22: alasan reset (TASK_WDT = ESP restart karena macet; BROWNOUT = tegangan drop)
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  strcpy(gResetWhy, "POWER_ON"); break;
    case ESP_RST_SW:       strcpy(gResetWhy, "SOFTWARE"); break;
    case ESP_RST_PANIC:    strcpy(gResetWhy, "PANIC/CRASH"); break;
    case ESP_RST_INT_WDT:  strcpy(gResetWhy, "INT_WDT"); break;
    case ESP_RST_TASK_WDT: strcpy(gResetWhy, "TASK_WDT"); break;
    case ESP_RST_WDT:      strcpy(gResetWhy, "WDT"); break;
    case ESP_RST_BROWNOUT: strcpy(gResetWhy, "BROWNOUT"); break;
    case ESP_RST_EXT:      strcpy(gResetWhy, "TOMBOL_EN"); break;
    default:               strcpy(gResetWhy, "LAIN"); break;
  }
  Serial.print(F("Alasan boot: ")); Serial.println(gResetWhy);

  // rev 22: watchdog. Macet > WDT_TIMEOUT_S -> ESP restart (setup: katup TUTUP, PID & motor mati)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdtCfg = { .timeout_ms = WDT_TIMEOUT_S * 1000, .idle_core_mask = 0, .trigger_panic = true };
  esp_task_wdt_reconfigure(&wdtCfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);           // loop() (loopTask) diawasi

  // Semua input limit pakai pull-up EKSTERNAL 10k (GPIO34/35/39 tidak punya pull internal)
  for (int i = 0; i < LS_COUNT; i++) pinMode(LS[i].pin, INPUT);

  pinMode(PIN_X_EN, OUTPUT);
  setDrivers(false);

  stepX.setPinsInverted(X_DIR_INVERT, false, false);
  stepX.setMinPulseWidth(20);
  applySpeedAccel();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
  Serial.print(F("AP ")); Serial.println(ok ? F("aktif") : F("GAGAL"));
  Serial.print(F("IP: ")); Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, []() {
    // rev 20: larang cache, supaya browser/HP tidak memakai halaman lama setelah firmware di-upload
    server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.send_P(200, "text/html", INDEX_HTML);
  });
  server.begin();
  ws.begin();
  ws.onEvent(onWsEvent);

  xTaskCreatePinnedToCore(uiTask, "ui", 8192, NULL, 1, &gUiTask, 0);
  buzzPlay(1);                      // bip pendek saat boot = buzzer hidup
}

void loop() {
  esp_task_wdt_reset();             // rev 22: loop() masih hidup
  server.handleClient();
  ws.loop();

  enforceSafety();
  stepX.run();
  stepTestUpdate();
  servoUpdate();
  dcUpdate();
  buzzerUpdate();

  pidWatchdog();
  if (gTempFresh) { gTempFresh = false; pidUpdate(); broadcastTemp(); }   // PID 1 Hz, dari nilai tersaring

  unsigned long now = millis();
  if (now - lastStatus >= 250) { lastStatus = now; broadcastStatus(); }
}