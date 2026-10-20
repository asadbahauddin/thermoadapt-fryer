/*
 * DEEP FRYER CERDAS - MONITOR & KONTROL VIA WEB + LCD (rev 15)
 * ESP32 = WiFi Access Point (192.168.4.1) + WebSocket.
 * Isi: suhu MAX31855, 4 limit switch, Z = MOTOR DC (BTS7960), X = stepper (TB6600),
 *      SERVO katup gas 0-180 derajat (kalibrasi), BUZZER, LCD TFT mirror dari web.
 *
 * REV 15 (dari rev 14):
 *   - BTS7960 sesuai wiring terbaru: GPIO26 = RPWM | GPIO27 = LPWM | GPIO14 = R_EN + L_EN.
 *     GPIO14 mengeluarkan PWM singkat saat boot; dcInit() dijalankan PERTAMA di setup() dan
 *     menahan EN LOW, tetapi selama bootloader (sebelum setup) firmware belum berjalan:
 *     WAJIB pull-down 10k ke GND di EN (14), RPWM (26), dan LPWM (27).
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
 *       (pemetaan rev 11 di bawah KELIRU, sudah diganti di rev 15: 26 = RPWM, 27 = LPWM, 14 = EN):
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

// ---------------- PIN ----------------
#define PIN_MAX_SCK 18
#define PIN_MAX_CS  5
#define PIN_MAX_SO  19

#define PIN_X_STEP 32
#define PIN_X_DIR  33
#define PIN_X_EN   25

// BTS7960 (rev 15, wiring terbaru; dicek kontinuitas oleh pengguna).
// Pull-down 10k ke GND di ketiga pin: GPIO14 mengeluarkan PWM singkat saat boot.
#define PIN_DC_RPWM 26
#define PIN_DC_LPWM 27
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
#define DC_POS_USES_RPWM    true     // positif (naik) memakai RPWM (GPIO26); false = LPWM
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
#define DEFAULT_SERVO_CLOSED_DEG 0       // sudut katup TUTUP (hasil kalibrasi: menutup di 0 derajat)
#define DEFAULT_SERVO_OPEN_DEG   55      // sudut katup BUKA  (hasil kalibrasi: membuka di 55 derajat)
#define SERVO_SLEW_DEG_S        0.0f     // 0 = tanpa batas laju (melompat ke target)

#define SERVO_SAFETY           false    // true: paksa katup ke sudut TUTUP bila sensor error/suhu lewat batas
#define SENSOR_FAULT_MS        5000UL
#define TEMP_CUTOFF_C          180.0f   // NILAI AWAL: cek titik asap minyak

// ---- PENANGANAN SENSOR SUHU (rev 15) ----
#define SENSOR_FILTER_ENABLE   1        // 0 = perilaku lama: gTemp = bacaan mentah (tanpa tolak lonjakan/hold/SG)
#define SENSOR_SAMPLE_MS       200UL    // periode baca MAX31855 (konversi internal ~100 ms)
#define SENSOR_PUBLISH_EVERY   5        // kirim ke web/logger & grafik LCD tiap 5 sampel = 1 Hz (seperti sebelumnya)
#define SENSOR_MAX_JUMP_C      5.0f     // beda > ini dari sampel valid terakhir = lonjakan, ditolak
                                        // (minyak fisik tidak berubah 5 C dalam 200 ms)
#define SENSOR_HOLD_MS         3000UL   // tahan nilai tersaring terakhir maks selama ini saat error
static_assert(SENSOR_HOLD_MS < SENSOR_FAULT_MS, "SENSOR_HOLD_MS harus < SENSOR_FAULT_MS (pengaman sensor)");

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
  <h1>Deep Fryer - Monitor &amp; Kontrol</h1>
  <div id="bench" class="err" style="font-weight:700"></div>

  <div class="box">
    <h2>Suhu Minyak</h2>
    <div id="temp">--.- &deg;C</div>
    <div id="status" class="err">Menghubungkan...</div>
    <canvas id="c" width="640" height="160"></canvas>
    <div><small><span style="color:#4ade80">&#9473; suhu</span> &nbsp;
      <span style="color:#f59e0b">&#9476; setpoint</span></small></div>
    <div style="margin-top:6px">Setpoint: <b id="spv">--</b> &deg;C &nbsp;
      <input id="spin" type="number" min="20" max="200" step="0.5">
      <button onclick="sendSp()">Set</button></div>
    <small>Setpoint saat ini hanya REFERENSI grafik (firmware belum punya PID).</small>
  </div>

  <div class="box">
    <h2>Servo Katup Gas (0-180&deg;)</h2>
    <div class="pos">Sudut: <span id="sva">0</span>&deg;
      <small>(target <span id="svg">0</span>&deg; | &asymp; <span id="svp">0</span>% bukaan <span id="svr"></span>)</small></div>
    <div id="svl" class="err" style="font-weight:700;min-height:18px"></div>
    <input id="svs" type="range" min="0" max="55" value="0"
           oninput="$('svv').textContent=this.value" onchange="send('sva,'+this.value)">
    <div>Slider: <span id="svv">0</span>&deg; (0&deg; = TUTUP, 55&deg; = BUKA)</div>
    <button onclick="send('sv,0')" style="font-size:1.2em">TUTUP (0&deg;)</button>
    <button onclick="send('sv,100')" style="font-size:1.2em">BUKA (55&deg;)</button><br>
    <div style="margin-top:8px">Bukaan:
      <button onclick="send('sv,0')">0%</button>
      <button onclick="send('sv,25')">25%</button>
      <button onclick="send('sv,50')">50%</button>
      <button onclick="send('sv,75')">75%</button>
      <button onclick="send('sv,100')">100%</button></div>
    <div style="margin-top:8px">Halus:
      <button onclick="send('svad,-5')">-5</button>
      <button onclick="send('svad,-1')">-1</button>
      <button onclick="send('svad,1')">+1</button>
      <button onclick="send('svad,5')">+5</button></div>
    <small>Sudut dikunci 0&deg;..55&deg; (tutup..buka). Tidak ada kalibrasi lagi.</small>
    <div style="display:none">
      <b id="svcd"></b><b id="svod"></b>
      <input id="sumin" type="number"><input id="sumax" type="number">
    </div>
  </div>

  <div class="box">
    <h2>Step Test (open-loop)</h2>
    <div>Bukaan: <input id="stang" type="number" value="30" min="1" max="55" step="1"> &deg; (maks 55)
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
    atau lewat batas waktu.<br>Pin: RPWM GPIO26, LPWM GPIO27, EN GPIO14 (arah belum diverifikasi).</small>
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
  <small id="info"></small>

<script>
  const MAXPTS = 120, data = [];
  const $ = id => document.getElementById(id);
  const cv = $('c'), g = cv.getContext('2d');
  let sock = null, enState = 0, ledsBuilt = false;

  function send(m){ if (sock && sock.readyState === 1) sock.send(m); else $('log').textContent='Belum terhubung'; }
  function setSA(ax){ send('spd,'+ax+','+$('spd'+ax).value); send('acc,'+ax+','+$('acc'+ax).value); }
  function toggleEn(){ send('en,' + (enState ? 0 : 1)); }
  function sendSp(){
    const v = parseFloat($('spin').value);
    if (!isFinite(v) || v < 20 || v > 200) { $('log').textContent = 'Setpoint harus 20-200 C'; return; }
    send('sp,' + v.toFixed(1));
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
    const W = Math.min(60, Math.floor(n/2));
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
    const ang = parseFloat($('stang').value), u = ang / 55 * 100;   // % bukaan (0..55 deg = 0..100%)
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

  function setField(id, val){ const el=$(id); if (document.activeElement !== el) el.value = val; }

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

      // servo
      $('sva').textContent = m.sa; $('svg').textContent = m.sg;
      const span = m.od - m.cd;
      $('svp').textContent = span ? Math.round((m.sa - m.cd) / span * 100) : 0;
      $('svr').textContent = m.sr ? '| PULSA MENTAH ' + m.sr + ' us' : '';
      $('svcd').textContent = m.cd; $('svod').textContent = m.od;
      if (document.activeElement !== $('svs')) { $('svs').value = m.sg; $('svv').textContent = m.sg; }
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
    if (stState == 1) { stepTestEnd(3, "Step test DIBATALKAN (STOP)"); return; }
    sendMsg("Semua gerak dihentikan"); return;
  }
  if (strcmp(cmd, "stx") == 0) {
    if (stState == 1) stepTestEnd(3, "Step test DIBATALKAN");
    else sendMsg("Tidak ada step test berjalan");
    return;
  }
  { float fa, fm; int dm;
    if (sscanf(cmd, "stt,%f,%f,%d", &fa, &fm, &dm) == 3) { stepTestStart(fa, fm, dm); return; } }
  if (stState == 1 && strncmp(cmd, "sv", 2) == 0) { sendMsg("Step test berjalan: servo dikunci (batal dulu)"); return; }
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
    if (a < SERVO_US_ABS_MIN || a > SERVO_US_ABS_MAX || b < SERVO_US_ABS_MIN || b > SERVO_US_ABS_MAX || abs(a - b) < 200) {
      sendMsg("Kalibrasi pulsa ditolak: 300-3000 us dan selisih minimal 200"); return;
    }
    svUsMin = a; svUsMax = b;
    servoSaveCal();
    sendMsg("Kalibrasi pulsa disimpan");
    return;
  }
  if (sscanf(cmd, "svset,%c", &ax) == 1 && (ax == 'c' || ax == 'o')) {
    if (svRawUs > 0) { sendMsg("Keluar dari mode pulsa mentah dulu (Kembali ke sudut)"); return; }
    int deg = (int)(svTargetDeg + 0.5f);
    if (ax == 'c') svClosedDeg = deg; else svOpenDeg = deg;
    servoSaveCal();
    sendMsg(ax == 'c' ? "Sudut TUTUP disimpan" : "Sudut BUKA disimpan");
    return;
  }
  if (sscanf(cmd, "svus,%d", &v) == 1) {
    if (v == 0) { svRawUs = 0; sendMsg("Kembali ke mode sudut"); return; }
    if (v < SERVO_US_ABS_MIN || v > SERVO_US_ABS_MAX) { sendMsg("Pulsa di luar 300-3000 us"); return; }
    svRawUs = v;
    return;
  }
  if (sscanf(cmd, "svad,%d", &v) == 1) { setServoAngle(svTargetDeg + v); return; }
  if (sscanf(cmd, "sva,%d", &v) == 1)  { setServoAngle((float)v); return; }
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
  if (type == WStype_TEXT && length > 0 && length < 48) {
    char buf[48];
    memcpy(buf, payload, length);
    buf[length] = 0;
    handleCommand(buf);
  }
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
    "\"dcc\":%d,\"dct\":%d,\"dcm\":%d,\"dcv\":%d,\"sp\":%.1f}",
    (int)BENCH_MODE, digitalRead(PIN_X_DIR), stepX.currentPosition(), stepX.distanceToGo() != 0,
    driversEnabled, spdX, accX,
    (int)lroundf(svCurDeg), (int)lroundf(svTargetDeg), (int)svRawUs, (int)svLock, svUsMin, svUsMax, svClosedDeg, svOpenDeg, (int)TEMP_CUTOFF_C,
    (int)lroundf(dcCur), (int)dcTarget, (int)(dcCap() * 100.0f + 0.5f), (int)MOTOR_SUPPLY_V, (float)gSetpoint);
  ws.broadcastTXT(b);
}

void broadcastTemp() {
  float t = gTemp;
  uint32_t r = gRaw;
  const char* d = rawDiag(r);
  float sp = gSetpoint;
  unsigned long seq = gTempSeq, ms = gTempMs;
  // rev 15: tr = mentah (null bila fault), held = nilai ditahan, ec = jumlah error sejak boot
  float tr = gTempRaw;
  char trs[16];
  if (isnan(tr)) strcpy(trs, "null"); else snprintf(trs, sizeof(trs), "%.2f", tr);
  int held = gTempHeld ? 1 : 0;
  unsigned long ec = gErrCount;
  char b[400];
  // sp, seq, ms (rev 12): setpoint, nomor urut sampel, waktu sampel ESP32 (ms)
  if (isnan(t)) snprintf(b, sizeof(b), "{\"type\":\"temp\",\"t\":null,\"ok\":false,\"f\":%d,\"raw\":\"%08lX\",\"d\":\"%s\",\"up\":%lu,\"sp\":%.1f,\"seq\":%lu,\"ms\":%lu,\"st\":%d,\"te\":%lu,\"tr\":%s,\"held\":%d,\"ec\":%lu}",
                         (int)gFault, (unsigned long)r, d, millis() / 1000, sp, seq, ms, (int)stState, (unsigned long)stElapsedS(), trs, held, ec);
  else          snprintf(b, sizeof(b), "{\"type\":\"temp\",\"t\":%.2f,\"ok\":true,\"raw\":\"%08lX\",\"d\":\"%s\",\"up\":%lu,\"sp\":%.1f,\"seq\":%lu,\"ms\":%lu,\"st\":%d,\"te\":%lu,\"tr\":%s,\"held\":%d,\"ec\":%lu}",
                         t, (unsigned long)r, d, millis() / 1000, sp, seq, ms, (int)stState, (unsigned long)stElapsedS(), trs, held, ec);
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

void uiUpdate() {
  static Field fTemp, fSens, fZ, fX, fDrv, fSv, fMsg, fWifi;
  static int8_t lHit[LS_COUNT] = { -1, -1, -1, -1 }, lRaw[LS_COUNT] = { -1, -1, -1, -1 };
  char b[80];

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

  snprintf(b, sizeof(b), "WiFi AP: %s  IP: %s  klien:%d  up:%lus", AP_SSID, AP_IP.toString().c_str(), WiFi.softAPgetStationNum(), millis() / 1000);
  putText(fWifi, b, 6, 302, 1, TFT_DARKGREY, 470);
}

void tftHardReset() {
  pinMode(PIN_TFT_RST, OUTPUT);
  digitalWrite(PIN_TFT_RST, HIGH); delay(10);
  digitalWrite(PIN_TFT_RST, LOW);  delay(50);
  digitalWrite(PIN_TFT_RST, HIGH); delay(150);
}

// ---------------- FILTER SENSOR SUHU (rev 15) ----------------
// Urutan per sampel (200 ms): fault MAX31855 / NaN -> tidak valid; beda > SENSOR_MAX_JUMP_C
// dari sampel valid terakhir -> lonjakan, tidak valid. Sampel valid masuk jendela SG.
// Sampel tidak valid: HOLD (output = nilai tersaring terakhir, nilai itu juga diisikan ke
// jendela supaya jarak waktu antar sampel tetap seragam) selama <= SENSOR_HOLD_MS sejak sampel
// valid terakhir. Lewat itu: output NAN (perilaku lama -> pengaman servo/step test bekerja),
// jendela dikosongkan, dan sampel valid berikutnya diterima tanpa cek lonjakan sebagai acuan
// baru (agar acuan yang keliru tidak mengunci sensor selamanya).
//
// SAVITZKY-GOLAY KAUSAL: jendela 11 sampel, polinomial orde 2, dievaluasi di titik TERAKHIR
// jendela (end-point) sehingga hanya memakai sampel masa lalu. Koefisien dibuat di PC, BUKAN
// dihitung di ESP32:
//     from scipy.signal import savgol_coeffs
//     savgol_coeffs(11, 2, pos=10, use='dot')   # scipy 1.10.1; urutan sampel terlama -> terbaru
// Hasilnya pecahan eksak berpenyebut 143: [18, 2, -9, -15, -16, -12, -3, 11, 30, 54, 83] / 143,
// jumlah = 143/143 = 1 (gain DC = 1).
// SIFAT (diuji di Python, lihat laporan rev 15), dan BUKAN "jeda ~5 sampel": karena fit
// polinomial orde 2 dievaluasi di titik ujung, rampa dan kurva kuadrat diteruskan TANPA jeda
// (kenaikan suhu linear tidak tertunda, jadi dead time step test tidak bertambah oleh filter
// untuk respons yang mulus). Harganya: penghalusan lemah (std noise turun hanya ke ~76%),
// 58% lonjakan yang lolos prefilter ikut masuk ke output, dan respons terhadap loncatan
// mendadak overshoot ~25% lalu undershoot ~14% selama ~2 detik. Jadi nilai SG pada transien
// tajam BISA sesaat lebih tinggi/rendah dari mentah. Prefilter lonjakan di atas menahan
// sebagian besar lonjakan sebelum masuk SG.
// Awal data / setelah reset: output = sampel mentah sampai jendela penuh (11 sampel = 2,2 s).
#define SG_WINDOW 11
static const float SG_COEF[SG_WINDOW] = {
  18.0f / 143, 2.0f / 143, -9.0f / 143, -15.0f / 143, -16.0f / 143, -12.0f / 143,
  -3.0f / 143, 11.0f / 143, 30.0f / 143, 54.0f / 143, 83.0f / 143
};

struct SensorFilter {
  float    win[SG_WINDOW];   // buffer sirkular
  uint8_t  n = 0, head = 0;  // jumlah terisi, posisi tulis berikutnya
  bool     haveRef = false;  // ada sampel valid acuan
  float    lastGood = NAN;   // sampel valid terakhir (acuan lonjakan)
  float    lastOut  = NAN;   // output tersaring terakhir (yang ditahan saat error)
  uint32_t lastGoodMs = 0;

  void reset() { n = 0; head = 0; haveRef = false; }
  void push(float v) { win[head] = v; head = (head + 1) % SG_WINDOW; if (n < SG_WINDOW) n++; }
  float sg() const {                       // jendela penuh: urut terlama -> terbaru
    float s = 0;
    for (int k = 0; k < SG_WINDOW; k++) s += SG_COEF[k] * win[(head + k) % SG_WINDOW];
    return s;
  }
};
SensorFilter sf;

// Proses satu sampel. Mengembalikan nilai untuk gTemp; *held = nilai ditahan;
// *accepted = sampel valid diterima (hanya ini yang memperbarui gLastValidMs).
float sensorFilterStep(float tv, uint32_t now, bool* held, bool* accepted) {
  *held = false; *accepted = false;
  bool valid = !isnan(tv);
#if SENSOR_FILTER_ENABLE
  if (valid && sf.haveRef && fabsf(tv - sf.lastGood) > SENSOR_MAX_JUMP_C) valid = false;   // lonjakan
  if (!valid) gErrCount = gErrCount + 1;
  if (valid) {
    sf.haveRef = true; sf.lastGood = tv; sf.lastGoodMs = now;
    sf.push(tv);
    sf.lastOut = (sf.n < SG_WINDOW) ? tv : sf.sg();
    *accepted = true;
    return sf.lastOut;
  }
  if (sf.haveRef && (now - sf.lastGoodMs) <= SENSOR_HOLD_MS) {   // HOLD
    sf.push(sf.lastOut);
    *held = true;
    return sf.lastOut;
  }
  sf.reset();                                                     // lewat batas hold
  return NAN;
#else
  if (!valid) gErrCount = gErrCount + 1;
  *accepted = valid;
  return tv;                                                      // perilaku lama
#endif
}

void uiTask(void*) {
  tftHardReset();
  tft.init();
  tft.setRotation(1);               // landscape 480x320
  uiStatic();
  if (!thermocouple.begin()) Serial.println(F("MAX31855 begin() GAGAL: cek wiring CS5, SO19, SCK18, VCC 3V3"));

  uint32_t lastTemp = 0, lastUi = 0;
  uint8_t  pubCnt = SENSOR_PUBLISH_EVERY - 1;     // sampel pertama langsung dikirim
  bool first = true;
  for (;;) {
    uint32_t now = millis();
    if (first || now - lastTemp >= SENSOR_SAMPLE_MS) {
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
        gTempMs = now;
        gTempSeq = gTempSeq + 1;
        gTempFresh = true;
        chartPush(out);
        chartDraw();
      }
    }
    if (now - lastUi >= 150) { lastUi = now; uiUpdate(); }   // laju LCD seperti sebelumnya
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
  prefs.end();
  gSetpoint = (isfinite(spSaved) && spSaved >= SETPOINT_MIN_C && spSaved <= SETPOINT_MAX_C)
              ? spSaved : DEFAULT_SETPOINT_C;
  svClosedDeg = DEFAULT_SERVO_CLOSED_DEG;   // sudut tutup/buka dipaku (tidak diambil dari flash)
  svOpenDeg   = DEFAULT_SERVO_OPEN_DEG;
  svTargetDeg = svCurDeg = (float)svClosedDeg;
  servoPwmBegin();
  servoWriteUs(servoUsFromDeg(svCurDeg));

  buzzInit();

  Serial.begin(115200);
  delay(200);

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

  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.begin();
  ws.begin();
  ws.onEvent(onWsEvent);

  xTaskCreatePinnedToCore(uiTask, "ui", 8192, NULL, 1, NULL, 0);
  buzzPlay(1);                      // bip pendek saat boot = buzzer hidup
}

void loop() {
  server.handleClient();
  ws.loop();

  enforceSafety();
  stepX.run();
  stepTestUpdate();
  servoUpdate();
  dcUpdate();
  buzzerUpdate();

  if (gTempFresh) { gTempFresh = false; broadcastTemp(); }

  unsigned long now = millis();
  if (now - lastStatus >= 250) { lastStatus = now; broadcastStatus(); }
}