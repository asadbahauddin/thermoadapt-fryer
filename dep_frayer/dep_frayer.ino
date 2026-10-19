/*
 * DEEP FRYER CERDAS - MONITOR & KONTROL VIA WEB + LCD (rev 11)
 * ESP32 = WiFi Access Point (192.168.4.1) + WebSocket.
 * Isi: suhu MAX31855, 4 limit switch, Z = MOTOR DC (BTS7960), X = stepper (TB6600),
 *      SERVO katup gas 0-180 derajat (kalibrasi), BUZZER, LCD TFT mirror dari web.
 *
 * REV 11 (dari v10):
 *   - STEPPER Z DIHAPUS dari kode. Pin 26/27/14 sekarang milik BTS7960 (sesuai wiring-mu):
 *       GPIO26 = EN (R_EN + L_EN disatukan) | GPIO27 = LPWM | GPIO14 = RPWM
 *     Satu-satunya stepper: X (STEP32, DIR33, EN25).
 *     Pull-down 10k di EN, LPWM, RPWM ke GND (GPIO14 mengeluarkan PWM sesaat saat boot;
 *     karena EN ada di GPIO26 dan ditarik LOW, motor tetap diam saat boot).
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

#define PIN_DC_EN   26     // R_EN + L_EN BTS7960
#define PIN_DC_LPWM 27
#define PIN_DC_RPWM 14

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
  // KOREKSI penamaan (hasil pengujian fisik): pin yang dulu diberi nama lain ternyata
  //   GPIO35 = Z Atas | GPIO17 = Z Bawah | GPIO34 = X Awal | GPIO39 = X Akhir
  // interlock = true: gerak berhenti saat limit di arah geraknya kena. Status sekarang: X Awal, X Akhir, Z Bawah = true;
  // Z Atas = false sampai terbukti terbaca (tes tekan tangan, lalu "Naik 25%"). Setelah lulus, ubah jadi true.
  { "Z Atas",  35, false, true, false },   // interlock Z OFF sementara (motor DC belum final)
  { "Z Bawah", 17, false, true, true  },   // terbukti bekerja -> interlock ON
  { "X Awal",  34, false, true, true  },
  { "X Akhir", 39, false, true, true  },
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
#define X_PLUS_TOWARD_AKHIR    false   // X Akhir kena => arah - diblok, arah + boleh
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
#define DC_POS_USES_RPWM    false    // positif (naik) memakai RPWM (GPIO14); false = LPWM
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
  </div>

  <div class="box">
    <h2>Servo Katup Gas (0-180&deg;)</h2>
    <div class="pos">Sudut: <span id="sva">0</span>&deg;
      <small>(target <span id="svg">0</span>&deg; | &asymp; <span id="svp">0</span>% bukaan <span id="svr"></span>)</small></div>
    <div id="svl" class="err" style="font-weight:700;min-height:18px"></div>
    <input id="svs" type="range" min="0" max="180" value="0"
           oninput="$('svv').textContent=this.value" onchange="send('sva,'+this.value)">
    <div>Slider: <span id="svv">0</span>&deg;</div>
    <button onclick="send('sva,0')">0&deg;</button>
    <button onclick="send('sva,45')">45&deg;</button>
    <button onclick="send('sva,90')">90&deg;</button>
    <button onclick="send('sva,135')">135&deg;</button>
    <button onclick="send('sva,180')">180&deg;</button><br>
    <button onclick="send('svad,-10')">-10</button>
    <button onclick="send('svad,-5')">-5</button>
    <button onclick="send('svad,-1')">-1</button>
    <button onclick="send('svad,1')">+1</button>
    <button onclick="send('svad,5')">+5</button>
    <button onclick="send('svad,10')">+10</button><br>
    <div style="margin-top:8px">Simpan sudut sekarang sebagai:
      <button onclick="send('svset,c')">TUTUP</button>
      <button onclick="send('svset,o')">BUKA</button><br>
      <small>Tersimpan: tutup <b id="svcd">0</b>&deg;, buka <b id="svod">0</b>&deg;</small></div>
    <div style="margin-top:8px">Bukaan:
      <button onclick="send('sv,0')">0%</button>
      <button onclick="send('sv,25')">25%</button>
      <button onclick="send('sv,50')">50%</button>
      <button onclick="send('sv,75')">75%</button>
      <button onclick="send('sv,100')">100%</button></div>
    <div style="margin-top:8px">Kalibrasi pulsa: 0&deg; = <input id="sumin" type="number" min="300" max="3000"> us,
      180&deg; = <input id="sumax" type="number" min="300" max="3000"> us
      <button onclick="send('svcal,'+$('sumin').value+','+$('sumax').value)">Simpan</button></div>
    <div style="margin-top:8px">Tes pulsa mentah: <input id="sraw" type="number" value="1500" min="300" max="3000"> us
      <button onclick="send('svus,'+$('sraw').value)">Kirim</button>
      <button onclick="send('svus,0')">Kembali ke sudut</button></div>
    <small>Mode kalibrasi: TANPA pembatas. Cari ujung jangkauan servo pelan-pelan dgn tes pulsa
    mentah; pulsa di luar jangkauan membuat servo menekan stopper.</small>
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
    atau lewat batas waktu.</small>
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

  function draw(){
    g.clearRect(0,0,cv.width,cv.height);
    const v = data.filter(x => x !== null);
    if (v.length < 2) return;
    let lo = Math.min(...v), hi = Math.max(...v);
    if (hi - lo < 5) { const m=(hi+lo)/2; lo=m-2.5; hi=m+2.5; }
    g.strokeStyle='#4ade80'; g.lineWidth=2; g.beginPath();
    let started=false;
    data.forEach((val,i)=>{
      if (val === null) { started=false; return; }
      const x = i*(cv.width/(MAXPTS-1));
      const y = cv.height-10 - (val-lo)/(hi-lo)*(cv.height-20);
      if(!started){g.moveTo(x,y);started=true;} else g.lineTo(x,y);
    });
    g.stroke();
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
      if (m.ok && m.t !== null){
        $('temp').innerHTML = m.t.toFixed(1)+' &deg;C';
        $('status').textContent='Sensor OK'; $('status').className='ok'; data.push(m.t);
      } else {
        $('temp').innerHTML='ERR';
        $('status').textContent='SENSOR ERROR: ' + ({1:'thermocouple LEPAS (open circuit)',2:'short ke GND',4:'short ke VCC'}[m.f] || 'tidak ada data (cek SCK/CS/SO & VCC 3V3)'); $('status').className='err'; data.push(null);
      }
      if (data.length > MAXPTS) data.shift();
      $('info').textContent = 'Uptime ' + m.up + ' s | MAX31855 raw ' + m.raw + ' | ' + m.d;
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
  deg = constrain(deg, 0.0f, 180.0f);
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
  if (strcmp(cmd, "stop") == 0) { hardStop(stepX); dcStop(); sendMsg("Semua gerak dihentikan"); return; }
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
    "\"dcc\":%d,\"dct\":%d,\"dcm\":%d,\"dcv\":%d}",
    (int)BENCH_MODE, digitalRead(PIN_X_DIR), stepX.currentPosition(), stepX.distanceToGo() != 0,
    driversEnabled, spdX, accX,
    (int)lroundf(svCurDeg), (int)lroundf(svTargetDeg), (int)svRawUs, (int)svLock, svUsMin, svUsMax, svClosedDeg, svOpenDeg, (int)TEMP_CUTOFF_C,
    (int)lroundf(dcCur), (int)dcTarget, (int)(dcCap() * 100.0f + 0.5f), (int)MOTOR_SUPPLY_V);
  ws.broadcastTXT(b);
}

void broadcastTemp() {
  float t = gTemp;
  uint32_t r = gRaw;
  const char* d = rawDiag(r);
  char b[240];
  if (isnan(t)) snprintf(b, sizeof(b), "{\"type\":\"temp\",\"t\":null,\"ok\":false,\"f\":%d,\"raw\":\"%08lX\",\"d\":\"%s\",\"up\":%lu}",
                         (int)gFault, (unsigned long)r, d, millis() / 1000);
  else          snprintf(b, sizeof(b), "{\"type\":\"temp\",\"t\":%.2f,\"ok\":true,\"raw\":\"%08lX\",\"d\":\"%s\",\"up\":%lu}",
                         t, (unsigned long)r, d, millis() / 1000);
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
    snprintf(b, sizeof(b), "%.1f C", t);
    putText(fTemp, b, 10, 34, 6, TFT_WHITE, 250);
    snprintf(b, sizeof(b), "Sensor OK  raw %08lX", (unsigned long)gRaw);
    putText(fSens, b, 10, 86, 2, TFT_GREEN, 470);
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

void uiTask(void*) {
  tftHardReset();
  tft.init();
  tft.setRotation(1);               // landscape 480x320
  uiStatic();
  if (!thermocouple.begin()) Serial.println(F("MAX31855 begin() GAGAL: cek wiring CS5, SO19, SCK18, VCC 3V3"));

  uint32_t lastTemp = 0;
  bool first = true;
  for (;;) {
    uint32_t now = millis();
    if (first || now - lastTemp >= 1000) {
      first = false; lastTemp = now;
      gRaw = maxReadRaw();
      float tv = thermocouple.readCelsius();          // NAN kalau fault
      gFault = isnan(tv) ? thermocouple.readError() : 0;
      gTemp = tv;
      if (!isnan(tv)) gLastValidMs = millis();
      gTempFresh = true;
      chartPush(tv);
      chartDraw();
    }
    uiUpdate();
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  // Motor DC dulu: EN LOW + PWM 0
  dcInit();

  // Servo: muat kalibrasi dari flash, langsung ke sudut TUTUP
  prefs.begin("fryer", true);
  svUsMin     = prefs.getInt("usmin", DEFAULT_SERVO_US_MIN);
  svUsMax     = prefs.getInt("usmax", DEFAULT_SERVO_US_MAX);
  svClosedDeg = prefs.getInt("cdeg",  DEFAULT_SERVO_CLOSED_DEG);
  svOpenDeg   = prefs.getInt("odeg",  DEFAULT_SERVO_OPEN_DEG);
  prefs.end();
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
  servoUpdate();
  dcUpdate();
  buzzerUpdate();

  if (gTempFresh) { gTempFresh = false; broadcastTemp(); }

  unsigned long now = millis();
  if (now - lastStatus >= 250) { lastStatus = now; broadcastStatus(); }
}