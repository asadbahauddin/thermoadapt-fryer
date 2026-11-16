/*
 * DEEP FRYER CERDAS - LCD DI ARDUINO UNO (pasangan firmware ESP32 rev 40)
 * (nama file tetap lcd_mega.ino; isi untuk Arduino UNO)
 * UNO hanya MENAMPILKAN data; semua kontrol tetap di ESP32 (web). Satu arah: ESP32 -> UNO.
 *
 * LCD: shield TFT 2.4" untuk Arduino UNO (jenis MCUFRIEND, 320x240, data paralel 8-bit, 5 V).
 *   Shield memakai D2-D9 (data), A0-A4 (kontrol), D10-D13 (slot SD, tidak dipakai). Touch tidak dipakai.
 *
 * WIRING SERIAL (SoftwareSerial, karena pin 0/1 dipakai USB dan tertutup shield)
 *   lewat level shifter bidirectional (BSS138): LV = 3V3 ESP32, HV = 5V UNO, GND kedua sisi disatukan
 *   ESP32 GPIO22 (TX2) -> LV1 ... HV1 -> UNO A5      19200 bps, 8N1
 *   ESP32 GND -> GND level shifter -> UNO GND        (WAJIB, ground bersama; mis. pin 6 header ICSP)
 *   A5 tertutup shield: solder kabel di kaki A5 bagian bawah UNO, atau pakai header susun (stacking).
 *   Pin 11 dipakai sebagai TX SoftwareSerial (tidak disambung ke mana pun).
 *
 * PROTOKOL (dari ESP32, 1 Hz + saat limit/tahap/PID berubah), satu baris:
 *   $DF,t,held,fault,sp,se,pe,am,ph,pu,pf,sv,uj,us,uf,ls,dc,xm,xp,pesan*CS\n
 *   CS = XOR semua byte di antara '$' dan '*', dua digit hex. Bingkai dengan CS salah dibuang.
 *
 * RAM UNO hanya 2 KB: teks format disimpan di flash (PSTR/F), elemen layar hanya menyimpan hash.
 * LIBRARY: MCUFRIEND_kbv + Adafruit GFX Library (Library Manager). Board: Arduino Uno.
 * Bila layar putih: jalankan contoh MCUFRIEND_kbv "LCD_ID_readreg", catat ID, isi LCD_FORCE_ID.
 */
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <SoftwareSerial.h>

#define PIN_LINK_RX      A5
#define PIN_LINK_TX      11        // wajib diisi untuk SoftwareSerial; tidak dipakai
#define LINK_BAUD        19200     // harus sama dengan MEGA_BAUD di firmware ESP32
#define LINK_TIMEOUT_MS  3000UL    // tanpa bingkai valid > 3 dtk -> tampil "TIDAK ADA DATA"
#define LCD_FORCE_ID     0         // 0 = deteksi otomatis; isi mis. 0x9341 bila deteksi gagal
#define LCD_ROTATION     1         // 1 = landscape 320x240; 3 = landscape terbalik

MCUFRIEND_kbv tft;
SoftwareSerial link(PIN_LINK_RX, PIN_LINK_TX);

// warna RGB565
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_RED     0xF800
#define C_GREEN   0x07E0
#define C_YELLOW  0xFFE0
#define C_CYAN    0x07FF
#define C_ORANGE  0xFD20
#define C_GREY    0x7BEF
#define C_LGREY   0xC618
#define C_DGREEN  0x03E0
#define C_MAROON  0x7800

// elemen teks LCD: cukup hash teks terakhir + warna (hemat RAM)
struct Field { uint16_t hash; uint16_t color; };

// ---------------- data dari ESP32 ----------------
struct Data {
  float t, sp, se, pu, pf;
  int held, fault, pe, am, ph, sv, uj, ls, dc, xm;
  long us, uf, xp;
  char msg[48];
};
Data d;
bool haveData = false;
uint32_t lastFrameMs = 0, nOk = 0, nBad = 0;

char line[180];
uint8_t lineLen = 0;
bool newSample = false;
uint32_t lastPush = 0;

#define CH_N 100                      // 100 titik = 100 detik pada 1 Hz
int16_t chartBuf[CH_N];               // suhu x10 (0,1 C); INT16_MIN = tidak ada data
uint8_t chartCnt = 0;

int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

float toF(const char* s) { return (strcmp_P(s, PSTR("nan")) == 0) ? NAN : (float)atof(s); }

// pecah bingkai; true bila valid
bool parseFrame(char* s) {
  if (s[0] != '$') return false;
  char* star = strrchr(s, '*');
  if (!star || hexVal(star[1]) < 0 || hexVal(star[2]) < 0) return false;
  uint8_t cs = 0;
  for (char* p = s + 1; p < star; p++) cs ^= (uint8_t)*p;
  if (cs != (uint8_t)(hexVal(star[1]) * 16 + hexVal(star[2]))) return false;
  *star = 0;
  if (strncmp_P(s, PSTR("$DF,"), 4) != 0) return false;

  char* f[19];
  uint8_t n = 0;
  char* p = s + 4;
  f[n++] = p;
  while (*p && n < 19) { if (*p == ',') { *p = 0; f[n++] = p + 1; } p++; }
  if (n != 19) return false;                    // 18 field angka + pesan

  d.t = toF(f[0]); d.held = atoi(f[1]); d.fault = atoi(f[2]); d.sp = toF(f[3]); d.se = toF(f[4]);
  d.pe = atoi(f[5]); d.am = atoi(f[6]); d.ph = atoi(f[7]); d.pu = toF(f[8]); d.pf = toF(f[9]);
  d.sv = atoi(f[10]); d.uj = atoi(f[11]); d.us = atol(f[12]); d.uf = atol(f[13]); d.ls = atoi(f[14]);
  d.dc = atoi(f[15]); d.xm = atoi(f[16]); d.xp = atol(f[17]);
  strncpy(d.msg, f[18], sizeof(d.msg) - 1); d.msg[sizeof(d.msg) - 1] = 0;
  return true;
}

void chartPush(float v) {
  int16_t x = isnan(v) ? INT16_MIN : (int16_t)lroundf(v * 10);
  if (chartCnt < CH_N) chartBuf[chartCnt++] = x;
  else { memmove(chartBuf, chartBuf + 1, (CH_N - 1) * sizeof(int16_t)); chartBuf[CH_N - 1] = x; }
}

// baca serial; dipanggil sering (juga di sela menggambar) agar buffer 64 byte SoftwareSerial tidak penuh
void pollLink() {
  while (link.available()) {
    char c = link.read();
    if (c == '$') lineLen = 0;                        // sinkron ulang di awal bingkai
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        line[lineLen] = 0;
        if (parseFrame(line)) {
          nOk++;
          lastFrameMs = millis();
          bool first = !haveData;
          haveData = true;
          if (first || millis() - lastPush >= 900) { lastPush = millis(); chartPush(d.t); newSample = true; }
        } else nBad++;
      }
      lineLen = 0;
    } else if (lineLen < sizeof(line) - 1) line[lineLen++] = c;
    else lineLen = 0;                                  // baris terlalu panjang: buang
  }
}

// ---------------- tampilan 320x240 ----------------
uint16_t hashStr(const char* s) { uint16_t h = 5381; while (*s) h = (h << 5) + h + (uint8_t)*s++; return h; }

// gambar teks hanya bila berubah; latar hitam menimpa teks lama sepanjang lebar w
void putText(Field& f, const char* s, int x, int y, uint8_t size, uint16_t color, int w) {
  pollLink();
  uint16_t h = hashStr(s);
  if (f.color == color && f.hash == h) return;
  f.hash = h; f.color = color;
  tft.fillRect(x, y, w, 8 * size, C_BLACK);
  tft.setTextSize(size);
  tft.setTextColor(color, C_BLACK);
  tft.setCursor(x, y);
  tft.print(s);
}

const char LS0[] PROGMEM = "Z Atas";
const char LS1[] PROGMEM = "Z Bawah";
const char LS2[] PROGMEM = "X Awal";
const char LS3[] PROGMEM = "X Akhir";
const char* const LS_NAME[4] PROGMEM = { LS0, LS1, LS2, LS3 };

void drawLimitBox(uint8_t i, bool hit) {
  pollLink();
  int x = 2 + i * 80, y = 150;
  uint16_t bg = hit ? C_MAROON : C_DGREEN;
  tft.fillRect(x, y, 76, 36, bg);
  tft.setTextColor(C_WHITE, bg);
  tft.setTextSize(1);
  tft.setCursor(x + 4, y + 4);
  tft.print((const __FlashStringHelper*)pgm_read_ptr(&LS_NAME[i]));
  tft.setTextSize(2);
  tft.setCursor(x + 4, y + 16);
  tft.print(hit ? F("KENA") : F("BEBAS"));
}

void chartDraw() {
  const int X = 190, Y = 22, W = 128, H = 44;
  pollLink();
  tft.fillRect(X, Y, W, H, C_BLACK);
  tft.drawRect(X, Y, W, H, C_GREY);
  int16_t lo = INT16_MAX, hi = INT16_MIN + 1; uint8_t n = 0;
  for (uint8_t k = 0; k < chartCnt; k++) if (chartBuf[k] != INT16_MIN) { lo = min(lo, chartBuf[k]); hi = max(hi, chartBuf[k]); n++; }
  if (n < 2) return;
  int16_t sp10 = (!isnan(d.sp) && d.sp > 0) ? (int16_t)lroundf(d.sp * 10) : INT16_MIN;
  if (sp10 != INT16_MIN) { lo = min(lo, sp10); hi = max(hi, sp10); }
  if (hi - lo < 50) { int16_t m = (hi + lo) / 2; lo = m - 25; hi = m + 25; }
  long span = hi - lo;
  if (sp10 != INT16_MIN) {            // garis setpoint (putus-putus)
    int ys = Y + H - 3 - (int)((long)(sp10 - lo) * (H - 6) / span);
    for (int x = X + 2; x < X + W - 2; x += 6) tft.drawFastHLine(x, ys, 3, C_ORANGE);
  }
  int px = -1, py = 0;
  for (uint8_t k = 0; k < chartCnt; k++) {
    if (chartBuf[k] == INT16_MIN) { px = -1; continue; }
    int x = X + 2 + (long)k * (W - 4) / (CH_N - 1);
    int y = Y + H - 3 - (int)((long)(chartBuf[k] - lo) * (H - 6) / span);
    if (px >= 0) tft.drawLine(px, py, x, y, C_GREEN);
    px = x; py = y;
  }
  tft.setTextSize(1);
  tft.setTextColor(C_LGREY, C_BLACK);
  tft.setCursor(X + 3, Y + 2);      tft.print((hi + 5) / 10);
  tft.setCursor(X + 3, Y + H - 10); tft.print((lo + 5) / 10);
}

void ujName(char* out, uint8_t n, int uj) {
  const char* s;
  switch (uj) {
    case 0: s = PSTR("Siap"); break;
    case 1: s = PSTR("1 Z naik"); break;
    case 2: s = PSTR("2 X ke awal"); break;
    case 3: s = PSTR("3 Tunggu"); break;
    case 4: s = PSTR("4 Z turun"); break;
    case 5: s = PSTR("5 Goreng"); break;
    case 6: s = PSTR("6 Z naik"); break;
    case 7: s = PSTR("7 X peniris"); break;
    case 8: s = PSTR("Selesai"); break;
    case 9: s = PSTR("Batal"); break;
    default: s = PSTR("?");
  }
  strncpy_P(out, s, n - 1); out[n - 1] = 0;
}

void uiStatic() {
  tft.fillScreen(C_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(C_CYAN, C_BLACK);
  tft.setCursor(4, 2);
  tft.print(F("DEEP FRYER"));
  tft.drawFastHLine(0, 19, 320, C_GREY);
}

int8_t lHit[4] = { -1, -1, -1, -1 };
Field fLink, fTemp, fSp, fPid, fValve, fUj, fZX, fMsg, fFoot;

void uiUpdate() {
  char b[56], v1[10], v2[10], un[14];
  bool fresh = haveData && (millis() - lastFrameMs < LINK_TIMEOUT_MS);

  strcpy_P(b, fresh ? PSTR("ESP32 OK") : PSTR("TIDAK ADA DATA"));
  putText(fLink, b, 150, 6, 1, fresh ? C_GREEN : C_RED, 168);
  if (!haveData) {
    strcpy_P(b, PSTR("--.-C"));
    putText(fTemp, b, 4, 24, 5, C_GREY, 184);
    strcpy_P(b, PSTR("Cek kabel ESP32 GPIO22 -> A5 dan GND"));
    putText(fMsg, b, 4, 214, 1, C_YELLOW, 312);
    return;
  }

  // suhu besar (ukuran 5 = 30 px per huruf)
  if (isnan(d.t)) { strcpy_P(b, PSTR("ERR")); putText(fTemp, b, 4, 24, 5, C_RED, 184); }
  else {
    dtostrf(d.t, 0, 1, v1);
    snprintf_P(b, sizeof(b), PSTR("%sC"), v1);
    putText(fTemp, b, 4, 24, 5, !fresh ? C_GREY : (d.held ? C_YELLOW : C_WHITE), 184);
  }

  // setpoint
  dtostrf(d.sp, 0, 1, v1);
  if (isnan(d.se)) snprintf_P(b, sizeof(b), PSTR("SP %s"), v1);
  else { dtostrf(d.se, 0, 1, v2); snprintf_P(b, sizeof(b), PSTR("SP %s ramp %s"), v1, v2); }
  putText(fSp, b, 4, 72, 2, C_ORANGE, 312);

  // status PID
  uint16_t pc = C_LGREY;
  if (!d.pe) snprintf_P(b, sizeof(b), d.am ? PSTR("PID OFF (adaptif)") : PSTR("PID OFF (konv.)"));
  else {
    const char* st = PSTR("");
    pc = C_GREEN;
    if (d.ph == 1)      { st = PSTR(" TAHAN"); pc = C_YELLOW; }
    else if (d.ph == 2) { st = PSTR(" AMAN:SENS"); pc = C_RED; }
    else if (d.ph == 3) { st = PSTR(" AMAN:180"); pc = C_RED; }
    else if (d.ph == 4) { st = PSTR(" AMAN:DATA"); pc = C_RED; }
    snprintf_P(b, sizeof(b), PSTR("PID %S%S"), d.am ? PSTR("ADAPTIF") : PSTR("KONV."), st);
  }
  putText(fPid, b, 4, 92, 2, pc, 312);

  // katup
  dtostrf(d.pu, 0, 1, v1); dtostrf(d.pf, 0, 1, v2);
  snprintf_P(b, sizeof(b), PSTR("Katup %s%% FF %s%%"), v1, v2);
  putText(fValve, b, 4, 112, 2, C_CYAN, 312);

  // Uji Sistem
  ujName(un, sizeof(un), d.uj);
  bool run = d.uj >= 1 && d.uj <= 7;
  if (run) snprintf_P(b, sizeof(b), PSTR("Uji %s %lds"), un, d.us);
  else     snprintf_P(b, sizeof(b), PSTR("Uji: %s"), un);
  putText(fUj, b, 4, 132, 2, run ? C_YELLOW : C_WHITE, 312);

  // limit switch
  for (uint8_t i = 0; i < 4; i++) {
    int8_t h = (d.ls >> i) & 1;
    if (h != lHit[i]) { lHit[i] = h; drawLimitBox(i, h); }
  }

  // Z dan X
  snprintf_P(b, sizeof(b), PSTR("Z %d%% %S | X %ld %S"), d.dc,
             d.dc > 0 ? PSTR("NAIK") : (d.dc < 0 ? PSTR("TURUN") : PSTR("diam")), d.xp, d.xm ? PSTR("gerak") : PSTR("diam"));
  putText(fZX, b, 4, 194, 1, C_WHITE, 312);

  putText(fMsg, d.msg, 4, 214, 1, C_YELLOW, 312);

  snprintf_P(b, sizeof(b), PSTR("Servo %d deg | timer %lds | OK %lu rusak %lu"), d.sv, d.uf, nOk, nBad);
  putText(fFoot, b, 4, 230, 1, C_GREY, 312);
}

void setup() {
  Serial.begin(115200);                // USB: debug (ID LCD)
  link.begin(LINK_BAUD);               // A5 <- ESP32 GPIO22
  uint16_t id = LCD_FORCE_ID ? LCD_FORCE_ID : tft.readID();
  Serial.print(F("ID LCD: 0x")); Serial.println(id, HEX);
  if (id == 0xD3D3) id = 0x9486;       // anjuran MCUFRIEND_kbv untuk ID tulis-saja
  tft.begin(id);
  tft.setRotation(LCD_ROTATION);
  uiStatic();
  uiUpdate();
}

void loop() {
  static uint32_t lastUi = 0;
  pollLink();
  uint32_t now = millis();
  if (now - lastUi >= 200) {
    lastUi = now;
    uiUpdate();
    if (newSample) { newSample = false; chartDraw(); }
  }
}
