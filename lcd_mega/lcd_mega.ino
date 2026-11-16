/*
 * DEEP FRYER CERDAS - LCD DI ARDUINO MEGA 2560 (pasangan firmware ESP32 rev 39)
 * Mega hanya MENAMPILKAN data; semua kontrol tetap di ESP32 (web). Satu arah: ESP32 -> Mega.
 *
 * LCD: shield TFT 2.4" untuk Arduino (jenis MCUFRIEND, 320x240, data paralel 8-bit, 5 V).
 *   Shield ditancap langsung ke header Mega (posisi sama seperti di UNO: D0-D13, A0-A5, power).
 *   Tanpa level converter. Touch dan slot SD di shield tidak dipakai.
 *
 * WIRING SERIAL
 *   ESP32 GPIO22 (TX2) -> Mega pin 19 (RX1)      115200 bps, 8N1 (langsung, tanpa resistor)
 *   ESP32 GND          -> Mega GND               (WAJIB, ground bersama)
 *
 * PROTOKOL (dari ESP32, 1 Hz + saat limit/tahap/PID berubah), satu baris:
 *   $DF,t,held,fault,sp,se,pe,am,ph,pu,pf,sv,uj,us,uf,ls,dc,xm,xp,pesan*CS\n
 *   CS = XOR semua byte di antara '$' dan '*', dua digit hex. Bingkai dengan CS salah dibuang.
 *
 * LIBRARY: MCUFRIEND_kbv + Adafruit GFX Library (Library Manager).
 * Board: Arduino Mega or Mega 2560.
 * Bila layar putih: jalankan contoh MCUFRIEND_kbv "LCD_ID_readreg", catat ID, isi LCD_FORCE_ID.
 */
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>

#define LINK_BAUD        115200
#define LINK_TIMEOUT_MS  3000UL    // tanpa bingkai valid > 3 dtk -> tampil "TIDAK ADA DATA"
#define LCD_FORCE_ID     0         // 0 = deteksi otomatis; isi mis. 0x9341 bila deteksi gagal
#define LCD_ROTATION     1         // 1 = landscape 320x240; 3 = landscape terbalik

MCUFRIEND_kbv tft;

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

// elemen teks LCD (didefinisikan di atas agar prototipe otomatis Arduino mengenalnya)
struct Field { char last[56]; uint16_t color; };

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

char line[220];
int  lineLen = 0;

int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

float toF(const char* s) { return (strcmp(s, "nan") == 0) ? NAN : (float)atof(s); }

// pecah bingkai; true bila valid
bool parseFrame(char* s) {
  if (s[0] != '$') return false;
  char* star = strrchr(s, '*');
  if (!star || hexVal(star[1]) < 0 || hexVal(star[2]) < 0) return false;
  uint8_t cs = 0;
  for (char* p = s + 1; p < star; p++) cs ^= (uint8_t)*p;
  if (cs != (uint8_t)(hexVal(star[1]) * 16 + hexVal(star[2]))) return false;
  *star = 0;
  if (strncmp(s, "$DF,", 4) != 0) return false;

  char* f[20];
  int n = 0;
  char* p = s + 4;
  f[n++] = p;
  while (*p && n < 19) { if (*p == ',') { *p = 0; f[n++] = p + 1; } p++; }
  if (n != 19) return false;                    // 18 field angka + pesan

  Data x;
  x.t = toF(f[0]); x.held = atoi(f[1]); x.fault = atoi(f[2]); x.sp = toF(f[3]); x.se = toF(f[4]);
  x.pe = atoi(f[5]); x.am = atoi(f[6]); x.ph = atoi(f[7]); x.pu = toF(f[8]); x.pf = toF(f[9]);
  x.sv = atoi(f[10]); x.uj = atoi(f[11]); x.us = atol(f[12]); x.uf = atol(f[13]); x.ls = atoi(f[14]);
  x.dc = atoi(f[15]); x.xm = atoi(f[16]); x.xp = atol(f[17]);
  strncpy(x.msg, f[18], sizeof(x.msg) - 1); x.msg[sizeof(x.msg) - 1] = 0;
  d = x;
  return true;
}

// ---------------- tampilan 320x240 ----------------
// gambar teks hanya bila berubah; latar hitam menimpa teks lama sepanjang lebar w
void putText(Field& f, const char* s, int x, int y, int size, uint16_t color, int w) {
  if (f.color == color && strcmp(f.last, s) == 0) return;
  strncpy(f.last, s, sizeof(f.last) - 1); f.last[sizeof(f.last) - 1] = 0; f.color = color;
  tft.fillRect(x, y, w, 8 * size, C_BLACK);
  tft.setTextSize(size);
  tft.setTextColor(color, C_BLACK);
  tft.setCursor(x, y);
  tft.print(s);
}

const char* LS_NAME[4] = { "Z Atas", "Z Bawah", "X Awal", "X Akhir" };

void drawLimitBox(int i, bool hit) {
  int x = 2 + i * 80, y = 150;
  uint16_t bg = hit ? C_MAROON : C_DGREEN;
  tft.fillRect(x, y, 76, 36, bg);
  tft.setTextColor(C_WHITE, bg);
  tft.setTextSize(1);
  tft.setCursor(x + 4, y + 4);  tft.print(LS_NAME[i]);
  tft.setTextSize(2);
  tft.setCursor(x + 4, y + 16); tft.print(hit ? "KENA" : "BEBAS");
}

#define CH_N 120                      // 120 titik = 2 menit pada 1 Hz
float chartBuf[CH_N];
int   chartCnt = 0;

void chartPush(float v) {
  if (chartCnt < CH_N) chartBuf[chartCnt++] = v;
  else { memmove(chartBuf, chartBuf + 1, (CH_N - 1) * sizeof(float)); chartBuf[CH_N - 1] = v; }
}

void chartDraw() {
  const int X = 190, Y = 22, W = 128, H = 44;
  tft.fillRect(X, Y, W, H, C_BLACK);
  tft.drawRect(X, Y, W, H, C_GREY);
  float lo = 1e9, hi = -1e9; int n = 0;
  for (int k = 0; k < chartCnt; k++) if (!isnan(chartBuf[k])) { lo = min(lo, chartBuf[k]); hi = max(hi, chartBuf[k]); n++; }
  if (n < 2) return;
  bool spOk = !isnan(d.sp) && d.sp > 0;
  if (spOk) { lo = min(lo, d.sp); hi = max(hi, d.sp); }
  if (hi - lo < 5) { float m = (hi + lo) / 2; lo = m - 2.5; hi = m + 2.5; }
  if (spOk) {                         // garis setpoint (putus-putus)
    int ys = Y + H - 3 - (int)((d.sp - lo) / (hi - lo) * (H - 6));
    for (int x = X + 2; x < X + W - 2; x += 6) tft.drawFastHLine(x, ys, 3, C_ORANGE);
  }
  int px = -1, py = 0;
  for (int k = 0; k < chartCnt; k++) {
    if (isnan(chartBuf[k])) { px = -1; continue; }
    int x = X + 2 + (long)k * (W - 4) / (CH_N - 1);
    int y = Y + H - 3 - (int)((chartBuf[k] - lo) / (hi - lo) * (H - 6));
    if (px >= 0) tft.drawLine(px, py, x, y, C_GREEN);
    px = x; py = y;
  }
  char b[12];
  tft.setTextSize(1);
  tft.setTextColor(C_LGREY, C_BLACK);
  dtostrf(hi, 0, 0, b); tft.setCursor(X + 3, Y + 2);      tft.print(b);
  dtostrf(lo, 0, 0, b); tft.setCursor(X + 3, Y + H - 10); tft.print(b);
}

const char* ujName(int uj) {
  switch (uj) {
    case 0: return "Siap";
    case 1: return "1 Z naik";
    case 2: return "2 X ke awal";
    case 3: return "3 Tunggu";
    case 4: return "4 Z turun";
    case 5: return "5 Goreng";
    case 6: return "6 Z naik";
    case 7: return "7 X peniris";
    case 8: return "Selesai";
    case 9: return "Batal";
  }
  return "?";
}

void uiStatic() {
  tft.fillScreen(C_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(C_CYAN, C_BLACK);
  tft.setCursor(4, 2);
  tft.print("DEEP FRYER");
  tft.drawFastHLine(0, 19, 320, C_GREY);
}

int8_t lHit[4] = { -1, -1, -1, -1 };
Field fLink, fTemp, fSp, fPid, fValve, fUj, fZX, fMsg, fFoot;

void uiUpdate() {
  char b[64], v1[12], v2[12];
  bool fresh = haveData && (millis() - lastFrameMs < LINK_TIMEOUT_MS);

  putText(fLink, fresh ? "ESP32 OK" : "TIDAK ADA DATA", 150, 6, 1, fresh ? C_GREEN : C_RED, 168);
  if (!haveData) {
    putText(fTemp, "--.-C", 4, 24, 5, C_GREY, 184);
    putText(fMsg, "Cek kabel ESP32 GPIO22 -> RX1 (19) dan GND", 4, 214, 1, C_YELLOW, 312);
    return;
  }

  // suhu besar (ukuran 5 = 30 px per huruf)
  if (isnan(d.t)) putText(fTemp, "ERR", 4, 24, 5, C_RED, 184);
  else {
    dtostrf(d.t, 0, 1, v1);
    snprintf(b, sizeof(b), "%sC", v1);
    putText(fTemp, b, 4, 24, 5, !fresh ? C_GREY : (d.held ? C_YELLOW : C_WHITE), 184);
  }

  // setpoint
  dtostrf(d.sp, 0, 1, v1);
  if (isnan(d.se)) snprintf(b, sizeof(b), "SP %s", v1);
  else { dtostrf(d.se, 0, 1, v2); snprintf(b, sizeof(b), "SP %s ramp %s", v1, v2); }
  putText(fSp, b, 4, 72, 2, C_ORANGE, 312);

  // status PID
  uint16_t pc = C_LGREY;
  if (!d.pe) snprintf(b, sizeof(b), "PID OFF (%s)", d.am ? "adaptif" : "konv.");
  else {
    const char* st = "";
    pc = C_GREEN;
    if (d.ph == 1)      { st = " TAHAN"; pc = C_YELLOW; }
    else if (d.ph == 2) { st = " AMAN:SENS"; pc = C_RED; }
    else if (d.ph == 3) { st = " AMAN:180"; pc = C_RED; }
    else if (d.ph == 4) { st = " AMAN:DATA"; pc = C_RED; }
    snprintf(b, sizeof(b), "PID %s%s", d.am ? "ADAPTIF" : "KONV.", st);
  }
  putText(fPid, b, 4, 92, 2, pc, 312);

  // katup
  dtostrf(d.pu, 0, 1, v1); dtostrf(d.pf, 0, 1, v2);
  snprintf(b, sizeof(b), "Katup %s%% FF %s%%", v1, v2);
  putText(fValve, b, 4, 112, 2, C_CYAN, 312);

  // Uji Sistem
  if (d.uj == 5)                   snprintf(b, sizeof(b), "Uji %s %lds", ujName(d.uj), d.us);
  else if (d.uj >= 1 && d.uj <= 7) snprintf(b, sizeof(b), "Uji %s %lds", ujName(d.uj), d.us);
  else                             snprintf(b, sizeof(b), "Uji: %s", ujName(d.uj));
  putText(fUj, b, 4, 132, 2, (d.uj >= 1 && d.uj <= 7) ? C_YELLOW : C_WHITE, 312);

  // limit switch
  for (int i = 0; i < 4; i++) {
    int8_t h = (d.ls >> i) & 1;
    if (h != lHit[i]) { lHit[i] = h; drawLimitBox(i, h); }
  }

  // Z dan X
  snprintf(b, sizeof(b), "Z %d%% %s | X %ld %s", d.dc, d.dc > 0 ? "NAIK" : (d.dc < 0 ? "TURUN" : "diam"),
           d.xp, d.xm ? "gerak" : "diam");
  putText(fZX, b, 4, 194, 1, C_WHITE, 312);

  putText(fMsg, d.msg, 4, 214, 1, C_YELLOW, 312);

  snprintf(b, sizeof(b), "Servo %d deg | timer %lds | OK %lu rusak %lu", d.sv, d.uf, nOk, nBad);
  putText(fFoot, b, 4, 230, 1, C_GREY, 312);
}

void setup() {
  Serial.begin(115200);                // USB: debug
  Serial1.begin(LINK_BAUD);            // RX1 pin 19 <- ESP32 GPIO22
  uint16_t id = LCD_FORCE_ID ? LCD_FORCE_ID : tft.readID();
  Serial.print(F("ID LCD: 0x")); Serial.println(id, HEX);
  if (id == 0xD3D3) id = 0x9486;       // anjuran MCUFRIEND_kbv untuk ID tulis-saja
  tft.begin(id);
  tft.setRotation(LCD_ROTATION);
  uiStatic();
  uiUpdate();
}

void loop() {
  static uint32_t lastUi = 0, lastPush = 0;
  static bool newSample = false;

  while (Serial1.available()) {
    char c = Serial1.read();
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
    } else if (lineLen < (int)sizeof(line) - 1) line[lineLen++] = c;
    else lineLen = 0;                                  // baris terlalu panjang: buang
  }

  uint32_t now = millis();
  if (now - lastUi >= 200) {
    lastUi = now;
    uiUpdate();
    if (newSample) { newSample = false; chartDraw(); }
  }
}
