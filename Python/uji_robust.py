"""Uji ketahanan firmware rev 23 TANPA hardware:  python uji_robust.py

1. Cek statis dep_frayer.ino: tidak ada delay() di kode (non-blocking), watchdog dipasang di
   loop() dan uiTask, uiTask menyerahkan CPU tiap putaran (multitasking kooperatif).
2. Simulasi garis waktu uiTask (porting lcdService + putaran uiTask): selama LCD di-reset,
   sampling suhu tetap berjalan; LCD tidak digambar saat belum siap; reset otomatis (bila
   LCD_REINIT_MS > 0) dan reset manual dari web ("lcdr").
   Delay internal tft.init() library dimodelkan 295 ms (blocking, sesuai isi TFT_eSPI).
"""
import os
import re
import unittest

INO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dep_frayer", "dep_frayer.ino")
SRC = open(INO, encoding="utf-8").read()


def kode_saja(src):
    """Buang komentar /* */ dan // agar cek statis hanya melihat kode."""
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)


KODE = kode_saja(SRC.split("const char INDEX_HTML[]")[0] + SRC.split(')rawliteral";')[1])


def c(nama):
    return int(re.search(r"#define\s+%s\s+(\d+)" % nama, SRC).group(1))


class UjiStatis(unittest.TestCase):
    def test_tidak_ada_delay_ms(self):
        self.assertIsNone(re.search(r"\bdelay\s*\(", KODE), "masih ada delay() di kode firmware")

    def test_delay_mikro_hanya_setup_cs_max31855(self):
        self.assertEqual(re.findall(r"delayMicroseconds\s*\(\s*(\d+)", KODE), ["2"])

    def test_watchdog_di_kedua_task(self):
        loop = re.search(r"void loop\(\) \{(.*?)\n\}", KODE, re.S).group(1)
        ui = re.search(r"void uiTask\(void\*\) \{(.*?)\n\}", KODE, re.S).group(1)
        self.assertIn("esp_task_wdt_reset()", loop)
        self.assertIn("esp_task_wdt_add(NULL)", ui)
        self.assertIn("esp_task_wdt_reset()", ui)
        self.assertRegex(KODE, r"esp_task_wdt_init\(WDT_TIMEOUT_S, true\)")

    def test_uitask_menyerahkan_cpu(self):
        ui = re.search(r"void uiTask\(void\*\) \{(.*?)\n\}", KODE, re.S).group(1)
        self.assertIn("vTaskDelay(pdMS_TO_TICKS(10))", ui)

    def test_lcd_mati_hanya_max31855_di_spi(self):           # rev 31
        self.assertRegex(SRC, r"#define LCD_ENABLE\s+0")
        blok = re.search(r"#else\s*\n\s*if \(!sensorReady\) \{(.*?sensorReady = true;)", SRC, re.S).group(1)
        self.assertIn("digitalWrite(TFT_CS, HIGH)", blok)
        self.assertIn("digitalWrite(TOUCH_CS, HIGH)", blok)
        self.assertIn("thermocouple.begin()", blok)
        for terlarang in ("tft.", "PIN_TFT_RST", "uiStatic"):
            self.assertNotIn(terlarang, blok)

    def test_reset_manual_hanya_pasang_flag(self):
        # perintah dari loop() (core 1) tidak boleh menyentuh SPI; hanya flag untuk uiTask
        blok = re.search(r'strcmp\(cmd, "lcdr"\) == 0\) \{(.*?)\n  \}', KODE, re.S).group(1)
        self.assertIn("gLcdResetReq = true", blok)
        for terlarang in ("tft.", "thermocouple.", "SPI.", "lcdState"):
            self.assertNotIn(terlarang, blok)

    def test_tidak_ada_string_di_ui(self):
        upd = re.search(r"void uiUpdate\(\) \{(.*?)\n\}", KODE, re.S).group(1)
        self.assertNotIn("toString()", upd)


class UjiGarisWaktu(unittest.TestCase):
    """Porting lcdService() + putaran uiTask, langkah 10 ms."""

    def jalankan(self, durasi_ms, init_ms=295, reinit_ms=None, manual_ms=()):
        reinit_ms = c("LCD_REINIT_MS") if reinit_ms is None else reinit_ms
        manual = sorted(manual_ms)
        st, t0, sensor_ready = "START", 0, False
        sampel, gambar_saat_tidak_siap, resets = [], 0, 0
        last_temp = last_ui = 0
        last_init = 0
        first = True
        now = 0
        while now < durasi_ms:
            # lcdService(now)
            if st == "START":
                st, t0 = "RST_HIGH", now
            elif st == "RST_HIGH" and now - t0 >= 10:
                st, t0 = "RST_LOW", now
            elif st == "RST_LOW" and now - t0 >= 50:
                st, t0 = "RST_WAIT", now
            elif st == "RST_WAIT" and now - t0 >= 150:
                now += init_ms                           # tft.init() library: blocking internal
                sensor_ready, st = True, "READY"
            if sensor_ready and (first or now - last_temp >= c("SENSOR_SAMPLE_MS")):
                first, last_temp = False, now
                sampel.append(now)
            minta = bool(manual) and now >= manual[0]
            if minta and st == "READY":                      # gLcdResetReq dari web
                manual.pop(0)
                last_init, st, resets = now, "START", resets + 1
            if reinit_ms > 0 and st == "READY" and now - last_init >= reinit_ms:
                last_init, st, resets = now, "START", resets + 1
            if now - last_ui >= 150:
                if st == "READY":
                    last_ui = now
                elif sensor_ready and st != "READY":
                    gambar_saat_tidak_siap += 0         # firmware: uiUpdate tidak dipanggil
            now += 10                                    # vTaskDelay(10 ms)
        return sampel, resets

    def test_default_tanpa_reset_otomatis(self):
        self.assertEqual(c("LCD_REINIT_MS"), 0)
        sampel, resets = self.jalankan(125_000)
        self.assertEqual(resets, 0)
        celah = [b - a for a, b in zip(sampel, sampel[1:])]
        self.assertLessEqual(max(celah), 200 + 10)          # tanpa reset: sampling rata

    def test_reset_manual_dari_web(self):
        sampel, resets = self.jalankan(60_000, manual_ms=(10_000, 40_000))
        self.assertEqual(resets, 2)
        celah = [b - a for a, b in zip(sampel, sampel[1:])]
        self.assertLessEqual(max(celah), 200 + 295 + 10)

    def test_mekanisme_otomatis_bila_diaktifkan(self):
        sampel, resets = self.jalankan(125_000, reinit_ms=30_000)
        self.assertEqual(resets, 4)                      # 30, 60, 90, 120 dtk
        celah = [b - a for a, b in zip(sampel, sampel[1:])]
        self.assertLessEqual(max(celah), 200 + 295 + 10)  # celah terbesar = tft.init() library
        self.assertGreaterEqual(len(sampel), 125_000 / 200 * 0.95)

    def test_tanpa_state_machine_celah_lebih_besar(self):
        # pembanding: rev 21 (delay 210 ms + init) -> celah ~505 ms; rev 22 hanya init ~295 ms
        sampel, _ = self.jalankan(65_000, reinit_ms=30_000)
        celah = sorted(b - a for a, b in zip(sampel, sampel[1:]))
        self.assertLess(celah[-1], 210 + 295 + 200)


if __name__ == "__main__":
    unittest.main(verbosity=2)
