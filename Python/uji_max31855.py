"""Uji dekode & validasi frame MAX31855 firmware rev 32 TANPA hardware:  python uji_max31855.py

maxDecode() di-porting baris-per-baris dari dep_frayer.ino; frame uji dibuat sesuai format datasheet
MAX31855 (D31..D18 suhu termokopel 0.25 C, D16 fault, D15..D4 cold junction 0.0625 C, D2..D0 jenis
fault, D17 & D3 selalu 0). Juga cek statis bahwa firmware membaca 2x dan tidak memakai readCelsius().
"""
import os, random, re, unittest

INO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dep_frayer", "dep_frayer.ino")
SRC = open(INO, encoding="utf-8").read()


def frame(tc, cj=30.0, fault=0):
    t = int(round(tc / 0.25)) & 0x3FFF
    c = int(round(cj / 0.0625)) & 0xFFF
    return (t << 18) | ((1 if fault else 0) << 16) | (c << 4) | (fault & 7)


def max_decode(v):
    """Porting maxDecode(): (suhu atau None, kode)."""
    if v in (0, 0xFFFFFFFF) or v & 0x00020008:
        return None, 0x10
    if v & 0x00010000:
        return None, (v & 7) if (v & 7) else 0x10
    cj = (v >> 4) & 0x0FFF
    if cj & 0x0800:
        cj -= 0x1000
    if not (-20.0 <= cj * 0.0625 <= 85.0):
        return None, 0x40
    t = (v >> 18) & 0x3FFF
    if t & 0x2000:
        t -= 0x4000
    return t * 0.25, 0


def sampel(ra, rb):
    """Porting blok sampling uiTask: baca 2x lalu dekode."""
    if ra != rb:
        return None, 0x20
    return max_decode(ra)


class UjiDekode(unittest.TestCase):
    def test_suhu_normal_dan_negatif(self):
        for tc in (65.0, 160.25, 199.75, 0.0, -10.5, 1000.0):
            self.assertEqual(max_decode(frame(tc)), (tc, 0), tc)

    def test_fault_termokopel(self):
        for f in (1, 2, 4):
            self.assertEqual(max_decode(frame(65, fault=f)), (None, f))

    def test_frame_tidak_sah(self):
        self.assertEqual(max_decode(0x00000000), (None, 0x10))   # library Adafruit: 0.0 C
        self.assertEqual(max_decode(0xFFFFFFFF), (None, 0x10))
        self.assertEqual(max_decode(frame(65) | (1 << 17)), (None, 0x10))
        self.assertEqual(max_decode(frame(65) | (1 << 3)), (None, 0x10))
        self.assertEqual(max_decode(frame(65) | (1 << 16)), (None, 0x10))   # D16 tanpa jenis fault

    def test_cold_junction_tidak_wajar(self):
        self.assertEqual(max_decode(frame(65, cj=120.0)), (None, 0x40))
        self.assertEqual(max_decode(frame(65, cj=-40.0)), (None, 0x40))
        self.assertEqual(max_decode(frame(65, cj=25.5)), (65.0, 0))

    def test_baca_2x_berbeda_ditolak(self):
        a = frame(65); b = a ^ (1 << 25)                         # 1 bit rusak di salah satu bacaan
        self.assertEqual(sampel(a, b), (None, 0x20))
        self.assertEqual(sampel(a, a), (65.0, 0))

    def test_efektivitas_baca_2x(self):
        rnd = random.Random(3); asli = frame(65); n = 100000; lolos = 0
        for _ in range(n):
            def baca():
                v = asli
                if rnd.random() < 0.05:
                    for b in rnd.sample(range(32), rnd.choice((1, 2, 3))): v ^= 1 << b
                return v
            t, _ = sampel(baca(), baca())
            lolos += t is not None and abs(t - 65) > 2
        self.assertLess(lolos / n, 1e-4)                         # < 0,01 % nilai salah lolos


class UjiStatis(unittest.TestCase):
    def test_baca_dua_kali_dan_tanpa_library_read(self):
        kode = re.sub(r"//[^\n]*", "", SRC)
        self.assertIn("uint32_t ra = maxReadRaw(), rb = maxReadRaw();", kode)
        self.assertNotRegex(kode, r"thermocouple\.readCelsius\(")
        self.assertNotRegex(kode, r"thermocouple\.readError\(")

    def test_porting_sama_dengan_firmware(self):
        self.assertIn("(v & 0x00020008UL)", SRC)
        self.assertIn("if (cjC < -20.0f || cjC > 85.0f)", SRC)
        self.assertIn("t * 0.25f", SRC)


if __name__ == "__main__":
    unittest.main(verbosity=2)
