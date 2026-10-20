"""Uji filter sensor suhu firmware rev 15 TANPA hardware:  python uji_filter_sensor.py

- Koefisien SG dibaca LANGSUNG dari dep_frayer.ino (bukan disalin), dibandingkan dengan scipy.
- sensor_filter_step() adalah porting baris-per-baris dari sensorFilterStep() di firmware
  (tidak ada compiler C++ host di laptop ini). Bila firmware diubah, porting ini harus ikut.
"""
import math
import os
import re
import unittest

import numpy as np

INO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dep_frayer", "dep_frayer.ino")


def baca_firmware():
    src = open(INO, encoding="utf-8").read()
    m = re.search(r"SG_COEF\[SG_WINDOW\]\s*=\s*\{(.*?)\};", src, re.S)
    koef = []
    for x in m.group(1).split(","):
        x = x.strip().replace("f", "")
        if x:
            a, b = (float(v) for v in x.split("/"))       # bentuk "18.0f / 143"
            koef.append(a / b)
    angka = lambda nama: float(re.search(r"#define\s+%s\s+([\d.]+)" % nama, src).group(1))
    return np.array(koef, float), {
        "jump": angka("SENSOR_MAX_JUMP_C"), "hold": angka("SENSOR_HOLD_MS"),
        "fault": angka("SENSOR_FAULT_MS"), "sample": angka("SENSOR_SAMPLE_MS"),
        "win": int(angka("SG_WINDOW"))}


KOEF, K = baca_firmware()


class Filter:
    """Porting sensorFilterStep() (SENSOR_FILTER_ENABLE 1)."""

    def __init__(self):
        self.win = [0.0] * K["win"]
        self.n = self.head = 0
        self.have_ref = False
        self.last_good = self.last_out = math.nan
        self.last_good_ms = 0
        self.err = 0

    def push(self, v):
        self.win[self.head] = v
        self.head = (self.head + 1) % K["win"]
        self.n = min(self.n + 1, K["win"])

    def sg(self):
        return sum(KOEF[k] * self.win[(self.head + k) % K["win"]] for k in range(K["win"]))

    def step(self, tv, now):
        valid = not math.isnan(tv)
        if valid and self.have_ref and abs(tv - self.last_good) > K["jump"]:
            valid = False
        if not valid:
            self.err += 1
        if valid:
            self.have_ref, self.last_good, self.last_good_ms = True, tv, now
            self.push(tv)
            self.last_out = tv if self.n < K["win"] else self.sg()
            return self.last_out, False, True
        if self.have_ref and (now - self.last_good_ms) <= K["hold"]:
            self.push(self.last_out)
            return self.last_out, True, False
        self.n = self.head = 0
        self.have_ref = False
        return math.nan, False, False


def jalankan(seri):
    """seri: list suhu mentah per 200 ms (nan = fault). -> out, held, last_valid_ms tiap sampel."""
    f = Filter()
    out, held, lv = [], [], []
    last_valid = 0
    for i, tv in enumerate(seri):
        now = int(i * K["sample"])
        o, h, acc = f.step(tv, now)
        if acc:
            last_valid = now
        out.append(o); held.append(h); lv.append(last_valid)
    return np.array(out), np.array(held), np.array(lv), f


class UjiKoefisien(unittest.TestCase):
    def test_sama_dengan_scipy_dan_jumlah_1(self):
        from scipy.signal import savgol_coeffs
        ref = savgol_coeffs(11, 2, pos=10, use="dot")
        np.testing.assert_allclose(KOEF, ref, atol=1e-9)
        self.assertAlmostEqual(KOEF.sum(), 1.0, places=12)
        self.assertAlmostEqual(float(np.float32(KOEF).sum(dtype=np.float32)), 1.0, places=6)

    def test_rampa_tanpa_jeda(self):
        n = np.arange(11, dtype=float)
        self.assertAlmostEqual(KOEF @ (2.0 * n + 30), 2.0 * 10 + 30, places=9)   # nilai titik terakhir
        self.assertAlmostEqual(KOEF @ (n ** 2), 100.0, places=9)

    def test_sifat_yang_dilaporkan(self):
        self.assertAlmostEqual(KOEF[-1], 83 / 143)                       # 58% lonjakan lolos
        self.assertAlmostEqual(math.sqrt((KOEF ** 2).sum()), 0.762, places=3)
        step = np.r_[np.zeros(10), np.ones(30)]
        y = [KOEF @ step[i - 10:i + 1] for i in range(10, 40)]
        self.assertAlmostEqual(max(y), 1.245, places=3)                  # overshoot ~25%
        self.assertAlmostEqual(min(y[5:]), 0.860, places=3)              # undershoot ~14%


class UjiLogikaFilter(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(1)
        t = np.arange(300) * 0.2
        self.bersih = 30 + 0.5 * t                                        # rampa 0.5 C/s
        self.mentah = np.round((self.bersih + rng.normal(0, 0.15, t.size)) / 0.25) * 0.25

    def test_hold_lalu_nan_setelah_3_detik(self):
        s = self.mentah.copy()
        s[100:125] = np.nan                                               # fault 5 s
        out, held, lv, _ = jalankan(s)
        self.assertTrue(held[100:115].all())                              # 3000 ms = 15 sampel ditahan
        self.assertTrue(np.all(out[100:115] == out[99]))                  # nilai = output terakhir
        self.assertTrue(np.isnan(out[115:125]).all())                     # lewat hold -> NAN
        self.assertFalse(np.isnan(out[125:]).any())                       # pulih lagi
        self.assertEqual(lv[124], 99 * 200)                               # gLastValidMs tidak maju saat hold
        # pengaman servo: sensorBad = isnan(gTemp) && now - gLastValidMs > 5000
        bad = [i for i in range(len(s)) if np.isnan(out[i]) and i * 200 - lv[i] > K["fault"]]
        self.assertEqual(bad, [])        # fault 5 s pas: belum lewat 5000 ms, sama seperti perilaku lama
        s[100:140] = np.nan                                               # fault 8 s
        out, held, lv, _ = jalankan(s)
        bad = [i for i in range(len(s)) if np.isnan(out[i]) and i * 200 - lv[i] > K["fault"]]
        self.assertEqual(bad[0], 125)    # 99*200 + 5000 < 125*200: pengaman aktif di waktu yang sama dgn lama

    def test_lonjakan_ditolak(self):
        s = self.mentah.copy()
        s[150] += 40; s[200] -= 30                                        # lonjakan tunggal
        out, held, _, f = jalankan(s)
        self.assertTrue(held[150] and held[200])
        self.assertEqual(f.err, 2)
        ref, _, _, _ = jalankan(self.mentah)
        self.assertLess(np.nanmax(np.abs(out[150:170] - ref[150:170])), 1.5)

    def test_acuan_boot_salah_tidak_mengunci(self):
        s = self.mentah.copy()
        s[0] = 900.0                                                      # sampel pertama sampah
        out, _, _, _ = jalankan(s)
        self.assertTrue(np.isnan(out[16:17]).all())                       # 15 sampel ditolak (hold) -> NAN
        self.assertLess(abs(out[60] - self.bersih[60]), 1.0)              # lalu acuan baru, normal

    def test_rampa_tanpa_jeda_dibanding_mentah(self):
        out, _, _, _ = jalankan(self.mentah)
        err = out[20:] - self.bersih[20:]
        self.assertLess(abs(err.mean()), 0.05)                            # tidak tertinggal di rampa
        self.assertLess(err.std(), np.std(self.mentah[20:] - self.bersih[20:]))   # noise turun

    def test_awal_jendela_mentah(self):
        out, _, _, _ = jalankan(self.mentah)
        np.testing.assert_array_equal(out[:10], self.mentah[:10])


if __name__ == "__main__":
    print("Konstanta dari firmware:", K)
    unittest.main(verbosity=2)
