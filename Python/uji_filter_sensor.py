"""Uji filter sensor suhu firmware rev 26 (cek kewajaran + Cascaded Filtering 3 tahap) TANPA hardware:
    python uji_filter_sensor.py

- Konstanta (FILT_MED_N, FILT_MA_N, FILT_EMA_ALPHA, SENSOR_HOLD_MS, ...) dibaca dari dep_frayer.ino.
- Filter = porting baris-per-baris struct SensorFilter + sensorFilterStep() dari firmware
  (tidak ada compiler C++ host). Bila firmware diubah, porting ini harus ikut diubah.
"""
import math
import os
import re
import unittest

import numpy as np

INO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dep_frayer", "dep_frayer.ino")
SRC = open(INO, encoding="utf-8").read()


def c(nama):
    return float(re.search(r"#define\s+%s\s+([\d.]+)" % nama, SRC).group(1))


K = {"med": int(c("FILT_MED_N")), "ma": int(c("FILT_MA_N")), "alpha": c("FILT_EMA_ALPHA"),
     "hold": c("SENSOR_HOLD_MS"), "fault": c("SENSOR_FAULT_MS"), "sample": c("SENSOR_SAMPLE_MS"),
     "rate": c("SENSOR_MAX_RATE_C_S"), "margin": c("SENSOR_JUMP_MARGIN_C"),
     "rmin": -float(re.search(r"#define\s+SENSOR_RANGE_MIN_C\s+-([\d.]+)", SRC).group(1)),
     "rmax": c("SENSOR_RANGE_MAX_C")}


class Filter:
    """Porting SensorFilter + sensorFilterStep() (SENSOR_FILTER_ENABLE 1)."""

    def __init__(self):
        self.med, self.ma = [0.0] * K["med"], [0.0] * K["ma"]
        self.mn = self.mh = self.an = self.ah = 0
        self.ema = math.nan
        self.have = False
        self.last_out = math.nan
        self.last_raw = math.nan
        self.ref = False
        self.last_good_ms = 0
        self.err = 0

    def reset(self):
        self.mn = self.mh = self.an = self.ah = 0
        self.ema = math.nan
        self.have = False

    def median(self, v):
        self.med[self.mh] = v
        self.mh = (self.mh + 1) % K["med"]
        self.mn = min(self.mn + 1, K["med"])
        t = sorted(self.med[:self.mn])
        return t[self.mn // 2] if self.mn % 2 else 0.5 * (t[self.mn // 2 - 1] + t[self.mn // 2])

    def movavg(self, v):
        self.ma[self.ah] = v
        self.ah = (self.ah + 1) % K["ma"]
        self.an = min(self.an + 1, K["ma"])
        return sum(self.ma[:self.an]) / self.an

    def ema_step(self, v):
        self.ema = v if math.isnan(self.ema) else self.ema + K["alpha"] * (v - self.ema)
        return self.ema

    def step(self, tv, now):
        valid = not math.isnan(tv)
        if valid and (tv < K["rmin"] or tv > K["rmax"]):     # rev 26: rentang
            valid = False
        if valid and self.ref:                               # rev 26: cek kewajaran
            allow = K["margin"] + K["rate"] * (now - self.last_good_ms) / 1000.0
            if abs(tv - self.last_raw) > allow:
                valid = False
        if not valid:
            self.err += 1
        if valid:
            self.have, self.ref, self.last_good_ms, self.last_raw = True, True, now, tv
            self.last_out = self.ema_step(self.movavg(self.median(tv)))
            return self.last_out, False, True
        if self.have and (now - self.last_good_ms) <= K["hold"]:
            return self.last_out, True, False
        self.reset()
        return math.nan, False, False


def jalankan(seri):
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


class UjiStatis(unittest.TestCase):
    def test_sg_sudah_dihapus_dan_tahap_lengkap(self):
        self.assertNotIn("SG_COEF", SRC)
        self.assertNotIn("SENSOR_MAX_JUMP_C", SRC.split("REV 15 (dari rev 14)")[1].split("*/")[1])
        for f in ("stage1Median", "stage2MovAvg", "stage3Ema"):
            self.assertIn(f, SRC)
        self.assertIn("sf.stage3Ema(sf.stage2MovAvg(sf.stage1Median(tv)))", SRC)   # urutan tahap


class UjiSifatFilter(unittest.TestCase):
    def test_sinyal_konstan_tidak_berubah(self):
        out, _, _, _ = jalankan([150.0] * 50)
        np.testing.assert_allclose(out, 150.0)

    def test_lonjakan_1_dan_2_sampel_dibuang(self):
        for n in (1, 2):
            s = np.full(60, 100.0); s[30:30 + n] = 140.0
            out, _, _, _ = jalankan(s)
            self.assertLess(np.max(np.abs(out - 100.0)), 1e-9, n)

    def test_jeda_rampa_sekitar_1_3_detik(self):
        s = 30 + 0.5 * np.arange(400) * K["sample"] / 1000
        out, _, _, _ = jalankan(s)
        jeda = (s[300] - out[300]) / (0.5 * K["sample"] / 1000)
        self.assertAlmostEqual(jeda, 2 + 2 + (1 - K["alpha"]) / K["alpha"], places=6)   # median 2 + MA 2 + EMA
        self.assertAlmostEqual(jeda * K["sample"] / 1000, 1.27, places=2)

    def test_noise_turun(self):
        rng = np.random.default_rng(0)
        out, _, _, _ = jalankan(rng.normal(0, 0.3, 5000))
        self.assertLess(np.std(out[50:]) / 0.3, 0.40)

    def test_loncatan_tanpa_overshoot(self):
        s = np.r_[np.full(20, 100.0), np.full(80, 110.0)]
        out, _, _, _ = jalankan(s)
        self.assertLessEqual(out.max(), 110.0 + 1e-9)
        self.assertTrue(np.all(np.diff(out) >= -1e-12))

    def test_awal_data_langsung_ada_nilai(self):
        out, _, _, _ = jalankan([150.0, 151.0, 152.0])
        self.assertFalse(np.isnan(out).any())
        self.assertEqual(out[0], 150.0)


class UjiHoldFault(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(1)
        self.s = 30 + 0.5 * np.arange(300) * 0.2 + rng.normal(0, 0.15, 300)

    def test_hold_3_detik_lalu_nan_lalu_pulih(self):
        s = self.s.copy(); s[100:125] = np.nan                  # fault 5 s
        out, held, lv, f = jalankan(s)
        self.assertTrue(held[100:115].all())                     # 15 sampel = 3000 ms
        self.assertTrue(np.all(out[100:115] == out[99]))         # filter dibekukan
        self.assertTrue(np.isnan(out[115:125]).all())
        self.assertFalse(np.isnan(out[125:]).any())
        self.assertEqual(out[125], s[125])                       # reset: mulai dari sampel pertama
        self.assertEqual(lv[124], 99 * 200)                      # gLastValidMs tidak maju saat hold
        self.assertEqual(f.err, 25)

    def test_pengaman_sensor_tidak_melonggar(self):
        s = self.s.copy(); s[100:140] = np.nan                  # fault 8 s
        out, _, lv, _ = jalankan(s)
        bad = [i for i in range(len(s)) if np.isnan(out[i]) and i * 200 - lv[i] > K["fault"]]
        self.assertEqual(bad[0], 125)                            # sama dgn perilaku tanpa filter


class UjiKewajaranDanPid(unittest.TestCase):
    """Skenario lapangan: setpoint 65 C, bacaan tiba-tiba 200 C. Filter + PID (porting) bersama."""

    def jalankan_pid(self, seri, sp=65.0):
        import uji_pid as up
        f, pid = Filter(), up.Pid()
        last_valid, log = 0, []
        for i, tv in enumerate(seri):
            now = int(i * K["sample"])
            out, held, acc = f.step(tv, now)
            if acc:
                last_valid = now
            if i == 0:
                pid.enable(out, now, held, sp, 20.0)
            elif i % 5 == 0:                                     # PID 1 Hz (sampel terkirim)
                pid.update(out, now, held, sp, now, last_valid)
            log.append((i, out, held, pid.on, pid.out))
        return pid, log

    def test_bacaan_200_sesaat_pid_tetap_hidup(self):
        s = np.full(300, 65.0); s[150:160] = 200.0                # 2 dtk bacaan salah
        pid, log = self.jalankan_pid(s)
        self.assertTrue(pid.on)                                    # PID TETAP HIDUP
        self.assertLess(max(r[1] for r in log), 66.0)              # 200 tidak pernah masuk gTemp
        self.assertTrue(all(r[2] for r in log[150:160]))           # ditahan (HELD)

    def test_bacaan_200_lama_pid_tetap_on_tahan_aman_sensor(self):
        # < ~27 dtk: lompatan 65 -> 200 C mustahil secara fisik -> SENSOR ERROR (kode 2)
        s = np.full(250, 65.0); s[150:] = 200.0                   # 20 dtk bacaan salah
        pid, log = self.jalankan_pid(s)
        self.assertTrue(pid.on)                                    # rev 27: PID tidak pernah mati otomatis
        self.assertEqual((pid.safe, pid.out), (2, 0.0))            # TAHAN AMAN karena SENSOR, katup TUTUP
        self.assertLess(max(r[1] for r in log if not np.isnan(r[1]) and r[0] < 165), 66.0)
        # > ~27 dtk: angka 200 secara fisik mungkin -> diterima -> suhu >= 180 (kode 3), tetap ON & TUTUP
        s = np.full(400, 65.0); s[150:] = 200.0
        pid, _ = self.jalankan_pid(s)
        self.assertTrue(pid.on)
        self.assertEqual((pid.safe, pid.out), (3, 0.0))

    def test_acuan_tidak_hilang_200_baru_diterima_setelah_27_detik(self):
        s = np.full(400, 65.0); s[150:] = 200.0
        out, held, _, _ = jalankan(s)
        pertama = next(i for i in range(150, 400) if not np.isnan(out[i]) and out[i] > 100)
        self.assertAlmostEqual((pertama - 149) * 0.2, (200 - 65 - K["margin"]) / K["rate"], delta=0.4)

    def test_sampah_saat_boot_tidak_jadi_acuan(self):
        s = np.r_[[900.0], np.full(50, 65.0)]
        out, _, _, f = jalankan(s)
        self.assertEqual(out[1], 65.0); self.assertEqual(f.err, 1)

    def test_kenaikan_nyata_bertahap_tetap_lolos(self):
        s = 65 + 0.5 * np.arange(1000) * 0.2                       # 0,5 C/dtk (pemanasan cepat)
        out, held, _, f = jalankan(s)
        self.assertFalse(held.any()); self.assertEqual(f.err, 0)

    def test_ubi_masuk_suhu_turun_cepat_tetap_lolos(self):
        s = np.r_[np.full(50, 160.0), 160 - 2.0 * np.arange(1, 26) * 0.2, np.full(50, 150.0)]   # turun 2 C/dtk
        out, held, _, f = jalankan(s)
        self.assertFalse(held.any()); self.assertEqual(f.err, 0)


if __name__ == "__main__":
    print("Konstanta dari firmware:", K)
    unittest.main(verbosity=2)
