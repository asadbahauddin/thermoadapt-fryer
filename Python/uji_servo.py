"""Uji jalur perintah servo firmware rev 20 TANPA hardware:  python uji_servo.py

Servo = porting baris-per-baris setServoAngle(), setServoAngleManual(), setServoPct(),
servoUsFromDeg(), dan cabang perintah sv*/svk/svset di handleCommand() dari dep_frayer.ino
(tidak ada compiler C++ host). Konstanta dibaca dari .ino. Ditambah cek statis bahwa SEMUA
perintah manual memakai setServoAngleManual (penyebab bug "mentok di 0 derajat").
"""
import os
import re
import unittest

INO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dep_frayer", "dep_frayer.ino")
SRC = open(INO, encoding="utf-8").read()


def c(nama):
    return int(re.search(r"#define\s+%s\s+(-?\d+)" % nama, SRC).group(1))


US_MIN_ABS, US_MAX_ABS = c("SERVO_US_ABS_MIN"), c("SERVO_US_ABS_MAX")
CAL_MIN, SPAN_MIN = c("SERVO_CAL_MIN_DEG"), c("SERVO_MIN_SPAN_DEG")


class Servo:
    def __init__(self, cd=0, od=55, us_min=500, us_max=2500):
        self.cd, self.od, self.us_min, self.us_max = cd, od, us_min, us_max
        self.target = float(cd)
        self.lock = 0
        self.pid_on = False
        self.step_on = False
        self.pesan = []

    def us(self, deg):                                   # servoUsFromDeg
        return self.us_min + (self.us_max - self.us_min) * deg / 180.0

    def set_angle(self, deg):                            # setServoAngle (PID/step test)
        deg = min(max(deg, self.cd), self.od)
        if self.lock and abs(deg - self.cd) > 0.5:
            self.pesan.append("terkunci"); return
        self.target = deg

    def set_manual(self, deg):                           # setServoAngleManual
        if deg < CAL_MIN or deg > 180:
            self.pesan.append("Sudut harus %d..180" % CAL_MIN); return
        u = self.us(deg)
        if u < US_MIN_ABS or u > US_MAX_ABS:
            self.pesan.append("Ditolak: pulsa %.0f us" % u); return
        if self.lock and abs(deg - self.cd) > 0.5:
            self.pesan.append("terkunci"); return
        self.target = float(deg)

    def set_pct(self, pct):                              # setServoPct
        pct = min(max(pct, 0), 100)
        self.set_angle(self.cd + (self.od - self.cd) * pct / 100.0)

    def cmd(self, s):                                    # cabang servo handleCommand
        if self.step_on and s.startswith("sv"):
            self.pesan.append("Step test berjalan"); return
        if self.pid_on and s.startswith("sv"):
            self.pesan.append("PID aktif"); return
        m = re.fullmatch(r"svk,(-?\d+)", s)
        if m:
            self.set_manual(float(m.group(1))); return
        m = re.fullmatch(r"svset,([co])", s)
        if m:
            deg = int(round(self.target)) if self.target >= 0 else -int(round(-self.target))  # lroundf
            if m.group(1) == "c" and deg > self.od - SPAN_MIN:
                self.pesan.append("Ditolak TUTUP"); return
            if m.group(1) == "o" and deg < self.cd + SPAN_MIN:
                self.pesan.append("Ditolak BUKA"); return
            if m.group(1) == "c":
                self.cd = deg
            else:
                self.od = deg
            return
        m = re.fullmatch(r"svad,(-?\d+)", s)
        if m:
            t = int(round(self.target)) if self.target >= 0 else -int(round(-self.target))
            self.set_manual(t + int(m.group(1))); return
        m = re.fullmatch(r"sva,(-?\d+)", s)
        if m:
            self.set_manual(float(m.group(1))); return
        m = re.fullmatch(r"sv,(-?\d+)", s)
        if m:
            self.set_pct(int(m.group(1))); return
        self.pesan.append("tidak dikenal")


class UjiStatis(unittest.TestCase):
    def test_semua_perintah_manual_pakai_fungsi_manual(self):
        for pola in (r'sscanf\(cmd, "svk,%d", &v\) == 1\) \{ setServoAngleManual',
                     r'sscanf\(cmd, "svad,%d", &v\) == 1\) \{ setServoAngleManual',
                     r'sscanf\(cmd, "sva,%d", &v\) == 1\)\s+\{ setServoAngleManual'):
            self.assertRegex(SRC, pola)

    def test_pid_dan_step_test_tetap_dikunci(self):
        self.assertRegex(SRC, r"void pidUpdate\(\)[\s\S]*?setServoAngle\(\(float\)svClosedDeg \+")
        self.assertRegex(SRC, r"void stepTestStart[\s\S]*?setServoAngle\(angle\)")
        self.assertNotRegex(SRC, r"void pidUpdate\(\)[^}]*setServoAngleManual")

    def test_halaman_tanpa_cache(self):
        self.assertIn('sendHeader("Cache-Control", "no-store', SRC)


class UjiPerintah(unittest.TestCase):
    def test_bug_lama_minus1_di_0_sekarang_jalan(self):
        s = Servo()
        s.cmd("svad,-1")
        self.assertEqual(s.target, -1)
        self.assertAlmostEqual(s.us(s.target), 488.89, places=2)
        s.cmd("sva,-5"); self.assertEqual(s.target, -5)
        s.cmd("svk,-2"); self.assertEqual(s.target, -2)

    def test_batas_pulsa(self):
        s = Servo()
        for _ in range(30):
            s.cmd("svad,-1")
        self.assertEqual(s.target, -18)                  # 300 us
        self.assertTrue(any("Ditolak: pulsa" in p for p in s.pesan))
        s.cmd("svk,-31"); self.assertIn("Sudut harus -30..180", s.pesan[-1])

    def test_simpan_tutup_negatif_dan_pid_ikut(self):
        s = Servo()
        s.cmd("svk,-3"); s.cmd("svset,c")
        self.assertEqual(s.cd, -3)                       # dulu (int)(x+0.5) -> -2
        s.cmd("sv,0"); self.assertEqual(s.target, -3)   # TUTUP / PID 0% = -3
        s.cmd("sv,100"); self.assertEqual(s.target, 55)
        s.set_angle(-10); self.assertEqual(s.target, -3)  # PID/step test tidak bisa lewat TUTUP

    def test_ditolak_saat_pid_atau_step_test(self):
        s = Servo(); s.pid_on = True
        s.cmd("svad,-1"); self.assertEqual(s.target, 0); self.assertEqual(s.pesan[-1], "PID aktif")
        s = Servo(); s.step_on = True
        s.cmd("svk,-1"); self.assertEqual(s.target, 0)

    def test_pulsa_0_derajat_diturunkan(self):
        s = Servo(us_min=490)
        s.cmd("svad,-1"); self.assertAlmostEqual(s.us(s.target), 490 - 2010 / 180, places=6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
