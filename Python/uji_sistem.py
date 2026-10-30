"""Uji fitur Uji Sistem (rev 33) TANPA hardware:  python uji_sistem.py

1. Cek statis dep_frayer.ino: konstanta sesuai permintaan (rentang SP-3..SP+7, 10 dtk, turun 50%,
   naik 55%, timer default 120 dtk), urutan tahap, STOP membatalkan, gerak manual ditolak, gerak uji
   tanpa batas waktu, jumlah format snprintf JSON "temp" = jumlah argumen.
2. Simulasi garis waktu (porting ujUpdate + model mekanik Z/X + limit switch), langkah 10 ms.
3. Parser logger membaca kolom uji_tahap / uji_detik / uji_timer_s.
"""
import os
import re
import unittest

from parser_esp import Penggabung

INO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dep_frayer", "dep_frayer.ino")
SRC = open(INO, encoding="utf-8").read()


def d(nama):
    return float(re.search(r"#define\s+%s\s+(-?[\d.]+)" % nama, SRC).group(1))


def blok(sig):
    i = SRC.index(sig)
    j = SRC.index("{", i)
    n, k = 0, j
    while True:
        n += {"{": 1, "}": -1}.get(SRC[k], 0)
        if n == 0:
            return SRC[j:k + 1]
        k += 1


class UjiStatis(unittest.TestCase):
    def test_konstanta_permintaan(self):
        self.assertEqual(d("UJ_BAND_LO_C"), 3.0)
        self.assertEqual(d("UJ_BAND_HI_C"), 7.0)
        self.assertEqual(d("UJ_STABLE_MS"), 10000)
        self.assertEqual(d("UJ_PCT_DOWN"), 50)
        self.assertEqual(d("UJ_PCT_UP"), 55)
        self.assertEqual(d("UJ_FRY_DEFAULT_S"), 120)

    def test_stop_membatalkan_uji(self):
        b = re.search(r'strcmp\(cmd, "stop"\) == 0\) \{(.*?)\n  \}', SRC, re.S).group(1)
        self.assertIn("ujEnd(9", b)

    def test_gerak_manual_ditolak_saat_uji(self):
        b = blok("void handleCommand(const char* cmd)")
        i_tolak = b.index("if (ujRunning() && (")
        for pref in ('"jog,"', '"dcz,"', '"en,"', '"stt,"', '"dir,"'):
            self.assertIn(pref, b[i_tolak:i_tolak + 400])
        self.assertLess(i_tolak, b.index('sscanf(cmd, "jog,'))
        self.assertLess(i_tolak, b.index('sscanf(cmd, "dcz,'))

    def test_gerak_uji_tanpa_batas_waktu(self):
        self.assertIn("!dcNoTimeout && (now - dcRunStart) > DC_MAX_RUN_MS", SRC)
        self.assertIn("dcNoTimeout = false;", blok("void dcSetTarget(int pct)"))   # gerak manual tetap dibatasi
        self.assertIn("dcNoTimeout = true", blok("void ujMoveZ(int pct)"))

    def test_uj_dipanggil_di_loop(self):
        self.assertIn("ujUpdate();", blok("void loop()"))

    def test_format_json_temp(self):
        b = blok("void broadcastTemp()")
        for m in re.finditer(r'snprintf\(b, sizeof\(b\), "(.*?)",\s*\n(.*?)\);', b, re.S):
            fmt, args = m.group(1), m.group(2)
            n_fmt = len(re.findall(r"%(?!%)[-0-9.]*l?[dusfX]", fmt))
            dalam, n_arg, cur = 0, 1, ""
            for ch in args:
                dalam += ch in "(" and 1 or 0
                dalam -= ch in ")" and 1 or 0
                if ch == "," and dalam == 0:
                    n_arg += 1
            self.assertEqual(n_fmt, n_arg, fmt[-60:])
            self.assertIn('\\"uj\\":%d,\\"us\\":%ld,\\"uf\\":%lu', fmt)


class Fryer:
    """Model mekanik: Z 0 (bawah) .. 100 (atas) mm, X 0 (awal) .. 4000 step (akhir)."""

    def __init__(self, z=50.0, x=2000, temps=None, z_bounce=False):
        self.z, self.x = z, x
        self.temps = temps or (lambda t: 160.0)
        self.z_bounce = z_bounce

    def ls(self, i, now):
        hit = [self.z >= 100, self.z <= 0, self.x <= 0, self.x >= 4000][i]
        if self.z_bounce and i in (0, 1) and hit and (now // 10) % 3 == 1:
            return False                           # pantulan kontak
        return hit


class SimUji:
    """Porting ujStart/ujEnter/ujUpdate + dcSetTarget/dcUpdate (limit) + enforceSafety + stop."""

    def __init__(self, f, sp=160.0, fry_s=120):
        self.f, self.sp, self.fry_ms = f, sp, fry_s * 1000
        self.state, self.t0, self.inband = 0, 0, 0
        self.dc, self.dcLimit, self.xgo, self.xLimit = 0, False, 0, False
        self.now = 0
        self.log = []                               # (ms, tahap)
        self.z_hist = []

    def lsHit(self, i):
        return self.f.ls(i, self.now)

    def dcSetTarget(self, pct):
        if pct != 0 and self.lsHit(0 if pct > 0 else 1):
            return
        self.dc, self.dcLimit = pct, False

    def moveX(self, dr):
        self.xgo, self.xLimit = (10 ** 9 if dr > 0 else -10 ** 9), False

    def enter(self, st):
        self.state, self.t0 = st, self.now
        self.log.append((self.now, st))
        if st == 1 and not self.lsHit(0):
            self.dcSetTarget(55)
        elif st == 2 and not self.lsHit(2):
            self.moveX(-1)
        elif st == 3:
            self.inband = 0
        elif st == 4:
            self.dcSetTarget(-50)
        elif st == 6:
            self.dcSetTarget(55)
        elif st == 7:
            self.moveX(1)

    def end(self, st):
        self.dc, self.xgo, self.state = 0, 0, st
        self.log.append((self.now, st))

    def start(self):
        self.enter(1)

    def stop(self):
        self.dc, self.xgo = 0, 0
        if 1 <= self.state <= 7:
            self.end(9)

    def update(self):
        st, now = self.state, self.now
        if not 1 <= st <= 7:
            return
        if st in (1, 6):
            if self.lsHit(0) or self.dcLimit:
                self.dc = 0
                self.enter(2 if st == 1 else 7)
            elif self.dc <= 0:
                self.end(9)
        elif st == 4:
            if self.lsHit(1) or self.dcLimit:
                self.dc = 0
                self.enter(5)
            elif self.dc >= 0:
                self.end(9)
        elif st in (2, 7):
            li = 2 if st == 2 else 3
            if self.lsHit(li) or self.xLimit:
                self.xgo = 0
                self.enter(3) if st == 2 else self.end(8)
            elif self.xgo == 0:
                self.end(9)
        elif st == 3:
            t = self.f.temps(now / 1000.0)
            inn = t is not None and self.sp - 3 <= t <= self.sp + 7
            if not inn:
                self.inband = 0
            elif not self.inband:
                self.inband = now or 1
            elif now - self.inband >= 10000:
                self.enter(4)
        elif st == 5 and now - self.t0 >= self.fry_ms:
            self.enter(6)

    def run(self, durasi_ms, stop_ms=None):
        self.start()                                   # tombol MULAI UJI pada t = 0
        while self.now < durasi_ms:
            self.update()                              # loop(): enforceSafety, stepX.run, ujUpdate, dcUpdate
            if self.xgo and self.lsHit(3 if self.xgo > 0 else 2):
                self.xgo, self.xLimit = 0, True        # enforceSafety
            if self.dc and self.lsHit(0 if self.dc > 0 else 1):
                self.dc, self.dcLimit = 0, True        # dcUpdate
            # mekanik: Z 100 mm dalam 4 dtk pada 50%; X 2 step/ms
            self.f.z = min(100.0, max(0.0, self.f.z + self.dc / 50.0 * 0.25))
            if self.xgo:
                self.f.x = min(4000, max(0, self.f.x + (2 if self.xgo > 0 else -2)))
            self.z_hist.append((self.now, self.f.z))
            if stop_ms is not None and self.now == stop_ms:
                self.stop()
            self.now += 10
        return [s for _, s in self.log]


class UjiGarisWaktu(unittest.TestCase):
    def test_urutan_lengkap(self):
        sim = SimUji(Fryer(z=50, x=2000))
        tahap = sim.run(300_000)
        self.assertEqual(tahap, [1, 2, 3, 4, 5, 6, 7, 8])
        self.assertEqual(sim.f.x, 4000)                       # tetap di X Akhir
        self.assertEqual(sim.f.z, 100)                        # Z di atas

    def test_timer_dimulai_saat_z_bawah_kena(self):
        sim = SimUji(Fryer(), fry_s=120)
        sim.run(300_000)
        t = dict((s, ms) for ms, s in sim.log)
        z_bawah_ms = next(ms for ms, z in sim.z_hist if z <= 0)
        self.assertAlmostEqual(t[5], z_bawah_ms, delta=20)
        self.assertEqual(t[6] - t[5], 120_000)               # tepat 2 menit di bawah
        dibawah = [ms for ms, z in sim.z_hist if z <= 0]
        self.assertGreaterEqual(max(dibawah) - min(dibawah), 119_900)

    def test_sudah_di_posisi_awal_langsung_tunggu_suhu(self):
        sim = SimUji(Fryer(z=100, x=0))
        sim.run(30_000)
        self.assertEqual([s for _, s in sim.log][:3], [1, 2, 3])
        self.assertLessEqual(sim.log[2][0], 20)           # tanpa gerak: langsung tahap 3

    def test_tunggu_10_detik_berturut_turut(self):
        # suhu: 150 (di luar) s/d 20 dtk, keluar sebentar di 25 dtk, lalu 158-166 berulang
        def temps(t):
            if t < 20 or 25 <= t < 26:
                return 150.0
            return 162.0 + 4.0 * ((int(t) % 2) * 2 - 1)
        sim = SimUji(Fryer(z=100, x=0, temps=temps))
        sim.run(60_000)
        t4 = dict((s, ms) for ms, s in sim.log)[4]
        self.assertEqual(t4, 36_000)                          # 26 dtk + 10 dtk berturut-turut

    def test_suhu_di_luar_rentang_tidak_turun(self):
        for suhu in (156.9, 167.1, None):
            sim = SimUji(Fryer(z=100, x=0, temps=lambda t, s=suhu: s))
            self.assertNotIn(4, sim.run(60_000), suhu)

    def test_batas_rentang_inklusif(self):
        for suhu in (157.0, 167.0):
            sim = SimUji(Fryer(z=100, x=0, temps=lambda t, s=suhu: s))
            self.assertIn(4, sim.run(60_000), suhu)

    def test_stop_membatalkan_saat_menggoreng(self):
        sim = SimUji(Fryer())
        sim.run(100_000, stop_ms=60_000)
        self.assertEqual(sim.log[-1][1], 9)
        self.assertEqual(sim.dc, 0)

    def test_pantulan_kontak_limit_tidak_membatalkan(self):
        sim = SimUji(Fryer(z_bounce=True))
        self.assertEqual(sim.run(300_000), [1, 2, 3, 4, 5, 6, 7, 8])


class UjiParser(unittest.TestCase):
    def test_kolom_uji(self):
        p = Penggabung()
        r = p.baris_dari_temp({"type": "temp", "t": 160.2, "ok": True, "sp": 160, "uj": 5, "us": 87,
                               "uf": 120}, 0, "x", 0)
        self.assertEqual((r["uji_tahap"], r["uji_detik"], r["uji_timer_s"]), (5, 87, 120))

    def test_firmware_lama_kosong(self):
        r = Penggabung().baris_dari_temp({"type": "temp", "t": 30.0, "ok": True}, 0, "x", 0)
        self.assertIsNone(r["uji_tahap"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
