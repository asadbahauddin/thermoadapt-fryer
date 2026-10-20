"""Uji logika (Adaptive) PID firmware rev 17 TANPA hardware:  python uji_pid.py

Pid = porting baris-per-baris pidAdaptGains()/pidEnable()/pidUpdate()/pidWatchdog() dari dep_frayer.ino
(tidak ada compiler C++ host). Konstanta dibaca dari .ino. Plant = FOPDT DUGAAN
(K, tau, L di bawah), BUKAN model alat: hasil overshoot hanya ilustrasi logika, bukan tuning.
"""
import math
import os
import re
import unittest

INO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dep_frayer", "dep_frayer.ino")
SRC = open(INO, encoding="utf-8").read()


def c(nama):
    return float(re.search(r"#define\s+%s\s+([-\d.]+)f?" % nama, SRC).group(1))


KP, KI, KD, UMAX = c("PID_DEFAULT_KP"), c("PID_DEFAULT_KI"), c("PID_DEFAULT_KD"), c("PID_DEFAULT_UMAX")
RAMP, OVER, DF = c("PID_SP_RAMP_C_S"), c("PID_OVER_C"), c("PID_D_FILTER")
CUTOFF, FAULT_MS = c("TEMP_CUTOFF_C"), c("SENSOR_FAULT_MS")
SP_MAX = CUTOFF - 15
LP, LI, LD = c("PID_DEFAULT_LP"), c("PID_DEFAULT_LI"), c("PID_DEFAULT_LD")
KP_MAX, KI_MAX, KD_MAX = c("PID_KP_MAX"), c("PID_KI_MAX"), c("PID_KD_MAX")


class Pid:
    def __init__(self, kp=KP, ki=KI, kd=KD, umax=UMAX, adapt=True, lp=LP, li=LI, ld=LD):
        self.kp, self.ki, self.kd, self.umax = kp, ki, kd, umax
        self.adapt, self.lp, self.li, self.ld = adapt, lp, li, ld
        self.kpe, self.kie, self.kde = kp, ki, kd
        self.on = False
        self.I = self.out = self.sp_eff = self.prev_t = self.df = 0.0
        self.prev_valid = False
        self.last_ms = 0
        self.alasan_mati = None

    def gains(self, e, dTdt):
        if not self.adapt:
            self.kpe, self.kie, self.kde = self.kp, self.ki, self.kd
            return
        self.kpe = min(self.kp + self.lp * abs(e), max(self.kp, KP_MAX))
        self.kie = min(self.ki + self.li * abs(e), max(self.ki, KI_MAX))
        self.kde = min(self.kd + self.ld * abs(dTdt), max(self.kd, KD_MAX))

    def disable(self, why):
        self.on, self.out, self.I = False, 0.0, 0.0
        self.alasan_mati = why

    def enable(self, t, ms, held, setpoint, opening_pct):
        if math.isnan(t) or held or setpoint > SP_MAX or t >= setpoint + OVER or t >= CUTOFF - 5:
            return False
        self.sp_eff, self.prev_t, self.df, self.prev_valid = t, t, 0.0, True
        self.gains(0.0, 0.0)
        self.I = min(max(opening_pct, 0.0), self.umax)
        self.out = self.I
        self.last_ms = ms
        self.on = True
        return True

    def update(self, t, ms, held, setpoint, now_ms, last_valid_ms):
        if not self.on:
            return
        if math.isnan(t):
            self.prev_valid = False
            if now_ms - last_valid_ms > FAULT_MS:
                self.disable("sensor")
            return
        if t >= setpoint + OVER or t >= CUTOFF - 5:
            self.disable("suhu"); return
        if setpoint > SP_MAX:
            self.disable("setpoint"); return
        dt = (ms - self.last_ms) / 1000.0
        self.last_ms = ms
        if dt < 0.05 or dt > 5.0:
            dt = 1.0
        stp = RAMP * dt
        self.sp_eff = setpoint if abs(setpoint - self.sp_eff) <= stp else self.sp_eff + (stp if setpoint > self.sp_eff else -stp)
        if held:
            return
        e = self.sp_eff - t
        if not self.prev_valid:
            self.prev_t, self.df, self.prev_valid = t, 0.0, True
        d_raw = (t - self.prev_t) / dt
        self.prev_t = t
        self.df = DF * self.df + (1 - DF) * d_raw
        self.gains(e, self.df)
        u_unsat = self.kpe * e + self.I - self.kde * self.df
        u = min(max(u_unsat, 0.0), self.umax)
        windup = (u_unsat > self.umax and e > 0) or (u_unsat < 0 and e < 0)
        if not windup:
            self.I = min(max(self.I + self.kie * e * dt, 0.0), self.umax)
        self.out = u

    def watchdog(self, now_ms, pub_ms):
        if self.on and now_ms - pub_ms > FAULT_MS:
            self.disable("watchdog")


def simulasi(detik=3600, sp=150.0, K=4.0, tau=600.0, L=20.0, T0=30.0, ganggu=None, pid=None, beban=None):
    """Loop tertutup 1 Hz dengan plant FOPDT dugaan. ganggu(i, T) -> (t_ukur, held) opsional.
    Catatan: suhu maks yang bisa dicapai = T0 + K*Umax. Dengan Umax 40%, setpoint 150 C hanya
    tercapai bila K >= (150-30)/40 = 3 C per % bukaan (K=2.2 -> maks 118 C)."""
    pid = pid or Pid()
    T, hist, rows = T0, [0.0] * int(L), []
    pid.enable(T, 0, False, sp, 0.0)
    last_valid = 0
    for i in range(1, detik):
        u_tunda = hist[-int(L)]
        T += (K * u_tunda - (T - T0)) / tau
        if beban is not None:
            T += beban(i)                                    # gangguan beban (ubi masuk)
        t_ukur, held = (T, False) if ganggu is None else ganggu(i, T)
        if not math.isnan(t_ukur) and not held:
            last_valid = i * 1000
        pid.update(t_ukur, i * 1000, held, sp, i * 1000, last_valid)
        hist.append(pid.out if pid.on else 0.0)
        rows.append((i, T, pid.out, pid.I, pid.sp_eff, pid.on, pid.kpe, pid.kie, pid.kde))
    return pid, rows


class UjiPid(unittest.TestCase):
    def test_bumpless_start(self):
        p = Pid()
        self.assertTrue(p.enable(100.0, 0, False, 150.0, 25.0))
        p.update(100.0, 1000, False, 150.0, 1000, 1000)
        # e = sp_eff - T = 0.15 -> keluaran hampir sama dengan bukaan saat start (25%)
        self.assertAlmostEqual(p.out, 25.0 + (KP + LP * RAMP) * RAMP, places=6)   # Kp efektif (adaptif)

    def test_enable_ditolak(self):
        p = Pid()
        self.assertFalse(p.enable(float("nan"), 0, False, 150, 0))
        self.assertFalse(p.enable(100, 0, True, 150, 0))           # HELD
        self.assertFalse(p.enable(100, 0, False, SP_MAX + 1, 0))   # setpoint > 165
        self.assertFalse(p.enable(170, 0, False, 150, 0))          # sudah >= sp + 15

    def test_antiwindup_integral_tidak_lewat_umax(self):
        pid, rows = simulasi(detik=1200)
        self.assertTrue(all(0.0 <= r[3] <= UMAX + 1e-9 for r in rows))
        self.assertTrue(all(0.0 <= r[2] <= UMAX + 1e-9 for r in rows))

    def test_loop_tertutup_mencapai_setpoint(self):
        pid, rows = simulasi(detik=7200, sp=150.0)
        T_akhir = rows[-1][1]
        maks = max(r[1] for r in rows)
        self.assertTrue(pid.on)
        self.assertLess(abs(T_akhir - 150.0), 1.0)
        print("\n  [ilustrasi plant dugaan] suhu akhir %.2f C, maks %.2f C (overshoot %.2f C)"
              % (T_akhir, maks, maks - 150.0))

    def test_umax_terlalu_kecil_terdeteksi(self):
        pid, rows = simulasi(detik=7200, sp=150.0, K=2.2)          # maks 30 + 2.2*40 = 118 C
        self.assertLess(rows[-1][1], 119.0)
        self.assertAlmostEqual(rows[-1][2], UMAX)                  # keluaran mentok di Umax

    def test_held_membekukan_integral(self):
        def g(i, T):
            return (T, 400 <= i < 403)                       # 3 sampel ditahan
        pid, rows = simulasi(detik=600, ganggu=g)
        I = {r[0]: r[3] for r in rows}
        out = {r[0]: r[2] for r in rows}
        self.assertEqual(I[400], I[399]); self.assertEqual(I[402], I[399])
        self.assertEqual(out[402], out[399])

    def test_sensor_nan_lama_mematikan_pid(self):
        def g(i, T):
            return (float("nan"), False) if i >= 300 else (T, False)
        pid, rows = simulasi(detik=320, ganggu=g)
        mati = next(r[0] for r in rows if not r[5])
        self.assertFalse(pid.on)
        self.assertEqual(pid.alasan_mati, "sensor")
        self.assertEqual(mati, 305)                          # 299 s valid terakhir + >5 s

    def test_overtemp_mematikan_pid(self):
        p = Pid(); p.enable(100, 0, False, 150, 0)
        p.update(165.0, 1000, False, 150, 1000, 1000)
        self.assertFalse(p.on); self.assertEqual(p.alasan_mati, "suhu")

    def test_watchdog(self):
        p = Pid(); p.enable(100, 0, False, 150, 0)
        p.watchdog(5000, 0); self.assertTrue(p.on)
        p.watchdog(5001, 0); self.assertFalse(p.on); self.assertEqual(p.alasan_mati, "watchdog")

    def test_derivatif_tanpa_lonjakan_setelah_gangguan(self):
        p = Pid(kd=10.0); p.enable(100, 0, False, 150, 0)
        p.update(100, 1000, False, 150, 1000, 1000)
        p.update(float("nan"), 2000, False, 150, 2000, 1000)
        p.update(104, 3000, False, 150, 3000, 3000)          # suhu loncat 4 C setelah NAN
        self.assertEqual(p.df, 0.0)


class UjiAdaptif(unittest.TestCase):
    def test_adaptif_off_sama_dengan_pid_biasa(self):
        a = simulasi(detik=1500, pid=Pid(adapt=False))[1]
        b = simulasi(detik=1500, pid=Pid(lp=0, li=0, ld=0))[1]
        self.assertEqual([r[1] for r in a], [r[1] for r in b])
        self.assertTrue(all(r[6] == KP and r[7] == KI for r in a))

    def test_gain_mengikuti_hukum_dan_batas(self):
        p = Pid()
        p.gains(10.0, 0.2)
        self.assertAlmostEqual(p.kpe, KP + LP * 10)
        self.assertAlmostEqual(p.kie, KI + LI * 10)
        self.assertAlmostEqual(p.kde, KD + LD * 0.2)
        p.gains(200.0, 50.0)
        self.assertEqual((p.kpe, p.kie, p.kde), (max(KP, KP_MAX), max(KI, KI_MAX), max(KD, KD_MAX)))
        q = Pid(kp=5.0)                                      # Kp0 > Kp_max: tidak diturunkan
        q.gains(0.0, 0.0); self.assertEqual(q.kpe, 5.0)

    def test_gain_dipakai_selalu_dalam_batas(self):
        rows = simulasi(detik=3000)[1]
        self.assertTrue(all(r[6] <= KP_MAX + 1e-9 and r[7] <= KI_MAX + 1e-9 and r[8] <= KD_MAX + 1e-9 for r in rows))

    def test_gangguan_ubi_adaptif_vs_biasa(self):
        # steady di 150 C, lalu ubi masuk pada t = 4000 s: suhu turun 10 C dalam 10 s
        beban = lambda i: -1.0 if 4000 <= i < 4010 else 0.0
        hasil = {}
        for nama, pid in (("biasa", Pid(adapt=False)), ("adaptif", Pid())):
            rows = simulasi(detik=6000, pid=pid, beban=beban)[1]
            seg = [r for r in rows if r[0] >= 4000]
            iae = sum(abs(150.0 - r[1]) for r in seg)
            turun = 150.0 - min(r[1] for r in seg)
            pulih = next((r[0] - 4000 for r in seg if r[0] > 4010 and abs(150.0 - r[1]) < 1.0), None)
            hasil[nama] = (iae, turun, pulih)
            print("\n  [plant dugaan] %-7s IAE %.0f C*s | turun maks %.2f C | pulih (<1 C) %s s"
                  % (nama, iae, turun, pulih))
        self.assertLessEqual(hasil["adaptif"][0], hasil["biasa"][0])


if __name__ == "__main__":
    print("Konstanta dari firmware: Kp %.3f Ki %.4f Kd %.2f Umax %.0f ramp %.2f | lambda %.3f %.4f %.2f | max %.1f %.3f %.1f"
          % (KP, KI, KD, UMAX, RAMP, LP, LI, LD, KP_MAX, KI_MAX, KD_MAX))
    unittest.main(verbosity=2)
