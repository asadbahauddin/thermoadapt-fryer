"""Grafik live: (1) suhu vs setpoint vs setpoint efektif PID, (2) sudut servo vs keluaran PID.
Jendela geser N detik terakhir; PNG akhir memuat seluruh durasi.

Panel 2 memakai satu skala "% bukaan" (0% = sudut TUTUP, 100% = sudut BUKA) agar sudut servo
dan keluaran PID bisa dibandingkan langsung; sumbu kanan menunjukkan sudutnya dalam derajat
(dari sudut tutup/buka yang dikirim firmware, cd/od).
"""
import math

import matplotlib.pyplot as plt
import numpy as np


def _nan(v):
    return float("nan") if v is None else float(v)


class GrafikLive:
    def __init__(self, jendela_s=300, judul="Deep Fryer"):
        self.jendela_s = jendela_s
        self.t, self.suhu, self.sp, self.se, self.sv, self.pu = [], [], [], [], [], []
        self.cd, self.od = 0.0, 55.0          # sudut tutup/buka terakhir (derajat)

        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6.5))
        self.fig.canvas.manager.set_window_title(judul)
        self.l_suhu, = self.ax1.plot([], [], color="tab:red", lw=1.6, label="Suhu terukur")
        self.l_sp, = self.ax1.plot([], [], color="tab:orange", lw=1.6, ls="--", label="Setpoint",
                                   drawstyle="steps-post")
        self.l_se, = self.ax1.plot([], [], color="tab:green", lw=1.4, ls=":", label="Setpoint efektif (PID)")
        self.ax1.set_ylabel("Suhu (°C)")
        self.ax1.grid(True, alpha=0.3)
        self.ax1.legend(loc="upper left")

        self.l_sv, = self.ax2.plot([], [], color="tab:blue", lw=1.6, label="Sudut servo",
                                   drawstyle="steps-post")
        self.l_pu, = self.ax2.plot([], [], color="tab:purple", lw=1.4, ls="--", label="Keluaran PID",
                                   drawstyle="steps-post")
        self._sumbu_derajat(self.ax2)
        self.ax2.set_ylabel("Bukaan katup (%)")
        self.ax2.set_xlabel("Waktu sejak mulai (s)")
        self.ax2.grid(True, alpha=0.3)
        self.ax2.legend(loc="upper left")
        self.fig.tight_layout(rect=(0, 0, 1, 0.95))
        self.judul = self.fig.suptitle(judul, fontsize=10)

    def _sumbu_derajat(self, ax):
        span = lambda: (self.od - self.cd) if self.od != self.cd else 1.0
        sec = ax.secondary_yaxis("right", functions=(lambda p: self.cd + p / 100.0 * span(),
                                                     lambda d: (d - self.cd) / span() * 100.0))
        sec.set_ylabel("Sudut servo (°)")
        return sec

    def tambah(self, baris):
        self.t.append(float(baris["elapsed_s"]))
        self.suhu.append(_nan(baris["suhu_c"]))
        self.sp.append(_nan(baris["setpoint_c"]))
        self.se.append(_nan(baris.get("setpoint_efektif_c")))
        self.sv.append(_nan(baris["servo_bukaan_pct"]))
        aktif = baris.get("pid_aktif")
        self.pu.append(_nan(baris.get("pid_output_pct")) if aktif == 1 else float("nan"))  # PID mati: tidak digambar
        if baris.get("servo_tutup_deg") is not None and baris.get("servo_buka_deg") is not None:
            self.cd, self.od = float(baris["servo_tutup_deg"]), float(baris["servo_buka_deg"])

    def set_judul(self, teks):
        self.judul.set_text(teks)

    @staticmethod
    def _ylim(ax, *seri, pad_min=1.0):
        v = np.concatenate([np.asarray(s, float) for s in seri]) if seri else np.array([])
        v = v[np.isfinite(v)]
        if v.size == 0:
            return
        lo, hi = float(v.min()), float(v.max())
        pad = max((hi - lo) * 0.08, pad_min)
        ax.set_ylim(lo - pad, hi + pad)

    def gambar(self):
        if not self.t:
            return
        t = np.asarray(self.t)
        tmax = t[-1]
        t0 = max(0.0, tmax - self.jendela_s)
        i0 = int(np.searchsorted(t, t0))
        sl = slice(i0, None)
        self.l_suhu.set_data(t[sl], self.suhu[sl])
        self.l_sp.set_data(t[sl], self.sp[sl])
        self.l_se.set_data(t[sl], self.se[sl])
        self.l_sv.set_data(t[sl], self.sv[sl])
        self.l_pu.set_data(t[sl], self.pu[sl])
        self.ax1.set_xlim(t0, max(t0 + self.jendela_s, tmax))
        self._ylim(self.ax1, self.suhu[sl], self.sp[sl], self.se[sl], pad_min=1.0)
        self._ylim(self.ax2, self.sv[sl], self.pu[sl], pad_min=5.0)
        self.fig.canvas.draw_idle()

    def jendela_terbuka(self):
        return plt.fignum_exists(self.fig.number)

    def simpan_png_penuh(self, path, judul):
        """Gambar SELURUH durasi pengujian (bukan hanya jendela geser) ke PNG."""
        if not self.t:
            return False
        fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(12, 7))
        a1.plot(self.t, self.suhu, color="tab:red", lw=1.2, label="Suhu terukur")
        a1.plot(self.t, self.sp, color="tab:orange", lw=1.4, ls="--", label="Setpoint", drawstyle="steps-post")
        a1.plot(self.t, self.se, color="tab:green", lw=1.2, ls=":", label="Setpoint efektif (PID)")
        a1.set_ylabel("Suhu (°C)")
        a1.grid(True, alpha=0.3)
        a1.legend(loc="best")
        a2.plot(self.t, self.sv, color="tab:blue", lw=1.2, label="Sudut servo", drawstyle="steps-post")
        a2.plot(self.t, self.pu, color="tab:purple", lw=1.2, ls="--", label="Keluaran PID", drawstyle="steps-post")
        self._sumbu_derajat(a2)
        a2.set_ylabel("Bukaan katup (%)")
        a2.set_xlabel("Waktu sejak mulai (s)")
        a2.grid(True, alpha=0.3)
        a2.legend(loc="best")
        self._ylim(a1, self.suhu, self.sp, self.se)
        self._ylim(a2, self.sv, self.pu, pad_min=5.0)
        fig.suptitle(judul, fontsize=10)
        fig.tight_layout(rect=(0, 0, 1, 0.96))
        fig.savefig(path, dpi=150)
        plt.close(fig)
        return True


def ringkas(v, fmt="%.1f"):
    return "--" if v is None or (isinstance(v, float) and math.isnan(v)) else fmt % v
