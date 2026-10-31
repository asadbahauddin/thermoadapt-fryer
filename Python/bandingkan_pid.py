"""Bandingkan PID ADAPTIF vs PID KONVENSIONAL dari CSV logger (Uji Sistem / uji kematangan).

    python bandingkan_pid.py data/deepfryer_A.csv data/deepfryer_B.csv [lebih banyak CSV ...]
    python bandingkan_pid.py data/*.csv --keluar hasil_banding

Setiap siklus goreng = saat keranjang mulai turun (uji_tahap 4) sampai keranjang naik lagi
(tahap 6). Mode PID diambil dari kolom pid_adaptif (1 adaptif, 0 konvensional) saat siklus mulai.
Metrik per siklus (t = 0 saat Z mulai turun, beban keripik masuk):
  suhu_awal     suhu saat Z mulai turun (C)
  turun_maks    setpoint - suhu terendah selama goreng (C, makin kecil makin baik)
  overshoot     suhu tertinggi selama goreng - setpoint (C, <= 0 berarti tidak lewat)
  pulih_s       waktu dari suhu terendah sampai kembali ke SP-3 C (detik; kosong = tidak pulih)
  iae / ise     integral |e| / e^2 selama goreng (C*s / C^2*s), e = setpoint - suhu
  rata_e        rata-rata |e| selama goreng (C)
  katup_rata    rata-rata bukaan katup (%), ff_maks = feedforward terbesar (%)
Keluaran: <keluar>.csv (tabel per siklus + rata-rata per mode) dan <keluar>.png (grafik siklus
yang diselaraskan pada t = 0, per mode warna berbeda).
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

BAND_PULIH_C = 3.0


def siklus_dari_csv(path):
    df = pd.read_csv(path, encoding="utf-8-sig")
    if "uji_tahap" not in df or df.uji_tahap.dropna().empty:
        return []
    df["t"] = df.elapsed_s - df.elapsed_s.iloc[0]
    df["suhu"] = df.suhu_c.interpolate(limit=10)
    tahap = df.uji_tahap.fillna(-1).astype(int)
    mulai = df.index[(tahap == 4) & (tahap.shift() != 4)]
    hasil = []
    for i0 in mulai:
        sisa = df.loc[i0:]
        akhir = sisa.index[sisa.uji_tahap.fillna(-1).astype(int).isin([6, 7, 8, 9, 0, 1, 2, 3])
                           & (sisa.index > i0) & (sisa.uji_tahap.fillna(-1).astype(int) != 5)
                           & (sisa.uji_tahap.fillna(-1).astype(int) != 4)]
        if len(akhir) == 0:
            continue
        i1 = akhir[0]
        if int(df.uji_tahap[i1]) != 6:          # dibatalkan sebelum timer habis -> tidak dibandingkan
            continue
        d = df.loc[i0:i1]
        sp = float(d.setpoint_c.iloc[0])
        t = (d.t - d.t.iloc[0]).to_numpy()
        T = d.suhu.to_numpy()
        e = sp - T
        dt = np.diff(t, prepend=t[0])
        imin = int(np.nanargmin(T))
        pulih = d.iloc[imin:]
        ok = pulih.index[pulih.suhu >= sp - BAND_PULIH_C]
        mode = d.pid_adaptif.dropna()
        hasil.append(dict(
            file=os.path.basename(path), t_mulai_s=round(float(d.t.iloc[0]), 1),
            mode=("ADAPTIF" if int(mode.iloc[0]) == 1 else "KONVENSIONAL") if len(mode) else "?",
            setpoint=sp, durasi_s=round(float(t[-1]), 1), suhu_awal=round(float(T[0]), 2),
            suhu_min=round(float(np.nanmin(T)), 2), turun_maks=round(sp - float(np.nanmin(T)), 2),
            overshoot=round(float(np.nanmax(T)) - sp, 2),
            pulih_s=round(float(df.t[ok[0]] - df.t[d.index[imin]]), 1) if len(ok) else None,
            iae=round(float(np.nansum(np.abs(e) * dt)), 1), ise=round(float(np.nansum(e ** 2 * dt)), 1),
            rata_e=round(float(np.nanmean(np.abs(e))), 2),
            katup_rata=round(float(d.servo_bukaan_pct.mean()), 1),
            ff_maks=round(float(d.pid_ff_pct.max()), 1) if "pid_ff_pct" in d and d.pid_ff_pct.notna().any() else None,
            _t=t, _T=T))
    return hasil


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--keluar", default=os.path.join("data", "banding_pid"))
    a = ap.parse_args(argv)
    semua = []
    for p in a.csv:
        semua += siklus_dari_csv(p)
    if not semua:
        print("Tidak ada siklus goreng lengkap (tahap 4 -> 5 -> 6) di CSV tersebut.")
        return 1
    tab = pd.DataFrame([{k: v for k, v in s.items() if not k.startswith("_")} for s in semua])
    pd.set_option("display.width", 200)
    print(tab.drop(columns=["file"]).to_string(index=False))
    kol = ["turun_maks", "overshoot", "pulih_s", "iae", "ise", "rata_e", "katup_rata"]
    rata = tab.groupby("mode")[kol].mean().round(2)
    rata.insert(0, "n_siklus", tab.groupby("mode").size())
    print("\nRata-rata per mode:\n" + rata.to_string())
    if tab.setpoint.nunique() > 1:
        print("\nCatatan: setpoint berbeda-beda; bandingkan per setpoint yang sama:")
        print(tab.groupby(["setpoint", "mode"])[["turun_maks", "overshoot", "iae"]].mean().round(2).to_string())

    with open(a.keluar + ".csv", "w", encoding="utf-8-sig", newline="") as f:
        tab.to_csv(f, index=False)
        f.write("\n")
        rata.to_csv(f)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 5.5))
    warna = {"ADAPTIF": "tab:red", "KONVENSIONAL": "tab:blue", "?": "gray"}
    sudah = set()
    for s in semua:
        lab = s["mode"] if s["mode"] not in sudah else None
        sudah.add(s["mode"])
        ax.plot(s["_t"], s["_T"] - s["setpoint"], color=warna[s["mode"]], lw=1.4, alpha=0.85, label=lab)
    ax.axhline(0, color="k", lw=1, ls="--")
    ax.axhspan(-BAND_PULIH_C, BAND_PULIH_C, color="green", alpha=0.08, label="SP ±3 °C")
    ax.set_xlabel("Waktu sejak keranjang mulai turun (s)")
    ax.set_ylabel("Suhu - setpoint (°C)")
    ax.set_title("Respons beban keripik: PID Adaptif vs PID Konvensional")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(a.keluar + ".png", dpi=130)
    print("\nTersimpan: %s.csv dan %s.png" % (a.keluar, a.keluar))
    return 0


if __name__ == "__main__":
    sys.exit(main())
