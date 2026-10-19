"""Logger data uji Deep Fryer (firmware rev 12) lewat WebSocket.

Pemakaian:
    python logger.py                 # sambung ke ws://192.168.4.1:81 (laptop di WiFi DeepFryer-ESP32)
    python logger.py --demo          # coba tanpa hardware (data simulasi)
    python logger.py -h              # semua opsi

Selama berjalan: grafik live (suhu vs setpoint, bukaan servo) + CSV ditulis tiap sampel.
Berhenti dengan MENUTUP jendela grafik atau Ctrl+C -> CSV ditutup dan PNG seluruh
pengujian disimpan otomatis di folder data/.
"""
import argparse
import os
import re
import signal
import sys
import threading
import time
from datetime import datetime

import config as cfg
from parser_esp import KOLOM, Penggabung, parse

FOLDER_INI = os.path.dirname(os.path.abspath(__file__))


def argumen():
    p = argparse.ArgumentParser(description="Logger data uji Deep Fryer via WebSocket")
    p.add_argument("--url", default=cfg.WS_URL, help="alamat WebSocket ESP32 [%(default)s]")
    p.add_argument("--folder", default=cfg.FOLDER_DATA, help="folder output [%(default)s]")
    p.add_argument("--skenario", default="", help="nama skenario opsional, ditambahkan ke nama file")
    p.add_argument("--jendela", type=float, default=cfg.JENDELA_S, help="lebar jendela grafik (s) [%(default)s]")
    p.add_argument("--durasi", type=float, default=0, help="berhenti otomatis setelah N detik (0 = manual)")
    p.add_argument("--tanpa-grafik", action="store_true", help="tanpa jendela grafik (PNG tetap disimpan)")
    p.add_argument("--demo", action="store_true", help="pakai data simulasi, tanpa ESP32")
    p.add_argument("--demo-kecepatan", type=float, default=1.0, help="percepatan waktu demo [%(default)s]")
    return p.parse_args()


def main():
    a = argumen()
    try:
        sys.stdout.reconfigure(errors="replace")   # konsol Windows: karakter aneh tidak membuat crash
    except Exception:
        pass

    import matplotlib
    if a.tanpa_grafik:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from grafik import GrafikLive, ringkas
    from pencatat import PencatatCSV

    folder = a.folder if os.path.isabs(a.folder) else os.path.join(FOLDER_INI, a.folder)
    os.makedirs(folder, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    skenario = re.sub(r"[^A-Za-z0-9_-]+", "_", a.skenario).strip("_")
    nama = "deepfryer_%s%s%s" % (stamp, "_" + skenario if skenario else "", "_DEMO" if a.demo else "")
    path_csv = os.path.join(folder, nama + ".csv")
    path_png = os.path.join(folder, nama + ".png")

    from koneksi import buat_antrean
    q = buat_antrean()
    if a.demo:
        from demo_esp import SumberDemo
        sumber = SumberDemo(q, kecepatan=a.demo_kecepatan)
    else:
        from koneksi import PenerimaWS
        sumber = PenerimaWS(a.url, q, cfg.WS_TIMEOUT_S, cfg.RECONNECT_MAKS_S)

    pencatat = PencatatCSV(path_csv, KOLOM, cfg.FSYNC_TIAP_BARIS)
    grafik = GrafikLive(a.jendela, judul=nama)
    if not a.tanpa_grafik:
        plt.show(block=False)

    selesai = threading.Event()
    alasan = ["?"]

    def stop(why):
        if not selesai.is_set():
            alasan[0] = why
            selesai.set()

    grafik.fig.canvas.mpl_connect("close_event", lambda e: stop("jendela grafik ditutup"))
    signal.signal(signal.SIGINT, lambda s, f: stop("Ctrl+C"))
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, lambda s, f: stop("Ctrl+Break"))

    print("Menyimpan ke  : %s" % path_csv)
    print("Sumber data   : %s" % ("SIMULASI (--demo)" if a.demo else a.url))
    print("Berhenti      : tutup jendela grafik atau tekan Ctrl+C\n")

    gab = Penggabung()
    t_mulai = None
    t_temp_terakhir = None
    t_gambar = 0.0
    n_rusak = 0
    peringatan_suhu = False
    baris = None
    sumber.start()

    try:
        while not selesai.is_set():
            # ---- kuras antrean ----
            for _ in range(5000):
                try:
                    t_mono, t_wall, teks = q.get_nowait()
                except Exception:
                    break
                m = parse(teks)
                if m is None:
                    n_rusak += 1
                    continue
                tipe = m["type"]
                if tipe == "st":
                    gab.status_masuk(m, t_mono)
                elif tipe == "msg":
                    gab.pesan_masuk(m)
                    print("\n[ESP] %s" % m.get("text"))
                elif tipe == "temp":
                    if t_mulai is None:
                        t_mulai = t_mono
                    t_temp_terakhir = t_mono
                    peringatan_suhu = False
                    iso = datetime.fromtimestamp(t_wall).isoformat(timespec="milliseconds")
                    baris = gab.baris_dari_temp(m, t_mono, iso, t_mono - t_mulai)
                    pencatat.tulis(baris)
                    grafik.tambah(baris)

            sekarang = time.monotonic()
            # ---- pengawas: status masuk tapi suhu tidak ----
            if (gab.status_t is not None and sekarang - gab.status_t < 2
                    and (t_temp_terakhir is None or sekarang - t_temp_terakhir > cfg.PERINGATAN_SUHU_DIAM_S)
                    and not peringatan_suhu):
                print("\n[!] Status ESP masuk tapi data suhu tidak datang > %.0f s" % cfg.PERINGATAN_SUHU_DIAM_S)
                peringatan_suhu = True

            # ---- tampilan ----
            if sekarang - t_gambar >= cfg.REFRESH_PLOT_S:
                t_gambar = sekarang
                if baris is not None:
                    info = "%s | %d sampel | suhu %s °C | SP %s °C | servo %s %% | hilang %d" % (
                        nama, pencatat.n, ringkas(baris["suhu_c"]), ringkas(baris["setpoint_c"]),
                        ringkas(baris["servo_bukaan_pct"], "%.0f"), gab.paket_hilang)
                    grafik.set_judul(info)
                    grafik.gambar()
                    sys.stdout.write("\r" + info[len(nama) + 3:].replace(" °C", " C") + "   ")
                    sys.stdout.flush()
                elif not getattr(sumber, "terhubung", threading.Event()).is_set() and not a.demo:
                    grafik.set_judul("%s | menunggu koneksi %s ..." % (nama, a.url))

            if a.durasi and baris is not None and baris["elapsed_s"] >= a.durasi:
                stop("durasi %.0f s tercapai" % a.durasi)
            if a.tanpa_grafik:
                time.sleep(0.05)
            else:
                plt.pause(0.05)
                if not grafik.jendela_terbuka():
                    stop("jendela grafik ditutup")
    except KeyboardInterrupt:
        stop("Ctrl+C")
    finally:
        sumber.stop()
        pencatat.tutup()
        print("\n\nBerhenti: %s" % alasan[0])
        if pencatat.n == 0:
            os.remove(path_csv)
            print("Tidak ada data suhu yang diterima -> CSV kosong dihapus, PNG tidak dibuat.")
            print("Cek: laptop tersambung ke WiFi DeepFryer-ESP32 dan web 192.168.4.1 bisa dibuka?")
        else:
            judul = "%s | %d sampel | paket hilang %d | ESP reset %d" % (
                nama, pencatat.n, gab.paket_hilang, gab.esp_reset)
            grafik.simpan_png_penuh(path_png, judul)
            print("CSV : %s (%d baris)" % (path_csv, pencatat.n))
            print("PNG : %s" % path_png)
            print("Paket suhu hilang: %d | ESP reset: %d | pesan rusak: %d | sambung: %d kali" % (
                gab.paket_hilang, gab.esp_reset, n_rusak, getattr(sumber, "n_sambung", 0)))
        try:
            plt.close("all")
        except Exception:
            pass


if __name__ == "__main__":
    main()
