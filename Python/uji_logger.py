"""Uji otomatis logger TANPA hardware:  python uji_logger.py

1. Parser: pesan persis format snprintf firmware rev 12, kasus rusak/tepi.
2. Koneksi: server WebSocket tiruan ESP32 yang memutus koneksi -> harus sambung ulang.
3. End-to-end: logger.py --demo dijalankan sebagai proses terpisah, lalu CSV & PNG diperiksa.
"""
import base64
import csv
import glob
import hashlib
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from koneksi import PenerimaWS, buat_antrean  # noqa: E402
from parser_esp import KOLOM, Penggabung, parse  # noqa: E402

# contoh keluaran snprintf firmware (broadcastTemp / broadcastStatus)
TEMP_OK = ('{"type":"temp","t":152.25,"ok":true,"raw":"09840000","d":"frame valid",'
           '"up":12,"sp":160.0,"seq":12,"ms":12034}')
TEMP_ERR = ('{"type":"temp","t":null,"ok":false,"f":1,"raw":"00010001",'
            '"d":"FAULT: thermocouple LEPAS (open)","up":13,"sp":160.0,"seq":13,"ms":13034}')
TEMP_LAMA = '{"type":"temp","t":150.00,"ok":true,"raw":"09600000","d":"frame valid","up":5}'
# rev 15 (snprintf broadcastTemp): nilai ditahan, mentah null
TEMP_HELD = ('{"type":"temp","t":151.80,"ok":true,"raw":"00010001","d":"FAULT: thermocouple LEPAS (open)",'
             '"up":14,"sp":160.0,"seq":14,"ms":14034,"st":1,"te":42,"tr":null,"held":1,"ec":7}')
# rev 16: PID aktif (snprintf broadcastTemp/broadcastStatus)
TEMP_REV16 = ('{"type":"temp","t":148.20,"ok":true,"raw":"09840000","d":"frame valid","up":16,"sp":160.0,'
              '"seq":16,"ms":16034,"st":0,"te":0,"tr":148.25,"held":0,"ec":7,"pe":1,"pu":23.45,"se":149.10}')
TEMP_REV16_MATI = ('{"type":"temp","t":148.20,"ok":true,"raw":"09840000","d":"frame valid","up":17,"sp":160.0,'
                   '"seq":17,"ms":17034,"st":0,"te":0,"tr":148.25,"held":0,"ec":7,"pe":0,"pu":0.00,"se":null}')
TEMP_REV15 = ('{"type":"temp","t":152.10,"ok":true,"raw":"09840000","d":"frame valid",'
              '"up":15,"sp":160.0,"seq":15,"ms":15034,"st":1,"te":43,"tr":152.25,"held":0,"ec":7}')
ST = ('{"type":"st","ls":[0,1,0,0],"raw":[1,0,1,1],"vr":[1,1,1,1],"il":[0,1,1,1],"pn":[35,17,34,39],'
      '"bn":0,"dx":1,"xp":-1200,"xm":0,"en":1,"sx":3000,"ax":3000,'
      '"sa":28,"sg":30,"sr":0,"sl":0,"um":500,"ux":2500,"cd":0,"od":55,"tc":180,'
      '"dcc":-50,"dct":-50,"dcm":50,"dcv":24,"sp":160.0}')
ST_REV16 = ST.replace('"sp":160.0}', '"sp":160.0,"kp":1.900,"ki":0.05600,"kd":0.000,"pm":40.0}')


class UjiParser(unittest.TestCase):
    def test_parse_rusak(self):
        for t in ["", "{rusak", "[1,2]", '{"t":1}', '{"type":5}', None]:
            self.assertIsNone(parse(t))

    def test_baris_lengkap(self):
        g = Penggabung()
        g.status_masuk(parse(ST), 10.0)
        r = g.baris_dari_temp(parse(TEMP_OK), 10.2, "2026-10-20T10:00:00.000", 0.0)
        self.assertEqual(set(r), set(KOLOM))
        self.assertEqual(r["suhu_c"], 152.25)
        self.assertEqual(r["setpoint_c"], 160.0)
        self.assertAlmostEqual(r["error_c"], 7.75)
        self.assertAlmostEqual(r["servo_bukaan_pct"], 28 / 55 * 100, places=2)
        self.assertAlmostEqual(r["servo_target_pct"], 30 / 55 * 100, places=2)
        self.assertEqual(r["dc_v_rata"], 6.0)               # |−50|% x 50% x 24 V
        self.assertEqual(r["x_pos_step"], -1200)
        self.assertEqual([r["ls_zatas_kena"], r["ls_zbawah_kena"]], [0, 1])
        self.assertEqual([r["ls_zatas_raw"], r["ls_zbawah_raw"]], [1, 0])
        self.assertEqual(r["status_umur_ms"], 200)
        for k in ("kp", "ki", "kd", "pid_output_pct", "pid_aktif", "pid_umax_pct", "buzzer_aktif"):
            self.assertIsNone(r[k], k)                       # pesan rev 12: belum ada -> kosong

    def test_sensor_error(self):
        g = Penggabung()
        r = g.baris_dari_temp(parse(TEMP_ERR), 0, "x", 0)
        self.assertIsNone(r["suhu_c"])
        self.assertIsNone(r["error_c"])
        self.assertEqual(r["sensor_ok"], 0)
        self.assertEqual(r["fault_kode"], 1)

    def test_rev15_mentah_held_ec_steptest(self):
        g = Penggabung()
        r = g.baris_dari_temp(parse(TEMP_HELD), 0, "x", 0)
        self.assertEqual(r["suhu_c"], 151.8)                 # nilai ditahan tetap tercatat
        self.assertIsNone(r["suhu_mentah_c"])                # tr null -> kosong
        self.assertEqual((r["suhu_ditahan"], r["sensor_error_total"]), (1, 7))
        self.assertEqual((r["step_test_status"], r["step_test_elapsed_s"]), (1, 42))
        r = g.baris_dari_temp(parse(TEMP_REV15), 1, "x", 1)
        self.assertEqual((r["suhu_c"], r["suhu_mentah_c"], r["suhu_ditahan"]), (152.1, 152.25, 0))
        r = g.baris_dari_temp(parse(TEMP_OK), 2, "x", 2)       # pesan rev 12 tanpa key baru
        self.assertIsNone(r["suhu_mentah_c"])
        self.assertIsNone(r["suhu_ditahan"])
        self.assertIsNone(r["step_test_status"])

    def test_rev16_pid(self):
        g = Penggabung()
        g.status_masuk(parse(ST_REV16), 0)
        r = g.baris_dari_temp(parse(TEMP_REV16), 0.1, "x", 0)
        self.assertEqual((r["pid_aktif"], r["pid_output_pct"], r["setpoint_efektif_c"]), (1, 23.45, 149.1))
        self.assertEqual((r["kp"], r["ki"], r["kd"], r["pid_umax_pct"]), (1.9, 0.056, 0.0, 40.0))
        self.assertEqual(r["servo_tutup_deg"], 0)                 # um/ux servo tidak tertukar dgn pm
        r = g.baris_dari_temp(parse(TEMP_REV16_MATI), 1.1, "x", 1)
        self.assertEqual(r["pid_aktif"], 0)
        self.assertIsNone(r["setpoint_efektif_c"])                # se null -> kosong

    def test_firmware_lama_tanpa_sp(self):
        g = Penggabung()
        r = g.baris_dari_temp(parse(TEMP_LAMA), 0, "x", 0)
        self.assertEqual(r["suhu_c"], 150.0)
        self.assertIsNone(r["setpoint_c"])
        self.assertIsNone(r["esp_seq"])

    def test_paket_hilang_dan_reset(self):
        g = Penggabung()
        for seq in (1, 2, 5, 6, 1, 2):
            g.baris_dari_temp(parse('{"type":"temp","t":1,"ok":true,"seq":%d}' % seq), 0, "x", 0)
        self.assertEqual(g.paket_hilang, 2)
        self.assertEqual(g.esp_reset, 1)

    def test_waktu_jam_esp_dan_reset(self):
        g = Penggabung()
        el = []
        # (ms ESP, elapsed jam laptop dengan jitter WiFi); ESP reset setelah sampel ke-3
        for ms, pc in ((5000, 0.0), (6000, 1.3), (7000, 1.9), (800, 30.0), (1800, 31.4)):
            r = g.baris_dari_temp(parse('{"type":"temp","t":1,"ok":true,"ms":%d}' % ms), 0, "x", pc)
            el.append(r["elapsed_s"])
        self.assertEqual(el, [0.0, 1.0, 2.0, 30.0, 31.0])

    def test_limit_kena_sesaat_tidak_hilang(self):
        g = Penggabung()
        g.status_masuk(parse(ST.replace('"ls":[0,1,0,0]', '"ls":[1,0,0,0]')), 0)
        g.status_masuk(parse(ST.replace('"ls":[0,1,0,0]', '"ls":[0,0,0,0]')), 0.25)
        r = g.baris_dari_temp(parse(TEMP_OK), 0.3, "x", 0)
        self.assertEqual(r["ls_zatas_kena"], 1)
        r = g.baris_dari_temp(parse(TEMP_OK), 1.3, "x", 1)
        self.assertEqual(r["ls_zatas_kena"], 0)              # akumulator di-reset

    def test_nilai_aneh(self):
        g = Penggabung()
        g.status_masuk(parse(ST.replace('"cd":0,"od":55', '"cd":40,"od":40')
                             .replace('"sa":28', '"sa":true')), 0)
        r = g.baris_dari_temp(parse(TEMP_OK.replace("152.25", "NaN")), 0, "x", 0)
        self.assertIsNone(r["suhu_c"])
        self.assertIsNone(r["servo_bukaan_pct"])
        self.assertIsNone(r["servo_sudut_deg"])


class ServerWSMini(threading.Thread):
    """Server WebSocket minimal (stdlib) yang meniru ESP32: kirim pesan teks berkala.
    putus_setelah: tutup koneksi setelah N pesan (meniru ESP restart / WiFi putus)."""

    def __init__(self, pesan, putus_setelah=None):
        super().__init__(daemon=True)
        self.pesan = pesan
        self.putus_setelah = putus_setelah
        self.sock = socket.socket()
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(2)
        self.port = self.sock.getsockname()[1]
        self.n_klien = 0

    @staticmethod
    def _frame(teks):
        d = teks.encode()
        n = len(d)
        hdr = bytes([0x81, n]) if n < 126 else bytes([0x81, 126]) + struct.pack(">H", n)
        return hdr + d

    def run(self):
        while True:
            try:
                c, _ = self.sock.accept()
            except OSError:
                return
            self.n_klien += 1
            req = c.recv(4096).decode()
            key = re.search(r"Sec-WebSocket-Key: (.+)\r\n", req).group(1).strip()
            acc = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11")
                                                .encode()).digest()).decode()
            c.sendall(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                       "Connection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n" % acc).encode())
            try:
                for i, p in enumerate(self.pesan):
                    if self.n_klien == 1 and self.putus_setelah and i >= self.putus_setelah:
                        break
                    c.sendall(self._frame(p))
                    time.sleep(0.01)
                time.sleep(0.5)
            except OSError:
                pass
            c.close()


class UjiKoneksiWS(unittest.TestCase):
    def test_terima_dan_sambung_ulang(self):
        pesan = [ST] + [TEMP_OK.replace('"seq":12', '"seq":%d' % i) for i in range(1, 21)]
        srv = ServerWSMini(pesan, putus_setelah=8)
        srv.start()
        q = buat_antrean()
        log = []
        r = PenerimaWS("ws://127.0.0.1:%d/" % srv.port, q, timeout_s=2, reconnect_maks_s=1,
                       log=log.append)
        r.start()
        diterima = []
        batas = time.time() + 15
        while time.time() < batas and len(diterima) < 8 + len(pesan):
            try:
                diterima.append(q.get(timeout=0.5)[2])
            except Exception:
                pass
        r.stop()
        self.assertGreaterEqual(r.n_sambung, 2, log)          # sambung ulang setelah server memutus
        self.assertEqual(len(diterima), 8 + len(pesan), log)  # tidak ada pesan rusak/terpotong
        self.assertTrue(all(parse(t) is not None for t in diterima))


class UjiEndToEnd(unittest.TestCase):
    def test_demo(self):
        d = tempfile.mkdtemp()
        try:
            here = os.path.dirname(os.path.abspath(__file__))
            p = subprocess.run([sys.executable, os.path.join(here, "logger.py"), "--demo",
                                "--demo-kecepatan", "100", "--durasi", "1000", "--tanpa-grafik",
                                "--folder", d, "--skenario", "uji e2e"],
                               capture_output=True, text=True, timeout=120)
            self.assertEqual(p.returncode, 0, p.stderr)
            csvs = glob.glob(os.path.join(d, "*_uji_e2e_DEMO.csv"))
            pngs = glob.glob(os.path.join(d, "*_uji_e2e_DEMO.png"))
            self.assertEqual(len(csvs), 1, os.listdir(d))
            self.assertEqual(len(pngs), 1)
            with open(csvs[0], encoding="utf-8-sig", newline="") as f:
                rows = list(csv.DictReader(f))
                self.assertEqual(list(rows[0].keys()), KOLOM)
            self.assertGreater(len(rows), 950)
            self.assertEqual(rows[-1]["paket_hilang"], "2")
            err = [r for r in rows if r["sensor_ok"] == "0"]
            self.assertEqual(len(err), 1)                     # rev 15: 2 detik ditahan, lalu error
            self.assertTrue(all(r["suhu_c"] == "" for r in err))
            held = [r for r in rows if r["suhu_ditahan"] == "1"]
            self.assertEqual(len(held), 2)
            self.assertTrue(all(r["suhu_c"] != "" and r["suhu_mentah_c"] == "" for r in held))
            self.assertEqual(rows[-1]["sensor_error_total"], "15")
            self.assertTrue(any(r["ls_zatas_kena"] == "1" for r in rows))
            self.assertTrue(any("Limit Z kena" in r["pesan_esp"] for r in rows))
            self.assertTrue(all(r["kp"] == "1.9" and r["pid_umax_pct"] == "40.0" for r in rows if r["n_status"]))
            pid = [r for r in rows if r["pid_aktif"] == "1"]
            self.assertGreater(len(pid), 0)                           # demo: PID mulai t = 800 s
            self.assertTrue(all(r["setpoint_efektif_c"] != "" for r in pid))
            self.assertTrue(all(0.0 <= float(r["pid_output_pct"]) <= 40.0 for r in pid))
            self.assertTrue(all(r["setpoint_efektif_c"] == "" for r in rows if r["pid_aktif"] == "0"))
            self.assertIn("pesan rusak: 1", p.stdout)
        finally:
            shutil.rmtree(d, ignore_errors=True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
