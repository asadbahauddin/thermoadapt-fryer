"""Sumber data SIMULASI untuk mencoba logger tanpa hardware (python logger.py --demo).

Mengirim pesan JSON dengan format PERSIS seperti firmware rev 15 (temp 1 Hz, st 4 Hz),
dari plant FOPDT sederhana (bukan model alat sebenarnya). Sengaja disisipkan kasus
sulit: paket hilang, sensor error, pesan rusak, limit switch kena sesaat, pesan msg.
"""
import json
import random
import threading
import time


class SumberDemo(threading.Thread):
    def __init__(self, antrean, kecepatan=1.0, log=print):
        super().__init__(daemon=True)
        self.q = antrean
        self.k = max(0.1, float(kecepatan))   # 10 = waktu simulasi 10x lebih cepat
        self.log = log
        self.berhenti = threading.Event()
        self.n_sambung = 1

    def stop(self):
        self.berhenti.set()

    def _kirim(self, obj_atau_teks):
        teks = obj_atau_teks if isinstance(obj_atau_teks, str) else json.dumps(obj_atau_teks)
        self.q.put((time.monotonic(), time.time(), teks))

    def run(self):
        self.log("[demo] sumber simulasi aktif (kecepatan %.1fx)" % self.k)
        rnd = random.Random(7)
        T, Ta, K, tau, L = 30.0, 30.0, 2.2, 180.0, 12.0
        cd, od = 0, 55
        hist_u = []
        seq = 0
        ms = 0
        ec = 0
        t_akhir = T
        sp = 160.0
        for k in range(10 ** 9):                 # 1 iterasi = 0.25 s waktu simulasi
            if self.berhenti.is_set():
                return
            ts = k * 0.25
            bukaan = 0 if ts < 10 else (60 if ts < 400 else (45 if ts < 800 else 55))
            if ts >= 600:
                sp = 170.0
            hist_u.append(bukaan)
            u_tunda = hist_u[max(0, len(hist_u) - 1 - int(L / 0.25))]
            T += (K * u_tunda - (T - Ta)) / tau * 0.25
            sudut = cd + (od - cd) * bukaan / 100.0
            ls_zatas = 1 if 120 <= ts < 120.5 else 0  # kena hanya 0.5 s
            st = {"type": "st", "ls": [ls_zatas, 0, 0, 0], "raw": [1 - ls_zatas, 1, 1, 1],
                  "vr": [1, 1, 1, 1], "il": [0, 1, 1, 1], "pn": [35, 17, 34, 39],
                  "bn": 0, "dx": 0, "xp": int(ts * 10), "xm": 1, "en": 1, "sx": 3000, "ax": 3000,
                  "sa": round(sudut), "sg": round(sudut), "sr": 0, "sl": 0, "um": 500, "ux": 2500,
                  "cd": cd, "od": od, "tc": 180, "dcc": 25 if 200 <= ts < 205 else 0,
                  "dct": 25 if 200 <= ts < 205 else 0, "dcm": 50, "dcv": 24, "sp": sp}
            self._kirim(st)
            if k % 4 == 0:
                seq += 1
                ms = int(ts * 1000)
                if seq in (50, 51):
                    pass                              # paket hilang
                elif 300 <= seq < 303:                # fault: firmware rev 15 menahan nilai <= 3 s, lalu null
                    ec += 5
                    if seq < 302:
                        self._kirim({"type": "temp", "t": t_akhir, "ok": True, "raw": "00010001",
                                     "d": "FAULT: thermocouple LEPAS (open)", "up": int(ts), "sp": sp,
                                     "seq": seq, "ms": ms, "st": 0, "te": 0, "tr": None, "held": 1, "ec": ec})
                    else:
                        self._kirim({"type": "temp", "t": None, "ok": False, "f": 1, "raw": "00010001",
                                     "d": "FAULT: thermocouple LEPAS (open)", "up": int(ts), "sp": sp,
                                     "seq": seq, "ms": ms, "st": 0, "te": 0, "tr": None, "held": 0, "ec": ec})
                else:
                    tr = round(T + rnd.gauss(0, 0.2), 2)
                    t_akhir = round(T, 2)             # pengganti nilai tersaring
                    self._kirim({"type": "temp", "t": t_akhir, "ok": True, "raw": "0A0C1234",
                                 "d": "frame valid", "up": int(ts), "sp": sp, "seq": seq, "ms": ms,
                                 "st": 0, "te": 0, "tr": tr, "held": 0, "ec": ec})
                if seq == 100:
                    self._kirim("{rusak")
                if seq == 200:
                    self._kirim({"type": "msg", "text": "Limit Z kena: motor DC berhenti"})
            self.berhenti.wait(0.25 / self.k)
