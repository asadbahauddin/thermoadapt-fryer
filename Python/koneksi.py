"""Penerima WebSocket di thread terpisah, dengan sambung ulang otomatis.

Library: websocket-client (pip install websocket-client), mode sinkron.
Setiap pesan masuk ke antrean sebagai (waktu_pc_monotonic, waktu_dunia, teks).
"""
import queue
import threading
import time

import websocket


class PenerimaWS(threading.Thread):
    def __init__(self, url, antrean, timeout_s=5.0, reconnect_maks_s=10.0, log=print):
        super().__init__(daemon=True)
        self.url = url
        self.q = antrean
        self.timeout_s = timeout_s
        self.reconnect_maks_s = reconnect_maks_s
        self.log = log
        self.berhenti = threading.Event()
        self.terhubung = threading.Event()
        self.n_sambung = 0
        self._ws = None

    def stop(self):
        self.berhenti.set()
        ws = self._ws
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass

    def run(self):
        jeda = 1.0
        while not self.berhenti.is_set():
            try:
                self._ws = websocket.create_connection(self.url, timeout=self.timeout_s)
                self.n_sambung += 1
                self.terhubung.set()
                self.log("[ws] terhubung ke %s%s" % (self.url, "" if self.n_sambung == 1 else " (sambung ulang)"))
                jeda = 1.0
                while not self.berhenti.is_set():
                    teks = self._ws.recv()          # timeout -> WebSocketTimeoutException
                    if isinstance(teks, bytes):
                        teks = teks.decode("utf-8", "replace")
                    if teks:
                        self.q.put((time.monotonic(), time.time(), teks))
            except websocket.WebSocketTimeoutException:
                if not self.berhenti.is_set():
                    self.log("[ws] tidak ada data %.0f s, sambung ulang..." % self.timeout_s)
            except Exception as e:                   # putus / AP hilang / ESP restart
                if not self.berhenti.is_set():
                    self.log("[ws] %s: %s. Coba lagi %.0f s..." % (type(e).__name__, e, jeda))
            finally:
                self.terhubung.clear()
                if self._ws is not None:
                    try:
                        self._ws.close()
                    except Exception:
                        pass
                    self._ws = None
            if not self.berhenti.is_set():
                self.berhenti.wait(jeda)
                jeda = min(jeda * 2, self.reconnect_maks_s)


def buat_antrean():
    return queue.Queue(maxsize=100000)
