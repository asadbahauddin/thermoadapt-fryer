"""Penulis CSV yang aman: baris ditulis langsung (bukan di akhir), flush tiap baris,
fsync berkala. Bila program/laptop mati mendadak, data sampai detik terakhir tetap ada."""
import csv
import os


class PencatatCSV:
    def __init__(self, path, kolom, fsync_tiap=10):
        self.path = path
        self.kolom = list(kolom)
        self.fsync_tiap = max(1, int(fsync_tiap))
        self.n = 0
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        # utf-8-sig: Excel membaca karakter non-ASCII dengan benar
        self._f = open(path, "w", newline="", encoding="utf-8-sig")
        self._w = csv.DictWriter(self._f, fieldnames=self.kolom, extrasaction="ignore")
        self._w.writeheader()
        self._f.flush()

    def tulis(self, baris):
        self._w.writerow({k: ("" if v is None else v) for k, v in baris.items()})
        self.n += 1
        self._f.flush()
        if self.n % self.fsync_tiap == 0:
            os.fsync(self._f.fileno())

    def tutup(self):
        if self._f and not self._f.closed:
            self._f.flush()
            os.fsync(self._f.fileno())
            self._f.close()
