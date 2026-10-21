"""Parser pesan WebSocket firmware rev 17 -> baris CSV (pesan rev 12-16 tetap terbaca).

Firmware mengirim 3 jenis pesan JSON:
  temp (1 Hz) : t, ok, f, raw, d, up, sp, seq, ms, st, te (rev 14), tr, held, ec (rev 15),
                pe, pu, se (rev 16: PID aktif, keluaran %, setpoint efektif),
                kpe, kie, kde (rev 17: gain yang SEDANG dipakai, adaptif)
                t = suhu tersaring (rev 21: median-5 -> moving avg-5 -> EMA, + hold), tr = mentah
  st   (4 Hz) : ls[4], raw[4], vr[4], il[4], pn[4], bn, dx, xp, xm, en, sx, ax,
                sa, sg, sr, sl, um, ux, cd, od, tc, dcc, dct, dcm, dcv, sp,
                kp, ki, kd, pm (rev 16; pm = Umax PID %, um = pulsa servo 0 derajat),
                ad, lp, li, ld (rev 17: adaptif ON/OFF, lambda). kp/ki/kd di 'st' = gain DASAR.
  msg         : text
Urutan limit switch di array: 0 Z Atas, 1 Z Bawah, 2 X Awal, 3 X Akhir.

Satu baris CSV = satu sampel suhu (pesan temp), digabung dengan status terakhir.
Kolom yang BELUM dikirim firmware (buzzer; PID pada firmware < rev 16) tetap ada di header, kosong.
"""
import json
import math

LS_NAMA = ["zatas", "zbawah", "xawal", "xakhir"]

KOLOM = (
    ["waktu_iso", "elapsed_s", "esp_seq", "esp_ms", "paket_hilang",
     "suhu_c", "sensor_ok", "fault_kode", "max31855_raw", "max31855_diag",
     # suhu_c = tersaring firmware (rev 21: cascade 3 tahap); mentah, flag ditahan, jumlah error sejak boot
     "suhu_mentah_c", "suhu_ditahan", "sensor_error_total",
     # rev 14: status step test di firmware (0 siap, 1 berjalan, 2 selesai, 3 berhenti) + detik berjalan
     "step_test_status", "step_test_elapsed_s",
     "setpoint_c", "error_c",
     # PID (rev 16): kosong bila firmware lama
     # kp/ki/kd = gain yang SEDANG dipakai (berubah tiap sampel saat adaptif); kp0.. = gain dasar
     "pid_aktif", "pid_output_pct", "setpoint_efektif_c", "kp", "ki", "kd", "pid_umax_pct",
     "pid_adaptif", "kp0", "ki0", "kd0", "lambda_p", "lambda_i", "lambda_d",
     "servo_sudut_deg", "servo_target_deg", "servo_bukaan_pct", "servo_target_pct",
     "servo_pulsa_mentah_us", "servo_kunci", "servo_tutup_deg", "servo_buka_deg",
     "dc_out_pct", "dc_target_pct", "dc_duty_maks_pct", "dc_supply_v", "dc_v_rata",
     "x_pos_step", "x_bergerak", "x_driver_aktif", "x_dir", "x_speed", "x_accel"]
    + ["ls_%s_kena" % n for n in LS_NAMA]
    + ["ls_%s_raw" % n for n in LS_NAMA]
    + ["buzzer_aktif",                      # belum dikirim firmware -> kosong
       "bench_mode", "status_umur_ms", "n_status", "pesan_esp"]
)


def parse(teks):
    """JSON -> dict, atau None bila rusak / bukan objek / tanpa 'type'."""
    try:
        m = json.loads(teks)
    except (ValueError, TypeError):
        return None
    if not isinstance(m, dict) or not isinstance(m.get("type"), str):
        return None
    return m


def _angka(v):
    """Angka valid (bukan bool/NaN/Inf) -> float, selain itu None."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    v = float(v)
    return v if math.isfinite(v) else None


def _bulat(v):
    """Penghitung/status -> int (agar CSV '15', bukan '15.0'), selain itu None."""
    v = _angka(v)
    return None if v is None else int(round(v))


def _bukaan_pct(deg, tutup, buka):
    deg, tutup, buka = _angka(deg), _angka(tutup), _angka(buka)
    if deg is None or tutup is None or buka is None or buka == tutup:
        return None
    return round((deg - tutup) / (buka - tutup) * 100.0, 2)


def _bit(arr, i):
    if isinstance(arr, list) and i < len(arr):
        v = _angka(arr[i])
        return None if v is None else int(v != 0)
    return None


class Penggabung:
    """Menggabung pesan st/msg ke sampel temp. Mendeteksi paket hilang & reset ESP."""

    def __init__(self):
        self.status = None
        self.status_t = None
        self.n_status = 0
        self.ls_kena_interval = [0, 0, 0, 0]
        self.pesan = []
        self.seq_terakhir = None
        self.paket_hilang = 0
        self.esp_reset = 0
        self._ms_dasar = None       # waktu ESP (ms) acuan
        self._el_dasar = 0.0        # elapsed (s) pada acuan itu
        self._ms_prev = None

    def _elapsed(self, ms, elapsed_pc):
        """Waktu sampel memakai jam ESP (bebas jitter WiFi). Tanpa 'ms' (firmware lama)
        -> jam laptop. Saat ESP reset (ms mundur) disambung dengan jam laptop."""
        if ms is None:
            return elapsed_pc
        if self._ms_dasar is None or (self._ms_prev is not None and ms < self._ms_prev):
            self._ms_dasar, self._el_dasar = ms, elapsed_pc
        self._ms_prev = ms
        return self._el_dasar + (ms - self._ms_dasar) / 1000.0

    def status_masuk(self, m, t_pc):
        self.status = m
        self.status_t = t_pc
        self.n_status += 1
        hit = m.get("ls")
        for i in range(4):
            if _bit(hit, i) == 1:
                self.ls_kena_interval[i] = 1   # limit yang kena sesaat di antara 2 sampel tetap tercatat

    def pesan_masuk(self, m):
        teks = m.get("text")
        if isinstance(teks, str) and teks:
            self.pesan.append(teks.replace("\n", " "))

    def baris_dari_temp(self, m, t_pc, waktu_iso, elapsed_pc):
        """Bentuk satu baris (dict kolom -> nilai; None = kosong).
        elapsed_pc = detik sejak sampel pertama menurut jam laptop (cadangan)."""
        r = dict.fromkeys(KOLOM)
        r["waktu_iso"] = waktu_iso
        r["elapsed_s"] = round(self._elapsed(_angka(m.get("ms")), elapsed_pc), 3)

        seq = _angka(m.get("seq"))
        if seq is not None:
            seq = int(seq)
            if self.seq_terakhir is not None:
                if seq > self.seq_terakhir + 1:
                    self.paket_hilang += seq - self.seq_terakhir - 1
                elif seq <= self.seq_terakhir:
                    self.esp_reset += 1        # ESP restart: nomor urut mulai dari awal
            self.seq_terakhir = seq
        r["esp_seq"] = seq
        r["esp_ms"] = _angka(m.get("ms"))
        r["paket_hilang"] = self.paket_hilang

        ok = m.get("ok") is True
        suhu = _angka(m.get("t")) if ok else None
        sp = _angka(m.get("sp"))
        r["suhu_c"] = suhu
        r["sensor_ok"] = int(ok)
        r["fault_kode"] = _angka(m.get("f")) if not ok else 0
        r["max31855_raw"] = m.get("raw") if isinstance(m.get("raw"), str) else None
        r["max31855_diag"] = m.get("d") if isinstance(m.get("d"), str) else None
        r["suhu_mentah_c"] = _angka(m.get("tr"))          # null (fault) -> kosong
        held = _angka(m.get("held"))
        r["suhu_ditahan"] = None if held is None else int(held != 0)
        r["sensor_error_total"] = _bulat(m.get("ec"))
        r["step_test_status"] = _bulat(m.get("st"))
        r["step_test_elapsed_s"] = _bulat(m.get("te"))
        r["pid_aktif"] = _bulat(m.get("pe"))
        r["pid_output_pct"] = _angka(m.get("pu"))
        r["setpoint_efektif_c"] = _angka(m.get("se"))     # null saat PID mati -> kosong
        r["kp"] = _angka(m.get("kpe"))                    # rev 17: gain yang dipakai sampel ini
        r["ki"] = _angka(m.get("kie"))
        r["kd"] = _angka(m.get("kde"))
        r["setpoint_c"] = sp
        r["error_c"] = round(sp - suhu, 3) if (sp is not None and suhu is not None) else None

        s = self.status
        if s is not None:
            r["servo_sudut_deg"] = _angka(s.get("sa"))
            r["servo_target_deg"] = _angka(s.get("sg"))
            r["servo_bukaan_pct"] = _bukaan_pct(s.get("sa"), s.get("cd"), s.get("od"))
            r["servo_target_pct"] = _bukaan_pct(s.get("sg"), s.get("cd"), s.get("od"))
            r["servo_pulsa_mentah_us"] = _angka(s.get("sr"))
            r["servo_kunci"] = _angka(s.get("sl"))
            r["servo_tutup_deg"] = _angka(s.get("cd"))
            r["servo_buka_deg"] = _angka(s.get("od"))
            dcc, dcm, dcv = _angka(s.get("dcc")), _angka(s.get("dcm")), _angka(s.get("dcv"))
            r["dc_out_pct"] = dcc
            r["dc_target_pct"] = _angka(s.get("dct"))
            r["dc_duty_maks_pct"] = dcm
            r["dc_supply_v"] = dcv
            if None not in (dcc, dcm, dcv):
                r["dc_v_rata"] = round(abs(dcc) / 100.0 * dcm / 100.0 * dcv, 3)
            r["x_pos_step"] = _angka(s.get("xp"))
            r["x_bergerak"] = _angka(s.get("xm"))
            r["x_driver_aktif"] = _angka(s.get("en"))
            r["x_dir"] = _angka(s.get("dx"))
            r["x_speed"] = _angka(s.get("sx"))
            r["x_accel"] = _angka(s.get("ax"))
            for i, n in enumerate(LS_NAMA):
                kini = _bit(s.get("ls"), i)
                r["ls_%s_kena" % n] = None if kini is None else max(kini, self.ls_kena_interval[i])
                r["ls_%s_raw" % n] = _bit(s.get("raw"), i)
            r["bench_mode"] = _angka(s.get("bn"))
            r["kp0"] = _angka(s.get("kp"))                    # gain DASAR (rev 17)
            r["ki0"] = _angka(s.get("ki"))
            r["kd0"] = _angka(s.get("kd"))
            r["pid_adaptif"] = _bulat(s.get("ad"))
            r["lambda_p"] = _angka(s.get("lp"))
            r["lambda_i"] = _angka(s.get("li"))
            r["lambda_d"] = _angka(s.get("ld"))
            if m.get("kpe") is None:                           # firmware rev 16: gain tetap = dasar
                r["kp"], r["ki"], r["kd"] = r["kp0"], r["ki0"], r["kd0"]
            r["pid_umax_pct"] = _angka(s.get("pm"))
            r["status_umur_ms"] = int(round((t_pc - self.status_t) * 1000))
            if r["setpoint_c"] is None:
                r["setpoint_c"] = _angka(s.get("sp"))
                if r["setpoint_c"] is not None and suhu is not None:
                    r["error_c"] = round(r["setpoint_c"] - suhu, 3)
        r["n_status"] = self.n_status
        r["pesan_esp"] = " | ".join(self.pesan) if self.pesan else None

        # reset akumulator interval
        self.n_status = 0
        self.ls_kena_interval = [0, 0, 0, 0]
        self.pesan = []
        return r
