"""Konfigurasi logger. Semua bisa ditimpa lewat argumen CLI (python logger.py -h)."""

WS_URL = "ws://192.168.4.1:81/"   # WebSocket ESP32 (softAP DeepFryer-ESP32)
FOLDER_DATA = "data"              # relatif terhadap folder Python/
JENDELA_S = 300                   # lebar jendela geser grafik live (detik)
REFRESH_PLOT_S = 0.5              # periode gambar ulang grafik
WS_TIMEOUT_S = 5.0                # tanpa pesan selama ini -> koneksi dianggap putus, sambung ulang
RECONNECT_MAKS_S = 10.0           # jeda maksimum antar percobaan sambung ulang
FSYNC_TIAP_BARIS = 10             # paksa tulis ke disk tiap N baris (aman bila laptop mati)
PERINGATAN_SUHU_DIAM_S = 5.0      # peringatkan bila status masuk tapi data suhu tidak
