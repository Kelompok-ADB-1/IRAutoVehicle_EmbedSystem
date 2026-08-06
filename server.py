"""
============================================================
SERVER SIMULASI (untuk testing dashboard tanpa hardware)
============================================================
Skrip ini meniru perilaku ESP32-S3 kendaraan: menyajikan
halaman dashboard yang SAMA PERSIS dengan yang ada di firmware
(dashboard.html), dan mengirim data sensor PALSU secara
berkala lewat SSE ke endpoint /events - persis seperti
kirimDataSSE() di kendaraan_firmware.ino.

Tujuannya: supaya kamu bisa develop/uji tampilan dashboard di
laptop, tanpa harus nunggu robot fisik selesai dirakit.

CARA PAKAI:
  1. Pastikan file "dashboard.html" ada di folder yang sama
     dengan server.py ini.
  2. Jalankan:  python3 server.py
  3. Buka browser ke:  http://localhost:8000

Tidak butuh library tambahan - semua pakai Python standar.
============================================================
"""

import json
import random
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PORT = 8000
INTERVAL_DETIK = 0.1  # samakan dengan INTERVAL_SIKLUS (100ms) di firmware asli
HTML_PATH = Path(__file__).parent / "dashboard.html"

# ------------------------------------------------------------
# STATE SIMULASI (meniru pembacaan sensor kendaraan)
# ------------------------------------------------------------
state = {
    "us_kiri": 80.0,
    "us_tengah": 80.0,
    "us_kanan": 80.0,
    "arah": "TENGAH",
}

ARAH_OPSI  = ["KIRI", "TENGAH", "KANAN", "HILANG"]
ARAH_BOBOT = [0.15, 0.55, 0.15, 0.15]

JARAK_AMAN_CM  = 20.0
JARAK_IDEAL_CM = 30.0
PWM_NORMAL = 150
PWM_PUTAR  = 120


def random_walk(nilai, minv, maxv, langkah=8.0):
    """Geser nilai sedikit demi sedikit, biar grafiknya keliatan natural
    (bukan lompat-lompat acak tiap siklus)."""
    nilai += random.uniform(-langkah, langkah)
    return max(minv, min(maxv, nilai))


def buat_data_palsu():
    """Logika ini SENGAJA dibuat mirip dengan bagian navigasi di
    kendaraan_firmware.ino, supaya pola MODE/PWM yang muncul di
    dashboard terasa realistis, bukan cuma angka acak."""

    state["us_kiri"]   = random_walk(state["us_kiri"], 5, 150)
    state["us_tengah"] = random_walk(state["us_tengah"], 5, 150)
    state["us_kanan"]  = random_walk(state["us_kanan"], 5, 150)

    # Arah sektor jarang berubah tiap siklus (biar tidak terlalu jitter)
    if random.random() < 0.08:
        state["arah"] = random.choices(ARAH_OPSI, weights=ARAH_BOBOT)[0]
    arah = state["arah"]

    us = {"KIRI": state["us_kiri"], "TENGAH": state["us_tengah"], "KANAN": state["us_kanan"]}
    target = us.get(arah, state["us_tengah"])

    halangan_tengah = state["us_tengah"] < JARAK_AMAN_CM
    semua_terhalang = all(v < JARAK_AMAN_CM for v in us.values())

    if arah == "HILANG":
        mode = "MENCARI"
        pwm_kiri, pwm_kanan = PWM_PUTAR, -PWM_PUTAR
    elif semua_terhalang:
        mode = "MUNDUR"
        pwm_kiri, pwm_kanan = -PWM_NORMAL, -PWM_NORMAL
    elif halangan_tengah:
        mode = "MENGHINDAR"
        if state["us_kiri"] > state["us_kanan"]:
            pwm_kiri, pwm_kanan = -PWM_PUTAR, PWM_PUTAR
        else:
            pwm_kiri, pwm_kanan = PWM_PUTAR, -PWM_PUTAR
    elif target < JARAK_IDEAL_CM:
        mode = "MENGIKUTI"
        pwm_kiri, pwm_kanan = 0, 0
    else:
        mode = "MENGIKUTI"
        if arah == "TENGAH":
            pwm_kiri, pwm_kanan = PWM_NORMAL, PWM_NORMAL
        elif arah == "KIRI":
            pwm_kiri, pwm_kanan = int(PWM_NORMAL * 0.5), PWM_NORMAL
        else:
            pwm_kiri, pwm_kanan = PWM_NORMAL, int(PWM_NORMAL * 0.5)

    return {
        "timestamp": int(time.time() * 1000),
        "us_kiri": round(state["us_kiri"], 1),
        "us_tengah": round(state["us_tengah"], 1),
        "us_kanan": round(state["us_kanan"], 1),
        "arah_sektor": arah,
        "mode_sistem": mode,
        "pwm_kiri": pwm_kiri,
        "pwm_kanan": pwm_kanan,
    }


# ------------------------------------------------------------
# HTTP HANDLER (meniru handleRoot() dan handleEvents() di firmware)
# ------------------------------------------------------------
class SimHandler(BaseHTTPRequestHandler):
    # Supress log baris default yang berisik di terminal
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            self._kirim_dashboard()
        elif self.path == "/events":
            self._kirim_stream_sse()
        else:
            self.send_response(404)
            self.end_headers()

    def _kirim_dashboard(self):
        try:
            html = HTML_PATH.read_text(encoding="utf-8")
        except FileNotFoundError:
            self.send_response(500)
            self.end_headers()
            self.wfile.write(b"dashboard.html tidak ditemukan di folder yang sama dengan server.py")
            return

        body = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _kirim_stream_sse(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        try:
            while True:
                data = buat_data_palsu()
                pesan = f"data: {json.dumps(data)}\n\n"
                self.wfile.write(pesan.encode("utf-8"))
                self.wfile.flush()
                time.sleep(INTERVAL_DETIK)
        except (BrokenPipeError, ConnectionResetError):
            # Wajar terjadi begitu tab browser ditutup / refresh
            pass


def main():
    server = ThreadingHTTPServer(("0.0.0.0", PORT), SimHandler)
    print("============================================================")
    print(" Server simulasi kendaraan (data PALSU untuk testing)")
    print(f" Buka di browser: http://localhost:{PORT}")
    print(" Tekan Ctrl+C untuk berhenti.")
    print("============================================================")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer dihentikan.")
        server.shutdown()


if __name__ == "__main__":
    main()