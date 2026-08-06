import json
import math
import random
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

t = 0


class Handler(BaseHTTPRequestHandler):

    def do_GET(self):

        if self.path == "/":

            with open("index.html", "rb") as f:
                html = f.read()

            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(html)

        elif self.path == "/events":

            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()

            global t

            while True:
                try:
                    t += 0.15

                    data = {
                        "mode_sistem": random.choice([
                            "MENGIKUTI",
                            "MENGHINDAR",
                            "MUNDUR",
                            "MENCARI",
                            "BERHENTI"
                        ]),
                        "arah_sektor": random.choice([
                            "LEFT",
                            "CENTER",
                            "RIGHT"
                        ]),
                        "us_kiri": round(40 + 20 * math.sin(t)),
                        "us_tengah": round(50 + 15 * math.sin(t + 1)),
                        "us_kanan": round(35 + 18 * math.cos(t)),
                        "pwm_kiri": random.randint(100, 255),
                        "pwm_kanan": random.randint(100, 255)
                    }

                    msg = f"data: {json.dumps(data)}\n\n"

                    self.wfile.write(msg.encode())
                    self.wfile.flush()

                    time.sleep(0.5)

                except (BrokenPipeError, ConnectionResetError):
                    break

        else:
            self.send_response(404)
            self.end_headers()


server = HTTPServer(("localhost", 8000), Handler)

print("Open http://localhost:8000")

server.serve_forever()