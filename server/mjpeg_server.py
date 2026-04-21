#!/usr/bin/env python3
"""
Lightweight MJPEG-over-HTTP server.
Captures from a V4L2 device using OpenCV (no GStreamer, no hardware encoder).
Compatible with cv2.VideoCapture on the client side.

Usage:
  python3 mjpeg_server.py [--device /dev/video0] [--port 8082] [--width 1280] [--height 720] [--fps 15] [--quality 70]
"""

import argparse
import threading
import time
import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer


# ── Shared frame store ────────────────────────────────────────────────────────

_lock  = threading.Lock()
_frame_jpg: bytes = b""
_fps_actual: float = 0.0


def capture_thread(device: str, width: int, height: int, fps: int, quality: int):
    global _frame_jpg, _fps_actual

    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS,          fps)

    if not cap.isOpened():
        print(f"[mjpeg] ERROR: could not open {device}")
        return

    encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
    frame_count = 0
    t0 = time.monotonic()

    print(f"[mjpeg] Capturing from {device} at {width}x{height}@{fps}fps")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue

        ok, jpg = cv2.imencode(".jpg", frame, encode_params)
        if not ok:
            continue

        with _lock:
            _frame_jpg = jpg.tobytes()

        frame_count += 1
        elapsed = time.monotonic() - t0
        if elapsed >= 2.0:
            _fps_actual = frame_count / elapsed
            frame_count = 0
            t0 = time.monotonic()


# ── HTTP handler ──────────────────────────────────────────────────────────────

class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # suppress per-request access logs

    def do_GET(self):
        if self.path == "/status":
            body = f'{{"fps": {_fps_actual:.1f}}}'.encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        # Stream endpoint (any other path)
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _lock:
                    jpg = _frame_jpg
                if not jpg:
                    time.sleep(0.05)
                    continue
                header = (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n"
                )
                self.wfile.write(header + jpg + b"\r\n")
                time.sleep(0.033)  # ~30 Hz max to client
        except (BrokenPipeError, ConnectionResetError):
            pass


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="MJPEG HTTP server (software encode)")
    ap.add_argument("--device",  default="/dev/video0")
    ap.add_argument("--port",    type=int, default=8082)
    ap.add_argument("--width",   type=int, default=1280)
    ap.add_argument("--height",  type=int, default=720)
    ap.add_argument("--fps",     type=int, default=15)
    ap.add_argument("--quality", type=int, default=70, help="JPEG quality 1-100")
    args = ap.parse_args()

    threading.Thread(
        target=capture_thread,
        args=(args.device, args.width, args.height, args.fps, args.quality),
        daemon=True,
    ).start()

    print(f"[mjpeg] Streaming at http://0.0.0.0:{args.port}/stream")
    print(f"[mjpeg] Status   at http://0.0.0.0:{args.port}/status")
    HTTPServer(("0.0.0.0", args.port), MJPEGHandler).serve_forever()
