#!/usr/bin/env python3
import subprocess
import socket
import select
import time
import glob

HOST = "0.0.0.0"
PORT = 6020

# Point this to the ACTUAL executable (not the folder)
ISCOMM_EXE = "/home/radxa/inertial-sense-sdk/ExampleProjects/ISComm/build/ISCommExample"

# Pick the first ACM port found
acm = sorted(glob.glob("/dev/ttyACM*"))
if not acm:
    raise RuntimeError("No /dev/ttyACM* found")
PORT_DEV = acm[0]
print(f"[IMU] using {PORT_DEV}")

# Launch ISComm with line-buffered stdout
process = subprocess.Popen(
    ["stdbuf", "-oL", "-eL", ISCOMM_EXE, PORT_DEV],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    bufsize=1,
)

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print(f"[IMU] TCP server listening on {HOST}:{PORT} ...")

conn, addr = server_socket.accept()
print(f"[IMU] Connected by {addr}")

# Reduce TCP latency for small frequent sends
conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

# Optional: print rate-limit for debug
last_dbg = 0.0

try:
    while True:
        # Block for at least one line
        line = process.stdout.readline()
        if not line:
            raise RuntimeError("ISCommExample ended or no stdout")

        # Drain any immediately-available lines so we keep only the newest
        last_line = line
        fd = process.stdout.fileno()
        while True:
            r, _, _ = select.select([fd], [], [], 0.0)
            if not r:
                break
            more = process.stdout.readline()
            if not more:
                break
            last_line = more

        # Send only the newest line
        try:
            conn.sendall(last_line.encode())
        except (BrokenPipeError, ConnectionResetError):
            print("[IMU] Client disconnected.")
            break

        # Debug print at 1 Hz max (avoid slowing the loop)
        now = time.time()
        if now - last_dbg > 1.0:
            last_dbg = now
            print(f"[IMU] tx: {last_line.strip()}")

finally:
    try:
        conn.close()
    except Exception:
        pass
    try:
        server_socket.close()
    except Exception:
        pass
    try:
        process.terminate()
    except Exception:
        pass
