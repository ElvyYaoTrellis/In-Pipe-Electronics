import pyvista as pv
from scipy.spatial.transform import Rotation as R
import math
import socket
import time
import threading
from vtkmodules.vtkCommonTransforms import vtkTransform

# -----------------------------
# Config
# -----------------------------
HOST = "192.168.8.232"
PORT = 6020
TEST = False

RENDER_HZ = 60.0     # visualization refresh rate
PRINT_RATE_HZ = 2.0  # debug print rate for parsed RPY (set 0 to disable)

# -----------------------------
# Parsing
# -----------------------------
def parse_euler_from_line(line: str):
    """
    Parse lines like:
      "INS TimeOfWeek: ..., LLA: ..., Euler: -162.1, -0.6,141.1"
    Returns (roll, pitch, yaw) in degrees, or None.
    """
    idx = line.find("Euler:")
    if idx == -1:
        return None

    e = line[idx + len("Euler:"):].strip()
    parts = [p.strip() for p in e.split(",")]
    if len(parts) < 3:
        return None

    try:
        r = float(parts[0])
        p = float(parts[1])
        y = float(parts[2])
    except ValueError:
        return None

    # Keep your original sign conventions
    roll = -r
    pitch = p
    yaw = -y
    return roll, pitch, yaw


# -----------------------------
# Shared state (latest sample)
# -----------------------------
lock = threading.Lock()
latest_rpy = (0.0, 0.0, 0.0)
latest_t = 0.0
stop_evt = threading.Event()


# -----------------------------
# Socket receiver thread
# -----------------------------
def recv_loop(sock: socket.socket):
    """
    Continuously receive TCP bytes, split into newline-terminated lines,
    parse Euler, and update only the latest_rpy (dropping older samples).
    """
    global latest_rpy, latest_t

    buf = ""
    while not stop_evt.is_set():
        try:
            data = sock.recv(4096)
            if not data:
                # peer closed
                break
            buf += data.decode(errors="ignore")

            # Process complete lines
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                parsed = parse_euler_from_line(line)
                if parsed is None:
                    continue
                with lock:
                    latest_rpy = parsed
                    latest_t = time.time()
        except (OSError, ConnectionError):
            break

    stop_evt.set()


# -----------------------------
# Connect (retry) + low-latency socket options
# -----------------------------
while True:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        # Disable Nagle (reduces latency for small frequent messages)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        s.connect((HOST, PORT))
        print("Connected successfully!")
        break
    except (ConnectionRefusedError, OSError) as e:
        print(f"Connection failed: {e}. Retrying in 2 seconds...")
        try:
            s.close()
        except Exception:
            pass
        time.sleep(2)

# Start receiver thread
t_recv = threading.Thread(target=recv_loop, args=(s,), daemon=True)
t_recv.start()


# -----------------------------
# PyVista setup
# -----------------------------
plotter = pv.Plotter()
imu = pv.Cube(x_length=1, y_length=0.5, z_length=0.2)
actor = plotter.add_mesh(imu, color="cyan", show_edges=True)

# Pump GUI events via update()
plotter.show(auto_close=False, interactive_update=True)


# -----------------------------
# Main render loop (never blocks on network)
# -----------------------------
dt = 1.0 / max(RENDER_HZ, 1.0)
last_print = 0.0
test_time = 0.0

try:
    while not stop_evt.is_set():
        if TEST:
            roll = 0.0
            pitch = math.sin(test_time) * 170.0
            yaw = 0.0
            test_time += 0.1
        else:
            with lock:
                roll, pitch, yaw = latest_rpy

        # Optional debug print at low rate
        if PRINT_RATE_HZ > 0:
            now = time.time()
            if now - last_print > (1.0 / PRINT_RATE_HZ):
                last_print = now
                with lock:
                    age_ms = (time.time() - latest_t) * 1000.0 if latest_t else float("inf")
                print(f"rpy(deg)= {roll:7.2f} {pitch:7.2f} {yaw:7.2f}   (sample age ~ {age_ms:5.1f} ms)")

        # Apply rotation matrix -> VTK transform (same as your original)
        rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()

        transform = vtkTransform()
        transform.Identity()
        transform.PostMultiply()
        transform.Concatenate([
            rot[0, 0], rot[0, 1], rot[0, 2], 0.0,
            rot[1, 0], rot[1, 1], rot[1, 2], 0.0,
            rot[2, 0], rot[2, 1], rot[2, 2], 0.0,
            0.0,       0.0,       0.0,       1.0
        ])

        actor.SetUserTransform(transform)

        # Pump events + redraw
        plotter.update()

        time.sleep(dt)

finally:
    stop_evt.set()
    try:
        s.close()
    except Exception:
        pass
