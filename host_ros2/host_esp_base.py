#!/usr/bin/env python3
"""host_esp_base.py  (VELOCITY-ONLY + RTSP VIDEO + RESPONSIVE MOTOR HTTP + UART OUT)

ROS 2 Humble base-station host.

What it does:
- Subscribes to /joy (sensor_msgs/msg/Joy)
- Receives *RTSP* video (OpenCV, rtsp://<RADXA_IP>:8554/stream)
- Sends *velocity* commands to the Radxa 2Dac2Motor HTTP server WITHOUT queueing
  (single sender thread, latest-only)

NEW (Axis split):
- UART motor uses /joy.axes[1]  -> sends -255..255 over /dev/ttyUSB0 (newline-terminated)
- HTTP Motor2 (M2) uses /joy.axes[4]

Radxa 2Dac2Motor HTTP API:
  GET  /health
  POST /motor {"m1": 0.25, "m2": -0.10}
  POST /brake {}
  POST /cmd   {"cmd":"ilimit1 0.35"}

NEW:
- Press X button to toggle Radxa camera stream between /dev/video0 and /dev/video10
  (requires Radxa camera-switch server on port 8081 with POST /cam/0 and /cam/10)

NOTE:
- Your original code used X (button index 1) for pressure down. That conflicts with camera switching.
  This version moves pressure-down to button index 2 (commonly "B").
"""

import os
import threading
import time
import queue
from concurrent.futures import ThreadPoolExecutor

import cv2
import requests
import serial

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Joy

# ------------------ Config ------------------
PRESSURE_UART_PATH = "/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_02857388-if00-port0"
PRESSURE_BAUD = 115200

RADXA_IP = "192.168.8.232"

# -------- RTSP VIDEO --------
RTSP_URL = f"rtsp://{RADXA_IP}:8554/stream"

MOTOR_HTTP_PT = 8005
LED_HTTP_PT = 8080
VIDEO_SWITCH_PT = 8081  # Radxa camera switch server port

MOTOR_HEALTH = f"http://{RADXA_IP}:{MOTOR_HTTP_PT}/health"
MOTOR_CMD_URL = f"http://{RADXA_IP}:{MOTOR_HTTP_PT}/cmd"
MOTOR_JSON_URL = f"http://{RADXA_IP}:{MOTOR_HTTP_PT}/motor"
MOTOR_BRAKE_URL = f"http://{RADXA_IP}:{MOTOR_HTTP_PT}/brake"

LED_ON_URL = f"http://{RADXA_IP}:{LED_HTTP_PT}/on"
LED_OFF_URL = f"http://{RADXA_IP}:{LED_HTTP_PT}/off"

# Camera switch endpoints on Radxa
CAM0_URL = f"http://{RADXA_IP}:{VIDEO_SWITCH_PT}/cam/0"
CAM10_URL = f"http://{RADXA_IP}:{VIDEO_SWITCH_PT}/cam/10"

# Default HTTP timeouts for non-motor endpoints (health/cmd/led/cam-switch)
HTTP_TIMEOUT = (0.5, 5.0)  # (connect, read)
RECONNECT_DELAY_S = 1.5

# -------- UART motor output (Axis UART -> -255..255) --------
M2_UART_PATH = "/dev/ttyUSB0"
M2_UART_BAUD = 115200
M2_UART_RATE_HZ = 50.0  # 50 Hz feels responsive
M2_UART_EPS = 1  # resend only if change >= 1 count

# ------------------ Joystick mapping ------------------
BTN_PRESSURE_UP = 3  # Y button → pressure +0.05
BTN_PRESSURE_DN = 2  # (changed) B button → pressure -0.05  (X is used for cam switch)

# Axis split:
AXIS_UART = 1  # UART motor uses axes[1]
AXIS_M2 = 4    # HTTP Motor 2 uses axes[4]

# Triggers (common)
AXIS_M1_FWD = 5  # RT
AXIS_M1_REV = 2  # LT

# Fallback digital buttons
BTN_M1_FWD = 6
BTN_M1_REV = 7

BTN_LED_ON = 4        # LB → LED ON
BTN_LED_OFF = 5       # RB → LED OFF

BTN_BRAKE_ALL = 0     # A → brake both motors

# Camera toggle on X button
BTN_CAM_SWITCH = 1    # X button → toggle /dev/video0 <-> /dev/video10
CAM_SWITCH_DEBOUNCE_S = 0.35

# ------------------ Velocity control ------------------
MOTOR_DEADZONE = 0.12
HTTP_VEL_INTERVAL = 0.05  # 20 Hz "compute" loop

VEL_EPS = 0.02
VEL_KEEPALIVE_S = 0.25

JOY_DEADMAN_S = 0.35

# ------------------ Pressure range ------------------
PRESSURE_MIN = 0.0
PRESSURE_MAX = 7.0

# ------------------ Shared state ------------------
def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def trigger_to_01(x: float, seen_negative: bool) -> float:
    """
    Normalize trigger value to [0..1].

    Two common conventions:
      A) [0..1]  rest=0, pressed=1
      B) [-1..1] rest=1, pressed=-1  (pressed decreases)
    """
    if seen_negative:
        return clamp((1.0 - x) * 0.5, 0.0, 1.0)
    return clamp(x, 0.0, 1.0)

lock = threading.Lock()
state = {
    "pressure": 0.0,

    "ax_m1": 0.0,
    "ax_m2": 0.0,        # HTTP motor2 axis (axes[4])
    "ax_uart": 0.0,      # UART motor axis (axes[1])

    "m1_fwd_seen_negative": False,
    "m1_rev_seen_negative": False,

    "last_joy_ts": 0.0,
    "last_vel_post_ts": 0.0,      # (compute loop timing)
    "last_vel_sent": (0.0, 0.0),  # what we *requested* last (not necessarily delivered)
    "last_keepalive_ts": 0.0,

    "joy_alive": False,
    "led_on": False,
    "prev_buttons": [],

    "active_cam": 0,              # 0 => /dev/video0, 10 => /dev/video10
    "last_cam_switch_ts": 0.0,

    "health_ok": None,
    "health_summary": "",
}

# ------------------ HTTP helpers (bounded threads) ------------------
_session = requests.Session()
_http_exec = ThreadPoolExecutor(max_workers=6)  # prevents infinite thread growth

def _post_async(url, json=None, timeout=HTTP_TIMEOUT):
    def _do():
        try:
            _session.post(url, json=json, timeout=timeout)
        except Exception as e:
            print(f"[http] POST {url} failed: {e}")
    _http_exec.submit(_do)

def _get_json(url, timeout=HTTP_TIMEOUT):
    r = _session.get(url, timeout=timeout)
    r.raise_for_status()
    return r.json()

# ------------------ Motor TX (latest-only; no command queueing) ------------------
_motor_lock = threading.Lock()
_motor_latest = {"m1": 0.0, "m2": 0.0, "dirty": False}
_motor_event = threading.Event()
_motor_ctrl_q: "queue.SimpleQueue[tuple[str, object]]" = queue.SimpleQueue()

# Fast motor timeouts to stay responsive under load
MOTOR_POST_TIMEOUT = (0.2, 0.2)  # (connect, read)
MOTOR_KEEPALIVE_S = 0.25

def motor_set_velocity(v1: float, v2: float):
    with _motor_lock:
        _motor_latest["m1"] = float(v1)
        _motor_latest["m2"] = float(v2)
        _motor_latest["dirty"] = True
    _motor_event.set()

def motor_request_brake():
    _motor_ctrl_q.put(("brake", None))
    _motor_event.set()

def motor_sender_loop():
    """
    Single thread responsible for ALL motor transmissions.
    - Never queues old velocities: sends newest only.
    - Brake has priority.
    """
    last_sent = (None, None)
    last_send_ts = 0.0

    while rclpy.ok():
        _motor_event.wait(timeout=0.05)

        # 1) Drain control queue (priority actions)
        while True:
            try:
                cmd, _payload = _motor_ctrl_q.get_nowait()
            except Exception:
                break

            if cmd == "brake":
                try:
                    _session.post(MOTOR_BRAKE_URL, json={}, timeout=MOTOR_POST_TIMEOUT)
                    print("[motor] BRAKE (sent)")
                except Exception as e:
                    print(f"[motor-http] POST /brake failed: {e}")

                with _motor_lock:
                    _motor_latest["m1"] = 0.0
                    _motor_latest["m2"] = 0.0
                    _motor_latest["dirty"] = False

                last_sent = (0.0, 0.0)
                last_send_ts = time.time()

        # 2) Latest velocity send
        now = time.time()
        with _motor_lock:
            m1 = _motor_latest["m1"]
            m2 = _motor_latest["m2"]
            dirty = _motor_latest["dirty"]
            if dirty:
                _motor_latest["dirty"] = False

        keepalive = (now - last_send_ts) >= MOTOR_KEEPALIVE_S
        changed = (
            last_sent[0] is None
            or abs(m1 - last_sent[0]) >= VEL_EPS
            or abs(m2 - last_sent[1]) >= VEL_EPS
        )

        if dirty or (keepalive and changed):
            try:
                _session.post(MOTOR_JSON_URL, json={"m1": m1, "m2": m2}, timeout=MOTOR_POST_TIMEOUT)
                last_sent = (m1, m2)
                last_send_ts = now
            except Exception as e:
                print(f"[motor-http] POST /motor failed: {e}")

        _motor_event.clear()

# ------------------ Video helpers (RTSP) ------------------
def _open_rtsp():
    """
    Try FFMPEG backend first (same stack ffplay uses), then fall back
    to a GStreamer pipeline if FFMPEG is not available.
    """
    # Option 1: FFMPEG backend with low-latency flags
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
        "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;0"
    )
    cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
    if cap.isOpened():
        print("[video] opened via FFMPEG backend")
        return cap

    # Option 2: GStreamer pipeline fallback (latency=0, no jitter buffer delay)
    print("[video] FFMPEG backend failed, trying GStreamer pipeline...")
    gst = (
        f"rtspsrc location={RTSP_URL} latency=0 protocols=tcp ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
        "appsink drop=true sync=false max-buffers=1"
    )
    cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        print("[video] opened via GStreamer backend")
        return cap

    return None

def _put_text(img, text, org, scale=0.6, color=(255, 255, 255), thickness=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)

# ------------------ Joy callback ------------------
def _edge(prev, cur):
    return prev == 0 and cur == 1

def joy_cb(msg: Joy):
    now = time.time()

    with lock:
        state["joy_alive"] = True
        state["last_joy_ts"] = now

        if not state["prev_buttons"]:
            state["prev_buttons"] = [0] * len(msg.buttons)

        # Camera switch on X (edge-triggered, debounced)
        if 0 <= BTN_CAM_SWITCH < len(msg.buttons) and _edge(state["prev_buttons"][BTN_CAM_SWITCH], msg.buttons[BTN_CAM_SWITCH]):
            if (now - float(state.get("last_cam_switch_ts", 0.0))) >= CAM_SWITCH_DEBOUNCE_S:
                next_cam = 10 if state.get("active_cam", 0) == 0 else 0
                url = CAM10_URL if next_cam == 10 else CAM0_URL
                _post_async(url, timeout=HTTP_TIMEOUT)
                state["active_cam"] = next_cam
                state["last_cam_switch_ts"] = now
                print(f"[video] switch requested -> /dev/video{next_cam}")

        # Pressure bumps (edge-triggered)
        if 0 <= BTN_PRESSURE_UP < len(msg.buttons) and _edge(state["prev_buttons"][BTN_PRESSURE_UP], msg.buttons[BTN_PRESSURE_UP]):
            state["pressure"] = clamp(state["pressure"] + 0.05, PRESSURE_MIN, PRESSURE_MAX)
        if 0 <= BTN_PRESSURE_DN < len(msg.buttons) and _edge(state["prev_buttons"][BTN_PRESSURE_DN], msg.buttons[BTN_PRESSURE_DN]):
            state["pressure"] = clamp(state["pressure"] - 0.05, PRESSURE_MIN, PRESSURE_MAX)

        # LED control (edge-triggered)
        if 0 <= BTN_LED_ON < len(msg.buttons) and _edge(state["prev_buttons"][BTN_LED_ON], msg.buttons[BTN_LED_ON]):
            _post_async(LED_ON_URL)
            state["led_on"] = True
            print("[LED] ON")
        if 0 <= BTN_LED_OFF < len(msg.buttons) and _edge(state["prev_buttons"][BTN_LED_OFF], msg.buttons[BTN_LED_OFF]):
            _post_async(LED_OFF_URL)
            state["led_on"] = False
            print("[LED] OFF")

        # Brake motors (priority)
        if 0 <= BTN_BRAKE_ALL < len(msg.buttons) and _edge(state["prev_buttons"][BTN_BRAKE_ALL], msg.buttons[BTN_BRAKE_ALL]):
            motor_request_brake()
            state["last_vel_sent"] = (0.0, 0.0)
            print("[motor] BRAKE (requested)")

        # Motor 1 (variable/level-controlled)
        have_m1_axes = (0 <= AXIS_M1_FWD < len(msg.axes)) and (0 <= AXIS_M1_REV < len(msg.axes))

        if have_m1_axes:
            raw_fwd = float(msg.axes[AXIS_M1_FWD])
            raw_rev = float(msg.axes[AXIS_M1_REV])

            if raw_fwd < -0.05:
                state["m1_fwd_seen_negative"] = True
            if raw_rev < -0.05:
                state["m1_rev_seen_negative"] = True

            fwd = trigger_to_01(raw_fwd, state["m1_fwd_seen_negative"])
            rev = trigger_to_01(raw_rev, state["m1_rev_seen_negative"])
            a1 = fwd - rev
        else:
            b_fwd = msg.buttons[BTN_M1_FWD] if len(msg.buttons) > BTN_M1_FWD else 0
            b_rev = msg.buttons[BTN_M1_REV] if len(msg.buttons) > BTN_M1_REV else 0
            a1 = (1.0 if b_fwd else 0.0) - (1.0 if b_rev else 0.0)

        # HTTP Motor 2 axis (axes[4])
        a2 = msg.axes[AXIS_M2] if 0 <= AXIS_M2 < len(msg.axes) else 0.0

        # UART axis (axes[1])
        au = msg.axes[AXIS_UART] if 0 <= AXIS_UART < len(msg.axes) else 0.0

        # deadzone
        state["ax_m1"] = 0.0 if abs(a1) < MOTOR_DEADZONE else a1
        state["ax_m2"] = 0.0 if abs(a2) < MOTOR_DEADZONE else a2
        state["ax_uart"] = 0.0 if abs(au) < MOTOR_DEADZONE else au

        state["prev_buttons"] = list(msg.buttons)

# ------------------ Worker threads ------------------
def pressure_loop(uart: serial.Serial):
    rate_hz = 20.0
    dt = 1.0 / rate_hz
    last_sent = None
    while rclpy.ok():
        with lock:
            p = state["pressure"]
        if last_sent is None or abs(p - last_sent) >= 0.01:
            try:
                uart.write(f"{p:.3f}\n".encode())
                last_sent = p
            except Exception as e:
                print("[pressure] write failed:", e)
        time.sleep(dt)

def _send_motor_velocity(v1: float, v2: float):
    # Latest-only: hand off to motor sender thread
    motor_set_velocity(v1, v2)

def motor_http_loop():
    """Compute joystick velocities and publish them to motor sender (rate-limited + deadman)."""
    while rclpy.ok():
        now = time.time()

        with lock:
            a1 = float(state["ax_m1"])
            a2 = float(state["ax_m2"])  # HTTP motor2 comes from axes[4]
            last_joy = float(state["last_joy_ts"])
            last_sent_v1, last_sent_v2 = state["last_vel_sent"]
            last_post = float(state["last_vel_post_ts"])
            last_keepalive = float(state["last_keepalive_ts"])

        # sign convention
        v1 = clamp(-a1, -1.0, 1.0)
        v2 = clamp(a2, -1.0, 1.0)

        # deadman
        if last_joy == 0.0 or (now - last_joy) > JOY_DEADMAN_S:
            v1, v2 = 0.0, 0.0

        # rate limit compute loop
        if (now - last_post) < HTTP_VEL_INTERVAL:
            time.sleep(0.01)
            continue

        changed = (abs(v1 - last_sent_v1) >= VEL_EPS) or (abs(v2 - last_sent_v2) >= VEL_EPS)
        keepalive = (now - last_keepalive) >= VEL_KEEPALIVE_S

        if changed or keepalive:
            _send_motor_velocity(v1, v2)
            with lock:
                state["last_vel_sent"] = (v1, v2)
                state["last_vel_post_ts"] = now
                if keepalive:
                    state["last_keepalive_ts"] = now
        else:
            with lock:
                state["last_vel_post_ts"] = now

        time.sleep(0.01)

# ----- UART output (axes[1] -> -255..255 on /dev/ttyUSB0) -----
def axis_to_pwm255(a: float) -> int:
    a = clamp(float(a), -1.0, 1.0)
    return int(round(a * 255.0))

def uart_m2_loop(uart: serial.Serial):
    dt = 1.0 / float(M2_UART_RATE_HZ)
    last_sent = None

    while rclpy.ok():
        now = time.time()
        with lock:
            au = float(state["ax_uart"])  # UART comes from axes[1]
            last_joy = float(state["last_joy_ts"])

        # deadman: joystick stale => command 0
        if last_joy == 0.0 or (now - last_joy) > JOY_DEADMAN_S:
            val = 0
        else:
            # If you want sign flipped, change to axis_to_pwm255(-au)
            val = axis_to_pwm255(au)

        if last_sent is None or abs(val - last_sent) >= M2_UART_EPS:
            try:
                uart.write(f"{val}\n".encode())
                last_sent = val
            except Exception as e:
                print("[m2-uart] write failed:", e)
                time.sleep(0.5)

        time.sleep(dt)

# ------------------ Terminal command loop ------------------
def terminal_cmd_loop():
    print("\n[terminal] Ready. Examples:")
    print("  ilimit1 0.35   | ilimit2 0.25")
    print("  istall1 0.80   | istall2 0.60")
    print("  brakeall       | health")
    print("  quit / exit\n")

    while rclpy.ok():
        try:
            line = input("[motor-cmd] > ").strip()
        except (EOFError, KeyboardInterrupt):
            try:
                rclpy.shutdown()
            except Exception:
                pass
            return

        if not line:
            continue

        cmd = line.strip().lower()
        if cmd in ("q", "quit", "exit"):
            try:
                rclpy.shutdown()
            except Exception:
                pass
            return

        if cmd in ("health", "status"):
            try:
                r = _session.get(MOTOR_HEALTH, timeout=HTTP_TIMEOUT)
                print(f"[health] {r.status_code}: {r.text}")
            except Exception as e:
                print("[health] failed:", e)
            continue

        if cmd in ("brakeall", "brake"):
            motor_request_brake()
            print("[brake] requested")
            continue

        # Forward everything else to POST /cmd
        try:
            r = _session.post(MOTOR_CMD_URL, json={"cmd": line}, timeout=HTTP_TIMEOUT)
            print(f"[cmd] {r.status_code}: {r.text}")
        except Exception as e:
            print("[cmd] failed:", e)

def health_poll_loop():
    while rclpy.ok():
        ok = None
        summary = ""
        try:
            js = _get_json(MOTOR_HEALTH)
            ok = True
            if isinstance(js, dict) and "motors" in js:
                m = js.get("motors", {})
                m1 = m.get("m1", {})
                m2 = m.get("m2", {})
                summary = (
                    f"m1_fault={m1.get('fault')} m2_fault={m2.get('fault')} "
                    f"m1_vel={m1.get('vel_cmd')} m2_vel={m2.get('vel_cmd')}"
                )
            else:
                summary = str(js)
        except Exception as e:
            ok = False
            summary = str(e)

        with lock:
            state["health_ok"] = ok
            state["health_summary"] = summary

        time.sleep(0.5)

def video_loop():
    print(f"[video] Opening RTSP stream {RTSP_URL}")
    cap = _open_rtsp()
    if cap is None or not cap.isOpened():
        print(f"[video] cannot open RTSP stream {RTSP_URL}")
        return

    win = "RTSP + Joystick Base Station (ROS2)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)

    last_ts = time.time()
    frames = 0
    fps = 0.0

    while rclpy.ok():
        ok, frame = cap.read()
        alpha = 1.0
        beta = 0
        if ok and frame is not None:
            frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)

        if not ok or frame is None:     
            print("[video] read failed, reconnecting...")
            cap.release()
            time.sleep(RECONNECT_DELAY_S)
            cap = _open_rtsp()
            if cap is None or not cap.isOpened():
                print("[video] reconnect failed, retrying...")
                continue
            print("[video] reconnected")
            continue

        frames += 1
        now = time.time()
        if now - last_ts >= 1.0:
            fps = frames / (now - last_ts)
            last_ts = now
            frames = 0

        with lock:
            p = state["pressure"]
            joy_ok = state["joy_alive"]
            led_on = state["led_on"]
            v1s, v2s = state["last_vel_sent"]
            h_ok = state["health_ok"]
            h_sum = state["health_summary"]
            cam = state.get("active_cam", 0)
            joy_age = (now - state["last_joy_ts"]) if state["last_joy_ts"] else 999.0

        _put_text(frame, f"FPS: {fps:4.1f}", (10, 24))
        _put_text(frame, f"Pressure: {p:.2f} bar", (10, 48))
        _put_text(frame, f"Last vel req:  m1 {v1s:+.2f}  m2 {v2s:+.2f}", (10, 72))
        _put_text(frame, f"LED: {'ON' if led_on else 'OFF'}   Joy age: {joy_age:.2f}s", (10, 96))
        _put_text(frame, f"Cam: /dev/video{cam}   (X toggles cam)", (10, 120))

        if h_ok is None:
            _put_text(frame, "Radxa /health: (no data yet)", (10, 144))
        elif h_ok:
            _put_text(frame, f"Radxa /health: OK {h_sum}", (10, 144))
        else:
            _put_text(frame, f"Radxa /health: ERR {h_sum}", (10, 144))

        _put_text(frame, "LB: LED ON  RB: LED OFF  A: BRAKE  X: CAM SW  (Q to quit video)", (10, 168))
        _put_text(frame, f"JOY: {'OK' if joy_ok else 'WAITING'}", (10, 192))

        cv2.imshow(win, frame)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

# ------------------ Main ------------------
def main():
    rclpy.init()

    node = Node("base_station_joy")
    node.create_subscription(Joy, "/joy", joy_cb, 10)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Open pressure UART (optional)
    uart = None
    # try:
    #     uart = serial.Serial(PRESSURE_UART_PATH, PRESSURE_BAUD, timeout=1)
    #     print("[pressure] opened", uart.port)
    # except Exception as e:
    #     print("[pressure] open failed:", e)

    if uart:
        threading.Thread(target=pressure_loop, args=(uart,), daemon=True).start()

    # Open UART motor output (axes[1] -> -255..255)
    m2_uart = None
    try:
        m2_uart = serial.Serial(M2_UART_PATH, M2_UART_BAUD, timeout=0)
        print("[m2-uart] opened", m2_uart.port)
    except Exception as e:
        print("[m2-uart] open failed:", e)

    if m2_uart:
        threading.Thread(target=uart_m2_loop, args=(m2_uart,), daemon=True).start()

    # Motor threads (IMPORTANT: sender must exist)
    threading.Thread(target=motor_sender_loop, daemon=True).start()
    threading.Thread(target=motor_http_loop, daemon=True).start()

    threading.Thread(target=health_poll_loop, daemon=True).start()
    threading.Thread(target=terminal_cmd_loop, daemon=True).start()

    try:
        video_loop()
    finally:
        # Brake on exit (best effort)
        try:
            motor_request_brake()
            time.sleep(0.05)
        except Exception:
            pass

        if uart:
            try:
                uart.close()
            except Exception:
                pass

        if m2_uart:
            try:
                m2_uart.close()
            except Exception:
                pass

        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

        try:
            _http_exec.shutdown(wait=False, cancel_futures=True)
        except Exception:
            pass

if __name__ == "__main__":
    main()
