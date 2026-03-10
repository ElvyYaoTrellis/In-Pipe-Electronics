#!/usr/bin/env python3
"""host_modified_ros2.py

ROS 2 Humble version of your base-station host.

What it does:
- Subscribes to /joy (sensor_msgs/msg/Joy) published by ROS2 joy_node
- Streams RTSP video from the Radxa (OpenCV)
- Drives the local TMotor over UART (velocity control)
- Sends *network motor* commands to the Radxa 2Dac2Motor HTTP server

Radxa 2Dac2Motor HTTP API (port 8000):
  GET  /health
  POST /cmd   {"cmd":"m1 2000"}
  POST /motor {"m1":2000, "m2":-500}
  POST /brake {} or {"motor":1}

Notes vs ROS1:
- No rospy/roscore; this uses rclpy and requires ros-humble-joy for joy_node.
- The executor runs in a background thread so the main thread can show OpenCV UI.
"""

import threading
import time

import cv2
import requests
import serial

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Joy
from tmotor_serial_control.servo_serial import TMotorManager

# ------------------ Config ------------------
# Stable serial paths (your mappings)
PRESSURE_UART_PATH = "/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_02857388-if00-port0"  # pressure
MOTOR_UART_PATH    = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"  # TMotor

PRESSURE_BAUD = 115200
MOTOR_BAUD    = 961200   # change to 921600 if your motor FW expects it

# TMotor velocity range (rad/s)
v_max = 55.0
v_min = -55.0

# Motor params (AK80-64)
Servo_Params_Serial = {
    'AK80-64': {
        'Type': 'AK80-64',
        'P_min': -12.5,
        'P_max': 12.5,
        'V_min': v_min - 1,
        'V_max': v_max + 1,
        'Curr_min': -15.0,
        'Curr_max': 15.0,
        'Temp_max': 40.0,
        'Kt': 0.115,
        'GEAR_RATIO': 80,
        'NUM_POLE_PAIRS': 21,
    }
}

# Radxa endpoints (video + HTTP control)
RADXA_IP  = "192.168.8.232"
RTSP_PORT = 8554
RTSP_PATH = "/stream"

MOTOR_HTTP_PT = 8000   # 2Dac2Motor server
LED_HTTP_PT   = 8080

RTSP_URL        = f"rtsp://{RADXA_IP}:{RTSP_PORT}{RTSP_PATH}"
MOTOR_HEALTH    = f"http://{RADXA_IP}:{MOTOR_HTTP_PT}/health"
MOTOR_CMD_URL   = f"http://{RADXA_IP}:{MOTOR_HTTP_PT}/cmd"
MOTOR_JSON_URL  = f"http://{RADXA_IP}:{MOTOR_HTTP_PT}/motor"
MOTOR_BRAKE_URL = f"http://{RADXA_IP}:{MOTOR_HTTP_PT}/brake"

LED_ON_URL  = f"http://{RADXA_IP}:{LED_HTTP_PT}/on"
LED_OFF_URL = f"http://{RADXA_IP}:{LED_HTTP_PT}/off"

HTTP_TIMEOUT = (0.2, 0.5)   # (connect, read)
RECONNECT_DELAY_S = 1.5

# ------------------ Joystick mapping ------------------
# NOTE: mappings depend on your controller/joy_node; adjust as needed.
AXIS_TMOTOR_VEL  = 3   # right stick horizontal → TMotor velocity
BTN_PRESSURE_UP  = 3   # Y button → pressure +0.05
BTN_PRESSURE_DN  = 1   # X button → pressure -0.05

# These two axes used to control servos; now they command the Radxa 2Dac2Motor motors.
AXIS_M1 = 0            # left stick horizontal   → motor1 (incremental counts)
AXIS_M2 = 1            # left stick vertical     → motor2 (incremental counts)

BTN_LED_ON  = 4        # LB → LED ON
BTN_LED_OFF = 5        # RB → LED OFF

BTN_BRAKE_ALL = 0      # A → brake both motors (POST /brake {})

# ------------------ Network motor incremental control ------------------
MOTOR_DEADZONE = 0.12

# Full-stick rate (counts / second). Tune to taste.
M1_RATE_COUNTS_PER_SEC = 12000.0
M2_RATE_COUNTS_PER_SEC = 12000.0

# How often to POST accumulated counts to /motor
HTTP_MOTOR_MIN_INTERVAL = 0.05  # 20 Hz

# If True, use POST /cmd with "m1 N" / "m2 N" ("servo-server-like")
# instead of POST /motor JSON.
USE_TEXT_CMD_ENDPOINT = True

# ------------------ Pressure range ------------------
PRESSURE_MIN = 0.0
PRESSURE_MAX = 7.0

# ------------------ Shared state ------------------
def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

lock = threading.Lock()
state = {
    "tmotor_vel": 0.0,
    "pressure": 0.0,

    # joystick axes (after deadzone)
    "ax_m1": 0.0,
    "ax_m2": 0.0,

    # pending *incremental* counts to send to Radxa
    "m1_pending": 0,
    "m2_pending": 0,

    "last_motor_post_ts": 0.0,
    "last_motor_sent": (0, 0),

    "joy_alive": False,
    "led_on": False,
    "prev_buttons": [],

    # health polling
    "health_ok": None,
    "health_summary": "",
}

# ------------------ HTTP helpers ------------------
_session = requests.Session()

def _post_async(url, json=None, timeout=HTTP_TIMEOUT):
    def _do():
        try:
            _session.post(url, json=json, timeout=timeout)
        except Exception as e:
            print(f"[http] POST {url} failed: {e}")
    threading.Thread(target=_do, daemon=True).start()

def _get_json(url, timeout=HTTP_TIMEOUT):
    r = _session.get(url, timeout=timeout)
    r.raise_for_status()
    return r.json()

# ------------------ Video helpers ------------------
def _open_rtsp(url: str):
    cap = cv2.VideoCapture(url)
    if cap.isOpened():
        return cap
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    if cap.isOpened():
        return cap
    gst = f"rtspsrc location={url} latency=0 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink"
    cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        return cap
    return None

def _put_text(img, text, org, scale=0.6, color=(255, 255, 255), thickness=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)

# ------------------ Joy callback ------------------
def _edge(prev, cur):
    return prev == 0 and cur == 1

def joy_cb(msg: Joy):
    with lock:
        state["joy_alive"] = True

        if not state["prev_buttons"]:
            state["prev_buttons"] = [0] * len(msg.buttons)

        # TMotor velocity from axis (scaled to [v_min, v_max])
        raw = msg.axes[AXIS_TMOTOR_VEL] if 0 <= AXIS_TMOTOR_VEL < len(msg.axes) else 0.0
        # robust mapping even if v_min != -v_max
        t = (raw + 1.0) * 0.5  # [-1,1] -> [0,1]
        state["tmotor_vel"] = (1.0 - t) * v_min + t * v_max

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

        # Brake both Radxa motors
        if 0 <= BTN_BRAKE_ALL < len(msg.buttons) and _edge(state["prev_buttons"][BTN_BRAKE_ALL], msg.buttons[BTN_BRAKE_ALL]):
            _post_async(MOTOR_BRAKE_URL, json={})
            state["m1_pending"] = 0
            state["m2_pending"] = 0
            print("[motor] BRAKE")

        # Motor 1 now on shoulder buttons (6/7) per your last change
        # buttons[7] -> increase, buttons[6] -> decrease
        a1 = (1.0 if msg.buttons[6] else 0.0) - (1.0 if msg.buttons[7] else 0.0)

        a2 = msg.axes[AXIS_M2] if 0 <= AXIS_M2 < len(msg.axes) else 0.0
        state["ax_m1"] = 0.0 if abs(a1) < MOTOR_DEADZONE else a1
        state["ax_m2"] = 0.0 if abs(a2) < MOTOR_DEADZONE else a2

        state["prev_buttons"] = list(msg.buttons)

# ------------------ Worker threads ------------------
def tmotor_loop(motor: TMotorManager):
    # ~100 Hz
    while rclpy.ok():
        with lock:
            vel = state["tmotor_vel"]
        try:
            motor.set_output_velocity_radians_per_second(vel)
            motor.update()
        except Exception as e:
            print("[tmotor] update failed:", e)
            time.sleep(0.05)
        time.sleep(0.01)


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


def motor_integrator_loop():
    """Accumulate incremental counts based on joystick axis deflection."""
    last = time.time()
    while rclpy.ok():
        now = time.time()
        dt = now - last
        last = now

        with lock:
            a1, a2 = state["ax_m1"], state["ax_m2"]

        # Match old directionality: a1 positive -> decreasing
        dm1 = int(round((-a1) * M1_RATE_COUNTS_PER_SEC * dt))
        dm2 = int(round(( a2) * M2_RATE_COUNTS_PER_SEC * dt))

        if dm1 != 0 or dm2 != 0:
            with lock:
                state["m1_pending"] += dm1
                state["m2_pending"] += dm2

        time.sleep(0.01)  # ~100 Hz


def _send_motor_counts(m1: int, m2: int):
    if USE_TEXT_CMD_ENDPOINT:
        if m1 != 0:
            _post_async(MOTOR_CMD_URL, json={"cmd": f"m1 {m1}"})
        if m2 != 0:
            _post_async(MOTOR_CMD_URL, json={"cmd": f"m2 {m2}"})
    else:
        payload = {}
        if m1 != 0:
            payload["m1"] = m1
        if m2 != 0:
            payload["m2"] = m2
        if payload:
            _post_async(MOTOR_JSON_URL, json=payload)


def motor_http_loop():
    """POST pending counts to the Radxa at a limited rate."""
    while rclpy.ok():
        now = time.time()
        m1 = m2 = 0
        do_send = False

        with lock:
            if (now - state["last_motor_post_ts"]) >= HTTP_MOTOR_MIN_INTERVAL:
                if state["m1_pending"] != 0 or state["m2_pending"] != 0:
                    m1 = state["m1_pending"]
                    m2 = state["m2_pending"]
                    state["m1_pending"] = 0
                    state["m2_pending"] = 0
                    state["last_motor_post_ts"] = now
                    state["last_motor_sent"] = (m1, m2)
                    do_send = True

        if do_send:
            _send_motor_counts(m1, m2)

        time.sleep(0.01)


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
            try:
                r = _session.post(MOTOR_BRAKE_URL, json={}, timeout=HTTP_TIMEOUT)
                print(f"[brake] {r.status_code}: {r.text}")
            except Exception as e:
                print("[brake] failed:", e)
            continue

        # Forward everything else to POST /cmd
        try:
            r = _session.post(MOTOR_CMD_URL, json={"cmd": line}, timeout=HTTP_TIMEOUT)
            print(f"[cmd] {r.status_code}: {r.text}")
        except Exception as e:
            print("[cmd] failed:", e)


def health_poll_loop():
    """Poll /health periodically for on-screen diagnostics."""
    while rclpy.ok():
        ok = None
        summary = ""
        try:
            js = _get_json(MOTOR_HEALTH)
            ok = True

            if isinstance(js, dict):
                faults = js.get("faults") or js.get("fault") or js.get("fault_flags")
                rc = js.get("rc") or js.get("rc_counts")
                status = js.get("status") or js.get("state") or js.get("ok")

                parts = []
                if status is not None:
                    parts.append(f"status={status}")
                if faults is not None:
                    parts.append(f"faults={faults}")
                if rc is not None:
                    parts.append(f"rc={rc}")
                summary = " ".join(parts)
            else:
                summary = str(js)
        except Exception as e:
            ok = False
            summary = str(e)

        with lock:
            state["health_ok"] = ok
            state["health_summary"] = summary

        time.sleep(0.5)  # 2 Hz


def video_loop():
    print("[video] Opening:", RTSP_URL)
    cap = _open_rtsp(RTSP_URL)
    if cap is None or not cap.isOpened():
        print(f"[video] cannot open {RTSP_URL}")
        return

    last_ts = time.time(); frames = 0; fps = 0.0
    while rclpy.ok():
        ok, frame = cap.read()
        if not ok or frame is None:
            print("[video] read failed, reconnecting...")
            cap.release()
            time.sleep(RECONNECT_DELAY_S)
            cap = _open_rtsp(RTSP_URL)
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
            v = state["tmotor_vel"]
            joy_ok = state["joy_alive"]
            led_on = state["led_on"]
            m1p, m2p = state["m1_pending"], state["m2_pending"]
            m1s, m2s = state["last_motor_sent"]
            h_ok = state["health_ok"]
            h_sum = state["health_summary"]

        _put_text(frame, f"FPS: {fps:4.1f}", (10, 24))
        _put_text(frame, f"Pressure: {p:.2f} bar   TMotor vel: {v:6.2f} rad/s", (10, 48))
        _put_text(frame, f"Radxa m1_pending: {m1p:+6d}  m2_pending: {m2p:+6d}", (10, 72))
        _put_text(frame, f"Last sent: m1 {m1s:+d}  m2 {m2s:+d}   LED: {'ON' if led_on else 'OFF'}", (10, 96))

        if h_ok is None:
            _put_text(frame, "Radxa /health: (no data yet)", (10, 120))
        elif h_ok:
            _put_text(frame, f"Radxa /health: OK {h_sum}", (10, 120))
        else:
            _put_text(frame, f"Radxa /health: ERR {h_sum}", (10, 120))

        _put_text(frame, "LB: LED ON  RB: LED OFF  A: BRAKE  (Q to quit video)", (10, 144))
        _put_text(frame, f"JOY: {'OK' if joy_ok else 'WAITING'}", (10, 168))

        cv2.imshow("RTSP + Joystick Base Station (ROS2)", frame)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
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

    # Local TMotor
    motor = TMotorManager(
        port=MOTOR_UART_PATH,
        baud=MOTOR_BAUD,
        motor_params=Servo_Params_Serial['AK80-64'],
        max_mosfett_temp=80,
    )
    motor.__enter__()
    motor.enter_velocity_control()
    motor.set_zero_position()
    motor.update()

    # Threads
    threading.Thread(target=tmotor_loop, args=(motor,), daemon=True).start()
    if uart:
        threading.Thread(target=pressure_loop, args=(uart,), daemon=True).start()

    threading.Thread(target=motor_integrator_loop, daemon=True).start()
    threading.Thread(target=motor_http_loop, daemon=True).start()
    threading.Thread(target=health_poll_loop, daemon=True).start()
    threading.Thread(target=terminal_cmd_loop, daemon=True).start()

    # Video loop blocks until quit (Q)
    try:
        video_loop()
    finally:
        # Stop local TMotor
        try:
            motor.set_output_velocity_radians_per_second(0.0)
            motor.update()
            motor.__exit__(None, None, None)
        except Exception:
            pass

        # Brake Radxa motors on exit
        try:
            _session.post(MOTOR_BRAKE_URL, json={}, timeout=HTTP_TIMEOUT)
        except Exception:
            pass

        if uart:
            try:
                uart.close()
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


if __name__ == "__main__":
    main()
