import sys
import os
import cv2
import math
import json
import threading
import time

os.environ["QT_API"] = "pyside6"

import pygame
import requests

from PySide6.QtWidgets import (
    QApplication, QLabel, QWidget, QGridLayout, QStackedLayout,
    QPushButton, QVBoxLayout, QHBoxLayout, QMainWindow, QGroupBox,
)
from PySide6.QtCore import QTimer, Qt, Signal, QEasingCurve, QPropertyAnimation, QRect
from PySide6.QtGui import QImage, QPixmap, QSurfaceFormat, QColor

from videolabel import VideoLabel
from glstldisplay import GLSTLDisplay
from matplotlib_3d_plot import Matplotlib3DPlot
from globals import axes_lock, axes, MOTOR_URL, LED_URL, CAM_URL, VIDEO_URL, MAX_VEL


# ─────────────────────────────────────────────
# Status sidebar
# ─────────────────────────────────────────────

def _make_group(title):
    box = QGroupBox(title)
    box.setStyleSheet("""
        QGroupBox {
            color: #aaaaaa;
            font-size: 9pt;
            border: 1px solid #444;
            border-radius: 4px;
            margin-top: 6px;
            padding-top: 4px;
        }
        QGroupBox::title { subcontrol-origin: margin; left: 6px; }
    """)
    layout = QVBoxLayout(box)
    layout.setSpacing(3)
    layout.setContentsMargins(6, 8, 6, 6)
    return box, layout


def _label(text="—"):
    lbl = QLabel(text)
    lbl.setStyleSheet("color: #dddddd; font-size: 9pt;")
    return lbl


def _status_dot(color="#888888"):
    dot = QLabel("●")
    dot.setStyleSheet(f"color: {color}; font-size: 10pt;")
    dot.setFixedWidth(16)
    return dot


class StatusPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(190)
        self.setStyleSheet("background: #1a1a1a;")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        title = QLabel("STATUS")
        title.setStyleSheet("color: #888; font-size: 8pt; font-weight: bold; letter-spacing: 2px;")
        layout.addWidget(title)

        # ── Connection ──────────────────────────
        conn_box, conn_l = _make_group("Connection")
        conn_row = QHBoxLayout()
        self.conn_dot = _status_dot()
        self.conn_label = _label("Disconnected")
        conn_row.addWidget(self.conn_dot)
        conn_row.addWidget(self.conn_label)
        conn_row.addStretch()
        conn_l.addLayout(conn_row)
        layout.addWidget(conn_box)

        # ── Motors ──────────────────────────────
        motor_box, motor_l = _make_group("Motors")
        self.m1_label = _label("M1 cmd:  —")
        self.m2_label = _label("M2 cmd:  —")
        self.m1_current = _label("M1 curr: —")
        self.m2_current = _label("M2 curr: —")
        self.motor_fault = _label("")
        self.motor_fault.setStyleSheet("color: #ff6b6b; font-size: 9pt;")
        for w in (self.m1_label, self.m2_label, self.m1_current, self.m2_current, self.motor_fault):
            motor_l.addWidget(w)
        layout.addWidget(motor_box)

        # ── Camera ──────────────────────────────
        cam_box, cam_l = _make_group("Camera")
        self.cam_active = _label("Active: —")
        self.cam_fps    = _label("FPS:    —")
        cam_l.addWidget(self.cam_active)
        cam_l.addWidget(self.cam_fps)
        btn_row = QHBoxLayout()
        mipi_btn = QPushButton("MIPI")
        usb_btn  = QPushButton("USB")
        for btn in (mipi_btn, usb_btn):
            btn.setFixedHeight(22)
            btn.setStyleSheet(
                "QPushButton { background:#333; color:#ccc; border:1px solid #555; border-radius:3px; font-size:8pt; }"
                "QPushButton:hover { background:#444; }"
            )
        mipi_btn.clicked.connect(lambda: self._switch_cam(0))
        usb_btn.clicked.connect(lambda: self._switch_cam(10))
        btn_row.addWidget(mipi_btn)
        btn_row.addWidget(usb_btn)
        cam_l.addLayout(btn_row)
        layout.addWidget(cam_box)

        # ── LED ─────────────────────────────────
        led_box, led_l = _make_group("LED")
        self.led_label = _label("—")
        led_l.addWidget(self.led_label)
        led_btn_row = QHBoxLayout()
        on_btn  = QPushButton("ON")
        off_btn = QPushButton("OFF")
        for btn in (on_btn, off_btn):
            btn.setFixedHeight(22)
            btn.setStyleSheet(
                "QPushButton { background:#333; color:#ccc; border:1px solid #555; border-radius:3px; font-size:8pt; }"
                "QPushButton:hover { background:#444; }"
            )
        on_btn.clicked.connect(self._led_on)
        off_btn.clicked.connect(self._led_off)
        led_btn_row.addWidget(on_btn)
        led_btn_row.addWidget(off_btn)
        led_l.addLayout(led_btn_row)
        layout.addWidget(led_box)

        # ── System ──────────────────────────────
        sys_box, sys_l = _make_group("System")
        self.battery_label = _label("Battery: —")
        self.current_label = _label("Current: —")
        self.uptime_label  = _label("Uptime:  —")
        self.joy_label     = _label("Joystick: —")
        for w in (self.battery_label, self.current_label, self.uptime_label, self.joy_label):
            sys_l.addWidget(w)
        layout.addWidget(sys_box)

        layout.addStretch()

        self._start_time = time.monotonic()

    # ── LED controls ────────────────────────────
    def _led_on(self):
        threading.Thread(
            target=lambda: requests.post(f"{LED_URL}/on", timeout=1),
            daemon=True,
        ).start()
        self.led_label.setText("ON  ●")
        self.led_label.setStyleSheet("color: #ffdd57; font-size: 9pt;")

    def _led_off(self):
        threading.Thread(
            target=lambda: requests.post(f"{LED_URL}/off", timeout=1),
            daemon=True,
        ).start()
        self.led_label.setText("OFF")
        self.led_label.setStyleSheet("color: #dddddd; font-size: 9pt;")

    # ── Camera switch ────────────────────────────
    def _switch_cam(self, idx):
        path = "/cam/0" if idx == 0 else "/cam/10"
        threading.Thread(
            target=lambda: requests.post(f"{CAM_URL}{path}", timeout=1),
            daemon=True,
        ).start()

    # ── Called from status poll thread (via Qt signal) ──
    def update_status(self, status: dict):
        connected = bool(status)
        if connected:
            self.conn_dot.setStyleSheet("color: #51cf66; font-size: 10pt;")
            self.conn_label.setText("Connected")
        else:
            self.conn_dot.setStyleSheet("color: #ff6b6b; font-size: 10pt;")
            self.conn_label.setText("Disconnected")

        # Motor
        if "motor" in status:
            m = status["motor"]
            motors = m.get("motors", {})
            m1 = motors.get("m1", {})
            m2 = motors.get("m2", {})
            self.m1_label.setText(f"M1 cmd:  {m1.get('vel_cmd', 0.0):+.0f}")
            self.m2_label.setText(f"M2 cmd:  {m2.get('vel_cmd', 0.0):+.0f}")

            ina = m.get("ina", {})
            first_ina = next(iter(ina.values()), {}) if ina else {}
            bus_v   = first_ina.get("bus_v")
            curr_a  = first_ina.get("current_a")
            if bus_v is not None:
                self.battery_label.setText(f"Battery: {bus_v:.1f} V")
            if curr_a is not None:
                self.current_label.setText(f"Current: {curr_a:.2f} A")

            faults = []
            for n, md in motors.items():
                flags = md.get("fault_flags", [])
                if flags:
                    faults.append(f"{n}: {', '.join(flags)}")
            self.motor_fault.setText("\n".join(faults))

        # Camera
        if "camera" in status:
            c = status["camera"]
            active = c.get("active", "—")
            name = "MIPI (cam0)" if active == 0 else "USB (cam10)" if active == 10 else str(active)
            self.cam_active.setText(f"Active: {name}")

        # Uptime (local, since Radxa doesn't expose it)
        elapsed = int(time.monotonic() - self._start_time)
        h, rem = divmod(elapsed, 3600)
        m_, s = divmod(rem, 60)
        self.uptime_label.setText(f"Uptime:  {h:02d}:{m_:02d}:{s:02d}")

    def update_joystick(self, connected: bool, name: str = ""):
        if connected:
            self.joy_label.setText(f"Joy: {name[:18]}")
            self.joy_label.setStyleSheet("color: #51cf66; font-size: 9pt;")
        else:
            self.joy_label.setText("Joy: none")
            self.joy_label.setStyleSheet("color: #888; font-size: 9pt;")

    def update_video_fps(self, fps: float):
        self.cam_fps.setText(f"FPS:    {fps:.1f}")


# ─────────────────────────────────────────────
# Background threads
# ─────────────────────────────────────────────

def joystick_thread(stop_event, status_panel_ref):
    pygame.init()
    pygame.joystick.init()

    joystick = None
    last_cmd = 0.0
    CMD_HZ = 20
    CMD_INTERVAL = 1.0 / CMD_HZ

    while not stop_event.is_set():
        # (Re)initialize joystick
        pygame.event.pump()
        count = pygame.joystick.get_count()
        if joystick is None and count > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            status_panel_ref[0].update_joystick(True, joystick.get_name())
        elif joystick is not None and count == 0:
            joystick = None
            status_panel_ref[0].update_joystick(False)

        if joystick is None:
            time.sleep(0.5)
            continue

        # Read axes
        n = joystick.get_numaxes()
        new_axes = [joystick.get_axis(i) for i in range(min(6, n))]
        with axes_lock:
            for i, v in enumerate(new_axes):
                axes[i] = v

        # Send motor commands at CMD_HZ
        now = time.monotonic()
        if now - last_cmd >= CMD_INTERVAL:
            # Left stick Y (axis 1) → m1, Right stick Y (axis 4) → m2
            # Negated: push stick forward = positive velocity
            m1 = -new_axes[1] * MAX_VEL if len(new_axes) > 1 else 0.0
            m2 = -new_axes[4] * MAX_VEL if len(new_axes) > 4 else 0.0
            try:
                requests.post(
                    f"{MOTOR_URL}/motor",
                    json={"m1": round(m1, 1), "m2": round(m2, 1), "timeout_s": 0.3},
                    timeout=0.1,
                )
            except requests.RequestException:
                pass
            last_cmd = now

        time.sleep(0.01)


def status_poll_thread(stop_event, signal_emit):
    while not stop_event.is_set():
        status = {}
        try:
            r = requests.get(f"{MOTOR_URL}/health", timeout=0.5)
            if r.ok:
                status["motor"] = r.json()
        except Exception:
            pass

        try:
            r = requests.get(f"{CAM_URL}/status", timeout=0.5)
            if r.ok:
                status["camera"] = r.json()
        except Exception:
            pass

        signal_emit(status)
        time.sleep(0.5)


# ─────────────────────────────────────────────
# Main window
# ─────────────────────────────────────────────

class MainWindow(QMainWindow):
    telemetry_updated = Signal(float, float, float, float)
    imu_label_updated = Signal(str)
    status_updated    = Signal(dict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("In-Pipe Robot — Operator Dashboard")
        self.setStyleSheet("background-color: #111111;")
        self.menu_visible = False

        # ── Widgets ──────────────────────────────
        self.image_label = VideoLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setFixedSize(320, 240)

        self.imu_label = QLabel("Waiting for IMU...")
        self.imu_label.setAlignment(Qt.AlignCenter)
        self.imu_label.setFixedSize(320, 240)
        self.imu_label.setStyleSheet("background:#222; border:1px solid #444; color:#ccc; font-size:9pt;")

        self.gl_display = GLSTLDisplay("CameraMountSmallBoard.stl")
        self.gl_display.setFixedSize(320, 240)

        self.localization_label = Matplotlib3DPlot()
        self.localization_label.setFixedSize(320, 240)

        self.status_panel = StatusPanel()
        self._status_panel_ref = [self.status_panel]  # mutable ref for thread

        # ── Menu button ──────────────────────────
        self.menu_button = QPushButton("☰")
        self.menu_button.setStyleSheet("color:#ccc; background:#333; border:none; font-size:14pt;")
        self.menu_button.setFixedSize(40, 40)

        # ── Layout ───────────────────────────────
        grid = QGridLayout()
        grid.setSpacing(4)
        grid.addWidget(self.image_label,       0, 0)
        grid.addWidget(self.imu_label,         0, 1)
        grid.addWidget(self.gl_display,        1, 1, 1, 2)
        grid.addWidget(self.localization_label, 1, 0)

        grid_widget = QWidget()
        grid_widget.setLayout(grid)

        self.stack = QStackedLayout()
        self.stack.addWidget(grid_widget)
        self._grid_widget = grid_widget

        center = QWidget()
        center_layout = QVBoxLayout(center)
        center_layout.setContentsMargins(0, 0, 0, 0)
        center_layout.addWidget(self.menu_button, alignment=Qt.AlignLeft)
        center_layout.addLayout(self.stack)

        root = QWidget()
        root_layout = QHBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)
        root_layout.addWidget(center, stretch=1)
        root_layout.addWidget(self.status_panel)
        self.setCentralWidget(root)

        # ── Video ─────────────────────────────────
        self.cap = cv2.VideoCapture(VIDEO_URL)
        self._frame_count = 0
        self._fps_t0 = time.monotonic()
        self._video_fps = 0.0

        self.video_timer = QTimer()
        self.video_timer.timeout.connect(self._update_frame)
        self.video_timer.start(16)  # ~60 Hz

        # ── IMU SSE stream ────────────────────────
        self._stop = threading.Event()
        threading.Thread(target=self._read_imu_sse, daemon=True).start()
        self.telemetry_updated.connect(self.gl_display.set_rotation)
        self.imu_label_updated.connect(self.imu_label.setText)

        # ── Status poll ───────────────────────────
        self.status_updated.connect(self.status_panel.update_status)
        threading.Thread(
            target=status_poll_thread,
            args=(self._stop, self.status_updated.emit),
            daemon=True,
        ).start()

        # ── Joystick ─────────────────────────────
        threading.Thread(
            target=joystick_thread,
            args=(self._stop, self._status_panel_ref),
            daemon=True,
        ).start()

        # ── Click-to-focus ────────────────────────
        self.image_label.clicked.connect(lambda: self._focus(self.image_label))
        self.gl_display.clicked.connect(lambda: self._focus(self.gl_display))
        self.localization_label.clicked.connect(lambda: self._focus(self.localization_label))

        # ── Sliding menu (resolution) ─────────────
        self._overlay = QWidget(self)
        self._overlay.setStyleSheet("background: rgba(0,0,0,120);")
        self._overlay.setVisible(False)
        self._overlay.mousePressEvent = lambda _: self._toggle_menu()

        self._menu = QWidget(self)
        self._menu.setStyleSheet("background: #2a2a2a;")
        self._menu.setFixedWidth(200)
        m_layout = QVBoxLayout(self._menu)
        lbl = QLabel("Video Resolution")
        lbl.setStyleSheet("color:#ccc; font-size:9pt;")
        m_layout.addWidget(lbl)
        for label, w, h in [("Low (320×240)", 320, 240), ("VGA (640×480)", 640, 480),
                              ("HD (960×720)", 960, 720), ("Full HD (1440×1080)", 1440, 1080)]:
            btn = QPushButton(label)
            btn.setStyleSheet("color:#ccc; background:#333; border:1px solid #555; padding:4px;")
            btn.clicked.connect(lambda _, ww=w, hh=h: self._set_resolution(ww, hh))
            m_layout.addWidget(btn)
        m_layout.addStretch()
        self.menu_button.clicked.connect(self._toggle_menu)

        self._resize_timer = QTimer()
        self._resize_timer.setSingleShot(True)
        self._resize_timer.timeout.connect(self._update_overlay_geo)

    # ── Video ─────────────────────────────────────
    def _update_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame.shape
        qt_img = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
        zoom = self.image_label.zoom_factor
        pw = int(self.image_label.width() * zoom)
        ph = int(self.image_label.height() * zoom)
        self.image_label.setPixmap(
            QPixmap.fromImage(qt_img).scaled(pw, ph, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )
        with axes_lock:
            self.image_label.overlay.update_axes(axes)

        # FPS counter
        self._frame_count += 1
        now = time.monotonic()
        elapsed = now - self._fps_t0
        if elapsed >= 1.0:
            self._video_fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_t0 = now
            self.status_panel.update_video_fps(self._video_fps)

    # ── IMU SSE ──────────────────────────────────
    def _read_imu_sse(self):
        # Kept for future use — connect to IMU_server SSE stream when available
        pass

    @staticmethod
    def _euler_to_quat(x, y, z):
        x, y, z = map(math.radians, (x, y, z))
        cx, sx = math.cos(x / 2), math.sin(x / 2)
        cy, sy = math.cos(y / 2), math.sin(y / 2)
        cz, sz = math.cos(z / 2), math.sin(z / 2)
        return (
            cx * cy * cz + sx * sy * sz,
            sx * cy * cz - cx * sy * sz,
            cx * sy * cz + sx * cy * sz,
            cx * cy * sz - sx * sy * cz,
        )

    # ── Focus / unfocus ───────────────────────────
    def _focus(self, widget):
        self._focused_widget = widget
        view = QWidget()
        layout = QVBoxLayout(view)
        widget.setFixedSize(640, 480)
        layout.addWidget(widget)

        if widget == self.image_label:
            zoom_row = QHBoxLayout()
            for label, factor in (("+", 1.1), ("−", 1 / 1.1)):
                btn = QPushButton(label)
                btn.setFixedSize(30, 30)
                btn.setStyleSheet("font-size:16pt; color:#ccc; background:#333;")
                btn.clicked.connect(lambda _, f=factor: self._adjust_zoom(f))
                zoom_row.addWidget(btn)
            align = QHBoxLayout()
            align.addStretch()
            align.addLayout(zoom_row)
            layout.addLayout(align)

        back = QPushButton("← Back")
        back.setFixedHeight(36)
        back.setStyleSheet("color:#ccc; background:#333; border:1px solid #555; border-radius:4px;")
        back.clicked.connect(self._unfocus)
        layout.addWidget(back)

        self.stack.addWidget(view)
        self.stack.setCurrentWidget(view)
        self._focus_view = view

    def _unfocus(self):
        widget = self._focused_widget
        widget.setFixedSize(320, 240)
        grid = self._grid_widget.layout()
        if widget == self.image_label:
            grid.addWidget(widget, 0, 0)
        elif widget == self.gl_display:
            grid.addWidget(widget, 1, 1, 1, 2)
        elif widget == self.localization_label:
            grid.addWidget(widget, 1, 0)
        self.stack.setCurrentWidget(self._grid_widget)
        self.stack.removeWidget(self._focus_view)
        self._focus_view.deleteLater()

    def _adjust_zoom(self, factor):
        self.image_label.zoom_factor = max(1.0, min(5.0, self.image_label.zoom_factor * factor))

    # ── Sliding menu ─────────────────────────────
    def _toggle_menu(self):
        if self.menu_visible:
            anim = QPropertyAnimation(self._menu, b"geometry")
            anim.setDuration(250)
            anim.setStartValue(QRect(0, 0, 200, self.height()))
            anim.setEndValue(QRect(-200, 0, 200, self.height()))
            anim.setEasingCurve(QEasingCurve.OutCubic)
            anim.start()
            self._menu_anim = anim
            self._overlay.setVisible(False)
            self.menu_visible = False
        else:
            self._overlay.setVisible(True)
            anim = QPropertyAnimation(self._menu, b"geometry")
            anim.setDuration(250)
            anim.setStartValue(QRect(-200, 0, 200, self.height()))
            anim.setEndValue(QRect(0, 0, 200, self.height()))
            anim.setEasingCurve(QEasingCurve.OutCubic)
            anim.start()
            self._menu_anim = anim
            self.menu_visible = True

    def _set_resolution(self, w, h):
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self._toggle_menu()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._resize_timer.start(10)

    def _update_overlay_geo(self):
        self._overlay.setGeometry(self.rect())
        self._menu.setGeometry(-200, 0, 200, self.height())

    def closeEvent(self, event):
        self._stop.set()
        self.cap.release()
        pygame.quit()
        super().closeEvent(event)


# ─────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────

if __name__ == "__main__":
    fmt = QSurfaceFormat()
    fmt.setAlphaBufferSize(8)
    fmt.setRenderableType(QSurfaceFormat.OpenGL)
    fmt.setProfile(QSurfaceFormat.CoreProfile)
    QSurfaceFormat.setDefaultFormat(fmt)

    app = QApplication(sys.argv)
    win = MainWindow()
    win.resize(870, 520)
    win.show()
    sys.exit(app.exec())
