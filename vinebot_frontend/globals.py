from threading import Lock

RADXA_IP  = "192.168.1.53"
MOTOR_URL = f"http://{RADXA_IP}:8005"
LED_URL      = f"http://{RADXA_IP}:8080"
LED_REAR_URL = f"http://{RADXA_IP}:8080/rear"
CAM_URL   = f"http://{RADXA_IP}:8081"
VIDEO_URL = f"rtsp://{RADXA_IP}:8554/stream"

# Scale joystick [-1, 1] to motor velocity units — tune to match your hardware
MAX_VEL = 2000

axes_lock = Lock()
axes = [0.0] * 6
