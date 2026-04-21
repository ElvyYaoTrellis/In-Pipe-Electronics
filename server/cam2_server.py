import subprocess

FAKE_IP       = "0.0.0.0"     # <-- change me if needed
CAM2_PORT      = 8080
CAM2_PORT = f"http://{FAKE_IP}:{CAM2_PORT}/"

if __name__ == "__main__":
    DEVICE = "/dev/video10"
    PORT = 8080
    subprocess.run(["ffmpeg", "-f", "v4l2", "-i", DEVICE, "-vf", "format=yuv420p","-vcodec", "mjpeg", "-q:v", "5", "-f", "mpjpeg", "-listen", "1", CAM2_PORT])