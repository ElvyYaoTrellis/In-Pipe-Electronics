#!/usr/bin/env python3
import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GLib

Gst.init(None)

# Change these if needed
DEVICE = "/dev/video0"
WIDTH  = 1920
HEIGHT = 1080
FPS    = 30
PORT   = "8554"
PATH   = "/stream"

# PIPELINE = (
#   "v4l2src device=/dev/video0 io-mode=2 do-timestamp=true ! "
#   "video/x-raw,format=UYVY,width=1920,height=1080 ! "
#   "queue max-size-buffers=1 leaky=downstream ! "
#   "videorate drop-only=true ! video/x-raw,framerate=20/1 ! "
#   "queue max-size-buffers=1 leaky=downstream ! "
#   "videoconvert ! video/x-raw,format=NV12 ! "
#   "queue max-size-buffers=1 leaky=downstream ! "
#   "mpph264enc ! h264parse ! "
#   "rtph264pay name=pay0 pt=96 config-interval=1"
# )

PIPELINE = (
  "v4l2src device=/dev/video0 io-mode=2 do-timestamp=true ! "
  "video/x-raw,format=UYVY,width=1920,height=1080 ! "
  "videorate drop-only=true ! video/x-raw,framerate=20/1 ! "
  "videoconvert ! video/x-raw,format=NV12 ! "
  "queue max-size-buffers=4 leaky=downstream ! "
  "mpph264enc gop=10 ! "
  "h264parse config-interval=-1 ! "
  "rtph264pay name=pay0 pt=96 config-interval=1"
)

class Factory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super().__init__()
        self.set_shared(True)

    def do_create_element(self, url):
        return Gst.parse_launch(PIPELINE)

server = GstRtspServer.RTSPServer()
server.set_service(PORT)

mounts = server.get_mount_points()
factory = Factory()
mounts.add_factory(PATH, factory)

server.attach(None)
print(f"RTSP ready: rtsp://0.0.0.0:{PORT}{PATH}")
GLib.MainLoop().run()