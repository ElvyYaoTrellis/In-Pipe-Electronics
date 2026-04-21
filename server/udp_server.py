#!/usr/bin/env python3
import argparse
import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

Gst.init(None)

def build_pipeline(device: str, width: int, height: int, fps: int, host: str, port: int) -> str:
    # RTP/H264 over UDP
    return (
        f"v4l2src device={device} io-mode=2 do-timestamp=true ! "
        f"video/x-raw,format=UYVY,width={width},height={height} ! "
        f"queue max-size-buffers=1 leaky=downstream ! "
        f"videorate drop-only=true ! video/x-raw,framerate={fps}/1 ! "
        f"queue max-size-buffers=1 leaky=downstream ! "
        f"videoconvert ! video/x-raw,format=NV12 ! "
        f"queue max-size-buffers=1 leaky=downstream ! "
        f"mpph264enc ! h264parse ! "
        f"rtph264pay pt=96 config-interval=1 ! "
        f"udpsink host={host} port={port} sync=false async=false"
    )

def on_bus_message(bus, message, loop):
    t = message.type
    if t == Gst.MessageType.ERROR:
        err, dbg = message.parse_error()
        print(f"[GStreamer ERROR] {err}\n  debug: {dbg}")
        loop.quit()
    elif t == Gst.MessageType.EOS:
        print("[GStreamer] EOS")
        loop.quit()
    return True

def main():
    ap = argparse.ArgumentParser(description="UDP (RTP/H264) video sender via GStreamer")
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--fps", type=int, default=20)   # matches your pipeline
    ap.add_argument("--host", default="192.168.1.2", help="Receiver IP (base station)")
    ap.add_argument("--port", type=int, default=5000, help="Receiver UDP port")
    args = ap.parse_args()

    pipeline_str = build_pipeline(args.device, args.width, args.height, args.fps, args.host, args.port)
    print("[PIPELINE]", pipeline_str)

    pipeline = Gst.parse_launch(pipeline_str)
    loop = GLib.MainLoop()

    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_bus_message, loop)

    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        print("[GStreamer] Failed to start pipeline")
        return 1

    print(f"UDP streaming to {args.host}:{args.port} (RTP/H264 payload=96)")
    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.set_state(Gst.State.NULL)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())