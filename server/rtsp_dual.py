#!/usr/bin/env python3
import argparse
import gi
import threading
import signal
import os
import time

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GLib
from aiohttp import web

Gst.init(None)


def _enc_element(enc: str, bitrate: int = 10_000_000, gop: int = 5) -> str:
    """Return the GStreamer encoder element string for the given encoder name.

    v4l2h264enc  — Pi VideoCore IV HW encoder (uses extra-controls, not bps/gop)
    x264enc      — software fallback (bitrate in kbps, different property names)
    mpph264enc   — Rockchip MPP HW encoder
    """
    if enc == "v4l2h264enc":
        return (
            f"v4l2h264enc extra-controls=\"controls,"
            f"video_bitrate={bitrate},"
            f"h264_i_frame_period={gop}\""
        )
    if enc == "x264enc":
        return (
            f"x264enc bitrate={bitrate // 1000} key-int-max={gop} "
            f"tune=zerolatency speed-preset=ultrafast"
        )
    return f"{enc} bps={bitrate} gop={gop}"


def _cam_src(device: str, mode: str, width: int, height: int,
             fps: int | None = None, extra_controls: str = "") -> str:
    """Return the source + caps string for a camera branch (no trailing !)."""
    fps_caps = f",framerate={fps}/1" if fps else ""
    extra = f' extra-controls="{extra_controls}"' if extra_controls else ""
    if mode == "csi":
        # Pi CSI camera via libcamera (unicam/libcamerasrc)
        return (
            f"libcamerasrc ! "
            f"video/x-raw,format=NV12,width={width},height={height}{fps_caps}"
        )
    if mode == "mjpg":
        return (
            f"v4l2src device={device} io-mode=2 do-timestamp=true{extra} ! "
            f"image/jpeg,width={width},height={height}{fps_caps} ! "
            f"queue max-size-buffers=1 leaky=downstream ! "
            f"jpegdec"
        )
    fmt = {"raw-uyvy": "UYVY", "raw-yuyv": "YUY2", "raw-yuy2": "YUY2"}.get(mode, "YUY2")
    return (
        f"v4l2src device={device} io-mode=2 do-timestamp=true{extra} ! "
        f"video/x-raw,format={fmt},width={width},height={height}{fps_caps}"
    )


def build_pipeline(args) -> str:
    enc_str = _enc_element(args.encoder, bitrate=args.bitrate, gop=args.gop)
    cam0_mode = getattr(args, "cam0_mode", "raw-yuyv")

    if cam0_mode == "csi":
        # libcamerasrc outputs DMA-BUF NV12 buffers which v4l2h264enc cannot
        # import directly (STREAMON fails with ESRCH). A single videoconvert
        # creates a CPU-backed I420 copy that the encoder accepts.
        # NV12->I420 is cheap (planar rearrangement only).
        return (
            f"libcamerasrc ! "
            f"video/x-raw,format=NV12,width={args.w0},height={args.h0},framerate={args.fps0}/1 ! "
            f"videoconvert ! video/x-raw,format=I420 ! "
            f"queue max-size-buffers=1 leaky=downstream ! "
            f"{enc_str} ! "
            f"h264parse config-interval=-1 ! "
            f"rtph264pay name=pay0 pt=96 config-interval=1"
        )

    # USB camera path
    return (
        _cam_src(args.dev0, cam0_mode, args.w0, args.h0, fps=args.fps0) + " ! "
        f"queue max-size-buffers=2 leaky=downstream ! "
        f"videorate drop-only=true ! video/x-raw,framerate={args.fps0}/1 ! "
        f"videoconvert ! video/x-raw,format=I420 ! "
        f"videoscale ! video/x-raw,format=I420,width={args.w0},height={args.h0} ! "
        f"videoconvert ! video/x-raw,format=NV12,width={args.w0},height={args.h0} ! "
        f"queue max-size-buffers=1 leaky=downstream ! "
        f"{enc_str} ! "
        f"h264parse config-interval=-1 ! "
        f"rtph264pay name=pay0 pt=96 config-interval=1"
    )


def _attach_fps_probe(pipeline):
    """Print encoder output FPS to terminal once per second."""
    pay = pipeline.get_by_name("pay0")
    if not pay:
        return
    pad = pay.get_static_pad("sink")
    state = {"n": 0, "t": time.monotonic()}

    def _probe(pad, info, *_):
        state["n"] += 1
        now = time.monotonic()
        elapsed = now - state["t"]
        if elapsed >= 1.0:
            print(f"[FPS] {state['n'] / elapsed:.1f}", flush=True)
            state["n"] = 0
            state["t"] = now
        return Gst.PadProbeReturn.OK

    pad.add_probe(Gst.PadProbeType.BUFFER, _probe)


class SingleFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.set_shared(True)

    def do_create_element(self, url):
        pipeline_str = build_pipeline(self.args)
        print(f"[RTSP] Creating pipeline:\n  {pipeline_str}\n")
        pipeline = Gst.parse_launch(pipeline_str)
        _attach_fps_probe(pipeline)
        return pipeline


def make_app(args):
    app = web.Application()

    async def status(_):
        return web.json_response({
            "ok": True,
            "device": args.dev0,
            "rtsp": f"rtsp://0.0.0.0:{args.rtsp_port}{args.rtsp_path}",
        })

    app.router.add_get("/status", status)
    return app


def main():
    ap = argparse.ArgumentParser(description="Single-camera RTSP server for Raspberry Pi")

    ap.add_argument("--rtsp_port", default="8554")
    ap.add_argument("--rtsp_path", default="/stream")
    ap.add_argument("--dev0", default="/dev/video0")

    ap.add_argument("--w0",   type=int, default=1280)
    ap.add_argument("--h0",   type=int, default=720)
    ap.add_argument("--fps0", type=int, default=30)

    ap.add_argument("--cam0_mode", choices=["csi", "raw-uyvy", "raw-yuyv", "mjpg"], default="csi",
                    help="Camera source mode: csi=Pi CSI camera (libcamerasrc), mjpg/raw-yuyv=USB camera")
    ap.add_argument("--encoder", default="x264enc",
                    help="GStreamer encoder (x264enc, v4l2h264enc, mpph264enc)")
    ap.add_argument("--bitrate", type=int, default=4_000_000, help="Encoder bitrate in bps")
    ap.add_argument("--gop",     type=int, default=5,          help="Keyframe interval (frames)")

    ap.add_argument("--http_port", type=int, default=8081)

    args = ap.parse_args()

    server = GstRtspServer.RTSPServer()
    server.set_service(args.rtsp_port)
    server.attach(None)

    factory = SingleFactory(args)
    factory.set_latency(0)
    server.get_mount_points().add_factory(args.rtsp_path, factory)
    print(f"[RTSP] rtsp://0.0.0.0:{args.rtsp_port}{args.rtsp_path}")

    loop = GLib.MainLoop()
    shutdown_event = threading.Event()
    shutting_down = False

    def _request_shutdown(signum, _frame):
        nonlocal shutting_down
        if shutting_down:
            os._exit(130)
        shutting_down = True
        print(f"[SIGNAL] {signal.Signals(signum).name} received, shutting down...")
        shutdown_event.set()
        GLib.idle_add(loop.quit)

    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)

    def run_http():
        import asyncio

        async def _main():
            app = make_app(args)
            runner = web.AppRunner(app)
            await runner.setup()
            try:
                site = web.TCPSite(runner, "0.0.0.0", args.http_port, reuse_address=True)
                await site.start()
            except OSError as e:
                print(f"[HTTP] Failed to bind 0.0.0.0:{args.http_port}: {e}")
                shutdown_event.set()
                GLib.idle_add(loop.quit)
                await runner.cleanup()
                return
            try:
                await asyncio.to_thread(shutdown_event.wait)
            finally:
                await runner.cleanup()

        asyncio.run(_main())

    http_thread = threading.Thread(target=run_http, daemon=True)
    http_thread.start()
    print(f"[HTTP] GET /status on port {args.http_port}")

    try:
        loop.run()
    finally:
        shutdown_event.set()
        http_thread.join(timeout=3)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
