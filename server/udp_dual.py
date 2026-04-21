#!/usr/bin/env python3
import argparse
import gi
import threading
import signal
import os

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
from aiohttp import web

Gst.init(None)

# def build_pipeline(device: str, host: str, port: int, width: int, height: int, fps: int,
#                    mode: str, encoder: str = "mpph264enc") -> str:
#     """
#     mode:
#       - "raw-uyvy" / "raw-yuy2": expects raw frames directly from v4l2
#       - "mjpg": expects MJPG from v4l2 and decodes via jpegdec
#     """
#     common_tail = (
#         f"queue max-size-buffers=1 leaky=downstream ! "
#         f"videorate drop-only=true ! video/x-raw,framerate={fps}/1 ! "
#         f"queue max-size-buffers=1 leaky=downstream ! "
#         f"videoconvert ! video/x-raw,format=NV12 ! "
#         f"queue max-size-buffers=1 leaky=downstream ! "
#         f"videoscale ! video/x-raw,format=NV12,width={width},height={height},framerate={fps}/1 ! "
#         f"queue max-size-buffers=1 leaky=downstream ! "
#         f"{encoder} ! h264parse ! "
#         f"rtph264pay pt=96 config-interval=1 ! "
#         f"udpsink host={host} port={port} sync=false async=false"
#     )

#     if mode == "raw-uyvy":
#         head = (
#             f"v4l2src device={device} io-mode=2 do-timestamp=true ! "
#             f"video/x-raw,format=UYVY,width={width},height={height} ! "
#         )
#         return head + common_tail

#     if mode == "raw-yuy2":
#         head = (
#             f"v4l2src device={device} io-mode=2 do-timestamp=true ! "
#             f"video/x-raw,format=YUY2,width={width},height={height} ! "
#         )
#         return head + common_tail

#     if mode == "mjpg":
#         head = (
#             f"v4l2src device={device} io-mode=2 do-timestamp=true ! "
#             f"image/jpeg,width={width},height={height},framerate={fps}/1 ! "
#             f"queue max-size-buffers=1 leaky=downstream ! "
#             f"jpegdec ! "
#         )
#         return head + common_tail

#     raise ValueError(f"Unknown mode: {mode}")

def build_pipeline(device: str, host: str, port: int, width: int, height: int, fps: int,
                   mode: str,
                   encoder: str = "mpph264enc",
                   low_light: bool = False,
                   ll_brightness: float = 0.35,
                   ll_contrast: float = 1.6,
                   ll_saturation: float = 0.8,
                   v4l2_extra_controls: str = "",
                   src_fps: int | None = None) -> str:
    """
    mode:
      - "raw-uyvy" / "raw-yuy2" / "raw-yuyv": raw frames directly from v4l2
      - "mjpg": MJPG from v4l2 and decodes via jpegdec

    src_fps:
      - if set, we request this framerate at the camera (helps negotiation)
      - useful for UVC cams that only support certain fps (your /dev/video10 is 30fps only)
    """

    extra = f' extra-controls="{v4l2_extra_controls}"' if v4l2_extra_controls else ""

    ll_boost = ""
    if low_light:
        ll_boost = (
            f"videobalance brightness={ll_brightness} contrast={ll_contrast} saturation={ll_saturation} ! "
        )

    # If camera fps is constrained, request src_fps at v4l2src, then drop to fps using videorate.
    src_fps_caps = f",framerate={src_fps}/1" if src_fps else ""

    common_tail = (
        f"queue max-size-buffers=4 leaky=downstream ! "
        f"videorate drop-only=true ! video/x-raw,framerate={fps}/1 ! "
        f"queue max-size-buffers=1 leaky=downstream ! "
        f"videoconvert ! "
        f"{ll_boost}"
        f"videoconvert ! video/x-raw,format=NV12 ! "
        f"queue max-size-buffers=1 leaky=downstream ! "
        f"videoscale ! video/x-raw,format=NV12,width={width},height={height},framerate={fps}/1 ! "
        f"queue max-size-buffers=1 leaky=downstream ! "
        f"{encoder} bps=10000000 gop=30 ! h264parse ! "        
        f"rtph264pay pt=96 config-interval=1 ! "
        f"udpsink host={host} port={port} sync=false async=false"
    )

    if mode == "raw-uyvy":
        head = (
            f"v4l2src device={device} io-mode=2 do-timestamp=true{extra} ! "
            f"video/x-raw,format=UYVY,width={width},height={height}{src_fps_caps} ! "
        )
        return head + common_tail

    if mode in ("raw-yuy2", "raw-yuyv"):
        head = (
            f"v4l2src device={device} io-mode=2 do-timestamp=true{extra} ! "
            f"video/x-raw,format=YUY2,width={width},height={height}{src_fps_caps} ! "
        )
        return head + common_tail


    if mode == "mjpg":
        head = (
            f"v4l2src device={device} io-mode=2 do-timestamp=true{extra} ! "
            f"image/jpeg,width={width},height={height}{src_fps_caps} ! "
            f"queue max-size-buffers=1 leaky=downstream ! "
            f"jpegdec ! "
        )
        return head + common_tail

    raise ValueError(f"Unknown mode: {mode}")



class StreamManager:
    def __init__(self, args, loop: GLib.MainLoop):
        self.args = args
        self.loop = loop
        self.pipeline = None
        self.bus = None

        # active cam: 0 or 10
        self.active = 0

        # for AUTO usb mode fallback logic
        self.dev10_tried = None   # None, "mjpg", "yuy2"

    def _teardown(self):
        if self.pipeline:
            try:
                self.pipeline.set_state(Gst.State.NULL)
            except Exception:
                pass
        if self.bus:
            try:
                self.bus.remove_signal_watch()
            except Exception:
                pass
        self.pipeline = None
        self.bus = None

    def _on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, dbg = message.parse_error()
            print(f"[GStreamer ERROR] {err}\n  debug: {dbg}")

            # If we’re on /dev/video10 with AUTO, try fallback once
            if self.active == 10 and self.args.dev10_mode == "auto":
                if self.dev10_tried == "mjpg":
                    print("[AUTO] MJPG failed; retrying /dev/video10 as YUY2")
                    self.dev10_tried = "yuy2"
                    self._restart_current()
                    return True
                elif self.dev10_tried == "yuy2":
                    print("[AUTO] YUY2 also failed; staying stopped")
                    self._teardown()
                    return True

            # Otherwise, stop pipeline but keep process alive
            self._teardown()
        elif t == Gst.MessageType.EOS:
            print("[GStreamer] EOS")
            self._teardown()
        return True

    def _start_pipeline(self, pipeline_str: str) -> bool:
        self._teardown()

        print("[PIPELINE]", pipeline_str)
        self.pipeline = Gst.parse_launch(pipeline_str)

        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message", self._on_bus_message)

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("[GStreamer] Failed to start pipeline (state change failure)")
            self._teardown()
            return False

        return True

    def _restart_current(self):
        # rebuild pipeline for current active cam
        if self.active == 0:
            p = build_pipeline(
                device=self.args.dev0,
                host=self.args.host,
                port=self.args.port,
                width=self.args.w0,
                height=self.args.h0,
                fps=self.args.fps0,
                mode="raw-uyvy",
                encoder=self.args.encoder,
            )
            self._start_pipeline(p)
        else:
            # /dev/video10
            if self.args.dev10_mode == "auto":
                # start by trying MJPG first
                if self.dev10_tried is None:
                    self.dev10_tried = "mjpg"
                mode = "mjpg" if self.dev10_tried == "mjpg" else "raw-yuy2"
            else:
                mode = "mjpg" if self.args.dev10_mode == "mjpg" else "raw-yuy2"

            p = build_pipeline(
                device=self.args.dev10,
                host=self.args.host,
                port=self.args.port,
                width=self.args.w10,
                height=self.args.h10,
                fps=self.args.fps10,
                mode=mode,                    # "mjpg" or "raw-yuy2"
                encoder=self.args.encoder,
                low_light=False,
                v4l2_extra_controls=self.args.v4l2_extra10,
                src_fps=20,
            )

            ok = self._start_pipeline(p)
            if (not ok) and self.args.dev10_mode == "auto":
                if self.dev10_tried == "mjpg":
                    print("[AUTO] Immediate MJPG start failure; retrying YUY2")
                    self.dev10_tried = "yuy2"
                    self._restart_current()


    def start(self):
        # default start on cam0
        self.active = 0
        self.dev10_tried = None
        self._restart_current()

    def switch_to(self, which: int):
        # schedule on GLib main thread
        def _do():
            if which == self.active:
                return False
            self.active = which
            if which == 10:
                self.dev10_tried = None  # reset auto probing each time you switch to usb cam
            print(f"[SWITCH] Switching to {'/dev/video0 (1080p)' if which==0 else '/dev/video10 (1080p MJPG)'}")
            self._restart_current()
            return False

        GLib.idle_add(_do)

def make_app(mgr: StreamManager):
    app = web.Application()

    async def status(_):
        return web.json_response({
            "ok": True,
            "active": mgr.active,
            "device": mgr.args.dev0 if mgr.active == 0 else mgr.args.dev10,
            "udp": f"{mgr.args.host}:{mgr.args.port}",
        })

    async def cam0(_):
        mgr.switch_to(0)
        return web.json_response({"ok": True, "switching_to": mgr.args.dev0})

    async def cam10(_):
        mgr.switch_to(10)
        return web.json_response({"ok": True, "switching_to": mgr.args.dev10})

    app.router.add_get("/status", status)
    app.router.add_post("/cam/0", cam0)
    app.router.add_post("/cam/10", cam10)
    return app

def main():
    ap = argparse.ArgumentParser(description="UDP (RTP/H264) sender with single-active camera switching")

    ap.add_argument("--host", default="192.168.1.2")
    ap.add_argument("--port", type=int, default=5000)

    ap.add_argument("--dev0", default="/dev/video0")
    ap.add_argument("--dev10", default="/dev/video10")

    # Cam0 (1080p)
    ap.add_argument("--w0", type=int, default=1280)
    ap.add_argument("--h0", type=int, default=720)
    ap.add_argument("--fps0", type=int, default=20)

    # Cam10 (USB) -> now 1080p MJPEG
    ap.add_argument("--w10", type=int, default=1920)
    ap.add_argument("--h10", type=int, default=1080)
    ap.add_argument("--fps10", type=int, default=10)

    # USB camera mode
    ap.add_argument("--dev10_mode", choices=["mjpg", "yuy2", "auto"], default="mjpg")


    # encoder (mpph264enc is hardware on Rockchip; swap if needed)
    ap.add_argument("--encoder", default="mpph264enc")

    # HTTP control
    ap.add_argument("--http_port", type=int, default=8081)


        # Low-light boost for /dev/video10
    ap.add_argument("--ll10", dest="ll10", action="store_true")
    ap.add_argument("--no_ll10", dest="ll10", action="store_false")
    ap.set_defaults(ll10=False)

    ap.add_argument("--ll10_brightness", type=float, default=0.55)
    ap.add_argument("--ll10_contrast",   type=float, default=1.8)
    ap.add_argument("--ll10_saturation", type=float, default=0.6)


    # ap.add_argument(
    #     "--v4l2_extra10",
    #     default="c,power_line_frequency=2,exposure_dynamic_framerate=1,gamma=2,backlight_compensation=1,sharpness=0,brightness=250,contrast=100,saturation=10"
    # )

    ap.add_argument("--v4l2_extra10", default="")
    # Optional: pass v4l2src extra-controls string for /dev/video10 (camera-dependent!)
    # Example: --v4l2_extra10 'c,exposure_auto=1,exposure_absolute=400,gain=255'
    # ap.add_argument("--v4l2_extra10", default="")


    args = ap.parse_args()

    loop = GLib.MainLoop()
    mgr = StreamManager(args, loop)
    mgr.start()

    shutting_down = False
    shutdown_event = threading.Event()

    def _request_shutdown(signum, _frame):
        nonlocal shutting_down
        if shutting_down:
            sig_name = signal.Signals(signum).name
            print(f"[SIGNAL] {sig_name} received again, forcing exit...")
            os._exit(130)
            return
        shutting_down = True
        sig_name = signal.Signals(signum).name
        print(f"[SIGNAL] {sig_name} received, shutting down...")
        shutdown_event.set()
        GLib.idle_add(loop.quit)

    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)

    def run_http():
        import asyncio
        async def _main():
            app = make_app(mgr)  # or make_app(switcher)
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

    print(f"[UDP] RTP/H264 -> {args.host}:{args.port}")
    print(f"[HTTP] POST /cam/0 or /cam/10, GET /status on port {args.http_port}")

    try:
        loop.run()
    finally:
        shutdown_event.set()
        mgr._teardown()
        http_thread.join(timeout=3)
        if http_thread.is_alive():
            print("[HTTP] Shutdown timed out; exiting process and letting OS reclaim socket.")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
