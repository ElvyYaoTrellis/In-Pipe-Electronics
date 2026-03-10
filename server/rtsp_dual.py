#!/usr/bin/env python3
import argparse
import gi
import threading
import signal
import os

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GLib
from aiohttp import web

Gst.init(None)


def build_pipeline(device: str, width: int, height: int, fps: int,
                   mode: str,
                   encoder: str = "mpph264enc",
                   low_light: bool = False,
                   ll_brightness: float = 0.35,
                   ll_contrast: float = 1.6,
                   ll_saturation: float = 0.8,
                   v4l2_extra_controls: str = "",
                   src_fps: int | None = None) -> str:
    """
    Build a GStreamer pipeline string for RTSP serving.
    Sink is rtph264pay (name=pay0) instead of udpsink.

    mode:
      - "raw-uyvy" / "raw-yuy2" / "raw-yuyv": raw frames from v4l2
      - "mjpg": MJPG from v4l2, decoded via jpegdec
    """
    extra = f' extra-controls="{v4l2_extra_controls}"' if v4l2_extra_controls else ""

    ll_boost = ""
    if low_light:
        ll_boost = (
            f"videobalance brightness={ll_brightness} contrast={ll_contrast} "
            f"saturation={ll_saturation} ! "
        )

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
        f"{encoder} bps=10000000 gop=30 ! h264parse config-interval=-1 ! "
        f"rtph264pay name=pay0 pt=96 config-interval=1"
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


class SwitchableFactory(GstRtspServer.RTSPMediaFactory):
    """RTSP media factory that builds its pipeline from the StreamManager."""

    def __init__(self, mgr: "StreamManager"):
        super().__init__()
        self.mgr = mgr
        self.set_shared(True)

    def do_create_element(self, url):
        pipeline_str = self.mgr.build_current_pipeline()
        print(f"[RTSP] Creating pipeline: {pipeline_str}")
        return Gst.parse_launch(pipeline_str)


class StreamManager:
    def __init__(self, args, server: GstRtspServer.RTSPServer):
        self.args = args
        self.server = server
        self.active = 0
        self.dev10_tried: str | None = None

    def build_current_pipeline(self) -> str:
        if self.active == 0:
            return build_pipeline(
                device=self.args.dev0,
                width=self.args.w0,
                height=self.args.h0,
                fps=self.args.fps0,
                mode="raw-uyvy",
                encoder=self.args.encoder,
            )

        # /dev/video10
        if self.args.dev10_mode == "auto":
            if self.dev10_tried is None:
                self.dev10_tried = "mjpg"
            mode = "mjpg" if self.dev10_tried == "mjpg" else "raw-yuy2"
        else:
            mode = "mjpg" if self.args.dev10_mode == "mjpg" else "raw-yuy2"

        return build_pipeline(
            device=self.args.dev10,
            width=self.args.w10,
            height=self.args.h10,
            fps=self.args.fps10,
            mode=mode,
            encoder=self.args.encoder,
            low_light=self.args.ll10,
            ll_brightness=self.args.ll10_brightness,
            ll_contrast=self.args.ll10_contrast,
            ll_saturation=self.args.ll10_saturation,
            v4l2_extra_controls=self.args.v4l2_extra10,
            src_fps=30,
        )

    def _remount(self):
        """
        Replace the RTSP mount point with a fresh factory.
        Any connected clients will lose the stream and must reconnect.
        """
        mounts = self.server.get_mount_points()
        mounts.remove_factory(self.args.rtsp_path)
        factory = SwitchableFactory(self)
        mounts.add_factory(self.args.rtsp_path, factory)
        label = "cam0 (dev0)" if self.active == 0 else "cam10 (dev10)"
        print(f"[RTSP] Remounted {self.args.rtsp_path} -> {label}")

    def start(self):
        self.active = 0
        self.dev10_tried = None
        mounts = self.server.get_mount_points()
        mounts.add_factory(self.args.rtsp_path, SwitchableFactory(self))
        print(f"[RTSP] rtsp://0.0.0.0:{self.args.rtsp_port}{self.args.rtsp_path}")

    def switch_to(self, which: int):
        def _do():
            if which == self.active:
                return False
            self.active = which
            if which == 10:
                self.dev10_tried = None  # reset auto-probe on each switch
            print(f"[SWITCH] -> {'cam0' if which == 0 else 'cam10'} (clients must reconnect)")
            self._remount()
            return False

        GLib.idle_add(_do)


def make_app(mgr: StreamManager):
    app = web.Application()

    async def status(_):
        return web.json_response({
            "ok": True,
            "active": mgr.active,
            "device": mgr.args.dev0 if mgr.active == 0 else mgr.args.dev10,
            "rtsp": f"rtsp://0.0.0.0:{mgr.args.rtsp_port}{mgr.args.rtsp_path}",
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
    ap = argparse.ArgumentParser(
        description="RTSP server with single-active camera switching (two cameras)"
    )

    ap.add_argument("--rtsp_port", default="8554",
                    help="RTSP server port (default: 8554)")
    ap.add_argument("--rtsp_path", default="/stream",
                    help="RTSP mount path (default: /stream)")

    ap.add_argument("--dev0", default="/dev/video0")
    ap.add_argument("--dev10", default="/dev/video10")

    # Cam0
    ap.add_argument("--w0", type=int, default=1280)
    ap.add_argument("--h0", type=int, default=720)
    ap.add_argument("--fps0", type=int, default=20)

    # Cam10 (USB)
    ap.add_argument("--w10", type=int, default=1920)
    ap.add_argument("--h10", type=int, default=1080)
    ap.add_argument("--fps10", type=int, default=30)

    ap.add_argument("--dev10_mode", choices=["mjpg", "yuy2", "auto"], default="mjpg")

    ap.add_argument("--encoder", default="mpph264enc")

    # HTTP control server
    ap.add_argument("--http_port", type=int, default=8081)

    # Low-light for cam10
    ap.add_argument("--ll10", dest="ll10", action="store_true")
    ap.add_argument("--no_ll10", dest="ll10", action="store_false")
    ap.set_defaults(ll10=False)
    ap.add_argument("--ll10_brightness", type=float, default=0.55)
    ap.add_argument("--ll10_contrast", type=float, default=1.8)
    ap.add_argument("--ll10_saturation", type=float, default=0.6)

    ap.add_argument("--v4l2_extra10", default="")

    args = ap.parse_args()

    server = GstRtspServer.RTSPServer()
    server.set_service(args.rtsp_port)
    server.attach(None)

    loop = GLib.MainLoop()
    mgr = StreamManager(args, server)
    mgr.start()

    shutdown_event = threading.Event()
    shutting_down = False

    def _request_shutdown(signum, _frame):
        nonlocal shutting_down
        if shutting_down:
            print(f"[SIGNAL] {signal.Signals(signum).name} again, forcing exit...")
            os._exit(130)
            return
        shutting_down = True
        print(f"[SIGNAL] {signal.Signals(signum).name} received, shutting down...")
        shutdown_event.set()
        GLib.idle_add(loop.quit)

    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)

    def run_http():
        import asyncio

        async def _main():
            app = make_app(mgr)
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

    print(f"[HTTP] POST /cam/0 or /cam/10, GET /status  on port {args.http_port}")
    print("[NOTE] Switching cameras disconnects current RTSP clients; they must reconnect.")

    try:
        loop.run()
    finally:
        shutdown_event.set()
        http_thread.join(timeout=3)
        if http_thread.is_alive():
            print("[HTTP] Shutdown timed out; letting OS reclaim socket.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
