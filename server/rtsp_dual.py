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


def build_dual_pipeline(args) -> str:
    """
    Build a single GStreamer pipeline with both cameras feeding an input-selector.
    Switching cameras = one property set on the selector element (name=sel).

    Both branches are normalized to cam0's output resolution/fps before the selector
    so caps are compatible. The shared encoder tail follows the selector.

    Switching:
        sel = pipeline.get_by_name("sel")
        sel.set_property("active-pad", sel.get_static_pad("sink_0"))  # cam0
        sel.set_property("active-pad", sel.get_static_pad("sink_1"))  # cam10
    """
    out_w   = args.w0
    out_h   = args.h0
    out_fps = args.fps0
    enc     = args.encoder
    extra10 = f' extra-controls="{args.v4l2_extra10}"' if args.v4l2_extra10 else ""

    # Low-light boost for cam10 (optional)
    ll_boost = ""
    if args.ll10:
        ll_boost = (
            f"videobalance brightness={args.ll10_brightness} "
            f"contrast={args.ll10_contrast} "
            f"saturation={args.ll10_saturation} ! "
        )

    # Normalize each branch to I420 at the output resolution before the selector.
    # We stop at I420 (not NV12) so that the NV12 conversion happens AFTER
    # the selector in the tail — this prevents "RGA Blit fail, invalid argument"
    # on Rockchip, where input-selector output buffers confuse the RGA importer
    # inside mpph264enc.
    def normalize(fps_in=None):
        return (
            f"queue max-size-buffers=2 leaky=downstream ! "
            f"videorate drop-only=true ! video/x-raw,framerate={out_fps}/1 ! "
            f"videoconvert ! video/x-raw,format=I420 ! "
            f"videoscale ! video/x-raw,format=I420,width={out_w},height={out_h} ! "
            f"queue max-size-buffers=1 leaky=downstream ! "
        )

    # ---- Cam0 branch (sink_0) ----
    cam0_branch = (
        f"v4l2src device={args.dev0} io-mode=2 do-timestamp=true ! "
        f"video/x-raw,format=UYVY,width={args.w0},height={args.h0} ! "
        + normalize()
        + "sel.sink_0 "
    )

    # ---- Cam10 branch (sink_1) ----
    dev10_mode = args.dev10_mode if args.dev10_mode != "auto" else "mjpg"
    if dev10_mode == "mjpg":
        cam10_src = (
            f"v4l2src device={args.dev10} io-mode=2 do-timestamp=true{extra10} ! "
            f"image/jpeg,width={args.w10},height={args.h10},framerate={args.fps10}/1 ! "
            f"queue max-size-buffers=1 leaky=downstream ! "
            f"jpegdec ! "
        )
    else:
        cam10_src = (
            f"v4l2src device={args.dev10} io-mode=2 do-timestamp=true{extra10} ! "
            f"video/x-raw,format=YUY2,width={args.w10},height={args.h10},framerate={args.fps10}/1 ! "
        )

    cam10_branch = (
        cam10_src
        + f"videoconvert ! video/x-raw,format=I420 ! {ll_boost}"
        + normalize(args.fps10)
        + "sel.sink_1 "
    )

    # ---- Selector + shared encoder tail ----
    # After the selector:
    #   I420 -> BGRx  (pure software; I420->RGB is a well-known sw path)
    #   BGRx -> NV12  (clean RGA path on Rockchip; avoids EINVAL)
    #   NV12 -> mpph264enc
    tail = (
        f"input-selector name=sel sync-streams=false ! "
        f"videoconvert ! video/x-raw,format=BGRx,width={out_w},height={out_h} ! "
        f"videoconvert ! video/x-raw,format=NV12,width={out_w},height={out_h} ! "
        f"queue max-size-buffers=1 leaky=downstream ! "
        f"{enc} bps=10000000 gop=5 ! "
        f"h264parse config-interval=-1 ! "
        f"rtph264pay name=pay0 pt=96 config-interval=1"
    )

    return cam0_branch + cam10_branch + tail


class DualFactory(GstRtspServer.RTSPMediaFactory):
    """RTSP factory that builds the dual-camera pipeline and registers the selector."""

    def __init__(self, mgr: "StreamManager"):
        super().__init__()
        self.mgr = mgr
        self.set_shared(True)

    def do_create_element(self, url):
        pipeline_str = build_dual_pipeline(self.mgr.args)
        print(f"[RTSP] Creating pipeline:\n  {pipeline_str}\n")
        pipeline = Gst.parse_launch(pipeline_str)
        sel = pipeline.get_by_name("sel")
        self.mgr._register_selector(sel)
        return pipeline


class StreamManager:
    def __init__(self, args, server: GstRtspServer.RTSPServer):
        self.args = args
        self.server = server
        self.active = 0
        self._selector = None  # populated when first client connects

    def _register_selector(self, sel):
        self._selector = sel
        # Apply any switch that happened before a client connected
        pad_name = "sink_0" if self.active == 0 else "sink_1"
        sel.set_property("active-pad", sel.get_static_pad(pad_name))
        print(f"[RTSP] Selector ready, active pad: {pad_name}")

    def start(self):
        self.active = 0
        mounts = self.server.get_mount_points()
        factory = DualFactory(self)
        factory.set_latency(0)
        mounts.add_factory(self.args.rtsp_path, factory)
        print(f"[RTSP] rtsp://0.0.0.0:{self.args.rtsp_port}{self.args.rtsp_path}")
        print("[RTSP] Both cameras loaded; switch instantly with POST /cam/0 or /cam/10")

    def switch_to(self, which: int):
        def _do():
            if which == self.active:
                return False
            self.active = which
            if self._selector is None:
                print(f"[SWITCH] Queued -> {'cam0' if which == 0 else 'cam10'} (no client yet)")
                return False
            pad_name = "sink_0" if which == 0 else "sink_1"
            self._selector.set_property("active-pad", self._selector.get_static_pad(pad_name))
            print(f"[SWITCH] -> {'cam0' if which == 0 else 'cam10'} (seamless, no reconnect)")
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
        description="RTSP server with seamless dual-camera switching via input-selector"
    )

    ap.add_argument("--rtsp_port", default="8554")
    ap.add_argument("--rtsp_path", default="/stream")

    ap.add_argument("--dev0",  default="/dev/video0")
    ap.add_argument("--dev10", default="/dev/video10")

    # Cam0 — also sets the output resolution/fps for both cameras
    ap.add_argument("--w0",   type=int, default=1280)
    ap.add_argument("--h0",   type=int, default=720)
    ap.add_argument("--fps0", type=int, default=20)

    # Cam10 (USB) — capture resolution; output is scaled to cam0 size
    ap.add_argument("--w10",   type=int, default=1920)
    ap.add_argument("--h10",   type=int, default=1080)
    ap.add_argument("--fps10", type=int, default=30)

    ap.add_argument("--dev10_mode", choices=["mjpg", "yuy2", "auto"], default="mjpg")
    ap.add_argument("--encoder", default="mpph264enc")

    # HTTP control
    ap.add_argument("--http_port", type=int, default=8081)

    # Low-light for cam10
    ap.add_argument("--ll10", dest="ll10", action="store_true")
    ap.add_argument("--no_ll10", dest="ll10", action="store_false")
    ap.set_defaults(ll10=False)
    ap.add_argument("--ll10_brightness", type=float, default=0.55)
    ap.add_argument("--ll10_contrast",   type=float, default=1.8)
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
