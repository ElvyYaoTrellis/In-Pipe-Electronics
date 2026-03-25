#!/usr/bin/env python3
"""Single-camera RTSP server for the Radxa MIPI/CSI camera (/dev/video0, rkisp_mainpath).

Usage:
    sudo python3 rtsp_mipi.py
    sudo python3 rtsp_mipi.py --w0 1280 --h0 720 --fps0 30
"""
import argparse
import threading
import signal
import os

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GLib
from aiohttp import web

Gst.init(None)


def build_pipeline(args) -> str:
    return (
        f"( "
        f"v4l2src device={args.dev0} io-mode=2 do-timestamp=true ! "
        f"video/x-raw,format=NV12,width={args.w0},height={args.h0} ! "
        f"videoconvert ! video/x-raw,format=I420 ! "
        f"queue max-size-buffers=1 leaky=downstream ! "
        f"videoconvert ! video/x-raw,format=NV12,width={args.w0},height={args.h0} ! "
        f"{args.encoder} bps={args.bitrate} gop={args.gop} ! "
        f"h264parse config-interval=-1 ! "
        f"rtph264pay name=pay0 pt=96 config-interval=1"
        f" )"
    )


def main():
    ap = argparse.ArgumentParser(description="Single-camera RTSP server (MIPI/CSI)")
    ap.add_argument("--dev0",      default="/dev/video0")
    ap.add_argument("--w0",        type=int, default=1920)
    ap.add_argument("--h0",        type=int, default=1080)
    ap.add_argument("--fps0",      type=int, default=21)
    ap.add_argument("--encoder",   default="mpph264enc")
    ap.add_argument("--bitrate",   type=int, default=10_000_000)
    ap.add_argument("--gop",       type=int, default=5)
    ap.add_argument("--rtsp_port", default="8554")
    ap.add_argument("--rtsp_path", default="/stream")
    ap.add_argument("--http_port", type=int, default=8081)
    args = ap.parse_args()

    pipeline_str = build_pipeline(args)
    print(f"[RTSP] Pipeline:\n  {pipeline_str}\n")

    server = GstRtspServer.RTSPServer()
    server.set_service(args.rtsp_port)

    factory = GstRtspServer.RTSPMediaFactory()
    factory.set_launch(pipeline_str)
    factory.set_shared(True)
    factory.set_latency(0)

    mounts = server.get_mount_points()
    mounts.add_factory(args.rtsp_path, factory)
    server.attach(None)

    print(f"[RTSP] rtsp://0.0.0.0:{args.rtsp_port}{args.rtsp_path}")

    loop = GLib.MainLoop()
    shutdown_event = threading.Event()
    shutting_down = False

    def _shutdown(signum, _frame):
        nonlocal shutting_down
        if shutting_down:
            os._exit(130)
        shutting_down = True
        print(f"\n[SIGNAL] {signal.Signals(signum).name} — shutting down...")
        shutdown_event.set()
        GLib.idle_add(loop.quit)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    async def status(_):
        return web.json_response({
            "ok": True,
            "device": args.dev0,
            "rtsp": f"rtsp://0.0.0.0:{args.rtsp_port}{args.rtsp_path}",
        })

    def run_http():
        import asyncio
        async def _main():
            app = web.Application()
            app.router.add_get("/status", status)
            runner = web.AppRunner(app)
            await runner.setup()
            site = web.TCPSite(runner, "0.0.0.0", args.http_port, reuse_address=True)
            await site.start()
            print(f"[HTTP] GET /status on port {args.http_port}")
            try:
                await asyncio.to_thread(shutdown_event.wait)
            finally:
                await runner.cleanup()
        asyncio.run(_main())

    http_thread = threading.Thread(target=run_http, daemon=True)
    http_thread.start()

    try:
        loop.run()
    finally:
        shutdown_event.set()
        http_thread.join(timeout=3)


if __name__ == "__main__":
    raise SystemExit(main())
