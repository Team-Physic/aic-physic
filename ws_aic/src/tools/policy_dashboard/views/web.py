"""HTTP API, MJPEG streams, and static browser view."""

from __future__ import annotations

import asyncio
import signal
from pathlib import Path
from typing import Any

from aiohttp import web
from models import CAMERAS, DashboardState


STREAM_POLL_SECONDS = 0.03
STATIC_ROOT = Path(__file__).with_name("static")


def _asset_response(filename: str, content_type: str) -> web.Response:
    return web.Response(
        body=(STATIC_ROOT / filename).read_bytes(),
        content_type=content_type,
        headers={"Cache-Control": "no-store"},
    )


def create_web_app(state: DashboardState) -> web.Application:
    """Create a view over the current dashboard model."""

    async def index(_request: web.Request) -> web.Response:
        return _asset_response("index.html", "text/html")

    async def styles(_request: web.Request) -> web.Response:
        return _asset_response("styles.css", "text/css")

    async def script(_request: web.Request) -> web.Response:
        return _asset_response("app.js", "application/javascript")

    async def rpy_sphere_script(_request: web.Request) -> web.Response:
        return _asset_response("rpy_sphere.js", "application/javascript")

    async def haptic_script(_request: web.Request) -> web.Response:
        return _asset_response("haptic.js", "application/javascript")

    async def orbit_canvas_script(_request: web.Request) -> web.Response:
        return _asset_response("orbit_canvas.js", "application/javascript")

    async def coordinate_viewer_script(_request: web.Request) -> web.Response:
        return _asset_response("coordinate_viewer.js", "application/javascript")

    async def api_state(_request: web.Request) -> web.Response:
        return web.json_response(
            state.snapshot(), headers={"Cache-Control": "no-store"}
        )

    async def health(_request: web.Request) -> web.Response:
        return web.json_response({"status": "ok"})

    async def stream(request: web.Request) -> web.StreamResponse:
        camera = request.match_info["camera"]
        if camera not in CAMERAS:
            raise web.HTTPNotFound(text=f"unknown camera: {camera}")
        response = web.StreamResponse(
            status=200,
            headers={
                "Content-Type": "multipart/x-mixed-replace; boundary=frame",
                "Cache-Control": "no-store, no-cache, must-revalidate",
                "Pragma": "no-cache",
            },
        )
        await response.prepare(request)
        sequence = 0
        try:
            while request.transport is not None and not request.transport.is_closing():
                frame = state.frame(camera)
                if frame is None or frame.sequence == sequence:
                    await asyncio.sleep(STREAM_POLL_SECONDS)
                    continue
                header = (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(frame.jpeg)}\r\n".encode()
                    + f"X-ROS-Stamp: {frame.stamp}\r\n\r\n".encode()
                )
                await response.write(header + frame.jpeg + b"\r\n")
                sequence = frame.sequence
        except asyncio.CancelledError:
            raise
        except (BrokenPipeError, ConnectionResetError, RuntimeError):
            pass
        return response

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/assets/styles.css", styles)
    app.router.add_get("/assets/app.js", script)
    app.router.add_get("/assets/rpy_sphere.js", rpy_sphere_script)
    app.router.add_get("/assets/haptic.js", haptic_script)
    app.router.add_get("/assets/orbit_canvas.js", orbit_canvas_script)
    app.router.add_get("/assets/coordinate_viewer.js", coordinate_viewer_script)
    app.router.add_get("/api/state", api_state)
    app.router.add_get("/healthz", health)
    app.router.add_get("/stream/{camera}", stream)
    return app


async def serve(
    state: DashboardState,
    host: str,
    port: int,
    logger: Any,
) -> None:
    """Serve until SIGINT or SIGTERM."""

    runner = web.AppRunner(
        create_web_app(state), access_log=None, shutdown_timeout=2.0
    )
    await runner.setup()
    site = web.TCPSite(runner, host=host, port=port)
    await site.start()

    display_host = "127.0.0.1" if host in {"0.0.0.0", "::"} else host
    logger.info(f"browser dashboard: http://{display_host}:{port}")
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for handled_signal in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(handled_signal, stop.set)
        except NotImplementedError:
            pass
    try:
        await stop.wait()
    finally:
        await runner.cleanup()
