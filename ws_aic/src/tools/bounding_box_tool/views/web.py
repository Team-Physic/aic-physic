"""Browser view and packaged static assets for the annotation editor."""

from __future__ import annotations

from pathlib import Path

from aiohttp import web

STATIC_ROOT = Path(__file__).with_name("static")
ASSETS = {
    "index.html": "text/html",
    "styles.css": "text/css",
    "app.js": "application/javascript",
}


def asset_response(filename: str) -> web.Response:
    """Return a packaged view asset with development-friendly caching."""

    content_type = ASSETS.get(filename)
    if content_type is None:
        raise web.HTTPNotFound()
    return web.Response(
        body=(STATIC_ROOT / filename).read_bytes(),
        content_type=content_type,
        headers={"Cache-Control": "no-store"},
    )
