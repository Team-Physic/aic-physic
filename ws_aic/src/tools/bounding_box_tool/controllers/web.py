"""HTTP controller for the bounding box editor model and browser view."""

from __future__ import annotations

from typing import Any, Awaitable, Callable

from aiohttp import web

from models import (
    AnnotationConflictError,
    EditorModel,
    EditorModelError,
    ValidationError,
)
from views import asset_response

MAX_JSON_BYTES = 8 * 1024 * 1024


async def _json_body(request: web.Request) -> dict[str, Any]:
    if request.content_length is not None and request.content_length > MAX_JSON_BYTES:
        raise web.HTTPRequestEntityTooLarge(
            max_size=MAX_JSON_BYTES, actual_size=request.content_length
        )
    try:
        value = await request.json()
    except Exception as exc:
        raise ValidationError("request body must be valid JSON") from exc
    if not isinstance(value, dict):
        raise ValidationError("request body must be a JSON object")
    return value


def _index(request: web.Request) -> int:
    try:
        return int(request.match_info["index"])
    except (KeyError, ValueError) as exc:
        raise ValidationError("image index must be an integer") from exc


@web.middleware
async def error_middleware(
    request: web.Request,
    handler: Callable[[web.Request], Awaitable[web.StreamResponse]],
) -> web.StreamResponse:
    """Map model errors to stable JSON responses."""

    try:
        return await handler(request)
    except AnnotationConflictError as exc:
        return web.json_response({"error": str(exc)}, status=409)
    except ValidationError as exc:
        return web.json_response({"error": str(exc)}, status=400)
    except EditorModelError as exc:
        return web.json_response({"error": str(exc)}, status=422)


def create_app(model: EditorModel) -> web.Application:
    """Compose HTTP routes around an editor model."""

    async def index(_request: web.Request) -> web.Response:
        return asset_response("index.html")

    async def asset(request: web.Request) -> web.Response:
        return asset_response(request.match_info["filename"])

    async def health(_request: web.Request) -> web.Response:
        return web.json_response({"status": "ok"})

    async def state(_request: web.Request) -> web.Response:
        return web.json_response(model.snapshot())

    async def open_dataset(request: web.Request) -> web.Response:
        body = await _json_body(request)
        path = body.get("path")
        if not isinstance(path, str) or not path.strip():
            raise ValidationError("path must be a non-empty string")
        return web.json_response(model.open_path(path.strip()))

    async def image_state(request: web.Request) -> web.Response:
        return web.json_response(model.image_snapshot(_index(request)))

    async def image_content(request: web.Request) -> web.FileResponse:
        response = web.FileResponse(model.image_path(_index(request)))
        response.headers["Cache-Control"] = "no-store"
        response.headers["X-Content-Type-Options"] = "nosniff"
        return response

    async def save(request: web.Request) -> web.Response:
        body = await _json_body(request)
        revision = body.get("revision")
        if not isinstance(revision, str):
            raise ValidationError("revision must be a string")
        result = model.save(_index(request), body.get("annotations"), revision)
        return web.json_response(result)

    async def auto_visibility(request: web.Request) -> web.Response:
        body = await _json_body(request)
        result = model.auto_visibility(_index(request), body.get("annotations"))
        return web.json_response(result)

    async def delete_image(request: web.Request) -> web.Response:
        return web.json_response(model.delete_image(_index(request)))

    app = web.Application(
        middlewares=[error_middleware], client_max_size=MAX_JSON_BYTES
    )
    app["editor_model"] = model
    app.router.add_get("/", index)
    app.router.add_get("/assets/{filename}", asset)
    app.router.add_get("/healthz", health)
    app.router.add_get("/api/state", state)
    app.router.add_post("/api/datasets/open", open_dataset)
    app.router.add_get("/api/images/{index}", image_state)
    app.router.add_get("/api/images/{index}/content", image_content)
    app.router.add_put("/api/images/{index}/annotations", save)
    app.router.add_post("/api/images/{index}/auto-visibility", auto_visibility)
    app.router.add_delete("/api/images/{index}", delete_image)
    return app
