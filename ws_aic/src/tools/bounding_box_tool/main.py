"""Composition root for the browser-based bounding box editor."""

from __future__ import annotations

import argparse
import os
import sys
import threading
import webbrowser

from aiohttp import web

from controllers import create_app
from models import EditorModel, EditorModelError


def _port(value: str) -> int:
    parsed = int(value)
    if not 1 <= parsed <= 65535:
        raise argparse.ArgumentTypeError("port must be between 1 and 65535")
    return parsed


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Edit AIC YOLO-pose boxes and keypoints in a browser."
    )
    parser.add_argument(
        "path",
        nargs="?",
        help="Dataset root, image directory, or a single image.",
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("BOUNDING_BOX_TOOL_HOST", "127.0.0.1"),
        help="HTTP bind address (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=_port,
        default=os.environ.get("BOUNDING_BOX_TOOL_PORT", "5000"),
        help="HTTP port (default: 5000)",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Do not open the browser automatically.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Start the local HTTP server and browser view."""

    args = _parser().parse_args(sys.argv[1:] if argv is None else argv)
    try:
        model = EditorModel(args.path)
    except EditorModelError as exc:
        print(f"bounding_box_tool: {exc}", file=sys.stderr)
        return 2

    display_host = "127.0.0.1" if args.host in {"0.0.0.0", "::"} else args.host
    url = f"http://{display_host}:{args.port}"
    print(f"Bounding Box Tool: {url}")
    if args.host not in {"127.0.0.1", "localhost", "::1"}:
        print("Warning: non-loopback binding exposes local dataset editing over HTTP.")
    if not args.no_browser:
        threading.Timer(0.6, webbrowser.open, args=(url,)).start()
    web.run_app(create_app(model), host=args.host, port=args.port, print=None)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
