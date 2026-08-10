"""bounding_box_tool console entry point."""

from __future__ import annotations

import argparse
import sys

from bounding_box_tool.viewer import BoundingBoxViewer
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication


def _parser() -> argparse.ArgumentParser:
    """viewer 시작 경로만 받는 최소 CLI parser를 생성한다."""
    parser = argparse.ArgumentParser(
        description="View AIC images and YOLO-pose annotations without editing them."
    )
    parser.add_argument(
        "path",
        nargs="?",
        help="Dataset root, image directory, or a single image.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """PyQt application과 read-only viewer를 실행한다."""
    arguments = sys.argv[1:] if argv is None else argv
    args, qt_arguments = _parser().parse_known_args(arguments)
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication([sys.argv[0], *qt_arguments])
    window = BoundingBoxViewer(args.path)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
