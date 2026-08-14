"""ROS image buffer conversion helpers independent from the Qt event loop."""

from __future__ import annotations

import numpy as np


def image_message_to_rgb(message) -> np.ndarray:
    """Convert a packed ROS rgb8/bgr8 Image-like object into owned RGB pixels."""
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    encoding = str(message.encoding).lower()
    if height <= 0 or width <= 0:
        raise ValueError("image dimensions must be positive")
    if encoding not in {"rgb8", "bgr8"}:
        raise ValueError(f"unsupported image encoding: {message.encoding!r}")
    row_bytes = width * 3
    if step < row_bytes:
        raise ValueError(f"image step {step} is smaller than {row_bytes}")
    buffer = np.frombuffer(message.data, dtype=np.uint8)
    required = height * step
    if buffer.size < required:
        raise ValueError(
            f"image buffer has {buffer.size} bytes; expected at least {required}"
        )
    packed = buffer[:required].reshape(height, step)[:, :row_bytes]
    image = packed.reshape(height, width, 3)
    return image.copy() if encoding == "rgb8" else image[:, :, ::-1].copy()
