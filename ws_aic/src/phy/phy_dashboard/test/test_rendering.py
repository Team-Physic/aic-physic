from types import SimpleNamespace

import numpy as np
import pytest

from phy_dashboard.rendering import image_message_to_rgb


def test_bgr_image_with_row_padding_converts_to_owned_rgb():
    rows = np.array(
        [
            [1, 2, 3, 4, 5, 6, 99, 99],
            [7, 8, 9, 10, 11, 12, 99, 99],
        ],
        dtype=np.uint8,
    )
    message = SimpleNamespace(
        height=2,
        width=2,
        step=8,
        encoding="bgr8",
        data=rows.tobytes(),
    )

    rgb = image_message_to_rgb(message)

    np.testing.assert_array_equal(
        rgb,
        np.array(
            [[[3, 2, 1], [6, 5, 4]], [[9, 8, 7], [12, 11, 10]]],
            dtype=np.uint8,
        ),
    )
    assert rgb.flags.owndata


def test_image_conversion_rejects_short_buffer():
    message = SimpleNamespace(
        height=2,
        width=2,
        step=6,
        encoding="rgb8",
        data=b"short",
    )

    with pytest.raises(ValueError, match="expected at least 12"):
        image_message_to_rgb(message)


def test_image_conversion_rejects_unsupported_encoding():
    message = SimpleNamespace(
        height=1,
        width=1,
        step=3,
        encoding="mono8",
        data=b"\x00\x00\x00",
    )

    with pytest.raises(ValueError, match="unsupported image encoding"):
        image_message_to_rgb(message)
