from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from phy_data_collection.policy.dataset import _camera_matrix


def _image(width: int, height: int) -> SimpleNamespace:
    return SimpleNamespace(width=width, height=height)


def _camera_info(width: int, height: int) -> SimpleNamespace:
    return SimpleNamespace(
        width=width,
        height=height,
        k=[600.0, 0.0, width / 2, 0.0, 600.0, height / 2, 0.0, 0.0, 1.0],
    )


def test_camera_matrix_keeps_rgb_calibration() -> None:
    camera_info = _camera_info(864, 768)

    matrix = _camera_matrix(camera_info, _image(864, 768))

    np.testing.assert_allclose(matrix, np.asarray(camera_info.k).reshape(3, 3))


def test_camera_matrix_scales_depth_calibration_to_rgb_image() -> None:
    camera_info = _camera_info(576, 512)

    matrix = _camera_matrix(camera_info, _image(864, 768))

    np.testing.assert_allclose(
        matrix,
        np.diag([1.5, 1.5, 1.0]) @ np.asarray(camera_info.k).reshape(3, 3),
    )


def test_camera_matrix_rejects_different_aspect_ratio() -> None:
    assert _camera_matrix(_camera_info(640, 480), _image(864, 768)) is None
