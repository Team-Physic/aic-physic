from pathlib import Path

import cv2
import numpy as np

from bounding_box_tool.dataset import PoseAnnotation
from bounding_box_tool.occlusion import apply_auto_visibility


def _annotation(
    class_id: int,
    left: float,
    top: float,
    right: float,
    bottom: float,
) -> PoseAnnotation:
    return PoseAnnotation(
        class_id=class_id,
        label=f"SFP_{class_id:02d}",
        bbox=(
            (left + right) / 2.0,
            (top + bottom) / 2.0,
            right - left,
            bottom - top,
        ),
        keypoints=(
            (left, top, 2),
            (right, top, 2),
            (right, bottom, 2),
            (left, bottom, 2),
        ),
    )


def _write_image(path: Path) -> None:
    image = np.full((100, 100, 3), 255, dtype=np.uint8)
    image[:, 60:] = 0
    assert cv2.imwrite(str(path), image)


def test_auto_visibility_deletes_fully_occluded_object(tmp_path):
    image_path = tmp_path / "sample.jpg"
    _write_image(image_path)

    result = apply_auto_visibility(
        image_path,
        (_annotation(1, 0.70, 0.20, 0.90, 0.80),),
    )

    assert result.annotations == ()
    assert result.deleted_objects == 1


def test_auto_visibility_marks_only_occluded_keypoints(tmp_path):
    image_path = tmp_path / "sample.jpg"
    _write_image(image_path)

    result = apply_auto_visibility(
        image_path,
        (_annotation(1, 0.20, 0.20, 0.80, 0.80),),
    )

    assert result.deleted_objects == 0
    assert [point[2] for point in result.annotations[0].keypoints] == [2, 1, 1, 2]
    assert result.occluded_keypoints == 2
