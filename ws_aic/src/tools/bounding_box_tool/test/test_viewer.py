import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from bounding_box_tool.dataset import PoseAnnotation, load_annotations
from bounding_box_tool.viewer import (
    BoundingBoxViewer,
    _move_annotation,
    _resize_annotation,
    _set_keypoint,
)
from PyQt5.QtGui import QColor, QImage
from PyQt5.QtWidgets import QApplication


def test_viewer_opens_dataset_and_renders_object_list(tmp_path):
    root = tmp_path / "pose"
    image_dir = root / "images/train/right/trial_000"
    annotation_dir = root / "annotations/train/right/trial_000"
    image_dir.mkdir(parents=True)
    annotation_dir.mkdir(parents=True)
    (root / "yolo_pose.yaml").write_text(
        "names:\n  9: SFP_41\nkpt_shape: [4, 3]\n",
        encoding="utf-8",
    )
    image_path = image_dir / "sample.jpg"
    QImage(120, 80, QImage.Format_RGB32).save(str(image_path))
    (annotation_dir / "sample.txt").write_text(
        "9 0.5 0.5 0.2 0.2 "
        "0.4 0.4 2 0.6 0.4 2 0.6 0.6 2 0.4 0.6 2\n",
        encoding="utf-8",
    )
    app = QApplication.instance() or QApplication([])

    window = BoundingBoxViewer(root)
    app.processEvents()

    assert window.image_list.count() == 1
    assert window.object_list.count() == 1
    assert window.object_list.item(0).text().startswith("SFP_41")
    assert window.canvas.image_size == (120, 80)
    assert "sample.txt" in window.annotation_path.text()
    window.close()


def test_viewer_add_delete_and_save_annotations(tmp_path):
    root = tmp_path / "pose"
    image_dir = root / "images/train/right/trial_000"
    annotation_dir = root / "annotations/train/right/trial_000"
    image_dir.mkdir(parents=True)
    annotation_dir.mkdir(parents=True)
    (root / "yolo_pose.yaml").write_text(
        "names:\n  9: SFP_41\nkpt_shape: [4, 3]\n",
        encoding="utf-8",
    )
    image_path = image_dir / "sample.jpg"
    QImage(120, 80, QImage.Format_RGB32).save(str(image_path))
    annotation_path = annotation_dir / "sample.txt"
    annotation_path.touch()
    app = QApplication.instance() or QApplication([])
    window = BoundingBoxViewer(root)
    annotation = PoseAnnotation(
        class_id=9,
        label="SFP_41",
        bbox=(0.5, 0.5, 0.2, 0.2),
        keypoints=(
            (0.4, 0.4, 2),
            (0.6, 0.4, 2),
            (0.6, 0.6, 2),
            (0.4, 0.6, 2),
        ),
    )

    window.canvas.add_annotation(annotation)

    assert window.dirty
    assert window.object_list.count() == 1
    assert window.save_current()
    assert load_annotations(annotation_path, {9: "SFP_41"})[0] == (annotation,)

    window.canvas.delete_selected()

    assert window.save_current()
    assert annotation_path.read_text(encoding="utf-8") == ""
    window.close()


def test_geometry_edit_helpers_move_resize_and_update_keypoint():
    annotation = PoseAnnotation(
        class_id=0,
        label="SFP_00",
        bbox=(0.5, 0.5, 0.2, 0.2),
        keypoints=(
            (0.4, 0.4, 2),
            (0.6, 0.4, 2),
            (0.6, 0.6, 2),
            (0.4, 0.6, 2),
        ),
    )

    moved = _move_annotation(annotation, 0.1, -0.1)
    resized = _resize_annotation(moved, 2, 0.8, 0.7)
    updated = _set_keypoint(resized, 0, 0.25, 0.35)

    assert moved.bbox == (0.6, 0.4, 0.2, 0.2)
    assert resized.bbox == pytest.approx((0.65, 0.5, 0.3, 0.4))
    assert updated.keypoints[0] == (0.25, 0.35, 2)
