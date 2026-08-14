import json
import math
import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from bounding_box_tool.dataset import ImageDataset, ImageEntry, PoseAnnotation
from bounding_box_tool.viewer import (
    AnnotationCanvas,
    BoundingBoxViewer,
    _move_annotation_selection,
)
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QPixmap
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication, QMessageBox


def test_move_mixed_selection_uses_one_clamped_delta():
    first = PoseAnnotation(
        class_id=0,
        label="first",
        bbox=(0.4, 0.4, 0.2, 0.2),
        keypoints=((0.3, 0.3, 2), (0.5, 0.3, 2), (0.5, 0.5, 2), (0.3, 0.5, 2)),
    )
    second = PoseAnnotation(
        class_id=1,
        label="second",
        bbox=(0.8, 0.8, 0.2, 0.2),
        keypoints=((0.9, 0.9, 2), (0.7, 0.7, 2), (0.9, 0.7, 2), (0.7, 0.9, 2)),
    )

    moved = _move_annotation_selection(
        (first, second), {0}, {(1, 0)}, delta_x=0.3, delta_y=0.3
    )

    assert all(
        math.isclose(actual, expected)
        for actual, expected in zip(moved[0].bbox[:2], (0.5, 0.5))
    )
    assert all(
        math.isclose(actual, expected)
        for actual, expected in zip(moved[0].keypoints[0][:2], (0.4, 0.4))
    )
    assert moved[1].bbox == second.bbox
    assert all(
        math.isclose(actual, expected)
        for actual, expected in zip(moved[1].keypoints[0][:2], (1.0, 1.0))
    )
    assert moved[1].keypoints[1] == second.keypoints[1]


def test_canvas_undo_restores_previous_annotations():
    app = QApplication.instance() or QApplication([])
    canvas = AnnotationCanvas()
    annotation = PoseAnnotation(
        class_id=0,
        label="object",
        bbox=(0.5, 0.5, 0.2, 0.2),
        keypoints=((0.4, 0.4, 2), (0.6, 0.4, 2), (0.6, 0.6, 2), (0.4, 0.6, 2)),
    )

    canvas.add_annotation(annotation)
    canvas.undo()

    assert canvas.annotations == ()
    assert not canvas.can_undo
    app.processEvents()


def test_group_drag_mouse_move_does_not_use_single_annotation_branch():
    app = QApplication.instance() or QApplication([])
    canvas = AnnotationCanvas()
    annotation = PoseAnnotation(
        class_id=0,
        label="object",
        bbox=(0.5, 0.5, 0.2, 0.2),
        keypoints=((0.4, 0.4, 2), (0.6, 0.4, 2), (0.6, 0.6, 2), (0.4, 0.6, 2)),
    )
    canvas._pixmap_item = canvas._scene.addPixmap(QPixmap(100, 100))
    canvas._annotations = [annotation]
    canvas._selected_annotations = {0}
    canvas._drag = ("group", -1, -1)
    canvas._drag_origin = QPointF(10.0, 10.0)
    canvas._drag_original_annotations = (annotation,)

    class Event:
        accepted = False

        def pos(self):
            return canvas.mapFromScene(QPointF(20.0, 20.0))

        def accept(self):
            self.accepted = True

    event = Event()
    canvas.mouseMoveEvent(event)

    assert event.accepted
    assert canvas.annotations[0].bbox[:2] == (0.6, 0.6)
    app.processEvents()


def test_delete_key_handles_range_selected_boxes(monkeypatch):
    app = QApplication.instance() or QApplication([])
    annotation = PoseAnnotation(
        class_id=0,
        label="object",
        bbox=(0.5, 0.5, 0.2, 0.2),
        keypoints=((0.4, 0.4, 2), (0.6, 0.4, 2), (0.6, 0.6, 2), (0.4, 0.6, 2)),
    )
    entry = ImageEntry(Path("image.jpg"), Path("annotation.txt"), "image.jpg")
    monkeypatch.setattr(
        QMessageBox, "question", lambda *_args, **_kwargs: QMessageBox.Yes
    )

    viewer = BoundingBoxViewer()
    viewer.dataset = ImageDataset(Path("."), Path("."), (entry,), {})
    viewer.current_index = 0
    viewer.canvas._annotations = [annotation]
    viewer.canvas._selected_annotations = {0}
    viewer._update_actions()
    viewer.show()
    app.processEvents()

    QTest.keyClick(viewer.canvas.viewport(), Qt.Key_Delete)
    app.processEvents()

    assert viewer.canvas.annotations == ()
    viewer.close()
    app.processEvents()


def test_ctrl_d_deletes_view_updates_samples_and_opens_next(tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    first_image = tmp_path / "images/train/center/trial_000/first.png"
    second_image = tmp_path / "images/train/center/trial_000/second.png"
    first_label = tmp_path / "annotations/train/center/trial_000/first.txt"
    second_label = tmp_path / "annotations/train/center/trial_000/second.txt"
    for path in (first_image, second_image, first_label, second_label):
        path.parent.mkdir(parents=True, exist_ok=True)
    pixmap = QPixmap(20, 20)
    assert pixmap.save(str(first_image))
    assert pixmap.save(str(second_image))
    first_label.write_text("", encoding="utf-8")
    second_label.write_text("", encoding="utf-8")
    first_relative = first_image.relative_to(tmp_path).as_posix()
    second_relative = second_image.relative_to(tmp_path).as_posix()
    samples = [
        {
            "id": "first",
            "images": {
                "left": "images/train/left/trial_000/first.png",
                "center": first_relative,
                "right": "images/train/right/trial_000/first.png",
            },
            "annotations": {
                "left": "annotations/train/left/trial_000/first.txt",
                "center": first_label.relative_to(tmp_path).as_posix(),
                "right": "annotations/train/right/trial_000/first.txt",
            },
            "annotation_object_counts": {"left": 1, "center": 0, "right": 1},
            "annotation_labels": {"left": ["left"], "center": [], "right": ["right"]},
        },
        {
            "id": "second",
            "images": {"center": second_relative},
            "annotations": {
                "center": second_label.relative_to(tmp_path).as_posix()
            },
            "annotation_object_counts": {"center": 0},
            "annotation_labels": {"center": []},
        },
    ]
    samples_path = tmp_path / "samples.jsonl"
    samples_path.write_text(
        "".join(json.dumps(row) + "\n" for row in samples), encoding="utf-8"
    )
    yolo_path = tmp_path / "yolo_pose.yaml"
    yolo_contents = "train: images/train\nnames:\n  0: object\n"
    yolo_path.write_text(yolo_contents, encoding="utf-8")
    entries = (
        ImageEntry(first_image, first_label, first_image.name),
        ImageEntry(second_image, second_label, second_image.name),
    )
    viewer = BoundingBoxViewer()
    viewer.dataset = ImageDataset(tmp_path, tmp_path, entries, {0: "object"})
    viewer.current_index = 0
    viewer.image_list.addItems([entry.display_path for entry in entries])
    viewer.image_list.setCurrentRow(0)
    viewer._update_actions()
    monkeypatch.setattr(
        QMessageBox, "question", lambda *_args, **_kwargs: QMessageBox.Yes
    )
    viewer.show()
    app.processEvents()

    QTest.keyClick(viewer.canvas.viewport(), Qt.Key_D, Qt.ControlModifier)
    app.processEvents()

    remaining = [
        json.loads(line)
        for line in samples_path.read_text(encoding="utf-8").splitlines()
    ]
    assert not first_image.exists()
    assert not first_label.exists()
    assert second_image.exists()
    assert second_label.exists()
    assert remaining[0]["images"] == {
        "left": "images/train/left/trial_000/first.png",
        "right": "images/train/right/trial_000/first.png",
    }
    for field in (
        "annotations",
        "annotation_object_counts",
        "annotation_labels",
    ):
        assert "center" not in remaining[0][field]
    assert remaining[1] == samples[1]
    assert yolo_path.read_text(encoding="utf-8") == yolo_contents
    assert viewer._current_entry() == entries[1]
    assert viewer.image_list.count() == 1
    viewer.close()
    app.processEvents()
