import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from bounding_box_tool.viewer import BoundingBoxViewer
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
