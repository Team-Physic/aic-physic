from pathlib import Path

from bounding_box_tool.dataset import ImageDataset, load_annotations


def _dataset(tmp_path: Path) -> Path:
    root = tmp_path / "pose"
    (root / "images/train/right/trial_000").mkdir(parents=True)
    (root / "annotations/train/right/trial_000").mkdir(parents=True)
    (root / "yolo_pose.yaml").write_text(
        "names:\n  0: SFP_00\n  9: SFP_41\nkpt_shape: [4, 3]\n",
        encoding="utf-8",
    )
    return root


def test_open_dataset_discovers_images_and_matching_annotations(tmp_path):
    root = _dataset(tmp_path)
    for number in (10, 2):
        (root / f"images/train/right/trial_000/sample{number}.jpg").touch()
        (root / f"annotations/train/right/trial_000/sample{number}.txt").touch()

    dataset = ImageDataset.open(root)

    assert dataset.dataset_root == root
    assert dataset.class_names == {0: "SFP_00", 9: "SFP_41"}
    assert [entry.image_path.name for entry in dataset.entries] == [
        "sample2.jpg",
        "sample10.jpg",
    ]
    assert dataset.entries[0].annotation_path == (
        root / "annotations/train/right/trial_000/sample2.txt"
    )
    assert dataset.entries[0].display_path == "train/right/trial_000/sample2.jpg"


def test_open_images_subdirectory_keeps_dataset_class_names(tmp_path):
    root = _dataset(tmp_path)
    image = root / "images/train/right/trial_000/sample.jpg"
    image.touch()

    dataset = ImageDataset.open(image.parent)

    assert dataset.dataset_root == root
    assert len(dataset.entries) == 1
    assert dataset.entries[0].image_path == image
    assert dataset.entries[0].annotation_path == (
        root / "annotations/train/right/trial_000/sample.txt"
    )


def test_load_annotations_reads_bbox_keypoints_and_label(tmp_path):
    annotation_path = tmp_path / "sample.txt"
    annotation_path.write_text(
        "9 0.5 0.6 0.2 0.4 "
        "0.4 0.4 2 0.6 0.4 2 0.6 0.8 2 0.4 0.8 2\n",
        encoding="utf-8",
    )

    annotations, warnings = load_annotations(annotation_path, {9: "SFP_41"})

    assert warnings == ()
    assert len(annotations) == 1
    assert annotations[0].class_id == 9
    assert annotations[0].label == "SFP_41"
    assert annotations[0].bbox == (0.5, 0.6, 0.2, 0.4)
    assert annotations[0].keypoints[3] == (0.4, 0.8, 2)


def test_load_annotations_skips_invalid_rows_with_warning(tmp_path):
    annotation_path = tmp_path / "sample.txt"
    annotation_path.write_text(
        "0 0.5 0.5\n"
        "0 1.5 0.6 0.2 0.4 0.4 0.4 2 0.6 0.4 2 0.6 0.8 2 0.4 0.8 2\n",
        encoding="utf-8",
    )

    annotations, warnings = load_annotations(annotation_path, {0: "SFP_00"})

    assert annotations == ()
    assert len(warnings) == 2
    assert "expected 17 fields" in warnings[0]
    assert "outside [0, 1]" in warnings[1]


def test_empty_annotation_is_valid_negative_sample(tmp_path):
    annotation_path = tmp_path / "sample.txt"
    annotation_path.touch()

    assert load_annotations(annotation_path, {}) == ((), ())
