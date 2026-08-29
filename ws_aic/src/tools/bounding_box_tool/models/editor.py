"""Application model for the browser annotation editor."""

from __future__ import annotations

import hashlib
import math
import threading
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any, Iterable

from models.dataset import (
    ImageDataset,
    ImageEntry,
    PoseAnnotation,
    load_annotations,
    samples_without_image,
    save_annotations,
    save_samples,
)
from models.occlusion import apply_auto_visibility

MAX_ANNOTATIONS = 10_000
MIN_BOX_SIZE = 1e-5


class EditorModelError(Exception):
    """Base class for errors that can safely be shown in the browser."""


class DatasetNotOpenError(EditorModelError):
    """Raised when an operation needs an open dataset."""


class AnnotationConflictError(EditorModelError):
    """Raised when the annotation changed after the browser loaded it."""


class ValidationError(EditorModelError):
    """Raised when browser-provided annotation data is invalid."""


def _file_revision(path: Path | None) -> str:
    """Return a content revision that also represents a missing label file."""

    if path is None or not path.is_file():
        return "missing"
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return f"sha256:{digest}"


def _annotation_dict(annotation: PoseAnnotation) -> dict[str, Any]:
    return {
        "class_id": annotation.class_id,
        "label": annotation.label,
        "bbox": list(annotation.bbox),
        "keypoints": [list(keypoint) for keypoint in annotation.keypoints],
    }


def _finite_float(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise ValidationError(f"{field} must be a number")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValidationError(f"{field} must be a number") from exc
    if not math.isfinite(parsed):
        raise ValidationError(f"{field} must be finite")
    return parsed


def _normalized(value: Any, field: str) -> float:
    parsed = _finite_float(value, field)
    if not 0.0 <= parsed <= 1.0:
        raise ValidationError(f"{field} must be between 0 and 1")
    return parsed


def parse_annotations(
    values: Any,
    class_names: dict[int, str],
) -> tuple[PoseAnnotation, ...]:
    """Validate browser JSON and turn it into domain annotations."""

    if not isinstance(values, list):
        raise ValidationError("annotations must be an array")
    if len(values) > MAX_ANNOTATIONS:
        raise ValidationError(f"annotations cannot exceed {MAX_ANNOTATIONS}")

    parsed_annotations: list[PoseAnnotation] = []
    for annotation_index, value in enumerate(values):
        prefix = f"annotations[{annotation_index}]"
        if not isinstance(value, dict):
            raise ValidationError(f"{prefix} must be an object")

        class_id = value.get("class_id")
        if isinstance(class_id, bool) or not isinstance(class_id, int):
            raise ValidationError(f"{prefix}.class_id must be an integer")
        if not 0 <= class_id <= 9999:
            raise ValidationError(f"{prefix}.class_id must be between 0 and 9999")

        bbox_value = value.get("bbox")
        if not isinstance(bbox_value, list) or len(bbox_value) != 4:
            raise ValidationError(f"{prefix}.bbox must contain four numbers")
        bbox = tuple(
            _normalized(item, f"{prefix}.bbox[{index}]")
            for index, item in enumerate(bbox_value)
        )
        if bbox[2] < MIN_BOX_SIZE or bbox[3] < MIN_BOX_SIZE:
            raise ValidationError(f"{prefix}.bbox width and height are too small")

        keypoints_value = value.get("keypoints")
        if not isinstance(keypoints_value, list) or len(keypoints_value) != 4:
            raise ValidationError(f"{prefix}.keypoints must contain four points")
        keypoints: list[tuple[float, float, int]] = []
        for keypoint_index, keypoint in enumerate(keypoints_value):
            keypoint_prefix = f"{prefix}.keypoints[{keypoint_index}]"
            if not isinstance(keypoint, list) or len(keypoint) != 3:
                raise ValidationError(
                    f"{keypoint_prefix} must contain x, y, visibility"
                )
            visibility = keypoint[2]
            if (
                isinstance(visibility, bool)
                or not isinstance(visibility, int)
                or visibility not in (0, 1, 2)
            ):
                raise ValidationError(
                    f"{keypoint_prefix}.visibility must be 0, 1, or 2"
                )
            keypoints.append(
                (
                    _normalized(keypoint[0], f"{keypoint_prefix}.x"),
                    _normalized(keypoint[1], f"{keypoint_prefix}.y"),
                    visibility,
                )
            )

        parsed_annotations.append(
            PoseAnnotation(
                class_id=class_id,
                label=class_names.get(class_id, f"class_{class_id}"),
                bbox=bbox,
                keypoints=tuple(keypoints),  # type: ignore[arg-type]
            )
        )
    return tuple(parsed_annotations)


class EditorModel:
    """Own the open dataset and all filesystem mutations.

    The browser is deliberately a client of this model instead of being trusted
    with paths. A lock keeps two requests from interleaving file mutations.
    """

    def __init__(self, initial_path: str | Path | None = None):
        self._lock = threading.RLock()
        self._dataset: ImageDataset | None = None
        self._generation = 0
        if initial_path is not None:
            self.open_path(initial_path)

    @property
    def dataset(self) -> ImageDataset | None:
        return self._dataset

    def open_path(self, path: str | Path) -> dict[str, Any]:
        """Open a dataset root, image directory, or single image."""

        try:
            dataset = ImageDataset.open(path)
        except (FileNotFoundError, OSError, ValueError) as exc:
            raise EditorModelError(str(exc)) from exc
        with self._lock:
            self._dataset = dataset
            self._generation += 1
            return self.snapshot()

    def snapshot(self) -> dict[str, Any]:
        """Return dataset metadata used to render the image navigator."""

        with self._lock:
            if self._dataset is None:
                return {"open": False, "generation": self._generation}
            dataset = self._dataset
            return {
                "open": True,
                "generation": self._generation,
                "selected_path": str(dataset.selected_path),
                "dataset_root": (
                    str(dataset.dataset_root)
                    if dataset.dataset_root is not None
                    else None
                ),
                "collection_policy": dataset.collection_policy,
                "class_names": [
                    {"id": class_id, "label": label}
                    for class_id, label in sorted(dataset.class_names.items())
                ],
                "images": [
                    {
                        "index": index,
                        "display_path": entry.display_path,
                        "editable": entry.annotation_path is not None,
                    }
                    for index, entry in enumerate(dataset.entries)
                ],
            }

    def _entry(self, index: int) -> tuple[ImageDataset, ImageEntry]:
        dataset = self._dataset
        if dataset is None:
            raise DatasetNotOpenError("open a dataset first")
        if not 0 <= index < len(dataset.entries):
            raise EditorModelError(f"image index out of range: {index}")
        return dataset, dataset.entries[index]

    def image_path(self, index: int) -> Path:
        with self._lock:
            _dataset, entry = self._entry(index)
            if not entry.image_path.is_file():
                raise EditorModelError(f"image no longer exists: {entry.image_path}")
            return entry.image_path

    def image_snapshot(self, index: int) -> dict[str, Any]:
        """Load one image's annotations and current file revision."""

        with self._lock:
            dataset, entry = self._entry(index)
            try:
                annotations, warnings = load_annotations(
                    entry.annotation_path, dataset.class_names
                )
                revision = _file_revision(entry.annotation_path)
            except OSError as exc:
                raise EditorModelError(str(exc)) from exc
            return {
                "index": index,
                "display_path": entry.display_path,
                "image_path": str(entry.image_path),
                "annotation_path": (
                    str(entry.annotation_path)
                    if entry.annotation_path is not None
                    else None
                ),
                "editable": entry.annotation_path is not None,
                "revision": revision,
                "annotations": [_annotation_dict(item) for item in annotations],
                "warnings": list(warnings),
                "image_url": f"/api/images/{index}/content",
            }

    def save(
        self,
        index: int,
        raw_annotations: Any,
        expected_revision: str,
    ) -> dict[str, Any]:
        """Validate and atomically save annotations unless the file changed."""

        with self._lock:
            dataset, entry = self._entry(index)
            if entry.annotation_path is None:
                raise EditorModelError("this image has no dataset annotation path")
            current_revision = _file_revision(entry.annotation_path)
            if expected_revision != current_revision:
                raise AnnotationConflictError(
                    "annotation changed on disk; reload the image before saving"
                )
            annotations = parse_annotations(raw_annotations, dataset.class_names)
            try:
                save_annotations(entry.annotation_path, annotations)
                revision = _file_revision(entry.annotation_path)
            except OSError as exc:
                raise EditorModelError(str(exc)) from exc
            return {
                "revision": revision,
                "annotations": [_annotation_dict(item) for item in annotations],
            }

    def auto_visibility(self, index: int, raw_annotations: Any) -> dict[str, Any]:
        """Apply RGB robot-arm visibility estimation without saving it."""

        with self._lock:
            dataset, entry = self._entry(index)
            annotations = parse_annotations(raw_annotations, dataset.class_names)
            try:
                result = apply_auto_visibility(
                    entry.image_path,
                    annotations,
                    preserve_fully_occluded=(dataset.collection_policy == "near-port"),
                )
            except (OSError, ValueError) as exc:
                raise EditorModelError(str(exc)) from exc
            payload = asdict(result)
            payload["annotations"] = [
                _annotation_dict(item) for item in result.annotations
            ]
            return payload

    def delete_image(self, index: int) -> dict[str, Any]:
        """Delete an image/label and update samples.jsonl as one operation."""

        with self._lock:
            dataset, entry = self._entry(index)
            root = dataset.dataset_root
            if root is None:
                raise EditorModelError("sample deletion requires a dataset root")
            samples_path = root / "samples.jsonl"
            try:
                updated_samples = samples_without_image(
                    samples_path, root, entry.image_path
                )
            except (OSError, ValueError) as exc:
                raise EditorModelError(str(exc)) from exc

            staged: list[tuple[Path, Path]] = []
            try:
                for path in (entry.image_path, entry.annotation_path):
                    if path is None or not path.exists():
                        continue
                    temporary = path.with_name(f".{path.name}.deleting")
                    if temporary.exists():
                        raise FileExistsError(
                            f"temporary file already exists: {temporary}"
                        )
                    path.replace(temporary)
                    staged.append((path, temporary))
            except OSError as exc:
                self._restore_staged(staged)
                raise EditorModelError(str(exc)) from exc

            try:
                save_samples(samples_path, updated_samples)
            except OSError as exc:
                self._restore_staged(staged)
                raise EditorModelError(str(exc)) from exc

            cleanup_warnings: list[str] = []
            for _original, temporary in staged:
                try:
                    temporary.unlink()
                except OSError as exc:
                    cleanup_warnings.append(str(exc))

            entries = dataset.entries
            self._dataset = replace(
                dataset, entries=entries[:index] + entries[index + 1 :]
            )
            self._generation += 1
            next_index = min(index, len(self._dataset.entries) - 1)
            return {
                "dataset": self.snapshot(),
                "next_index": next_index if next_index >= 0 else None,
                "warnings": cleanup_warnings,
            }

    @staticmethod
    def _restore_staged(staged: Iterable[tuple[Path, Path]]) -> None:
        for original, temporary in reversed(tuple(staged)):
            try:
                temporary.replace(original)
            except OSError:
                # Preserve the original failure. The staged file remains recoverable.
                pass
