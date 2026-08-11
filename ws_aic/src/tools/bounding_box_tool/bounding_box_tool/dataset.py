"""YOLO-pose dataset discovery, parsing, and saving."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

import yaml

IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png"}
SAMPLE_CAMERA_FIELDS = (
    "images",
    "annotations",
    "annotation_object_counts",
    "annotation_labels",
)


@dataclass(frozen=True)
class PoseAnnotation:
    """정규화된 YOLO-pose 객체 한 행."""

    class_id: int
    label: str
    bbox: tuple[float, float, float, float]
    keypoints: tuple[
        tuple[float, float, int],
        tuple[float, float, int],
        tuple[float, float, int],
        tuple[float, float, int],
    ]


@dataclass(frozen=True)
class ImageEntry:
    """표시할 image와 대응 annotation 경로."""

    image_path: Path
    annotation_path: Path | None
    display_path: str


@dataclass(frozen=True)
class ImageDataset:
    """선택한 경로에서 발견한 image와 class 이름."""

    selected_path: Path
    dataset_root: Path | None
    entries: tuple[ImageEntry, ...]
    class_names: dict[int, str]

    @classmethod
    def open(cls, selected_path: str | Path) -> "ImageDataset":
        """dataset root, image directory 또는 단일 image를 연다."""
        selected = Path(selected_path).expanduser().resolve()
        if not selected.exists():
            raise FileNotFoundError(f"path does not exist: {selected}")
        root = find_dataset_root(selected)
        image_base = _image_search_base(selected, root)
        image_paths = _discover_images(image_base)
        if not image_paths:
            raise ValueError(f"no supported images found under: {image_base}")
        display_base = image_base.parent if image_base.is_file() else image_base
        entries = tuple(
            ImageEntry(
                image_path=image_path,
                annotation_path=annotation_path_for(image_path, root),
                display_path=_display_path(image_path, display_base),
            )
            for image_path in image_paths
        )
        return cls(
            selected_path=selected,
            dataset_root=root,
            entries=entries,
            class_names=load_class_names(root),
        )


def _natural_key(path: Path) -> tuple:
    """숫자 구간을 정수로 비교하는 안정적인 path 정렬 key를 반환한다."""
    return tuple(
        (0, int(token)) if token.isdigit() else (1, token.lower())
        for token in re.split(r"(\d+)", str(path))
    )


def _discover_images(path: Path) -> tuple[Path, ...]:
    """단일 파일 또는 directory 아래 지원 image를 재귀적으로 찾는다."""
    if path.is_file():
        return (path,) if path.suffix.lower() in IMAGE_SUFFIXES else ()
    return tuple(
        sorted(
            (
                candidate
                for candidate in path.rglob("*")
                if candidate.is_file() and candidate.suffix.lower() in IMAGE_SUFFIXES
            ),
            key=_natural_key,
        )
    )


def find_dataset_root(path: Path) -> Path | None:
    """yolo_pose.yaml 또는 images directory를 기준으로 dataset root를 찾는다."""
    start = path.parent if path.is_file() else path
    for candidate in (start, *start.parents):
        if (candidate / "yolo_pose.yaml").is_file():
            return candidate
    for candidate in (start, *start.parents):
        if candidate.name == "images":
            return candidate.parent
    return None


def _image_search_base(selected: Path, root: Path | None) -> Path:
    """dataset root를 선택했으면 images만, 그 외에는 선택 경로만 검색한다."""
    if selected.is_dir() and root == selected and (selected / "images").is_dir():
        return selected / "images"
    return selected


def _display_path(image_path: Path, base: Path) -> str:
    """image list에 표시할 가능한 한 짧은 상대 경로를 반환한다."""
    try:
        return str(image_path.relative_to(base))
    except ValueError:
        return image_path.name


def annotation_path_for(image_path: Path, root: Path | None) -> Path | None:
    """images 상대 경로를 annotations 또는 labels TXT 경로로 변환한다."""
    if root is None:
        return None
    try:
        relative = image_path.relative_to(root / "images")
    except ValueError:
        return None
    annotation_root = root / "annotations"
    if not annotation_root.exists() and (root / "labels").exists():
        annotation_root = root / "labels"
    return (annotation_root / relative).with_suffix(".txt")


def load_class_names(root: Path | None) -> dict[int, str]:
    """yolo_pose.yaml의 names mapping을 정수 class ID로 읽는다."""
    if root is None:
        return {}
    config_path = root / "yolo_pose.yaml"
    if not config_path.is_file():
        return {}
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    names = data.get("names", {})
    if isinstance(names, list):
        return {index: str(name) for index, name in enumerate(names)}
    if isinstance(names, dict):
        result = {}
        for class_id, name in names.items():
            try:
                result[int(class_id)] = str(name)
            except (TypeError, ValueError):
                continue
        return result
    return {}


def load_annotations(
    path: Path | None,
    class_names: dict[int, str],
) -> tuple[tuple[PoseAnnotation, ...], tuple[str, ...]]:
    """YOLO-pose TXT를 읽고 잘못된 행은 warning으로 반환한다."""
    if path is None or not path.is_file():
        return (), ()
    annotations = []
    warnings = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        fields = line.split()
        if not fields:
            continue
        if len(fields) != 17:
            warnings.append(f"line {line_number}: expected 17 fields, got {len(fields)}")
            continue
        try:
            class_id = int(fields[0])
            values = [float(value) for value in fields[1:]]
            bbox = tuple(values[:4])
            keypoints = tuple(
                (values[index], values[index + 1], int(values[index + 2]))
                for index in range(4, 16, 3)
            )
        except (TypeError, ValueError) as exc:
            warnings.append(f"line {line_number}: invalid number ({exc})")
            continue
        coordinates = [*bbox]
        coordinates.extend(
            coordinate
            for x, y, _visibility in keypoints
            for coordinate in (x, y)
        )
        if any(coordinate < 0.0 or coordinate > 1.0 for coordinate in coordinates):
            warnings.append(f"line {line_number}: normalized coordinate outside [0, 1]")
            continue
        annotations.append(
            PoseAnnotation(
                class_id=class_id,
                label=class_names.get(class_id, f"class_{class_id}"),
                bbox=bbox,
                keypoints=keypoints,
            )
        )
    return tuple(annotations), tuple(warnings)


def annotation_row(annotation: PoseAnnotation) -> str:
    """annotation을 17-field YOLO-pose 행으로 직렬화한다."""
    values = [str(annotation.class_id)]
    values.extend(f"{value:.9f}" for value in annotation.bbox)
    for x, y, visibility in annotation.keypoints:
        values.extend((f"{x:.9f}", f"{y:.9f}", str(int(visibility))))
    return " ".join(values)


def save_annotations(path: Path, annotations: tuple[PoseAnnotation, ...]) -> None:
    """annotation을 대응 TXT에 원자적으로 저장한다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    contents = "\n".join(annotation_row(annotation) for annotation in annotations)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(contents + ("\n" if contents else ""), encoding="utf-8")
    temporary.replace(path)


def samples_without_image(
    path: Path,
    dataset_root: Path,
    image_path: Path,
) -> str:
    """samples JSONL에서 현재 camera view를 제거한 전체 내용을 반환한다."""
    relative_image = image_path.relative_to(dataset_root).as_posix()
    updated_lines = []
    matches = 0
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), 1
    ):
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path.name} line {line_number}: {exc}") from exc
        images = row.get("images")
        cameras = (
            [camera for camera, value in images.items() if value == relative_image]
            if isinstance(images, dict)
            else []
        )
        if not cameras:
            updated_lines.append(line)
            continue
        matches += len(cameras)
        for camera in cameras:
            for field in SAMPLE_CAMERA_FIELDS:
                values = row.get(field)
                if isinstance(values, dict):
                    values.pop(camera, None)
        if row["images"]:
            updated_lines.append(json.dumps(row, ensure_ascii=False))
    if matches != 1:
        raise ValueError(
            f"expected one samples.jsonl entry for {relative_image}, found {matches}"
        )
    return "\n".join(updated_lines) + ("\n" if updated_lines else "")


def save_samples(path: Path, contents: str) -> None:
    """검증된 samples JSONL 전체를 원자적으로 교체한다."""
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(contents, encoding="utf-8")
    temporary.replace(path)
