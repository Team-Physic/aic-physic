"""RGB 기반 robot-arm occlusion 후처리."""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import numpy as np

from bounding_box_tool.dataset import PoseAnnotation

ROBOT_ARM_DARK_THRESHOLD = 40
ROBOT_ARM_MIN_AREA_RATIO = 0.01
ROBOT_ARM_MASK_DILATION_PX = 8
FULL_OCCLUSION_RATIO = 0.90


@dataclass(frozen=True)
class VisibilityResult:
    """현재 image의 자동 visibility 판정 결과."""

    annotations: tuple[PoseAnnotation, ...]
    deleted_objects: int
    occluded_keypoints: int
    mask_pixels: int


def robot_arm_mask(image: np.ndarray) -> np.ndarray:
    """영상 아래쪽에 연결된 큰 검은 영역을 robot arm mask로 반환한다."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dark = (gray <= ROBOT_ARM_DARK_THRESHOLD).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(dark, connectivity=8)
    height, width = gray.shape
    minimum_area = int(height * width * ROBOT_ARM_MIN_AREA_RATIO)
    mask = np.zeros_like(dark)
    for label in range(1, count):
        _, y, _, component_height, area = stats[label]
        if y + component_height < height or area < minimum_area:
            continue
        component = np.where(labels == label, 255, 0).astype(np.uint8)
        contours, _ = cv2.findContours(
            component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(mask, contours, -1, 1, thickness=cv2.FILLED)
    if np.any(mask):
        size = ROBOT_ARM_MASK_DILATION_PX * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        mask = cv2.dilate(mask, kernel)
    return mask.astype(bool)


def _pixel_point(
    x: float,
    y: float,
    width: int,
    height: int,
) -> tuple[int, int]:
    return (
        int(np.clip(round(x * width), 0, width - 1)),
        int(np.clip(round(y * height), 0, height - 1)),
    )


def _annotation_polygon(
    annotation: PoseAnnotation,
    width: int,
    height: int,
) -> np.ndarray:
    return np.asarray(
        [_pixel_point(x, y, width, height) for x, y, _ in annotation.keypoints],
        dtype=np.int32,
    )


def _occlusion_ratio(
    mask: np.ndarray,
    annotation: PoseAnnotation,
) -> float:
    height, width = mask.shape
    polygon_mask = np.zeros(mask.shape, dtype=np.uint8)
    cv2.fillPoly(
        polygon_mask,
        [_annotation_polygon(annotation, width, height)],
        1,
    )
    area = int(np.count_nonzero(polygon_mask))
    if area == 0:
        return 0.0
    return float(np.count_nonzero(mask & polygon_mask.astype(bool)) / area)


def apply_auto_visibility(
    image_path: str | Path,
    annotations: tuple[PoseAnnotation, ...],
    *,
    full_occlusion_ratio: float = FULL_OCCLUSION_RATIO,
) -> VisibilityResult:
    """완전 가림 객체는 제거하고 부분 가림 keypoint는 visibility=1로 바꾼다."""
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"cannot read image: {image_path}")
    mask = robot_arm_mask(image)
    height, width = mask.shape
    updated = []
    deleted_objects = 0
    occluded_keypoints = 0
    for annotation in annotations:
        if _occlusion_ratio(mask, annotation) >= full_occlusion_ratio:
            deleted_objects += 1
            continue
        keypoints = []
        for x, y, _visibility in annotation.keypoints:
            pixel_x, pixel_y = _pixel_point(x, y, width, height)
            visibility = 1 if mask[pixel_y, pixel_x] else 2
            occluded_keypoints += int(visibility == 1)
            keypoints.append((x, y, visibility))
        updated.append(replace(annotation, keypoints=tuple(keypoints)))
    return VisibilityResult(
        annotations=tuple(updated),
        deleted_objects=deleted_objects,
        occluded_keypoints=occluded_keypoints,
        mask_pixels=int(np.count_nonzero(mask)),
    )
