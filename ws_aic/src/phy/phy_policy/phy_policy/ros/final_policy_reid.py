"""FinalPolicy recovery용 frozen appearance encoder와 keypoint descriptor."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np


REPOSITORY_ROOT = next(
    (
        parent
        for parent in Path(__file__).resolve().parents
        if (parent / ".git").exists()
    ),
    Path.cwd(),
)
DEFAULT_MODEL_ROOT = Path("models/reid")
EFFICIENTNET_FILENAME = "efficientnet_b0_rwightman-7f5810bc.pth"


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except ValueError:
        return default


def _normalize(vector: np.ndarray) -> np.ndarray:
    value = np.asarray(vector, dtype=np.float32)
    norm = float(np.linalg.norm(value))
    return value / norm if norm > 1e-12 else value


def model_root() -> Path:
    """상대 model 경로는 repository root 기준으로 해석한다."""
    configured = Path(os.environ.get("AIC_REID_MODEL_DIR", DEFAULT_MODEL_ROOT))
    return configured if configured.is_absolute() else REPOSITORY_ROOT / configured


def cosine_similarity(left: np.ndarray, right: np.ndarray) -> float:
    """두 descriptor의 cosine similarity를 반환한다."""
    return float(np.dot(_normalize(left), _normalize(right)))


def bilinear_descriptor(feature_map: np.ndarray, point_xy: np.ndarray) -> np.ndarray:
    """Feature-map 실수 좌표의 인접 네 cell을 bilinear interpolation한다."""
    height, width, _ = feature_map.shape
    x = float(np.clip(point_xy[0], 0.0, max(0, width - 1)))
    y = float(np.clip(point_xy[1], 0.0, max(0, height - 1)))
    x0, y0 = int(np.floor(x)), int(np.floor(y))
    x1, y1 = min(x0 + 1, width - 1), min(y0 + 1, height - 1)
    wx, wy = x - x0, y - y0
    descriptor = (
        (1.0 - wx) * (1.0 - wy) * feature_map[y0, x0]
        + wx * (1.0 - wy) * feature_map[y0, x1]
        + (1.0 - wx) * wy * feature_map[y1, x0]
        + wx * wy * feature_map[y1, x1]
    )
    return _normalize(descriptor)


def keypoints_to_feature_map(
    keypoints: np.ndarray,
    image_shape: tuple[int, int],
    feature_shape: tuple[int, int],
) -> np.ndarray:
    """입력 pixel 중심을 feature cell 중심 좌표로 변환한다."""
    image_h, image_w = image_shape
    feature_h, feature_w = feature_shape
    points = np.asarray(keypoints, dtype=np.float32).copy()
    points[:, 0] = (points[:, 0] + 0.5) * feature_w / image_w - 0.5
    points[:, 1] = (points[:, 1] + 0.5) * feature_h / image_h - 0.5
    return points


def support_mask(
    feature_shape: tuple[int, int],
    feature_points: np.ndarray,
    mode: str,
) -> np.ndarray:
    """Keypoint bbox 또는 convex hull 내부의 feature cell mask를 만든다."""
    height, width = feature_shape
    points = np.asarray(feature_points, dtype=np.float32)
    mask = np.zeros((height, width), dtype=np.uint8)
    if mode == "convex_hull":
        polygon = np.rint(points).astype(np.int32)
        polygon[:, 0] = np.clip(polygon[:, 0], 0, width - 1)
        polygon[:, 1] = np.clip(polygon[:, 1], 0, height - 1)
        cv2.fillConvexPoly(mask, cv2.convexHull(polygon), 1)
    else:
        x0 = int(np.clip(np.floor(points[:, 0].min()), 0, width - 1))
        x1 = int(np.clip(np.ceil(points[:, 0].max()), 0, width - 1))
        y0 = int(np.clip(np.floor(points[:, 1].min()), 0, height - 1))
        y1 = int(np.clip(np.ceil(points[:, 1].max()), 0, height - 1))
        mask[y0 : y1 + 1, x0 : x1 + 1] = 1
    return mask.astype(bool)


@dataclass(frozen=True)
class AppearanceDescriptor:
    global_descriptor: np.ndarray
    local_descriptors: tuple[np.ndarray, ...]
    local_confidences: tuple[float, ...] = (1.0, 1.0, 1.0, 1.0)


class AppearanceReID:
    """한 encoder를 고정해 initial target과 recovery 후보를 비교한다."""

    VALID_ENCODERS = {"none", "efficientnet_b0", "dinov3_vits16"}
    VALID_ATTENTION = {"none", "keypoint", "convex_hull"}

    def __init__(self, logger):
        self.logger = logger
        self.encoder_name = os.environ.get("AIC_REID_ENCODER", "none").strip().lower()
        self.attention_mode = os.environ.get("AIC_REID_ATTENTION_MODE", "none").strip().lower()
        self.match_threshold = _env_float("AIC_REID_MATCH_THRESHOLD", 0.75)
        self.global_weight = float(np.clip(_env_float("AIC_REID_GLOBAL_WEIGHT", 0.5), 0.0, 1.0))
        self.input_width = max(64, _env_int("AIC_REID_INPUT_WIDTH", 384))
        self.warmup_runs = max(0, _env_int("AIC_REID_WARMUP_RUNS", 5))
        self.attention_region_frac = float(
            np.clip(_env_float("AIC_REID_ATTENTION_REGION_FRAC", 0.25), 0.01, 1.0)
        )
        self.device = "cpu"
        self.model: Any = None
        self.processor: Any = None
        self.stride = 32
        self.memory: AppearanceDescriptor | None = None

    @property
    def enabled(self) -> bool:
        return self.encoder_name != "none"

    def load(self) -> None:
        """선택 encoder 하나만 load하고 representative dummy frame으로 warmup한다."""
        if self.encoder_name not in self.VALID_ENCODERS:
            raise ValueError(f"unsupported AIC_REID_ENCODER={self.encoder_name!r}")
        if self.attention_mode not in self.VALID_ATTENTION:
            raise ValueError(
                f"unsupported AIC_REID_ATTENTION_MODE={self.attention_mode!r}"
            )
        if not self.enabled:
            return

        import torch

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        local_root = model_root()
        if self.encoder_name == "efficientnet_b0":
            if self.attention_mode != "none":
                raise ValueError("EfficientNet-B0 does not expose transformer attention maps")
            from torchvision.models import efficientnet_b0

            local_weights = local_root / "efficientnet_b0" / EFFICIENTNET_FILENAME
            if not local_weights.is_file():
                raise FileNotFoundError(
                    f"missing EfficientNet-B0 weights: {local_weights}; "
                    "run scripts/download_reid_models.py first"
                )
            self.model = efficientnet_b0(weights=None)
            self.model.load_state_dict(
                torch.load(local_weights, map_location="cpu", weights_only=True)
            )
            source = str(local_weights)
            self.model = self.model.features.to(self.device).eval()
            self.stride = 32
        else:
            from transformers import AutoImageProcessor, AutoModel

            configured_id = os.environ.get("AIC_REID_DINO_MODEL_ID", "").strip()
            local_model = local_root / "dinov3_vits16"
            if not configured_id and not (local_model / "config.json").is_file():
                raise FileNotFoundError(
                    f"missing DINOv3 model: {local_model}; "
                    "run scripts/download_reid_models.py first"
                )
            model_id = configured_id or str(local_model)
            local_only = Path(model_id).is_dir()
            self.processor = AutoImageProcessor.from_pretrained(
                model_id, local_files_only=local_only
            )
            model_arguments = {
                "low_cpu_mem_usage": True,
                "local_files_only": local_only,
            }
            if self.attention_mode != "none":
                # Attention weight를 반환하지 않는 optimized kernel 대신 QK softmax를 보존한다.
                model_arguments["attn_implementation"] = "eager"
            self.model = AutoModel.from_pretrained(
                model_id, **model_arguments
            ).to(self.device).eval()
            self.stride = int(getattr(self.model.config, "patch_size", 16))
            source = model_id

        # Frozen benchmark이므로 autograd state까지 명시적으로 끈다.
        for parameter in self.model.parameters():
            parameter.requires_grad_(False)

        # Model load만으로는 첫 CUDA kernel과 allocator 지연이 사라지지 않는다.
        dummy_h = max(self.stride, int(round(self.input_width * 768 / 864)))
        dummy = np.zeros((dummy_h, self.input_width, 3), dtype=np.uint8)
        for _ in range(self.warmup_runs):
            self._extract_feature_map(dummy)
        self.logger.info(
            "FinalPolicy ReID ready: "
            f"encoder={self.encoder_name}, attention={self.attention_mode}, "
            f"device={self.device}, warmup={self.warmup_runs}, source={source}"
        )

    def _resize(self, image_bgr: np.ndarray) -> np.ndarray:
        height, width = image_bgr.shape[:2]
        scale = self.input_width / max(1, width)
        resized_h = max(self.stride, int(round(height * scale / self.stride)) * self.stride)
        resized_w = max(self.stride, int(round(self.input_width / self.stride)) * self.stride)
        return cv2.resize(image_bgr, (resized_w, resized_h), interpolation=cv2.INTER_AREA)

    def _extract_feature_map(
        self, image_bgr: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray | None, tuple[int, int]]:
        import torch

        resized = self._resize(image_bgr)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        with torch.inference_mode():
            if self.encoder_name == "efficientnet_b0":
                tensor = torch.from_numpy(rgb).to(self.device).permute(2, 0, 1)
                tensor = tensor.unsqueeze(0).float().div_(255.0)
                mean = torch.tensor((0.485, 0.456, 0.406), device=self.device).view(1, 3, 1, 1)
                std = torch.tensor((0.229, 0.224, 0.225), device=self.device).view(1, 3, 1, 1)
                fmap = self.model((tensor - mean) / std)[0].permute(1, 2, 0)
                return fmap.cpu().float().numpy(), None, resized.shape[:2]

            batch = self.processor(
                images=rgb,
                return_tensors="pt",
                do_resize=False,
                do_center_crop=False,
            )
            batch = {key: value.to(self.device) for key, value in batch.items()}
            want_attention = self.attention_mode != "none"
            outputs = self.model(
                **batch,
                output_attentions=want_attention,
                return_dict=True,
            )
            hp = batch["pixel_values"].shape[-2] // self.stride
            wp = batch["pixel_values"].shape[-1] // self.stride
            patch_count = hp * wp
            tokens = outputs.last_hidden_state
            special_count = tokens.shape[1] - patch_count
            fmap = tokens[0, special_count:, :].reshape(hp, wp, -1)
            attention = None
            if want_attention:
                if not outputs.attentions:
                    raise RuntimeError(
                        "DINOv3 did not return attention weights with eager attention"
                    )
                layers = [
                    layer[0, :, special_count:, special_count:].float()
                    for layer in outputs.attentions
                ]
                attention = torch.stack(layers).mean(dim=(0, 1)).cpu().numpy()
            return fmap.cpu().float().numpy(), attention, resized.shape[:2]

    def _descriptor_from_map(
        self,
        feature_map: np.ndarray,
        attention: np.ndarray | None,
        resized_shape: tuple[int, int],
        original_shape: tuple[int, int],
        keypoints: np.ndarray,
        keypoint_confidences: np.ndarray | None = None,
    ) -> AppearanceDescriptor:
        scale_x = resized_shape[1] / original_shape[1]
        scale_y = resized_shape[0] / original_shape[0]
        resized_points = np.asarray(keypoints, dtype=np.float32) * (scale_x, scale_y)
        feature_points = keypoints_to_feature_map(
            resized_points, resized_shape, feature_map.shape[:2]
        )
        mode = "convex_hull" if self.attention_mode == "convex_hull" else "bbox"
        support = support_mask(feature_map.shape[:2], feature_points, mode)
        selected = feature_map[support]
        global_descriptor = _normalize(
            selected.mean(axis=0) if len(selected) else feature_map.mean(axis=(0, 1))
        )

        if attention is None:
            local = tuple(
                bilinear_descriptor(feature_map, point) for point in feature_points
            )
            confidences = tuple(
                float(value)
                for value in (
                    np.ones(len(local), dtype=np.float32)
                    if keypoint_confidences is None
                    else np.asarray(keypoint_confidences, dtype=np.float32)[: len(local)]
                )
            )
            return AppearanceDescriptor(global_descriptor, local, confidences)

        flat_features = feature_map.reshape(-1, feature_map.shape[-1])
        support_indices = np.flatnonzero(support.reshape(-1))
        local_descriptors = []
        for point in feature_points:
            px = int(np.clip(round(float(point[0])), 0, feature_map.shape[1] - 1))
            py = int(np.clip(round(float(point[1])), 0, feature_map.shape[0] - 1))
            seed_index = py * feature_map.shape[1] + px
            weights = attention[seed_index, support_indices].astype(np.float32)
            count = min(
                len(support_indices),
                max(1, int(np.ceil(self.attention_region_frac * len(support_indices)))),
            )
            top = np.argpartition(weights, -count)[-count:]
            region_indices = support_indices[top]
            region_weights = np.maximum(weights[top], 0.0)
            weight_sum = float(region_weights.sum())
            descriptor = (
                np.average(flat_features[region_indices], axis=0, weights=region_weights)
                if weight_sum > 1e-12
                else flat_features[region_indices].mean(axis=0)
            )
            local_descriptors.append(_normalize(descriptor))
        confidences = tuple(
            float(value)
            for value in (
                np.ones(len(local_descriptors), dtype=np.float32)
                if keypoint_confidences is None
                else np.asarray(keypoint_confidences, dtype=np.float32)[
                    : len(local_descriptors)
                ]
            )
        )
        return AppearanceDescriptor(
            global_descriptor, tuple(local_descriptors), confidences
        )

    @staticmethod
    def _view(candidate) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        cameras = ("center", "left", "right")
        for camera in cameras:
            detection = candidate.detections.get(camera)
            image = candidate.images.get(camera)
            if detection is not None and image is not None:
                return (
                    image,
                    np.asarray(detection["keypoints"], dtype=np.float32),
                    np.asarray(
                        detection.get("keypoint_confidence", np.ones(4)),
                        dtype=np.float32,
                    ),
                )
        return None

    def describe(self, candidate) -> AppearanceDescriptor | None:
        view = self._view(candidate)
        if view is None:
            return None
        image, keypoints, confidences = view
        feature_map, attention, resized_shape = self._extract_feature_map(image)
        return self._descriptor_from_map(
            feature_map,
            attention,
            resized_shape,
            image.shape[:2],
            keypoints,
            confidences,
        )

    def remember(self, candidate) -> bool:
        """Exact-class initial lock만 target prototype으로 저장한다."""
        if not self.enabled:
            return True
        self.memory = self.describe(candidate)
        return self.memory is not None

    def score(self, descriptor: AppearanceDescriptor) -> float:
        if self.memory is None:
            return float("-inf")
        global_score = cosine_similarity(
            descriptor.global_descriptor, self.memory.global_descriptor
        )
        local_scores = [
            cosine_similarity(current, reference)
            for current, reference in zip(
                descriptor.local_descriptors, self.memory.local_descriptors
            )
        ]
        current_confidence = np.asarray(
            descriptor.local_confidences[: len(local_scores)], dtype=np.float32
        )
        reference_confidence = np.asarray(
            self.memory.local_confidences[: len(local_scores)], dtype=np.float32
        )
        weights = np.maximum(current_confidence * reference_confidence, 0.0)
        local_score = (
            float(np.average(local_scores, weights=weights))
            if local_scores and float(weights.sum()) > 1e-12
            else global_score
        )
        return self.global_weight * global_score + (1.0 - self.global_weight) * local_score

    def select(self, candidates: list) -> Any | None:
        """Geometry 후보 중 target memory threshold를 통과한 최고 후보를 반환한다."""
        if not self.enabled or self.memory is None:
            return candidates[0] if candidates else None
        shared_camera = next(
            (
                camera
                for camera in ("center", "left", "right")
                if all(
                    candidate.detections.get(camera) is not None
                    and candidate.images.get(camera) is not None
                    for candidate in candidates
                )
            ),
            None,
        )
        if shared_camera is None:
            return None

        # 같은 frame의 후보마다 encoder를 다시 돌리지 않고 dense map을 공유한다.
        image = candidates[0].images[shared_camera]
        feature_map, attention, resized_shape = self._extract_feature_map(image)
        scored = []
        for candidate in candidates:
            keypoints = np.asarray(
                candidate.detections[shared_camera]["keypoints"], dtype=np.float32
            )
            confidences = np.asarray(
                candidate.detections[shared_camera].get(
                    "keypoint_confidence", np.ones(4)
                ),
                dtype=np.float32,
            )
            descriptor = self._descriptor_from_map(
                feature_map,
                attention,
                resized_shape,
                image.shape[:2],
                keypoints,
                confidences,
            )
            scored.append((self.score(descriptor), candidate))
        if not scored:
            return None
        score, candidate = max(scored, key=lambda item: item[0])
        if score < self.match_threshold:
            return None
        candidate.reid_score = float(score)
        return candidate
