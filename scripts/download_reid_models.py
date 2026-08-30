#!/usr/bin/env python3
"""EfficientNet-B0와 DINOv3 ViT-S/16을 저장소 상대 경로에 미리 받는다."""

from __future__ import annotations

import argparse
import hashlib
import urllib.request
from pathlib import Path

from tqdm import tqdm


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
EFFICIENTNET_URL = (
    "https://download.pytorch.org/models/efficientnet_b0_rwightman-7f5810bc.pth"
)
EFFICIENTNET_SHA256_PREFIX = "7f5810bc"
DINO_REPOSITORY = "facebook/dinov3-vits16-pretrain-lvd1689m"


def resolve_output_dir(relative_path: str) -> Path:
    """출력 경로를 저장소 내부로 제한한다."""
    requested = Path(relative_path)
    if requested.is_absolute():
        raise ValueError("--output-dir must be relative to the repository root")
    resolved = (REPOSITORY_ROOT / requested).resolve()
    if not resolved.is_relative_to(REPOSITORY_ROOT):
        raise ValueError("--output-dir must stay inside the repository")
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def download_efficientnet(output_dir: Path) -> Path:
    target = output_dir / "efficientnet_b0" / Path(EFFICIENTNET_URL).name
    if target.exists() and sha256(target).startswith(EFFICIENTNET_SHA256_PREFIX):
        return target

    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".part")
    try:
        with urllib.request.urlopen(EFFICIENTNET_URL) as response, temporary.open("wb") as file:
            total = int(response.headers.get("Content-Length", 0)) or None
            with tqdm(
                total=total,
                desc="EfficientNet-B0 (PyTorch)",
                unit="B",
                unit_scale=True,
            ) as progress:
                for chunk in iter(lambda: response.read(1024 * 1024), b""):
                    file.write(chunk)
                    progress.update(len(chunk))
        if not sha256(temporary).startswith(EFFICIENTNET_SHA256_PREFIX):
            raise RuntimeError("EfficientNet-B0 checksum mismatch")
        temporary.replace(target)
    finally:
        temporary.unlink(missing_ok=True)
    return target


def download_dinov3(output_dir: Path) -> Path:
    try:
        from huggingface_hub import snapshot_download
    except ImportError as error:
        raise RuntimeError("install huggingface-hub before downloading DINOv3") from error

    target = output_dir / "dinov3_vits16"
    try:
        # snapshot_download가 Meta repository의 파일별 tqdm 진행률과 재시작을 처리한다.
        snapshot_download(repo_id=DINO_REPOSITORY, local_dir=target)
    except Exception as error:
        raise RuntimeError(
            "DINOv3 access failed; accept its Hugging Face terms, then run `hf auth login`"
        ) from error
    return target


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        default="models/reid",
        help="repository-relative destination (default: models/reid)",
    )
    parser.add_argument(
        "--model",
        choices=("all", "efficientnet", "dinov3"),
        default="all",
    )
    args = parser.parse_args()
    output_dir = resolve_output_dir(args.output_dir)

    if args.model in {"all", "efficientnet"}:
        print(f"EfficientNet-B0: {download_efficientnet(output_dir).relative_to(REPOSITORY_ROOT)}")
    if args.model in {"all", "dinov3"}:
        print(f"DINOv3 ViT-S/16: {download_dinov3(output_dir).relative_to(REPOSITORY_ROOT)}")


if __name__ == "__main__":
    main()
