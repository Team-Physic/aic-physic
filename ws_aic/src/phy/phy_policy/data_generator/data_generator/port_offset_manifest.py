from __future__ import annotations

"""Dataset manifest writer for PortOffsetCollect."""


def _write_img2pos_data_yaml(self) -> None:
    """compact img2pos 데이터셋의 공통 schema를 data.yaml에 기록한다."""
    self._rpy_dataset_dir.mkdir(parents=True, exist_ok=True)
    (self._rpy_dataset_dir / "data.yaml").write_text(
        "\n".join(
            [
                "schema_version: 1",
                "task: img2pos",
                f"version: {self._rpy_dataset_version or 'default'}",
                "input: rgb_image",
                "samples: samples.jsonl",
                "image_layout: images/<split>/<camera>/*.jpg",
                "cameras: [left, center, right]",
                "target:",
                "  name: correction_xyz",
                "  definition: port_entrance - plug_reference",
                "  frame: base_link",
                "  unit: meter",
                "split:",
                "  group_by: trial_id",
                f"  validation_ratio: {self._rpy_val_ratio:.6f}",
                "",
            ]
        ),
        encoding="utf-8",
    )
