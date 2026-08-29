#!/usr/bin/env python3
"""Seed로 재현 가능한 SFP evaluation YAML을 생성한다."""

from __future__ import annotations

import argparse
import copy
import hashlib
import random
from pathlib import Path

import yaml


SEED_MAX = 2**32 - 1
NIC_RAILS = range(5)
NIC_TRANSLATION_MIN_M = -0.0215
NIC_TRANSLATION_MAX_M = 0.0234
GENERATOR_VERSION = 1


def derive_trial_seed(seed: int, trial_index: int) -> int:
    """base seed와 0-based trial index에서 독립적인 고정 seed를 만든다."""
    payload = f"aic-eval-v{GENERATOR_VERSION}:{seed}:{trial_index}".encode()
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")


def generate_config(template: dict, seed: int, num_trials: int) -> dict:
    """trial_1 scene을 복제해 seed가 결정하는 SFP trials를 반환한다."""
    if not 0 <= seed <= SEED_MAX:
        raise ValueError(f"seed must be between 0 and {SEED_MAX}")
    if num_trials < 1:
        raise ValueError("num_trials must be at least 1")

    try:
        base_trial = template["trials"]["trial_1"]
        base_trial["scene"]["task_board"]
        base_trial["tasks"]["task_1"]
    except (KeyError, TypeError) as exc:
        raise ValueError("template must contain trials.trial_1 SFP scene/task") from exc

    generated = copy.deepcopy(template)
    randomization = generated.get("randomization")
    if isinstance(randomization, dict):
        # 카드 조합은 이미 물질화된다. Engine의 두 번째 randomization을 막는다.
        randomization.pop("nic_cards", None)
        if not randomization:
            generated.pop("randomization")

    generated["generation"] = {
        "seed": seed,
        "num_trials": num_trials,
        "generator_version": GENERATOR_VERSION,
    }
    generated["trials"] = {}

    for trial_index in range(num_trials):
        rng = random.Random(derive_trial_seed(seed, trial_index))
        trial = copy.deepcopy(base_trial)
        task_board = trial["scene"]["task_board"]
        task = trial["tasks"]["task_1"]

        card_count = rng.randint(1, len(NIC_RAILS))
        active_rails = sorted(rng.sample(list(NIC_RAILS), card_count))
        target_rail = rng.choice(active_rails)
        target_port = rng.randint(0, 1)

        for rail in NIC_RAILS:
            rail_key = f"nic_rail_{rail}"
            if rail not in active_rails:
                task_board[rail_key] = {"entity_present": False}
                continue

            # 공식 NIC rail 범위에서 translation만 바꾸고 회전은 고정한다.
            translation = round(
                rng.uniform(NIC_TRANSLATION_MIN_M, NIC_TRANSLATION_MAX_M), 6
            )
            task_board[rail_key] = {
                "entity_present": True,
                "entity_name": f"nic_card_{rail}",
                "entity_pose": {
                    "translation": translation,
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                },
            }

        task["port_type"] = "sfp"
        task["port_name"] = f"sfp_port_{target_port}"
        task["target_module_name"] = f"nic_card_mount_{target_rail}"
        generated["trials"][f"trial_{trial_index + 1:04d}"] = trial

    return generated


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--num-trials", type=int, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    with args.template.open(encoding="utf-8") as stream:
        template = yaml.safe_load(stream)
    generated = generate_config(template, args.seed, args.num_trials)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        yaml.safe_dump(generated, stream, sort_keys=False)
    print(args.output)


if __name__ == "__main__":
    main()
