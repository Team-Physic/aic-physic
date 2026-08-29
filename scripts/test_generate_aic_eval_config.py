#!/usr/bin/env python3

import unittest
from pathlib import Path

import yaml

from generate_aic_eval_config import (
    NIC_TRANSLATION_MAX_M,
    NIC_TRANSLATION_MIN_M,
    generate_config,
)


ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = ROOT / "ws_aic/src/aic/aic_engine/config/eval_config.yaml"


class GenerateAicEvalConfigTest(unittest.TestCase):
    def test_seeded_sfp_trials_are_reproducible_and_valid(self) -> None:
        with TEMPLATE.open(encoding="utf-8") as stream:
            template = yaml.safe_load(stream)

        first = generate_config(template, seed=42, num_trials=20)
        second = generate_config(template, seed=42, num_trials=20)
        shorter = generate_config(template, seed=42, num_trials=3)
        self.assertEqual(first, second)
        self.assertEqual(
            list(first["trials"].values())[:3],
            list(shorter["trials"].values()),
        )
        self.assertNotIn("randomization", first)
        self.assertEqual(len(first["trials"]), 20)

        for trial in first["trials"].values():
            board = trial["scene"]["task_board"]
            task = trial["tasks"]["task_1"]
            active = {
                rail
                for rail in range(5)
                if board[f"nic_rail_{rail}"]["entity_present"]
            }
            target_rail = int(task["target_module_name"].rsplit("_", 1)[1])

            self.assertTrue(1 <= len(active) <= 5)
            self.assertIn(target_rail, active)
            self.assertIn(task["port_name"], {"sfp_port_0", "sfp_port_1"})
            for rail in active:
                pose = board[f"nic_rail_{rail}"]["entity_pose"]
                self.assertTrue(
                    NIC_TRANSLATION_MIN_M
                    <= pose["translation"]
                    <= NIC_TRANSLATION_MAX_M
                )
                self.assertEqual(
                    (pose["roll"], pose["pitch"], pose["yaw"]),
                    (0.0, 0.0, 0.0),
                )


if __name__ == "__main__":
    unittest.main()
