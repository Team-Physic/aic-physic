import argparse
import json
import math
import random
from collections import Counter

from phy_data_collection import main, runtime, scenario
from phy_data_collection.cli import build_parser
from phy_data_collection.scenario import make_trial_config


def _args(port_type="sfp"):
    return argparse.Namespace(
        port_type=port_type,
        trials=34,
        seed=30,
        cable_rotation_noise_rad=0.04,
        time_limit_s=600,
        robot_joint_noise_rad=0.06981317007977318,
        ros_domain_id_base=40,
        zenoh_port_base=7600,
    )


def test_trials_uniformly_sample_valid_nonempty_card_combinations():
    for port_type, masks, rail_count in (("sfp", range(1, 32), 5), ("sc", range(1, 4), 2)):
        args = _args(port_type)
        observed = Counter()
        for index in range(3400):
            config, metadata_by_task = make_trial_config(
                index,
                random.Random(30 + index * 1_000_003),
                args,
            )
            task_id, metadata = next(iter(metadata_by_task.items()))
            observed[metadata["combination_mask"]] += 1
            trial = next(iter(config["trials"].values()))
            task = trial["tasks"][task_id]
            target_rail = metadata["rail_idx"]

            assert metadata["port_type"] == port_type
            assert target_rail in metadata["active_rails"]
            prefix = "nic_card_mount" if port_type == "sfp" else "sc_port"
            assert task["target_module_name"] == f"{prefix}_{target_rail}"
            rail_prefix = "nic_rail" if port_type == "sfp" else "sc_rail"
            active = [
                rail
                for rail in range(rail_count)
                if trial["scene"]["task_board"][f"{rail_prefix}_{rail}"]["entity_present"]
            ]
            assert active == metadata["active_rails"]

        assert set(observed) == set(masks)
        assert max(observed.values()) / min(observed.values()) < 2.0


def test_worker_assignment_preserves_every_global_index_once():
    assignments = [main._worker_trial_indices(34, 4, worker) for worker in range(4)]
    flattened = [index for indices in assignments for index in indices]

    assert sorted(flattened) == list(range(34))
    assert len(flattened) == len(set(flattened))


def test_cable_rotation_noise_respects_total_angle_bound():
    args = _args()
    base = scenario._quaternion_from_rpy(
        scenario.LIMITS["cable_roll"],
        scenario.LIMITS["cable_pitch"],
        scenario.LIMITS["cable_yaw"],
    )
    errors = []
    for seed in range(100):
        actual = scenario._quaternion_from_rpy(
            *scenario._cable_rpy(random.Random(seed), args)
        )
        cosine = min(1.0, abs(sum(a * b for a, b in zip(base, actual))))
        errors.append(2.0 * math.acos(cosine))

    assert max(errors) <= args.cable_rotation_noise_rad + 1e-12
    assert max(errors) > 0.0


def test_policy_environment_contains_trial_index(tmp_path):
    args = build_parser().parse_args(["--dry-run", "--port-type", "sfp"])
    args.seed = 7
    main._prepare_args(args)

    env = runtime._policy_environment(
        args,
        stop_file=tmp_path / "stop",
        run_id="run",
        trial_index=31,
        trial_split="train",
    )

    assert env["AIC_PORTOFFSET_TRIAL_INDEX"] == "31"
    assert env["AIC_COLLECT_RANDOM_SEED"] == str(7 + 31 * 1_000_003)
    assert env["AIC_PORT_COLLECT_ROLL_LIMIT_RAD"] == str(args.port_roll_limit_rad)
    assert env["AIC_COLLECT_SETTLE_ORIENTATION_TOLERANCE_RAD"] == str(
        args.settle_orientation_tolerance_rad
    )
    assert env["AIC_IMG2POS_AUTO_ANNOTATE_PORTS"] == "false"
    assert not any(name.endswith("_DEG") for name in env)


def test_auto_annotate_ports_cli_enables_policy_environment(tmp_path):
    args = build_parser().parse_args(
        [
            "--dry-run",
            "--port-type",
            "sfp",
            "--auto-annotate-ports",
            "true",
        ]
    )
    args.seed = 7
    main._prepare_args(args)

    env = runtime._policy_environment(
        args,
        stop_file=tmp_path / "stop",
        run_id="run",
    )

    assert env["AIC_IMG2POS_AUTO_ANNOTATE_PORTS"] == "true"


def test_metadata_contains_seed_and_total_trials(monkeypatch, tmp_path):
    args = build_parser().parse_args(
        ["--dry-run", "--port-type", "sfp", "--trials", "34"]
    )
    monkeypatch.setattr(main.secrets, "randbits", lambda _bits: 7)
    main._prepare_args(args)
    monkeypatch.setattr(main, "dataset_dir", lambda _args: tmp_path)

    main._write_metadata(args)

    assert json.loads((tmp_path / "metadata.jsonl").read_text(encoding="utf-8")) == {
        "seed": 7,
        "trials": 34,
    }


def test_worker_continues_after_one_trial_failure(monkeypatch):
    attempted = []

    def run_trial(ctx, index, rng):
        attempted.append(index)
        if index == 1:
            raise RuntimeError("expected failure")

    monkeypatch.setattr(main, "_run_trial", run_trial)
    monkeypatch.setattr(main, "_persist_groups", lambda ctx: None)

    result = main._run_worker(_args(), "run", 0, [0, 1, 2])

    assert attempted == [0, 1, 2]
    assert result == 1
