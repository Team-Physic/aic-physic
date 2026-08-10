import argparse
import json
import random

from phy_data_collection import main, runtime
from phy_data_collection.cli import build_parser
from phy_data_collection.scenario import make_trial_config, trial_identity


def _args():
    return argparse.Namespace(
        sfp_trials=31,
        sc_trials=3,
        seed=30,
        cable_rpy_noise_deg=20.0,
        time_limit_s=600,
        robot_joint_noise_deg=4.0,
        ros_domain_id_base=40,
        zenoh_port_base=7600,
    )


def test_global_trial_indices_cover_all_nonempty_card_combinations():
    args = _args()
    observed = {"sfp": [], "sc": []}

    for index in range(args.sfp_trials + args.sc_trials):
        config, metadata_by_task = make_trial_config(
            index,
            random.Random(30 + index * 1_000_003),
            args,
        )
        task_id, metadata = next(iter(metadata_by_task.items()))
        port_type = metadata["port_type"]
        observed[port_type].append(metadata["combination_mask"])
        trial = next(iter(config["trials"].values()))
        task = trial["tasks"][task_id]
        target_rail = metadata["rail_idx"]

        assert target_rail in metadata["active_rails"]
        if port_type == "sfp":
            assert task["target_module_name"] == f"nic_card_mount_{target_rail}"
            active = [
                rail
                for rail in range(5)
                if trial["scene"]["task_board"][f"nic_rail_{rail}"]["entity_present"]
            ]
        else:
            assert task["target_module_name"] == f"sc_port_{target_rail}"
            active = [
                rail
                for rail in range(2)
                if trial["scene"]["task_board"][f"sc_rail_{rail}"]["entity_present"]
            ]
        assert active == metadata["active_rails"]

    assert observed["sfp"] == list(range(1, 32))
    assert observed["sc"] == list(range(1, 4))


def test_worker_assignment_preserves_every_global_index_once():
    assignments = [main._worker_trial_indices(34, 4, worker) for worker in range(4)]
    flattened = [index for indices in assignments for index in indices]

    assert sorted(flattened) == list(range(34))
    assert len(flattened) == len(set(flattened))


def test_combination_identity_is_independent_of_seed():
    assert trial_identity(0, 31) == ("sfp", 0, 1, "00001")
    assert trial_identity(30, 31) == ("sfp", 30, 31, "11111")
    assert trial_identity(31, 31) == ("sc", 0, 1, "01")
    assert trial_identity(33, 31) == ("sc", 2, 3, "11")


def test_policy_environment_contains_trial_index(tmp_path):
    args = build_parser().parse_args(["--dry-run", "--seed", "7"])
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


def test_metadata_contains_seed_and_total_trials(monkeypatch, tmp_path):
    args = build_parser().parse_args(
        ["--dry-run", "--seed", "7", "--sfp-trials", "31", "--sc-trials", "3"]
    )
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
