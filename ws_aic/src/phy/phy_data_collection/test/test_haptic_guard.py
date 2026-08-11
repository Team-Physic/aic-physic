from __future__ import annotations

from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion

from phy_data_collection.policy import motion
from phy_data_collection.runner.cli import build_parser


def _vector(values: tuple[float, float, float]) -> SimpleNamespace:
    return SimpleNamespace(x=values[0], y=values[1], z=values[2])


def _stamp(nanoseconds: int | None) -> SimpleNamespace | None:
    if nanoseconds is None:
        return None
    return SimpleNamespace(
        sec=nanoseconds // 1_000_000_000,
        nanosec=nanoseconds % 1_000_000_000,
    )


def _pose(
    *,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> Pose:
    return Pose(
        position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
        orientation=Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3],
        ),
    )


def _observation(
    force: tuple[float, float, float],
    stamp_ns: int | None,
    *,
    tare: tuple[float, float, float] = (0.0, 0.0, 0.0),
    pose: Pose | None = None,
) -> SimpleNamespace:
    return SimpleNamespace(
        wrist_wrench=SimpleNamespace(
            header=SimpleNamespace(stamp=_stamp(stamp_ns)),
            wrench=SimpleNamespace(force=_vector(force)),
        ),
        controller_state=SimpleNamespace(
            fts_tare_offset=SimpleNamespace(
                wrench=SimpleNamespace(force=_vector(tare))
            ),
            tcp_pose=pose or _pose(),
        ),
    )


class _Logger:
    def __init__(self) -> None:
        self.info_messages: list[str] = []
        self.error_messages: list[str] = []

    def info(self, message: str) -> None:
        self.info_messages.append(message)

    def error(self, message: str) -> None:
        self.error_messages.append(message)


class _Policy:
    def __init__(self) -> None:
        self.haptic_baseline_timeout_s = 0.1
        self.haptic_baseline_samples = 3
        self.haptic_force_threshold_n = 5.0
        self.haptic_contact_duration_s = 0.2
        self.settle_poll_s = 0.0
        self.logger = _Logger()
        self.commanded_poses: list[Pose] = []

    def sleep_for(self, _seconds: float) -> None:
        pass

    def get_logger(self) -> _Logger:
        return self.logger

    def set_pose_target(
        self,
        _move_robot,
        pose: Pose,
        _stiffness,
        _damping,
    ) -> int:
        self.commanded_poses.append(motion._copy_pose(pose))
        return 0


def test_wrist_force_subtracts_tare_and_rotates_to_base() -> None:
    half_sqrt = float(np.sqrt(0.5))
    observation = _observation(
        (3.0, 0.0, 0.0),
        1_000_000_000,
        tare=(1.0, 0.0, 0.0),
        pose=_pose(quaternion=(0.0, 0.0, half_sqrt, half_sqrt)),
    )

    force, stamp_ns = motion._wrist_force_in_base(observation)

    np.testing.assert_allclose(force, [0.0, 2.0, 0.0], atol=1e-9)
    assert stamp_ns == 1_000_000_000


def test_wrist_force_rejects_missing_timestamp() -> None:
    assert motion._wrist_force_in_base(_observation((1.0, 0.0, 0.0), None)) is None


def test_haptic_guard_requires_sustained_force_and_resets() -> None:
    guard = motion.HapticGuard(np.zeros(3), threshold_n=5.0, duration_s=0.2)

    assert not guard.observe(_observation((6.0, 0.0, 0.0), 1_000_000_000))
    assert not guard.observe(_observation((20.0, 0.0, 0.0), 1_000_000_000))
    assert not guard.observe(_observation((6.0, 0.0, 0.0), 1_100_000_000))
    assert guard.observe(_observation((6.0, 0.0, 0.0), 1_200_000_000))

    assert not guard.observe(_observation((5.0, 0.0, 0.0), 1_300_000_000))
    assert not guard.observe(_observation((7.0, 0.0, 0.0), 1_400_000_000))
    assert guard.observe(_observation((7.0, 0.0, 0.0), 1_600_000_000))
    assert guard.peak_delta_force_n == 7.0


def test_prepare_haptic_guard_uses_distinct_frame_median() -> None:
    policy = _Policy()
    observations = iter(
        [
            _observation((1.0, 0.0, 0.0), 1_000_000_000),
            _observation((9.0, 0.0, 0.0), 1_100_000_000),
            _observation((3.0, 0.0, 0.0), 1_200_000_000),
        ]
    )

    guard = motion.prepare_haptic_guard(policy, lambda: next(observations))

    assert guard is not None
    np.testing.assert_allclose(guard.baseline_force, [3.0, 0.0, 0.0])
    assert guard.threshold_n == 5.0
    assert guard.duration_ns == 200_000_000


def test_follow_holds_current_pose_when_contact_is_detected() -> None:
    policy = _Policy()
    current_pose = _pose(xyz=(0.02, 0.0, 0.0))
    observation = _observation((10.0, 0.0, 0.0), 1_000_000_000, pose=current_pose)
    guard = motion.HapticGuard(np.zeros(3), threshold_n=5.0, duration_s=0.0)

    followed = motion._follow(
        policy,
        move_robot=None,
        start=_pose(),
        target=_pose(xyz=(0.10, 0.0, 0.0)),
        steps=3,
        dt=0.0,
        label="test",
        stiffness=[1.0] * 6,
        damping=[1.0] * 6,
        get_observation=lambda: observation,
        haptic_guard=guard,
    )

    assert not followed
    assert len(policy.commanded_poses) == 2
    assert policy.commanded_poses[-1].position.x == current_pose.position.x
    assert policy.logger.error_messages


def test_pose_convergence_reports_haptic_contact() -> None:
    policy = _Policy()
    policy.settle_timeout_s = 0.1
    policy.settle_position_tolerance_m = 0.001
    policy.settle_orientation_tolerance_rad = 0.01
    policy.settle_stable_observations = 3
    observation = _observation((10.0, 0.0, 0.0), 1_000_000_000)
    guard = motion.HapticGuard(np.zeros(3), threshold_n=5.0, duration_s=0.0)

    settled_stamp, detail = motion.wait_for_pose_convergence(
        policy,
        lambda: observation,
        _pose(),
        command_stamp_ns=0,
        haptic_guard=guard,
    )

    assert settled_stamp is None
    assert detail["failure_reason"] == "haptic_contact"
    assert detail["peak_delta_force_n"] == 10.0


def test_retreat_finishes_at_requested_pose() -> None:
    policy = _Policy()
    observation = _observation(
        (0.0, 0.0, 0.0),
        1_000_000_000,
        pose=_pose(xyz=(0.03, 0.0, 0.0)),
    )
    target = _pose(xyz=(0.0, 0.0, 0.0))

    retreated = motion.retreat_to_pose(
        policy,
        lambda: observation,
        move_robot=None,
        target=target,
        stiffness=[1.0] * 6,
        damping=[1.0] * 6,
    )

    assert retreated
    assert policy.commanded_poses[-1].position.x == target.position.x


def test_cli_exposes_haptic_guard_defaults() -> None:
    args = build_parser().parse_args(["--port-type", "sfp"])

    assert args.haptic_guard is True
    assert args.haptic_force_threshold_n == 20.0
    assert args.haptic_contact_duration_s == 0.2
