"""PortOffsetCollect 수집 시각 일치 조건의 회귀 테스트."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time as TimeMessage

from data_generator.port_offset_dataset import (
    _observation_sync_metadata,
    _save_xyz_rpy_sample,
    _tf_sync_metadata,
    _wait_for_synchronized_observation,
)
from data_generator.port_offset_runtime import (
    _collect_log_text,
    _lookup_transform_at,
)
from data_generator.port_offset_stage_motion import (
    _capture_failure_reason,
    _time_difference_text,
)


def _stamp(nanoseconds: int):
    """nanosecond 정수로 가짜 ROS time message를 만든다."""
    return SimpleNamespace(
        sec=nanoseconds // 1_000_000_000,
        nanosec=nanoseconds % 1_000_000_000,
    )


def _message(nanoseconds: int):
    """header timestamp와 단위 transform을 가진 가짜 ROS message를 만든다."""
    return SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(nanoseconds), frame_id="base_link"),
        child_frame_id="test_frame",
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def _observation(left: int, center: int, right: int, controller: int):
    """세 camera와 controller timestamp를 가진 observation을 만든다."""
    return SimpleNamespace(
        left_image=_message(left),
        center_image=_message(center),
        right_image=_message(right),
        controller_state=_message(controller),
    )


def _policy(tolerance_ns: int = 30_000_000):
    """timestamp helper 호출에 필요한 최소 policy 대역을 만든다."""
    return SimpleNamespace(
        collect_sync_tolerance_ns=tolerance_ns,
        _image_msg_for_camera=lambda obs, name: getattr(obs, f"{name}_image"),
    )


def test_observation_within_tolerance_is_accepted() -> None:
    """모든 source가 허용 오차 안이면 center 시각 기준으로 승인한다."""
    policy = _policy()
    valid, metadata = _observation_sync_metadata(
        policy,
        _observation(
            1_000_000_000,
            1_010_000_000,
            1_020_000_000,
            1_025_000_000,
        ),
    )

    assert valid is True
    assert metadata["capture_stamp_ns"] == 1_010_000_000
    assert metadata["skew_ns"] == {
        "camera": 20_000_000,
        "controller": 15_000_000,
    }


def test_camera_time_difference_over_tolerance_is_rejected() -> None:
    """camera timestamp 범위가 허용 오차를 넘으면 sample을 거부한다."""
    valid, metadata = _observation_sync_metadata(
        _policy(),
        _observation(
            1_000_000_000,
            1_010_000_000,
            1_050_000_000,
            1_010_000_000,
        ),
    )

    assert valid is False
    assert metadata["rejection_reason"] == "camera_time_difference_exceeded"


def test_controller_time_difference_over_tolerance_is_rejected() -> None:
    """controller timestamp가 center camera에서 멀면 sample을 거부한다."""
    valid, metadata = _observation_sync_metadata(
        _policy(),
        _observation(
            1_000_000_000,
            1_010_000_000,
            1_020_000_000,
            1_050_000_000,
        ),
    )

    assert valid is False
    assert metadata["rejection_reason"] == "controller_time_difference_exceeded"


def test_static_tf_is_allowed_but_stale_dynamic_tf_is_rejected() -> None:
    """static TF의 0 stamp는 허용하고 오래된 dynamic TF는 거부한다."""
    policy = _policy()
    timestamps = {
        "capture_stamp_ns": 1_010_000_000,
        "skew_ns": {},
        "sync_valid": True,
    }
    valid, metadata = _tf_sync_metadata(
        policy,
        timestamps,
        {
            "port": _message(0),
            "plug": _message(1_050_000_000),
        },
    )

    assert valid is False
    assert metadata["tf"]["port"]["is_static"] is True
    assert metadata["rejection_reason"] == "tf_time_difference_exceeded"


def test_nonzero_port_snapshot_is_treated_as_trial_static() -> None:
    """trial 시작에 저장한 port TF는 과거 stamp여도 정적 source로 승인한다."""
    timestamps = {
        "capture_stamp_ns": 2_000_000_000,
        "skew_ns": {},
        "sync_valid": True,
    }

    valid, metadata = _tf_sync_metadata(
        _policy(),
        timestamps,
        {
            "port": _message(1_000_000_000),
            "plug": _message(2_010_000_000),
        },
        static_sources={"port"},
    )

    assert valid is True
    assert metadata["tf"]["port"]["is_static_snapshot"] is True
    assert metadata["tf"]["port"]["skew_ns"] == 0
    assert metadata["tf"]["port"]["parent_frame_id"] == "base_link"
    assert metadata["tf"]["port"]["child_frame_id"] == "test_frame"
    assert metadata["tf"]["port"]["transform"]["translation_m"]["x"] == 0.0


def test_wait_selects_next_synchronized_observation() -> None:
    """첫 observation이 어긋나면 제한 시간 안의 다음 유효 observation을 선택한다."""
    policy = _policy()
    policy.collect_sync_wait_timeout_sec = 0.1
    policy.collect_sync_poll_sec = 0.001
    policy._observation_sync_metadata = lambda obs: _observation_sync_metadata(
        policy,
        obs,
    )
    policy._collect_log_text = lambda message, _color: message
    policy.get_logger = lambda: SimpleNamespace(info=lambda _message: None)
    policy.sleep_for = lambda _seconds: None
    observations = iter(
        [
            _observation(1_000_000_000, 1_010_000_000, 1_100_000_000, 1_010_000_000),
            _observation(2_000_000_000, 2_010_000_000, 2_020_000_000, 2_015_000_000),
        ]
    )

    observation, metadata = _wait_for_synchronized_observation(
        policy,
        lambda: next(observations),
    )

    assert observation is not None
    assert metadata["capture_stamp_ns"] == 2_010_000_000
    assert metadata["sync_valid"] is True
    assert "observation" in metadata["wait_ns"]


def test_wait_rejects_cached_observation_from_before_command() -> None:
    """동기화된 frame이어도 현재 명령보다 오래됐으면 다음 frame을 기다린다."""
    policy = _policy()
    policy.collect_sync_wait_timeout_sec = 0.1
    policy.collect_sync_poll_sec = 0.001
    policy._observation_sync_metadata = lambda obs: _observation_sync_metadata(
        policy,
        obs,
    )
    policy._collect_log_text = lambda message, _color: message
    policy.get_logger = lambda: SimpleNamespace(info=lambda _message: None)
    policy.sleep_for = lambda _seconds: None
    observations = iter(
        [
            _observation(2_000_000_000, 2_010_000_000, 2_020_000_000, 2_015_000_000),
            _observation(3_000_000_000, 3_010_000_000, 3_020_000_000, 3_015_000_000),
        ]
    )

    observation, metadata = _wait_for_synchronized_observation(
        policy,
        lambda: next(observations),
        min_capture_stamp_ns=2_500_000_000,
    )

    assert observation is not None
    assert metadata["capture_stamp_ns"] == 3_010_000_000
    assert metadata["sync_valid"] is True


def test_time_difference_log_uses_milliseconds() -> None:
    """사용자 로그에는 각 source의 시각 차이를 ms 단위로 표시한다."""
    text = _time_difference_text(
        {"skew_ns": {"camera": 20_000_000, "controller": 5_000_000}}
    )

    assert text == "camera=20.000 ms, controller=5.000 ms"


def test_capture_result_colors_include_green_and_red() -> None:
    """저장 성공과 실패 상태가 각각 초록색과 빨간색 ANSI 로그를 사용한다."""
    policy = SimpleNamespace(collect_color_log=True)

    success = _collect_log_text(policy, "CAPTURE SAVED", "green", bold=True)
    failure = _collect_log_text(policy, "CAPTURE FAILED", "red", bold=True)

    assert "\033[32m" in success
    assert "\033[31m" in failure


def test_capture_failure_reason_explains_camera_time_difference() -> None:
    """내부 분류 키를 사용자가 이해할 수 있는 실패 이유로 변환한다."""
    reason = _capture_failure_reason(
        "observation_sync_timeout:camera_time_difference_exceeded"
    )

    assert "대기시간 내" in reason
    assert "세 camera 촬영 시각 차이" in reason


def test_save_result_explains_missing_observation() -> None:
    """저장 실패는 bool뿐 아니라 사용자가 확인할 이유도 반환한다."""
    saved, reason = _save_xyz_rpy_sample(
        SimpleNamespace(),
        "episode",
        None,
        "collect",
        0,
        None,
        None,
        None,
        None,
        {},
        None,
    )

    assert saved is False
    assert reason == "Observation을 받지 못함"


def test_lookup_transform_at_uses_requested_camera_timestamp() -> None:
    """plug TF는 별도 cache 없이 지정한 camera 시각으로 메인 TF2에서 조회한다."""
    calls = {}
    expected = _message(1_500_000_000)

    class FakeBuffer:
        """요청받은 TF2 인자를 기록하는 최소 가짜 buffer."""

        def can_transform(self, *_args):
            """테스트에서는 지정 시각 TF를 즉시 사용할 수 있다고 응답한다."""
            return True

        def lookup_transform(self, target, source, query_time, *, timeout):
            """조회 시각과 frame을 기록하고 준비된 transform을 반환한다."""
            calls.update({
                "target": target,
                "source": source,
                "query_time_ns": query_time.nanoseconds,
                "timeout_ns": timeout.nanoseconds,
            })
            return expected

    policy = SimpleNamespace(
        _parent_node=SimpleNamespace(_tf_buffer=FakeBuffer()),
        collect_sync_wait_timeout_sec=1.0,
    )

    result = _lookup_transform_at(
        policy,
        "base_link",
        "plug",
        TimeMessage(sec=1, nanosec=500_000_000),
    )

    assert result is expected
    assert calls == {
        "target": "base_link",
        "source": "plug",
        "query_time_ns": 1_500_000_000,
        "timeout_ns": 1_000_000_000,
    }
