#!/usr/bin/env python3
"""
collect_lerobot_data.py
───────────────────────
collect_lerobot_data_aarch.py와 동일한 로직이지만, aarch64 소스 빌드 대신
distrobox + pixi 환경을 사용한다.

흐름:
  for N 세트:
    1. Trial 1·2·3 각각의 랜덤 파라미터를 담은 aic_engine config YAML 생성
    2. /tmp/aic_custom_config.yaml 로 저장 (distrobox 컨테이너와 /tmp 공유)
    3. distrobox 내에서 bash -c "/entrypoint.sh ..." 로 Gazebo + AIC engine 시작
       (spawn_task_board:=false, spawn_cable:=false → 엔진이 YAML에서 직접 스폰)
    4. Gazebo 초기화 대기
    5. pixi run ros2 run으로 aic_model + LeRobot 정책 시작
    6. episode_summary.json 수로 완료 감지 (프로세스 조기 종료 감지 포함)
    7. 프로세스 종료 → 다음 세트

사용법:
  ros2 run phy_data_collection collect_lerobot_data                                              # 기본: 10 세트 × 7 에피소드
  ros2 run phy_data_collection collect_lerobot_data --sets 50 --diversify                        # 50세트, 보드 위치도 랜덤화
  ros2 run phy_data_collection collect_lerobot_data --sets 5 --dry-run                           # 명령어만 출력 (실행 X)
  ros2 run phy_data_collection collect_lerobot_data --gazebo-wait 60                             # Gazebo 초기화 대기를 60초로
  ros2 run phy_data_collection collect_lerobot_data --headless                                   # Gazebo GUI·RViz 없이 백그라운드 실행
  ros2 run phy_data_collection collect_lerobot_data --lerobot-out-dir ~/data --lerobot-repo-id aic-sejong/ds  # LeRobot 저장
"""

import argparse
import json
import math
import os
import random
import shlex
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml

from phy_data_collection.paths import PROJECT_ROOT

LIMITS = {
    "nic_translation":   (-0.0215, 0.0234),
    "nic_yaw":           (-math.radians(10), math.radians(10)),
    "sc_translation":    (-0.06, 0.055),
    "board_yaw_trial12": (0.0, 3.1415),
    "board_yaw_trial3":  (0.0, 3.1415),
    "board_x_trial12":   (0.13, 0.17),
    "board_y_trial12":   (-0.25, -0.15),
    "board_x_trial3":    (0.15, 0.19),
    "board_y_trial3":    (-0.05, 0.05),
    # gripper offset 기준값 (Sample Config) 및 노이즈 범위
    "gripper_offset_noise": (-0.002, 0.002),
    "nic_gripper_offset_x": 0.0,
    "nic_gripper_offset_y": 0.015385,
    "nic_gripper_offset_z": 0.04245,
    "sc_gripper_offset_x":  0.0,
    "sc_gripper_offset_y":  0.015385,
    "sc_gripper_offset_z":  0.04045,
}

def rnd(low: float, high: float) -> float:
    return random.uniform(low, high)

GAZEBO_INIT_WAIT = 60   # Gazebo 초기화 대기 (초) — Zenoh peer 안정화 포함
EPISODE_TIMEOUT  = 1200 # 세트당 최대 대기 시간 (초)
TASK_TIME_LIMIT   = 600  # 데이터 저장까지 포함한 task별 최대 실행 시간 (초)

# ──────────────────────────────────────────
# 경로 설정
# ──────────────────────────────────────────

ROOT = PROJECT_ROOT

WS_SRC               = ROOT / "ws_aic" / "src"
PIXI_WS              = ROOT / "ws_aic" / "src"
YOLO_MODEL_DEFAULT   = WS_SRC / "model" / "ais_yolo-2" / "weights" / "best.pt"
ENGINE_CONFIG_TMP    = Path("/tmp/aic_custom_config.yaml")
SCENARIO_PARAMS_TMP  = Path("/tmp/aic_scenario_params.json")
EPISODE_TRACKING_DIR = Path("/tmp/aic_episodes")
POLICY_STOP_FILE     = Path("/tmp/aic_policy_stop")

POLICY_MODULES = {
    "LeRobot": "data_generator.LeRobot",
    "DataCollect": "data_generator.LeRobot",
}
DEFAULT_REPO_IDS = {
    "LeRobot": "aic-sejong-team/aic-dataset",
    "DataCollect": "aic-sejong-team/aic-dataset",
}
DEFAULT_LEROBOT_OUT_DIRS = {
    "LeRobot": Path("ws_aic/data/lerobot"),
    "DataCollect": Path("ws_aic/data/lerobot"),
}

# ──────────────────────────────────────────
# aic_engine config YAML 생성
# ──────────────────────────────────────────

def _scoring_section() -> dict:
    """scoring.topics: 고정 값 (sample_config.yaml 기준)."""
    return {
        "topics": [
            {"topic": {"name": "/joint_states",
                       "type": "sensor_msgs/msg/JointState"}},
            {"topic": {"name": "/tf",
                       "type": "tf2_msgs/msg/TFMessage"}},
            {"topic": {"name": "/tf_static",
                       "type": "tf2_msgs/msg/TFMessage",
                       "latched": True}},
            {"topic": {"name": "/scoring/tf",
                       "type": "tf2_msgs/msg/TFMessage"}},
            {"topic": {"name": "/aic/gazebo/contacts/off_limit",
                       "type": "ros_gz_interfaces/msg/Contacts"}},
            {"topic": {"name": "/fts_broadcaster/wrench",
                       "type": "geometry_msgs/msg/WrenchStamped"}},
            {"topic": {"name": "/aic_controller/joint_commands",
                       "type": "aic_control_interfaces/msg/JointMotionUpdate"}},
            {"topic": {"name": "/aic_controller/pose_commands",
                       "type": "aic_control_interfaces/msg/MotionUpdate"}},
            {"topic": {"name": "/scoring/insertion_event",
                       "type": "std_msgs/msg/String"}},
            {"topic": {"name": "/aic_controller/controller_state",
                       "type": "aic_control_interfaces/msg/ControllerState"}},
        ]
    }


def _task_board_limits_section() -> dict:
    """task_board_limits: LIMITS와 동일하게 고정."""
    return {
        "nic_rail": {"min_translation": LIMITS["nic_translation"][0],
                     "max_translation": LIMITS["nic_translation"][1]},
        "sc_rail":  {"min_translation": LIMITS["sc_translation"][0],
                     "max_translation": LIMITS["sc_translation"][1]},
    }


def _robot_section() -> dict:
    """robot.home_joint_positions: 고정 값."""
    return {
        "home_joint_positions": {
            "shoulder_pan_joint":  -0.1597,
            "shoulder_lift_joint": -1.3542,
            "elbow_joint":         -1.6648,
            "wrist_1_joint":       -1.6933,
            "wrist_2_joint":        1.5710,
            "wrist_3_joint":        1.4110,
        }
    }


def _board_pose(trial: int, diversify: bool) -> dict:
    """task_board pose 랜덤 생성.

    trial 1 = SFP/NIC (구 trial_1·2 통합)
    trial 2 = SC       (구 trial_3)

    yaw 값은 모두 동일하게 0~360 diversify
    """
    if trial == 1:
        x   = rnd(*LIMITS["board_x_trial12"]) if diversify else 0.15
        y   = rnd(*LIMITS["board_y_trial12"]) if diversify else -0.2
        yaw = rnd(*LIMITS["board_yaw_trial12"])
    else:
        x   = rnd(*LIMITS["board_x_trial3"]) if diversify else 0.17
        y   = rnd(*LIMITS["board_y_trial3"]) if diversify else 0.0
        yaw = rnd(*LIMITS["board_yaw_trial3"])
    return {"x": x, "y": y, "z": 1.14, "roll": 0.0, "pitch": 0.0, "yaw": yaw}


def _nic_rails(active_rail: int, translation: float, yaw: float) -> dict:
    """nic_rail_0 ~ nic_rail_4: active_rail 위치에만 NIC 카드 배치."""
    rails = {}
    for i in range(5):
        if i == active_rail:
            rails[f"nic_rail_{i}"] = {
                "entity_present": True,
                "entity_name": f"nic_card_{active_rail}",
                "entity_pose": {
                    "translation": translation,
                    "roll": 0.0, "pitch": 0.0,
                    "yaw": yaw,
                },
            }
        else:
            rails[f"nic_rail_{i}"] = {"entity_present": False}
    return rails


def _sc_rails_nic(translation: float) -> dict:
    """NIC trial용 sc_rail: sc_rail_0에 배경 SC 마운트, sc_rail_1 absent."""
    return {
        "sc_rail_0": {
            "entity_present": True,
            "entity_name": "sc_mount_0",
            "entity_pose": {
                "translation": translation,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            },
        },
        "sc_rail_1": {"entity_present": False},
    }


def _sc_rails_sc(active_rail: int, translation: float) -> dict:
    """SC trial용 sc_rail: active_rail 위치에 삽입 대상 SC 마운트, 나머지 absent."""
    rails = {}
    for i in range(2):
        if i == active_rail:
            rails[f"sc_rail_{i}"] = {
                "entity_present": True,
                "entity_name": f"sc_mount_{active_rail}",
                "entity_pose": {
                    "translation": translation,
                    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                },
            }
        else:
            rails[f"sc_rail_{i}"] = {"entity_present": False}
    return rails


def _mount_rails_nic() -> dict:
    """NIC trial용 mount rail 배치. translation 고정 (평가 대상 아님)."""
    def present(name: str) -> dict:
        return {
            "entity_present": True,
            "entity_name": name,
            "entity_pose": {"translation": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }
    def absent() -> dict:
        return {"entity_present": False}

    return {
        "lc_mount_rail_0":  present("lc_mount_0"),
        "sfp_mount_rail_0": present("sfp_mount_0"),
        "sc_mount_rail_0":  present("sc_mount_0"),
        "lc_mount_rail_1":  present("lc_mount_1"),
        "sfp_mount_rail_1": absent(),
        "sc_mount_rail_1":  absent(),
    }


def _mount_rails_sc() -> dict:
    """SC trial용 mount rail 배치. translation 고정 (평가 대상 아님)."""
    def present(name: str) -> dict:
        return {
            "entity_present": True,
            "entity_name": name,
            "entity_pose": {"translation": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }
    def absent() -> dict:
        return {"entity_present": False}

    return {
        "lc_mount_rail_0":  absent(),
        "sfp_mount_rail_0": present("sfp_mount_0"),
        "sc_mount_rail_0":  present("sc_mount_2"),
        "lc_mount_rail_1":  present("lc_mount_1"),
        "sfp_mount_rail_1": absent(),
        "sc_mount_rail_1":  absent(),
    }


def _make_nic_trial(nic_rail: int, diversify: bool) -> tuple[dict, dict]:
    """NIC trial: nic_rail_N에 NIC 카드 삽입.

    Returns:
        (engine_config, scenario_params) — scenario_params는 LeRobot 프레임에 저장될 랜덤 파라미터.
    """
    board   = _board_pose(1, diversify)
    nic_t   = rnd(*LIMITS["nic_translation"])
    nic_y   = rnd(*LIMITS["nic_yaw"])
    sc_bg_t = rnd(*LIMITS["sc_translation"])

    offset_x = LIMITS["nic_gripper_offset_x"] + rnd(*LIMITS["gripper_offset_noise"])
    offset_y = LIMITS["nic_gripper_offset_y"] + rnd(*LIMITS["gripper_offset_noise"])
    offset_z = LIMITS["nic_gripper_offset_z"] + rnd(*LIMITS["gripper_offset_noise"])
    sfp_port_idx = random.choice([0, 1])
    sfp_port_name = f"sfp_port_{sfp_port_idx}"

    task_board = {"pose": board}
    task_board.update(_nic_rails(nic_rail, nic_t, nic_y))
    task_board.update(_sc_rails_nic(sc_bg_t))
    task_board.update(_mount_rails_nic())

    scenario_params = {
        "trial_type":       0,         # 0 = NIC
        "rail_idx":         nic_rail,
        "board_x":          board["x"],
        "board_y":          board["y"],
        "board_yaw":        board["yaw"],
        "gripper_offset_x": offset_x,
        "gripper_offset_y": offset_y,
        "gripper_offset_z": offset_z,
        "nic_translation":  nic_t,
        "nic_yaw":          nic_y,
        "sc_translation":   sc_bg_t,
        "sfp_port_idx":     sfp_port_idx,
    }

    config = {
        "scene": {
            "task_board": task_board,
            "cables": {
                "cable_0": {
                    "pose": {
                        "gripper_offset": {"x": offset_x, "y": offset_y, "z": offset_z},
                        "roll": 0.4432, "pitch": -0.4838, "yaw": 1.3303,
                    },
                    "attach_cable_to_gripper": True,
                    "cable_type": "sfp_sc_cable",
                }
            },
        },
        "tasks": {
            f"nic_rail{nic_rail}_{sfp_port_name}_task_1": {
                "cable_type": "sfp_sc",
                "cable_name": "cable_0",
                "plug_type": "sfp",
                "plug_name": "sfp_tip",
                "port_type": "sfp",
                "port_name": sfp_port_name,
                "target_module_name": f"nic_card_mount_{nic_rail}",
                "time_limit": TASK_TIME_LIMIT,
            }
        },
    }
    return config, scenario_params


def _make_sc_trial(sc_rail: int, diversify: bool) -> tuple[dict, dict]:
    """SC trial: sc_rail_N에 SC 케이블 삽입.

    Returns:
        (engine_config, scenario_params) — scenario_params는 LeRobot 프레임에 저장될 랜덤 파라미터.
    """
    board  = _board_pose(2, diversify)
    sc_t   = rnd(*LIMITS["sc_translation"])

    offset_x = LIMITS["sc_gripper_offset_x"] + rnd(*LIMITS["gripper_offset_noise"])
    offset_y = LIMITS["sc_gripper_offset_y"] + rnd(*LIMITS["gripper_offset_noise"])
    offset_z = LIMITS["sc_gripper_offset_z"] + rnd(*LIMITS["gripper_offset_noise"])

    task_board = {"pose": board}
    for i in range(5):
        task_board[f"nic_rail_{i}"] = {"entity_present": False}
    task_board.update(_sc_rails_sc(sc_rail, sc_t))
    task_board.update(_mount_rails_sc())

    scenario_params = {
        "trial_type":       1,         # 1 = SC
        "rail_idx":         sc_rail,
        "board_x":          board["x"],
        "board_y":          board["y"],
        "board_yaw":        board["yaw"],
        "gripper_offset_x": offset_x,
        "gripper_offset_y": offset_y,
        "gripper_offset_z": offset_z,
        "nic_translation":  0.0,       # SC trial에서는 해당 없음
        "nic_yaw":          0.0,
        "sc_translation":   sc_t,
    }

    config = {
        "scene": {
            "task_board": task_board,
            "cables": {
                "cable_1": {
                    "pose": {
                        "gripper_offset": {"x": offset_x, "y": offset_y, "z": offset_z},
                        "roll": 0.4432, "pitch": -0.4838, "yaw": 1.3303,
                    },
                    "attach_cable_to_gripper": True,
                    "cable_type": "sfp_sc_cable_reversed",
                }
            },
        },
        "tasks": {
            f"sc_rail{sc_rail}_task_1": {
                "cable_type": "sfp_sc",
                "cable_name": "cable_1",
                "plug_type": "sc",
                "plug_name": "sc_tip",
                "port_type": "sc",
                "port_name": "sc_port_base",
                "target_module_name": f"sc_port_{sc_rail}",
                "time_limit": TASK_TIME_LIMIT,
            }
        },
    }
    return config, scenario_params


def generate_engine_config(diversify: bool = True) -> tuple[dict, dict]:
    """
    sample_config.yaml 구조에 맞는 커스텀 aic_engine config 딕셔너리 생성.

    세트당 7개 trial:
      nic_rail0 ~ nic_rail4 : NIC 카드 삽입 (rail 0~4 순서대로)
      sc_rail0  ~ sc_rail1  : SC 케이블 삽입 (rail 0~1 순서대로)

    Returns:
        (engine_config, all_scenario_params)
        all_scenario_params: task_id → scenario_params dict (LeRobot 기록용)
    """
    trials: dict = {}
    all_scenario_params: dict = {}

    for rail in range(5):
        cfg, params = _make_nic_trial(rail, diversify)
        trials[f"trial_nic{rail}"] = cfg
        all_scenario_params[f"nic_rail{rail}_task_1"] = params

    for rail in range(2):
        cfg, params = _make_sc_trial(rail, diversify)
        trials[f"trial_sc{rail}"] = cfg
        all_scenario_params[f"sc_rail{rail}_task_1"] = params

    config = {
        "scoring": _scoring_section(),
        "task_board_limits": _task_board_limits_section(),
        "trials": trials,
        "robot": _robot_section(),
    }
    return config, all_scenario_params


def save_engine_config(config: dict, path: Path) -> None:
    """config 딕셔너리를 YAML 파일로 저장."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True,
                  sort_keys=False)


# ──────────────────────────────────────────
# 에피소드 감시
# ──────────────────────────────────────────

def count_completed_episodes() -> int:
    """episode_summary.json 파일 수로 완료된 에피소드 수를 반환."""
    return len(list(EPISODE_TRACKING_DIR.glob("*/episode_summary.json")))


def wait_for_episodes(
    episodes_before: int,
    target_count: int,
    timeout: int = EPISODE_TIMEOUT,
    watch_procs: list | None = None,
) -> tuple[int, str]:
    """
    target_count 개의 새 에피소드 완료까지 대기.

    watch_procs 에 프로세스를 넘기면, 해당 프로세스가 조기 종료될 경우
    즉시 대기를 중단하고 'proc_died' 를 반환한다.

    Returns:
        (실제로 완료된 새 에피소드 수, 종료 이유: 'ok' | 'timeout' | 'proc_died')
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        if watch_procs:
            for proc in watch_procs:
                if proc is not None and proc.poll() is not None:
                    new_eps = count_completed_episodes() - episodes_before
                    print(f"\n[감지] 프로세스(PID {proc.pid}) 조기 종료 "
                          f"(returncode={proc.returncode}) → 대기 중단")
                    return new_eps, "proc_died"

        current = count_completed_episodes()
        new_eps = current - episodes_before
        elapsed = int(time.time() - (deadline - timeout))
        print(
            f"\r[대기] 에피소드 {new_eps}/{target_count} 완료  ({elapsed}s 경과)",
            end="", flush=True,
        )
        if new_eps >= target_count:
            print()
            return new_eps, "ok"
        time.sleep(5)

    print()
    new_eps = count_completed_episodes() - episodes_before
    print(f"[경고] {timeout}초 내에 {target_count}개 에피소드가 완료되지 않았습니다. "
          f"(실제 완료: {new_eps}개)")
    return new_eps, "timeout"


# ──────────────────────────────────────────
# 프로세스 관리
# ──────────────────────────────────────────

def start_gazebo(
    config_path: Path,
    headless: bool = False,
    dry_run: bool = False,
) -> "subprocess.Popen | None":
    """
    distrobox에서 Gazebo + AIC engine 시작.

    bash -c 를 통해 args를 하나의 문자열로 전달한다.
    (distrobox -r 모드에서 -- 이후 다중 인자가 깨지는 문제 방지)
    """
    launch_args = [
        "spawn_task_board:=false",
        "spawn_cable:=false",
        "ground_truth:=true",
        "start_aic_engine:=true",
        f"aic_engine_config_file:={config_path}",
    ]
    if headless:
        launch_args += ["gazebo_gui:=false", "launch_rviz:=false"]

    args_str = " ".join(shlex.quote(a) for a in launch_args)
    cmd = ["distrobox", "enter", "-r", "aic_eval_physic", "--", "bash", "-c",
           f"/entrypoint.sh {args_str}"]

    if dry_run:
        mode = "헤드리스" if headless else "GUI"
        print(f"[DRY-RUN] Gazebo 명령어 ({mode}):")
        print("  " + " ".join(cmd))
        return None

    mode = "헤드리스(GUI 없음)" if headless else "GUI"
    print(f"[Gazebo] distrobox 시작... (config: {config_path}, 모드: {mode})")
    proc = subprocess.Popen(cmd, stderr=subprocess.STDOUT)
    time.sleep(1)
    if proc.poll() is not None:
        print(f"[에러] distrobox 프로세스가 즉시 종료됨 (returncode={proc.returncode}). "
              "터미널 위 출력을 확인하세요.")
    return proc


def start_policy(
    step_hz: float = 20.0,
    data_policy: str = "LeRobot",
    lerobot_out_dir: "Path | None" = None,
    lerobot_repo_id: str = "",
    lerobot_run_id: str = "",
    lerobot_version: str = "master",
    yolo_model_path: "Path | None" = None,
    dry_run: bool = False,
) -> "subprocess.Popen | None":
    """
    pixi run으로 aic_model + LeRobot 계열 정책 노드 시작.
    대부분의 설정(Hz, YOLO 경로 등)은 이제 policy.py 내부 기본값으로 처리됨.
    """
    env = os.environ.copy()
    policy_module = POLICY_MODULES[data_policy]

    # LeRobot 저장 관련 설정만 전달
    if lerobot_out_dir and lerobot_repo_id:
        env["AIC_LEROBOT_OUT_DIR"]      = str(lerobot_out_dir)
        env["AIC_LEROBOT_REPO_ID"]      = lerobot_repo_id
        env["AIC_LEROBOT_RUN_ID"]       = lerobot_run_id
        env["AIC_LEROBOT_FPS"]          = str(int(step_hz))
        env["AIC_LEROBOT_VERSION"]      = lerobot_version
        env["AIC_VISION_OFFSET_DATASET_DIR"] = str(
            Path(lerobot_out_dir) / lerobot_version / "vision_offset_dataset"
        )
        # 중복 방지: 정책 노드 레벨의 자동 업로드는 항상 끔
        env["AIC_LEROBOT_PUSH_TO_HUB"]  = "false"

    # YOLO 모델 경로: 명시적으로 항상 설정 (쉘 환경변수의 낡은 값 방지)
    resolved_yolo = Path(yolo_model_path).resolve() if yolo_model_path else YOLO_MODEL_DEFAULT
    env["AIC_YOLO_MODEL_PATH"] = str(resolved_yolo)

    cmd = (
        f"cd {PIXI_WS} && pixi run ros2 run aic_model aic_model "
        f"--ros-args -p policy:={policy_module}"
    )

    if dry_run:
        print("[DRY-RUN] Policy 명령어:")
        if lerobot_out_dir and lerobot_repo_id:
            print(f"  AIC_LEROBOT_OUT_DIR={lerobot_out_dir}")
            print(f"  AIC_LEROBOT_REPO_ID={lerobot_repo_id}")
            print(f"  AIC_VISION_OFFSET_DATASET_DIR={Path(lerobot_out_dir) / lerobot_version / 'vision_offset_dataset'}")
        print(f"  data_policy={data_policy}")
        print(f"  {cmd}")
        return None

    print(f"[Policy] {data_policy} 시작 ({step_hz}Hz)")
    proc = subprocess.Popen(cmd, shell=True, env=env, stderr=subprocess.STDOUT)
    time.sleep(2)
    if proc.poll() is not None:
        print(f"[에러] aic_model 프로세스가 즉시 종료됨 (returncode={proc.returncode}). "
              "policy 초기화 에러일 수 있습니다.")
    return proc


def stop_policy_gracefully(policy_proc, timeout: int = 120) -> None:
    """stop-file을 써서 policy_proc가 finalize() 후 스스로 종료하도록 유도한다."""
    if policy_proc is None or policy_proc.poll() is not None:
        return
    print(f"[종료] Policy 정상 종료 신호 전송 (최대 {timeout}초 대기)...")
    POLICY_STOP_FILE.touch()
    try:
        policy_proc.wait(timeout=timeout)
        print("[종료] Policy 정상 종료 완료")
    except subprocess.TimeoutExpired:
        print("[강제종료] Policy timeout → kill")
        policy_proc.kill()
    finally:
        POLICY_STOP_FILE.unlink(missing_ok=True)


def push_dataset_to_hub(
    lerobot_out_dir: Path,
    lerobot_repo_id: str,
    lerobot_version: str = "master",
) -> None:
    dataset_root = Path(lerobot_out_dir).resolve() / lerobot_version
    if not (dataset_root / "meta" / "info.json").exists():
        print(f"[HF Hub] LeRobot dataset missing at {dataset_root}")
        return

    print(f"\n[HF Hub] Pushing standard LeRobot dataset to {lerobot_repo_id} (branch: {lerobot_version})")

    inline_script = (
        f"from lerobot.datasets.lerobot_dataset import LeRobotDataset\n"
        f"ds = LeRobotDataset.resume(repo_id='{lerobot_repo_id}', root='{str(dataset_root)}')\n"
        f"ds.push_to_hub(branch='{lerobot_version}')\n"
    )

    cmd = f"cd {PIXI_WS} && pixi run python3 -c {shlex.quote(inline_script)}"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

    if result.returncode == 0:
        print(result.stdout.strip())
        print(f"[HF Hub] Successfully uploaded LeRobot format to https://huggingface.co/datasets/{lerobot_repo_id}/tree/{lerobot_version}")
    else:
        print(f"[HF Hub] Upload failed:\n{result.stderr.strip()}")

def terminate_processes(*procs):
    """
    프로세스 트리 전체를 완전히 정리.

    distrobox enter 프로세스만 죽이면 내부에서 뜬 Gazebo·ROS2 노드들이
    좀비로 남아 다음 세션의 포트·Zenoh·GPU 리소스를 점유한다.
    rootful distrobox의 내부 프로세스는 root 권한으로 실행되므로 sudo pkill 사용.
    """
    # 1단계: Python이 직접 들고 있는 Popen 종료
    for proc in procs:
        if proc is None or proc.poll() is not None:
            continue
        print(f"[종료] PID {proc.pid} 종료 중...")
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            print(f"[강제종료] PID {proc.pid} kill")
            proc.kill()

    # 2단계: distrobox 내부에서 실행된 프로세스들 정리
    _GZ_PATTERNS = [
        "gz sim",
        "gz_server",
        "gzserver",
        "ruby.*gz",
    ]
    _ROS_PATTERNS = [
        "aic_engine",
        "aic_model",
        "aic_adapter",
        "robot_state_publisher",
        "ros_gz_bridge",
        "ros2_control_node",
        "controller_manager",
        "component_container",
        "ros2.*spawner",
        "rviz2",
        "static_transform_publisher",
        "topic_tools",
    ]
    _ZENOH_PATTERNS = [
        "zenoh",
    ]

    print("[정리] Gazebo·ROS2·Zenoh 잔존 프로세스 종료 중...")
    for pattern in _GZ_PATTERNS + _ROS_PATTERNS + _ZENOH_PATTERNS:
        # rootful distrobox 내부 프로세스는 root 소유 → sudo 필요
        subprocess.run(["sudo", "pkill", "-9", "-f", pattern], capture_output=True)

    # 3단계: 공유 메모리 정리 (종료된 PID의 Zenoh shm + ros2_control shm)
    subprocess.run(
        ["bash", "-c",
         "for f in /dev/shm/*.zenoh; do "
         "  pid=$(basename $f .zenoh); "
         "  kill -0 $pid 2>/dev/null || rm -f $f; "
         "done; "
         "rm -f /dev/shm/ros2_control_*"],
        capture_output=True,
    )

    # 4단계: 소켓·GPU 컨텍스트 반환 대기
    time.sleep(10)


# ──────────────────────────────────────────
# 메인 루프
# ──────────────────────────────────────────
def run_collection_loop(
    n_sets: int,
    diversify: bool,
    gazebo_wait: int,
    step_hz: float,
    data_policy: str,
    headless: bool,
    dry_run: bool,
    lerobot_out_dir: "Path | None" = None,
    lerobot_repo_id: str = "",
    lerobot_version: str = "master",
    yolo_model_path: "Path | None" = None,
    push_to_hub: bool = False,
):
    EPISODE_TRACKING_DIR.mkdir(parents=True, exist_ok=True)

    lerobot_run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    use_lerobot = bool(lerobot_out_dir and lerobot_repo_id)

    print("=== AIC 데이터 수집 시작 (distrobox + pixi 환경) ===")
    print(f"  세트 수        : {n_sets}")
    print(f"  정책           : {data_policy} ({POLICY_MODULES[data_policy]})")

    # 1. aic_engine config YAML 생성 (에피소드 수 자동 감지를 위해 미리 한 번 호출)
    test_config, _ = generate_engine_config(diversify=diversify)
    trials_per_set = len(test_config["trials"])
    print(f"  세트당 에피소드: {trials_per_set} (자동 감지)")

    if use_lerobot:
        push_str = f"LeRobot → {lerobot_repo_id} (hub 업로드: {'ON' if push_to_hub else 'OFF — 로컬만'})"
    else:
        push_str = "raw"
    print(f"  저장 모드      : {push_str}")
    print(f"  engine config  : {ENGINE_CONFIG_TMP}")
    print(f"  YOLO 모델      : {yolo_model_path or YOLO_MODEL_DEFAULT}")
    print(f"  diversify      : {diversify}")
    print(f"  headless       : {headless}")
    print(f"  dry-run        : {dry_run}")

    if not dry_run:
        # 시작 전 잔존 프로세스 정리 (이전 수동 실행 또는 비정상 종료 대비)
        print("[정리] 잔존 ROS2/Gazebo 프로세스 정리 중...")
        terminate_processes()
        print("[정리] 완료\n")

    total_collected = 0

    for set_idx in range(1, n_sets + 1):
        print(f"\n{'='*60}")
        print(f"  세트 {set_idx} / {n_sets}   (누적 에피소드: {total_collected})")
        print(f"{'='*60}")

        # 1. aic_engine config YAML 생성
        config, all_scenario_params = generate_engine_config(diversify=diversify)

        # 2. YAML → /tmp (distrobox와 공유)
        if not dry_run:
            save_engine_config(config, ENGINE_CONFIG_TMP)
            print(f"[저장] engine config → {ENGINE_CONFIG_TMP}")

        # 3. scenario_params JSON 저장 (/tmp — policy가 task별 랜덤 파라미터 읽기용)
        SCENARIO_PARAMS_TMP.write_text(
            json.dumps(all_scenario_params, ensure_ascii=False), encoding="utf-8"
        )

        # 4. 에피소드 기준점 기록
        episodes_before = count_completed_episodes()

        # 5. Gazebo → Policy 순으로 시작
        GAZEBO_HEAD_START = 5  # aic_engine의 model discovery timeout 전에 policy를 띄운다.

        gazebo_proc = start_gazebo(ENGINE_CONFIG_TMP, headless=headless, dry_run=dry_run)
        if not dry_run:
            print(f"[대기] Gazebo 초기화 대기 중... ({GAZEBO_HEAD_START}초)")
            time.sleep(GAZEBO_HEAD_START)

        policy_proc = start_policy(
            step_hz=step_hz,
            data_policy=data_policy,
            lerobot_out_dir=lerobot_out_dir,
            lerobot_repo_id=lerobot_repo_id,
            lerobot_run_id=lerobot_run_id,
            lerobot_version=lerobot_version,
            yolo_model_path=yolo_model_path,
            dry_run=dry_run,
        )
        if not dry_run:
            remaining = max(0, gazebo_wait - GAZEBO_HEAD_START)
            print(f"[대기] aic_model 안정화 대기 중... ({remaining}초 추가)")
            time.sleep(remaining)

        if dry_run:
            print("[DRY-RUN] 실제 실행 없이 종료.")
            continue

        # 6. 에피소드 완료 대기 (프로세스 조기 종료 감지 시 즉시 종료)
        try:
            completed, reason = wait_for_episodes(
                episodes_before, trials_per_set,
                timeout=EPISODE_TIMEOUT,
                watch_procs=[gazebo_proc, policy_proc],
            )
        except KeyboardInterrupt:
            print("\n[중단] Ctrl+C 감지. 프로세스 종료 중...")
            stop_policy_gracefully(policy_proc, timeout=60)
            terminate_processes(gazebo_proc)
            if use_lerobot and push_to_hub:
                push_dataset_to_hub(lerobot_out_dir, lerobot_repo_id, lerobot_version)
            print(f"[중단] 총 수집 에피소드: {total_collected}")
            sys.exit(0)

        if reason != "ok":
            print(f"[경고] 세트 {set_idx} 비정상 종료 (이유: {reason}) → 다음 세트로 이동.")

        total_collected += completed
        print(f"[완료] 세트 {set_idx}: {completed}개 에피소드 수집 (누적: {total_collected})")

        # 7. 프로세스 종료 — policy는 stop-file로 정상 종료, 나머지는 강제 종료
        stop_policy_gracefully(policy_proc)
        terminate_processes(gazebo_proc)

        if set_idx < n_sets:
            print("[대기] 다음 세트 준비 중... (5초)")
            time.sleep(5)

    print(f"\n{'='*60}")
    print(f"=== 수집 완료: {n_sets} 세트, 총 {total_collected} 에피소드 ===")
    print(f"{'='*60}")

    if use_lerobot and not dry_run and push_to_hub:
        push_dataset_to_hub(lerobot_out_dir, lerobot_repo_id, lerobot_version)
    elif use_lerobot and not dry_run:
        print(f"\n[HF Hub] 업로드 생략 (--no-push-to-hub 지정). "
              f"로컬 경로: {lerobot_out_dir / lerobot_version}")


# ──────────────────────────────────────────
# CLI
# ──────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="AIC 자동 데이터 수집 루프 (distrobox + pixi 환경)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
        예시:
        ros2 run phy_data_collection collect_lerobot_data                                              # 10 세트 × 7 에피소드
        ros2 run phy_data_collection collect_lerobot_data --sets 50 --diversify                        # 50세트, 보드 위치도 랜덤화
        ros2 run phy_data_collection collect_lerobot_data --sets 5 --dry-run                           # 명령어만 출력
        ros2 run phy_data_collection collect_lerobot_data --gazebo-wait 60                             # Gazebo 초기화 대기 60초
        ros2 run phy_data_collection collect_lerobot_data --headless                                   # Gazebo & RViz GUI 없이 실행
        ros2 run phy_data_collection collect_lerobot_data --lerobot-out-dir ~/data --lerobot-repo-id aic-sejong/ds
""",
    )
    parser.add_argument("--sets",             type=int,  default=10,
                        help="수집할 세트 수 (기본: 10)")
    parser.add_argument("--diversify",        action="store_true",
                        help="보드 위치/yaw도 범위 내에서 랜덤화")
    parser.add_argument("--gazebo-wait",      type=int,  default=GAZEBO_INIT_WAIT,
                        help=f"Gazebo 초기화 대기 시간(초, 기본: {GAZEBO_INIT_WAIT})")
    parser.add_argument("--step-hz",          type=float, default=20.0,
                        help="스텝 샘플링 주파수 Hz (기본: 10Hz)")
    parser.add_argument("--data-policy",      choices=sorted(POLICY_MODULES), default="LeRobot",
                        help="수집 정책 선택. LeRobot/DataCollect는 기본 LeRobot 에피소드 수집용")
    parser.add_argument("--headless",         action="store_true",
                        help="Gazebo GUI·RViz 없이 백그라운드 실행 (gazebo_gui:=false launch_rviz:=false)")
    parser.add_argument("--dry-run",          action="store_true",
                        help="명령어만 출력하고 실제 실행하지 않음")
    parser.add_argument("--lerobot-out-dir",  type=Path, default=None,
                        help="LeRobot 데이터셋 로컬 저장 경로 (미지정 시 정책별 기본 경로 사용)")
    parser.add_argument("--lerobot-repo-id",  type=str,  default=None,
                        help="HuggingFace repo ID (미지정 시 정책별 기본 repo 사용)")
    parser.add_argument("--lerobot-version",  type=str,  default="v1.0",
                        help="데이터셋 버전/브랜치 이름 (예: v1.0)")
    parser.add_argument("--yolo-model",       type=Path, default=None,
                        help=f"YOLO 모델 .pt 경로 (기본: {YOLO_MODEL_DEFAULT})")
    parser.add_argument("--push-to-hub",      dest="push_to_hub", action="store_true", default=True,
                        help="수집 완료 후 HuggingFace Hub에 LeRobot 데이터셋 업로드 (기본: 활성)")
    parser.add_argument("--no-push-to-hub",   dest="push_to_hub", action="store_false",
                        help="HuggingFace Hub 업로드 비활성화")

    args = parser.parse_args()
    lerobot_out_dir = (args.lerobot_out_dir or DEFAULT_LEROBOT_OUT_DIRS[args.data_policy]).resolve()
    yolo_model_path = Path(args.yolo_model).resolve() if args.yolo_model else None
    lerobot_repo_id = args.lerobot_repo_id or DEFAULT_REPO_IDS[args.data_policy]
    run_collection_loop(
        n_sets          = args.sets,
        diversify       = args.diversify,
        gazebo_wait     = args.gazebo_wait,
        step_hz         = args.step_hz,
        data_policy     = args.data_policy,
        headless        = args.headless,
        dry_run         = args.dry_run,
        lerobot_out_dir = lerobot_out_dir,
        lerobot_repo_id = lerobot_repo_id,
        lerobot_version = args.lerobot_version,
        yolo_model_path = yolo_model_path,
        push_to_hub     = args.push_to_hub,
    )

if __name__ == "__main__":
    main()
