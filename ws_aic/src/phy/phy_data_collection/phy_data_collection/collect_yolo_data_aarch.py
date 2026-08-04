#!/usr/bin/env python3
"""
collect_yolo_data_aarch.py
───────────────────────────
collect_lerobot_data_aarch.py의 다양한 시나리오 생성/Gazebo 관리와
collect_dataset.py의 YOLO 데이터셋 수집(카메라→TF→bbox)을 통합한 스크립트.

각 시나리오(NIC rail 0~4, SC rail 0~1)마다:
  1. 랜덤 파라미터로 engine config YAML 생성
  2. Zenoh 라우터 + Gazebo 시작
  3. DatasetCollector 노드로 3대 카메라 스냅샷 수집 → YOLO 라벨 자동 생성
  4. Gazebo 종료 후 다음 시나리오로

출력 구조 (YOLO dataset format):
  <output>/
  ├── images/
  │   ├── train/  s00001_nic0_snap0000_left.jpg, ...
  │   └── val/
  ├── labels/
  │   ├── train/  s00001_nic0_snap0000_left.txt
  │   └── val/
  └── data.yaml

Classes:
  0: sfp_port
  1: sc_port
  2: sfp_port_tip
  3: sc_port_tip

사용법:
  cd ~/aic-physic/ws_aic/src
  ros2 run phy_data_collection collect_yolo_data_aarch --sets 10
  ros2 run phy_data_collection collect_yolo_data_aarch --sets 20 --snapshots 30 --diversify --headless
"""
import argparse
import json
import math
import os
import random
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

from phy_data_collection.paths import PROJECT_ROOT

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, TransformException


# ──────────────────────────────────────────
# 랜덤 파라미터 LIMITS
# ──────────────────────────────────────────
LIMITS = {
    "nic_translation":      (-0.0215, 0.0234),
    "nic_yaw":              (-math.radians(10), math.radians(10)),
    "sc_translation":       (-0.06, 0.055),
    "mount_translation":    (-0.09425, 0.09425),
    "mount_yaw":            (-math.radians(60), math.radians(60)),
    "board_yaw_trial12":    (0.0, 3.1415),
    "board_yaw_trial3":     (0.0, 3.1415),
    "board_x_trial12":      (0.13, 0.17),
    "board_y_trial12":      (-0.25, -0.15),
    "board_x_trial3":       (0.15, 0.19),
    "board_y_trial3":       (-0.05, 0.05),
    "gripper_offset_noise": (-0.002, 0.002),
    "nic_gripper_offset_y": 0.015385,
    "nic_gripper_offset_z": 0.04245,
    "sc_gripper_offset_y":  0.015385,
    "sc_gripper_offset_z":  0.04045,
}


def rnd(low: float, high: float) -> float:
    return random.uniform(low, high)


# ──────────────────────────────────────────
# YOLO 클래스 / 카메라 정의
# ──────────────────────────────────────────
SFP_PORT_FRAMES = [
    f"task_board/nic_card_mount_{mount_idx}/sfp_port_{port_idx}_link"
    for mount_idx in range(5)
    for port_idx in range(2)
]

SC_PORT_FRAMES = [
    f"task_board/sc_port_{idx}/sc_port_base_link"
    for idx in range(2)
]


def _port_tip_frames(port_frames):
    """포트 중심 frame 이름에서 포트 입구 tip frame 이름을 만든다."""
    return [f"{frame}_entrance" for frame in port_frames]


PORT_DEFINITIONS = [
    (0, "sfp_port", SFP_PORT_FRAMES, (0.014, 0.010)),
    (1, "sc_port", SC_PORT_FRAMES, (0.012, 0.025)),
    (2, "sfp_port_tip", _port_tip_frames(SFP_PORT_FRAMES), (0.006, 0.006)),
    (3, "sc_port_tip", _port_tip_frames(SC_PORT_FRAMES), (0.006, 0.006)),
]

CAMERAS = [
    ("left",   "left_camera/optical"),
    ("center", "center_camera/optical"),
    ("right",  "right_camera/optical"),
]

GAZEBO_INIT_WAIT = 60

# NIC rail 0~4 × 5 + SC rail 0~1 × 2 = 7 시나리오/세트
SCENARIOS_PER_SET = [
    ("nic", 0), ("nic", 1), ("nic", 2), ("nic", 3), ("nic", 4),
    ("sc",  0), ("sc",  1),
]


# ──────────────────────────────────────────
# 경로 설정
# ──────────────────────────────────────────
ROOT                 = PROJECT_ROOT
PIXI_WS              = ROOT / "ws_aic" / "src"
DEFAULT_OUTPUT_DIR   = ROOT / "ws_aic" / "src" / "data" / "yolo"
DEFAULT_SCENARIO_DIR = Path("/tmp/aic_scenario_params")
ENGINE_CONFIG_TMP    = Path("/tmp/aic_yolo_config.yaml")
WS_AIC_SETUP         = ROOT / "ws_aic" / "install" / "setup.bash"
EPISODE_TRACKING_DIR = Path("/tmp/aic_yolo_episodes")
SCENARIO_PARAMS_TMP  = Path("/tmp/aic_yolo_scenario_params.json")

# pixi 전용 패키지 경로 (lerobot, cv2 등)
PIXI_SITE_PACKAGES = PIXI_WS / ".pixi" / "envs" / "default" / "lib" / "python3.12" / "site-packages"
PIXI_LIB           = PIXI_WS / ".pixi" / "envs" / "default" / "lib"


def _apply_pixi_env(env: dict) -> None:
    """소스 빌드 Python에서 pixi 패키지를 안전하게 사용할 수 있도록 env를 in-place 수정.

    PYTHONPATH : data_generator 루트 + pixi site-packages
    LD_LIBRARY_PATH : pixi lib/ (pixi cv2의 OpenSSL 버전 불일치 방지)
    """
    data_generator_parent = str(
        ROOT / "ws_aic" / "src" / "phy" / "phy_policy" / "data_generator"
    )
    pixi_site = str(PIXI_SITE_PACKAGES)
    pixi_lib  = str(PIXI_LIB)

    py_extras = f"{data_generator_parent}:{pixi_site}"
    existing_py = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = f"{py_extras}:{existing_py}" if existing_py else py_extras

    existing_ld = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = f"{pixi_lib}:{existing_ld}" if existing_ld else pixi_lib


# ──────────────────────────────────────────
# 수학 유틸리티 (3D→이미지 투영)
# ──────────────────────────────────────────
def transform_to_matrix(t) -> np.ndarray:
    tx, ty, tz = t.translation.x, t.translation.y, t.translation.z
    qx, qy, qz, qw = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    R = np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx + zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx + yy)],
    ])
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = [tx, ty, tz]
    return M


def project_to_camera(point_3d_base, K, T_base_to_cam):
    p_cam = T_base_to_cam @ np.append(point_3d_base, 1.0)
    x, y, z = p_cam[:3]
    if z < 1e-6:
        return None, None, None
    return float(K[0, 0] * x / z + K[0, 2]), float(K[1, 1] * y / z + K[1, 2]), float(z)


def compute_bbox_from_size(depth, port_size_m, K, margin=1.2):
    """포트 실제 크기(m)를 핀홀 근사로 이미지 bbox 픽셀 크기로 변환."""
    w_m, h_m = port_size_m
    return (K[0, 0] * w_m / depth * margin,
            K[1, 1] * h_m / depth * margin)


# ──────────────────────────────────────────
# ROS 노드
# ──────────────────────────────────────────
class DatasetCollector(Node):
    def __init__(self):
        super().__init__("yolo_dataset_collector")
        self._cam_info = {}
        self._latest_image = {}
        for name, _ in CAMERAS:
            self.create_subscription(
                CameraInfo, f"/{name}_camera/camera_info",
                lambda msg, n=name: self._on_info(n, msg), 10,
            )
            self.create_subscription(
                Image, f"/{name}_camera/image",
                lambda msg, n=name: self._on_image(n, msg), 10,
            )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.get_logger().info("DatasetCollector 노드 시작됨.")

    def reset(self):
        """Gazebo 재시작 후 이전 이미지/카메라 정보 캐시 초기화."""
        self._cam_info.clear()
        self._latest_image.clear()
        # TF 버퍼도 재초기화: 이전 Gazebo 세션의 stale 데이터가
        # 새 세션의 낮은 sim-time TF를 TF_OLD_DATA로 거부하는 문제 방지.
        self._tf_listener.unregister()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.get_logger().info("[DatasetCollector] 리셋 완료 (TF 버퍼 재초기화).")

    def _on_info(self, name, msg):
        self._cam_info[name] = msg

    def _on_image(self, name, msg):
        # 인코딩별 채널 수 매핑 (알 수 없으면 3 가정)
        ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(msg.encoding, 3)
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, ch)
        except ValueError:
            self.get_logger().warn(f"[{name}] reshape 실패: enc={msg.encoding} h={msg.height} w={msg.width}")
            return
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == "rgba8":
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        elif msg.encoding == "bgra8":
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        elif msg.encoding == "mono8":
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        self._latest_image[name] = img

    def has_all_data(self) -> bool:
        return (len(self._cam_info) == len(CAMERAS)
                and len(self._latest_image) == len(CAMERAS))

    def wait_for_data(self, timeout=20.0) -> bool:
        start = time.time()
        while not self.has_all_data() and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.has_all_data()

    def discover_existing_ports(self) -> list:
        found = []
        for class_id, class_name, candidate_frames, size_m in PORT_DEFINITIONS:
            for frame in candidate_frames:
                try:
                    tf = self._tf_buffer.lookup_transform("base_link", frame, Time())
                    found.append({
                        "class_id":   class_id,
                        "class_name": class_name,
                        "frame":      frame,
                        "pos_3d":     np.array([tf.transform.translation.x,
                                                tf.transform.translation.y,
                                                tf.transform.translation.z]),
                        "size_m":     size_m,
                    })
                except TransformException:
                    continue
        return found

    def wait_for_ports(self, timeout=15.0) -> list:
        start = time.time()
        ports = self.discover_existing_ports()
        while not ports and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            ports = self.discover_existing_ports()
        return ports

    def collect_one_frame(
        self,
        scenario_label: str,
        snap_id: int,
        out_dir: Path,
        is_val: bool = False,
    ) -> bool:
        """현재 장면 스냅샷을 YOLO 형식으로 저장. 저장 성공 시 True 반환."""
        split = "val" if is_val else "train"

        cam_T_in_base = {}
        for name, frame in CAMERAS:
            try:
                tf = self._tf_buffer.lookup_transform("base_link", frame, Time())
                cam_T_in_base[name] = transform_to_matrix(tf.transform)
            except TransformException as ex:
                self.get_logger().warn(f"카메라 {name} TF 실패: {ex}")
                return False

        ports = self.discover_existing_ports()
        if not ports:
            return False

        saved = False
        for name, _ in CAMERAS:
            if name not in self._latest_image:
                continue

            img = self._latest_image[name].copy()
            h, w = img.shape[:2]
            K = np.array(self._cam_info[name].k).reshape(3, 3)
            T_base_to_cam = np.linalg.inv(cam_T_in_base[name])

            yolo_labels = []
            for p in ports:
                u, v, depth = project_to_camera(p["pos_3d"], K, T_base_to_cam)
                if u is None:
                    continue
                if not (0 <= u < w and 0 <= v < h):
                    continue
                if not (0.05 <= depth <= 2.0):
                    continue
                bw, bh = compute_bbox_from_size(depth, p["size_m"], K)
                yolo_labels.append(
                    f"{p['class_id']} "
                    f"{np.clip(u/w, 0, 1):.6f} "
                    f"{np.clip(v/h, 0, 1):.6f} "
                    f"{np.clip(bw/w, 0.001, 1):.6f} "
                    f"{np.clip(bh/h, 0.001, 1):.6f}"
                )

            if not yolo_labels:
                continue

            stem = f"{scenario_label}_snap{snap_id:04d}_{name}"
            img_path = out_dir / "images" / split / f"{stem}.jpg"
            lbl_path = out_dir / "labels" / split / f"{stem}.txt"
            img_path.parent.mkdir(parents=True, exist_ok=True)
            lbl_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(img_path), img)
            lbl_path.write_text("\n".join(yolo_labels))
            saved = True

        return saved


# ──────────────────────────────────────────
# Engine config 생성 (시나리오별 단일 trial)
# ──────────────────────────────────────────
def _scoring_section() -> dict:
    return {"topics": [
        {"topic": {"name": "/joint_states",                      "type": "sensor_msgs/msg/JointState"}},
        {"topic": {"name": "/tf",                                "type": "tf2_msgs/msg/TFMessage"}},
        {"topic": {"name": "/tf_static",                         "type": "tf2_msgs/msg/TFMessage", "latched": True}},
        {"topic": {"name": "/scoring/tf",                        "type": "tf2_msgs/msg/TFMessage"}},
        {"topic": {"name": "/aic/gazebo/contacts/off_limit",     "type": "ros_gz_interfaces/msg/Contacts"}},
        {"topic": {"name": "/fts_broadcaster/wrench",            "type": "geometry_msgs/msg/WrenchStamped"}},
        {"topic": {"name": "/aic_controller/joint_commands",     "type": "aic_control_interfaces/msg/JointMotionUpdate"}},
        {"topic": {"name": "/aic_controller/pose_commands",      "type": "aic_control_interfaces/msg/MotionUpdate"}},
        {"topic": {"name": "/scoring/insertion_event",           "type": "std_msgs/msg/String"}},
        {"topic": {"name": "/aic_controller/controller_state",   "type": "aic_control_interfaces/msg/ControllerState"}},
    ]}


def _task_board_limits_section() -> dict:
    return {
        "nic_rail":   {"min_translation": LIMITS["nic_translation"][0],
                       "max_translation": LIMITS["nic_translation"][1]},
        "sc_rail":    {"min_translation": LIMITS["sc_translation"][0],
                       "max_translation": LIMITS["sc_translation"][1]},
        "mount_rail": {"min_translation": LIMITS["mount_translation"][0],
                       "max_translation": LIMITS["mount_translation"][1]},
    }


def _robot_section() -> dict:
    return {"home_joint_positions": {
        "shoulder_pan_joint":  -0.1597,
        "shoulder_lift_joint": -1.3542,
        "elbow_joint":         -1.6648,
        "wrist_1_joint":       -1.6933,
        "wrist_2_joint":        1.5710,
        "wrist_3_joint":        1.4110,
    }}


def _board_pose(trial_type: str, diversify: bool) -> dict:
    if trial_type == "nic":
        x   = rnd(*LIMITS["board_x_trial12"]) if diversify else 0.15
        y   = rnd(*LIMITS["board_y_trial12"]) if diversify else -0.2
        yaw = rnd(*LIMITS["board_yaw_trial12"])
    else:
        x   = rnd(*LIMITS["board_x_trial3"]) if diversify else 0.17
        y   = rnd(*LIMITS["board_y_trial3"]) if diversify else 0.0
        yaw = rnd(*LIMITS["board_yaw_trial3"])
    return {"x": x, "y": y, "z": 1.14, "roll": 0.0, "pitch": 0.0, "yaw": yaw}


def _nic_rails(active_rail: int) -> dict:
    rails = {}
    for i in range(5):
        if i == active_rail:
            rails[f"nic_rail_{i}"] = {
                "entity_present": True,
                "entity_name": f"nic_card_{active_rail}",
                "entity_pose": {
                    "translation": rnd(*LIMITS["nic_translation"]),
                    "roll": 0.0, "pitch": 0.0,
                    "yaw": rnd(*LIMITS["nic_yaw"]),
                },
            }
        else:
            rails[f"nic_rail_{i}"] = {"entity_present": False}
    return rails


def _sc_rails_nic() -> dict:
    return {
        "sc_rail_0": {
            "entity_present": True,
            "entity_name": "sc_mount_0",
            "entity_pose": {"translation": rnd(*LIMITS["sc_translation"]),
                            "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        },
        "sc_rail_1": {"entity_present": False},
    }


def _sc_rails_sc(active_rail: int) -> dict:
    rails = {}
    for i in range(2):
        if i == active_rail:
            rails[f"sc_rail_{i}"] = {
                "entity_present": True,
                "entity_name": f"sc_mount_{active_rail}",
                "entity_pose": {"translation": rnd(*LIMITS["sc_translation"]),
                                "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            }
        else:
            rails[f"sc_rail_{i}"] = {"entity_present": False}
    return rails


def _mount_rails_nic() -> dict:
    def present(name: str) -> dict:
        return {"entity_present": True, "entity_name": name,
                "entity_pose": {"translation": rnd(*LIMITS["mount_translation"]),
                                "roll": 0.0, "pitch": 0.0, "yaw": 0.0}}
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
    def present(name: str) -> dict:
        return {"entity_present": True, "entity_name": name,
                "entity_pose": {"translation": rnd(*LIMITS["mount_translation"]),
                                "roll": 0.0, "pitch": 0.0, "yaw": 0.0}}
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


def make_nic_trial_config(nic_rail: int, diversify: bool) -> dict:
    task_board = {"pose": _board_pose("nic", diversify)}
    task_board.update(_nic_rails(nic_rail))
    task_board.update(_sc_rails_nic())
    task_board.update(_mount_rails_nic())
    return {
        "scoring":            _scoring_section(),
        "task_board_limits":  _task_board_limits_section(),
        "robot":              _robot_section(),
        "trials": {
            f"trial_nic{nic_rail}": {
                "scene": {
                    "task_board": task_board,
                    "cables": {"cable_0": {
                        "pose": {"gripper_offset": {"x": 0.0,
                                                    "y": LIMITS["nic_gripper_offset_y"],
                                                    "z": LIMITS["nic_gripper_offset_z"]},
                                 "roll": 0.4432, "pitch": -0.4838, "yaw": 1.3303},
                        "attach_cable_to_gripper": True,
                        "cable_type": "sfp_sc_cable",
                    }},
                },
                "tasks": {f"nic_rail{nic_rail}_task_1": {
                    "cable_type": "sfp_sc", "cable_name": "cable_0",
                    "plug_type": "sfp", "plug_name": "sfp_tip",
                    "port_type": "sfp", "port_name": "sfp_port_0",
                    "target_module_name": f"nic_card_mount_{nic_rail}",
                    "time_limit": 180,
                }},
            }
        },
    }


def make_sc_trial_config(sc_rail: int, diversify: bool) -> dict:
    task_board = {"pose": _board_pose("sc", diversify)}
    for i in range(5):
        task_board[f"nic_rail_{i}"] = {"entity_present": False}
    task_board.update(_sc_rails_sc(sc_rail))
    task_board.update(_mount_rails_sc())
    return {
        "scoring":            _scoring_section(),
        "task_board_limits":  _task_board_limits_section(),
        "robot":              _robot_section(),
        "trials": {
            f"trial_sc{sc_rail}": {
                "scene": {
                    "task_board": task_board,
                    "cables": {"cable_1": {
                        "pose": {"gripper_offset": {"x": 0.0,
                                                    "y": LIMITS["sc_gripper_offset_y"],
                                                    "z": LIMITS["sc_gripper_offset_z"]},
                                 "roll": 0.4432, "pitch": -0.4838, "yaw": 1.3303},
                        "attach_cable_to_gripper": True,
                        "cable_type": "sfp_sc_cable_reversed",
                    }},
                },
                "tasks": {f"sc_rail{sc_rail}_task_1": {
                    "cable_type": "sfp_sc", "cable_name": "cable_1",
                    "plug_type": "sc", "plug_name": "sc_tip",
                    "port_type": "sc", "port_name": "sc_port_base",
                    "target_module_name": f"sc_port_{sc_rail}",
                    "time_limit": 180,
                }},
            }
        },
    }


def save_engine_config(config: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True, sort_keys=False)


# ──────────────────────────────────────────
# 프로세스 관리
# ──────────────────────────────────────────
def _ros2_env() -> dict:
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    env["ZENOH_CONFIG_OVERRIDE"] = (
        "transport/shared_memory/enabled=true;"
        "transport/shared_memory/transport_optimization/pool_size=536870912"
    )
    return env


def is_zenoh_running() -> bool:
    return subprocess.run(["pgrep", "-f", "rmw_zenohd"], capture_output=True).returncode == 0


def start_zenoh(dry_run: bool = False):
    cmd = f"source {WS_AIC_SETUP} && ros2 run rmw_zenoh_cpp rmw_zenohd"
    if dry_run:
        print(f"[DRY-RUN] Zenoh: {cmd}")
        return None
    if is_zenoh_running():
        print("[Zenoh] 이미 실행 중 — 기존 라우터 사용.")
        return None
    print("[Zenoh] 라우터 시작...")
    proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                            env=_ros2_env(), stderr=subprocess.STDOUT)
    time.sleep(1)
    if proc.poll() is not None:
        print(f"[에러] Zenoh 즉시 종료 (returncode={proc.returncode}).")
    return proc


def start_gazebo(config_path: Path, headless: bool = False, dry_run: bool = False):
    launch_args = [
        "spawn_task_board:=false",   # aic_engine이 config에 따라 spawn
        "spawn_cable:=false",
        "ground_truth:=true",
        "start_aic_engine:=true",            # 쥌 스폰 + TF 발행 주체
        f"aic_engine_config_file:={config_path}",
    ]
    if headless:
        launch_args += ["gazebo_gui:=false", "launch_rviz:=false"]
    cmd = (
        f"source {WS_AIC_SETUP} && "
        f"ros2 launch aic_bringup aic_gz_bringup.launch.py "
        + " ".join(launch_args)
    )
    if dry_run:
        print(f"[DRY-RUN] Gazebo ({'헤드리스' if headless else 'GUI'}): {cmd}")
        return None
    print(f"[Gazebo] 시작... ({'헤드리스' if headless else 'GUI'})")
    proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                            env=_ros2_env(), stderr=subprocess.STDOUT)
    time.sleep(1)
    if proc.poll() is not None:
        print(f"[에러] Gazebo 즉시 종료 (returncode={proc.returncode}).")
    return proc


def count_completed_episodes() -> int:
    return len(list(EPISODE_TRACKING_DIR.glob("*/episode_summary.json")))


def start_aic_model(dry_run: bool = False):
    """LeRobot policy로 trial을 실제 실행해 Task Board가 반드시 spawn되도록 한다.

    autocapture는 lifecycle만 수행해 trial이 시작되지 않으므로 entity spawn이
    보장되지 않는다. LeRobot을 사용하면 aic_engine이 trial을 시작하고
    Task Board를 spawn한다. LeRobot 저장은 env var 제거로 비활성화한다.
    """
    env = _ros2_env()
    _apply_pixi_env(env)
    env.pop("AIC_LEROBOT_REPO_ID", None)
    env.pop("AIC_LEROBOT_OUT_DIR",  None)
    env["AIC_LEROBOT_PUSH_TO_HUB"]        = "false"
    env["AIC_CAPTURE_DIR"]                = str(EPISODE_TRACKING_DIR)
    env["AIC_SCENARIO_PARAMS_FILE"]       = str(SCENARIO_PARAMS_TMP)
    env["AIC_CAPTURE_STEP_SLEEP_SEC"]     = "0.1"
    cmd = (
        f"source {WS_AIC_SETUP} && "
        f"ros2 run aic_model aic_model "
        f"--ros-args -p policy:=data_generator.LeRobot"
    )
    if dry_run:
        print(f"[DRY-RUN] aic_model: {cmd}")
        return None
    print("[aic_model] 시작 (LeRobot policy — Task Board spawn 보장, LeRobot 저장 비활성)...")
    proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                            env=env, stderr=subprocess.STDOUT)
    time.sleep(1)
    if proc.poll() is not None:
        print(f"[에러] aic_model 즉시 종료 (returncode={proc.returncode}).")
    return proc


def terminate_processes(*procs, stop_zenoh: bool = False):
    """
    프로세스 트리 전체를 완전히 정리.

    Python Popen 종료 후 OS 레벨에서 관련 프로세스를 모두 정리한다.

    Args:
        *procs      : 종료할 Popen 객체들
        stop_zenoh  : True 일 때만 Zenoh 라우터도 함께 종료.
                      외부에서 Zenoh를 별도로 관리하는 경우 False(기본값)로 두면
                      스크립트가 Zenoh를 건드리지 않는다.
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

    # 2단계: ros2 launch에서 파생된 프로세스들 정리
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
        "rmw_zenohd",
    ]

    patterns = _GZ_PATTERNS + _ROS_PATTERNS
    if stop_zenoh:
        patterns += _ZENOH_PATTERNS
        print("[정리] Gazebo·ROS2·Zenoh 잔존 프로세스 종료 중...")
    else:
        print("[정리] Gazebo·ROS2 잔존 프로세스 종료 중... (Zenoh는 유지)")

    for pattern in patterns:
        # sudo 없이 pkill — 자신이 띄운 프로세스는 권한 없이 종료 가능
        subprocess.run(["pkill", "-9", "-f", pattern], capture_output=True)

    # 3단계: 공유 메모리 정리 (종료된 PID의 Zenoh shm + ros2_control shm)
    subprocess.run(
        ["bash", "-c",
         # 살아있는 PID의 zenoh 파일은 보존, 죽은 것만 삭제
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

def run_yolo_collection_loop(
    n_sets: int,
    n_snapshots: int,
    out_dir: Path,
    scenario_dir: Path,
    diversify: bool,
    gazebo_wait: int,
    headless: bool,
    val_ratio: float,
    dry_run: bool,
):
    # 날짜별 서브디렉터리: yolo/<YYYYMMDD>/
    date_str = datetime.now().strftime("%Y%m%d")
    out_dir = out_dir / date_str
    out_dir.mkdir(parents=True, exist_ok=True)

    scenario_dir.mkdir(parents=True, exist_ok=True)

    total_scenarios = n_sets * len(SCENARIOS_PER_SET)
    print("=== AIC YOLO 데이터 수집 시작 (aarch64 소스 빌드 환경) ===")
    print(f"  세트 수         : {n_sets}")
    print(f"  시나리오당 스냅샷: {n_snapshots}  ×3 카메라")
    print(f"  총 시나리오     : {total_scenarios}  (NIC×5 + SC×2 per set)")
    print(f"  예상 이미지     : {total_scenarios * n_snapshots * 3}")
    print(f"  출력 경로       : {out_dir}")
    print(f"  diversify       : {diversify}")
    print(f"  headless        : {headless}")
    print(f"  dry-run         : {dry_run}")

    if not dry_run:
        # 시작 전 잔존 프로세스 정리 (Zenoh 제외)
        print("[정리] 잔존 ROS2/Gazebo 프로세스 정리 중... (Zenoh 제외)")
        terminate_processes()
        print("[정리] 완료\n")
        EPISODE_TRACKING_DIR.mkdir(parents=True, exist_ok=True)

    # 누적 카운터 초기화
    total_saved    = 0
    scenario_counter = 0

    # ROS2 초기화 + DatasetCollector 노드 생성
    rclpy.init()
    collector = DatasetCollector()

    # Zenoh 라우터 시작 (이미 실행 중이면 기존 사용)
    zenoh_proc = start_zenoh(dry_run)

    try:
        for set_idx in range(1, n_sets + 1):
            print(f"\n{'='*60}")
            print(f"  세트 {set_idx} / {n_sets}  (저장된 프레임: {total_saved})")
            print(f"{'='*60}")

            for scenario_type, rail_idx in SCENARIOS_PER_SET:
                scenario_counter += 1
                scenario_label = f"s{scenario_counter:05d}_{scenario_type}{rail_idx}"
                print(f"\n[시나리오 {scenario_counter}/{total_scenarios}]"
                      f"  {scenario_type.upper()} rail {rail_idx}  ({scenario_label})")

                # 1. Engine config 생성 & 저장, scenario_params JSON 저장
                config = (make_nic_trial_config(rail_idx, diversify)
                          if scenario_type == "nic"
                          else make_sc_trial_config(rail_idx, diversify))
                save_engine_config(config, ENGINE_CONFIG_TMP)

                # LeRobot policy가 읽는 scenario_params 파일 생성
                if scenario_type == "nic":
                    task_key = f"nic_rail{rail_idx}_task_1"
                    params = {"trial_type": 0, "rail_idx": rail_idx,
                              "gripper_offset_x": 0.0,
                              "gripper_offset_y": LIMITS["nic_gripper_offset_y"],
                              "gripper_offset_z": LIMITS["nic_gripper_offset_z"]}
                else:
                    task_key = f"sc_rail{rail_idx}_task_1"
                    params = {"trial_type": 1, "rail_idx": rail_idx,
                              "gripper_offset_x": 0.0,
                              "gripper_offset_y": LIMITS["sc_gripper_offset_y"],
                              "gripper_offset_z": LIMITS["sc_gripper_offset_z"]}
                SCENARIO_PARAMS_TMP.write_text(
                    json.dumps({task_key: params}, ensure_ascii=False), encoding="utf-8"
                )

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                json_path = scenario_dir / f"{scenario_label}_{timestamp}.json"
                with open(json_path, "w", encoding="utf-8") as f:
                    json.dump(
                        {"meta": {"set_idx": set_idx, "scenario_type": scenario_type,
                                  "rail_idx": rail_idx, "timestamp": timestamp,
                                  "diversify": diversify},
                         "engine_config": config},
                        f, indent=2, ensure_ascii=False,
                    )

                # 2. Gazebo + aic_model 시작
                # LeRobot policy가 trial을 실제로 실행해야 Task Board가 spawn된다.
                # ─ 중요: aic_engine의 aic_model 탐색 타임아웃은 ~10-15초이므로
                #         Gazebo 시작 직후 빠르게 aic_model을 올려야 씬 spawn에 성공한다.
                episodes_before = count_completed_episodes()
                collector.reset()
                gazebo_proc = start_gazebo(ENGINE_CONFIG_TMP, headless=headless)

                # Gazebo 기본 인프라(controller_manager 등) 대기 — 짧게
                # aic_engine 탐색 타임아웃은 ~17초이므로 최대한 빨리 aic_model을 올려야 한다.
                # collect_lerobot_data_aarch.py도 25초 뒤 policy를 시작하는데,
                # Gazebo 부팅 자체에 ~10초 걸리므로 실제 탐색 시작은 t=10s — 잔여 17초 안에 등록 가능.
                GAZEBO_PRE_WAIT = 5   # 최소 대기: ROS2 기본 노드가 올라오는 시간
                print(f"[대기] Gazebo 인프라 대기 중... ({GAZEBO_PRE_WAIT}초)")
                pre_deadline = time.time() + GAZEBO_PRE_WAIT
                while time.time() < pre_deadline:
                    rclpy.spin_once(collector, timeout_sec=0.5)

                # aic_model 조기 시작 → aic_engine 탐색 타임아웃 이내에 등록
                policy_proc = start_aic_model(dry_run)

                # 나머지 초기화 대기 (씬 spawn + 카메라/TF 안정화)
                remaining = max(0, gazebo_wait - GAZEBO_PRE_WAIT)
                print(f"[대기] 씬 spawn + 안정화 대기 중... ({remaining}초)")
                deadline = time.time() + remaining
                while time.time() < deadline:
                    rclpy.spin_once(collector, timeout_sec=0.5)

                # 3. 카메라 데이터 수신 확인
                print("[대기] 카메라 데이터 확인... (최대 20초)")
                if not collector.wait_for_data(timeout=20.0):
                    print(f"[경고] 카메라 데이터 수신 실패 — {scenario_label} 스킵")
                    terminate_processes(gazebo_proc)
                    continue

                # TF 버퍼가 충분히 쌓이도록 추가 spin
                tf_deadline = time.time() + 5.0
                while time.time() < tf_deadline:
                    rclpy.spin_once(collector, timeout_sec=0.1)

                ports = collector.wait_for_ports(timeout=10.0)
                if not ports:
                    print(f"[경고] 포트 TF 없음 — {scenario_label} 스킵")
                    terminate_processes(gazebo_proc)
                    continue

                print(f"[확인] 포트 {len(ports)}개 발견. 스냅샷 수집 시작.")

                # 4. 스냅샷 N장 수집
                saved_this = 0
                val_step = max(1, int(1.0 / val_ratio)) if val_ratio > 0 else 0
                for snap_id in range(n_snapshots):
                    rclpy.spin_once(collector, timeout_sec=0.1)
                    is_val = (val_step > 0 and snap_id % val_step == 0)
                    if collector.collect_one_frame(scenario_label, snap_id, out_dir, is_val):
                        saved_this += 1
                    time.sleep(0.5)

                total_saved += saved_this
                print(f"[완료] {scenario_label}: {saved_this}/{n_snapshots} 저장 "
                      f"(누적: {total_saved})")

                # 5. Gazebo 종료 후 다음 시나리오로
                terminate_processes(gazebo_proc)
                if scenario_counter < total_scenarios:
                    print("[대기] 다음 시나리오 준비... (5초)")
                    time.sleep(5)

    except KeyboardInterrupt:
        print("\n[중단] Ctrl+C 감지. 정리 중...")

    # data.yaml
    data_yaml = out_dir / "data.yaml"
    data_yaml.write_text(
        f"path: {out_dir}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"names:\n"
        f"  0: sfp_port\n"
        f"  1: sc_port\n"
        f"  2: sfp_port_tip\n"
        f"  3: sc_port_tip\n"
    )

    collector.destroy_node()
    rclpy.shutdown()
    terminate_processes(zenoh_proc, stop_zenoh=True)

    total_images = len(list((out_dir / "images").rglob("*.jpg"))) if (out_dir / "images").exists() else 0
    print(f"\n{'='*60}")
    print(f"=== 완료: {n_sets} 세트 × {len(SCENARIOS_PER_SET)} 시나리오 ===")
    print(f"  저장된 프레임수 : {total_saved}")
    print(f"  총 이미지 파일  : {total_images}")
    print(f"  data.yaml       : {data_yaml}")
    print(f"{'='*60}")


# ──────────────────────────────────────────
# CLI
# ──────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="AIC YOLO 데이터 자동 수집 (aarch64 소스 빌드 환경)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예시:
  pixi run ros2 run phy_data_collection collect_yolo_data_aarch --sets 10
  pixi run ros2 run phy_data_collection collect_yolo_data_aarch --sets 20 --snapshots 30 --diversify
  pixi run ros2 run phy_data_collection collect_yolo_data_aarch --sets 5 --headless --dry-run
""",
    )
    parser.add_argument("--sets",         type=int,   default=10,
                        help="수집할 세트 수 (기본: 10)")
    parser.add_argument("--snapshots",    type=int,   default=20,
                        help="시나리오당 스냅샷 수 (기본: 20)")
    parser.add_argument("--output",       type=Path,  default=DEFAULT_OUTPUT_DIR,
                        help="YOLO 데이터셋 출력 경로")
    parser.add_argument("--scenario-dir", type=Path,  default=DEFAULT_SCENARIO_DIR,
                        help="JSON 시나리오 기록 경로")
    parser.add_argument("--diversify",    action="store_true",
                        help="보드 위치/yaw를 범위 내에서 랜덤화")
    parser.add_argument("--gazebo-wait",  type=int,   default=GAZEBO_INIT_WAIT,
                        help=f"Gazebo 초기화 대기 시간 초 (기본: {GAZEBO_INIT_WAIT})")
    parser.add_argument("--headless",     action="store_true",
                        help="Gazebo GUI·RViz 없이 실행")
    parser.add_argument("--val-ratio",    type=float, default=0.3,
                        help="검증 세트 비율 (기본: 0.3)")
    parser.add_argument("--dry-run",      action="store_true",
                        help="명령어만 출력하고 실제 실행하지 않음")

    args = parser.parse_args()
    run_yolo_collection_loop(
        n_sets       = args.sets,
        n_snapshots  = args.snapshots,
        out_dir      = args.output,
        scenario_dir = args.scenario_dir,
        diversify    = args.diversify,
        gazebo_wait  = args.gazebo_wait,
        headless     = args.headless,
        val_ratio    = args.val_ratio,
        dry_run      = args.dry_run,
    )


if __name__ == "__main__":
    main()
