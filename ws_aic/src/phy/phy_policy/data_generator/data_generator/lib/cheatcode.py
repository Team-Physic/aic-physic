import numpy as np

# transforms3d 0.4.1 calls np.maximum_sctype which was removed in NumPy 2.0.
# Restore it from numpy._core so the system package loads cleanly.
if not hasattr(np, "maximum_sctype"):
    from numpy._core.numerictypes import maximum_sctype as _maximum_sctype
    np.maximum_sctype = _maximum_sctype

from geometry_msgs.msg import Point, Pose, Quaternion, Transform
from transforms3d._gohlketransforms import quaternion_multiply, quaternion_slerp


def _rotate_vector(q_wxyz: tuple[float, float, float, float], vec_xyz: np.ndarray) -> np.ndarray:
    # Rotate a local-frame vector into the parent/world frame using the frame quaternion.
    q_vec = (0.0, float(vec_xyz[0]), float(vec_xyz[1]), float(vec_xyz[2]))
    q_inv = (q_wxyz[0], -q_wxyz[1], -q_wxyz[2], -q_wxyz[3])
    rotated = quaternion_multiply(quaternion_multiply(q_wxyz, q_vec), q_inv)
    return np.array([rotated[1], rotated[2], rotated[3]], dtype=float)


def _project_perpendicular(vec_xyz: np.ndarray, axis_xyz: np.ndarray) -> np.ndarray:
    # Keep only the part of vec_xyz that is perpendicular to axis_xyz.
    return vec_xyz - axis_xyz * float(np.dot(vec_xyz, axis_xyz))


class CheatCodePlanner:
    def __init__(self, i_gain: float = 0.15, max_integrator_windup: float = 0.05):
        self.i_gain = float(i_gain)
        self.max_integrator_windup = float(max_integrator_windup)
        self.reset()

    def reset(self) -> None:
        self.tip_x_error_integrator = 0.0
        self.tip_y_error_integrator = 0.0

    def build_pose(
        self,
        port_transform: Transform,
        plug_transform: Transform,
        gripper_transform: Transform,
        slerp_fraction: float = 1.0,
        position_fraction: float = 1.0,
        z_offset: float = 0.1,
        reset_xy_integrator: bool = False,
    ) -> tuple[Pose, dict[str, float]]:
        q_port = (
            port_transform.rotation.w,
            port_transform.rotation.x,
            port_transform.rotation.y,
            port_transform.rotation.z,
        )
        q_plug = (
            plug_transform.rotation.w,
            plug_transform.rotation.x,
            plug_transform.rotation.y,
            plug_transform.rotation.z,
        )
        q_plug_inv = (
            -q_plug[0],
            q_plug[1],
            q_plug[2],
            q_plug[3],
        )
        q_diff = quaternion_multiply(q_port, q_plug_inv)

        q_gripper = (
            gripper_transform.rotation.w,
            gripper_transform.rotation.x,
            gripper_transform.rotation.y,
            gripper_transform.rotation.z,
        )
        q_gripper_target = quaternion_multiply(q_diff, q_gripper)
        q_gripper_slerp = quaternion_slerp(q_gripper, q_gripper_target, slerp_fraction)

        gripper_xyz = np.array(
            [
                gripper_transform.translation.x,
                gripper_transform.translation.y,
                gripper_transform.translation.z,
            ],
            dtype=float,
        )
        port_xyz = np.array(
            [
                port_transform.translation.x,
                port_transform.translation.y,
                port_transform.translation.z,
            ],
            dtype=float,
        )
        plug_xyz = np.array(
            [
                plug_transform.translation.x,
                plug_transform.translation.y,
                plug_transform.translation.z,
            ],
            dtype=float,
        )
        plug_tip_gripper_offset = gripper_xyz - plug_xyz
        port_axis = _rotate_vector(q_port, np.array([0.0, 0.0, -1.0], dtype=float)) # port 바깥쪽 접근축을 baselink 좌표계에 정렬
        norm = float(np.linalg.norm(port_axis))
        if norm <= 1e-9:
            port_axis = np.array([0.0, 0.0, -1.0], dtype=float)
        else:
            port_axis /= norm

        target_plug_xyz = port_xyz + port_axis * float(z_offset)

        tip_x_error = port_xyz[0] - plug_xyz[0]
        tip_y_error = port_xyz[1] - plug_xyz[1]
        tip_delta = plug_xyz - port_xyz
        tip_axis_distance = float(np.dot(tip_delta, port_axis))
        tip_distance = float(np.linalg.norm(tip_delta))

        if reset_xy_integrator:
            self.reset()
        else:
            self.tip_x_error_integrator = float(
                np.clip(
                    self.tip_x_error_integrator + tip_x_error,
                    -self.max_integrator_windup,
                    self.max_integrator_windup,
                )
            )
            self.tip_y_error_integrator = float(
                np.clip(
                    self.tip_y_error_integrator + tip_y_error,
                    -self.max_integrator_windup,
                    self.max_integrator_windup,
                )
            )

        # z_offset is the intended port-to-plug-tip distance along the outward port approach axis.
        # Apply the current gripper-to-plug offset so the commanded TCP pose puts the plug
        # tip, not the gripper frame, at that point.
        correction_xyz = _project_perpendicular(
            np.array(
                [
                    self.i_gain * self.tip_x_error_integrator,
                    self.i_gain * self.tip_y_error_integrator,
                    0.0,
                ],
                dtype=float,
            ),
            port_axis,
        )
        corrected_target_plug_xyz = target_plug_xyz + correction_xyz
        target_xyz = corrected_target_plug_xyz + plug_tip_gripper_offset

        blend_xyz = (
            position_fraction * target_xyz[0] + (1.0 - position_fraction) * gripper_xyz[0],
            position_fraction * target_xyz[1] + (1.0 - position_fraction) * gripper_xyz[1],
            position_fraction * target_xyz[2] + (1.0 - position_fraction) * gripper_xyz[2],
        )

        pose = Pose(
            position=Point(
                x=float(blend_xyz[0]),
                y=float(blend_xyz[1]),
                z=float(blend_xyz[2]),
            ),
            orientation=Quaternion(
                w=float(q_gripper_slerp[0]),
                x=float(q_gripper_slerp[1]),
                y=float(q_gripper_slerp[2]),
                z=float(q_gripper_slerp[3]),
            ),
        )
        extras = {
            "tip_x_error": float(tip_x_error),
            "tip_y_error": float(tip_y_error),
            "tip_z_error": float(port_xyz[2] - plug_xyz[2]),
            "tip_distance": tip_distance,
            "tip_axis_distance": tip_axis_distance,
            "tip_x_error_integrator": float(self.tip_x_error_integrator),
            "tip_y_error_integrator": float(self.tip_y_error_integrator),
            "target_x": float(target_xyz[0]),
            "target_y": float(target_xyz[1]),
            "target_z": float(target_xyz[2]),
            "target_plug_x": float(corrected_target_plug_xyz[0]),
            "target_plug_y": float(corrected_target_plug_xyz[1]),
            "target_plug_z": float(corrected_target_plug_xyz[2]),
            "z_offset": float(z_offset),
            "slerp_fraction": float(slerp_fraction),
            "position_fraction": float(position_fraction),
            "axis_perpendicular_correction": {
                "x": float(correction_xyz[0]),
                "y": float(correction_xyz[1]),
                "z": float(correction_xyz[2]),
            },
            "port_axis": {
                "x": float(port_axis[0]),
                "y": float(port_axis[1]),
                "z": float(port_axis[2]),
            },
            # [수정 이유] 매 스텝의 실제 파지 오프셋(gripper/tcp와 plug 사이의 상대 포즈)을 기록하여
            # 학습 데이터셋(steps.jsonl)에서 그리퍼 상태를 정확히 추적할 수 있도록 함.
            "gripper_offset": {
                "x": float(plug_tip_gripper_offset[0]),
                "y": float(plug_tip_gripper_offset[1]),
                "z": float(plug_tip_gripper_offset[2]),
            },
        }
        return pose, extras
