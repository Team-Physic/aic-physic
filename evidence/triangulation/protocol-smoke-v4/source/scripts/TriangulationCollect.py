# 고정 manifest의 관측 자세만 실행하고 각 조건의 성공·실패를 빠짐없이 저장한다.
# GT는 정지한 task board의 entrance snapshot이며 카메라 TF는 RGB 촬영 시각에 조회한다.
# 저장 NPZ에는 BGR 영상·depth·투영행렬·GT가 들어가며 YOLO는 별도 평가에서 실행한다.
# 실제 삽입은 수행하지 않는다. ROS policy loader가 같은 이름의 class를 불러온다.
# 환경: AIC_TRIANGULATION_JOB 필수 JSON 경로. 조건·저장 경로·가시성 수치는 해당 JSON을 따른다.
from __future__ import annotations

import json
import os
import time
from pathlib import Path

import cv2
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion

from triangulation_protocol import CAMERAS, look_at_tcp, visibility, write_json, digest
from phy_data_collection.policy.PortOffsetCollect import PortOffsetCollect
from phy_data_collection.policy import dataset, motion
from phy_data_collection.policy.se3 import transform_matrix, matrix_quaternion
from phy_policy.ros.final_policy_vision import PortVision, parse_model_class


def matrix(transform):
    """ROS Transform을 base 기준 4×4 행렬로 바꾼다."""
    p, q = transform.translation, transform.rotation
    return transform_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


class TriangulationCollect(PortOffsetCollect):
    def __init__(self, parent_node):
        """기존 controller·RGB·depth 구독을 재사용하고 계획 파일을 읽는다."""
        self.job = json.loads(Path(os.environ['AIC_TRIANGULATION_JOB']).read_text())
        super().__init__(parent_node)
        self.output = Path(self.job['output'])
        self.output.mkdir(parents=True, exist_ok=True)

    def capture_case(self, case, port_frame, port_snapshot, get_observation, move_robot):
        """목표 자세 도달을 확인한 뒤 같은 시각의 RGB·TF와 대응 depth를 보존한다."""
        base_port = matrix(port_snapshot.transform)
        tcp_optical = np.linalg.inv(self._tool0_tcp) @ self._tool0_optical['center']
        target = look_at_tcp(base_port, tcp_optical, case['distance_m'], case['angle_deg'])
        q = matrix_quaternion(target[:3, :3])
        pose = Pose(position=Point(x=float(target[0,3]), y=float(target[1,3]), z=float(target[2,3])),
                    orientation=Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])))
        row = dict(case, target=self.job['target'], card_count=self.job['card_count'],
                   commanded_base_T_tcp=target.tolist(), gt_base_T_port=base_port.tolist(),
                   gt_stamp_ns=dataset._stamp_ns(port_snapshot.header.stamp))
        start = motion._tcp_pose(get_observation())
        if start is None:
            return dict(row, status='missing_controller_state')
        guard = motion.prepare_haptic_guard(self, get_observation)
        if guard is None:
            return dict(row, status='haptic_baseline_unavailable')
        steps = min(120, max(10, int(np.ceil(np.linalg.norm(target[:3,3] - [start.position.x, start.position.y, start.position.z])/.003))))
        if not motion._follow(self, move_robot, start, pose, steps, .03, 'triangulation',
                              motion.STIFFNESS, motion.DAMPING, get_observation=get_observation, haptic_guard=guard):
            motion.retreat_to_pose(self, get_observation, move_robot, start, motion.STIFFNESS, motion.DAMPING)
            return dict(row, status='haptic_contact')
        stamp = self.set_pose_target(move_robot, pose)
        deadline = time.monotonic() + self.settle_timeout_s
        translation_error = rotation_error = float('inf')
        settled = None
        while time.monotonic() < deadline:
            settled, metrics = motion.wait_for_pose_convergence(self, get_observation, pose, stamp, guard)
            if settled is None:
                break
            current = motion._tcp_pose(get_observation())
            translation_error, rotation_error = motion._pose_error(current, pose)
            if translation_error <= .002 and rotation_error <= np.deg2rad(1):
                break
            stamp = settled
        row['settle'] = {k: float(v) if isinstance(v, (float, np.floating)) and np.isfinite(v) else
                         (None if isinstance(v, (float, np.floating)) else v) for k,v in metrics.items()}
        if settled is None:
            return dict(row, status=metrics.get('failure_reason', 'settle_timeout'))
        row.update(command_position_error_m=float(translation_error), command_angle_error_rad=float(rotation_error))
        # 기존 settle은 움직임의 정지를 판정하므로, 명령이 clamp되어 멈춘 경우를 별도로 거부한다.
        if translation_error > .002 or rotation_error > np.deg2rad(1):
            return dict(row, status='pose_not_reached')
        deadline = time.monotonic() + self.sync_wait_timeout_s
        depths = {}
        while time.monotonic() < deadline:
            observation, timing = dataset.wait_for_observation(self, get_observation, settled)
            if observation is None:
                break
            capture_stamp = dataset._stamp_ns(observation.center_image.header.stamp)
            depths = {c: self.depth_image_at(c, capture_stamp) for c in CAMERAS}
            if all(m is not None and dataset._stamp_ns(m.header.stamp) == capture_stamp for m in depths.values()):
                break
            settled = capture_stamp
        else:
            observation = None
            timing = {'rejection_reason': 'exact_rgb_depth_sync_timeout'}
        row['timing'] = timing
        if observation is None:
            return dict(row, status='rgb_sync_unavailable')
        for camera in CAMERAS:
            image = dataset.image_for_camera(observation,camera)
            info = dataset.camera_info_for(observation,camera)
            if info is None or (info.width,info.height) != (image.width,image.height):
                return dict(row,status='calibration_resolution_mismatch')
        vision = PortVision(self, parse_model_class(self.job['target']))
        data = vision._projection_data(observation)
        if data is None:
            return dict(row, status='projection_unavailable')
        after = self.lookup_latest_stamped('base_link', port_frame)
        if not np.allclose(base_port, matrix(after.transform), atol=1e-7, rtol=0):
            return dict(row, status='ground_truth_changed')
        corners = np.asarray(dataset._port_outer_corners_base('sfp', port_snapshot.transform))
        archive = {'gt_corners': corners}
        row.update(stamp_ns=data['stamp_ns'], visibility={}, frame_ids={}, camera_stamps_ns={}, depth_stamps_ns={}, camera_info={})
        for camera in CAMERAS:
            message = dataset.image_for_camera(observation, camera)
            info = dataset.camera_info_for(observation, camera)
            depth_msg = depths.get(camera)
            depth = dataset._depth_image_meters(depth_msg) if depth_msg is not None else None
            row['depth_stamps_ns'][camera] = dataset._stamp_ns(depth_msg.header.stamp) if depth_msg else None
            k = np.asarray(info.k).reshape(3,3)
            row['camera_info'][camera] = {'width':info.width,'height':info.height,'k':list(info.k),'d':list(info.d)}
            row['visibility'][camera] = visibility(corners, data['transforms'][camera], k,
                data['images'][camera].shape, depth, self.job['margin_px'], self.job['depth_occlusion_tolerance_m'])
            row['frame_ids'][camera] = message.header.frame_id
            row['camera_stamps_ns'][camera] = dataset._stamp_ns(message.header.stamp)
            for kind in ('images', 'projections', 'transforms', 'camera_origins'):
                archive[f'{kind}_{camera}'] = data[kind][camera]
            archive[f'intrinsic_{camera}'] = k
            if depth is not None:
                archive[f'depth_{camera}'] = depth
        row['eligible'] = sum(v['visible'] for v in row['visibility'].values()) >= 2
        row['actual_center_distance_m'] = float(np.linalg.norm(data['camera_origins']['center']-base_port[:3,3]))
        view = data['camera_origins']['center'] - base_port[:3,3]
        local = base_port[:3,:3].T @ view
        row['actual_angle_deg'] = float(np.rad2deg(np.arctan2(local[0], -local[2])))
        row['archive'] = f'{case["id"]}.npz'
        np.savez_compressed(self.output/row['archive'], **archive)
        row['archive_sha256'] = digest(self.output/row['archive'])
        # 모든 카메라의 실제 영상에 GT 외곽을 표시해 가시성 판정을 육안으로 검토할 수 있게 한다.
        for camera in CAMERAS:
            image = data['images'][camera].copy()
            uv = row['visibility'][camera]['corners_uv']
            if all(v is not None for point in uv for v in point):
                cv2.polylines(image, [np.rint(uv).astype('int32')], True, (0,200,255), 2)
            cv2.putText(image, f'GT {self.job["target"]}: {row["visibility"][camera]["reason"]}',
                        (12,28), cv2.FONT_HERSHEY_SIMPLEX, .65, (0,200,255), 2)
            cv2.imwrite(str(self.output/f'{case["id"]}_{camera}_gt.png'), image)
        return dict(row, status='captured' if row['eligible'] else 'ineligible_view')

    def insert_cable(self, task, get_observation, move_robot, send_feedback):
        """각 고정 조건을 한 번씩 시도하고 실패도 기록한다. 삽입 동작은 수행하지 않는다."""
        send_feedback('triangulation evaluation capture')
        episode = self.capture_root / f'{os.getpid()}_{task.id}'
        episode.mkdir(parents=True, exist_ok=True)
        counts = dict(lift_up=0, approach=0, collect=0, attempts=0, haptic_contacts=0, clearance_rejections=0)
        port_frame = self._port_frame(task)
        problem = None
        snapshot = None
        try:
            if not self._wait_for_tf('base_link', port_frame):
                raise RuntimeError('port TF unavailable')
            snapshot = self.lookup_latest_stamped('base_link', port_frame)
            context = {'counts': counts, **motion.control_for(task)}
            if not motion.lift(self, context, get_observation, move_robot):
                raise RuntimeError('initial lift failed')
        except Exception as exc:
            problem = str(exc)
        for case in self.job['cases']:
            counts['attempts'] += 1
            try:
                row = dict(case, status='setup_failed', detail=problem) if problem else self.capture_case(
                    case, port_frame, snapshot, get_observation, move_robot)
            except Exception as exc:
                row = dict(case, status='capture_error', detail=f'{type(exc).__name__}: {exc}')
            write_json(self.output/f'{case["id"]}.json', row)
            counts['collect'] += row['status'] == 'captured'
            self.get_logger().info(f'TRIANGULATION {case["id"]}: {row["status"]}')
        # status=ok는 모든 예정 조건의 종료 기록을 뜻한다. 가시성과 YOLO 성능은 별도 집계한다.
        self._finish(episode, task, counts, 'ok')
        # 촬영 완료를 InsertCable 성공으로 반환하지 않는다. runner가 완료 기록을 읽고 종료한다.
        while not self.stop_file.exists():
            time.sleep(.1)
        return False
