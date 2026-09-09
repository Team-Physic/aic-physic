#!/usr/bin/env python3
# 저장된 Gazebo 카메라·TF로 현재 PortVision의 검출 포함 위치 오차를 평가한다.
# 정답 entrance TF는 채점·계산 검증에만 사용하며 YOLO 추론 입력에 전달하지 않는다.
# 동일 촬영 시각의 세 영상을 묶고 base_link 기준 meter 좌표를 mm 오차로 보고한다.
# 원본 bag·모델 해시, 프레임별 실패와 결과를 남겨 동일 입력 재실행을 지원한다.
# 인자: --bag 반복 필수, --output 필수, --weights 기본 ws_aic/model/best.pt.
# --stride 양의 프레임 간격(기본 1), --max-frames 0은 전체, --device 기본 cpu.
# --variants 기본 original; dark/bright는 명시적 합성 밝기, drop_*는 합성 카메라 차폐.
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
PREFIX = ROOT / 'ws_aic/src/.pixi/envs/default'
os.environ.setdefault('AMENT_PREFIX_PATH', str(PREFIX))
os.environ.setdefault('AIC_REID_ENCODER', 'none')
os.environ.setdefault('PYTHONDONTWRITEBYTECODE', '1')
sys.path[:0] = [str(ROOT / 'ws_aic/src/phy/phy_policy'),
                str(ROOT / 'ws_aic/src/phy/phy_data_collection')]

import cv2
import numpy as np
import rosbag2_py
import tf2_ros
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from ultralytics import YOLO
from phy_data_collection.policy.dataset import _port_outer_corners_base
from phy_policy.ros.final_policy_vision import CAMERAS, PortVision, parse_model_class, project_point


def sha256(path):
    """대용량 원본 bag도 메모리에 올리지 않고 입력 식별자를 계산한다."""
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def messages(path, topics):
    """원본 MCAP의 선택 topic을 기록 순서대로 역직렬화한다."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
             if t.name in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        yield topic, deserialize_message(raw, types[topic])


class RecordedPolicy:
    """원래 camera TF 조회 계약을 기록된 TF로 충족한다. GT는 이 인터페이스 밖에서 채점한다."""
    def __init__(self, bag):
        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=100000))
        self.infos = {}
        self.frames = set()
        self.warnings = Counter()
        topics = ['/tf', '/tf_static', '/scoring/tf'] + [f'/{c}_camera/camera_info' for c in CAMERAS]
        for topic, msg in messages(bag, topics):
            if topic.endswith('camera_info'):
                self.infos[topic.split('/')[1].removesuffix('_camera')] = msg
            else:
                for transform in msg.transforms:
                    self.frames.add(transform.child_frame_id)
                    if topic == '/tf_static':
                        self.buffer.set_transform_static(transform, 'recorded')
                    else:
                        self.buffer.set_transform(transform, 'recorded')

    def lookup_transform_at(self, target, source, stamp):
        """촬영 시각에 보간한 TF를 반환하며 최신 값으로 대체하지 않는다."""
        return self.buffer.lookup_transform(target, source, Time.from_msg(stamp))

    def get_logger(self):
        return self

    def info(self, message):
        pass

    def warn(self, message):
        self.warnings[str(message)] += 1


def observations(bag, infos, counts):
    """같은 nanosecond의 RGB 세 장만 반환하고 불일치 프레임을 별도 집계한다."""
    pending = defaultdict(dict)
    for topic, msg in messages(bag, [f'/{c}_camera/image' for c in CAMERAS]):
        camera = topic.split('/')[1].removesuffix('_camera')
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        pending[stamp][camera] = msg
        if len(pending[stamp]) == 3:
            group = pending.pop(stamp)
            counts['synchronized_triplets'] += 1
            yield SimpleNamespace(**{**{f'{c}_image': group[c] for c in CAMERAS},
                                     **{f'{c}_camera_info': infos.get(c) for c in CAMERAS}})
        # 정상 bag은 카메라 간격이 한 프레임 안이다. 중단된 topic이 메모리를 채우지 않게 한다.
        while len(pending) > 20:
            unmatched = pending.pop(min(pending))
            counts['unpaired_images'] += len(unmatched)
    counts['unpaired_images'] += sum(map(len, pending.values()))


def summarize(rows):
    """실패를 분모에서 제거하지 않고 coverage와 성공 표본의 위치 오차를 분리한다."""
    unscorable = sum(r['status'] == 'ground_truth_unavailable' for r in rows)
    input_frames = len(rows)
    rows = [r for r in rows if r['status'] != 'ground_truth_unavailable']
    errors = [r['error_mm'] for r in rows if r.get('error_mm') is not None]
    result = {'input_frames': input_frames, 'unscorable_frames': unscorable,
              'attempts': len(rows), 'estimates': len(errors),
              'coverage': len(errors) / len(rows) if rows else None,
              'failures': dict(Counter(r['status'] for r in rows if r['status'] != 'ok'))}
    for name, q in [('median_mm', 50), ('p95_mm', 95), ('max_mm', 100)]:
        result[name] = float(np.percentile(errors, q)) if errors else None
    result['within_5mm_all_attempts'] = sum(x <= 5 for x in errors) / len(rows) if rows else None
    return result


def evaluate_bag(bag, model, args):
    """현재 추론 경로와 정답 2D 계산 경로를 같은 프레임에서 독립 실행한다."""
    match = re.search(r'cards([01]+)_rail(\d+)_sfp_port_(\d+)', bag.name)
    if not match:
        raise ValueError(f'bag name must specify cards<mask>_rail<N>_sfp_port_<N>: {bag}')
    mask, rail, port = match.groups()
    target = parse_model_class(f'SFP_{rail}{port}')
    policy = RecordedPolicy(bag)
    frames = [f for f in policy.frames if f'nic_card_mount_{rail}/' in f
              and f.endswith(f'sfp_port_{port}_link_entrance')]
    if len(frames) != 1:
        raise ValueError(f'expected unique entrance frame, got {frames}')
    vision = PortVision(policy, target, model=model)
    counts = Counter()
    rows = []
    retained = set()
    output = args.output / bag.name
    output.mkdir(parents=True, exist_ok=True)
    for index, observation in enumerate(observations(bag, policy.infos, counts)):
        if index % args.stride:
            continue
        if args.max_frames and counts['evaluated_triplets'] >= args.max_frames:
            break
        counts['evaluated_triplets'] += 1
        stamp = observation.center_image.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        unavailable = 'ground_truth_unavailable'
        try:
            gt_tf = policy.lookup_transform_at('base_link', frames[0], stamp).transform
            unavailable = 'invalid_tf_or_input'
            data = vision._projection_data(observation)
            if data is None:
                raise ValueError('invalid camera calibration or synchronization')
        except Exception as exc:
            for variant in ['geometry', *args.variants]:
                rows.append({'stamp_ns': stamp_ns, 'variant': variant,
                             'status': unavailable, 'detail': str(exc), 'error_mm': None})
            continue
        corners = np.asarray(_port_outer_corners_base('sfp', gt_tf))
        truth = corners.mean(axis=0)
        gt_detections = {}
        visible = {}
        for camera in CAMERAS:
            uv = np.asarray([project_point(data['projections'][camera], p) for p in corners])
            height, width = data['images'][camera].shape[:2]
            depth = (data['transforms'][camera] @ np.append(truth, 1.0))[2]
            visible[camera] = bool(depth > 0 and np.all(uv >= 0) and
                                   np.all(uv[:, 0] < width) and np.all(uv[:, 1] < height))
            gt_detections[camera] = [{'class_name': target.class_name, 'confidence': 1.0,
                                     'keypoints': uv, 'uv': uv.mean(axis=0)}]
        for variant in ['geometry', *args.variants]:
            images = data['images']
            if variant == 'geometry':
                detections = gt_detections
            else:
                if variant in ('dark', 'bright'):
                    scale = 0.5 if variant == 'dark' else 1.5
                    images = {c: np.clip(im.astype(float)*scale, 0, 255).astype('uint8') for c, im in images.items()}
                elif variant.startswith('drop_'):
                    blocked = variant.removeprefix('drop_').split('_')
                    images = {c: np.zeros_like(im) if c in blocked else im for c, im in images.items()}
                detections = vision._detect(images, data['headers'])
            candidate_data = dict(data, images=images)
            estimates = vision._estimate_candidates(candidate_data, detections)
            estimate = estimates[0] if estimates else None
            cameras = sum(bool(detections[c]) for c in CAMERAS)
            row = {'stamp_ns': stamp_ns, 'variant': variant, 'card_count': mask.count('1'),
                   'class_name': target.class_name, 'camera_detections': {c: len(detections[c]) for c in CAMERAS},
                   'gt_corners_inside_image': visible,
                   'gt_xyz_m': truth.tolist(), 'status': 'ok' if estimate else
                   ('fewer_than_two_detections' if cameras < 2 else 'geometry_gate_rejected'),
                   'error_mm': float(np.linalg.norm(estimate.xyz-truth)*1000) if estimate else None,
                   'predicted_xyz_m': estimate.xyz.tolist() if estimate else None,
                   'reprojection_rms_px': estimate.reprojection_rms_px if estimate else None,
                   'center_distance_m': float(np.linalg.norm(truth-data['camera_origins']['center']))}
            rows.append(row)
            # 원본 입력과 수치 증거를 최소 한 번씩 보존한다. 합성 변형은 명시적으로 이름 붙인다.
            key = (variant, row['status'])
            if key not in retained:
                retained.add(key)
                stem = f'{stamp_ns}_{variant}_{row["status"]}'
                for camera in CAMERAS:
                    cv2.imwrite(str(output/f'{stem}_{camera}.png'), images[camera])
                (output/f'{stem}.json').write_text(json.dumps({**row,
                    'projections': {c: data['projections'][c].tolist() for c in CAMERAS},
                    'keypoints': {c: [d['keypoints'].tolist() for d in detections[c]] for c in CAMERAS}}, indent=2))
        if counts['evaluated_triplets'] % 20 == 0:
            print(f'{bag.name}: {counts["evaluated_triplets"]} triplets', flush=True)
    (output/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    return {'bag': str(bag), 'target': target.class_name, 'cards_mask': mask,
            'counts': dict(counts), 'gt_frame': frames[0],
            'inputs': {str(p): sha256(p) for p in sorted(bag.glob('*.mcap'))},
            'by_variant': {v: summarize([r for r in rows if r['variant'] == v])
                           for v in ['geometry', *args.variants]}}


def main():
    """조건·버전·입력 해시와 평가 결과를 출력 디렉터리에 보존한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--weights', type=Path, default=ROOT/'ws_aic/model/best.pt')
    parser.add_argument('--stride', type=int, default=1)
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--variants', nargs='+', default=['original'],
                        choices=['original', 'dark', 'bright', 'drop_left', 'drop_center', 'drop_right', 'drop_left_center'])
    args = parser.parse_args()
    if args.stride < 1 or args.max_frames < 0:
        parser.error('stride must be positive; max-frames nonnegative')
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ['AIC_YOLO_DEVICE'] = args.device
    import torch
    torch.set_num_threads(4)
    model = YOLO(str(args.weights))
    report = {'scope': 'recorded Gazebo replay; not physical insertion or independent held-out certification',
              'error_definition': 'Euclidean base_link port entrance position error in mm',
              'geometry_input': 'GT-projected corners; excludes detector error',
              'synthetic_variants': {'dark': 'pixel * 0.5', 'bright': 'pixel * 1.5 clipped',
                                     'drop_*': 'black entire selected camera image'},
              'argv': sys.argv, 'model_sha256': sha256(args.weights),
              'script_sha256': sha256(__file__), 'torch': torch.__version__, 'opencv': cv2.__version__,
              'production_source_sha256': {str(p.relative_to(ROOT)): sha256(p) for p in [
                  ROOT/'ws_aic/src/phy/phy_policy/phy_policy/ros/final_policy_vision.py',
                  ROOT/'ws_aic/src/phy/phy_policy/phy_policy/ros/geometry.py',
                  ROOT/'ws_aic/src/phy/phy_data_collection/phy_data_collection/policy/dataset.py']},
              'scenes': []}
    for bag in args.bag:
        report['scenes'].append(evaluate_bag(bag.resolve(), model, args))
        (args.output/'summary.json').write_text(json.dumps(report, indent=2))
    print(json.dumps([s['by_variant'] for s in report['scenes']], indent=2))


if __name__ == '__main__':
    main()
