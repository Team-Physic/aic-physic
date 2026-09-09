# 고정 평가 계획과 목표를 향하는 카메라 자세를 만든다.
# seed로 장면을 한 번 생성한 뒤 모든 실수값을 JSON에 보존하여 PC 간 재추출을 피한다.
# 위치 단위는 m, 각도는 degree이며 포트 입구의 외향 법선은 포트 로컬 -Z이다.
# 관측 평면은 포트 로컬 XZ, 중앙 카메라 optical +Z가 목표 중심을 향한다.
from __future__ import annotations

import hashlib
import itertools
import json
import os
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PREFIX = ROOT / 'ws_aic/src/.pixi/envs/default'
sys.path[:0] = [str(ROOT/'ws_aic/src/phy/phy_data_collection'),
                str(ROOT/'ws_aic/src/phy/phy_policy')]
CAMERAS = ('left', 'center', 'right')


def configure_environment():
    """PC별 AIC override를 제거한다. 평가 설정은 계획·CLI·고정 소스만 사용한다."""
    for name in list(os.environ):
        if name.startswith('AIC_'):
            del os.environ[name]
    os.environ.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp', AIC_REID_ENCODER='none')


def digest(path):
    """원본 파일을 변경하지 않고 재현성 식별자를 계산한다."""
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def make_plan(seed, distances, lights, margin, weights):
    """5개 카드 수 × 10개 목표 × 2개 배치를 균등 생성하고 실행 순서만 섞는다."""
    from phy_data_collection.runner import scenario
    from phy_data_collection.runner.cli import build_parser
    from phy_data_collection.runner.constants import WORLD_TEMPLATE_PATH
    if len(distances) != 3 or len(lights) != 3 or len(set(distances)) != 3 or len(set(lights)) != 3:
        raise ValueError('three distinct distances and light scales are required')
    if not all(np.isfinite(v) and v > 0 for v in [*distances, *lights]) or not np.isfinite(margin) or margin < 0:
        raise ValueError('distances/lights must be positive finite; margin nonnegative')
    args = build_parser().parse_args(['--port-type', 'sfp'])
    layouts = []
    for index, (count, rail, port, repeat) in enumerate(itertools.product(range(1, 6), range(5), range(2), range(2))):
        layout_seed = seed + index * 1_000_003
        rng = random.Random(layout_seed)
        rails = [rail, *rng.sample([r for r in range(5) if r != rail], count - 1)]
        mask = sum(1 << r for r in rails)
        old_id, trial, _ = scenario._make_sfp_trial(index, mask, rng, args)
        task = trial['tasks'].pop(old_id)
        task.update(target_module_name=f'nic_card_mount_{rail}', port_name=f'sfp_port_{port}')
        task_id = f'triangulation_{index:03d}_cards{mask:05b}_rail{rail}_sfp_port_{port}'
        trial['tasks'][task_id] = task
        config = {'scoring': scenario._scoring_section(),
                  'task_board_limits': scenario._task_board_limits_section(),
                  'robot': scenario._robot_section(rng, args.robot_joint_noise_rad),
                  'trials': {f'layout_{index:03d}': trial}}
        layouts.append({'id': f'layout_{index:03d}', 'seed': layout_seed, 'repeat': repeat,
                        'card_count': count, 'active_rails': sorted(rails),
                        'target': f'SFP_{rail}{port}', 'config': config})
    random.Random(seed).shuffle(layouts)
    # GI를 양쪽 renderer에서 동일하게 제외한다. software smoke만 다른 world를 쓰지 않는다.
    worlds = []
    for scale in lights:
        tree = ET.parse(WORLD_TEMPLATE_PATH)
        for parent in tree.iter():
            for child in list(parent):
                if child.tag == 'plugin' and 'GlobalIlluminationVct' in child.get('filename', ''):
                    parent.remove(child)
        for light in tree.findall('./world/light'):
            intensity = light.find('intensity')
            if intensity is None:
                intensity = ET.SubElement(light, 'intensity')
                intensity.text = '1'
            intensity.text = format(float(intensity.text) * scale, '.17g')
        worlds.append(ET.tostring(tree.getroot(), encoding='unicode'))
    annotation = (ROOT/'ws_aic/src/phy/phy_data_collection/config/ur_gz_depth.urdf.xacro').read_text()
    anchor = '<camera name="${name}_annotation_depth">'
    if annotation.count(anchor) != 1:
        raise ValueError('unexpected annotation camera template')
    # Depth CameraInfo가 RGB topic에 섞이지 않게 분리한다. RGB 추론 경로의 K는 그대로 유지한다.
    annotation = annotation.replace(anchor, anchor+'\n          <camera_info_topic>/${name}/depth_camera_info</camera_info_topic>')
    return {'schema': 2, 'seed': seed, 'layout_count': 100, 'annotation_description': annotation,
            'distances_m': list(distances), 'angles_deg': [-15, 0, 15],
            'light_scales': list(lights), 'margin_px': margin,
            'min_visible_cameras': 2, 'depth_occlusion_tolerance_m': 0.002,
            'angle_frame': 'port local XZ; outward=-Z; camera optical +Z looks at port; optical +Y=port +Y',
            'gt_contract': 'fixed task-board entrance snapshot, checked unchanged before/after capture',
            'model_sha256': digest(weights), 'world_source_sha256': digest(WORLD_TEMPLATE_PATH),
            'worlds': worlds, 'layouts': layouts}


def cases(plan, layout_id, light_index):
    """같은 배치·조명의 9개 조건을 먼 거리부터 생성한다. 실패 시 대체하지 않는다."""
    return [{'id': f'{layout_id}_l{light_index}_d{di}_a{ai}',
             'layout_id': layout_id, 'light_index': light_index,
             'light_scale': plan['light_scales'][light_index],
             'distance_m': distance, 'angle_deg': angle}
            for di, distance in sorted(enumerate(plan['distances_m']), key=lambda p: -p[1])
            for ai, angle in enumerate(plan['angles_deg'])]


def look_at_tcp(base_port, tcp_optical, distance, angle_deg):
    """포트 외향 -Z와 +X 사이의 부호 있는 각도로 optical pose를 만들고 TCP로 환산한다."""
    theta = np.deg2rad(angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    optical = np.eye(4)
    optical[:3, :3] = base_port[:3, :3] @ np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
    optical[:3, 3] = base_port[:3, 3] - distance * optical[:3, 2]
    return optical @ np.linalg.inv(tcp_optical)


def visibility(corners, camera_from_base, intrinsic, image_shape, depth, margin, tolerance):
    """외곽 여백과 포트 평면 5×5 지점 앞의 depth 가림을 검사한다. depth 부재는 통과시키지 않는다."""
    height, width = image_shape[:2]
    points = (camera_from_base @ np.c_[corners, np.ones(4)].T).T[:, :3]
    pixels = (intrinsic @ points.T).T
    uv = pixels[:, :2] / pixels[:, 2:3]
    inside = bool(np.isfinite(uv).all() and np.all(points[:, 2] > 0) and
                  np.all(uv >= margin) and np.all(uv[:, 0] < width-margin) and np.all(uv[:, 1] < height-margin))
    result = {'corners_uv': [[float(v) if np.isfinite(v) else None for v in row] for row in uv],
              'inside_margin': inside, 'visible': False}
    if not inside:
        return dict(result, reason='outside_margin')
    if depth is None or depth.ndim != 2 or depth.shape[0]*width != depth.shape[1]*height:
        return dict(result, reason='depth_unavailable')
    checks = []
    for a, b in itertools.product(np.linspace(0, 1, 5), repeat=2):
        xyz = (1-b)*((1-a)*corners[0]+a*corners[1]) + b*((1-a)*corners[3]+a*corners[2])
        point = (camera_from_base @ np.append(xyz, 1))[:3]
        pixel = intrinsic @ point
        u, v = pixel[:2] / pixel[2]
        x = int(np.clip(round((u+.5)*depth.shape[1]/width-.5), 0, depth.shape[1]-1))
        y = int(np.clip(round((v+.5)*depth.shape[0]/height-.5), 0, depth.shape[0]-1))
        patch = depth[max(0,y-1):y+2, max(0,x-1):x+2]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        observed = float(np.quantile(valid, .2)) if valid.size else None
        checks.append({'expected_depth_m': float(point[2]), 'observed_depth_m': observed})
    if any(p['observed_depth_m'] is None for p in checks):
        reason = 'depth_unknown'
    elif any(p['observed_depth_m'] + tolerance < p['expected_depth_m'] for p in checks):
        reason = 'occluded'
    else:
        reason = 'visible'
    return dict(result, visible=reason == 'visible', reason=reason, depth_checks=checks)


def write_json(path, data):
    """완성된 JSON만 결과 경로에 노출한다."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    temporary.replace(path)


def job_complete(returncode, rows, expected_ids):
    """인프라 실패·미기록을 완료로 오인하지 않으며 정상 관측 불충족은 종료 결과로 인정한다."""
    terminal = {'captured', 'ineligible_view', 'pose_not_reached', 'haptic_contact'}
    return returncode == 0 and len(rows) == len(expected_ids) and {r['id'] for r in rows} == set(expected_ids) and all(r['status'] in terminal for r in rows)
