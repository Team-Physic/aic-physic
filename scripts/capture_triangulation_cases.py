#!/usr/bin/env python3
# 기존 PHY 수집 runner를 재사용해 카드 수와 seed가 고정된 Gazebo 평가 장면을 만든다.
# 로봇 제어·동기화·가시성 판정·rosbag 저장은 기존 수집 정책을 그대로 사용한다.
# 생성된 trial/world, 영상, 카메라 TF와 정답 annotation을 원래 수집 위치에 보존한다.
# software rendering용 생성 world에서만 VCT GI를 제거하며 원본 world는 수정하지 않는다.
# 인자: --seed 필수 정수, --exact-card-count 필수 1/3/5, 나머지는 기존 collector CLI와 동일.
# 환경: DISPLAY는 실행 중인 컨테이너 X 서버, LIBGL_ALWAYS_SOFTWARE=1은 CPU 렌더링 선택.
from __future__ import annotations

import os
import shlex
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PREFIX = ROOT/'ws_aic/src/.pixi/envs/default'
sys.path.insert(0, str(ROOT/'ws_aic/src/phy/phy_data_collection'))
os.environ['PATH'] = str(PREFIX/'bin') + os.pathsep + os.environ['PATH']
os.environ.setdefault('AMENT_PREFIX_PATH', str(PREFIX))
from phy_data_collection.runner import main as runner, runtime, scenario, world
from phy_data_collection.runner.cli import build_parser

START_LOGGED_PROCESS = runtime._start_logged_process


def launch_with_software_renderer(command, **kwargs):
    """소유 launcher에 렌더러와 trial별 ROS_HOME을 설정해 기존 lock 권한 충돌을 피한다."""
    if command[0] == 'distrobox':
        launcher = Path(command[-1])
        contents = launcher.read_text()
        anchor = '. /ws_aic/install/setup.bash'
        assert anchor in contents
        contents = contents.replace(anchor, anchor + '\n' + '\n'.join([
            'export DISPLAY=:99',
            'export LIBGL_ALWAYS_SOFTWARE=1',
            'export __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/aic-no-egl.json',
            'export LP_NUM_THREADS=4',
            f'export ROS_HOME={shlex.quote(str(launcher.parent / "ros_home"))}',
            'mkdir -p "$ROS_HOME"',
        ]), 1)
        launcher.write_text(contents)
    return START_LOGGED_PROCESS(command, **kwargs)


def fixed_card_config(index, rng, args):
    """seed별 활성 레일을 선택하고 기존 생성기로 고정 카드 수 trial을 만든다."""
    rails = rng.sample(range(5), args.exact_card_count)
    mask = sum(1 << rail for rail in rails)
    task_id, trial, params = scenario._make_sfp_trial(index, mask, rng, args)
    robot = scenario._robot_section(rng, args.robot_joint_noise_rad)
    config = {'scoring': scenario._scoring_section(),
              'task_board_limits': scenario._task_board_limits_section(),
              'trials': {f'trial_{index:04d}_sfp': trial}, 'robot': robot}
    params[task_id]['robot_home_joint_positions'] = robot['home_joint_positions']
    return config, params


def software_world(index, rng, args, path):
    """생성된 평가 world에만 렌더링 호환성 설정을 적용한다."""
    result, lighting = world.write_randomized_world(index, rng, args, path)
    if result is None:
        raise ValueError('evaluation capture requires --randomize-lighting true')
    tree = ET.parse(result)
    for parent in tree.iter():
        for child in list(parent):
            if child.tag == 'plugin' and 'GlobalIlluminationVct' in child.attrib.get('filename', ''):
                parent.remove(child)
    tree.write(result, encoding='unicode')
    lighting['vct_removed_for_software_rendering'] = True
    return result, lighting


def main():
    """기존 lifecycle과 종료 처리를 사용해 단일 worker의 평가 trial들을 실행한다."""
    parser = build_parser()
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--exact-card-count', type=int, choices=[1, 3, 5], required=True)
    args = parser.parse_args()
    if args.port_type != 'sfp' or args.workers != 1:
        parser.error('this evaluation adapter supports SFP and one worker')
    runner._prepare_args(args)
    args.split_assignments = runner._split_assignments(args)
    runner.make_trial_config = fixed_card_config
    runner.write_randomized_world = software_world
    runtime._start_logged_process = launch_with_software_renderer
    if not args.dry_run:
        runner._write_metadata(args)
    run_id = f'triangulation-{args.seed}-{time.strftime("%Y%m%dT%H%M%S")}-{os.getpid()}'
    result = runner._run_worker(args, run_id, 0, list(range(args.trials)))
    if result == 0 and not args.dry_run:
        return runner._finalize_dataset(args)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
