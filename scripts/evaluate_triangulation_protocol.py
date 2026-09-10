#!/usr/bin/env python3
# 고정 프로토콜로 저장한 실제 영상과 카메라 행렬을 현재 PortVision으로 재평가한다.
# 평가 완료된 적격 사례를 분모로 사용하며 중단·완료 시 표와 그래프를 저장한다.
# GT는 채점과 계산 전용 비교 경로에만 사용하며 YOLO 입력은 원본 BGR 영상이다.
# 원본 NPZ·모델 해시, 검출 bbox 영상과 mm 오차를 기록하여 GPU PC에서도 재검증한다.
# 인자: --resume은 동일 설정의 완료 case를 재사용하며 기본값은 비활성이다.
# 인자: --capture/--output 필수 경로, --weights 기본 ws_aic/model/best.pt, --device 기본 cpu(GPU는 0).
from __future__ import annotations

import argparse
import hashlib
import inspect
import signal
import json
import os
from pathlib import Path
import sys

from triangulation_report import summary, load_capture, aggregate, write_report
from triangulation_protocol import ROOT, PREFIX, CAMERAS, digest, write_json, configure_environment
configure_environment()
os.environ.setdefault('AMENT_PREFIX_PATH', str(PREFIX))
os.environ['AIC_REID_ENCODER'] = 'none'

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from phy_policy.ros.final_policy_vision import PortVision, parse_model_class, project_point


class ReplayLogger:
    """PortVision의 로그 계약만 제공한다. 추론에 GT 조회 경로를 제공하지 않는다."""
    def get_logger(self):
        return self
    def info(self, message):
        pass
    def warn(self, message):
        print(message)


def evaluate_case(folder, row, model, output):
    """1개 적격 관측에서 production 검출 경로와 GT corner 계산 경로를 분리한다."""
    result = dict(row, error_mm=None)
    if not row.get('eligible',False):
        return result
    archive = folder/row['archive']
    if digest(archive) != row['archive_sha256']:
        raise ValueError(f'capture archive changed: {archive}')
    with np.load(archive, allow_pickle=False) as arrays:
        data = {kind:{c:arrays[f'{kind}_{c}'] for c in CAMERAS}
                for kind in ('images','projections','transforms','camera_origins')}
        corners = arrays['gt_corners']
    truth = corners.mean(axis=0)
    stamp = Time(sec=row['stamp_ns']//10**9,nanosec=row['stamp_ns']%10**9)
    data.update(stamp=stamp,stamp_ns=row['stamp_ns'])
    headers = {c:Header(stamp=stamp,frame_id=row['frame_ids'][c]) for c in CAMERAS}
    def debug(camera, prediction, image, header):
        cv2.imwrite(str(output/f'{row["id"]}_{camera}_yolo.png'),prediction.plot())
    vision = PortVision(ReplayLogger(),parse_model_class(row['target']),model=model,debug_image_callback=debug)
    detections = vision._detect(data['images'],headers)
    estimates = vision._estimate_candidates(data,detections)
    estimate = estimates[0] if estimates else None
    result['camera_detections'] = {c:len(detections[c]) for c in CAMERAS}
    result['keypoints'] = {c:[d['keypoints'].tolist() for d in detections[c]] for c in CAMERAS}
    result['archive_sha256'] = digest(archive)
    result['status'] = 'ok' if estimate else ('fewer_than_two_detections' if sum(bool(v) for v in detections.values())<2 else 'geometry_gate_rejected')
    if estimate is not None:
        result.update(error_mm=float(np.linalg.norm(estimate.xyz-truth)*1000),
                      predicted_xyz_m=estimate.xyz.tolist(),reprojection_rms_px=estimate.reprojection_rms_px)
    gt_detections = {}
    for c in CAMERAS:
        uv = np.array([project_point(data['projections'][c],p) for p in corners])
        gt_detections[c] = [{'keypoints':uv,'uv':uv.mean(axis=0),'confidence':1.0,'class_name':row['target']}] if row['visibility'][c]['visible'] else []
    geometry = vision._estimate_candidates(data,gt_detections)
    result['geometry_error_mm'] = float(np.linalg.norm(geometry[0].xyz-truth)*1000) if geometry else None
    return result


def source_provenance(plan):
    """알려진 보고 전용 업그레이드만 허용하며 검출 함수와 나머지 소스 해시는 고정한다."""
    upgrades = {}
    for name, expected in plan['source_sha256'].items():
        actual = digest(ROOT/name)
        if actual == expected:
            continue
        legacy = (name == 'scripts/evaluate_triangulation_protocol.py' and
                  expected == '83b8fe385077d018613ffc5da777ef996b820445db84d9c9a8821ced954e3f94' and
                  hashlib.sha256(inspect.getsource(evaluate_case).rstrip().encode()).hexdigest() == '6fc2d7174298e703e71d1d03a3c583535d0e6ecb0140eacaf225c0f2c96ca932')
        if not legacy:
            raise ValueError(f'source differs from frozen plan: {name}')
        upgrades[name] = dict(frozen=expected, current=actual, reason='reporting upgrade; evaluate_case unchanged')
    return upgrades


def process(args, plan, run, provenance, evaluator=evaluate_case, model_factory=YOLO):
    """case JSON을 먼저 확정하고 중단 시 디스크 기준으로 부분 보고서를 만든다."""
    rows, jobs = load_capture(args.capture, plan, run)
    state, model = 'completed', None
    def restore(row):
        path = args.output/f'{row["id"]}.json'
        if path.is_file():
            saved = json.loads(path.read_text())
            if saved.get('input_sha256') == row['input_sha256']:
                return saved
        return row
    for row in rows:
        row['input_sha256'] = hashlib.sha256(json.dumps(row, sort_keys=True).encode()).hexdigest()
    try:
        for i, row in enumerate(rows):
            if row.get('eligible'):
                archive = args.capture/f'job_{row["job_index"]:03d}'/row['archive']
                if not archive.resolve().is_relative_to(args.capture.resolve()):
                    raise ValueError(f'archive outside capture: {archive}')
                if digest(archive) != row['archive_sha256']:
                    raise ValueError(f'capture archive changed: {archive}')
            saved = restore(row)
            if saved['evaluation_state'] != 'pending':
                rows[i] = saved
                continue
            print(f'[{i+1}/{len(rows)}] job={row["job_index"]} {row["id"]} '
                  f'distance={row["distance_m"]}m angle={row["angle_deg"]}deg light={row["light_scale"]}', flush=True)
            if row['eligible'] and model is None:
                model = model_factory(str(args.weights))
            result = evaluator(args.capture/f'job_{row["job_index"]:03d}', row, model, args.output)
            result['evaluation_state'] = 'evaluated' if row['eligible'] else 'skipped'
            write_json(args.output/f'{row["id"]}.json', result)
            rows[i] = result
    except KeyboardInterrupt:
        state = 'interrupted'
    except Exception as error:
        state = 'failed'
        provenance['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        # 저장 완료 직후의 SIGINT도 복구한다. 보고서 확정 중 두 번째 SIGINT는 무시한다.
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            rows = [restore(row) for row in rows]
            report = aggregate(plan, rows, jobs, state, provenance)
            write_report(args.output, report, rows)
            print(f'{state}: {args.output / "report.html"}', flush=True)
        finally:
            signal.signal(signal.SIGINT, previous)
    return 130 if state == 'interrupted' else 0


def main():
    """동일 계획·모델·장치·평가 코드에서만 완료 case를 재사용한다."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--weights',type=Path,default=ROOT/'ws_aic/model/best.pt')
    parser.add_argument('--device',default='cpu')
    parser.add_argument('--resume',action='store_true')
    args = parser.parse_args()
    plan = json.loads((args.capture/'plan.json').read_text())
    if digest(args.weights) != plan['model_sha256']:
        parser.error('model differs from frozen plan')
    upgrades = source_provenance(plan)
    run = json.loads((args.capture/'run.json').read_text())
    signature = dict(plan_sha256=digest(args.capture/'plan.json'), model_sha256=digest(args.weights),
                     device=args.device, torch=torch.__version__, cuda=torch.version.cuda, opencv=cv2.__version__,
                     selected_cases=run['selected_cases'],
                     evaluator_sha256=digest(Path(__file__)),
                     report_sha256=digest(ROOT/'scripts/triangulation_report.py'))
    manifest = args.output/'evaluation.json'
    if args.resume:
        if not manifest.is_file() or json.loads(manifest.read_text()) != signature:
            parser.error('--resume requires matching evaluation.json (plan/model/device/code/selection)')
    elif args.output.exists():
        parser.error('--output must be new; use --resume for an existing compatible evaluation')
    args.output.mkdir(parents=True, exist_ok=True)
    write_json(manifest, signature)
    os.environ['AIC_YOLO_DEVICE'] = args.device
    os.environ.update(AIC_YOLO_CONFIDENCE='0.8',AIC_YOLO_KEYPOINT_CONFIDENCE='0.25',
                      AIC_TRIANGULATION_REPROJECTION_MAX_PX='30',
                      AIC_FINAL_POLICY_WORKSPACE_MIN='-1.0,-1.0,-0.1',AIC_FINAL_POLICY_WORKSPACE_MAX='1.0,1.0,1.5')
    torch.set_num_threads(4)
    provenance = dict(signature, argv=sys.argv, capture_run=run, reporting_upgrades=upgrades)
    return process(args, plan, run, provenance)


if __name__ == '__main__':
    raise SystemExit(main())
