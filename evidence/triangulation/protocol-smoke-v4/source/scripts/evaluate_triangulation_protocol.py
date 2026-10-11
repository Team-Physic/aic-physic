#!/usr/bin/env python3
# 고정 프로토콜로 저장한 실제 영상과 카메라 행렬을 현재 PortVision으로 재평가한다.
# 가시성 조건을 충족한 모든 사례를 분모에 넣고 미검출도 실패로 보존한다.
# GT는 채점과 계산 전용 비교 경로에만 사용하며 YOLO 입력은 원본 BGR 영상이다.
# 원본 NPZ·모델 해시, 검출 bbox 영상과 mm 오차를 기록하여 GPU PC에서도 재검증한다.
# 인자: --capture/--output 필수 경로, --weights 기본 ws_aic/model/best.pt, --device 기본 cpu(GPU는 0).
from __future__ import annotations

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import sys

from triangulation_protocol import ROOT, PREFIX, CAMERAS, digest, write_json
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


def summary(rows):
    """관측 불충족과 관측 충족 후 알고리즘 실패를 서로 다른 분모로 보고한다."""
    eligible = [r for r in rows if r.get('eligible', False)]
    errors = [r['error_mm'] for r in eligible if r.get('error_mm') is not None]
    return {'planned_cases':len(rows), 'eligible_cases':len(eligible),
            'observation_failures':dict(Counter(r['status'] for r in rows if not r.get('eligible',False))),
            'estimates':len(errors), 'estimate_rate':len(errors)/len(eligible) if eligible else None,
            'detection_failures':dict(Counter(r['status'] for r in eligible if r['status']!='ok')),
            'median_mm':float(np.median(errors)) if errors else None,
            'p95_mm':float(np.percentile(errors,95)) if errors else None,
            'max_mm':max(errors) if errors else None}


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


def main():
    """소스와 모델을 확인한 뒤 원본 capture를 바꾸지 않고 새 결과 경로에 저장한다."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--weights',type=Path,default=ROOT/'ws_aic/model/best.pt')
    parser.add_argument('--device',default='cpu')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('--output must be a new directory')
    plan = json.loads((args.capture/'plan.json').read_text())
    if digest(args.weights) != plan['model_sha256']:
        parser.error('model differs from frozen plan')
    for name, expected in plan['source_sha256'].items():
        if digest(ROOT/name) != expected:
            parser.error(f'source differs from frozen plan: {name}')
    os.environ['AIC_YOLO_DEVICE'] = args.device
    os.environ.update(AIC_YOLO_CONFIDENCE='0.8',AIC_YOLO_KEYPOINT_CONFIDENCE='0.25',
                      AIC_TRIANGULATION_REPROJECTION_MAX_PX='30',
                      AIC_FINAL_POLICY_WORKSPACE_MIN='-1.0,-1.0,-0.1',AIC_FINAL_POLICY_WORKSPACE_MAX='1.0,1.0,1.5')
    torch.set_num_threads(4)
    args.output.mkdir(parents=True)
    model = YOLO(str(args.weights))
    rows = []
    run = json.loads((args.capture/'run.json').read_text())
    for index, expected in run['selected_cases'].items():
        folder = args.capture/f'job_{int(index):03d}'
        complete = folder/'complete.json'
        recorded = {r['id']:r for r in json.loads(complete.read_text())['cases']} if complete.exists() else {}
        for case in expected:
            row = recorded.get(case['id'],dict(case,status='not_recorded'))
            result = evaluate_case(folder,row,model,args.output)
            rows.append(result)
            write_json(args.output/f'{row["id"]}.json',result)
    if not rows:
        raise ValueError('no completed capture jobs')
    report = {'scope':'fixed Gazebo observations; no physical insertion; software and GPU renders may differ',
              'argv':sys.argv,'plan_sha256':digest(args.capture/'plan.json'),'model_sha256':digest(args.weights),
              'torch':torch.__version__,'cuda':torch.version.cuda,'opencv':cv2.__version__,
              'device':str(getattr(model.predictor,'device',args.device)),
              'capture_run':json.loads((args.capture/'run.json').read_text()),
              'overall':summary(rows),
              'by_card_count':{str(n):summary([r for r in rows if r.get('card_count')==n]) for n in range(1,6)},
              'geometry_max_mm':max((r['geometry_error_mm'] for r in rows if r.get('geometry_error_mm') is not None),default=None)}
    write_json(args.output/'summary.json',report)
    print(json.dumps(report['overall'],ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
