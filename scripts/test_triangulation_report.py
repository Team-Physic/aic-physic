#!/usr/bin/env python3
# 부분 보고서의 분모, case 단위 저장, 중단과 재개를 실제 프로토콜 입력으로 검증한다.
# 모델 추론만 대체하여 GPU 없이 SIGINT와 저장 경계를 재현한다.
# 임시 산출물은 전용 TemporaryDirectory 내부에만 만들고 종료 시 정리한다.
# 위치 오차 fixture 단위는 mm이며 실제 정확도 측정값이 아니다.
# 인자·환경변수: 사용자 설정 없이 저장소 Python으로 직접 실행한다.
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from types import SimpleNamespace
from unittest.mock import patch

from evaluate_triangulation_protocol import process, source_provenance
from triangulation_protocol import ROOT, write_json
from triangulation_report import aggregate, load_capture, summary


def check():
    """실제 신호로 중단하고 저장된 case를 재사용하며 변경된 입력만 재평가한다."""
    capture = ROOT/'evidence/triangulation/logging-check-v2'
    plan = json.loads((capture/'plan.json').read_text())
    run = json.loads((capture/'run.json').read_text())
    assert source_provenance(plan)
    invalid=copy.deepcopy(plan)
    invalid['source_sha256']['scripts/evaluate_triangulation_protocol.py']='0'*64
    try:
        source_provenance(invalid)
        raise AssertionError('unknown code accepted')
    except ValueError:
        pass
    metrics = summary([dict(status='ok', eligible=True, error_mm=2, evaluation_state='evaluated'),
                       dict(status='captured', eligible=True, evaluation_state='pending')])
    assert metrics['estimate_rate'] == 1 and metrics['pending_eligible_cases'] == 1
    assert not metrics['detection_failures']
    with tempfile.TemporaryDirectory(prefix='tri-report-') as tmp:
        root = Path(tmp)
        rows, jobs = load_capture(capture, plan, run)
        initial=aggregate(plan, rows, jobs, 'interrupted', {})
        assert initial['counts']['selected_jobs']==1
        assert sum(c['selected_jobs'] for c in initial['combinations'])==1
        assert sum(c['planned_unique_layouts'] for c in initial['combinations'])==len(plan['layouts'])
        first = rows[0]
        second = dict(first, id=first['id']+'_second')
        args = SimpleNamespace(capture=capture, output=root, weights='unused')
        calls = []
        def evaluate(folder, row, model, output):
            calls.append(row['id'])
            if len(calls) == 2:
                os.kill(os.getpid(), signal.SIGINT)
            return dict(row, status='ok', error_mm=2.0, geometry_error_mm=0.001)
        def snapshot(*_):
            return copy.deepcopy(([first, second], jobs))
        with patch('evaluate_triangulation_protocol.load_capture', side_effect=snapshot):
            assert process(args, plan, run, {}, evaluate, lambda _: None) == 130
            report = json.loads((root/'summary.json').read_text())
            assert report['state']=='interrupted' and report['counts']['processed_cases']==1
            assert report['overall']['pending_eligible_cases']==1
            assert report['overall']['estimate_rate']==1
            for name in ['report.html','overview.png','cases.csv','combinations.csv','ranges.csv','jobs.csv','metrics.csv']:
                assert (root/name).stat().st_size > 0
            assert process(args, plan, run, {}, evaluate, lambda _: None)==0
            assert len(calls)==3  # 완료된 첫 case는 다시 추론하지 않는다.
            assert process(args, plan, run, {}, evaluate, lambda _: None)==0
            assert len(calls)==3
            first['stamp_ns'] += 1
            assert process(args, plan, run, {}, evaluate, lambda _: None)==0
            assert len(calls)==4
        # 추론 시작 전 중단도 0개 완료 보고서를 남긴다.
        early = root/'early'; early.mkdir(); args.output=early
        def interrupted(_):
            raise KeyboardInterrupt
        assert process(args, plan, run, {}, model_factory=interrupted)==130
        report=json.loads((early/'summary.json').read_text())
        assert report['counts']['processed_cases']==0 and report['overall']['median_mm'] is None
        # 처리 예외도 실패 보고서를 남기고 오류를 숨기지 않는다.
        failure=root/'failure'; failure.mkdir(); args.output=failure
        def broken(_):
            raise RuntimeError('injected inference failure')
        try:
            process(args, plan, run, {}, model_factory=broken)
            raise AssertionError('failure swallowed')
        except RuntimeError:
            pass
        assert json.loads((failure/'summary.json').read_text())['state']=='failed'
        invalid_resume=subprocess.run([sys.executable, str(ROOT/'scripts/evaluate_triangulation_protocol.py'),
            '--capture',str(capture),'--output',str(failure),'--resume'],capture_output=True,text=True)
        assert invalid_resume.returncode==2 and 'matching evaluation.json' in invalid_resume.stderr
        # 최신 attempt의 미완료 결과를 이전 complete.json으로 대체하지 않는다.
        partial=root/'capture'; folder=partial/'job_268'; attempt=folder/'attempt_001'; attempt.mkdir(parents=True)
        write_json(folder/'complete.json', {'cases':[dict(first, status='captured')]})
        row=dict(first); row.pop('archive',None); row.update(status='visibility_failed',eligible=False)
        write_json(attempt/f'{first["id"]}.json',row)
        loaded, _=load_capture(partial,plan,run)
        assert loaded[0]['status']=='visibility_failed' and not loaded[0]['eligible']
    print('PASS: SIGINT, partial/final reports, resume, changed input, early interrupt, latest attempt, denominator')


if __name__ == '__main__':
    check()
