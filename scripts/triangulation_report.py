# 삼각측량 평가의 부분·최종 집계를 생성하며 ROS나 모델을 실행하지 않는다.
# 입력은 고정 계획, 선택 job, 관측 행과 완료된 평가 행이며 JSON·CSV·PNG·HTML을 저장한다.
# 거리·위치 오차 단위는 각각 m·mm, 각도는 deg이며 누락값은 0으로 대체하지 않는다.
# 조명 반복 job과 고유 배치를 구분하고 실제 카드 생성은 포트 TF 관측 근거와 함께 표기한다.
from __future__ import annotations

import csv
import html
import io
import json
from collections import Counter
from datetime import datetime
from pathlib import Path

import numpy as np
from triangulation_protocol import write_json


def summary(rows):
    """대기 중인 적격 관측은 추론 실패나 위치 산출률 분모에 포함하지 않는다."""
    eligible = [r for r in rows if r.get('eligible', False)]
    evaluated = [r for r in eligible if r.get('evaluation_state', 'evaluated') == 'evaluated']
    errors = [r['error_mm'] for r in evaluated if r.get('error_mm') is not None]
    return dict(planned_cases=len(rows), eligible_cases=len(eligible), evaluated_eligible_cases=len(evaluated),
                pending_eligible_cases=len(eligible)-len(evaluated), estimates=len(errors),
                estimate_rate=len(errors)/len(evaluated) if evaluated else None,
                observation_failures=dict(Counter(r.get('capture_status', r['status']) for r in rows if not r.get('eligible', False))),
                detection_failures=dict(Counter(r['status'] for r in evaluated if r['status'] != 'ok')),
                median_mm=float(np.median(errors)) if errors else None,
                p95_mm=float(np.percentile(errors, 95)) if errors else None,
                max_mm=max(errors) if errors else None)


def load_capture(capture, plan, run):
    """최신 attempt만 읽어 중단 직전 저장 case를 복구한다. 오래된 재시도 결과와 섞지 않는다."""
    rows, jobs = [], []
    layouts = {l['id']: l for l in plan['layouts']}
    for key, expected in run['selected_cases'].items():
        folder = capture/f'job_{int(key):03d}'
        attempts = sorted(p for p in folder.glob('attempt_*') if p.is_dir())
        recorded = {}
        if attempts:
            # complete.json이 이전 attempt를 가리킬 수 있으므로 최신 원본 행이 우선이다.
            latest = attempts[-1]
            for case in expected:
                path = latest/f'{case["id"]}.json'
                if path.is_file():
                    row = json.loads(path.read_text())
                    if row.get('archive'):
                        archive = Path(row['archive'])
                        if archive.is_absolute() or '..' in archive.parts:
                            raise ValueError(f'unsafe archive path: {archive}')
                        row['archive'] = str(Path(latest.name)/archive)
                    recorded[case['id']] = row
        elif (folder/'complete.json').is_file():
            recorded = {r['id']: r for r in json.loads((folder/'complete.json').read_text())['cases']}
        group = []
        for case in expected:
            row = dict(case)
            row.update(recorded.get(case['id'], {'status':'not_recorded' if attempts else 'not_attempted'}))
            row.update(job_index=int(key), capture_status=row['status'], evaluation_state='pending', error_mm=None)
            row['eligible'] = bool(row.get('eligible', False))
            group.append(row)
        rows.extend(group)
        layout = layouts[expected[0]['layout_id']]
        # 포트 TF 확인은 보드 생성 근거다. 개별 카드 entity 개수의 독립 검증은 아니다.
        observed = any('gt_base_T_port' in r or r.get('archive') for r in group)
        jobs.append(dict(job_index=int(key), layout_id=layout['id'], card_count=layout['card_count'],
                         target=layout['target'], active_rails=layout['active_rails'],
                         card_mask=format(sum(1 << r for r in layout['active_rails']), '05b'),
                         light_scale=expected[0]['light_scale'], attempted=bool(attempts or recorded),
                         board_observed=bool(observed), attempts=len(attempts),
                         recorded_cases=len(recorded), captured_cases=sum(r['capture_status']=='captured' for r in group)))
    if not rows:
        raise ValueError('capture run has no selected cases')
    return rows, jobs


def aggregate(plan, rows, jobs, state, provenance):
    """전체 계획·선택 범위·실제 보드 관측·평가 완료를 별도 분모로 집계한다."""
    overall = summary(rows)
    grouped = {key: {str(v):summary([r for r in rows if r.get(key)==v]) for v in values}
               for key, values in [('card_count', range(1,6)), ('distance_m', plan['distances_m']),
                                   ('angle_deg', plan['angles_deg']), ('light_scale', plan['light_scales']),
                                   ('target', [f'SFP_{r}{p}' for r in range(5) for p in range(2)])]}
    combinations = []
    for mask in range(1,32):
        bits = format(mask, '05b')
        selected = [j for j in jobs if j['card_mask']==bits]
        observed = [j for j in selected if j['board_observed']]
        combinations.append(dict(card_mask=bits, active_rails=','.join(str(r) for r in range(5) if mask & (1<<r)),
                                 card_count=mask.bit_count(), planned_unique_layouts=sum(
                                     sum(1<<r for r in l['active_rails'])==mask for l in plan['layouts']),
                                 selected_jobs=len(selected), selected_unique_layouts=len({j['layout_id'] for j in selected}),
                                 attempted_jobs=sum(j['attempted'] for j in selected),
                                 board_observed_jobs=len(observed), observed_unique_layouts=len({j['layout_id'] for j in observed}),
                                 captured_cases=sum(j['captured_cases'] for j in selected)))
    parameters = []
    def flatten(value, path=''):
        if isinstance(value, dict):
            for k,v in value.items():
                yield from flatten(v, f'{path}.{k}' if path else k)
        elif isinstance(value, (int,float)) and not isinstance(value,bool):
            yield path, value
    for label, ids in [('planned', {l['id'] for l in plan['layouts']}),
                       ('selected', {j['layout_id'] for j in jobs}),
                       ('board_observed', {j['layout_id'] for j in jobs if j['board_observed']})]:
        values = {}
        for layout in plan['layouts']:
            if layout['id'] not in ids:
                continue
            trial = next(iter(layout['config']['trials'].values()))
            for name, value in flatten(dict(scene=trial['scene'], robot=layout['config']['robot'])):
                values.setdefault(name, []).append(value)
        parameters.extend(dict(scope=label, parameter=k, minimum=min(v), maximum=max(v), samples=len(v)) for k,v in values.items())
    geometry_errors = [r['geometry_error_mm'] for r in rows if r.get('evaluation_state')=='evaluated' and r.get('geometry_error_mm') is not None]
    counts = dict(full_plan_layouts=len(plan['layouts']), full_plan_jobs=len(plan['layouts'])*len(plan['light_scales']),
                  full_plan_cases=len(plan['layouts'])*len(plan['light_scales'])*len(plan['distances_m'])*len(plan['angles_deg']),
                  selected_jobs=len(jobs), selected_layouts=len({j['layout_id'] for j in jobs}),
                  attempted_jobs=sum(j['attempted'] for j in jobs),
                  board_observed_jobs=sum(j['board_observed'] for j in jobs),
                  observed_unique_layouts=len({j['layout_id'] for j in jobs if j['board_observed']}),
                  captured_cases=sum(r['capture_status']=='captured' for r in rows),
                  processed_cases=sum(r['evaluation_state']!='pending' for r in rows), selected_cases=len(rows))
    return dict(schema=1, state=state, generated_at=datetime.now().astimezone().isoformat(),
                coverage_complete=counts['captured_cases']==len(rows), counts=counts, overall=overall,
                by_card_count=grouped.pop('card_count'), by_condition=grouped,
                geometry=dict(estimates=len(geometry_errors), median_mm=float(np.median(geometry_errors)) if geometry_errors else None,
                              p95_mm=float(np.percentile(geometry_errors,95)) if geometry_errors else None,
                              max_mm=max(geometry_errors) if geometry_errors else None),
                randomization=dict(seed=plan['seed'], distances_m=plan['distances_m'], angles_deg=plan['angles_deg'],
                                   light_scales=plan['light_scales'], configured_limits=plan['layouts'][0]['config']['task_board_limits'],
                                   sampled_ranges=parameters), combinations=combinations, jobs=jobs, provenance=provenance)


def write_report(output, report, rows):
    """집계 JSON을 먼저 원자적으로 저장하고 같은 스냅샷에서 CSV·PNG·HTML을 만든다."""
    write_json(output/'summary.json', report)
    def table_file(name, records):
        keys = list(records[0]) if records else ['empty']
        stream = io.StringIO()
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(records)
        temporary = output/(name+'.tmp')
        temporary.write_text(stream.getvalue(), encoding='utf-8-sig')
        temporary.replace(output/name)
    table_file('combinations.csv', report['combinations'])
    table_file('jobs.csv', report['jobs'])
    table_file('ranges.csv', report['randomization']['sampled_ranges'])
    case_keys = ['id','job_index','card_count','target','distance_m','angle_deg','light_scale','capture_status',
                 'evaluation_state','status','error_mm','geometry_error_mm']
    table_file('cases.csv', [{k:r.get(k) for k in case_keys} for r in rows])
    group_rows = [dict(group='overall', value='all', **report['overall'])]
    for key, groups in dict(card_count=report['by_card_count'], **report['by_condition']).items():
        group_rows += [dict(group=key, value=value, **metrics) for value,metrics in groups.items()]
    flat = [{k:json.dumps(v) if isinstance(v,dict) else v for k,v in r.items()} for r in group_rows]
    table_file('metrics.csv', flat)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    counts = report['counts']
    fig, axes = plt.subplots(2,2, figsize=(14,9), layout='constrained')
    fig.suptitle(f'Triangulation — {report["state"].upper()} | {counts["processed_cases"]}/{counts["selected_cases"]} processed')
    colors = ['#bfc0c0','#4f5d75','#eb6c36']
    ax = axes[0,0]
    labels = ['Selected','Captured','Eligible','Evaluated','Estimates']
    vals = [counts['selected_cases'],counts['captured_cases'],report['overall']['eligible_cases'],
            report['overall']['evaluated_eligible_cases'],report['overall']['estimates']]
    bars=ax.bar(labels, vals, color=colors[1]); ax.bar_label(bars); ax.set_title('Case coverage'); ax.set_ylabel('Cases');ax.margins(y=.2)
    ax=axes[0,1]; x=np.arange(5)
    for i,(field,label) in enumerate([('selected_jobs','Selected'),('attempted_jobs','Attempted'),('board_observed_jobs','Board observed')]):
        values=[sum(c[field] for c in report['combinations'] if c['card_count']==n) for n in range(1,6)]
        ax.bar(x+(i-1)*.25,values,.25,label=label,color=colors[i])
    ax.set_xticks(x,range(1,6));ax.set_xlabel('NIC cards');ax.set_ylabel('Jobs');ax.set_title('Card counts (jobs, including lighting repeats)');ax.legend()
    for ax,key,title in [(axes[1,0],'error_mm','YOLO + triangulation'),(axes[1,1],'geometry_error_mm','GT-corner calculation only')]:
        values=sorted(r[key] for r in rows if r.get('evaluation_state')=='evaluated' and r.get(key) is not None)
        if values:
            ax.step(values,np.arange(1,len(values)+1)/len(values),where='post',color=colors[2])
            ax.plot(values,np.arange(1,len(values)+1)/len(values),'.',color=colors[1])
        else:
            ax.text(.5,.5,'No estimates — N/A',ha='center',transform=ax.transAxes)
        ax.set_title(f'{title} (n={len(values)})');ax.set_xlabel('3D error (mm)');ax.set_ylabel('Cumulative fraction');ax.set_ylim(0,1.05)
    for ax in axes.flat:
        ax.spines[['top','right']].set_visible(False)
    png=output/'overview.png'; fig.savefig(output/'overview.tmp.png',dpi=150);plt.close(fig);(output/'overview.tmp.png').replace(png)
    def table(records):
        if not records: return '<p>자료 없음</p>'
        keys=list(records[0]); fmt=lambda v: 'N/A' if v is None else (f'{v:.6g}' if isinstance(v,float) else str(v))
        return '<div class="table"><table><thead><tr>'+''.join(f'<th>{html.escape(k)}</th>' for k in keys)+'</tr></thead><tbody>'+''.join(
            '<tr>'+''.join(f'<td>{html.escape(fmt(r.get(k)))}</td>' for k in keys)+'</tr>' for r in records)+'</tbody></table></div>'
    import base64
    encoded=base64.b64encode(png.read_bytes()).decode()
    content=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Triangulation report</title><style>body{{font-family:system-ui,sans-serif;margin:32px auto;padding:0 20px;max-width:1200px;color:#2d3142;background:#f5f5f5}}h1{{font-size:30px}}.table{{overflow:auto;margin:18px 0}}table{{border-collapse:collapse;background:white;font-size:13px}}th,td{{padding:9px;border-bottom:1px solid #ddd;text-align:right;white-space:nowrap}}th{{background:#ececec}}img{{width:100%}}.status{{border-left:5px solid #eb6c36;padding:16px;background:white}}</style>
<h1>Triangulation 평가 보고서</h1><p class="status">평가 상태: {html.escape(report['state'])} · 처리 {counts['processed_cases']}/{counts['selected_cases']} · 촬영 {counts['captured_cases']}/{counts['selected_cases']}</p>
<p>interrupted/failed는 부분 보고서입니다. completed는 선택된 입력 처리 완료이며 전체 2,700조건 성공을 의미하지 않습니다.</p>
<p>위치 산출률 = estimates / evaluated_eligible_cases. 대기 중인 적격 관측은 실패로 세지 않습니다. 오차 통계는 산출 성공 사례만 포함합니다. N/A는 0이 아닙니다.</p>
<img alt="관측 수, 카드 수별 job 분포, YOLO 및 계산 오차 누적 분포" src="data:image/png;base64,{encoded}">
<h2>범위와 진행률</h2>{table([counts])}<h2>전체 정확도</h2>{table([report['overall']])}<h2>계산 자체 오차</h2>{table([report['geometry']])}
<h2>카드 조합</h2><p>mask의 오른쪽 비트가 rail 0입니다. selected는 계획, attempted는 실행 시도, board_observed는 포트 TF/영상 근거가 있는 job입니다. 개별 카드 entity 수를 독립 검증한 수치는 아닙니다. unique_layouts는 조명 반복을 중복 계산하지 않습니다.</p>{table(report['combinations'])}
<h2>조건별 정확도</h2>{table(flat)}<h2>랜덤화 범위</h2><p>seed={report['randomization']['seed']}. 아래 값은 고정 config의 표본 최솟값·최댓값입니다. board_observed 범위도 해당 배치의 설정값이며 실측 위치 범위가 아닙니다. 위치·translation은 m, config의 회전·관절은 rad; 관측 angle_deg는 degree입니다.</p>{table(report['randomization']['sampled_ranges'])}
<h3>설정상 허용 범위</h3><pre>{html.escape(json.dumps(report['randomization']['configured_limits'],ensure_ascii=False,indent=2))}</pre>
<h2>Job 상세</h2>{table(report['jobs'])}<h2>출처</h2><pre>{html.escape(json.dumps(report['provenance'],ensure_ascii=False,indent=2))}</pre></html>'''
    temp=output/'report.html.tmp';temp.write_text(content);temp.replace(output/'report.html')
