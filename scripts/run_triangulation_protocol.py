#!/usr/bin/env python3
# 균등 랜덤 장면을 plan.json에 동결하고 기존 Gazebo lifecycle로 실행한다.
# plan은 100개 배치와 조명 world 3개를 저장하며 run은 RNG를 다시 사용하지 않는다.
# 소스·모델 해시, 실행 환경, 예정 조건의 성공·실패와 원본 입력을 보존한다.
# software smoke와 외부 GPU 실행은 같은 world를 사용하며 renderer를 구분해 기록한다.
# 인자: plan --output 필수 새 경로, --seed 기본 20260909, --weights 기본 ws_aic/model/best.pt.
# plan 기본값: --distances-m 0.35 0.45 0.55(m), --light-scales 0.7 1 1.3, --margin-px 20(px).
# run --plan/--output 필수, --renderer gpu/software 기본 gpu, --smoke는 1장·SFP_00·중간 거리·0도·기준 조명.
# run --visualize는 GPU renderer의 단일 job에서 Gazebo GUI와 RViz를 켜며 host DISPLAY가 필요하다.
# plan.log/protocol.log는 job별 layout·light·거리·각도와 실행 결과를 JSON Lines로 보존한다.
# run --job-indices는 0~299 선택, --resume은 같은 계획과 renderer의 종료된 job을 건너뛴다.
# 환경: ROS/Pixi·Distrobox 필요. 상속 AIC_* 제거·Zenoh 고정. ROS_HOME·Xvfb는 launcher별 격리.
from __future__ import annotations

import argparse
from collections import Counter
import copy
import json
import math
import os
import platform
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

from triangulation_protocol import ROOT, PREFIX, make_plan, cases, digest, write_json, job_complete, configure_environment, append_protocol_log

os.environ['PATH'] = str(PREFIX/'bin') + os.pathsep + os.environ['PATH']
os.environ.setdefault('AMENT_PREFIX_PATH', str(PREFIX))


def source_hashes():
    """수집·추론·asset·의존성 lock 입력과 신규 평가 코드를 해시한다."""
    names = subprocess.check_output(['git','ls-files','ws_aic/src','scripts'], cwd=ROOT, text=True).splitlines()
    names += [str(p.relative_to(ROOT)) for p in (ROOT/'scripts').glob('*triangulation*.py')]
    names += ['scripts/TriangulationCollect.py']
    return {name: digest(ROOT/name) for name in sorted(set(names)) if (ROOT/name).is_file()}


def installed_hashes(container):
    """image ID로 알 수 없는 설치 파일 변경도 검사한다. 실행 중 생성되는 pyc만 제외한다."""
    code = '''import hashlib, json
from pathlib import Path
root = Path('/ws_aic/install')
hashes = {}
for path in sorted(root.rglob('*')):
    if path.is_file() and '__pycache__' not in path.parts and path.suffix != '.pyc':
        with path.open('rb') as stream:
            hashes[str(path.relative_to(root))] = hashlib.file_digest(stream, 'sha256').hexdigest()
if not hashes:
    raise RuntimeError('simulator installation missing')
print(json.dumps(hashes, sort_keys=True))
'''
    return json.loads(subprocess.check_output(['docker','exec',container,'python3','-c',code],text=True))


def freeze_plan(args):
    """전체 랜덤 값을 새 경로에 저장한다. 실행 시 같은 seed를 다시 추출하지 않는다."""
    if args.output.exists():
        raise ValueError('plan output must be a new directory')
    plan = make_plan(args.seed, args.distances_m, args.light_scales, args.margin_px, args.weights)
    plan['source_sha256'] = source_hashes()
    plan['git_commit'] = subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip()
    image = subprocess.check_output(['docker','inspect','--format','{{.Image}}',args.distrobox],text=True).strip()
    plan['engine_image_id'] = image
    plan['engine_repo_digests'] = json.loads(subprocess.check_output(['docker','inspect','--format','{{json .RepoDigests}}',image],text=True))
    plan['installed_sha256'] = installed_hashes(args.distrobox)
    write_json(args.output/'plan.json', plan)
    log_path = args.output/'plan.log'
    append_protocol_log(log_path, 'PLAN_FROZEN', plan=str((args.output/'plan.json').resolve()),
                        layout_count=len(plan['layouts']), job_count=len(plan['layouts']) * 3,
                        case_count=len(plan['layouts']) * 3 * 9,
                        distances_m=plan['distances_m'], angles_deg=plan['angles_deg'],
                        light_scales=plan['light_scales'])
    jobs = [(layout, light) for layout in plan['layouts'] for light in range(3)]
    for job_index, (layout, light_index) in enumerate(jobs):
        append_protocol_log(
            log_path,
            'JOB_PLAN',
            job_index=job_index,
            layout_id=layout['id'],
            target=layout['target'],
            card_count=layout['card_count'],
            active_rails=layout['active_rails'],
            light_index=light_index,
            light_scale=plan['light_scales'][light_index],
            cases=cases(plan, layout['id'], light_index),
        )
    print(f'Frozen: 100 layouts, 300 simulator jobs, 2700 observations: {args.output / "plan.json"}')


def launcher_environment(command, renderer):
    """trial별 ROS lock과 Xvfb를 격리한다. GPU 실행에는 software 전용 설정을 넘기지 않는다."""
    if command[0] != 'distrobox':
        return
    launcher = Path(command[-1])
    contents = launcher.read_text()
    anchor = '. /ws_aic/install/setup.bash'
    if anchor not in contents:
        raise ValueError('unsupported engine setup in launcher')
    lines = [f'export ROS_HOME={shlex.quote(str(launcher.parent/"ros_home"))}', 'mkdir -p "$ROS_HOME"']
    if renderer == 'software':
        lines += ['export LIBGL_ALWAYS_SOFTWARE=1',
            'export __EGL_VENDOR_LIBRARY_FILENAMES="$ROS_HOME/no-egl.json"', 'export LP_NUM_THREADS=4',
            'Xvfb -displayfd 3 -screen 0 1280x1024x24 -nolisten tcp 3>"$ROS_HOME/display" >"$ROS_HOME/xvfb.log" 2>&1 &',
            'xvfb_pid=$!', 'for attempt in {1..50}; do [ -s "$ROS_HOME/display" ] && break; sleep 0.1; done',
            '[ -s "$ROS_HOME/display" ] || { kill "$xvfb_pid"; exit 1; }',
            'export DISPLAY=$(cat "$ROS_HOME/display" | sed "s/^/:/")']
        contents = contents.replace('trap cleanup_router EXIT',
            'trap \'cleanup_router; kill "$xvfb_pid" 2>/dev/null || true; wait "$xvfb_pid" 2>/dev/null || true\' EXIT')
    else:
        lines += ['unset LIBGL_ALWAYS_SOFTWARE __EGL_VENDOR_LIBRARY_FILENAMES']
    launcher.write_text(contents.replace(anchor, anchor+'\n'+'\n'.join(lines), 1))


def run_plan(args):
    """불변 계획을 job별로 실행하고 종료하지 못한 조건도 결과 집계에 포함한다."""
    from phy_data_collection.runner import main as runner, runtime
    from phy_data_collection.runner.cli import build_parser
    if not args.plan.is_file():
        raise ValueError(f'--plan must be an existing JSON file: {args.plan}')
    print('[preflight] checking plan, container installation and source hashes', flush=True)
    plan = json.loads(args.plan.read_text())
    if plan.get('schema') != 2 or len(plan['layouts']) != 100:
        raise ValueError('unsupported/incomplete plan')
    image = subprocess.check_output(['docker','inspect','--format','{{.Image}}',args.distrobox],text=True).strip()
    if image != plan['engine_image_id']:
        raise ValueError('simulator image differs from frozen plan')
    if installed_hashes(args.distrobox) != plan['installed_sha256']:
        raise ValueError('actual /ws_aic/install files differ from frozen plan')
    gpu = None
    if args.renderer == 'gpu':
        gpu = subprocess.check_output(['docker','exec',args.distrobox,'nvidia-smi',
            '--query-gpu=name,driver_version,memory.total','--format=csv,noheader'],text=True).strip()
    for name, expected in plan['source_sha256'].items():
        if not (ROOT/name).is_file() or digest(ROOT/name) != expected:
            raise ValueError(f'source changed: {name}; generate a new plan version')
    signature = {'plan_sha256': digest(args.plan), 'renderer': args.renderer, 'visualize': args.visualize,
                 'smoke': args.smoke, 'timeout_s': args.timeout_s}
    jobs = [(layout, light) for layout in plan['layouts'] for light in range(3)]
    if args.smoke and args.job_indices:
        raise ValueError('--smoke and --job-indices are mutually exclusive')
    indices = args.job_indices if args.job_indices is not None else list(range(len(jobs)))
    if args.smoke:
        indices = [next(i for i,(l,light) in enumerate(jobs) if l['card_count']==1 and l['target']=='SFP_00' and l['repeat']==0 and light==1)]
    if not indices or len(set(indices)) != len(indices) or any(i < 0 or i >= len(jobs) for i in indices):
        raise ValueError('job indices must be unique integers in 0..299')
    if args.visualize:
        if args.renderer != 'gpu':
            raise ValueError('--visualize requires --renderer gpu')
        if len(indices) != 1:
            raise ValueError('--visualize requires exactly one selected job; pass --job-indices N')
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            raise ValueError('--visualize requires DISPLAY or WAYLAND_DISPLAY in the host shell')
    selections = {}
    for index in indices:
        layout, li = jobs[index]
        selected = cases(plan,layout['id'],li)
        if args.smoke:
            selected = [c for c in selected if c['distance_m']==plan['distances_m'][1] and c['angle_deg']==0]
        selections[str(index)] = [dict(c,target=layout['target'],card_count=layout['card_count']) for c in selected]
    if args.output.exists():
        if not (args.output/'run.json').is_file():
            raise ValueError('output exists without run.json; use a new output directory')
        previous = json.loads((args.output/'run.json').read_text())
        if not args.resume or previous['signature'] != signature or previous['selected_cases'] != selections:
            raise ValueError('resume requires identical plan, renderer, mode and selected jobs')
        log_path = args.output/'protocol.log'
        append_protocol_log(log_path, 'RUN_RESUMED', plan=str(args.plan.resolve()),
                            renderer=args.renderer, visualize=args.visualize,
                            selected_job_indices=indices)
    else:
        args.output.mkdir(parents=True)
        shutil.copy2(args.plan,args.output/'plan.json')
        write_json(args.output/'run.json',{'signature':signature,'selected_cases':selections,
            'argv':sys.argv,'python':sys.version,'platform':platform.platform(),'gpu':gpu,'engine_image_id':image})
        log_path = args.output/'protocol.log'
        append_protocol_log(log_path, 'RUN_STARTED', plan=str(args.plan.resolve()),
                            renderer=args.renderer, visualize=args.visualize,
                            selected_job_indices=indices)
    base_start = runtime._start_logged_process
    def start(command, **kwargs):
        launcher_environment(command, args.renderer)
        env = kwargs.get('env')
        if env is not None:
            env.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp', AIC_REID_ENCODER='none', AIC_IMG2POS_ANNOTATION_DEPTH_MAX_SKEW_SEC='0',
                       AIC_IMG2POS_ANNOTATION_DEPTH_TIMEOUT_SEC='0.1',
                       AIC_COLLECT_HAPTIC_BASELINE_TIMEOUT_S=str(args.timeout_s))
        return base_start(command, **kwargs)
    runtime._start_logged_process = start
    os.environ['PYTHONPATH'] = os.pathsep.join([str(ROOT/'scripts'),
        str(ROOT/'ws_aic/src/phy/phy_policy'), os.environ.get('PYTHONPATH','')])
    all_rows, failed = [], False
    for index in indices:
        layout, li = jobs[index]
        job_root = args.output/f'job_{index:03d}'
        selected = selections[str(index)]
        if args.resume and (job_root/'complete.json').exists():
            done = json.loads((job_root/'complete.json').read_text())
            if job_complete(done['runner_returncode'],done['cases'],[c['id'] for c in selected]):
                append_protocol_log(log_path, 'JOB_SKIPPED', job_index=index,
                                    layout_id=layout['id'], target=layout['target'],
                                    card_count=layout['card_count'], light_index=li,
                                    light_scale=plan['light_scales'][li], case_count=len(selected),
                                    reason='resume_complete')
                all_rows.extend(done['cases'])
                continue
        # 재시도는 이전 raw 입력·실패 로그를 덮어쓰지 않고 새로운 attempt에 보존한다.
        folder = job_root/f'attempt_{len(list(job_root.glob("attempt_*"))):03d}'
        # 재시도가 이전 dataset과 충돌하지 않도록 보조 artifact도 해당 attempt 안에 보존한다.
        runtime.DATASET_ROOT = (folder/'collector_metadata').resolve()
        runtime.EPISODE_TRACKING_DIR = (folder/'episode_tracking').resolve()
        runner.COLLECTION_LOG_ROOT = (folder/'logs').resolve()
        job = dict(output=str(folder.resolve()), protocol_log=str(log_path.resolve()),
                   job_index=index, attempt=folder.name, cases=selected, target=layout['target'],
                   card_count=layout['card_count'], margin_px=plan['margin_px'],
                   depth_occlusion_tolerance_m=plan['depth_occlusion_tolerance_m'])
        write_json(folder/'job.json', job)
        os.environ['AIC_TRIANGULATION_JOB'] = str((folder/'job.json').resolve())
        render_args = (['--no-headless', '--launch-rviz', 'true'] if args.visualize else
                       ['--no-headless' if args.renderer=='software' else '--headless'])
        opts = build_parser().parse_args(['--port-type','sfp','--collection-policy','board-view',
            '--trials','300','--samples-per-trial',str(len(selected)),'--auto-annotate-ports','true',
            '--dataset-version',f'triangulation-{args.output.name}-{index:03d}',
            '--distrobox',args.distrobox,'--policy','TriangulationCollect','--policy-start-wait-s','15',
            '--settle-timeout-s',str(args.timeout_s),'--sync-wait-timeout-s',str(args.timeout_s),
            '--trial-timeout-s',str(args.timeout_s*10),'--time-limit-s','3600','--color-log','false',
            *render_args])
        opts.seed = plan['seed']
        runner._prepare_args(opts)
        opts.split_assignments = runner._split_assignments(opts)
        config = copy.deepcopy(layout['config'])
        trial = next(iter(config['trials'].values()))
        old_id = next(iter(trial['tasks']))
        task_id = f'{old_id}_light{li}'
        trial['tasks'][task_id] = trial['tasks'].pop(old_id)
        trial['tasks'][task_id]['time_limit'] = 3600
        runtime.write_inputs(config, folder/'engine_config.yaml')
        (folder/'world.sdf').write_text(plan['worlds'][li])
        description = folder/'annotation.urdf.xacro'
        description.write_text(plan['annotation_description'])
        runtime.ANNOTATION_ROBOT_DESCRIPTION_PATH = description.resolve()
        runner.make_trial_config = lambda _i,_rng,_args: (copy.deepcopy(config), {task_id: {'layout_id':layout['id'],'target':layout['target']}})
        def frozen_world(_i,_rng,_args,path):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(plan['worlds'][li])
            return path, {'enabled': False, 'reason': f'frozen light scale {plan["light_scales"][li]}'}
        runner.write_randomized_world = frozen_world
        run_id = f'triangulation-{args.output.name}-{index:03d}-{os.getpid()}'
        append_protocol_log(
            log_path,
            'JOB_STARTED',
            job_index=index,
            layout_id=layout['id'],
            target=layout['target'],
            card_count=layout['card_count'],
            active_rails=layout['active_rails'],
            light_index=li,
            light_scale=plan['light_scales'][li],
            cases=selected,
            renderer=args.renderer,
            visualize=args.visualize,
        )
        print(f'JOB {index}: {layout["id"]} {layout["target"]}, cards={layout["card_count"]}, light={plan["light_scales"][li]}', flush=True)
        try:
            result = runner._run_worker(opts, run_id, 0, [index])
        except Exception as exc:
            append_protocol_log(log_path, 'JOB_ERROR', job_index=index,
                                layout_id=layout['id'], light_index=li,
                                error=f'{type(exc).__name__}: {exc}')
            raise
        rows = []
        for case in selected:
            path = folder/f'{case["id"]}.json'
            row = json.loads(path.read_text()) if path.exists() else dict(case,status='not_recorded')
            if row.get('archive'):
                row['archive'] = f'{folder.name}/{row["archive"]}'
            rows.append(row)
        failed |= not job_complete(result,rows,[c['id'] for c in selected])
        all_rows.extend(rows)
        write_json(job_root/'complete.json', {'runner_returncode':result,'cases':rows})
        append_protocol_log(
            log_path,
            'JOB_FINISHED',
            job_index=index,
            layout_id=layout['id'],
            target=layout['target'],
            light_index=li,
            light_scale=plan['light_scales'][li],
            runner_returncode=result,
            statuses=dict(Counter(str(row.get('status', 'missing')) for row in rows)),
            cases=[{'id': row.get('id'), 'distance_m': row.get('distance_m'),
                    'angle_deg': row.get('angle_deg'), 'status': row.get('status')}
                   for row in rows],
        )
        log = runner._trial_log_path(opts,run_id,0,index)
        if log.exists():
            shutil.copy2(log,folder/'runner.log')
        stage = runner.CONFIG_DIR/f'{run_id}-w00-{os.getpid()}'
        if stage.exists():
            shutil.copytree(stage,folder/'launch_evidence',dirs_exist_ok=True,ignore=shutil.ignore_patterns('ros_home'))
            registry = stage/runner.REGISTRY_FILENAME
            groups = json.loads(registry.read_text()).get('groups',[]) if registry.exists() else []
            if not groups:
                shutil.rmtree(stage)
        if result == 130:
            break
    recorded_ids = {r['id'] for r in all_rows}
    all_rows.extend(dict(c,status='not_attempted') for group in selections.values() for c in group if c['id'] not in recorded_ids)
    write_json(args.output/'capture_summary.json', {'selected_job_indices':indices,
        'planned_case_count':len(all_rows),
        'recorded_case_count':sum(r['status'] not in ('not_recorded','not_attempted') for r in all_rows),
        'statuses':dict(Counter(r['status'] for r in all_rows)),
        'eligible_captures':sum(r['status']=='captured' for r in all_rows),'cases':all_rows})
    failed = bool(failed or (args.smoke and not any(r['status']=='captured' for r in all_rows)))
    append_protocol_log(log_path, 'RUN_FINISHED', selected_job_indices=indices,
                        planned_case_count=len(all_rows),
                        recorded_case_count=sum(r['status'] not in ('not_recorded','not_attempted') for r in all_rows),
                        statuses=dict(Counter(r['status'] for r in all_rows)),
                        failed=bool(failed))
    return int(failed)


def main():
    """계획 생성과 실행을 분리한다. 재실행은 저장한 plan.json을 필수로 받는다."""
    configure_environment()
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='command',required=True)
    plan = sub.add_parser('plan')
    plan.add_argument('--output',type=Path,required=True)
    plan.add_argument('--seed',type=int,default=20260909)
    plan.add_argument('--weights',type=Path,default=ROOT/'ws_aic/model/best.pt')
    plan.add_argument('--distances-m',type=float,nargs=3,default=[.35,.45,.55])
    plan.add_argument('--light-scales',type=float,nargs=3,default=[.7,1,1.3])
    plan.add_argument('--margin-px',type=float,default=20)
    plan.add_argument('--distrobox',default='aic_eval_physic')
    run = sub.add_parser('run')
    run.add_argument('--plan',type=Path,required=True)
    run.add_argument('--output',type=Path,required=True)
    run.add_argument('--renderer',choices=['gpu','software'],default='gpu')
    run.add_argument('--visualize',action='store_true',
                     help='single GPU job with visible Gazebo GUI and RViz')
    run.add_argument('--distrobox',default='aic_eval_physic')
    run.add_argument('--timeout-s',type=float,default=120)
    run.add_argument('--smoke',action='store_true')
    run.add_argument('--job-indices',type=int,nargs='+')
    run.add_argument('--resume',action='store_true')
    args = parser.parse_args()
    if args.command == 'plan':
        freeze_plan(args)
        return 0
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0:
        parser.error('--timeout-s must be positive and finite')
    return run_plan(args)


if __name__ == '__main__':
    raise SystemExit(main())
