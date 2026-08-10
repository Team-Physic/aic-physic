# phy_data_collection

랜덤화된 AIC 시뮬레이션에서 `phy_policy.ros.PortOffsetCollect`를 실행해 img2pos
학습 데이터를 수집하고 품질을 평가하는 ROS 2 Python 패키지입니다. 수집 runner는
trial별 pose 수렴 후 이미지를 저장하며, 여러 headless simulator를 격리해 병렬 실행할
수 있습니다.

## 실행

명령은 Pixi workspace인 `ws_aic/src`에서 실행합니다.

```bash
cd ws_aic/src

PIXI_FROZEN=true pixi run ros2 run phy_data_collection \
  collect_portoffset_randomization_data \
  --trials 100 \
  --workers 2 \
  --samples-per-trial 40 \
  --port-types sfp,sc \
  --dataset-version img2pos-v1 \
  --push-to-hub false \
  --record-rosbag false \
  --seed 42
```

처음 실행하거나 source를 수정한 뒤에는 로컬 ROS package를 다시 설치합니다.

```bash
cd ws_aic/src
pixi reinstall --frozen ros-kilted-phy-policy ros-kilted-phy-data-collection
```

## 처리 흐름

```text
trial config·world 랜덤화
  → Distrobox에서 Gazebo/AIC engine 시작
  → 선택 시 rosbag 시작
  → host Pixi 환경에서 PortOffsetCollect 시작
  → lift-up → ground-truth approach → 20mm 안전거리 밖의 50/10/5/2mm tier pose 순회
  → controller가 limit 적용 후 수락한 reference pose가 멈추고 실제 TCP의 frame 간
    이동량이 연속 frame에서 허용 범위 안으로 수렴할 때까지 대기
  → 새 camera frame과 같은 시각의 plug TF 검증
  → 가시 camera의 JPEG와 XYZ label 저장
  → 전체 collection summary 생성 → 선택 시 Hugging Face에 한 번 업로드
  → policy → rosbag → simulator 순서로 종료
```

runner 내부의 policy와 rosbag은 이미 활성화된 Pixi 환경의 `ros2`를 직접 실행합니다.
따라서 자식 process에서 Pixi 환경을 다시 해석하지 않습니다.

## 주요 옵션

전체 목록은 다음 명령으로 확인합니다.

```bash
PIXI_FROZEN=true pixi run ros2 run phy_data_collection \
  collect_portoffset_randomization_data --help
```

### Trial과 출력

| 옵션 | 기본값 | 설명 |
| --- | ---: | --- |
| `--trials` | `100` | 실행할 독립 trial 수 |
| `--workers` | `1` | 동시에 실행할 격리 simulator 수 |
| `--samples-per-trial` | `40` | trial마다 저장할 capture 수 |
| `--port-types` | `sfp,sc` | 수집할 connector 종류 |
| `--port-order` | `round_robin` | connector 선택 순서; `random` 가능 |
| `--dataset-version` | 자동 생성 | `ws_aic/data/img2pos/` 아래 새 version 경로 |
| `--resume` | 꺼짐 | 기존 version에 명시적으로 이어서 수집 |
| `--val-ratio`, `--test-ratio` | `0.15`, `0.15` | trial 단위 validation/test 비율 |
| `--seed` | `30` | scenario randomization seed |
| `--headless` | 켜짐 | Gazebo GUI 비활성화; `--no-headless`로 해제 |
| `--launch-rviz` | `false` | RViz 실행 여부 |
| `--distrobox` | `aic_eval_physic` | eval container 이름 |

dataset version을 생략하면 실행 시각으로 새 이름을 만들고, 기존 version은 `--resume`
없이는 수정하지 않습니다. 따라서 Hugging Face의 기존 PoC branch도 자동으로 덮어쓰지
않습니다.

### 병렬 headless 실행

각 worker는 고유 `ROS_DOMAIN_ID`, Zenoh router TCP port, Gazebo partition을 사용합니다.
trial index는 worker에 round-robin으로 분배되며 JSONL append는 file lock으로 보호합니다.
기본 격리 값은 `ROS_DOMAIN_ID=40+worker`, Zenoh `7600+worker`입니다.

```bash
PIXI_FROZEN=true pixi run ros2 run phy_data_collection \
  collect_portoffset_randomization_data \
  --trials 100 --workers 2 --samples-per-trial 40 \
  --dataset-version img2pos-v1
```

`--workers`를 늘리면 Gazebo마다 CPU, RAM과 GPU VRAM을 별도로 사용합니다. 먼저 2개로
확인하고 자원 여유에 따라 늘리십시오. 병렬 모드에서는 GUI와 RViz가 항상 꺼집니다.
`--ros-domain-id-base`, `--zenoh-port-base`, `--worker-start-delay-s`로 격리 값을 조정할
수 있습니다.

### Pose 분포

| 옵션 | 기본값 | 설명 |
| --- | ---: | --- |
| `--dx-min-mm`, `--dx-max-mm` | `-50`, `50` | port-local X 범위 |
| `--dy-min-mm`, `--dy-max-mm` | `-50`, `50` | port-local Y 범위 |
| `--dz-min-mm`, `--dz-max-mm` | `0`, `50` | 최소 안전거리에 더하는 비관통 방향 port-local Z 범위 |
| `--sampling-tiers-mm` | `50,10,5,2` | coarse 및 근접 위치 sampling tier |
| `--sampling-tier-weights` | `1,1,1,1` | tier별 capture quota 비율 |
| `--port-roll-limit-deg` | `25` | 대칭 roll 범위 |
| `--port-pitch-limit-deg` | `25` | 대칭 pitch 범위 |
| `--port-yaw-limit-deg` | `35` | 대칭 yaw 범위 |
| `--roll-min-deg` … `--yaw-max-deg` | 미지정 | 비대칭 RPY 범위 override |
| `--rpy-norm-max-rad` | 미지정 | 생성된 RPY vector norm 상한 |
| `--base-z-offset-mm` | `20` | 모든 collect target이 유지할 접근축 최소 안전거리 |

기본 40 capture는 각 tier에 10개씩 배정됩니다. X/Y는 `±tier`, Z는 20mm 최소
안전거리에 `0~tier`를 더하고 RPY 범위도 tier 비율로 축소합니다. 첫 sample의 추가
translation과 RPY는 모두 0이지만 port와의 안전거리 20mm는 유지합니다. 물리 부하가
작은 coarse tier부터 near tier 순서로 진행하며, 각 tier 내부 값은 무작위화합니다.
`--dz-min-mm`는 음수를 허용하지 않고 `--base-z-offset-mm`는 20mm 미만을 허용하지
않습니다.

### Sample 승인

| 옵션 | 기본값 | 설명 |
| --- | ---: | --- |
| `--min-visible-cameras` | `2` | 저장에 필요한 최소 가시 camera 수 |
| `--visibility-margin-px` | `64` | 영상 경계에서 제외할 여백 |
| `--sync-tolerance-ms` | `30` | camera/controller/동적 TF 최대 시각 차이 |
| `--sync-wait-timeout-s` | `1` | 새 Observation과 capture 시각 TF 대기시간 |
| `--settle-timeout-s` | `8` | reference와 실제 TCP 움직임 수렴 제한시간 |
| `--settle-position-tolerance-mm` | `1` | 연속 controller frame 간 TCP 위치 이동 허용량 |
| `--settle-orientation-tolerance-deg` | `1` | 연속 controller frame 간 TCP 자세 이동 허용량 |
| `--settle-stable-observations` | `3` | 연속으로 통과해야 하는 서로 다른 controller frame 수 |
| `--capture-attempt-multiplier` | `2` | 각 sample의 capture 시도 상한 |

center image의 `header.stamp`를 capture 시각으로 사용합니다. 세 image와 controller가
허용 오차 안에 있고, 해당 시각의 plug TF를 조회할 수 있으며, port가 지정 개수 이상의
camera에 보이고 촬영 시점의 실제 TF를 port-local 좌표로 변환한 뒤 20mm 안전거리를
제외한 offset이 배정된 sampling tier 안에 있을 때만 저장합니다. 명령 직후 frame은
사용하지 않고 controller reference와 실제 TCP 움직임이 수렴한 이후의 다음 camera
frame을 선택합니다. 조건을 통과하지 못한 tier는 재시도하며 목표 capture 수를 채우지
못한 trial은 실패 처리합니다.

임피던스 제어에서는 외력과 유한 stiffness 때문에 reference와 실제 TCP 사이에
정상상태 tracking error가 남을 수 있습니다. 이 값은 log 진단용으로만 기록하고 동작
완료 조건으로 사용하지 않으며, 실제 TF label의 tier 검사가 최종 sample 정확도를
보장합니다.

`collection_summary.json`의 `actual_sampling_offset_box_coverage_mm`은 촬영 시점 TF를
기준으로 안전거리를 제외한 실제 port-local offset이 각 ±2/5/10/50mm box 안에 들어온
capture 수와 비율입니다. `target_xyz_m`에는 추론과 제어에 필요한 안전거리까지 포함된
`base_link` correction을 그대로 저장합니다.

port는 trial 동안 고정되므로 시작 시 한 번 snapshot하고, 움직이는 plug만 center image
시각으로 조회합니다. 학습 row에는 `capture_stamp_ns`와 승인 source 중 가장 큰
`max_sync_skew_ns`만 남기며 상세 원본 이벤트는 선택적 rosbag으로 보관합니다.

### Hugging Face

| 옵션 | 기본값 | 설명 |
| --- | ---: | --- |
| `--push-to-hub` | `false` | 전체 수집·검증 뒤 dataset 업로드 여부 |
| `--hf-repo-id` | `team-physic/aic-align` | Hugging Face dataset repository |
| `--hf-revision` | dataset version | 업로드 branch/revision |
| `--hf-private` | 꺼짐 | 새 repository를 private으로 생성 |

인증은 프로젝트 `.env`의 `HF_TOKEN` 또는 `hf auth login`으로 설정합니다. `.env`와
token은 Git에 포함하지 않습니다.

### rosbag

`--record-rosbag true`이면 각 trial을 독립 MCAP으로 저장합니다. 기본 위치는
`rosbags/portoffset/<dataset-version>/<run-id>/<trial>/`입니다. 다음을 모두 만족해야
trial이 정상 종료됩니다.

- `metadata.yaml` 존재
- message count가 0보다 큼
- 모든 MCAP 파일의 시작·종료 magic이 유효함

## 데이터셋

```text
ws_aic/data/img2pos/<version>/
├── data.yaml
├── samples.jsonl
└── images/
    ├── train/<left|center|right>/*.jpg
    ├── val/<left|center|right>/*.jpg
    └── test/<left|center|right>/*.jpg
```

한 JSONL row는 한 camera image에 대응합니다.

| 필드 | 의미 |
| --- | --- |
| `id`, `capture_id` | image와 동시 capture 식별자 |
| `trial_id`, `split` | trial 식별자와 trial 단위 train/val/test 분할 |
| `image`, `camera`, `connector` | JPEG 상대 경로와 입력 구분 |
| `target_xyz_m` | `base_link`의 `port_entrance - plug_reference` correction |
| `sampling_offset_xyz_m` | 최소 안전거리를 제외한 실제 port-local XYZ; tier 분포 감사용 |
| `sampling_tier_mm` | 해당 capture에 배정된 coarse/near tier |
| `capture_stamp_ns` | center image 촬영 ROS 시각 |
| `max_sync_skew_ns` | 승인 source들의 최대 시각 차이 |
| `settle_*` | 촬영 전 연속 controller frame 간 최종 TCP 이동량과 대기시간 |

같은 trial의 모든 camera와 capture는 같은 split에 배정됩니다. 기본 100 trial은 정확히
70 train, 15 validation, 15 test로 배정됩니다. command pose, RPY,
scenario, projection, 전체 TF는 img2pos 입력·정답이 아니므로 dataset에 중복 저장하지
않습니다.

수집 완료 시 `collection_summary.json`에 capture/trial 수, split 누수, connector·tier
분포와 실제 XYZ label 범위를 기록합니다. 모델 예측 JSONL은 다음 명령으로 평가합니다.

```bash
PIXI_FROZEN=true pixi run ros2 run phy_data_collection evaluate_img2pos \
  --dataset-dir ../data/img2pos/img2pos-v1 \
  --predictions predictions.jsonl \
  --insertion-results insertion_results.jsonl \
  --output evaluation.json
```

prediction row는 `id`, `predicted_xyz_m`를 포함합니다. 결과는 전체와 split별
prediction coverage, `XYZ MAE(mm)`, `3D error p95(mm)`, `5mm 이내 비율`을 기록합니다.
선택적인 insertion result row의 `success` 값으로 실제 closed-loop 삽입 성공률을
계산합니다.

## 현재 모듈 역할

| 모듈/함수 | 책임 |
| --- | --- |
| `main._run_trial()` | simulator·rosbag·policy 시작/종료 조율 |
| `scenario.make_trial_config()` | task board, cable, robot 초기 조건 생성 |
| `world.write_randomized_world()` | 조명과 배경 world 생성 |
| `runtime._policy_environment()` | img2pos·pose·동기화 설정 전달 |
| `PortOffsetCollect.insert_cable()` | lift-up, approach, collect 단계 실행 |
| `motion.collect()` | pose 이동과 촬영 시점 TF 조회 |
| `motion.wait_for_pose_convergence()` | controller reference와 실제 TCP 움직임 정지 판정 |
| `dataset.wait_for_observation()` | 현재 명령보다 새로운 동기화 Observation 선택 |
| `dataset.target_xyz()` | 촬영 시점의 XYZ correction 계산 |
| `dataset.save_sample()` | 가시성 검사 후 JPEG와 compact JSONL 기록 |
| `evaluation.summarize_dataset()` | trial split 누수와 실제 label 분포 감사 |
