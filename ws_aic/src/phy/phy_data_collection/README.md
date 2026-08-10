# phy_data_collection

랜덤화된 AIC 시뮬레이션에서 `phy_policy.ros.PortOffsetCollect`를 실행해 img2pos
학습 데이터를 수집하는 ROS 2 Python 패키지입니다. 현재 제공하는 실행 파일은
`collect_portoffset_randomization_data` 하나입니다.

## 실행

명령은 Pixi workspace인 `ws_aic/src`에서 실행합니다.

```bash
cd ws_aic/src

PIXI_FROZEN=true pixi run ros2 run phy_data_collection \
  collect_portoffset_randomization_data \
  --trials 2 \
  --samples-per-trial 20 \
  --port-types sfp,sc \
  --dataset-version align-260810 \
  --push-to-hub false \
  --record-rosbag false \
  --cleanup \
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
  → lift-up → ground-truth approach → stratified XYZ/RPY pose 순회
  → 새 camera frame과 같은 시각의 plug TF 검증
  → 가시 camera의 JPEG와 XYZ label 저장
  → episode summary/Hugging Face 업로드
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
| `--trials` | `20` | 실행할 독립 trial 수 |
| `--samples-per-trial` | `24` | trial마다 시도할 capture 수 |
| `--port-types` | `sfp,sc` | 수집할 connector 종류 |
| `--port-order` | `round_robin` | connector 선택 순서; `random` 가능 |
| `--dataset-version` | 빈 문자열 | `ws_aic/data/img2pos/` 아래 version 경로 |
| `--seed` | `30` | scenario randomization seed |
| `--headless` | 꺼짐 | Gazebo GUI 비활성화 |
| `--launch-rviz` | `true` | RViz 실행 여부 |
| `--distrobox` | `aic_eval_physic` | eval container 이름 |

### Pose 분포

| 옵션 | 기본값 | 설명 |
| --- | ---: | --- |
| `--dx-min-mm`, `--dx-max-mm` | `-50`, `50` | port-local X 범위 |
| `--dy-min-mm`, `--dy-max-mm` | `-50`, `50` | port-local Y 범위 |
| `--dz-min-mm`, `--dz-max-mm` | `0`, `100` | port-local Z 범위 |
| `--port-roll-limit-deg` | `25` | 대칭 roll 범위 |
| `--port-pitch-limit-deg` | `25` | 대칭 pitch 범위 |
| `--port-yaw-limit-deg` | `35` | 대칭 yaw 범위 |
| `--roll-min-deg` … `--yaw-max-deg` | 미지정 | 비대칭 RPY 범위 override |
| `--rpy-norm-max-rad` | 미지정 | 생성된 RPY vector norm 상한 |
| `--base-z-offset-mm` | `0` | 모든 collect target에 더하는 접근축 거리 |

첫 sample은 항상 translation과 RPY가 모두 0입니다. 나머지는 각 축을 계층화
샘플링하고 translation 거리 순서로 실행합니다.

### Sample 승인

| 옵션 | 기본값 | 설명 |
| --- | ---: | --- |
| `--min-visible-cameras` | `2` | 저장에 필요한 최소 가시 camera 수 |
| `--visibility-margin-px` | `64` | 영상 경계에서 제외할 여백 |
| `--sync-tolerance-ms` | `30` | camera/controller/동적 TF 최대 시각 차이 |
| `--sync-wait-timeout-s` | `1` | 새 Observation과 capture 시각 TF 대기시간 |

center image의 `header.stamp`를 capture 시각으로 사용합니다. 세 image와 controller가
허용 오차 안에 있고, 해당 시각의 plug TF를 조회할 수 있으며, port가 지정 개수 이상의
camera에 보일 때만 저장합니다. 조건을 통과하지 못한 시도는 JPEG와 JSONL을 남기지
않습니다.

port는 trial 동안 고정되므로 시작 시 한 번 snapshot하고, 움직이는 plug만 center image
시각으로 조회합니다. 학습 row에는 `capture_stamp_ns`와 승인 source 중 가장 큰
`max_sync_skew_ns`만 남기며 상세 원본 이벤트는 선택적 rosbag으로 보관합니다.

### Hugging Face

| 옵션 | 기본값 | 설명 |
| --- | ---: | --- |
| `--push-to-hub` | `false` | 성공 trial 뒤 dataset 업로드 여부 |
| `--hf-repo-id` | `aic-sejong-team/aic-vision-offset-dataset` | Hugging Face dataset repository |
| `--hf-revision` | `main` | 업로드 branch/revision |
| `--upload-on-port-type` | 빈 문자열 | 지정한 `sfp` 또는 `sc` trial에서만 업로드 |
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
    └── val/<left|center|right>/*.jpg
```

한 JSONL row는 한 camera image에 대응합니다.

| 필드 | 의미 |
| --- | --- |
| `id`, `capture_id` | image와 동시 capture 식별자 |
| `trial_id`, `split` | trial 식별자와 trial 단위 train/val 분할 |
| `image`, `camera`, `connector` | JPEG 상대 경로와 입력 구분 |
| `target_xyz_m` | `base_link`의 `port_entrance - plug_reference` correction |
| `capture_stamp_ns` | center image 촬영 ROS 시각 |
| `max_sync_skew_ns` | 승인 source들의 최대 시각 차이 |

같은 trial의 모든 camera와 capture는 같은 split에 배정됩니다. command pose, RPY,
scenario, projection, 전체 TF는 img2pos 입력·정답이 아니므로 dataset에 중복 저장하지
않습니다.

## 현재 모듈 역할

| 모듈/함수 | 책임 |
| --- | --- |
| `main._run_trial()` | simulator·rosbag·policy 시작/종료 조율 |
| `scenario.make_trial_config()` | task board, cable, robot 초기 조건 생성 |
| `world.write_randomized_world()` | 조명과 배경 world 생성 |
| `runtime._policy_environment()` | img2pos·pose·동기화 설정 전달 |
| `PortOffsetCollect.insert_cable()` | lift-up, approach, collect 단계 실행 |
| `motion.collect()` | pose 이동과 촬영 시점 TF 조회 |
| `dataset.wait_for_observation()` | 현재 명령보다 새로운 동기화 Observation 선택 |
| `dataset.target_xyz()` | 촬영 시점의 XYZ correction 계산 |
| `dataset.save_sample()` | 가시성 검사 후 JPEG와 compact JSONL 기록 |
