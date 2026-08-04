# phy_data_collection

`ais_auto_capture`의 자동 수집·검증 도구를 Team Physic 작업공간용 ROS 2 Python 패키지로 포팅한 버전입니다.

## 빌드와 실행 준비

```bash
cd ws_aic
colcon build --symlink-install --packages-up-to phy_data_collection
source install/setup.bash
```

각 도구는 `ros2 run phy_data_collection <실행 파일>`로 실행합니다. PortOffset 수집 및 검증 기능은 기존 정책 패키지를 포팅한 `data_generator`를 런타임 의존성으로 사용하므로 해당 패키지도 함께 빌드해야 합니다.

## 스크립트 설명

| 파일 | 환경 | 역할 |
|---|---|---|
| `collect_portoffset_randomization_data.py` | distrobox/eval engine | Align Phase에서 사용될 vision-offset 정렬 데이터 자동 수집 |
| `validate_portoffset_trial.py` | host Pixi | 저장 sample과 동일 trial MCAP의 영상·시각·TF·label 일치 검사 |
| `collect_lerobot_data.py` | distrobox (x86) | LeRobot 에피소드 자동 수집 |
| `collect_lerobot_data_aarch.py` | 소스 빌드 (aarch64) | LeRobot 에피소드 자동 수집 |
| `collect_yolo_data_aarch.py` | 소스 빌드 (aarch64) | TF 기반 YOLO 데이터셋 자동 수집 |
| `plot_scenario_randomization.py` | host Pixi | 시나리오 랜덤화 분포 plot 생성 |

## 1. collect_portoffset_randomization_data.py — PortOffsetCollect 정렬 데이터 수집

랜덤화된 task board/cable/lighting 조건에서 `data_generator.PortOffsetCollect` policy를 실행해 vision-offset 정렬 학습 데이터를 수집한다.

명령은 `ws_aic/src`에서 실행하며, runner는 Distrobox와 host Pixi policy의 내부 작업 디렉터리를 `ws_aic/src/aic`으로 고정한다. 따라서 호출한 shell의 현재 디렉터리와 관계없이 AIC package와 asset 탐색 기준이 일치한다.

```bash
pixi run ros2 run phy_data_collection collect_portoffset_randomization_data \
  --trials 2 \
  --samples-per-trial 20 \
  --port-types sfp,sc \
  --dataset-version 0803-001 \
  --push-to-hub false \
  --vision-offset-repo-id aic-sejong-team/aic-vision-offset-dataset \
  --vision-offset-hf-revision 0726-001 \
  --upload-on-port-type sc \
  --record-rosbag true \
  --cleanup \
  --seed 42
```

### 1-1. 파라미터

#### 수집 규모 및 trial 구성

| CLI 옵션 | 기본값 | 조정 목적 |
|---|---:|---|
| `--trials` | `20 trials` | 생성할 독립 Gazebo 시나리오 수 |
| `--seed` | `30` | randomization 재현 또는 다른 표본 생성 |
| `--port-types` | `sfp,sc` | `sfp`, `sc` 또는 두 포트 계열 선택 |
| `--port-order` | `round_robin` | `--port-types`에 적힌 순서대로 포트 타입을 번갈아 배치; 복원 랜덤 선택은 `random` 사용 |
| `--samples-per-trial` | `24 samples/trial` | trial별 offset sample 시도 수 |
| `--time-limit-s` | `600 s` | 생성되는 AIC task 제한 시간 |
| `--trial-timeout-s` | 미지정(`s`) | collector 완료 대기시간 override; 미지정 시 `time-limit-s + 180 s` |

#### Task Board와 target module randomization

PortOffset 수집기의 scene 범위는 `portoffset_randomization/constants.py`의 `LIMITS`가 기준입니다. Task Board translation은 world X/Y만 랜덤화하고 Z는 고정하며, rotation은 yaw만 랜덤화하고 roll/pitch는 고정합니다.

| 대상 | Translation | Rotation | 선택 범위 |
|---|---|---|---|
| SFP Task Board | world X `0.13 ~ 0.17 m`, Y `-0.25 ~ -0.20 m`, Z `1.14 m` 고정 | roll/pitch `0 rad` 고정, yaw `3.10 ~ 3.1415 rad` | NIC rail `0~4`, SFP port `0~1` |
| SC Task Board | world X `0.15 ~ 0.19 m`, Y `-0.05 ~ 0.05 m`, Z `1.14 m` 고정 | roll/pitch `0 rad` 고정, yaw `3.10 ~ 3.1415 rad` | SC rail `0~1` |
| SFP NIC module | rail translation `-0.0215 ~ 0.0234 m` | local yaw `-10 ~ +10 deg` | 선택된 NIC rail 하나만 활성화 |
| SC port module | rail translation `-0.06 ~ 0.055 m` | local yaw `0 rad` 고정 | 선택된 SC rail 하나만 활성화 |

#### 로봇 초기 자세 randomization

| CLI 옵션 | 기본값 | 조정 목적 |
|---|---:|---|
| `--robot-joint-noise-deg` | `4 deg` | robot home joint별 uniform noise `±4 deg` |
| `--cable-rpy-noise-deg` | `20 deg` | cable 초기 roll/pitch/yaw uniform noise `±20 deg` |

#### Port-local XYZ/RPY sample 분포

| CLI 옵션 | 기본값 | 조정 목적 |
|---|---:|---|
| `--dx-min-mm`, `--dx-max-mm` | `-50 mm`, `50 mm` | port-local X translation 범위 |
| `--dy-min-mm`, `--dy-max-mm` | `-50 mm`, `50 mm` | port-local Y translation 범위 |
| `--dz-min-mm`, `--dz-max-mm` | `0 mm`, `100 mm` | port 바깥쪽 접근축 translation 범위 |
| `--port-roll-limit-deg` | `25 deg` | 대칭 roll 범위 `[-25, +25] deg` |
| `--port-pitch-limit-deg` | `25 deg` | 대칭 pitch 범위 `[-25, +25] deg` |
| `--port-yaw-limit-deg` | `35 deg` | 대칭 yaw 범위 `[-35, +35] deg` |
| `--roll-min-deg`, `--roll-max-deg` | 미지정(`deg`) | 비대칭 roll 범위 override |
| `--pitch-min-deg`, `--pitch-max-deg` | 미지정(`deg`) | 비대칭 pitch 범위 override |
| `--yaw-min-deg`, `--yaw-max-deg` | 미지정(`deg`) | 비대칭 yaw 범위 override |
| `--rpy-norm-max-rad` | 미지정(`rad`) | 생성된 RPY vector norm 상한 |
| `--base-z-offset-mm` | `0 mm` | 모든 collect target에 더할 공통 접근축 거리 |

#### Sample 승인 조건

| CLI 옵션 | 기본값 | 조정 목적 |
|---|---:|---|
| `--min-visible-cameras` | `2 cameras` | sample을 저장하기 위해 포트가 보여야 하는 최소 camera 수 |
| `--visibility-margin-px` | `64 px` | image 경계에서 제외할 pixel margin |
| `--sync-tolerance-ms` | `30 ms` | center image 기준 camera·ControllerState·동적 plug TF의 최대 시각 차이 |
| `--sync-wait-timeout-s` | `1 s` | 유효 Observation과 capture 시각 plug TF의 최대 대기시간 |

#### Dataset sample visibility 적용 범위

common-FOV 조건은 triangulation에만 적용되는 것이 아니다. 다만 적용 시점이 다르다.

| 경로 | 적용 시점 | 동작 |
|---|---|---|
| `ais_triangulation/run_triangulation_cases.py | generate_cases()` | simulator 실행 전 YAML 생성 | fixed home 기준 common-FOV를 통과한 candidate만 trial로 채택 |
| `phy/phy_policy/data_generator/data_generator/port_offset_dataset.py | _port_projection_for_camera()` | PortOffset sample capture 시점 | 실제 ControllerState camera pose, CameraInfo와 port TF로 camera별 pixel·depth 계산 |
| `phy/phy_policy/data_generator/data_generator/port_offset_dataset.py | _save_xyz_rpy_sample()` | JPEG·metadata 쓰기 직전 | 기본 64 px margin 안에 port가 보이는 camera가 두 개 이상일 때만 sample 저장 |

PortOffset sample이 visibility 또는 timestamp 조건을 통과하지 못하면 JPEG, camera metadata JSON과 `metadata.jsonl`을 쓰지 않으며 저장 count도 증가시키지 않는다. 일부 camera 파일을 쓴 뒤 최소 camera 수를 충족하지 못한 경우 생성된 부분 파일도 삭제한다.

이 검사는 실제 capture 시점의 기하학적 port 중심 가시성을 보장한다. Mesh 가림, 조명, lens distortion과 YOLO confidence 통과까지 보장하지는 않는다. `--min-visible-cameras` 또는 `--visibility-margin-px`를 명시하면 기본 승인 조건을 override한다.

#### 조명과 배경 randomization

연속값은 min/max를 `μ ± 3σ`로 해석한 truncated Gaussian에서 sampling합니다.

| CLI 옵션 | 기본값 | 조정 목적 |
|---|---:|---|
| `--randomize-lighting` | `true` | randomized world 생성 여부 |
| `--light-intensity-scale-min`, `--light-intensity-scale-max` | `0.65×`, `1.35×` | light intensity 배율 범위 |
| `--light-color-jitter` | `0.12` (RGB, `0~1`) | 흰색 기준 RGB 채널별 최대 변화량 |
| `--light-pose-xy-jitter-m` | `0.25 m` | light X/Y 위치 최대 변화량 |
| `--light-pose-z-jitter-m` | `0.20 m` | light Z 위치 최대 변화량 |
| `--ambient-min`, `--ambient-max` | `0`, `0.08` (RGB, `0~1`) | scene ambient RGB 밝기 범위 |
| `--background-min`, `--background-max` | `0.08`, `0.20` (RGB, `0~1`) | scene background RGB 밝기 범위 |

#### HuggingFace 업로드

| CLI 옵션 | 기본값 | 조정 목적 |
|---|---:|---|
| `--dataset-version` | 빈 문자열 | 결과 version 디렉터리와 metadata version |
| `--push-to-hub` | `false` | 수집 완료 후 dataset 업로드 여부 |
| `--vision-offset-repo-id` | `aic-sejong-team/aic-vision-offset-dataset` | 업로드할 dataset repository |
| `--vision-offset-hf-revision` | `main` | 업로드할 branch/revision |
| `--upload-on-port-type` | 빈 문자열 | `sfp` 또는 `sc` trial에서만 누적 dataset 업로드; 빈 값은 매 성공 trial |
| `--hf-private` | `false` | 새 repository의 private 생성 여부 |

### 1-2. rosbag 자동 녹화

`--record-rosbag true`는 각 trial을 독립 MCAP으로 기록하며, offline sample 일치 검사의 원본 데이터로 사용합니다.

```text
Gazebo + Zenoh 시작
  → rosbag 준비 확인
  → policy 실행 및 종료
  → rosbag SIGINT finalize 및 검증
  → Gazebo 종료
  → 다음 trial
```

| 옵션 | 기본값 | 역할 |
|---|---:|---|
| `--record-rosbag` | `false` | `true`이면 자동 녹화를 활성화합니다. |
| `--rosbag-output-dir` | `aic-physic/rosbags/portoffset` | 최상위 출력 경로입니다. |
| `--rosbag-topics` | `/clock`, TF, controller, 카메라 토픽 | 녹화할 토픽 목록입니다. |
| `--rosbag-start-timeout-s` | `20 s` | recorder 시작 대기시간입니다. |
| `--rosbag-stop-grace-s` | `30 s` | SIGINT finalize 대기시간입니다. |

기본 출력은 `aic-physic/rosbags/portoffset/{dataset-version}/{run-id}/{trial}`입니다. `metadata.yaml`, 0보다 큰 message count, 모든 MCAP의 시작·종료 magic이 유효해야 green bold `RECORDING COMPLETED` 로그가 출력되고 다음 trial로 진행합니다. 실패 시 해당 run을 중단합니다.

| 함수 | 핵심 책임 |
|---|---|
| `_run_trial()` | `Gazebo → rosbag → policy` 시작 순서와 `policy → rosbag → Gazebo` 종료 순서를 보장합니다. |
| `start_rosbag()` / `wait_for_rosbag_start()` | recorder를 시작하고 녹화 준비를 확인합니다. |
| `stop_rosbag()` / `validate_rosbag()` | SIGINT 종료와 MCAP 완결성 검증을 수행합니다. |
| `cleanup_stale_processes()` | 비정상 종료 후 남은 recorder를 SIGINT 우선으로 정리합니다. |

### 1-3. 수집 시각 일치 검사

center image `header.stamp`가 sample 기준 시각입니다. collector는 최대 `--sync-wait-timeout-s` 동안 새 Observation을 순차 확인하고, 세 Image와 ControllerState가 `--sync-tolerance-ms` 안에 들어온 첫 Observation을 선택합니다.

포트는 trial 중 움직이지 않는다는 시나리오 조건을 사용해 trial 시작 시 최신 TF를 한 번 `port_tf_snapshot`으로 저장합니다. 이 snapshot은 `is_static_snapshot=true`로 기록하고 시각 차이 계산에서 제외하므로, 1 Hz port TF 발행 주기를 각 capture 시각에 맞추기 위해 기다리지 않습니다. 한편 플러그는 위치가 계속 변화하므로 선택된 center image 시각의 TF를 최대 동일 timeout 동안 기다려 조회합니다.

`_lookup_transform_at()`은 별도 TF cache를 만들지 않고 policy의 메인 TF2 buffer에서 `center_image.header.stamp`를 지정해 `base_link <- plug` transform을 한 번 조회합니다. TF2가 해당 시각의 transform을 반환하지 못하면 sample을 저장하지 않습니다.

```text
동작 방식:
  새 Observation 수신
  → center image의 새로운 timestamp 선택
  → 해당 timestamp의 plug TF 조회
  → 조건을 통과하면 실제 plug-port 관계 저장
  → 다음 sample은 새로운 timestamp로 다시 시작

trial 시작:
  port_tf_snapshot = latest(base_link <- port entrance)

저장 조건:
  camera_time_difference <= sync_tolerance
  controller_time_difference <= sync_tolerance
  dynamic plug TF time_difference <= sync_tolerance
```

timeout이나 허용 오차를 넘긴 sample은 JPEG, 카메라별 sample metadata JSON 및 `metadata.jsonl`을 쓰지 않고 저장 count도 증가시키지 않습니다.

카메라별 sample metadata JSON과 `metadata.jsonl`의 `timestamps`에는 `capture_stamp_ns`, camera별 stamp, `controller_stamp_ns`, port/plug TF stamp, `is_static_snapshot`, `skew_ns`, `wait_ns`, `sync_tolerance_ns`, `sync_valid`, `dataset_write_stamp_ns`가 기록되며 모든 시각 값의 단위는 `ns`입니다. 호환성을 위해 유지하는 `skew_ns`는 source 사이의 최대 시각 차이를 `ns` 단위로 저장하는 필드입니다.

| 함수 | 현재 역할 |
|---|---|
| `_add_sync_args()` / `_policy_environment()` | 허용 오차·대기 timeout·색상 설정을 policy 환경변수로 전달합니다. |
| `init_runtime()` | 허용 오차와 최대 대기시간을 초기화합니다. |
| `insert_cable()` / `_lookup_latest_transform_stamped()` | trial 시작 시 고정 포트 TF를 한 번 snapshot합니다. |
| `_wait_for_synchronized_observation()` | timeout 동안 Image/ControllerState 조건을 만족하는 Observation을 선택합니다. |
| `_lookup_transform_at()` / `_tf_sync_metadata()` | 메인 TF2 buffer에서 center image 시각의 동적 plug TF를 한 번 조회하고 port snapshot과 함께 metadata를 검증합니다. |
| `_stage_collect()` | 정지 속도 판정 없이 수집 시각 일치 조건을 통과한 sample만 저장기로 전달합니다. |
| `_save_xyz_rpy_sample()` | 이미지·label·timestamp metadata가 모두 기록된 뒤에만 count를 증가시키고 성공 여부와 이유를 반환합니다. |

저장 시도 결과는 색상과 이유를 함께 출력합니다.

| 로그 | 색상 | 의미 |
|---|---|---|
| `CAPTURE SAVED` | 초록색 bold | JPEG와 metadata가 실제로 기록되고 저장 count가 증가함 |
| `CAPTURE FAILED` | 빨간색 bold | 시각 차이 초과, TF 조회 실패, visibility 부족 또는 파일 저장 실패로 sample이 저장되지 않음 |

시각 관련 실패 로그에는 camera/controller/plug TF의 실제 시각 차이와 허용 범위를 `ms` 단위로 표시합니다. 파일 저장 도중 실패하면 해당 시도에서 생성한 부분 JPEG와 metadata JSON을 삭제합니다.

### 1-4. Offline sample 일치 검사

`validate_portoffset_trial.py`는 같은 trial의 dataset과 MCAP을 직접 읽어 모든 저장 sample을 자동 검사합니다.

```bash
pixi run ros2 run phy_data_collection validate_portoffset_trial \
  --dataset-dir "$(git rev-parse --show-toplevel)/ws_aic/data/phy_portoffset_randomization/0726-001" \
  --rosbag "$(git rev-parse --show-toplevel)/rosbags/portoffset/0726-001/<run-id>/<trial-dir>"
```

| 검사 항목 | 합격 조건 | 단위 |
|---|---|---|
| 저장 JPEG | 동일 camera와 `header.stamp`의 MCAP Image를 수집기와 같은 방식으로 JPEG encoding한 결과와 일치 | `byte` |
| 세 camera 시각 | `max(left, center, right) - min(left, center, right) <= sync_tolerance` | `ns`, 보고서에 `ms` 병기 |
| ControllerState 시각 | `abs(controller - center_image) <= sync_tolerance` | `ns`, 보고서에 `ms` 병기 |
| port entrance | trial 시작 snapshot과 capture 시각의 위치 차이 `0.1 mm`, 회전 차이 `0.001 rad` 이하 | `mm`, `rad` |
| plug reference | capture 시각의 TF가 존재하고 기록된 transform과 위치 차이 `0.1 mm`, 회전 차이 `0.001 rad` 이하 | `mm`, `rad` |
| `location`, `label` | MCAP port/plug TF와 plug local offset으로 재계산한 값의 위치 차이 `0.1 mm`, 회전 차이 `0.001 rad` 이하 | `mm`, `rad` |

결과는 기본적으로 trial rosbag 디렉터리의 `portoffset_validation.json`에 기록되며, 전체 sample이 합격하면 exit code `0`, 불합격 sample이 있으면 `1`, 입력 또는 실행 오류는 `2`를 반환합니다. 보고서에는 sample별 `PASS/FAIL`, camera/controller/plug TF 시각 차이, TF와 label 오차 및 실패 원인이 포함됩니다.

새로 수집한 metadata는 `collection.run_id`, `collection.trial_index`, `collection.rosbag_path`로 MCAP을 정확히 연결합니다. 기존 metadata에는 이 값이 없으므로 `--sample-id <id>`를 명시해 검사합니다.

MCAP topic 자체가 없으면 `pixi run ros2 bag info <trial-dir>`로 message count를 확인하고 collector와 recorder가 동일한 `rmw_zenoh_cpp`/Zenoh router를 사용하는지 확인합니다.

### 1-5. Hugging Face에 업로드되는 데이터 포맷

`_upload_vision_offset_dataset_to_hub()`는 현재 dataset version 디렉터리 전체를 dataset repository의 지정 revision에 업로드합니다. rosbag은 별도 `rosbags/` 경로이므로 이 업로드에 포함되지 않습니다.

```text
<dataset-version>/
├── data.yaml
├── metadata.jsonl
├── images/<train|val>/<SFP|SC>/<left|center|right>/*.jpg
└── metadata/<train|val>/<SFP|SC>/<left|center|right>/*.json
```

포트가 보이는 카메라마다 JPEG 한 장과 같은 이름의 sample metadata JSON 한 개를 저장합니다. 예를 들어 `..._center.jpg`의 label·timestamp 정보는 `..._center.json`에 있습니다. 기본값에서는 64 px margin 안쪽에 포트가 보이는 camera가 두 개 이상일 때만 sample을 저장하며, 반드시 세 camera가 모두 저장되는 것은 아닙니다.

각 카메라별 sample metadata JSON의 핵심 필드는 다음과 같습니다.

| 필드 | 의미 | 좌표 기준 | 값 단위 |
|---|---|---|---|
| `image`, `camera`, `connector` | JPEG 상대 경로와 camera/connector 구분 | dataset 경로 | 문자열 |
| `collection` | 정확한 trial의 `run_id`, `trial_index`, `rosbag_path` | 수집 실행 | 문자열, index |
| `task`, `scenario` | task ID와 board/cable/lighting randomization | world/scenario 설정 | 각 하위 필드에 명시 |
| `command` | 해당 step에서 명령한 TCP pose | `base_link` | 위치 `m`, 회전 quaternion |
| `plug_reference` | cable TF 원점에서 실제 plug 기준점까지의 local offset | 선택된 plug frame | `m` |
| `collect.local_*` | 의도적으로 가한 XYZ/RPY sample | port-local 축 | 위치 `m`, 회전 `rad`/`deg` |
| `location` | 실제 `plug_reference - port_entrance` 위치와 `R_plug R_port^T` 회전 | `base_link` | 위치 `m`, 회전 `rad` |
| `label` | 정렬 correction인 `port_entrance - plug_reference` 위치와 `R_port R_plug^T` 회전 | `base_link` | 위치 `m`, 회전 `rad` |
| `visibility` | port projection, depth, image 크기와 visible camera 목록 | camera별 | pixel `px`, depth `m` |
| `timestamps` | source 시각, 최대 시각 차이, 대기시간과 저장 승인 결과 | ROS time | `ns` |
| `timestamps.tf.*.transform` | 저장에 사용한 port/plug transform | `base_link` | 위치 `m`, 회전 quaternion |

`location`과 `label`은 독립적인 두 offset이 아니라 동일한 plug-port 관계의 정방향 상태와 역방향 correction입니다. raw port/plug TF message 전체는 dataset에 넣지 않지만 저장 계산에 사용한 frame과 transform은 metadata에 기록하고, 원본 message는 trial MCAP에 보관합니다.

---

## 2. collect_lerobot_data.py — LeRobot 에피소드 수집 (distrobox / x86)

`collect_lerobot_data_aarch.py`와 동일한 역할이지만 distrobox 컨테이너 환경에서 동작한다. aarch64 환경에서는 `collect_lerobot_data_aarch.py`를 사용한다.

---

## 3. collect_lerobot_data_aarch.py — LeRobot 에피소드 수집 (aarch64)

세트당 7개 trial(NIC×5 + SC×2)을 Gazebo에서 자동 실행하며 LeRobot 포맷으로 저장한다.

**흐름**
1. trial별 랜덤 파라미터로 aic_engine config YAML 생성
2. Zenoh 라우터 → Gazebo(`aic_gz_bringup`) → `LeRobot` policy 순으로 시작
3. `episode_summary.json` 파일 수로 완료 감지
4. Gazebo 종료 → 다음 세트 반복

**사용법**
```bash
# 기본: 10 세트 × 7 에피소드
ros2 run phy_data_collection collect_lerobot_data_aarch

# 50 세트, 보드 위치/yaw 랜덤화
ros2 run phy_data_collection collect_lerobot_data_aarch --sets 50 --diversify

# 명령어만 출력 (실제 실행 X)
ros2 run phy_data_collection collect_lerobot_data_aarch --sets 5 --dry-run

# Gazebo GUI·RViz 없이 실행
ros2 run phy_data_collection collect_lerobot_data_aarch --headless

# LeRobot 로컬 저장 + HuggingFace 업로드
ros2 run phy_data_collection collect_lerobot_data_aarch \
  --lerobot-out-dir ~/data \
  --lerobot-repo-id aic-sejong-team/aic-dataset
```

### LeRobot 공통 랜덤화 파라미터

LeRobot episode 자동 수집 스크립트는 세트당 SFP/NIC trial 5개와 SC trial 2개를 생성한다.

| 파라미터 종류 | 범위 | 역할 |
|---|---|---|
| NIC/SFP card translation | `-0.0215 ~ 0.0234 m` | SFP/NIC target card의 rail 방향 위치를 랜덤화합니다. |
| NIC/SFP card yaw | `-10 ~ +10 deg` | SFP/NIC target card의 yaw를 랜덤화합니다. |
| SC port translation | `-0.06 ~ 0.055 m` | SC target port의 rail 방향 위치를 랜덤화합니다. |
| SFP task board X | `0.13 ~ 0.17 m` | `--diversify` 사용 시 SFP/NIC trial의 task board X 위치를 랜덤화합니다. |
| SFP task board Y | `-0.25 ~ -0.15 m` | `--diversify` 사용 시 SFP/NIC trial의 task board Y 위치를 랜덤화합니다. |
| SC task board X | `0.15 ~ 0.19 m` | `--diversify` 사용 시 SC trial의 task board X 위치를 랜덤화합니다. |
| SC task board Y | `-0.05 ~ 0.05 m` | `--diversify` 사용 시 SC trial의 task board Y 위치를 랜덤화합니다. |
| Task board yaw | `0.0 ~ 3.1415 rad` | 모든 trial에서 task board yaw를 랜덤화합니다. |
| Gripper offset noise | 각 축 `-0.002 ~ +0.002 m` | cable gripper offset 기준값에 미세 오차를 추가합니다. |
| SFP gripper offset base | `[0, 0.015385, 0.04245] m` | SFP cable grasp offset 기준값입니다. |
| SC gripper offset base | `[0, 0.015385, 0.04045] m` | SC cable grasp offset 기준값입니다. |

---

## 4. collect_yolo_data_aarch.py — YOLO 데이터셋 수집 (aarch64)

시나리오별로 Gazebo를 별도 세션으로 실행하며 3대 카메라 스냅샷 + TF 기반 bbox 라벨을 자동 생성한다.

**흐름 (시나리오당)**
1. 랜덤 파라미터로 aic_engine config YAML 생성 + scenario_params JSON 저장
2. Zenoh 라우터 → Gazebo → `LeRobot` policy 시작
   - `LeRobot` policy를 사용해야 Task Board가 실제로 spawn됨
   - `autocapture`는 lifecycle만 수행하므로 entity spawn이 보장되지 않음
3. 카메라 데이터 및 포트 TF 확인 후 스냅샷 N장 수집
4. YOLO 라벨 자동 생성 (TF 기반 핀홀 투영)
5. Gazebo 종료 → 다음 시나리오

**시나리오 구성 (세트당 7개)**
- NIC rail 0~4: SFP 포트 레이블 (`sfp_port`, class 0)
- SC rail 0~1: SC 포트 레이블 (`sc_port`, class 1)

**출력 구조**
```
<output>/<YYYYMMDD>/
├── images/
│   ├── train/  s00001_nic0_snap0000_left.jpg, ...
│   └── val/
├── labels/
│   ├── train/  s00001_nic0_snap0000_left.txt
│   └── val/
└── data.yaml
```

**사용법**
```bash
# 기본: 10 세트 × 7 시나리오, 스냅샷 20장
ros2 run phy_data_collection collect_yolo_data_aarch --sets 10

# 스냅샷 수 / 보드 위치 랜덤화
ros2 run phy_data_collection collect_yolo_data_aarch --sets 20 --snapshots 30 --diversify

# Gazebo GUI 없이, 명령어만 출력 테스트
ros2 run phy_data_collection collect_yolo_data_aarch --sets 5 --headless --dry-run

# 출력 경로 지정
ros2 run phy_data_collection collect_yolo_data_aarch --sets 10 --output ~/data/yolo
```

**주요 옵션**

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--sets` | `10 sets` | 수집 세트 수 |
| `--snapshots` | `20 images/scenario` | 시나리오당 스냅샷 수 |
| `--diversify` | off | 보드 x/y 위치 랜덤화 |
| `--headless` | off | Gazebo GUI·RViz 비활성 |
| `--gazebo-wait` | `60 s` | Gazebo 초기화 대기시간 |
| `--val-ratio` | `0.3` (비율, `0~1`) | 검증 세트 비율 |
| `--output` | `src/data/yolo` | YOLO 데이터셋 출력 경로 |
| `--dry-run` | off | 명령어만 출력 |

---

## 5. plot_scenario_randomization.py — 시나리오 랜덤화 plot 생성

plot은 `phy_data_collection.portoffset_randomization.constants.LIMITS`와 수집 CLI의 현재 기본값을 직접 읽어 Task Board, target port 및 조명 분포를 그립니다.

```bash
cd ws_aic/src/aic
pixi run ros2 run phy_data_collection plot_scenario_randomization
```

기본 출력은 [scenario_randomization_distributions.png](../../../../readme/photo/scenario_randomization_distributions.png)입니다.

```bash
pixi run ros2 run phy_data_collection plot_scenario_randomization \
  --output /tmp/scenario_randomization.png \
  --dpi 200 \
  --port-types sfp,sc \
  --port-order random

pixi run ros2 run phy_data_collection plot_scenario_randomization --help
```

조명 범위는 `--light-intensity-scale-min/max`, `--light-color-jitter`, `--light-pose-xy-jitter-m`, `--light-pose-z-jitter-m`, `--ambient-min/max`, `--background-min/max`로 덮어쓸 수 있습니다.
