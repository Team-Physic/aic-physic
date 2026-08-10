# phy_data_collection

랜덤화된 AIC 시뮬레이션에서 거리 구간별 collection policy를 실행해 img2pos 학습
데이터를 수집하고 품질을 평가하는 ROS 2 Python 패키지입니다. 수집 runner는
trial별 pose 수렴 후 이미지를 저장하며, 여러 headless simulator를 격리해 병렬 실행할
수 있습니다.

## 실행

명령은 Pixi workspace인 `ws_aic/src`에서 실행합니다.

```bash
cd ws_aic/src

PIXI_FROZEN=true pixi run ros2 run phy_data_collection \
  collect_portoffset_randomization_data \
  --collection-policy board-view \
  --port-type sfp \
  --trials 155 \
  --workers 5 \
  --samples-per-trial 20 \
  --dataset-version img2pos-v1 \
  --push-to-hub false \
  --record-rosbag false
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
  → host Pixi 환경에서 선택한 collection policy 시작
  → 정책별 거리·횡방향 위치·카메라 각도 pose 순회
  → controller가 limit 적용 후 수락한 reference pose가 멈추고 실제 TCP의 frame 간
    이동량이 연속 frame에서 허용 범위 안으로 수렴할 때까지 대기
  → 새 camera frame과 같은 시각의 plug TF 검증
  → 동기화된 세 camera의 JPEG와 공통 XYZ label 저장
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
| `--port-type` | 필수 | 모든 trial에서 사용할 connector: `sfp` 또는 `sc` |
| `--trials` | `34` | 실행할 전체 trial 수 |
| `--workers` | `1` | 동시에 실행할 격리 simulator 수 |
| `--color-log {true,false}` | `true` | terminal ANSI 색상 로그 사용 여부 |
| `--samples-per-trial` | `40` | trial마다 저장할 capture 수 |
| `--time-limit-s` | `600` | AIC trial config에 전달할 제한시간(초) |
| `--trial-timeout-s` | 자동 (`780`) | runner가 summary를 기다리는 시간; 미지정 시 `time-limit-s + 180` |
| `--collection-policy` | `near-port` | `board-view`, `descent`, `near-port` 중 선택 |
| `--policy` | 정책별 자동 선택 | `collection-policy`에 연결된 ROS policy module을 직접 override |
| `--policy-start-wait-s` | `5` | simulator 시작 뒤 policy를 실행하기 전 대기시간(초) |
| `--dataset-version` | 자동 생성 | `ws_aic/data/img2pos/` 아래 새 version 경로 |
| `--resume` | 꺼짐 | 기존 version에 명시적으로 이어서 수집 |
| `--val-ratio`, `--test-ratio` | `0.15`, `0.15` | trial 단위 validation/test 비율 |
| `--headless`, `--no-headless` | `--headless` | Gazebo GUI 비활성화/활성화; `workers=1`일 때만 `--no-headless`가 적용됨 |
| `--launch-rviz {true,false}` | `false` | RViz 실행 여부; headless 또는 병렬 실행이면 항상 꺼짐 |
| `--distrobox` | `aic_eval_physic` | eval container 이름 |
| `--engine-setup` | `/ws_aic/install/setup.bash` | 호환용 parser 옵션; 현재 simulator launch도 이 기본 경로를 고정 source함 |
| `--ros-domain-id-base` | `40` | worker 0의 ROS domain ID |
| `--zenoh-port-base` | `7600` | worker 0의 Zenoh router TCP port |
| `--worker-start-delay-s` | `2` | 병렬 worker 시작 간격(초) |
| `--robot-joint-noise-rad` | `0.069813` | home joint 각 축의 독립 대칭 잡음 범위 |
| `--cable-rotation-noise-rad` | `0.04` | 기준 cable 자세 대비 전체 회전각 상한 |

dataset version을 생략하면 실행 시각으로 새 이름을 만들고, 기존 version은 `--resume`
없이는 수정하지 않습니다. 따라서 Hugging Face의 기존 PoC branch도 자동으로 덮어쓰지
않습니다.

Cable 회전 잡음은 roll/pitch/yaw 각 축에 독립적으로 더하지 않습니다. 무작위
회전축과 `0~--cable-rotation-noise-rad` 회전각으로 quaternion을 만들어
기준 cable 자세에 합성하므로, 전체 상대 회전 오차는 기본 `0.04rad` 이하입니다.

Boolean 옵션 형식은 세 종류입니다. `--headless`만 `--headless`/`--no-headless` 쌍을
사용합니다. `--color-log`, `--launch-rviz`, `--push-to-hub`, `--record-rosbag`,
`--randomize-lighting`에는 `true` 또는 `false` 값을 반드시 붙입니다. `--resume`,
`--hf-private`, `--cleanup`, `--cleanup-only`, `--dry-run`은 옵션이 있으면 켜지는 flag입니다.

### 병렬 headless 실행

각 worker는 고유 `ROS_DOMAIN_ID`, Zenoh router TCP port, Gazebo partition을 사용합니다.
trial index는 worker에 round-robin으로 분배되며 JSONL append는 file lock으로 보호합니다.
실행마다 master seed가 자동 생성되며, 각 global trial은 서로 다른 RNG로 card 조합,
Task Board pose와 조명을 독립 추출합니다. connector는 `--port-type`으로 고정합니다. 한
trial이 실패해도 worker는 남은 index를 계속 실행하며, 전체 종료 코드는 실패를 유지합니다.
기본 격리 값은 `ROS_DOMAIN_ID=40+worker`, Zenoh `7600+worker`입니다.

```bash
PIXI_FROZEN=true pixi run ros2 run phy_data_collection \
  collect_portoffset_randomization_data \
  --port-type sfp --trials 34 --workers 2 --samples-per-trial 40 \
  --dataset-version img2pos-v1
```

`--workers`를 늘리면 Gazebo마다 CPU, RAM과 GPU VRAM을 별도로 사용합니다. 먼저 2개로
확인하고 자원 여유에 따라 늘리십시오. 병렬 모드에서는 `--no-headless`나
`--launch-rviz true`를 전달해도 GUI와 RViz가 항상 꺼집니다.
`--ros-domain-id-base`, `--zenoh-port-base`, `--worker-start-delay-s`로 격리 값을 조정할
수 있습니다.

### Pose 분포

| 정책 | 동작 |
| --- | --- |
| `board-view` | center camera optical axis가 보드 중앙을 향하도록 750~850mm 거리에서 횡방향 위치와 각도를 무작위화하고, 목표 port가 영상 안에 있는 capture만 저장 |
| `descent` | 선택 포트를 향해 550mm부터 20mm 안전거리까지 먼 순서로 내려오며 거리·횡방향 위치·각도를 무작위화 |
| `near-port` | 기존 20mm 안전거리 기준 50/10/5/2mm coarse/near tier를 수집 |

`--port-type sfp`면 각 trial이 31개 non-empty 5-bit card 조합에서 하나를 uniform
추출합니다. `--port-type sc`면 3개 non-empty 2-bit 조합에서 하나를 uniform 추출합니다.
bit의 오른쪽부터 rail 0, 1, ...에 대응하며 `0`은 card 없음, `1`은 card 생성을 뜻합니다.
target rail도 추출된 조합의 활성 rail 중에서 uniform 선택합니다.

```text
SFP: Uniform({00001, 00010, ..., 11111})
SC:  Uniform({01, 10, 11})
```

따라서 `--trials 34`여도 모든 조합이 정확히 한 번씩 나온다는 보장은 없습니다. worker마다
담당한 각 trial에서 새 조합을 추출하므로 같은 조합이 반복되거나 일부 조합이 빠질 수 있습니다.

| 옵션 | 기본값 | 설명 |
| --- | ---: | --- |
| `--dx-min-mm`, `--dx-max-mm` | `-50`, `50` | port-local X 범위 |
| `--dy-min-mm`, `--dy-max-mm` | `-50`, `50` | port-local Y 범위 |
| `--dz-min-mm`, `--dz-max-mm` | `0`, `50` | 최소 안전거리에 더하는 비관통 방향 port-local Z 범위 |
| `--sampling-tiers-mm` | `50,10,5,2` | coarse 및 근접 위치 sampling tier |
| `--sampling-tier-weights` | `1,1,1,1` | tier별 capture quota 비율 |
| `--port-roll-limit-rad` | `0.436332` | 대칭 roll 범위 |
| `--port-pitch-limit-rad` | `0.436332` | 대칭 pitch 범위 |
| `--port-yaw-limit-rad` | `0.610865` | 대칭 yaw 범위 |
| `--roll-min-rad`, `--roll-max-rad` | 미지정 | 비대칭 roll 범위 override |
| `--pitch-min-rad`, `--pitch-max-rad` | 미지정 | 비대칭 pitch 범위 override |
| `--yaw-min-rad`, `--yaw-max-rad` | 미지정 | 비대칭 yaw 범위 override |
| `--rpy-norm-max-rad` | 미지정 | 생성된 RPY vector norm 상한 |
| `--base-z-offset-mm` | `20` | descent/near-port가 유지할 접근축 최소 안전거리 |
| `--board-distance-min-mm`, `--board-distance-max-mm` | `750`, `850` | board-view center camera optical 거리 범위 |
| `--board-lateral-limit-mm` | `30` | board-view camera-plane X/Y 대칭 범위 |
| `--board-angle-limit-rad` | `0.261799` | board-view RPY 대칭 범위 |
| `--descent-start-distance-mm` | `550` | descent 시작 거리 |
| `--descent-lateral-limit-mm` | `40` | descent port-local X/Y 대칭 범위 |
| `--descent-angle-limit-rad` | `0.349066` | descent RPY 대칭 범위 |

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
| `--sync-tolerance-ms` | `30` | capture stamp와 controller/동적 TF 최대 시각 차이 |
| `--sync-wait-timeout-s` | `1` | 새 Observation과 capture 시각 TF 대기시간 |
| `--settle-timeout-s` | `8` | reference와 실제 TCP 움직임 수렴 제한시간 |
| `--settle-position-tolerance-mm` | `1` | 연속 controller frame 간 TCP 위치 이동 허용량 |
| `--settle-orientation-tolerance-rad` | `0.0174533` | 연속 controller frame 간 TCP 자세 이동 허용량 |
| `--settle-stable-observations` | `3` | 연속으로 통과해야 하는 서로 다른 controller frame 수 |
| `--settle-poll-s` | `0.02` | pose 수렴 여부를 다시 확인하는 polling 간격(초) |
| `--max-attempts` | `2` | 가시성 실패를 제외한 동일 waypoint의 최대 capture 시도 횟수 |

세 image의 `header.stamp`가 정확히 같고 controller가 허용 오차 안에 있는 Observation만
사용합니다. 공통 capture stamp로 plug TF를 한 번 조회하여 하나의 `target_xyz_m`을
계산합니다.
모든 collection policy에서 목표 port가 지정 개수 이상의 camera에 보일 때만 승인합니다.
승인된 capture는 일부 camera에서 port가 보이지 않더라도 동기화된 left/center/right 이미지 3장과
공통 `target_xyz_m`을 한 묶음으로 저장합니다. 영상 아래쪽 경계에 연결된 큰 검은 영역은
robot arm으로 판정하며, 이 영역에 가리지 않고 실제 image 범위 안에 투영된 camera만 가시
camera 수에 포함합니다. 따라서 한 camera가 robot arm에 가려져도 다른 두 camera에서 보이면
capture를 승인합니다. 이미지 한 장이라도 저장하지 못하면 partial
capture를 삭제하고 재시도합니다. port 가시성 조건을 통과하지 못하면 같은 waypoint를
반복하지 않고 다음 waypoint로 이동합니다. 준비된 waypoint를 모두 사용해도 저장된
capture 수가 목표보다 적으면 새 waypoint 묶음을 생성해 계속 수집합니다. `descent`와
`near-port`는 실제 TF의 접근축 거리가 20mm 이상인지 검사하고, `near-port`는 추가로
20mm 안전거리를 제외한 port-local offset이 배정된 sampling tier 안에 있는지도
검사합니다. 명령 직후 frame은
사용하지 않고 controller reference와 실제 TCP 움직임이 수렴한 이후의 다음 camera
frame을 선택합니다. 조건을 통과하지 못한 tier는 재시도하며 목표 capture 수를 채우지
못한 trial은 실패 처리합니다.

임피던스 제어에서는 외력과 유한 stiffness 때문에 reference와 실제 TCP 사이에
정상상태 tracking error가 남을 수 있습니다. 이 값은 log 진단용으로만 기록하고 동작
완료 조건으로 사용하지 않습니다. 대신 실제 TF label을 저장하고, `descent`와
`near-port`의 최소거리 검사 및 `near-port`의 tier 검사가 최종 sample 정확도와
안전거리를 보장합니다.

`collection_summary.json`의 `near_port_sampling_offset_box_coverage_mm`은 `near-port`
capture만 대상으로, 촬영 시점 TF에서 안전거리를 제외한 실제 port-local offset이 각
±2/5/10/50mm box 안에 들어온 수와 비율입니다. `target_xyz_m`에는 추론과 제어에 필요한
안전거리까지 포함된 `base_link` correction을 그대로 저장합니다.

port는 trial 동안 고정되므로 시작 시 한 번 snapshot하고, 움직이는 plug는 공통 capture
시각으로 조회합니다. 학습 row에는 하나의 `capture_stamp_ns`와 승인 source 중 가장 큰
`max_sync_skew_ns`만 남기며 상세 원본 이벤트는 선택적 rosbag으로 보관합니다.

### World randomization

조명, ambient와 background 값은 각 옵션 범위를 ±3σ로 해석한 truncated Gaussian에서
trial마다 추출합니다.

| 옵션 | 기본값 | 설명 |
| --- | ---: | --- |
| `--randomize-lighting {true,false}` | `true` | trial별 world 조명·배경 randomization 사용 여부 |
| `--light-intensity-scale-min`, `--light-intensity-scale-max` | `0.65`, `1.35` | 원본 light intensity에 곱할 scale 범위 |
| `--light-color-jitter` | `0.12` | 흰색 기준 RGB 각 채널의 독립 대칭 jitter 범위 |
| `--light-pose-xy-jitter-m` | `0.25` | light X/Y 위치의 독립 대칭 jitter 범위(m) |
| `--light-pose-z-jitter-m` | `0.20` | light Z 위치의 대칭 jitter 범위(m) |
| `--ambient-min`, `--ambient-max` | `0.0`, `0.08` | scene ambient grayscale 범위 |
| `--background-min`, `--background-max` | `0.08`, `0.20` | scene background grayscale 범위 |

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

| 옵션 | 기본값 | 설명 |
| --- | ---: | --- |
| `--record-rosbag {true,false}` | `false` | trial별 MCAP 기록 여부 |
| `--rosbag-output-dir` | `<repo>/rosbags/portoffset` | dataset version, run, trial 디렉터리를 만들 기준 경로 |
| `--rosbag-topics` | 기본 10개 topic | 공백으로 구분해 기록할 topic 목록 지정 |
| `--rosbag-start-timeout-s` | `20` | recorder 시작 확인 제한시간(초) |
| `--rosbag-stop-grace-s` | `30` | MCAP finalize를 위한 SIGINT 대기시간(초) |

- `metadata.yaml` 존재
- message count가 0보다 큼
- 모든 MCAP 파일의 시작·종료 magic이 유효함

기본 기록 topic은 다음과 같습니다.

```text
/clock
/joint_states
/tf
/tf_static
/scoring/tf
/aic_controller/controller_state
/aic_controller/pose_commands
/left_camera/image
/center_camera/image
/right_camera/image
```

### 실행·정리 고급 옵션

| 옵션 | 기본값 | 설명 |
| --- | ---: | --- |
| `--policy-stop-grace-s` | `10` | stop file 전달 뒤 policy 정상 종료 대기시간(초) |
| `--post-summary-wait-s` | `3` | collection summary 뒤 AIC scoring/reset 대기시간(초) |
| `--sim-sigint-grace-s` | `5` | simulator SIGINT 정상 종료 대기시간(초) |
| `--sim-cleanup-grace-s` | `2` | 남은 소유 process group의 SIGTERM 대기시간(초) |
| `--sim-sigkill-grace-s` | `1` | 최종 SIGKILL 뒤 종료 확인시간(초) |
| `--between-trial-wait-s` | `3` | 한 trial 정리 후 다음 trial 시작 전 대기시간(초) |
| `--cleanup` | 꺼짐 | 이전 실행이 registry에 남긴 소유 process group을 정리한 뒤 수집 계속 |
| `--cleanup-only` | 꺼짐 | 이전 실행의 소유 process group만 정리하고 종료 |
| `--dry-run` | 꺼짐 | config와 randomized world만 생성·출력하고 simulator·policy·rosbag은 실행하지 않음 |

## 데이터셋

```text
ws_aic/data/img2pos/<version>/
├── data.yaml
├── metadata.jsonl
├── samples.jsonl
└── images/
    ├── train/<left|center|right>/trial_<index>/*.jpg
    ├── val/<left|center|right>/trial_<index>/*.jpg
    └── test/<left|center|right>/trial_<index>/*.jpg
```

JPEG 파일명은
`<connector>_card_<rail 순서 card mask>_rail<target rail>[_port<port>]_num<sample>_<camera>.jpg`
형식입니다. task ID의 bitmask는 오른쪽부터 rail 0이지만 파일명은 왼쪽부터 rail 0으로
읽을 수 있도록 문자열을 뒤집습니다. 따라서 task ID의 `cards10100`은 파일명에서
`card_00101`로 표시됩니다. 해당 trial에서 SFP rail 4의 port 0을 찍은 첫 center image는
`images/train/center/trial_000/sfp_card_00101_rail4_port0_num001_center.jpg`로
저장됩니다. SC도 같은 규칙을 사용하며 `cards10`은 `card_01`로 표시합니다.

`metadata.jsonl`은 수집 실행마다 한 row를 추가합니다.

| 필드 | 의미 |
| --- | --- |
| `seed` | 실행 시작 시 자동 생성된 master seed |
| `trials` | 실행한 전체 trial 수 |

`samples.jsonl`의 한 row는 동일한 `capture_stamp_ns`를 공유하는 동기화 capture 하나에 대응합니다.

| 필드 | 의미 |
| --- | --- |
| `id` | 동기화 capture 식별자 |
| `trial_id`, `split` | trial 식별자와 trial 단위 train/val/test 분할 |
| `images`, `connector` | 항상 `left`, `center`, `right`를 모두 갖는 JPEG 상대 경로 mapping과 connector 구분 |
| `collection_policy` | `board-view`, `descent`, `near-port` 수집 구간 |
| `target_xyz_m` | 공통 촬영 시점 `base_link`의 `port_entrance - plug_reference` correction |
| `sampling_offset_xyz_m` | 최소 안전거리를 제외한 실제 port-local XYZ; tier 분포 감사용 |
| `sampling_tier_mm` | near-port capture의 coarse/near tier; 다른 정책은 `null` |
| `view_distance_m` | 촬영 시점 plug와 port 사이 접근축 거리 |
| `capture_stamp_ns` | 묶인 camera images의 공통 촬영 ROS 시각 |
| `max_sync_skew_ns` | 승인 source들의 최대 시각 차이 |
| `settle_*` | 촬영 전 연속 controller frame 간 최종 TCP 이동량과 대기시간 |

같은 trial의 모든 capture는 같은 split에 배정됩니다. 기본 34 trial은 정확히
24 train, 5 validation, 5 test로 배정됩니다. command pose, RPY,
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
| `PortOffsetCollect.insert_cable()` | 선택 정책에 맞춰 lift-up, approach, collect 단계 구성 |
| `motion.build_samples()` | 정책별 거리·위치·각도 분포 생성 |
| `motion.collect()` | pose 이동과 공통 capture 시점 TF 조회 |
| `motion.wait_for_pose_convergence()` | controller reference와 실제 TCP 움직임 정지 판정 |
| `dataset.wait_for_observation()` | 현재 명령보다 새로운 동기화 Observation 선택 |
| `dataset.target_xyz()` | 촬영 시점의 XYZ correction 계산 |
| `dataset.save_sample()` | 가시성 검사 후 JPEG와 compact JSONL 기록 |
| `evaluation.summarize_dataset()` | trial split 누수와 실제 label 분포 감사 |
