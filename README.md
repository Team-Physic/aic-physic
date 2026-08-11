# AIC Physic

Team Physic의 [AI for Industry Challenge](https://github.com/intrinsic-dev/aic) 프로젝트입니다.

AI for Industry Challenge는 시뮬레이션 환경에서 UR5e 로봇 팔이 케이블을 지정된
포트에 삽입하는 산업 자동화 태스크입니다. 참가자는 카메라 관측, 로봇 상태,
힘/토크 센서 정보를 이용해 포트 위치와 자세를 추정하고 삽입 정책을 구현합니다.

현재 이 저장소에는 공식 AIC Toolkit 소스와, 거리 구간별 img2pos 학습 데이터를 자동
수집하는 세 정책 및 공통 runner가 포함되어 있습니다.

## 주요 기능

- 보드 전경, 포트 하강, 포트 근접의 세 수집 정책 선택
- 거리·횡방향 위치·카메라 각도의 stratified random sampling
- 근접 정책에서 50/10/5/2mm tier별 XYZ/RPY offset 집중 샘플링
- plug와 port 사이에 기본 20mm 안전거리를 유지해 접촉 없이 촬영
- controller reference와 실제 TCP 움직임 수렴 뒤 timestamp가 일치하는 카메라 frame 저장
- 좌·중앙·우 카메라와 controller state의 ROS timestamp 동기화 검사
- 동일 촬영시각의 카메라 image들을 한 capture로 묶고 해당 시점 TF로 단일 label 계산
- 포트가 기본 2개 이상의 카메라에 보이는 sample만 저장
- trial 단위 train/validation/test 분할과 compact JPEG·JSONL 생성
- 독립 Zenoh/Gazebo partition을 사용한 headless 병렬 수집
- 선택적 Hugging Face Dataset 업로드

## 저장소 구조

| 경로 | 역할 |
| --- | --- |
| `ws_aic/src/aic/` | 저장소에 포함된 공식 [`intrinsic-dev/aic`](https://github.com/intrinsic-dev/aic) Toolkit 소스 |
| `ws_aic/src/phy/phy_policy/` | 공식 example policy 형식을 따르는 Team Physic 정책 패키지 |
| `ws_aic/src/phy/phy_data_collection/` | randomized trial·rosbag·policy lifecycle runner |
| `ws_aic/data/` | 로컬 데이터셋 기본 출력 위치, Git 추적 제외 |
| `ws_aic/model/` | 로컬 모델 파일 위치, Git 추적 제외 |
| `docs/git-conventions.md` | 브랜치 및 커밋 메시지 규칙 |
| `readme/gif/`, `readme/photo/` | README에 사용할 영상 및 이미지 자산 |

## 요구사항

| 항목 | 요구사항 |
| --- | --- |
| OS | Ubuntu 24.04 |
| ROS 2 | Kilted Kaiju, Pixi 환경에서 제공 |
| Package manager | Pixi `0.67.2` |
| Container | Docker, Distrobox |
| Simulator | Gazebo |
| Middleware | `rmw_zenoh_cpp` |
| GPU | NVIDIA GPU 권장 |

공식 설치 기준은 AIC Toolkit의
[Getting Started](https://github.com/intrinsic-dev/aic/blob/main/docs/getting_started.md)를
참고하십시오.

## 환경 설정

### 1. 저장소 받기

```bash
git clone https://github.com/Team-Physic/aic-physic.git
cd aic-physic
```

공식 Toolkit 소스는 `ws_aic/src/aic/`에 함께 포함되어 있어 별도 submodule 초기화가
필요하지 않습니다.

### 2. Pixi 환경 설치

```bash
pixi self-update --version 0.67.2
cd ws_aic/src
pixi install --frozen
```

### 3. Eval 컨테이너 준비

```bash
export DBX_CONTAINER_MANAGER=docker

docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest
distrobox create -r --nvidia \
  -i ghcr.io/intrinsic-dev/aic/aic_eval:latest \
  aic_eval_physic
```

GPU를 사용하지 않는 환경에서는 `--nvidia`를 제거합니다.

## 데이터 수집 실행

runner가 randomized config/world를 만들고 `ground_truth:=true` simulator, 선택적
rosbag, 선택한 collection policy를 순서대로 시작하고 종료합니다.

```bash
cd aic-physic/ws_aic/src

PIXI_FROZEN=true pixi run ros2 run phy_data_collection \
  collect_portoffset_randomization_data \
  --collection-policy near-port \
  --port-type sfp \
  --trials 34 \
  --workers 2 \
  --samples-per-trial 40 \
  --dataset-version img2pos-v1 \
  --push-to-hub false
```

터미널에는 전체 trial/capture 진행률과 worker별 현재 trial·capture 수·경과시간만
표시됩니다. Gazebo·ROS 상세
출력은 `ws_aic/logs/data_collection/<dataset-version>/<run-id>/` 아래 trial별 파일에
저장됩니다.

전체 CLI와 데이터 형식은
[`phy_data_collection/README.md`](ws_aic/src/phy/phy_data_collection/README.md)를
참고하십시오.

## 주요 환경변수

| 환경변수 | 기본값 | 설명 |
| --- | --- | --- |
| `AIC_COLLECT_STEPS` | runner에서 `40` | trial당 저장할 offset capture 수 |
| `AIC_IMG2POS_COLLECTION_POLICY` | `near-port` | runner가 선택한 수집 정책 기록값 |
| `AIC_IMG2POS_DATASET_VERSION` | 빈 문자열 | 데이터셋 하위 버전 디렉터리 |
| `AIC_IMG2POS_DATASET_DIR` | `ws_aic/data/img2pos[/<version>]` | 데이터셋 출력 위치 |
| `AIC_IMG2POS_MIN_VISIBLE_CAMERAS` | `2` | 저장에 필요한 최소 카메라 수 |
| `AIC_IMG2POS_AUTO_ANNOTATE_PORTS` | `false` | 활성 포트 bbox/4-keypoint YOLO-pose annotation 생성 |
| `AIC_IMG2POS_VISIBILITY_MARGIN_PX` | `64.0` | 영상 경계로부터 필요한 여백 |
| `AIC_COLLECT_SYNC_TOLERANCE_MS` | `30.0` | camera/controller/TF 허용 시각 차이 |
| `AIC_IMG2POS_VAL_RATIO` | `0.15` | validation trial 비율 |
| `AIC_IMG2POS_TEST_RATIO` | `0.15` | test trial 비율 |
| `AIC_COLLECT_RANDOM_SEED` | 미지정 | offset 샘플링 재현용 seed |
| `AIC_COLLECT_SETTLE_TIMEOUT_SEC` | `8.0` | reference와 실제 TCP 움직임 수렴 제한시간 |
| `AIC_PORT_COLLECT_BASE_Z_OFFSET_M` | `0.020` | port 접근축 방향 최소 안전거리 |

XYZ/RPY 범위는 `AIC_PORT_COLLECT_DX_MIN_MM`처럼
`AIC_PORT_COLLECT_<AXIS>_MIN/MAX_*` 환경변수로 조정할 수 있습니다.

## 데이터셋 구조

기본 출력은 `ws_aic/data/img2pos/<version>/`에 생성됩니다.

```text
img2pos/<version>/
├── data.yaml
├── metadata.jsonl
├── samples.jsonl
├── yolo_pose.yaml
├── labels -> annotations
├── images/<train|val|test>/<camera>/trial_<index>/*.jpg
└── annotations/<train|val|test>/<camera>/trial_<index>/*.txt
```

`metadata.jsonl`은 수집 실행마다 자동 생성된 `seed`와 총 `trials`를 한 행으로 기록합니다.
`samples.jsonl`은 동기화된 capture당 한 행이며 `images`에 camera별 JPEG 경로를 묶고,
`collection_policy`, 단일 `target_xyz_m`, 실제 port-local sampling offset과 관측 거리,
sampling tier, 촬영 시각·동기화 오차, pose 수렴 품질을 한 번만 기록합니다.
같은 `trial_id`는 항상
같은 split에 배정되어 train/validation/test 사이의 trial 누수를 막습니다. 수집 완료 후
`collection_summary.json`으로 실제 capture 수를 확인하고, `near-port` capture에 대해서만
tier·안전거리 기준 ±2/5/10/50mm sampling offset 분포를 검증합니다.

`--auto-annotate-ports true`를 지정하면 scene의 활성 포트마다 실제 외곽 4점과 그
min/max bbox를 YOLO-pose 형식으로 함께 저장합니다. 실제 TXT는 `annotations/`에 있고,
Ultralytics가 기본 `labels/` 경로로도 읽을 수 있도록 symlink를 생성합니다. 자세한 class,
keypoint 순서와 치수는
[`phy_data_collection/README.md`](ws_aic/src/phy/phy_data_collection/README.md#데이터셋)를
참고하십시오. SFP label은 0부터 세는 rail/port를 결합한 `SFP_<rail><port>`이며,
예를 들어 rail 4의 port 1은 `SFP_41`입니다.

저장 결과는 PyQt annotation editor로 확인하고 수정할 수 있습니다.

```bash
cd ws_aic/src
PIXI_FROZEN=true pixi run view_annotations ../data/img2pos/<version>
```

폴더 탐색, 객체 추가·삭제, bbox/keypoint 수정, class 변경과 robot-arm occlusion 기반
visibility 후처리를 제공합니다. 자세한 사용법은
[`bounding_box_tool/README.md`](ws_aic/src/tools/bounding_box_tool/README.md)를
참고하십시오.

## 협업 규칙

브랜치 이름과 영어 커밋 메시지 형식은
[`docs/git-conventions.md`](docs/git-conventions.md)를 따릅니다.
