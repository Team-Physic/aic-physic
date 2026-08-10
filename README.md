# AIC Physic

Team Physic의 [AI for Industry Challenge](https://github.com/intrinsic-dev/aic) 프로젝트입니다.

AI for Industry Challenge는 시뮬레이션 환경에서 UR5e 로봇 팔이 케이블을 지정된
포트에 삽입하는 산업 자동화 태스크입니다. 참가자는 카메라 관측, 로봇 상태,
힘/토크 센서 정보를 이용해 포트 위치와 자세를 추정하고 삽입 정책을 구현합니다.

현재 이 저장소에는 공식 AIC Toolkit과, 포트 주변의 XYZ/RPY offset 학습 데이터를
수집하는 `PortOffsetCollect` 정책이 포함되어 있습니다.

## 주요 기능

- 포트 기준 XYZ/RPY offset을 계층화 샘플링해 로봇 목표 자세 생성
- 좌·중앙·우 카메라와 controller state의 ROS timestamp 동기화 검사
- 캡처 시점의 TF를 조회해 영상과 ground-truth label의 시각 일치 보장
- 포트가 기본 2개 이상의 카메라에 보이는 sample만 저장
- train/validation 분할과 카메라별 이미지·JSON metadata 생성
- 선택적 Hugging Face Dataset 업로드

## 저장소 구조

| 경로 | 역할 |
| --- | --- |
| `ws_aic/src/aic/` | 공식 [`intrinsic-dev/aic`](https://github.com/intrinsic-dev/aic) 서브모듈 |
| `ws_aic/src/phy/phy_policy/data_generator/` | `PortOffsetCollect` 데이터 수집 정책 |
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

### 1. 저장소와 공식 Toolkit 받기

```bash
git clone --recurse-submodules \
  https://github.com/Team-Physic/aic-physic.git
cd aic-physic
```

이미 서브모듈 없이 clone했다면 다음 명령으로 공식 Toolkit을 받습니다.

```bash
git submodule update --init --recursive
```

### 2. Pixi 환경 설치

```bash
pixi self-update --version 0.67.2
cd ws_aic/src/aic
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

`PortOffsetCollect`는 ground-truth TF를 이용하므로 eval 환경에서
`ground_truth:=true`가 필요합니다.

### Terminal 1: 시뮬레이터와 평가 엔진

```bash
export DBX_CONTAINER_MANAGER=docker

distrobox enter -r aic_eval_physic -- /entrypoint.sh \
  ground_truth:=true \
  start_aic_engine:=true
```

### Terminal 2: 데이터 수집 정책

```bash
cd aic-physic/ws_aic/src/aic

export PYTHONPATH="$PWD/../phy/phy_policy/data_generator${PYTHONPATH:+:$PYTHONPATH}"
export AIC_RPY_DATASET_VERSION=trial-001
export AIC_VISION_OFFSET_PUSH_TO_HUB=0

pixi run --frozen ros2 run aic_model aic_model \
  --ros-args \
  -p use_sim_time:=true \
  -p policy:=data_generator.PortOffsetCollect
```

로컬 수집만 할 때는 의도하지 않은 외부 업로드를 막기 위해
`AIC_VISION_OFFSET_PUSH_TO_HUB=0`을 명시하는 것을 권장합니다.

## 주요 환경변수

| 환경변수 | 기본값 | 설명 |
| --- | --- | --- |
| `AIC_COLLECT_STEPS` | `1000` | trial당 offset sample 수 |
| `AIC_RPY_DATASET_VERSION` | 빈 문자열 | 데이터셋 하위 버전 디렉터리 |
| `AIC_VISION_OFFSET_DATASET_DIR` | `ws_aic/data/img2pos[/<version>]` | 데이터셋 출력 위치 |
| `AIC_RPY_MIN_VISIBLE_CAMERAS` | `2` | 저장에 필요한 최소 카메라 수 |
| `AIC_RPY_VISIBILITY_MARGIN_PX` | `64.0` | 영상 경계로부터 필요한 여백 |
| `AIC_COLLECT_SYNC_TOLERANCE_MS` | `30.0` | camera/controller/TF 허용 시각 차이 |
| `AIC_RPY_RANDOMIZATION_VAL_RATIO` | `0.3` | validation trial 비율 |
| `AIC_COLLECT_RANDOM_SEED` | 미지정 | offset 샘플링 재현용 seed |
| `AIC_VISION_OFFSET_PUSH_TO_HUB` | `true` | 수집 완료 후 Hugging Face 업로드 여부 |
| `AIC_VISION_OFFSET_REPO_ID` | 빈 문자열 | 업로드할 Hugging Face Dataset 저장소 |

XYZ/RPY 범위는 `AIC_PORT_COLLECT_DX_MIN_MM`처럼
`AIC_PORT_COLLECT_<AXIS>_MIN/MAX_*` 환경변수로 조정할 수 있습니다.

## 데이터셋 구조

기본 출력은 `ws_aic/data/img2pos/<version>/`에 생성됩니다.

```text
img2pos/<version>/
├── data.yaml
├── samples.jsonl
└── images/
    ├── train/<camera>/*.jpg
    └── val/<camera>/*.jpg
```

`samples.jsonl`은 카메라 이미지 한 장당 한 행이며 이미지 경로, camera/connector,
`target_xyz_m`, 촬영 시각과 최대 동기화 오차만 기록합니다. 같은 `trial_id`는 항상
같은 split에 배정되어 train/validation 사이의 trial 누수를 막습니다.

## 테스트

```bash
cd ws_aic/src/aic

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$PWD/../phy/phy_policy/data_generator" \
pixi run --frozen python -m pytest -q \
  ../phy/phy_policy/data_generator/test/test_port_offset_module_layout.py \
  ../phy/phy_policy/data_generator/test/test_port_offset_timestamp_sync.py
```

## 협업 규칙

브랜치 이름과 영어 커밋 메시지 형식은
[`docs/git-conventions.md`](docs/git-conventions.md)를 따릅니다.
