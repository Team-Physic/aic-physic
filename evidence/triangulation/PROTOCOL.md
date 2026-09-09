# 고정 삼각측량 평가 프로토콜

## 범위와 개수

사용자가 확정한 범위는 **정지한 로봇의 YOLO 포함 위치 평가와 계산 경로 검증**이다.
NIC 1~5장 × 목표 SFP_00~SFP_41 10종 × 독립 배치 2회 = **100개 배치 trial**.
목표 카드를 반드시 포함하며, 나머지 활성 레일과 허용 범위 내 카드 위치를 랜덤화한다.
같은 배치에서는 카드·목표를 고정하고 관측 조건을 바꾼다. 단안만 남기는 검은 영상 시험은 제외한다.

| 항목 | 조건 | 상태 |
|---|---|---|
| 각도 | −15°, 0°, +15° | 사용자 확정 |
| 거리 | 중앙 카메라 광학 중심–포트 중심 0.35 / 0.45 / 0.55m | 예비 접촉 시험 후 선택한 기본값 |
| 조명 | world의 각 광원 intensity ×0.7 / ×1.0 / ×1.3 | smoke 후보값 |
| 가시성 | 최소 두 카메라에서 외곽 4점이 영상 안에 있고 가림 없음 | 사용자 확정 |
| 여백 | 외곽 4점에서 화면 경계까지 20px 이상 | smoke 후보값 |
| 가림 검사 | 포트 평면 5×5 표본의 depth, 앞쪽 가림 허용 오차 2mm | 구현 기준, 유한 표본 검사 |
| 자세 수렴 | 목표 TCP 위치 오차 ≤2mm, 회전 오차 ≤1°, 정지 확인 | 구현 기준 |
| 동기화 | RGB 3대와 depth 3대가 같은 촬영 timestamp | 구현 기준 |

관측 평면은 **포트 로컬 XZ**이다. 입구의 외향 법선은 **로컬 −Z**, 양의 각도는 그 법선에서 로컬 +X 방향으로 이동한다.
중앙 카메라의 optical +Z를 목표 중심으로 향하게 하고 optical +Y를 포트 +Y에 맞춰 roll을 고정한다.
이는 로봇 joint 각도나 world yaw와 다르다. 실제 거리·각도와 명령 TCP 행렬을 각 결과에 함께 저장한다.

100배치 × 조명 3종 = **300개 simulator job**. 각 job은 거리 3종 × 각도 3종의 **9개 관측**을 실행한다.
총 **2,700개 예정 관측**이며, 같은 배치의 관측을 서로 독립적인 trial로 세지 않는다.
조명마다 simulator를 재시작하지만 같은 배치 JSON을 사용한다. 조명 외 광원 위치·색·배경은 template 값으로 유지한다.

## 재현성 계약

- seed로 생성한 **모든 카드·보드·로봇·케이블 값과 world XML을 plan.json에 저장**한다. 실행할 때 난수를 다시 뽑지 않는다.
- 목표는 카드 수마다 각 ID를 2회 배정하고 순서를 섞는다. 실패한 목표를 잘 보이는 목표로 바꾸지 않는다.
- 코드·asset·Pixi lock 파일·YOLO 가중치 SHA-256 및 simulator image digest를 확인한다. 실제 container `/ws_aic/install` 파일도 별도로 해시하며, 불일치 시 실행을 거부한다.
- 실행기는 상속받은 `AIC_*` 설정을 제거한 후 계획·CLI·버전 고정 코드의 값만 사용한다. ROS 통신은 `rmw_zenoh_cpp`로 고정한다. 시간 제한 변경도 resume 서명에 포함한다.
- GPU 모델/드라이버 차이에 따른 렌더링·물리·추론 결과의 bit 단위 동일성은 보장하지 않는다. **입력 계획의 동일성과 실행 결과의 추적 가능성**을 보장한다.
- software smoke와 GPU 본평가는 별도 결과다. GI(VCT)는 양쪽에서 동일하게 제거하고 원본 world는 수정하지 않는다.
- RGB CameraInfo와 depth CameraInfo topic을 분리한다. 해상도가 맞지 않는 보정값은 거부한다.
- GT는 명시적인 `..._link_entrance` frame의 static task-board entrance snapshot이다. 일반 port link로 대체하지 않는다. 촬영 전후 pose 일치를 검사하며, 촬영 시각의 카메라 TF와 혼동하지 않는다.
- GT는 촬영 자세 생성·관측 조건 검사·채점에만 사용한다. YOLO 추론에는 RGB 영상만 제공한다.
- 가시성이 좋은 결과만 골라 평가하지 않는다. **관측 적격성은 depth/GT로 판단하고, 적격 관측의 YOLO 미검출도 실패로 집계**한다.

## 실행 파일

| 파일 | 핵심 함수와 책임 |
|---|---|
| `scripts/triangulation_protocol.py` | `make_plan`: 균등 랜덤 배치·world 동결. `cases`: 고정 조건 ID. `look_at_tcp`: 목표 중심 관측 자세. `visibility`: 여백·depth 검사. `job_complete`: 조건 누락·인프라 실패 판정. `configure_environment`: 외부 설정 격리. `digest`/`write_json`: 입력 식별·원자적 기록 |
| `scripts/run_triangulation_protocol.py` | `source_hashes`/`installed_hashes`/`freeze_plan`: 소스·실제 설치 파일·이미지 버전 고정. `launcher_environment`: ROS lock·렌더러 격리. `run_plan`: job 실행·resume·이력 보존. `main`: CLI 검증 |
| `scripts/TriangulationCollect.py` | `matrix`: TF 행렬 변환. `TriangulationCollect.__init__`: 계획·기존 센서 구독. `capture_case`: 도달·정지·동기화·가시성·NPZ 저장. `insert_cable`: 고정 반복·모든 종료 상태 기록, 실제 삽입 성공 반환하지 않음 |
| `scripts/evaluate_triangulation_protocol.py` | `evaluate_case`: 현재 PortVision 추론과 GT 계산 경로 분리. `summary`: 적격 관측의 실패 포함 통계. `main`: 입력 검증·누락 조건 집계·결과 저장. `ReplayLogger`: 추론 로그 인터페이스 |
| `scripts/test_triangulation_protocol.py` | 균등 배정·난수 재현·2,700 ID·조명·look-at·가림·중단 복구 검사 |

기존 `capture_triangulation_cases.py`와 `evaluate_triangulation.py`는 이전 recorded-baseline 재현용이다.
새 프로토콜에는 위 실행 파일을 사용한다. 생산 정책의 검출·삼각측량 코드는 수정하지 않았다.

## 외부 GPU PC 준비

프로젝트 [README](../../README.md)의 Pixi/Distrobox 환경을 준비한다. Pixi는 `ws_aic/src/pixi.lock`을 frozen 설치한다.
simulator는 `latest` 대신 아래 고정 이미지를 사용한다.
이 평가에서는 container에 host Toolkit을 다시 빌드하거나 덮어씌우지 않는다. host 소스와 image 안의 설치 소스가 다를 수 있어 실제 설치 파일 해시를 따로 확인한다.

```bash
export DBX_CONTAINER_MANAGER=docker
IMAGE=ghcr.io/intrinsic-dev/aic/aic_eval@sha256:9aa2ffdbb946d38edde1bac7b5f02a44cfbea26e3b04a9c74e09f14c97472923
docker pull "$IMAGE"
distrobox create -r --nvidia -i "$IMAGE" aic_eval_physic
distrobox enter aic_eval_physic -- nvidia-smi
```

동일 커밋, 완성된 `plan.json`, 기존 `ws_aic/model/best.pt`를 함께 옮긴다. 가중치는 Git ignore 대상이므로 별도로 전달해야 한다.
아래 명령은 **프로젝트 루트**에서 실행한다. `PLAN`은 전달받은 고정 계획의 상대 또는 절대 경로로 지정한다.

```bash
PLAN=evidence/triangulation/protocol/plan.json
PYTHONDONTWRITEBYTECODE=1 ws_aic/src/.pixi/envs/default/bin/python3 \
  scripts/test_triangulation_protocol.py

# 외부 PC에서도 먼저 동일 실행 경로의 작은 smoke를 확인한다.
PYTHONDONTWRITEBYTECODE=1 ws_aic/src/.pixi/envs/default/bin/python3 \
  scripts/run_triangulation_protocol.py run --plan "$PLAN" \
  --output evidence/triangulation/gpu-smoke --renderer gpu --smoke

# 전체 100배치 / 300 simulator job / 2,700관측
PYTHONDONTWRITEBYTECODE=1 ws_aic/src/.pixi/envs/default/bin/python3 \
  scripts/run_triangulation_protocol.py run --plan "$PLAN" \
  --output evidence/triangulation/gpu-full --renderer gpu

# 중단 재개: 계획·출력·job 선택·renderer를 그대로 유지하고 --resume만 추가한다.
PYTHONDONTWRITEBYTECODE=1 ws_aic/src/.pixi/envs/default/bin/python3 \
  scripts/run_triangulation_protocol.py run --plan "$PLAN" \
  --output evidence/triangulation/gpu-full --renderer gpu --resume

# 캡처와 별도로 실제 YOLO를 재실행한다. CPU 사용 시 --device cpu.
PYTHONDONTWRITEBYTECODE=1 ws_aic/src/.pixi/envs/default/bin/python3 \
  scripts/evaluate_triangulation_protocol.py \
  --capture evidence/triangulation/gpu-full \
  --output evidence/triangulation/gpu-full-evaluation --device 0
```

`--job-indices`로 부분 실행할 경우 해당 job 목록을 고정하고 재개 시 동일하게 지정한다.
인프라 실패 job은 새로운 attempt 디렉터리에서 다시 실행하며, 이전 기록은 유지한다.
관측 불충족·미도달 등 정상적으로 기록된 실패는 성공을 얻기 위해 재시도하지 않는다.

## 결과 읽기

- `plan.json`: 100개 배치와 world, 조건, 해시.
- `run.json`: 실행 전에 동결한 전체 예정 조건 목록, renderer·GPU·image 정보.
- `job_NNN/attempt_NNN/`: engine YAML·world·depth 설명·조건별 JSON·NPZ·GT overlay·실행 로그.
- 같은 attempt 안의 `episode_tracking`·`collector_metadata`는 기존 runner의 완료 감시·metadata이며 학습용 분할 평가 결과가 아니다.
- `job_NNN/complete.json`: 해당 job의 최신 시도 결과. 파일 존재만으로 성공을 뜻하지 않는다.
- `capture_summary.json`: 예정·기록·관측 적격 수와 실패 사유.
- 평가 출력 `summary.json`: 적격 관측을 분모로 한 추정률, 성공 추정의 median/P95/max(mm), 계산 전용 오차.
- 평가 출력 `*_yolo.png`: 실제 모델이 그린 BBOX·keypoint. GT overlay와 구분한다.

가시성 검사는 포트 평면의 유한 표본 depth 검사다. 포트 전체의 완벽한 비가림 증명은 아니며, 대표 GT overlay와 YOLO 영상을 함께 검토한다.
테스트용 초기 시도와 본평가는 별도로 보존한다. 삽입 성공률·실물 성능·학습 데이터 독립성은 이 프로토콜의 검증 결과로 주장하지 않는다.

## 현 PC smoke 상태

초기 `smoke-v1`은 software renderer 진행이 너무 느려 중단,
`smoke-v2`는 힘 센서 기준값 대기 시간 부족, `smoke-v3`은 목표 방향 오차 1° 초과로 촬영 거부를 기록했다.
`smoke-v4` 기본 장면은 세 카메라 가시성·동기화 촬영을 통과했으며 YOLO는 미검출이었다.
다중 카드 예비시험에서는 0.25m에서 접촉이 발생했고, 0.45/0.55m를 포함한 적격 6건 중 1건이 YOLO 포함 추정에 성공했다(오차 0.286mm).
최종 plan은 접촉 조건을 제외한 0.35/0.45/0.55m로 동결했다. 최종 smoke는 GPU renderer·CPU/GPU YOLO 평가와 resume를 통과했다.
전체 2,700건 실행과 통계 주장은 외부 GPU PC에서 명령을 실행한 뒤에만 유효하다.
