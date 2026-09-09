# 삼각측량 평가 파일 안내

현재 평가 기준과 외부 GPU PC 실행 방법은 [고정 프로토콜](PROTOCOL.md)에 있다.
NIC 1~5장, 100개 배치, 2,700개 예정 관측의 계획·촬영·재평가 스크립트를 추가했다.
아래 recorded-baseline은 이전 분석 이력이며 새 평가 결과에 합산하지 않는다.
기존 추론 정책의 로직과 모델 가중치는 수정하지 않았다.

## 실행 파일

| 파일 | 역할 |
|---|---|
| [evaluate_triangulation.py](../../scripts/evaluate_triangulation.py) | 저장된 RGB·CameraInfo·TF를 현재 PortVision으로 재평가하고 정답 포트 입구 위치와 비교 |
| [capture_triangulation_cases.py](../../scripts/capture_triangulation_cases.py) | 기존 PHY collector를 재사용해 카드 수 1/3/5와 seed가 고정된 추가 Gazebo 사례 수집 |
| [test_evaluate_triangulation.py](../../scripts/test_evaluate_triangulation.py) | 미검출을 분모에서 누락하지 않는지, 정답 부재와 알고리즘 실패를 구분하는지 검사 |
| [test_capture_triangulation_cases.py](../../scripts/test_capture_triangulation_cases.py) | 정확한 카드 수, 같은 seed의 재현성, 생성 world의 VCT 제거 검사 |

## 기존 데이터 재평가

프로젝트 루트에서 다음 명령을 실행한다. 원래 rosbag과 `ws_aic/model/best.pt`가 필요하다.
출력은 기존 결과와 다른 디렉터리로 지정해 비교한다.

```bash
PYTHONDONTWRITEBYTECODE=1 ws_aic/src/.pixi/envs/default/bin/python3 \
  scripts/evaluate_triangulation.py \
  --bag rosbags/portoffset/reid-smoke/20260830T154524-140148-w00-140150/trial_0000_portoffset_sfp_0000_cards11001_rail4_sfp_port_0 \
  --bag rosbags/portoffset/reid-smoke/20260830T155623-148200-w00-148202/trial_0000_portoffset_sfp_0000_cards00101_rail0_sfp_port_0 \
  --output evidence/triangulation/recheck \
  --variants original dark bright drop_left drop_center drop_right drop_left_center
```

## 보존한 근거

- `recorded-baseline/summary.json`: 실제 실행 인자, 모델·스크립트·생산 코드·bag 해시, 조건별 요약.
- `recorded-baseline/<trial>/frames.jsonl`: 촬영 시각, 정답·추정 XYZ, 오차(mm), 재투영 오차, 실패 이유.
- 같은 trial 폴더의 PNG·JSON: 변형·성공 여부별 대표 입력 영상과 projection matrix·키포인트.
- `recorded-baseline.log`: 기존 bag 재평가 실행 로그.
- `capture-c1.log`, `capture-c1b.log`: 추가 수집 실행의 상위 로그. 상세 로그는 `ws_aic/logs/data_collection/triangulation-20260909-c1*/`에 있다.

## 결과 해석

| 장면 | 정답을 비교할 수 있는 프레임 | 실제 YOLO 포함 유효 추정 | 유효 추정의 중앙값 / P95 / 최대 오차 |
|---|---:|---:|---|
| NIC 3장, SFP_40 | 22 | 0/22 | 산출 불가 |
| NIC 2장, SFP_00 | 20 | 5/20 | 0.336 / 1.274 / 1.485 mm |

동기화된 영상은 총 351개 묶음이다. 그중 309개는 같은 시각의 정답 TF가 없어 채점에서 제외하고 별도 집계했다.
같은 장면의 연속 프레임이므로 42개의 독립 장면으로 해석하면 안 된다.
정답 2D 좌표를 입력한 계산 경로의 최대 오차는 3.43e-13 mm 미만이었다. 이는 부동소수점 수준의 계산 일관성 검증이며 실제 검출 정확도가 아니다.
미검출은 유효 추정률의 분모에 포함한다. 위치 오차가 작은 5개 프레임만으로 전체 성능이 좋다고 주장할 수 없다.
`dark`·`bright`는 픽셀 배율 변형, `drop_*`는 지정 카메라 전체를 검게 만든 합성 차폐다. 실제 조명 변경·물리 가림 시험과 구분한다.
가중치 학습 데이터와 이 bag의 독립성은 확인하지 못했다. 삽입 성공률과 실물 성능은 이 평가 대상이 아니다.

## 현재 평가 조건

- NIC 1~5장 × 목표 10종 × 독립 배치 2회 = 100개 배치 trial.
- 각 배치에 거리 3종 × 각도 −15°/0°/+15° × 실제 Gazebo 조명 3종 = 27개 관측.
- 최소 두 카메라의 화면 여백·depth 가시성을 확인하고, 적격 관측의 YOLO 미검출도 실패로 집계한다.
- `drop_*` 합성 검은 영상은 새 프로토콜에서 제외한다.
- 모든 랜덤 값·입력 파일·모델 해시를 계획에 동결한다. 성공 사례만 다시 뽑지 않는다.
- 거리·조명·여백 수치와 현재 smoke의 검증 범위는 [PROTOCOL.md](PROTOCOL.md)에 명시한다.

이전 `capture-c1*` 수집 실패는 Gazebo EGL 종료와 ROS controller lock 오류였다.
새 실행기는 trial별 ROS_HOME과 renderer 설정을 격리한다. 이전 실패 로그는 이력으로만 보존한다.
