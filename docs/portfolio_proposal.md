# 범용 포트폴리오 구성 검토안

- 작성일: 2026-09-07
- 상태: 사용자 검토용. 실제 발표자료 반영 전.
- 기준 소스: [ai-job-search/cv/ppt/presentation.tex](/home/swlinux/Desktop/workspace/ai-job-search/cv/ppt/presentation.tex)
- 기준 시각 자료: [presentation.pdf](/home/swlinux/Desktop/workspace/ai-job-search/cv/ppt/presentation.pdf)
- 기존 빌드 결과: [presentation_tex.pdf](/home/swlinux/Desktop/workspace/ai-job-search/cv/ppt/presentation_tex.pdf)
- 합의된 방향: 로봇 perception·시뮬레이션 직무를 지향하는 범용 포트폴리오. Mobin을 센서 융합 프로젝트로 포함한다.
- 이번 범위: 전체 순서·분량·중복 조정안과 원고. 실제 LaTeX·PDF·PPTX 및 구현 코드는 변경하지 않는다.
- 참조 형식: 해당 프로젝트의 공통 발표자료는 PPTX가 아닌 Beamer LaTeX와 PDF다. 원본 PDF의 이미지 영역을 LaTeX에서 잘라 재사용하는 구조이므로 실제 반영 시 원본 PDF를 덮어쓰지 않는다.
- 검토 기준: 15개 LaTeX frame 전체와 관련 PDF 텍스트, AIC 코드·작성자 커밋, 기존 포트폴리오 검증 계획. 이번 작업은 내용 검토이며 전체 시각적 레이아웃 검증은 수행하지 않았다.

## 1. 기준 자료의 현재 구성

| 기존 페이지 | 현재 내용 | 반영 제안 |
|---|---|---|
| 1 | 표지 | 공통 포트폴리오 표지 유지. 특정 기업명은 넣지 않음 |
| 2–3 | 자기소개·기술 스택 | 유지. 중복 자격 정보와 서로 다른 취득일은 후보자 프로필에 맞춰 정정 |
| 4 | 프로젝트 목차 | Mobin 추가 |
| 5–6 | AIC 개요·랜덤화 데이터 수집 | 기존 모델 개발 기여 보존. 최근 병렬 수집 개선과 기여 범위 보강 |
| 7 | AIC 삼각측량과 MAE 약 5mm 성과 | 기존 자료 보존. 실험 조건과 현재 실행 정책의 평가를 구분 |
| 8–9 | AIC 회귀 모델 구조·결과 | 그대로 유지. Worst Case 비교라는 조건을 생략하지 않음 |
| 10–12 | RoboDine 개요·자세 추정·센서 융합 회피 | 3장 모두 유지. Mobin과 다른 프로젝트로 구분 |
| 13–15 | 구조물 안정성 개요·모델·실험 | 3장 모두 유지. 비교 기준별 성과를 구분 |

현재 기준 자료에는 Q&A 프로젝트가 없다. 이전 Job-Hunting 파일에서 도출한 Q&A 재배치와 RoboDine 개요 오류 정정안은 적용하지 않는다.

RoboDine에도 센서 융합 사례가 이미 있다. [기존 원고 245행](/home/swlinux/Desktop/workspace/ai-job-search/cv/ppt/presentation.tex:245)은 “단안 카메라 및 2D LiDAR 거리 정보를 융합한 ROS2 기반 장애물 인식 및 회피 노드 설계”라고 기재한다. 이는 기존 발표자료의 경험 서술이며 이번 작업에서 해당 프로젝트 코드를 새로 검증한 결과는 아니다. 사용자가 확인한 Mobin을 이 사례로 대체하거나 두 프로젝트의 성과를 합치지 않는다.

## 2. 추천 순서와 분량

**기존 15장을 유지·보강하고 AIC 2장, Mobin 3장을 추가한 총 20장**을 추천한다. 새 목차·자기소개 페이지를 추가하지 않고 기존 페이지를 갱신한다.

| 새 페이지 | 주제 | 처리 |
|---|---|---|
| 1–3 | 표지·자기소개·기술 스택 | 기존 1–3장 유지 |
| 4 | 프로젝트 목차 | AIC, Mobin, RoboDine, 구조물 안정성 순서로 갱신 |
| 5–6 | AIC 개요·데이터 수집 | 기존 5–6장에 최근 기여 반영 |
| 7–9 | AIC 삼각측량·회귀 모델·기존 실험 결과 | 기존 7–9장 보존 |
| 10 | AIC 목표 추적·접근 | 최근 구현 추가 |
| 11 | AIC 재식별과 검증 범위 | ReID 통합 추가 |
| 12 | Mobin 개요 | 목적·본인 기여·현재 범위 |
| 13 | Mobin 센서 융합 사례 | 확인된 구현과 실험 자료 중심 |
| 14 | Mobin 평가와 한계 | 기존 실행 결과와 미완료 항목 구분 |
| 15–17 | RoboDine 개요·자세 추정·센서 융합 회피 | 기존 10–12장 이동·보존 |
| 18–20 | 구조물 안정성 개요·모델·실험 | 기존 13–15장 이동·보존 |

추가 분량을 줄여야 할 경우 ReID 상세를 부록으로 옮길 수 있다. 기존 실험 결과를 삭제해 공간을 확보하지 않는다. Q&A는 이번 기준 자료에 없으므로 새로 끌어오지 않는다.

## 3. 프로젝트별 강조점과 중복 처리

| 프로젝트 | 강조점 | 중복 설명 처리 |
|---|---|---|
| AIC | 시뮬레이션 데이터, 포트 위치 추정과 학습 모델, 최근 추적·접근 구현 | 기존 모델 성과와 현재 정책 실행 검증을 구분 |
| Mobin | 센서 융합 구현과 실험 결과를 통한 한계 진단 | RoboDine과 별개 프로젝트로 소개. 안정적 자율주행 성과로 확대하지 않음 |
| RoboDine | 기존 실물 로봇 자세 추정·센서 기반 장애물 대응 사례 | 기존 3장을 보존하고 Mobin의 결과·환경을 섞지 않음 |
| 구조물 안정성 | 학습 모델 설계와 비교 실험 | 학습 특징 결합을 로봇 센서 융합과 같은 성과로 표현하지 않음 |

기술 원리 설명은 필요한 프로젝트에 한 번만 충분히 제시하고, 다른 프로젝트에서는 입력·해결한 문제·결과의 차이를 설명한다. 실제 슬라이드 제작 시 공간 관계와 데이터 흐름 설명에는 해당 시각화 skill을 사용한다. 현재는 구성·원고 검토 단계이므로 새 기술 다이어그램을 만들지 않는다.

## 4. AIC 보강 원고

기존 7–9장의 실험·모델 자료를 보존하면서 다음 원고를 5–6장과 새 10–11장에 반영한다.


아래 원고는 코드로 확인한 구현 범위를 중심으로 작성했다. 처리량·정확도·성공률 개선을 새로 주장하지 않는다.

### 5장: AI for Industry Challenge

**부제**: 산업용 케이블 삽입 과제의 시각 인식과 시뮬레이션 실험

- 과제: 시뮬레이션에서 목표 포트를 인식하고 UR5e 로봇의 케이블 삽입을 수행하는 산업 자동화 과제.
- 담당 기여: 데이터 수집의 병렬 실행 개선, 목표 포트 추적·접근 정책, appearance ReID 통합.
- 현재 구현 범위: 목표 포트 탐색과 접근. 삽입 완료 및 전체 과제 성공률은 별도 검증 대상.

발표 메모: 공식 AIC Toolkit을 기반으로 한 팀 프로젝트이며, 공통 기반과 본인이 수정한 부분을 구분한다. 기존 5–9장의 모델 개발·실험 성과와 최근 추적·ReID 통합 기여를 함께 소개한다. 모델 오프라인 평가와 정책 실행 성과를 구분한다.

### 6장: 반복 실험을 위한 시뮬레이션 데이터 수집

- 문제: 카드 배치와 포트 위치가 달라지는 조건에서 학습·검증용 데이터를 반복 수집해야 했다.
- 본인 기여: 수집 작업의 병렬 실행과 카드 배치 randomization, 포트 가시성 판정을 개선했다.
- 구현 결과: worker별 Gazebo·ROS·Zenoh 설정을 분리해 수집 작업을 실행하는 구조를 구성했다.
- 검증 범위: 병렬 실행 구현과 작성자 커밋 확인. 실제 수집 시간 단축률은 미측정 상태로 표기한다.

자료 배치안: 실제 수집 이미지와 실행 조건을 함께 제시한다. 이미 존재하는 분포 그림은 생성 조건과 해당 버전의 일치를 확인한 뒤 사용한다. 팀원이 구현한 trial split까지 본인 단독 설계로 포함하지 않는다.

### 10장: 목표 포트 추적과 로봇 접근

- 문제: 로봇이 움직이는 동안 목표 포트의 관측이 변하므로 검출 결과의 유효성을 확인해야 했다.
- 본인 기여: 기존 위치 추정에 KLT 추적을 결합하고 목표 포트를 이용하는 접근 정책을 개발했다.
- 구현 결과: 추적 결과의 재투영 오차와 위치 변화에 제한을 두고 유효한 추정만 사용하도록 했다.
- 검증 범위: 추적·접근 로직 구현 확인. 현재 실행 함수의 완료 지점은 접근 완료다.

자료 배치안: 현재 코드 버전과 대응하는 목표 추적 화면을 사용한다. 기존 GIF는 팀 공용 시연 자료 후보이며, 내용·촬영 조건 확인 전 개인 성과나 삽입 성공 증거로 사용하지 않는다.

### 11장: 가림 이후 목표 재식별

- 문제: 목표를 다시 관측할 때 같은 종류의 다른 포트와 구분할 기준이 필요했다.
- 본인 기여: appearance descriptor와 target memory를 비교하는 ReID를 기존 후보 선택에 통합했다.
- 구현 결과: 유사도 점수가 기준에 못 미치는 후보를 거부하는 판단을 추가했다.
- 남은 검증: 실제 영상에서 threshold 보정과 재식별 성공률, ID 전환, 지연시간 측정.

발표 메모: ReID 정확도 향상이나 실시간 처리 달성으로 표현하지 않는다. 구현과 측정 완료를 구분하며, 신규 개발을 약속하는 일정표는 넣지 않는다.

## 5. AIC 구현 근거


이 절은 원고 검토용이다. 코드 전문이나 커밋 목록을 PPT 본문에 그대로 붙이지 않는다.

### 병렬 수집

- 본인 커밋: `db81c8b`, 작성자 `JungSeong <jungseonglian@sju.ac.kr>`.
- [worker 설정](/home/swlinux/Desktop/workspace/aic-physic/ws_aic/src/phy/phy_data_collection/phy_data_collection/runner/main.py:811), [실행 환경 반영](/home/swlinux/Desktop/workspace/aic-physic/ws_aic/src/phy/phy_data_collection/phy_data_collection/runner/runtime.py:108).

```python
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["GZ_PARTITION"] = partition
    env["IGN_PARTITION"] = partition
    env["ZENOH_CONFIG_OVERRIDE"] = _peer_zenoh_override(zenoh_port)
```

### 목표 추적

- 본인 커밋: `749a2c3`, 작성자 `JungSeong <jungseonglian@sju.ac.kr>`.
- [PortVision.track()](/home/swlinux/Desktop/workspace/aic-physic/ws_aic/src/phy/phy_policy/phy_policy/ros/final_policy_vision.py:529)의 추적 결과 검사.

```python
            projection_error = float(np.linalg.norm(flow_center - projected))
            if projection_error > self.track_reprojection_max_px:
                continue
```

중간 코드 생략. 이후 3D 위치 변화도 검사한다.

```python
        if float(np.linalg.norm(current.xyz - previous.xyz)) > self.track_max_3d_jump_m:
            return None
```

### 재식별

- 본인 커밋: `c5f25ce`, 작성자 `JungSeong <jungseonglian@sju.ac.kr>`.
- [PortVision.estimate()](/home/swlinux/Desktop/workspace/aic-physic/ws_aic/src/phy/phy_policy/phy_policy/ros/final_policy_vision.py:514)에서 생성한 후보를 `return self.reid.select(candidates)`로 전달한다.
- [AppearanceReID.select()](/home/swlinux/Desktop/workspace/aic-physic/ws_aic/src/phy/phy_policy/phy_policy/ros/final_policy_reid.py:457)의 판단.

```python
        score, candidate = max(scored, key=lambda item: item[0])
        if score < self.match_threshold:
            return None
```

### 완료 범위

[FinalPolicy.insert_cable()](/home/swlinux/Desktop/workspace/aic-physic/ws_aic/src/phy/phy_policy/phy_policy/ros/FinalPolicy.py:507)의 마지막 경로.

```python
        send_feedback("FinalPolicy: target_locked_approach")
        if not self._stage_approach(vision, get_observation, move_robot):
            return False
        send_feedback("FinalPolicy: approach complete")
        return True
```

함수 이름과 반환값만으로 실제 케이블 삽입 성공을 주장하지 않는다. 위 항목은 코드 확인이며, 이번 작업에서 ROS/Gazebo 실행이나 기존 테스트의 재실행은 하지 않았다.

## 6. Mobin 3장 원고 초안

사용자가 센서 융합 프로젝트를 Mobin으로 확인했다. 아래는 문장 방향 초안이며, 본문에 넣을 상세 기술 설명과 영상은 Mobin의 현재 구현·실험 버전에 맞춰 최종 대조한다.

### 12장: Mobin

- 프로젝트 주제: 로봇의 장애물 인식을 위한 센서 융합과 주행 판단 개발.
- 소개할 기여: 센서 관측을 연결하는 구현과 반복 가능한 실험·평가 자료 정리.
- 현재 범위: 구현 항목과 실행 평가의 실패·미완료 항목을 함께 제시.

### 13장: 센서 융합 구현

- 해결한 문제와 실제 입력 센서를 먼저 제시한다.
- 본인이 수정한 구현 범위와 대표 관측 결과를 나란히 보여준다.
- RoboDine의 기존 2D LiDAR 회피 사례와 데이터·기여·평가 결과를 구분한다.

### 14장: 실행 평가와 한계

- 하나의 실행 결과에 대해 조건, 관측 결과, 남은 문제를 함께 설명한다.
- [seeded-multi-person-tracking/summary.json](/home/swlinux/Desktop/workspace/mobin/trials/seeded-multi-person-tracking/summary.json)의 실제 기록은 다음과 같다.

```json
  "patrol_completed": false,
  "patrol_status": "not_started",
```

이 기록은 해당 실행에서 patrol 완료가 확인되지 않았음을 보여준다. 프로젝트 전체의 모든 실행을 대표한다고 단정하지 않으며, 완주나 안정적 자율주행 달성으로 소개하지 않는다. 새로운 개발·평가 일정을 포트폴리오 제작의 선행 조건으로 두지 않는다.

## 7. 성과와 참고 자료의 사용 기준

| 항목 | 현재 근거 | 처리 |
|---|---|---|
| AIC MAE 약 5mm | 기준 발표자료 7장 | 기존 성과 보존. 현 FinalPolicy의 신규 E2E 측정 결과로 재해석하지 않음 |
| AIC XYZ 21.3%·RPY 16.5% 개선 | 기준 발표자료 8–9장 | 모델 구조·실험 표 보존. Worst Case 비교라는 원래 조건 유지 |
| 최근 AIC 추적·ReID | 현재 코드·본인 커밋 | 구현 기여로 추가. 정확도·속도 개선 수치 추가 금지 |
| RoboDine 14% | 기준 발표자료 10–11장 | 기존 표의 단위·비교 기준을 확인해 표현. 정확도 증가와 위치 오차 감소를 혼용하지 않음 |
| RoboDine 센서 융합 | 기준 발표자료 10·12장 | 기존 내용 보존. Mobin으로 이동하거나 이름 변경하지 않음 |
| 구조물 안정성 개선율 | 기준 발표자료 13–15장 | 기존 결과와 비교 기준 보존. Best/Worst trial 비교를 baseline 개선과 혼용하지 않음 |
| AIC·Mobin 검증 일정 | ai-job-search의 2026-09-01 계획 문서 | 계획은 완료 증거가 아님. 이번 작업에서 실행하거나 미래 개발 일정으로 재약속하지 않음 |

기존 [실행 및 증거 계획](/home/swlinux/Desktop/workspace/ai-job-search/docs/portfolio-validation/aic-mobin-execution-and-evidence-plan.md)과 [용어집](/home/swlinux/Desktop/workspace/ai-job-search/docs/portfolio-validation/glossary.md)을 재사용한다. 별도 ADR이나 중복 용어집은 만들지 않는다.

## 8. 다음 검토

- 기준 자료 변경과 Mobin 프로젝트 식별은 사용자 지시로 확정했다.
- 총 20장 구성안과 보강 원고는 아직 실제 발표자료에 반영하지 않았다.
- 사용자 수락 이후 기존 Beamer 편집 형식을 유지해 반영하고 PDF를 생성·검증하는 방식을 기본으로 한다. PPTX 변환은 별도 요청이 있을 때 수행한다.
- 원본 presentation.pdf는 이미지 참조 자산이므로 보존하고 결과 PDF는 별도 파일로 생성한다.
