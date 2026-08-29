# PATCH Index

- 최종 수정일: 2026-08-29
- 브랜치: `feature/remind`
- 코드 기준: `ad00899` + working tree
- 결론: **PATCH 번호는 현재 적용 우선순위이며 작성일 순서가 아니다.**

### Why?

기존 `patch/` 문서는 번호 없이 흩어져 있어 선행 문제와 장기 검토 대상을 구분하기 어려웠다. 같은 목적의 문서를 인접 번호로 배치하고, 현재 `feature/approach` 검증을 막는 FinalPolicy의 동기 YOLO·motion 병목을 첫 PATCH로 올렸다. 시야 이탈 후 target identity 복구를 바로 다음 PATCH로 두고, 현재 적용 계획이 없는 Isaac Lab은 마지막이다.

### PATCH 구성

| 묶음 | PATCH | 역할 |
|---|---|---|
| 실행·성능 | [PATCH_00 - FinalPolicy 실행 및 Runtime 성능](PATCH_00_final_policy_runtime_performance.md) | `feature/approach` 실행 순서·검증 범위 명시<br>Live YOLO 0.1 FPS와 motion 저속의 결합 원인 분석<br>background inference·KLT-only 개선안과 통과 기준 제시 |
| Target identity | [PATCH_01 - REMIND ReID Tracker](PATCH_01_remind_reid_tracker.md) | 현재 YOLO Pose detector와 KLT 유지<br>keypoint mask·appearance memory로 재방문 identity 복구<br>Task·3D gate 뒤 보조 ReID 적용 기준 제시 |
| 실행 기반 | [PATCH_02 - Gazebo rendering 성능](PATCH_02_gazebo_rendering_performance.md) | AIC가 Intel renderer를 선택한 원인 확인<br>NVIDIA container 경로와 AIC scene 부하 비교<br>재현·검증·완화 순서 제시 |
| 시간 무결성 | [PATCH_03 - Planned Motion Timestamp](PATCH_03_planned_motion_timestamp.md) | motion 계획 종료시각 정의<br>실제 완료시각과 계획시각 분리<br>dataset 적용 지점 제안 |
| 시간 무결성 | [PATCH_04 - Rerun Timestamp Debugging](PATCH_04_rerun_sensor_timestamp_debugging.md) | camera·controller·TF timeline 시각화<br>MCAP 기반 사후 검사 절차<br>정량 validator와 역할 분리 |
| Motion planning | [PATCH_05 - MoveIt 적용 판단](PATCH_05_moveit_application.md) | AIC 적용 범위와 비용 분석<br>Jacobian·joint-space 판단 정리<br>hybrid controller 경계 제안 |
| Motion planning | [PATCH_06 - MoveIt 실습 환경](PATCH_06_moveit_practice_environment.md) | ROS 2 Jazzy Docker/Distrobox 실습<br>Panda demo 검증 절차<br>AIC Kilted와 환경 분리 |
| Runtime recovery | [PATCH_07 - Behavior Tree 복구](PATCH_07_behavior_tree_perception_recovery.md) | perception failure 분기<br>재관측·재시도·중단 정책<br>현재 project 적용 지점 |
| Runtime recovery | [PATCH_08 - Vision/F-T Align Retry](PATCH_08_visual_force_align_retry.md) | camera·wrench 동기화 조건<br>규칙 기반 lift·재관측·bounded retry<br>학습 모델·실습 자료와 검증 지표 |
| 장기 simulator 검토 | [PATCH_09 - Isaac Lab 적용 판단](PATCH_09_isaac_lab_application.md) | 대규모 병렬 simulation·RL 적합성<br>Gazebo·MoveIt과 역할 비교<br>현재 미적용 판단 |

### 번호 정책

- 새 PATCH는 의존성과 현재 우선순위에 따라 번호를 정한다.
- 같은 문제를 검증·운영하는 문서는 인접 번호로 둔다.
- 한 문서가 독립 실행 목표 두 개를 가지거나 500줄을 크게 넘으면 분리하고 상호 링크한다.
- 적용 생각이 없는 장기 검토는 현재 실행·데이터 문제 뒤에 둔다.
