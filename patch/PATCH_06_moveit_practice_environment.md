# PATCH_06 - MoveIt ROS 2 Jazzy 실습 환경

- 작성일: 2026-08-09
- 브랜치: `feature/data-collection-node`
- 코드 기준: `e7e9342f73be` + working tree
- 범위: MoveIt tutorial용 Docker/Distrobox와 AIC 적용 전 검증
- 구현 상태: 절차 설계 완료, AIC MoveIt configuration 미적용
- 결론: **첫 실습은 별도 Jazzy container의 Panda/tutorial demo로 수행하고, AIC Kilted workspace와 overlay하지 않는다.**

### Why?

MoveIt UI와 planning 개념을 확인하기 위해 AIC controller 전체를 먼저 수정할 필요는 없다. AIC는 ROS 2 Kilted이고 공식 MoveIt tutorial은 Jazzy 환경을 제공하므로, 두 ROS environment를 한 shell에 섞으면 package path와 middleware 상태를 판별하기 어려워진다.

[PATCH_05 - MoveIt 적용 판단](PATCH_05_moveit_application.md)은 AIC에서 MoveIt이 필요한 범위와 controller 경계를 다룬다. 이 문서는 재현 가능한 실습 환경만 다룬다.

### What I Made

- 한 번 확인할 Docker Compose 경로
- 반복 개발할 Distrobox 경로
- RViz marker, Plan, Plan & Execute의 최소 통과 조건
- tutorial 결과를 AIC UR5e에 옮길 때 필요한 작업 범위

### What was problem

현재 AIC에는 `moveit_msgs` dependency가 있지만 이것만으로 `move_group`, SRDF, kinematics, Planning Scene, RViz MotionPlanning plugin과 trajectory execution controller가 생기지 않는다.

| 환경 | 적합한 목적 | 제한 |
|---|---|---|
| Docker tutorial image | RViz와 planning flow 1회 확인 | container 종료 후 수정 내용을 별도 volume 없이 잃을 수 있음 |
| Distrobox `jazzy-release` | package 수정과 반복 실습 | host ROS setup이 유입되지 않도록 전용 home 필요 |
| 현재 AIC Kilted | 최종 UR5e integration | tutorial 검증 전 바로 적용하면 controller 변경 범위가 큼 |

### How it changed

#### 방법 A: 공식 Docker tutorial

새 terminal에서 실행한다. AIC Pixi shell을 source하지 않는다.

```bash
mkdir -p /home/swlinux/.local/share/moveit2-jazzy
cd /home/swlinux/.local/share/moveit2-jazzy

wget -O docker-compose.yml \
  https://raw.githubusercontent.com/moveit/moveit2_tutorials/main/.docker/docker-compose.yml

export XAUTHORITY="${XAUTHORITY:-/home/swlinux/.Xauthority}"
touch "$XAUTHORITY"
xhost +si:localuser:root

DOCKER_IMAGE=main-jazzy-tutorial-source \
docker compose run --rm --name moveit2_jazzy cpu
```

container 안:

```bash
source /opt/ros/jazzy/setup.bash
test -f /root/ws_moveit/install/setup.bash && \
  source /root/ws_moveit/install/setup.bash

export ROS_DOMAIN_ID=91
unset RMW_IMPLEMENTATION

echo "$ROS_DISTRO"
ros2 pkg prefix moveit_ros_move_group
ros2 pkg prefix moveit2_tutorials
ros2 launch moveit2_tutorials demo.launch.py
```

실습 종료 뒤 X 접근을 회수한다.

```bash
xhost -si:localuser:root
```

#### 방법 B: 반복 실습용 Distrobox

```bash
export DBX_CONTAINER_MANAGER=docker
mkdir -p /home/swlinux/.local/share/distrobox/moveit_jazzy_home

distrobox create --pull \
  --name moveit_jazzy \
  --image docker.io/moveit/moveit2:jazzy-release \
  --home /home/swlinux/.local/share/distrobox/moveit_jazzy_home

distrobox enter moveit_jazzy
```

container 안:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=91
unset RMW_IMPLEMENTATION

sudo apt update
sudo apt install -y ros-jazzy-moveit-resources-panda-moveit-config

ros2 pkg prefix moveit_ros_move_group
ros2 pkg prefix moveit_resources_panda_moveit_config
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

MoveIt planning은 GPU가 필수 아니다. RViz software rendering이 병목이고 NVIDIA container runtime이 이미 검증된 경우에만 `--nvidia` container를 별도 이름으로 만든다.

#### 실습 통과 조건

1. `echo "$ROS_DISTRO"`가 `jazzy`.
2. `moveit_ros_move_group` package 경로 출력.
3. RViz에 Panda와 MotionPlanning panel 표시.
4. marker 이동만으로 robot current state는 변하지 않음.
5. `Plan`이 trajectory를 계산·표시.
6. `Plan & Execute`가 simulated joint state를 target으로 이동.
7. collision 또는 unreachable goal에서 planning 실패.

```mermaid
flowchart LR
    A["Jazzy container"] --> B["Panda/tutorial demo"]
    B --> C["RViz goal 지정"]
    C --> D["Plan"]
    D --> E["Plan & Execute"]
    E --> F["AIC UR5e planning-only config"]
```

#### AIC 적용 시 추가 작업

1. AIC UR5e URDF 기준 `aic_moveit_config` 생성.
2. SRDF planning group과 `gripper/tcp` end effector 정의.
3. kinematics와 joint limit 설정.
4. Task Board collision geometry를 Planning Scene에 등록.
5. fake controller로 planning-only 검증.
6. Gazebo execution이 필요할 때 `FollowJointTrajectory` backend 또는 AIC adapter 연결.
7. pre-insertion 이후 기존 Cartesian impedance controller로 전환.

Jazzy tutorial container를 AIC Kilted runtime에 직접 overlay하지 않는다.

### 검증 기준

- Docker 또는 Distrobox 경로 하나에서 Panda demo 실행
- RViz의 `Plan`과 `Plan & Execute` 차이 확인
- `ROS_DOMAIN_ID=91`에서 AIC node가 보이지 않음
- AIC source와 controller가 실습만으로 변경되지 않음

### 참조 자료

| 출처 | 사용 범위 |
|---|---|
| [MoveIt Getting Started](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html) | Jazzy tutorial 설치와 실행 |
| [MoveIt Docker Guide](https://moveit.picknik.ai/main/doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu.html) | 공식 Compose image와 CPU/GPU service |
| [MoveIt Quickstart in RViz](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) | marker, Plan, Plan & Execute |
| [moveit2_tutorials](https://github.com/moveit/moveit2_tutorials) | 공식 실습 source |
| [Panda MoveIt config](https://docs.ros.org/en/jazzy/p/moveit_resources_panda_moveit_config/) | Distrobox binary demo |
