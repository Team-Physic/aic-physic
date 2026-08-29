# Policy Dashboard

`FinalPolicy`가 발행하는 세 wrist camera의 YOLO pose 결과와 triangulation 좌표를
브라우저에서 실시간으로 확인하는 개발 도구다. ROS 패키지는 아니지만 `rclpy`로
policy의 debug topic을 구독하고, JPEG로 변환해 하나의 HTTP 서버에서 스트리밍한다.
하단에는 EE·cable tip의 RPY/로컬 +Z 방향, wrist force-torque sensor의 최근 15초
시계열, 실제 port와 triangulation 및 EE·cable 궤적을 함께 보는 3D viewer를 표시한다.

data collection runner와 연결하거나 simulation을 등록하지 않는다. 한 번에 하나의
policy 실행을 대상으로 하며, dashboard와 simulator가 같은 ROS domain 및 Zenoh
환경을 사용하면 실행 순서와 관계없이 ROS discovery로 연결된다.

## Topics

- `/final_policy/yolo/left/image`
- `/final_policy/yolo/center/image`
- `/final_policy/yolo/right/image`
- `/final_policy/triangulated_port_xyz`
- `/final_policy/task`
- `/aic_controller/controller_state`
- `/fts_broadcaster/wrench`
- `/tf` (`base_link` 기준 cable tip 방향, development ground truth)

Dashboard가 실행 중일 때만 FinalPolicy가 image subscriber를 감지하고 YOLO overlay를
생성한다.

EE 방향은 `ControllerState.tcp_pose.orientation`을 사용한다. Cable 방향은 현재
`Task`의 `cable_name/plug_name_link` TF를 선택하며, 두 방향 모두 frame의 로컬 `+Z`
축을 `base_link`에 표시한다. Cable TF는 simulator를 `ground_truth:=true`로 실행할
때만 볼 수 있다.

3D position viewer의 실제 port는
`task_board/{target_module_name}/{port_name}_link`, cable은
`{cable_name}/{plug_name}_link` TF를 사용한다. 현재 engine의 한 trial당 한 task
규약에 맞춰 `/final_policy/task`가 새로 발행될 때마다 별도 trial로 구분한다. EE와
cable 궤적은 trial별 탭에 보존되며 최근 24개 trial을 선택해 다시 볼 수 있다. RPY
sphere와 position viewer 모두 마우스 드래그로 회전하고 휠로 확대·축소하며,
더블클릭하면 기본 시점으로 돌아간다.

상단 force/torque 수치는 `/fts_broadcaster/wrench`의 raw 값이므로 payload에 의한
편향을 그대로 보여준다. 15초 그래프의 force는 raw 축값을 사용하고 torque에만 ROS
수신 주기 기준 0.3 Hz low-pass filter를 적용한다. 이 cutoff는 49.6 Hz 안정 구간에서
후보들을 비교해 가장 작은 torque 표준편차를 보인 값이다. Controller tare는 engine이
cable 연결 후 robot이 1초 동안 안정된 다음 수행하며 raw broadcaster 값에는 적용되지
않는다.

## Structure

- `models/`: ROS나 웹 프레임워크에 의존하지 않는 최신 frame·좌표 상태
- `controllers/image.py`: camera topic 구독과 image-to-JPEG 변환
- `controllers/pose.py`: task와 EE pose 및 cable·port TF를 dashboard 상태로 변환
- `controllers/haptic.py`: raw force-torque topic 구독
- `controllers/triangulation.py`: triangulated port topic 구독
- `controllers/node.py`: feature controller 조합과 ROS node 수명주기
- `views/static/rpy_sphere.js`: RPY sphere 렌더링
- `views/static/haptic.js`: low-pass와 force-torque chart 렌더링
- `views/static/coordinate_viewer.js`: port와 motion trajectory 3D 렌더링
- `views/static/orbit_canvas.js`: Canvas drag orbit과 wheel zoom
- `views/`: HTTP API, MJPEG stream, browser UI
- `main.py`: model/controller/view의 생성과 종료 수명주기

## Run

루트 환경에서는 다음 명령으로 실행한다.

```bash
cd ws_aic/src
pixi run --as-is policy_dashboard
```

같은 ROS Python 환경이 이미 활성화되어 있다면 Pixi task 없이도 실행할 수 있다.

```bash
cd ws_aic/src
python tools/policy_dashboard/main.py
```

그다음 <http://127.0.0.1:8080>을 연다. 다른 장비의 브라우저에서 볼 때만 trusted
network에서 외부 접속을 허용한다.

`--as-is`는 이미 설치된 `.pixi` 환경을 그대로 활성화한다. 일반 `pixi run`이나
`--frozen`은 dashboard와 무관한 workspace path package까지 재빌드할 수 있으므로,
이 개발 도구를 실행할 때는 `--as-is`를 사용한다.

```bash
pixi run --as-is policy_dashboard --host 0.0.0.0 --port 8080
```

이 서버에는 인증이나 TLS가 없으므로 public network에 직접 노출하면 안 된다.
Dashboard와 simulator의 `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID`, Zenoh endpoint는
동일해야 한다.

## Options

```text
--host ADDRESS        HTTP bind address (default: 127.0.0.1)
--port PORT           HTTP port (default: 8080)
--jpeg-quality 1..100 JPEG quality (default: 85)
--cable-frame FRAME   Cable tip TF override (default: current task frame)
```

각 값은 `POLICY_DASHBOARD_HOST`, `POLICY_DASHBOARD_PORT`,
`POLICY_DASHBOARD_JPEG_QUALITY`, `POLICY_DASHBOARD_CABLE_FRAME` 환경변수로도
설정할 수 있다. Task frame 대신 첫 SFP trial의 cable frame을 고정하려면 다음처럼
실행한다.

```bash
pixi run --as-is policy_dashboard --cable-frame cable_0/sfp_tip_link
```

FinalPolicy의 YOLO device를 강제로 지정할 때는 policy 프로세스에
`AIC_YOLO_DEVICE=0` 또는 `AIC_YOLO_DEVICE=cpu`를 설정한다.
