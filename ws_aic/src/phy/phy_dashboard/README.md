# PHY Dashboard

`FinalPolicy`가 이미 계산한 Ultralytics YOLO pose 결과를 세 wrist camera에서
실시간으로 확인하는 PyQt dashboard다. Dashboard가 연결되지 않으면 policy는
overlay를 생성하거나 debug image를 복사하지 않는다.

## Topics

- `/final_policy/yolo/left/image`
- `/final_policy/yolo/center/image`
- `/final_policy/yolo/right/image`
- `/final_policy/triangulated_port_xyz`

## Run

일반 AIC simulator와 같은 ROS 환경에서는 다음과 같이 실행한다.

```bash
PIXI_FROZEN=true pixi run phy_dashboard
```

격리된 collection runner의 worker 0(`ROS_DOMAIN_ID=110`, Zenoh port `7610`)에
연결할 때는 다음 환경을 맞춘다.

```bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp \
ROS_DOMAIN_ID=110 \
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/localhost:7610"];transport/shared_memory/enabled=false' \
PIXI_FROZEN=true pixi run phy_dashboard
```

최종 policy 실행에는 GPU 추론을 권장한다.

```bash
export AIC_YOLO_DEVICE=0
```
