# PATCH_02 - Gazebo Rendering 성능 원인과 해결

- 작성일: 2026-08-09
- 브랜치: `feature/data-collection-node`
- 코드 기준: `e7e9342f73be` + working tree
- 비교 대상: `mobin/docker/compose.nvidia.yaml`, `aic-physic`의 `aic_eval_physic`
- 구현 상태: Distrobox 초기화·PRIME offload 검증 완료, Gazebo RTF benchmark 미수행
- 결론: **`aic_eval_physic`은 기본값에서 Intel HD Graphics 630을 선택하지만, 같은 container에 PRIME offload environment를 전달하면 GTX 1050을 사용한다. Container 재생성은 필요 없다.**

### Why?

`mobin` Gazebo는 같은 host에서 렌더링 렉이 거의 없지만 AIC Gazebo는 느리다. 두 실행은 simulator workload와 GPU 전달 방식이 모두 다르므로 “Gazebo 자체가 느리다” 또는 “container라서 느리다”로 결론 내릴 수 없다.

실제 OpenGL renderer를 먼저 확인해야 한다. GPU 이름이 다르면 scene 최적화 전에 container GPU 전달을 고치는 것이 가장 작은 해결이다.

### What I Made

- 실행 중인 `mobin-sim-1`의 Compose GPU request, environment, OpenGL renderer 확인
- `aic_eval_physic`의 Docker configuration, device 전달, OpenGL renderer 확인
- AIC image를 Docker `--gpus all`로 직접 실행해 NVIDIA runtime 호환성 확인
- AIC world의 GUI/sensor Global Illumination과 camera rendering 부하 확인
- 원인별 확인 명령과 변경 범위가 작은 해결 순서 작성

### What was problem

#### 측정 결과

2026-08-09 같은 host에서 얻은 결과다.

| 검사 | 결과 | 판정 |
|---|---|---|
| Host GPU | `NVIDIA GeForce GTX 1050`, driver `580.173.02`, VRAM 4096 MiB | NVIDIA GPU와 driver 존재 |
| `mobin-sim-1 | glxinfo -B` | `OpenGL renderer string: NVIDIA GeForce GTX 1050/PCIe/SSE2` | NVIDIA rendering 확인 |
| `aic_eval_physic | glxinfo -B` | `OpenGL renderer string: Mesa Intel(R) HD Graphics 630 (KBL GT2)` | environment 미지정 시 Intel rendering |
| `aic_eval_physic | PRIME env + glxinfo -B` | `OpenGL renderer string: NVIDIA GeForce GTX 1050/PCIe/SSE2` | 같은 container에서 NVIDIA rendering 확인 |
| AIC image + `docker run --gpus all` | `NVIDIA GeForce GTX 1050, 580.173.02` | AIC image 자체는 NVIDIA runtime 사용 가능 |

`aic_eval_physic`의 첫 초기화 중에는 `ubuntu(UID 1000)`를 `swlinux`로 변경한다. 사용자 명령이 이 작업보다 먼저 실행되어 `unable to find user swlinux`가 발생했다. 이후 log의 `container_setup_done`, `getent passwd swlinux`, `/etc/passwd.done`을 확인했다.

`-- bash -lc 'glxinfo ... | grep ...'`는 Distrobox 1.7이 최종 command를 `eval`하는 과정에서 pipe와 quote를 다시 해석해 `eval: OpenGL: not found`를 발생시켰다. Renderer 검사에는 중첩 shell이 필요 없으므로 direct command를 사용한다.

실제 Gazebo 실행 중 FPS·RTF는 측정하지 않았으므로 **NVIDIA 선택은 확인했지만 성능 개선량은 아직 미측정**이다.

#### Container 설정 차이

`../../mobin/docker/compose.nvidia.yaml` | [`services.sim` GPU 설정](../../mobin/docker/compose.nvidia.yaml#L10):

```yaml
# ../../mobin/docker/compose.nvidia.yaml | services.sim
services:
  sim:
    gpus: all
    environment:
      LIBGL_ALWAYS_SOFTWARE: "0"
      NVIDIA_DRIVER_CAPABILITIES: graphics,display,utility,compute
      __NV_PRIME_RENDER_OFFLOAD: "1"
      __GLX_VENDOR_LIBRARY_NAME: nvidia
```

`mobin`은 Docker device request와 PRIME offload environment를 모두 명시한다. `docker compose config`에서도 `gpus: count: -1`이 확인됐다.

현재 `aic_eval_physic`은 Distrobox `--nvidia=1` 초기화로 host NVIDIA driver와 `nvidia-smi`를 사용할 수 있다. Docker `HostConfig.DeviceRequests`는 `null`이지만 PRIME environment를 명시하면 NVIDIA OpenGL renderer가 정상 선택된다. 따라서 이 container에는 `--gpus all` 재생성보다 실행 process에 PRIME environment를 전달하는 변경이 작다.

[Gazebo 공식 troubleshooting](https://gazebosim.org/docs/latest/troubleshooting/)도 Intel/NVIDIA hybrid system에서 Gazebo가 Intel GPU를 선택할 수 있으며 `__NV_PRIME_RENDER_OFFLOAD=1`, `__GLX_VENDOR_LIBRARY_NAME=nvidia`로 NVIDIA offload를 선택하라고 설명한다.

[NVIDIA Container Toolkit 문서](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)는 Docker runtime을 `nvidia-ctk`로 구성하고 GPU를 container에 명시적으로 요청하는 절차를 제공한다. [NVIDIA driver capability 문서](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/1.9.0/user-guide.html#driver-capabilities)는 OpenGL에 `graphics`, X11에 `display`, `nvidia-smi`에 `utility` capability가 필요하다고 정의한다.

#### AIC scene 부하 차이

| 파일 위치 | 설정 | 현재 동작 |
|---|---|---|
| [`ws_aic/src/aic/aic_description/world/aic.sdf`](../ws_aic/src/aic/aic_description/world/aic.sdf#L35) | GUI `GlobalIlluminationVct` | `256^3` voxel, anisotropic, bounce 3의 VCT GI를 GUI scene에 사용 |
| [`ws_aic/src/aic/aic_description/world/aic.sdf`](../ws_aic/src/aic/aic_description/world/aic.sdf#L104) | sensor `global_illumination` | OGRE2 sensor renderer에도 VCT GI를 활성화 |
| [`ws_aic/src/aic/aic_description/urdf/ur_gz.urdf.xacro`](../ws_aic/src/aic/aic_description/urdf/ur_gz.urdf.xacro#L146) | camera instances | left·center·right camera 3개 생성 |
| [`ws_aic/src/aic/aic_assets/models/Basler Camera/basler_camera_macro.xacro`](<../ws_aic/src/aic/aic_assets/models/Basler Camera/basler_camera_macro.xacro#L85>) | camera sensor | camera당 `1152×1024`, RGB, 20 Hz rendering |

`ws_aic/src/aic/aic_description/urdf/ur_gz.urdf.xacro` | [camera instances](../ws_aic/src/aic/aic_description/urdf/ur_gz.urdf.xacro#L146)와 `ws_aic/src/aic/aic_assets/models/Basler Camera/basler_camera_macro.xacro` | [camera sensor 설정](<../ws_aic/src/aic/aic_assets/models/Basler Camera/basler_camera_macro.xacro#L85>)을 사용한 raw pixel rendering rate 계산:

$$
N_{pixel/s}
=
3\times1152\times1024\times20
=
70{,}778{,}880\;\text{pixel/s}
$$

`3`은 camera 수, `1152×1024`는 frame당 pixel 수, `20`은 camera별 frame rate(Hz)다. 결과 단위는 pixel/s다. 이는 compression·transport 전 camera image pixel 수만 센 값이다. GUI view, GI voxel update, shading, contact/physics, ROS bridge 비용은 포함하지 않는다. 따라서 같은 GPU를 사용해도 단순 TurtleBot3 world와 AIC world의 FPS·RTF가 같아야 한다는 가정은 성립하지 않는다.

#### 원인 우선순위

1. **확인됨:** AIC 검사 환경이 Intel HD Graphics 630 renderer를 선택함.
2. **확인됨:** `mobin`은 GTX 1050 renderer를 선택함.
3. **확인됨:** AIC는 GUI와 sensor renderer의 VCT GI, camera 3개를 사용함.
4. **확인됨:** 같은 `aic_eval_physic`에서 PRIME environment를 주면 GTX 1050 renderer로 전환됨.
5. **미측정:** NVIDIA 전환 뒤 GTX 1050 4 GB에서 AIC가 목표 RTF 1.0을 지속하는지.
6. **미측정:** 남은 병목이 GPU rendering, physics CPU, camera transport 중 무엇인지.

### How it changed

코드는 변경하지 않았다. 기존 container 초기화를 완료하고 실행 process에 PRIME environment를 전달한다.

#### 1. 첫 Distrobox 초기화 완료

```bash
export DBX_CONTAINER_MANAGER=docker

distrobox enter -r aic_eval_physic
```

첫 진입에서는 다음 작업이 끝날 때까지 terminal을 종료하지 않는다.

- `Container Setup Complete!` 출력
- `New password:`에서 container의 `swlinux` password 설정
- shell prompt 진입

현재 container는 `getent passwd swlinux`와 `/etc/passwd.done`이 이미 확인되어 재생성할 필요가 없다.

#### 2. 진입한 shell에서 NVIDIA renderer 선택

```bash
export LIBGL_ALWAYS_SOFTWARE=0
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

glxinfo -B | grep -E "OpenGL vendor|OpenGL renderer"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
```

통과 조건:

- `OpenGL vendor string: NVIDIA Corporation`
- `OpenGL renderer string: NVIDIA GeForce GTX 1050...`
- `nvidia-smi`가 GPU와 driver를 출력

검증된 실제 출력은 `NVIDIA GeForce GTX 1050/PCIe/SSE2`다.

#### 3. 같은 shell에서 AIC 실행

```bash
/entrypoint.sh \
  ground_truth:=true \
  start_aic_engine:=true \
  gazebo_gui:=true \
  launch_rviz:=false
```

같은 shell에서 `export`한 environment를 `/entrypoint.sh`와 그 child인 `gz sim`이 상속한다.

#### 4. 초기화 이후 host one-liner

```bash
export DBX_CONTAINER_MANAGER=docker

distrobox enter -r aic_eval_physic -- env \
  LIBGL_ALWAYS_SOFTWARE=0 \
  __NV_PRIME_RENDER_OFFLOAD=1 \
  __GLX_VENDOR_LIBRARY_NAME=nvidia \
  /entrypoint.sh \
  ground_truth:=true \
  start_aic_engine:=true \
  gazebo_gui:=true \
  launch_rviz:=false \
  spawn_task_board:=false \
  spawn_cable:=false \
  model_discovery_timeout_seconds:=600 \
  aic_engine_config_file:=/home/swlinux/Desktop/workspace/aic-physic/ws_aic/src/aic/aic_engine/config/eval_config.yaml
```

#### 5. 같은 AIC command로 RTF 비교

Gazebo 실행 중 다른 terminal:

```bash
watch -n 1 nvidia-smi
```

기록 항목:

| 항목 | 의미 |
|---|---|
| OpenGL renderer | 실제 GUI rendering GPU |
| `gz sim` GPU memory/utilization | NVIDIA rendering process 존재 |
| Gazebo RTF | simulation time / wall time |
| GUI on/off RTF 차이 | GUI view의 추가 비용 |
| camera topic rate | sensor renderer가 20 Hz를 유지하는지 |

#### 6. NVIDIA 전환 후에도 느릴 때

완화 순서:

1. `launch_rviz:=false` 유지.
2. dataset 수집에서 GUI가 필요 없으면 `gazebo_gui:=false`로 실행.
3. GUI on/off 모두 느리면 physics·camera sensor 비용을 분리 측정.
4. 마지막으로만 GI를 임시 비활성화해 A/B 비교.

`gazebo_gui:=false`는 GUI window 비용을 제거하지만 simulated camera sensor rendering까지 없애지는 않는다. camera dataset 수집 중 camera sensor를 끄면 목적 자체가 깨진다.

GI 비활성화는 [`aic.sdf`의 GUI GI](../ws_aic/src/aic/aic_description/world/aic.sdf#L35)와 [sensor GI](../ws_aic/src/aic/aic_description/world/aic.sdf#L108)의 `enabled`를 모두 `false`로 바꾸는 실험이다. **Vision dataset의 조명 분포가 달라지므로 영구 해결로 먼저 사용하지 않는다.**

### 검증 기준

1. `container_setup_done`, `getent passwd swlinux`, `/etc/passwd.done`이 확인된다.
2. PRIME environment를 지정한 `glxinfo -B`가 GTX 1050을 출력한다.
3. Gazebo 실행 중 `nvidia-smi`에 `gz sim`이 나타난다.
4. 동일 trial·GUI 옵션에서 Intel/NVIDIA RTF를 각각 기록한다.
5. NVIDIA 전환 뒤 camera 3개 topic rate가 목표 20 Hz에 근접하는지 확인한다.
6. GI 변경 전후 dataset image appearance 차이를 표본 image로 확인한다.
7. NVIDIA 전환만으로 목표 RTF를 만족하면 world·camera 설정은 변경하지 않는다.

### 참조 자료

| 출처 | 사용 범위 |
|---|---|
| [Gazebo Troubleshooting](https://gazebosim.org/docs/latest/troubleshooting/) | Hybrid Intel/NVIDIA GPU와 PRIME render offload |
| [NVIDIA Container Toolkit 설치](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) | Docker NVIDIA runtime 구성 |
| [NVIDIA Driver Capabilities](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/1.9.0/user-guide.html#driver-capabilities) | `graphics`, `display`, `utility` 의미 |
| [AIC Troubleshooting](../ws_aic/src/aic/docs/troubleshooting.md#L1) | AIC의 low RTF, discrete GPU, GI 완화 안내 |
