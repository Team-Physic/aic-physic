# Bounding Box Tool

저장된 image와 YOLO-pose bbox·4-keypoint annotation을 확인하는 read-only PyQt5
viewer입니다. LabelImg와 비슷하게 image 목록, 중앙 canvas, 객체 목록을 한 화면에
배치합니다. annotation 생성·수정·저장 기능은 아직 제공하지 않습니다.

## 실행

`ws_aic/src`에서 실행합니다.

```bash
PIXI_FROZEN=true pixi run view_annotations ../data/img2pos/<version>
```

이 viewer는 ROS node가 아니며 ROS graph에 연결하지 않습니다. `PyQt5`와 `PyYAML`이
설치된 Python 환경에서는 tool directory에서 직접 실행할 수도 있습니다.

```bash
cd tools/bounding_box_tool
python -m bounding_box_tool.main ../../../data/img2pos/<version>
```

경로를 생략하면 빈 viewer가 열리며 `Open Folder`로 다음 중 하나를 선택할 수 있습니다.

- `yolo_pose.yaml`과 `images/`가 있는 dataset root
- `images/` 또는 그 아래 split/camera/trial directory
- annotation 없이 image만 들어 있는 일반 directory

dataset을 찾으면 `images/<relative>.jpg`에 대응하는
`annotations/<relative>.txt`를 읽고, `yolo_pose.yaml`의 `names`로 class ID를
`SFP_00` 같은 label로 표시합니다. 빈 TXT는 객체가 없는 정상적인 negative sample로
표시합니다.

## 조작

| 조작 | 기능 |
| --- | --- |
| `Ctrl+O` | directory 열기 |
| `←` / `A` | 이전 image |
| `→` / `D` | 다음 image |
| mouse wheel / `Ctrl++`, `Ctrl+-` | 확대·축소 |
| mouse drag | 확대된 image 이동 |
| `F` | 화면 맞춤 |
| `Show Annotations` | bbox/keypoint overlay 표시 전환 |

왼쪽에는 재귀 탐색한 image 경로, 오른쪽에는 class label을 표시합니다. 객체에 mouse를
올리면 정규화 bbox를 확인할 수 있습니다.
잘못된 annotation 행은 건너뛰고 오른쪽 warning 영역에 원인을 표시합니다.
