# Bounding Box Tool

저장된 image와 YOLO-pose bbox·4-keypoint annotation을 검수하고 수정하는 PyQt5
editor입니다. LabelImg와 비슷하게 image 목록, 중앙 canvas, 객체 목록을 한 화면에
배치합니다.

## 실행

`ws_aic/src`에서 실행합니다.

```bash
PIXI_FROZEN=true pixi run view_annotations ../data/img2pos/<version>
```

이 도구는 ROS node가 아니며 ROS graph에 연결하지 않습니다. `PyQt5`, `PyYAML`,
`NumPy`, OpenCV가 설치된 Python 환경에서는 tool directory에서 직접 실행할 수도
있습니다.

```bash
cd tools/bounding_box_tool
python -m bounding_box_tool.main ../../../data/img2pos/<version>
```

경로를 생략하면 빈 editor가 열리며 `Open Folder`로 dataset root, `images/` 아래
directory 또는 단일 image를 선택할 수 있습니다. dataset을 찾으면
`images/<relative>.jpg`에 대응하는 `annotations/<relative>.txt`를 읽고
`yolo_pose.yaml`의 `names`로 `SFP_00` 같은 label을 표시합니다.

## 편집

| 조작 | 기능 |
| --- | --- |
| `Ctrl+O` | directory 열기 |
| `Ctrl+S` | 현재 annotation 저장 |
| `W` / `Add Box` | canvas에서 새 bbox를 drag하고 class 선택 |
| 객체 또는 우측 목록 click | 편집할 객체 선택 |
| bbox 내부 drag | bbox와 4개 keypoint 함께 이동 |
| bbox corner drag | bbox 크기와 keypoint 상대 위치 조절 |
| keypoint drag | 해당 keypoint 위치 수정 |
| `E` / 객체 double click | class 변경 |
| `Delete` | 선택 객체 삭제 |
| `V` / `Auto Visibility` | 현재 image의 robot-arm occlusion 자동 반영 |
| `←` / `A`, `→` / `D` | 이전/다음 image |
| mouse wheel / `Ctrl++`, `Ctrl+-` | 확대·축소 |
| 빈 canvas drag | 확대된 image 이동 |
| `F` | 화면 맞춤 |
| `Show Annotations` | overlay 표시 전환 |

수정 후 title에 `*`가 표시됩니다. 저장하지 않고 다른 image를 열거나 종료하면
Save/Discard/Cancel을 묻습니다. 저장은 기존 17-field YOLO-pose TXT를 원자적으로
교체하며 모든 객체를 삭제하면 빈 annotation 파일을 남깁니다.

## Auto Visibility

`Auto Visibility`는 현재 image 한 장에만 적용되며 바로 저장하지 않습니다. 결과를
검수한 뒤 `Ctrl+S`로 저장합니다.

1. image 아래쪽에 연결된 큰 검은 영역을 robot-arm mask로 추출합니다.
2. 4개 keypoint polygon과 mask가 90% 이상 겹치면 완전 가림으로 판단해 객체를 삭제합니다.
3. 남은 객체에서 mask 안 keypoint는 `visibility=1`, 보이는 keypoint는 `2`로 바꿉니다.

RGB 기반 근사 판정이므로 결과를 반드시 시각적으로 확인해야 합니다. `1`은 점선 원,
`2`는 채워진 원으로 표시됩니다. 수집 프로세스가 같은 annotation 파일을 쓰는 동안에는
editor로 저장하지 말고 수집 완료 후 후처리하십시오.

빈 TXT는 객체가 없는 정상적인 negative sample로 처리합니다. 잘못된 annotation 행은
건너뛰고 오른쪽 warning 영역에 원인을 표시합니다.
