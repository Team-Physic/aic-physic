# Bounding Box Tool

저장된 image와 YOLO-pose bbox·4-keypoint annotation을 검수하고 수정하는 로컬 웹
editor입니다. Python application은 Model / View / Controller로 분리되어 있고,
브라우저 canvas에서 annotation을 편집합니다. ROS node가 아니며 ROS graph에 연결하지
않습니다.

## 실행

`ws_aic/src`에서 실행합니다.

```bash
PIXI_FROZEN=true pixi run view_annotations ../data/img2pos/<version>
```

기본 주소 `http://127.0.0.1:5000`이 자동으로 열립니다. 브라우저를 자동 실행하지
않거나 port를 바꾸려면 다음 옵션을 사용합니다.

```bash
PIXI_FROZEN=true pixi run view_annotations --no-browser --port 8090 ../data/img2pos/<version>
```

tool directory에서 직접 실행할 수도 있습니다.

```bash
cd tools/bounding_box_tool
python main.py ../../../data/img2pos/<version>
```

경로를 생략하면 빈 editor가 열립니다. 상단 경로 입력란에 dataset root, `images/`
아래 directory 또는 단일 image의 **서버 기준 경로**를 입력합니다. 웹 브라우저의
보안 모델상 OS folder picker가 서버의 경로를 전달할 수 없으므로 경로를 직접
입력합니다.

dataset을 찾으면 `images/<relative>.jpg`에 대응하는
`annotations/<relative>.txt`를 읽고 `yolo_pose.yaml`의 `names`로 `SFP_00` 같은
label을 표시합니다.

## MVC 구조

| 역할 | 파일 | 책임 |
| --- | --- | --- |
| Model | `models/editor.py`, `models/dataset.py`, `models/occlusion.py` | dataset 상태, annotation 검증·저장, 삭제, visibility 계산 |
| View | `views/web.py`, `views/static/` | HTML/CSS/Canvas UI와 browser-side working copy rendering |
| Controller | `controllers/web.py`, `views/static/app.js`의 `EditorController` | HTTP API와 사용자 입력 orchestration |
| Composition root | `main.py` | Model·Controller·View 조립 및 HTTP server 실행 |

`127.0.0.1` 외의 주소에 bind하면 dataset 편집 API가 network에 노출되므로 신뢰할 수
있는 환경에서만 `--host 0.0.0.0` 등을 사용하십시오.

## 편집

| 조작 | 기능 |
| --- | --- |
| `Ctrl+O` | dataset 경로 입력란으로 이동 |
| `Ctrl+S` | 현재 annotation 저장 |
| `W` / `Add Box` | canvas에서 새 bbox를 drag하고 class 선택 |
| 객체 또는 우측 목록 click | 편집할 객체 선택 |
| bbox 내부 drag | bbox와 4개 keypoint 함께 이동 |
| `Shift` + canvas drag 후 선택 항목 drag | 범위 안 bbox와 keypoint 함께 이동 |
| bbox corner drag | bbox 크기와 keypoint 상대 위치 조절 |
| keypoint drag | 해당 keypoint 위치 수정 |
| `E` 또는 inspector class | class 변경 |
| inspector visibility | keypoint visibility `0/1/2` 변경 |
| `Delete` | 확인 후 단일 또는 범위 선택 bbox 삭제 |
| `Ctrl+D` | 현재 image·annotation 삭제, `samples.jsonl` 갱신 후 다음 image |
| `Ctrl+Z` | 바로 이전 annotation 상태로 복원 |
| `V` / `Auto Visibility` | 현재 image의 robot-arm occlusion 자동 반영 |
| `←` / `A`, `→` / `D` | 이전/다음 image |
| mouse wheel / `Ctrl++`, `Ctrl+-` | 확대·축소 |
| 빈 canvas drag | 확대된 image 이동 |
| `F` | 화면 맞춤 |
| `Overlay` | annotation 표시 전환 |

저장하지 않은 상태는 상단에 `UNSAVED`로 표시됩니다. 다른 image나 dataset을 열 때
Save/Discard/Cancel 흐름을 거칩니다. 서버는 browser가 읽은 label revision과 현재
파일 내용을 비교하므로, 다른 tab이나 수집 process가 label을 변경한 경우 `409`
충돌을 반환하고 덮어쓰지 않습니다. 모든 객체를 삭제하고 저장하면 빈 annotation
파일을 남깁니다.

`Ctrl+D`는 확인 후 현재 camera view의 image와 annotation을 영구 삭제하고,
`samples.jsonl`의 `images`, `annotations`, `annotation_object_counts`,
`annotation_labels`에서 해당 camera 항목을 제거합니다. 남은 camera가 없으면 해당
JSONL 행도 제거합니다. 이 파일 삭제는 undo로 복원되지 않습니다.

## Auto Visibility

`Auto Visibility`는 현재 image 한 장에만 적용되며 바로 저장하지 않습니다. 결과를
검수한 뒤 `Ctrl+S`로 저장합니다.

1. image 아래쪽에 연결된 큰 검은 영역을 robot-arm mask로 추출합니다.
2. 4개 keypoint polygon과 mask가 90% 이상 겹치면 완전 가림으로 판단합니다.
3. `sampling.collection_policy: near-port` dataset은 완전 가림 객체를 삭제하지 않고
   4개 keypoint를 모두 `visibility=1`로 유지합니다. 다른 policy는 객체를 삭제합니다.
4. 남은 객체에서 mask 안 keypoint는 `visibility=1`, 보이는 keypoint는 `2`로
   바꿉니다. `near-port`에서 기존 depth 판정이 `1`인 keypoint는 RGB mask 밖이어도
   `1`을 유지합니다.

RGB 기반 근사 판정이므로 결과를 반드시 시각적으로 확인해야 합니다. `1`은 점선 원,
`2`는 채워진 원으로 표시됩니다. 빈 TXT는 객체가 없는 정상 negative sample로
처리하고, 잘못된 annotation 행은 건너뛴 뒤 오른쪽 warning 영역에 원인을 표시합니다.
