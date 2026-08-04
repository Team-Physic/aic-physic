"""PortOffsetCollect 모듈 분할 규칙의 회귀 테스트."""

import ast
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_ROOT = PACKAGE_ROOT / "data_generator"
sys.path.insert(0, str(PACKAGE_ROOT))

from data_generator.PortOffsetCollect import PortOffsetCollect  # noqa: E402


EXPECTED_METHOD_MODULES = {
    "_lookup_transform": "data_generator.port_offset_runtime",
    "_lookup_latest_transform_stamped": "data_generator.port_offset_runtime",
    "_lookup_transform_at": "data_generator.port_offset_runtime",
    "_plug_location_label_in_base_frame": "data_generator.port_offset_labels",
    "_camera_intrinsic_matrix": "data_generator.port_offset_frames",
    "_finish_data_collection_episode": "data_generator.port_offset_episode",
    "_observation_sync_metadata": "data_generator.port_offset_dataset",
    "_wait_for_synchronized_observation": "data_generator.port_offset_dataset",
    "_stage_collect": "data_generator.port_offset_stage_motion",
    "insert_cable": "data_generator.port_offset_stage_episode",
}


def _port_offset_files() -> list[Path]:
    """구조 규칙을 검사할 PortOffset Python 파일 목록을 반환한다."""
    return [
        MODULE_ROOT / "PortOffsetCollect.py",
        *sorted(MODULE_ROOT.glob("port_offset_*.py")),
    ]


def test_methods_are_bound_from_role_modules() -> None:
    """대표 policy 메서드가 의도한 역할 모듈에서 바인딩되는지 확인한다."""
    for method_name, module_name in EXPECTED_METHOD_MODULES.items():
        method = getattr(PortOffsetCollect, method_name)
        assert method.__module__ == module_name


def test_control_path_is_ground_truth_only() -> None:
    """Approach와 collect 제어 경로에 YOLO 및 triangulation이 없는지 확인한다."""
    control_files = [
        MODULE_ROOT / "PortOffsetCollect.py",
        MODULE_ROOT / "port_offset_stage_episode.py",
        MODULE_ROOT / "port_offset_stage_motion.py",
    ]
    control_source = "\n".join(
        path.read_text(encoding="utf-8") for path in control_files
    ).lower()
    assert "yolo" not in control_source
    assert "triangulat" not in control_source
    assert not hasattr(PortOffsetCollect, "_triangulate_yolo_port")


def test_port_offset_files_do_not_exceed_500_lines() -> None:
    """PortOffset 관련 Python 파일이 각각 500줄 이하인지 확인한다."""
    too_long = {
        path.name: len(path.read_text(encoding="utf-8").splitlines())
        for path in _port_offset_files()
        if len(path.read_text(encoding="utf-8").splitlines()) > 500
    }
    assert too_long == {}


def test_all_port_offset_functions_have_korean_docstrings() -> None:
    """PortOffset 관련 모든 함수에 한글 docstring이 있는지 확인한다."""
    missing: list[str] = []
    for path in _port_offset_files():
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            docstring = ast.get_docstring(node) or ""
            if not any("가" <= char <= "힣" for char in docstring):
                missing.append(f"{path.name}:{node.lineno}:{node.name}")
    assert missing == []
