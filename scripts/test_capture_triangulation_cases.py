# 평가 장면 생성기의 카드 수·seed 계약과 렌더러 설정을 검사한다.
# 기존 collector 기본 인자를 사용하고 생성 world는 임시 디렉터리에서 자동 삭제한다.
# 실제 시뮬레이터나 네트워크에 접속하지 않는다.
import random
import tempfile
from pathlib import Path
from capture_triangulation_cases import build_parser, fixed_card_config, software_world


def test_case_generation_contract():
    """정확한 카드 수와 동일 seed 재현, 생성 world의 VCT 제거를 확인한다."""
    args = build_parser().parse_args(['--port-type', 'sfp'])
    for count in (1, 3, 5):
        args.exact_card_count = count
        config, params = fixed_card_config(0, random.Random(42), args)
        assert (config, params) == fixed_card_config(0, random.Random(42), args)
        board = next(iter(config['trials'].values()))['scene']['task_board']
        actual = sum(v.get('entity_present', False) for k, v in board.items() if k.startswith('nic_rail_'))
        assert actual == count
    with tempfile.TemporaryDirectory(prefix='triangulation-world-test-') as folder:
        path, _ = software_world(0, random.Random(42), args, Path(folder)/'world.sdf')
        assert 'GlobalIlluminationVct' not in path.read_text()


if __name__ == '__main__':
    test_case_generation_contract()
    print('Card-count, seed and renderer checks passed.')
