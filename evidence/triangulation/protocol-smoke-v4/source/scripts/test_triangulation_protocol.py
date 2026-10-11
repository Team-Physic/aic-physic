# 평가 계획의 균등 배정·랜덤 재현·실제 광원 변화와 관측 기하를 검사한다.
# 가짜 모델이나 simulator 성공을 가정하지 않으며 파일은 임시 디렉터리에서만 만든다.
# 보이지 않는 포트와 depth 부재를 정상 사례로 통과시키는 회귀를 막는다.
from collections import Counter
import json
import tempfile
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from triangulation_protocol import make_plan, cases, look_at_tcp, visibility, job_complete
from run_triangulation_protocol import launcher_environment


def test_protocol_contract():
    """100개 배치의 균등성, 2700개 고유 조건, 같은 seed 입력의 일치를 확인한다."""
    with tempfile.TemporaryDirectory(prefix='triangulation-contract-') as folder:
        weights = Path(folder)/'weights'
        weights.write_bytes(b'checksum test only')
        plan = make_plan(20260909, [.25,.35,.45], [.7,1,1.3], 20, weights)
        assert json.dumps(plan,sort_keys=True) == json.dumps(make_plan(20260909,[.25,.35,.45],[.7,1,1.3],20,weights),sort_keys=True)
        assert Counter((l['card_count'],l['target']) for l in plan['layouts']) == {
            (n,f'SFP_{r}{p}'):2 for n in range(1,6) for r in range(5) for p in range(2)}
        identifiers = []
        for layout in plan['layouts']:
            trial = next(iter(layout['config']['trials'].values()))
            task = next(iter(trial['tasks'].values()))
            board = trial['scene']['task_board']
            active = [r for r in range(5) if board[f'nic_rail_{r}']['entity_present']]
            assert len(active) == layout['card_count']
            assert int(layout['target'][4]) in active
            assert task['target_module_name'] == f'nic_card_mount_{layout["target"][4]}'
            assert task['port_name'] == f'sfp_port_{layout["target"][5]}'
            for r in active:
                assert -.0215 <= board[f'nic_rail_{r}']['entity_pose']['translation'] <= .0234
            for li in range(3):
                identifiers.extend(c['id'] for c in cases(plan,layout['id'],li))
        assert len(identifiers) == len(set(identifiers)) == 2700
        intensities = [[float(v.text) for v in ET.fromstring(w).findall('./world/light/intensity')] for w in plan['worlds']]
        assert np.allclose(intensities[0],np.array(intensities[1])*.7)
        assert np.allclose(intensities[2],np.array(intensities[1])*1.3)
        assert '<camera_info_topic>/${name}/depth_camera_info</camera_info_topic>' in plan['annotation_description']
        for renderer in ('software','gpu'):
            path = Path(folder)/'launch.sh'
            path.write_text('. /ws_aic/install/setup.bash\ntrap cleanup_router EXIT\n')
            launcher_environment(['distrobox',str(path)],renderer)
            assert 'export ROS_HOME=' in path.read_text()
            assert ('Xvfb' in path.read_text()) == (renderer=='software')
            subprocess.run(['bash','-n',str(path)],check=True)
        print('100 layouts / 2700 cases / seed / light / renderer isolation passed')


def test_look_at_and_visibility():
    """offset이 있는 카메라에서도 목표와 거리·각도를 보존하고 가림을 거부한다."""
    port = np.eye(4)
    port[:3,3] = [.1,-.2,.8]
    extrinsic = np.eye(4)
    extrinsic[:3,3] = [.02,-.11,-.2]
    for angle in (-15,0,15):
        optical = look_at_tcp(port,extrinsic,.35,angle) @ extrinsic
        view = optical[:3,3]-port[:3,3]
        assert abs(np.linalg.norm(view)-.35) < 1e-12
        assert np.allclose(optical[:3,2],-view/.35)
        assert abs(np.rad2deg(np.arctan2(view[0],-view[2]))-angle) < 1e-12
        assert abs(np.linalg.det(optical[:3,:3])-1) < 1e-12
    corners = np.array([[-.01,-.01,1],[.01,-.01,1],[.01,.01,1],[-.01,.01,1]])
    k = np.array([[500,0,100],[0,500,100],[0,0,1]])
    good = visibility(corners,np.eye(4),k,(200,200,3),np.ones((100,100)),20,.002)
    assert good['visible']
    assert not visibility(corners,np.eye(4),k,(200,200,3),None,20,.002)['visible']
    assert not visibility(corners,np.eye(4),k,(200,200,3),np.full((100,100),.8),20,.002)['visible']
    assert not visibility(corners+[0.3,0,0],np.eye(4),k,(200,200,3),np.ones((100,100)),20,.002)['visible']
    assert not visibility(corners,np.eye(4),k,(200,200,3),np.full((100,100),np.nan),20,.002)['visible']
    print('Look-at geometry / clipping / occlusion / unknown depth passed')


def test_resume_does_not_accept_missing_cases():
    """중단·setup 오류·조건 누락을 resume의 완료 상태로 취급하지 않는다."""
    assert job_complete(0,[{'id':'a','status':'ineligible_view'}],['a'])
    assert not job_complete(130,[{'id':'a','status':'captured'}],['a'])
    assert not job_complete(0,[{'id':'a','status':'setup_failed'}],['a'])
    assert not job_complete(0,[{'id':'a','status':'captured'}],['a','b'])
    print('Resume completeness and failure checks passed')


if __name__ == '__main__':
    test_protocol_contract()
    test_look_at_and_visibility()
    test_resume_does_not_accept_missing_cases()
