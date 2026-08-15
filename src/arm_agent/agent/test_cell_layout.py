"""구역 좌표가 네 곳에서 같은지 검사한다.

같은 숫자가 언어가 다른 파일 넷에 흩어져 있다. 한 곳을 import 할 수 없으니
cell_layout.yaml을 진실로 두고 나머지를 텍스트로 읽어 대조한다.

  ① arm_perception/config/cell_layout.yaml   진실
  ② arm_agent/agent/agent.py 의 ZONE          구역 판정
  ③ arm_skills/src/skill_server.cpp 의 kTargets  팔이 실제로 가는 곳
  ④ arm_perception/worlds/scene_*.sdf 의 zone_*  눈에 보이는 판

ROS를 import 하지 않는다. agent.py도 실행하지 않고 ast로 소스에서 값만 꺼내므로
rclpy 없이 pytest가 돈다('노드 != 도구'와 같은 이유).
"""

import ast
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

import yaml

TOL = 1e-6


def _src_root():
    """이 파일에서 위로 올라가며 src/ 를 찾는다. 어디서 pytest를 돌려도 같게 잡힌다."""
    for parent in Path(__file__).resolve().parents:
        if parent.name == 'src':
            return parent
    raise RuntimeError('src/ 를 찾지 못했다')


SRC = _src_root()
LAYOUT = SRC / 'arm_perception' / 'config' / 'cell_layout.yaml'
AGENT = SRC / 'arm_agent' / 'agent' / 'agent.py'
SKILL = SRC / 'arm_skills' / 'src' / 'skill_server.cpp'
WORLDS = sorted((SRC / 'arm_perception' / 'worlds').glob('scene_*.sdf'))


def _truth():
    """진실. name -> (x0, x1, y0, y1, place_target) 로 돌려준다."""
    data = yaml.safe_load(LAYOUT.read_text())['zones']
    return {
        name: (z['x'][0], z['x'][1], z['y'][0], z['y'][1], z['place_target'])
        for name, z in data.items()
    }


def _center(x0, x1, y0, y1):
    return (round((x0 + x1) / 2.0, 6), round((y0 + y1) / 2.0, 6))


def _agent_zone():
    """agent.py의 ZONE 딕셔너리를 소스에서 꺼낸다(실행하지 않는다)."""
    tree = ast.parse(AGENT.read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == 'ZONE' for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError('agent.py에서 ZONE을 찾지 못했다')


def _skill_targets():
    """skill_server.cpp의 kTargets를 정규식으로 꺼낸다. name -> (x, y)."""
    text = SKILL.read_text()
    block = re.search(r'kTargets\s*=\s*\{(.*?)\n\};', text, re.S)
    assert block, 'skill_server.cpp에서 kTargets 블록을 찾지 못했다'
    found = re.findall(
        r'\{\s*"([A-Za-z_]+)"\s*,\s*\{\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\}\s*\}',
        block.group(1))
    return {name: (float(x), float(y)) for name, x, y in found}


def _sdf_zones(path):
    """씬 파일의 zone_* 모델. name -> (cx, cy, width, height)."""
    root = ET.parse(path).getroot()
    out = {}
    for model in root.iter('model'):
        name = model.get('name', '')
        if not name.startswith('zone_'):
            continue
        pose = [float(v) for v in model.find('pose').text.split()]
        size = [float(v) for v in model.find('.//box/size').text.split()]
        out[name[len('zone_'):]] = (pose[0], pose[1], size[0], size[1])
    return out


def test_layout_file_exists():
    """진실 파일이 있고 비어 있지 않은가. 나머지 테스트가 전부 이 파일에 기댄다."""
    assert LAYOUT.is_file(), f'{LAYOUT} 가 없다'
    assert _truth(), '구역이 하나도 없다'


def test_zones_do_not_overlap():
    """구역이 겹치면 zone_of가 어느 쪽을 돌려줄지 파일 순서에 달리게 된다."""
    zones = _truth()
    names = sorted(zones)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            ax0, ax1, ay0, ay1, _ = zones[a]
            bx0, bx1, by0, by1, _ = zones[b]
            overlap = ax0 < bx1 and bx0 < ax1 and ay0 < by1 and by0 < ay1
            assert not overlap, f'{a} 와 {b} 가 겹친다'


def test_agent_zone_matches_layout():
    """② agent.py ZONE == 진실. 사각형 네 값이 그대로 있어야 한다."""
    zones = _truth()
    got = _agent_zone()
    assert set(got) == set(zones), (
        f'구역 이름이 다르다. agent.py={sorted(got)} / yaml={sorted(zones)}')
    for name, (x0, x1, y0, y1, _) in zones.items():
        assert len(got[name]) == 4, (
            f'{name}: agent.py의 ZONE 값이 4개가 아니다({got[name]}). '
            '사각형 (x0, x1, y0, y1) 형식이어야 한다')
        for want, have, axis in zip((x0, x1, y0, y1), got[name], 'x0 x1 y0 y1'.split()):
            assert abs(want - have) < TOL, f'{name}.{axis} : yaml {want} != agent.py {have}'


def test_skill_targets_match_layout_centers():
    """③ skill_server kTargets == place 목적지 구역의 중심."""
    zones = _truth()
    got = _skill_targets()
    want = {n: _center(*z[:4]) for n, z in zones.items() if z[4]}
    assert set(got) == set(want), (
        f'place 목적지가 다르다. cpp={sorted(got)} / yaml={sorted(want)}')
    for name, (wx, wy) in want.items():
        gx, gy = got[name]
        assert math.isclose(wx, gx, abs_tol=TOL), f'{name}.x : yaml {wx} != cpp {gx}'
        assert math.isclose(wy, gy, abs_tol=TOL), f'{name}.y : yaml {wy} != cpp {gy}'


@pytest.mark.parametrize('world', WORLDS, ids=lambda p: p.name)
def test_sdf_pads_match_layout(world):
    """④ 씬의 zone_* 판 == 진실. 중심과 크기가 모두 맞아야 한다."""
    zones = _truth()
    got = _sdf_zones(world)
    if not got:
        # 셀 배치 이전에 만든 씬(one_block, two_colors 등)에는 구역 판이 없다.
        # 하나라도 있으면 전부 맞아야 한다 - 일부만 남는 것이 제일 위험하다.
        pytest.skip('구역 판이 없는 씬')
    assert set(got) == set(zones), (
        f'{world.name}: 구역 판이 다르다. sdf={sorted(got)} / yaml={sorted(zones)}')
    for name, (x0, x1, y0, y1, _) in zones.items():
        cx, cy = _center(x0, x1, y0, y1)
        gcx, gcy, gw, gh = got[name]
        assert math.isclose(cx, gcx, abs_tol=TOL), f'{name} 중심 x : {cx} != {gcx}'
        assert math.isclose(cy, gcy, abs_tol=TOL), f'{name} 중심 y : {cy} != {gcy}'
        assert math.isclose(x1 - x0, gw, abs_tol=TOL), f'{name} 폭 : {x1 - x0} != {gw}'
        assert math.isclose(y1 - y0, gh, abs_tol=TOL), f'{name} 높이 : {y1 - y0} != {gh}'
