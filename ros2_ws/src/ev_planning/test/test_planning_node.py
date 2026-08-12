"""planning_node.py의 콘 트랙 추종 조향 계산(_compute_steering_deg) 단위 테스트."""
import pytest
import rclpy

from ev_interfaces.msg import Cone, ConeArray
from ev_planning.planning_node import PlanningNode


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = PlanningNode()
    yield n
    n.destroy_node()


def _cone(color, cx, area=1000.0, side_ok=True):
    c = Cone()
    c.color = color
    c.cx = float(cx)
    c.area = float(area)
    c.side_ok = side_ok
    return c


def test_both_sides_visible_steers_toward_midpoint(node):
    cones = ConeArray(image_width=640, cones=[_cone('YELLOW', 115.0), _cone('BLUE', 535.0)])

    steer = node._compute_steering_deg(cones)

    assert steer == pytest.approx(0.46875, abs=1e-6)


def test_only_left_cone_steers_away_from_it(node):
    cones = ConeArray(image_width=640, cones=[_cone('YELLOW', 115.0)])

    steer = node._compute_steering_deg(cones)

    assert steer > 0  # 오른쪽으로 회피 (게이트가 콘 오른쪽에 있다고 가정)


def test_only_right_cone_steers_away_from_it(node):
    cones = ConeArray(image_width=640, cones=[_cone('BLUE', 535.0)])

    steer = node._compute_steering_deg(cones)

    assert steer < 0  # 왼쪽으로 회피


def test_no_cones_returns_none(node):
    cones = ConeArray(image_width=640, cones=[])

    assert node._compute_steering_deg(cones) is None


def test_side_ok_false_cone_is_ignored(node):
    """반대쪽에서 검출된 콘(side_ok=False)은 조향 계산에서 무시해야 한다."""
    cones = ConeArray(image_width=640, cones=[
        _cone('YELLOW', 535.0, side_ok=False),  # 오검출 의심 - 무시돼야 함
        _cone('BLUE', 535.0, side_ok=True),
    ])

    steer = node._compute_steering_deg(cones)

    assert steer < 0  # YELLOW가 무시되면 BLUE만 남아 왼쪽으로 회피


def test_picks_largest_area_cone_per_color(node):
    cones = ConeArray(image_width=640, cones=[
        _cone('YELLOW', 50.0, area=100.0),     # 더 왼쪽이지만 면적 작음(멀리 있음)
        _cone('YELLOW', 200.0, area=5000.0),   # 면적 큼(가까움) - 이게 선택돼야 함
        _cone('BLUE', 440.0, area=5000.0),
    ])

    steer = node._compute_steering_deg(cones)

    expected_target_cx = (200.0 + 440.0) / 2.0
    expected_offset = (expected_target_cx - 320.0) / 320.0
    expected_steer = expected_offset * node.steer_gain_deg
    assert steer == pytest.approx(expected_steer, abs=1e-6)


def test_steer_is_clamped_to_max(node):
    cones = ConeArray(image_width=640, cones=[_cone('BLUE', 639.0)])

    steer = node._compute_steering_deg(cones)

    assert abs(steer) <= node.max_steer_deg
