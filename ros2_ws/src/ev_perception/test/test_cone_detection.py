"""cone_detection.py 검출 로직 단위 테스트 (합성 이미지 사용, 실카메라 불필요)."""
import numpy as np

from ev_perception import cone_detection

YELLOW_BGR = (0, 220, 220)   # cone_detection.LOWER/UPPER_YELLOW 범위 안
BLUE_BGR = (180, 60, 20)     # cone_detection.LOWER/UPPER_BLUE 범위 안


def _make_frame(w=640, h=480):
    return np.zeros((h, w, 3), dtype=np.uint8)


def _paint_cone(frame, x, y, w, h, bgr):
    frame[y:y + h, x:x + w] = bgr


def test_detects_yellow_on_left_as_side_ok():
    frame = _make_frame()
    _paint_cone(frame, 80, 200, 70, 100, YELLOW_BGR)  # 화면 왼쪽, aspect=100/70≈1.43

    dets = cone_detection.detect_cones(frame)

    assert len(dets['YELLOW']) == 1
    assert dets['BLUE'] == []
    _, _, _, _, _, _, _, side, side_ok = dets['YELLOW'][0]
    assert side == 'LEFT'
    assert side_ok is True


def test_detects_blue_on_right_as_side_ok():
    frame = _make_frame()
    _paint_cone(frame, 500, 200, 70, 100, BLUE_BGR)  # 화면 오른쪽

    dets = cone_detection.detect_cones(frame)

    assert len(dets['BLUE']) == 1
    _, _, _, _, _, _, _, side, side_ok = dets['BLUE'][0]
    assert side == 'RIGHT'
    assert side_ok is True


def test_yellow_on_right_is_flagged_not_side_ok():
    """노란색(왼쪽 기대) 콘이 화면 오른쪽에서 검출되면 side_ok=False여야 한다 (오검출 의심 플래그)."""
    frame = _make_frame()
    _paint_cone(frame, 500, 200, 70, 100, YELLOW_BGR)

    dets = cone_detection.detect_cones(frame)

    assert len(dets['YELLOW']) == 1
    _, _, _, _, _, _, _, side, side_ok = dets['YELLOW'][0]
    assert side == 'RIGHT'
    assert side_ok is False


def test_too_small_blob_is_ignored():
    frame = _make_frame()
    _paint_cone(frame, 80, 200, 10, 14, YELLOW_BGR)  # area ~140 < MIN_AREA(250)

    dets = cone_detection.detect_cones(frame)

    assert dets['YELLOW'] == []


def test_wrong_aspect_ratio_is_ignored():
    frame = _make_frame()
    _paint_cone(frame, 80, 100, 300, 100, YELLOW_BGR)  # h/w = 100/300 ≈ 0.33 < MIN_ASPECT(0.8)

    dets = cone_detection.detect_cones(frame)

    assert dets['YELLOW'] == []


def test_classify_side_dead_zone():
    frame_w = 640
    center = frame_w / 2
    dead = frame_w * cone_detection.CENTER_DEAD_ZONE

    assert cone_detection.classify_side(center, frame_w) == 'CENTER'
    assert cone_detection.classify_side(center - dead - 1, frame_w) == 'LEFT'
    assert cone_detection.classify_side(center + dead + 1, frame_w) == 'RIGHT'
