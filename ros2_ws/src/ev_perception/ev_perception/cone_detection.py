"""~/Desktop/EV_formula_camera 에서 그대로 포팅한 라바콘 HSV 색상 검출 순수 로직.

라바콘 색상 인식: 오른쪽 = 파란색(남색/네이비), 왼쪽 = 노란색. YOLO 없이 순수 영상처리만 사용.
카메라 캡처/ROS2 연동은 이 파일에 두지 않는다 (cone_detector_node.py 담당) - 원본 스크립트가
setup_camera()/read_frame()만 교체하면 되게 분리해둔 것과 같은 이유로, 여기는 프레임(BGR
ndarray)을 받아 콘 후보 리스트만 돌려준다.
"""
import cv2
import numpy as np

# --- 라바콘 색상 범위 (HSV, OpenCV 기준 H:0-179) --- 원본 EV_formula_camera 기본값
LOWER_YELLOW = np.array([18, 90, 90])
UPPER_YELLOW = np.array([35, 255, 255])

# 파란색(남색/네이비) 라바콘 - 하늘색보다 어둡고 채도가 있는 남색 톤
LOWER_BLUE = np.array([95, 70, 30])
UPPER_BLUE = np.array([130, 255, 220])

# --- 라바콘으로 인정할 형태 기준 ---
MIN_AREA = 250                      # 이보다 작은 덩어리는 무시 (노이즈/원거리 오탐)
MIN_ASPECT, MAX_ASPECT = 0.8, 3.0   # 라바콘은 세로로 긴 삼각형 형태 (h/w)
MAX_CONES_PER_COLOR = 6             # 화면당 색상별로 반환할 최대 개수 (넓이 큰 순)

CENTER_DEAD_ZONE = 0.06             # 화면 중심 기준 이 비율 안쪽은 '중앙'으로 보고 위치 판정을 보류

EXPECTED_SIDE = {'YELLOW': 'LEFT', 'BLUE': 'RIGHT'}

_KERNEL = np.ones((5, 5), np.uint8)


def build_masks(hsv):
    mask_yellow = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
    mask_blue = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)

    masks = {'YELLOW': mask_yellow, 'BLUE': mask_blue}
    for name, m in masks.items():
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, _KERNEL)
        masks[name] = cv2.morphologyEx(m, cv2.MORPH_CLOSE, _KERNEL)
    return masks


def find_cones(mask):
    """마스크 안에서 라바콘 모양(세로로 긴 덩어리) 후보를 찾는다.

    반환: [(x, y, w, h, area, cx, cy), ...] area 큰 순
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cones = []

    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA:
            continue

        x, y, w, h = cv2.boundingRect(c)
        if w == 0:
            continue
        aspect = h / w
        if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
            continue

        cx, cy = x + w / 2, y + h / 2
        cones.append((x, y, w, h, area, cx, cy))

    cones.sort(key=lambda c: c[4], reverse=True)
    return cones[:MAX_CONES_PER_COLOR]


def classify_side(cx, frame_w):
    """콘 중심의 x좌표를 보고 화면 왼쪽/오른쪽/중앙 중 어디인지 반환한다."""
    center = frame_w / 2
    dead = frame_w * CENTER_DEAD_ZONE
    if cx < center - dead:
        return 'LEFT'
    if cx > center + dead:
        return 'RIGHT'
    return 'CENTER'


def detect_cones(frame):
    """BGR 프레임 한 장에서 색상별 콘 후보를 검출한다.

    반환: {'YELLOW': [(x,y,w,h,area,cx,cy,side,side_ok), ...], 'BLUE': [...]}
    """
    frame_w = frame.shape[1]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks = build_masks(hsv)

    detections = {'YELLOW': [], 'BLUE': []}
    for color, mask in masks.items():
        for (x, y, w, h, area, cx, cy) in find_cones(mask):
            side = classify_side(cx, frame_w)
            side_ok = (side == EXPECTED_SIDE[color] or side == 'CENTER')
            detections[color].append((x, y, w, h, area, cx, cy, side, side_ok))

    return detections
