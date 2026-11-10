"""카메라 화면에서 주황색 게이트를 색으로 찾는다.

게이트 틀(기둥 + 위아래 막대)은 주황색이라, 주황색만 남기면 가운데에 구멍이 뚫린 모양이 된다.
그 구멍이 드론이 지나갈 곳이다. 구멍 크기(픽셀)와 실제 크기(course.GATE_WIDTH)를 비교하면
게이트까지 거리와, 드론이 게이트 한가운데서 옆/위아래로 얼마나 벗어났는지(m)를 알 수 있다.
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np

import course

HORIZONTAL_FOV = 1.74  # 드론 카메라 가로 시야각 (라디안, mono_cam 모델 설정값)

# 주황색 범위 (OpenCV HSV: 색상 0~180)
ORANGE_LOW = np.array([5, 150, 120])
ORANGE_HIGH = np.array([22, 255, 255])

MIN_HOLE_RATIO = 0.04  # 구멍이 화면 가로의 이만큼보다 작으면 너무 멀어서 안 믿는다


@dataclass
class GateView:
    x: int  # 구멍 한가운데 (픽셀)
    y: int
    width: int  # 구멍 가로 (픽셀)
    height: int
    distance: float  # 게이트까지 거리 추정 (m)
    side: float  # 드론이 게이트 한가운데보다 왼쪽에 있는 정도 (m, course.Gate.offset과 같은 기준)
    up: float  # 드론이 게이트 한가운데보다 위에 있는 정도 (m)


def find_gate(frame):
    """frame에서 가장 큰 게이트 구멍을 찾는다. 없으면 None."""
    h, w = frame.shape[:2]
    mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), ORANGE_LOW, ORANGE_HIGH)
    # RETR_CCOMP: 바깥 테두리와 그 안의 구멍을 따로 알려준다. 부모가 있는 윤곽선이 구멍이다
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return None

    best = None
    for contour, (_, _, _, parent) in zip(contours, hierarchy[0]):
        if parent < 0:
            continue
        x, y, bw, bh = cv2.boundingRect(contour)
        touches_edge = x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1
        if bw < w * MIN_HOLE_RATIO or touches_edge:
            continue  # 너무 작거나, 화면 밖으로 잘려서 크기를 믿을 수 없음
        if best is None or bw * bh > best[2] * best[3]:
            best = (x, y, bw, bh)
    if best is None:
        return None

    x, y, bw, bh = best
    focal = (w / 2) / math.tan(HORIZONTAL_FOV / 2)  # 초점거리 (픽셀)
    meters_per_pixel = course.GATE_WIDTH / bw
    cx, cy = x + bw // 2, y + bh // 2
    return GateView(
        x=cx, y=cy, width=bw, height=bh,
        distance=focal * course.GATE_WIDTH / bw,
        side=(cx - w / 2) * meters_per_pixel,  # 구멍이 화면 오른쪽 = 드론이 왼쪽에 있음
        up=(cy - h / 2) * meters_per_pixel,  # 구멍이 화면 아래쪽 = 드론이 위에 있음
    )


def draw_gate(frame, view):
    """찾은 게이트 구멍에 표시를 그린다."""
    if view is None:
        return
    top_left = (view.x - view.width // 2, view.y - view.height // 2)
    bottom_right = (view.x + view.width // 2, view.y + view.height // 2)
    cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 3)
    cv2.drawMarker(frame, (view.x, view.y), (0, 255, 0), cv2.MARKER_CROSS, 30, 3)
    cv2.putText(frame, f"gate {view.distance:.1f}m", (top_left[0], top_left[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
