"""착륙할 자리 고르기: 드론이 내려앉을 바닥이 평평하고 넉넉한지 본다. (app.py가 쓴다)

드론은 가로세로 약 0.8m라서, 한가운데만 지붕 위이고 나머지가 허공이면
가장자리에 걸쳐 내려앉다가 기울어져 떨어진다. 기둥 꼭대기(1m), 게이트 막대(0.25m),
사람 머리 위도 마찬가지다. 벽에 바짝 붙어 있으면 내려가다 프로펠러가 벽에 닿는다.

그래서 드론 크기(FOOTPRINT)만큼의 자리를 여러 점으로 찍어 보고:
  - 모든 점 아래 바닥 높이가 같고
  - 드론 높이에서 그 자리를 가로막는 물체(벽)가 없으면
그 자리에 착륙한다. 아니면 가까운 곳 중에 그런 자리를 찾아서 그리로 옮긴 다음 착륙한다.
넘어진 드론을 세울 때(X 키)도 같은 방법으로 자리를 고른다.
"""

import math
import time

from mavsdk.plugins.offboard import PositionNedYaw

import course

FOOTPRINT = 0.55  # 드론 한가운데에서 프로펠러 끝까지 + 여유 (m)
MOVE_MARGIN = 0.3  # 다른 자리로 옮길 때는 이만큼 더 넉넉한 곳을 고른다 (내려앉으며 조금 밀려도 괜찮게)
FLAT_TOLERANCE = 0.05  # 바닥 높이가 이만큼까지 달라도 평평하다고 본다 (m)
SEARCH_RADIUS = 6.0  # 이 거리 안에서 다른 자리를 찾는다 (m)
SEARCH_STEP = 0.25
PATH_MARGIN = 0.6  # 옮겨 가는 길이 장애물에서 이만큼 떨어져야 한다 (m)

SOLIDS = course.obstacles()


def _inside(solid, x, y, margin=0.0):
    """위에서 봤을 때 점 (x, y)가 solid 안에 있는지."""
    c, s = math.cos(solid.yaw), math.sin(solid.yaw)
    dx, dy = x - solid.x, y - solid.y
    lx, ly = dx * c + dy * s, -dx * s + dy * c
    return abs(lx) <= solid.size_x / 2 + margin and abs(ly) <= solid.size_y / 2 + margin


def _local(solid, x, y):
    """점 (x, y)를 solid 기준 좌표로 (solid의 한가운데가 0, 축이 나란하게)."""
    c, s = math.cos(solid.yaw), math.sin(solid.yaw)
    dx, dy = x - solid.x, y - solid.y
    return dx * c + dy * s, -dx * s + dy * c


def _overlaps(solid, x, y, radius):
    """위에서 봤을 때 (x, y) 둘레 radius 원(드론 자리)이 solid와 겹치는지."""
    return _flat_distance(solid, x, y) < radius


def _covers(solid, x, y, radius):
    """위에서 봤을 때 드론 자리(원)가 solid 위에 통째로 들어가는지."""
    lx, ly = _local(solid, x, y)
    return abs(lx) + radius <= solid.size_x / 2 and abs(ly) + radius <= solid.size_y / 2


def headroom(x, y, z, radius=FOOTPRINT):
    """높이 z에 있는 드론 위로 얼마나 비어 있는지 (m). 위에 아무것도 없으면 무한대.
    게이트 막대 밑에 떨어져 있으면 이륙하다 부딪힌다."""
    room = math.inf
    for solid in SOLIDS:
        bottom = solid.z - solid.size_z / 2
        # 바닥에서 시작하는 옆 물체(기둥, 벽)는 '위'가 아니다. 드론보다 위에 떠 있는 것만 (게이트 막대 등)
        if bottom > z + 0.2 and _overlaps(solid, x, y, radius):
            room = min(room, bottom - z)
    return room


def spot_floor(x, y, z, min_headroom=0.0, radius=FOOTPRINT):
    """(x, y)에 높이 z에서 내려앉으면 닿을 바닥 높이. 걸치거나 벽에 닿는 자리면 None.
    드론 자리는 반지름 radius인 원으로 본다.
    min_headroom: 바닥 위로 이만큼은 비어 있어야 한다 (다시 이륙할 수 있게)."""
    below = []  # 드론 자리와 겹치고 드론보다 아래에 있는 물체 (지붕, 기둥 꼭대기, 막대 등)
    for solid in SOLIDS:
        if not _overlaps(solid, x, y, radius):
            continue
        top = solid.z + solid.size_z / 2
        bottom = solid.z - solid.size_z / 2
        if top <= z + FLAT_TOLERANCE:
            below.append(solid)
        elif bottom < z + 0.3:
            return None  # 드론 높이에 걸쳐 있다 = 벽 속이거나 벽에 닿는다
    if below:
        # 가장 높은 바닥 위에 드론 자리가 통째로 들어가야 한다 (아니면 가장자리에 걸친다)
        highest = max(below, key=lambda b: b.z + b.size_z / 2)
        if not _covers(highest, x, y, radius):
            return None
        floor = highest.z + highest.size_z / 2
    else:
        floor = 0.0  # 땅
    if headroom(x, y, floor, radius) < min_headroom:
        return None
    return floor


def _path_clear(x0, y0, x1, y1, z):
    """높이 z에서 (x0, y0) → (x1, y1)로 옆으로 옮겨 갈 때 부딪히는 게 없는지."""
    for solid in SOLIDS:
        top = solid.z + solid.size_z / 2
        bottom = solid.z - solid.size_z / 2
        if top < z - FOOTPRINT or bottom > z + FOOTPRINT:
            continue  # 그 높이에는 없다
        # 이미 그 물체에 PATH_MARGIN보다 가까이 있으면(벽 옆) 지금 거리만큼만 띄우고 본다.
        # 그러면 멀어지거나 나란히 가는 길은 괜찮고, 더 다가가는 길만 막힌다
        margin = min(PATH_MARGIN, _flat_distance(solid, x0, y0) - 0.05)
        if margin >= 0 and course.segment_hits_box((x0, y0), (x1, y1), solid, margin):
            return False
    return True


def _flat_distance(solid, x, y):
    """위에서 봤을 때 점 (x, y)에서 solid까지 거리 (안에 있으면 0)."""
    c, s = math.cos(solid.yaw), math.sin(solid.yaw)
    dx, dy = x - solid.x, y - solid.y
    lx, ly = dx * c + dy * s, -dx * s + dy * c
    return math.hypot(max(0.0, abs(lx) - solid.size_x / 2), max(0.0, abs(ly) - solid.size_y / 2))


def find_spot(x, y, z, check_path=True, min_headroom=0.0):
    """높이 z에 있는 드론이 착륙할 자리 (x, y, 바닥 높이). 지금 자리가 괜찮으면 지금 자리.
    가까운 곳부터 찾는다. check_path면 높이 z에서 옆으로 옮겨 갈 길이 막히지 않은 곳만.
    못 찾으면 None."""
    floor = spot_floor(x, y, z, min_headroom)
    if floor is not None:
        return x, y, floor
    steps = int(SEARCH_RADIUS / SEARCH_STEP)
    for k in range(1, steps + 1):
        r = k * SEARCH_STEP
        n = max(8, int(2 * math.pi * r / SEARCH_STEP))
        for i in range(n):
            a = 2 * math.pi * i / n
            cx, cy = x + r * math.cos(a), y + r * math.sin(a)
            floor = spot_floor(cx, cy, z, min_headroom, FOOTPRINT + MOVE_MARGIN)
            if floor is not None and (not check_path or _path_clear(x, y, cx, cy, z)):
                return cx, cy, floor
    return None


class Lander:
    """L 키: 지금 자리가 괜찮으면 바로 착륙하고, 아니면 고른 자리로 옆으로 옮긴 다음 착륙한다."""

    ARRIVE_DIST = 0.15  # 고른 자리에 이만큼 가까워지고
    ARRIVE_SPEED = 0.2  # 이보다 느려지면 착륙 시작
    MOVE_TIMEOUT_S = 15

    def __init__(self):
        self.target = None  # 옮겨 가는 중이면 (x, y, z, yaw_deg)
        self._deadline = 0.0

    @property
    def active(self):
        return self.target is not None

    def cancel(self):
        self.target = None

    def request(self, link, x, y, z):
        """착륙을 시작하거나 착륙할 자리로 옮기기 시작한다. 알릴 글을 돌려준다."""
        spot = find_spot(x, y, z)
        if spot is None:
            return "근처에 평평한 착륙 자리가 없음. 다른 곳으로 옮긴 뒤 다시 L"
        sx, sy, floor = spot
        where = "땅" if floor < 0.05 else f"{floor:.1f}m 높이 바닥"
        if math.hypot(sx - x, sy - y) < self.ARRIVE_DIST:
            return link.start_land() or f"착륙 ({where})"
        yaw_deg = course.world_yaw_to_ned_deg(link.yaw())
        self.target = (sx, sy, z, yaw_deg)
        self._deadline = time.monotonic() + self.MOVE_TIMEOUT_S
        return f"가장자리·좁은 곳이라 {math.hypot(sx - x, sy - y):.1f}m 옮긴 뒤 착륙 ({where})"

    def step(self, link, x, y):
        """옮겨 가는 중이면 (명령, 알릴 글). 도착하면 착륙을 시작한다. 할 일 없으면 (None, None)."""
        if self.target is None:
            return None, None
        if not link.flying:
            self.target = None
            return None, None
        tx, ty, tz, yaw_deg = self.target
        arrived = math.hypot(tx - x, ty - y) < self.ARRIVE_DIST and link.speed() < self.ARRIVE_SPEED
        if arrived or time.monotonic() > self._deadline:
            self.target = None
            return None, link.start_land() or "착륙 시작"
        return PositionNedYaw(*course.world_to_ned(tx, ty, tz), yaw_deg), None
