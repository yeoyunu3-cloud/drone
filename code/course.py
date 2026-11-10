"""도시 맵 코스: 건물, 게이트, 사람 위치를 적어 둔 유일한 원본.

make_world.py가 이걸 읽어 Gazebo 맵(worlds/city.sdf)을 만들고,
app.py가 이걸 읽어 게이트 통과 판정과 자동 비행 경로에 쓴다.

좌표는 Gazebo 세계 좌표 (단위 m).
  x: 동쪽 (드론이 처음 바라보는 방향)   y: 북쪽   z: 위
PX4(NED)로 바꾸면 north = y, east = x, down = -z 이다.
"""

import math
from dataclasses import dataclass

GATE_WIDTH = 2.5  # 게이트 구멍 가로 (m)
GATE_HEIGHT = 2.5  # 게이트 구멍 세로 (m)
GATE_BAR = 0.25  # 게이트 틀 두께 (m)

APPROACH_DIST = 4.0  # 자동 모드: 게이트 이만큼 앞에서 정렬을 시작한다
EXIT_DIST = 1.5  # 통과 판정 직후 드론이 있을 만한 위치 (게이트 뒤 이만큼)
PATH_MARGIN = 1.5  # 자동 비행 경로가 건물에서 최소 이만큼 떨어져야 한다
ROUTE_CLEARANCE = 2.0  # 길이 막혀서 위로 넘어갈 때, 가장 높은 장애물 꼭대기보다 이만큼 더 높이


@dataclass(frozen=True)
class Solid:
    """공중에 떠 있을 수도 있는 직육면체 (충돌 방지용). (x, y, z)는 한가운데, yaw는 z축 회전."""
    x: float
    y: float
    z: float
    size_x: float
    size_y: float
    size_z: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Box:
    """바닥에 놓인 직육면체 (건물, 기둥). yaw는 z축 회전(라디안)."""
    name: str
    x: float
    y: float
    size_x: float
    size_y: float
    height: float
    yaw: float = 0.0

    def solid(self):
        return Solid(self.x, self.y, self.height / 2, self.size_x, self.size_y, self.height, self.yaw)


@dataclass(frozen=True)
class Gate:
    """드론이 지나가는 문. (x, y, z)는 구멍의 한가운데, yaw는 지나가는 방향."""
    name: str
    x: float
    y: float
    z: float
    yaw: float

    def offset(self, x, y, z):
        """점이 게이트 기준으로 (앞뒤, 옆, 위아래) 얼마나 떨어져 있는지.
        앞뒤는 지나가는 방향이 +, 옆은 왼쪽이 +."""
        dx, dy = x - self.x, y - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return dx * c + dy * s, -dx * s + dy * c, z - self.z

    def point(self, along, side=0.0):
        """게이트 기준 (앞뒤, 옆) 위치를 세계 좌표 (x, y)로."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + along * c - side * s, self.y + along * s + side * c

    def frame(self):
        """게이트 틀 전체를 덮는 바닥 직육면체 (경로 검사용)."""
        return Box(self.name, self.x, self.y, GATE_BAR, GATE_WIDTH + 2 * GATE_BAR,
                   self.z + GATE_HEIGHT / 2 + GATE_BAR, self.yaw)

    def parts(self):
        """틀을 이루는 막대 4개: (이름, 게이트 기준 앞뒤, 옆, 높이, 크기 앞뒤, 옆, 높이).
        양쪽 기둥은 땅에서부터 세운다. make_world.py가 이걸로 맵을 그린다."""
        w, h, t = GATE_WIDTH, GATE_HEIGHT, GATE_BAR
        top = self.z + h / 2 + t
        side = w / 2 + t / 2
        return [
            ("left_post", 0, side, top / 2, t, t, top),
            ("right_post", 0, -side, top / 2, t, t, top),
            ("top_bar", 0, 0, self.z + h / 2 + t / 2, t, w + 2 * t, t),
            ("bottom_bar", 0, 0, self.z - h / 2 - t / 2, t, w, t),
        ]

    def solids(self):
        """틀 막대 4개를 세계 좌표 Solid로 (충돌 방지용)."""
        result = []
        for _, along, side, z, sx, sy, sz in self.parts():
            x, y = self.point(along, side)
            result.append(Solid(x, y, z, sx, sy, sz, self.yaw))
        return result


START = (0.0, 0.0)  # 드론 출발점
CRUISE_Z = 3.0  # 출발과 귀환 때 높이

# 게이트 한가운데 위치 (x, y, z). 이 순서대로 지나간다
_GATE_CENTERS = [
    (12.0, 0.0, 2.5),
    (26.0, 8.0, 4.0),
    (40.0, 8.0, 2.5),
    (48.0, -6.0, 3.0),
    (34.0, -18.0, 4.5),
    (14.0, -16.0, 2.5),
]


def _make_gates():
    # 게이트는 바로 앞 게이트(첫 게이트는 출발점)에서 오는 방향을 바라보게 세운다
    gates = []
    prev = START
    for i, (x, y, z) in enumerate(_GATE_CENTERS, start=1):
        gates.append(Gate(f"gate_{i}", x, y, z, math.atan2(y - prev[1], x - prev[0])))
        prev = (x, y)
    return gates


GATES = _make_gates()

BUILDINGS = [
    Box("building_1", 20.0, -6.0, 8.0, 6.0, 12.0),
    Box("building_2", 33.0, 1.0, 6.0, 6.0, 18.0),
    Box("building_3", 55.0, 10.0, 8.0, 8.0, 15.0),
    Box("building_4", 44.0, -20.0, 6.0, 8.0, 10.0),
    Box("building_5", 24.0, -26.0, 10.0, 6.0, 20.0),
    Box("building_6", -6.0, -14.0, 6.0, 6.0, 8.0),
    Box("building_7", 8.0, 14.0, 10.0, 8.0, 14.0),
    Box("building_8", 40.0, 24.0, 8.0, 8.0, 25.0),
    Box("building_9", 24.0, 20.0, 6.0, 6.0, 9.0),
    Box("pillar_1", 33.0, -9.0, 1.0, 1.0, 10.0),
    Box("pillar_2", 22.0, -20.5, 1.0, 1.0, 10.0),
]

PERSON = (9.0, -5.0)  # 사람 따라가기 모드용 사람 위치 (출발점에서 앞쪽 오른편)
PERSON_SIZE = (0.6, 0.6, 1.8)


def obstacles():
    """드론이 부딪힐 수 있는 모든 것 (건물, 기둥, 게이트 틀, 사람). 충돌 방지(safety.py)가 쓴다."""
    solids = [b.solid() for b in BUILDINGS]
    for g in GATES:
        solids += g.solids()
    sx, sy, sz = PERSON_SIZE
    solids.append(Solid(PERSON[0], PERSON[1], sz / 2, sx, sy, sz))
    return solids


def auto_waypoints():
    """자동 모드가 지나가는 지점: [(종류, 게이트 번호, x, y, z), ...]
    종류는 'approach'(게이트 앞 정렬 지점), 'exit'(게이트 바로 뒤), 'home'(출발점 위)."""
    points = []
    for i, g in enumerate(GATES):
        ax, ay = g.point(-APPROACH_DIST)
        ex, ey = g.point(EXIT_DIST)
        points.append(("approach", i, ax, ay, g.z))
        points.append(("exit", i, ex, ey, g.z))
    points.append(("home", None, START[0], START[1], CRUISE_Z))
    return points


def segment_hits_box(p, q, box, margin=0.0):
    """바닥에서 본 선분 p→q가 box(사방으로 margin만큼 키운 것)를 지나는지."""
    # box 기준 좌표로 바꾸면 축에 나란한 직사각형과 선분의 교차 문제가 된다
    c, s = math.cos(box.yaw), math.sin(box.yaw)

    def local(pt):
        dx, dy = pt[0] - box.x, pt[1] - box.y
        return dx * c + dy * s, -dx * s + dy * c

    (x0, y0), (x1, y1) = local(p), local(q)
    half = (box.size_x / 2 + margin, box.size_y / 2 + margin)
    t0, t1 = 0.0, 1.0
    for start, delta, h in ((x0, x1 - x0, half[0]), (y0, y1 - y0, half[1])):
        if abs(delta) < 1e-9:
            if abs(start) > h:
                return False
            continue
        a, b = (-h - start) / delta, (h - start) / delta
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
        if t0 > t1:
            return False
    return True


def check_course(margin=PATH_MARGIN):
    """자동 비행 경로가 건물이나 다른 게이트에 부딪히는지 검사. 문제 목록을 돌려준다 (없으면 [])."""
    problems = []
    points = [("start", None, START[0], START[1], CRUISE_Z)] + auto_waypoints()
    for (kind_a, gate_a, xa, ya, _), (kind_b, gate_b, xb, yb, _) in zip(points, points[1:]):
        through = gate_a if kind_a == "approach" else None  # approach→exit 구간은 그 게이트를 지나간다
        obstacles = BUILDINGS + [g.frame() for i, g in enumerate(GATES) if i != through]
        for box in obstacles:
            # 게이트 틀은 얇아서 margin을 조금만 준다
            m = margin if box.name.startswith(("building", "pillar")) else 0.5
            if segment_hits_box((xa, ya), (xb, yb), box, m):
                problems.append(f"{kind_a}→{kind_b} 경로가 {box.name}에 걸림")
    return problems


def route_blockers(p, q):
    """바닥에서 본 선분 p→q를 가로막는 것들 (건물·기둥, 게이트 틀, 사람). check_course와 같은 여유로 본다."""
    person = Box("person", PERSON[0], PERSON[1], PERSON_SIZE[0], PERSON_SIZE[1], PERSON_SIZE[2])
    boxes = [(b, PATH_MARGIN) for b in BUILDINGS] + [(g.frame(), 0.5) for g in GATES] + [(person, 0.5)]
    return [box for box, margin in boxes if segment_hits_box(p, q, box, margin)]


def route_height(p, q):
    """p에서 q로 곧장 가면 부딪히는 것이 있을 때, 그 위로 넘어갈 높이. 막힌 게 없으면 None.
    장애물은 바닥부터 꼭대기까지 꽉 찬 기둥으로 본다 (게이트 틀도). 그래서 막힌 게 없으면
    어떤 높이로 가든 부딪히지 않는다."""
    blockers = route_blockers(p, q)
    if not blockers:
        return None
    return max(b.height for b in blockers) + ROUTE_CLEARANCE


class GateTracker:
    """드론 위치를 계속 받아서 다음 게이트를 지나갔는지 판정한다."""

    def __init__(self, gates=GATES):
        self.gates = gates
        self.reset()

    def reset(self):
        """게이트 1번부터 다시."""
        self.next = 0  # 다음에 지나갈 게이트 번호
        self._prev_along = None

    @property
    def done(self):
        return self.next >= len(self.gates)

    def update(self, x, y, z):
        """이번에 다음 게이트를 지나갔으면 True."""
        if self.done:
            return False
        along, side, up = self.gates[self.next].offset(x, y, z)
        crossed = self._prev_along is not None and self._prev_along < 0 <= along
        self._prev_along = along
        if crossed and abs(side) < GATE_WIDTH / 2 and abs(up) < GATE_HEIGHT / 2:
            self.next += 1
            self._prev_along = None
            return True
        return False


def ned_to_world(north, east, down):
    """PX4 위치(NED)를 Gazebo 세계 좌표로."""
    return east, north, -down


def world_to_ned(x, y, z):
    """Gazebo 세계 좌표를 PX4 위치(NED)로."""
    return y, x, -z


def world_yaw_to_ned_deg(yaw):
    """세계 좌표 yaw(동쪽이 0, 반시계 +, 라디안)를 PX4 yaw(북쪽이 0, 시계 +, 도)로."""
    return (90.0 - math.degrees(yaw) + 180.0) % 360.0 - 180.0
