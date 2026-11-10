"""app.py의 세 가지 비행 모드.

모든 모드는 같은 모양이다:
  start(link)          모드가 켜질 때 한 번
  on_key(key)          키 입력 (이 모드가 쓰는 키면 True)
  step(frame, link)    1초에 30번. 카메라 화면과 드론 상태를 보고 Offboard 명령을 돌려준다
  status()             화면에 보여줄 한 줄
"""

import math
import time

from mavsdk.plugins.offboard import PositionNedYaw, VelocityBodyYawspeed, VelocityNedYaw

import course
import gate_vision
from drone_link import STOP
from follow_person import compute_velocity
from keyboard_control import CLIMB_SPEED, KEY_HOLD_S, MOVES, SPEED, YAW_SPEED
from vision import find_target


def clamp(value, limit):
    return max(-limit, min(limit, value))


def angle_diff(a, b):
    """두 각도(라디안)의 차이를 -pi ~ pi로."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


class ManualMode:
    """키보드로 직접 조종. 키는 keyboard_control.py와 같다.

    키는 '목표 속도'만 정하고, 실제로 보내는 속도는 진짜 드론처럼 서서히 따라간다.
    (키를 누르는 순간 0 → 1.5 m/s로 튀지 않고, 떼도 스르르 멈춘다)
    held_keys.HeldKeys가 키 상태를 읽을 수 있으면 누른 동안만, 여러 키를 같이 눌러 움직인다.
    못 읽으면 예전처럼 macOS 키 반복으로 '누르고 있음'을 짐작한다.
    """
    name = "MANUAL"

    # 1초에 바꿀 수 있는 속도: (앞뒤, 좌우 m/s, 위아래 m/s, 회전 도/s)
    ACCEL = (2.5, 2.5, 1.5, 120.0)
    # 목표에 가까워지면 더 천천히 바꾼다 (차이의 몇 배/초). 멈출 때 '툭' 서지 않고 스르르 선다
    EASE = 4.0

    def __init__(self, keys=None):
        self.keys = keys
        self.target = (0.0, 0.0, 0.0, 0.0)  # (앞, 오른쪽, 아래, 회전)
        self.current = (0.0, 0.0, 0.0, 0.0)
        self.last_key_time = 0.0
        self.last_step = time.time()

    def start(self, link):
        self.target = self.current = (0.0, 0.0, 0.0, 0.0)
        self.last_step = time.time()

    def _use_held(self):
        return self.keys is not None and self.keys.available

    def on_key(self, key):
        if key == ord(" "):
            self.target = (0.0, 0.0, 0.0, 0.0)
            return True
        if key not in MOVES:
            return False
        if self.keys is not None:
            self.keys.saw_key_event(key)
        if self._use_held():
            return True  # 움직임은 step에서 키 상태로 정한다
        self.target = self._target_for([MOVES[key]])
        self.last_key_time = time.time()
        return True

    @staticmethod
    def _target_for(moves):
        forward, right, down, yaw = (clamp(sum(m[i] for m in moves), 1) for i in range(4))
        scale = 1 / math.hypot(forward, right) if forward and right else 1  # 대각선도 같은 속도
        return (forward * SPEED * scale, right * SPEED * scale, down * CLIMB_SPEED, yaw * YAW_SPEED)

    def step(self, frame, link):
        now = time.time()
        dt = min(now - self.last_step, 0.2)  # 창이 잠깐 멈췄다 와도 한 번에 크게 튀지 않게
        self.last_step = now
        if self._use_held():
            self.target = self._target_for([MOVES[k] for k in self.keys.held()])
        elif now - self.last_key_time > KEY_HOLD_S:
            self.target = (0.0, 0.0, 0.0, 0.0)  # 키를 떼면 멈춘다
        self.current = tuple(
            cur + clamp(goal - cur, min(accel, self.EASE * abs(goal - cur)) * dt)
            for cur, goal, accel in zip(self.current, self.target, self.ACCEL)
        )
        return VelocityBodyYawspeed(*self.current)

    def status(self):
        return "W/S A/D move  Up/Down climb  Left/Right turn  Space stop"


class FollowMode:
    """YOLO로 사람을 찾아 따라간다. 계산은 follow_person.py와 같다."""
    name = "FOLLOW"

    def __init__(self, model):
        self.model = model
        self.target = None

    def start(self, link):
        self.target = None

    def on_key(self, key):
        return False

    def step(self, frame, link):
        self.target = find_target(self.model, frame)
        return compute_velocity(self.target)

    def status(self):
        return "following person" if self.target else "no person -> hover"


class RaceMode:
    """게이트 코스를 자동으로 통과한다.

    게이트마다 세 단계:
      approach  게이트 앞 APPROACH_DIST(m) 지점까지 좌표로 날아가서 게이트를 정면으로 본다
      pass      게이트 쪽으로 앞으로 가면서, 카메라로 구멍 한가운데를 맞춘다
                (카메라에 안 잡히면 좌표로 맞춘다)
      exit      통과 판정 뒤 EXIT_DIST(m)만큼 곧장 빠져나간다 (바로 꺾으면 틀에 걸린다)
    마지막 게이트를 지나면 출발점 위로 돌아와 멈춘다.
    """
    name = "AUTO"

    APPROACH_RADIUS = 0.5  # 정렬 지점에 이만큼 가까워지면 통과 시작 (m)
    APPROACH_YAW = math.radians(8)  # 방향 오차가 이보다 작아야 통과 시작
    PASS_SPEED = 1.5  # 게이트를 지나갈 때 앞으로 가는 속도 (m/s)
    CORRECTION_GAIN = 1.2  # 한가운데에서 1m 벗어나면 1.2 m/s로 되돌아간다
    MAX_CORRECTION = 0.8
    CAMERA_MIN_DIST = 1.5  # 게이트에 이보다 가까우면 화면에서 잘려서 카메라를 안 쓴다
    CAMERA_MAX_ERROR = 2.5  # 카메라 거리와 좌표 거리가 이만큼 다르면 다른 게이트를 본 것
    ROUTE_NEAR = 1.0  # 목표까지 이보다 가까우면 길 검사 없이 곧장 (바로 위에서 내려가는 중)

    def __init__(self, tracker):
        self.tracker = tracker
        self.phase = "approach"
        self.gate_index = tracker.next
        self.exit_gate = None
        self.home_yaw = None
        self.view = None
        self.source = ""
        self.route = ""  # 장애물 위로 넘어가는 중이면 안내 글

    def start(self, link):
        self.phase = "approach"
        self.gate_index = self.tracker.next
        self.home_yaw = None

    def on_key(self, key):
        return False

    def step(self, frame, link):
        x, y, z = link.position()
        self.view = None
        if self.tracker.next != self.gate_index:  # 방금 게이트를 지나갔다
            self.exit_gate = self.tracker.gates[self.gate_index]
            self.gate_index = self.tracker.next
            self.phase = "exit"

        if self.phase == "exit":
            # 바로 꺾으면 아직 틀 안에 걸쳐 있는 몸통이 틀에 부딪힌다. EXIT_DIST만큼 곧장 빠져나간다
            along, side, up = self.exit_gate.offset(x, y, z)
            if along < course.EXIT_DIST:
                return self._through(self.exit_gate, side, up)
            self.phase = "approach"

        if self.tracker.done:
            return self._go_home(x, y, z, link)
        gate = self.tracker.gates[self.gate_index]
        yaw_deg = course.world_yaw_to_ned_deg(gate.yaw)
        along, side, up = gate.offset(x, y, z)

        if self.phase == "approach":
            ax, ay = gate.point(-course.APPROACH_DIST)
            close = math.hypot(x - ax, y - ay) < self.APPROACH_RADIUS and abs(up) < self.APPROACH_RADIUS
            if not (close and abs(angle_diff(link.yaw(), gate.yaw)) < self.APPROACH_YAW):
                return self._fly_to(x, y, z, ax, ay, gate.z, yaw_deg)
            self.phase = "pass"
            self.route = ""

        if along > course.EXIT_DIST + 1.0:  # 통과 판정 없이 지나쳐 버렸으면 다시 정렬
            self.phase = "approach"
            return STOP

        self.source = "position"
        view = gate_vision.find_gate(frame) if -along > self.CAMERA_MIN_DIST else None
        if view and abs(view.distance + along) < self.CAMERA_MAX_ERROR:
            self.view, self.source = view, "camera"
            side, up = view.side, view.up
        return self._through(gate, side, up)

    def _through(self, gate, side, up):
        """게이트 방향으로 앞으로 가면서, 한가운데에서 벗어난 만큼(side, up) 되돌린다."""
        yaw_deg = course.world_yaw_to_ned_deg(gate.yaw)
        # 게이트 기준 (앞, 왼쪽, 위) 속도를 세계 좌표로 돌린 다음 NED로
        v_side = -clamp(self.CORRECTION_GAIN * side, self.MAX_CORRECTION)
        v_up = -clamp(self.CORRECTION_GAIN * up, self.MAX_CORRECTION)
        c, s = math.cos(gate.yaw), math.sin(gate.yaw)
        vx = self.PASS_SPEED * c - v_side * s
        vy = self.PASS_SPEED * s + v_side * c
        north, east, down = course.world_to_ned(vx, vy, v_up)
        return VelocityNedYaw(north, east, down, yaw_deg)

    def _fly_to(self, x, y, z, tx, ty, tz, yaw_deg):
        """(tx, ty, tz)로 간다. 곧장 가는 길에 건물·게이트가 있으면 먼저 그 위로 올라가서 넘어간다.
        (아무 데서나 3을 누르면 코스 경로가 아니라서 길이 막혀 있을 수 있다)"""
        over = None
        if math.hypot(tx - x, ty - y) > self.ROUTE_NEAR:
            over = course.route_height((x, y), (tx, ty))
        if over is None or over <= tz:
            self.route = ""
            return PositionNedYaw(*course.world_to_ned(tx, ty, tz), yaw_deg)
        if z < over - 0.5:
            self.route = f"climb to {over:.0f}m"
            return PositionNedYaw(*course.world_to_ned(x, y, over), yaw_deg)  # 그 자리에서 위로
        self.route = f"over obstacles at {over:.0f}m"
        return PositionNedYaw(*course.world_to_ned(tx, ty, over), yaw_deg)

    def _go_home(self, x, y, z, link):
        self.phase = "home"
        if self.home_yaw is None:  # 돌아가는 방향을 한 번만 정한다 (도착해서 빙글빙글 돌지 않게)
            self.home_yaw = math.atan2(course.START[1] - y, course.START[0] - x)
        return self._fly_to(x, y, z, course.START[0], course.START[1], course.CRUISE_Z,
                            course.world_yaw_to_ned_deg(self.home_yaw))

    def status(self):
        if self.phase == "home":
            return "course done -> back to start (L: land)" + (f" [{self.route}]" if self.route else "")
        total = len(self.tracker.gates)
        source = f" ({self.source})" if self.phase == "pass" else (f" [{self.route}]" if self.route else "")
        return f"gate {min(self.gate_index + 1, total)}/{total}: {self.phase}{source}"
