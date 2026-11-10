"""PX4 드론과의 연결, 이륙, 착륙, Offboard 명령을 한 곳에 모은 것. (app.py가 쓴다)

명령은 MAVSDK Offboard 목표 세 가지 중 하나를 그대로 넘긴다:
  PositionNedYaw        이 위치로 가라
  VelocityNedYaw        동서남북 기준 속도
  VelocityBodyYawspeed  드론 기준 속도 (앞, 오른쪽, 아래, 회전)

이륙과 착륙은 기다리지 않는다 (기다리는 동안 조종 창이 멈추지 않게).
start_take_off / start_land로 시작하고, 조종 루프가 매번 update()를 불러서 다음 단계로 넘긴다.
상태: landed(시동 꺼짐) → taking_off → flying → landing → landed
PX4가 거절하거나 오래 걸리면 예외 대신 이유를 글로 돌려준다.
"""

import math
import time

from mavsdk import ComponentType, Configuration, Mavsdk
from mavsdk.plugins.action import Action
from mavsdk.plugins.offboard import Offboard, PositionNedYaw, VelocityBodyYawspeed, VelocityNedYaw
from mavsdk.plugins.telemetry import FlightMode, Telemetry

import course

STOP = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)

# 시간은 time.monotonic()으로 잰다 (Mac이 잠자기에 들어간 동안은 세지 않아서, 깨어나자마자 시간 초과로 보지 않는다)
TAKEOFF_TIMEOUT_S = 20  # 이 안에 목표 높이에 못 가면 포기하고 내려온다
LAND_TIMEOUT_S = 60  # 이 안에 착륙이 안 끝나면 모터를 끈다
DISARM_WAIT_S = 4  # 바닥에 닿고 PX4가 스스로 시동을 끄기를 이만큼 기다린다
MAX_TAKEOFF_TILT_DEG = 30  # 이보다 기울어 있으면 넘어진 것: 이륙하지 않는다
OFFBOARD_RETRY_S = 1.0  # PX4가 Offboard에서 빠져나가 있으면 이만큼 뒤에 다시 켠다


def open_mavsdk():
    """드론 신호를 받을 준비. 프로그램에서 한 번만 만든다 (같은 포트를 두 번 열 수 없다)."""
    config = Configuration.create_with_component_type(ComponentType.GROUND_STATION)
    mavsdk = Mavsdk(config)
    mavsdk.add_any_connection("udpin://0.0.0.0:14540")
    return mavsdk


def wait_for_drone(mavsdk, timeout_s):
    """드론이 나타날 때까지 기다린다. 시간 안에 안 나타나면 None."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        drone = mavsdk.first_autopilot(5.0)
        if drone is not None:
            return drone
    return None


def _reason(error):
    """MAVSDK 예외를 짧은 글로."""
    return str(error).strip() or type(error).__name__


class DroneLink:
    def __init__(self, drone):
        self.action = Action(drone)
        self.telemetry = Telemetry(drone)
        self.offboard = Offboard(drone)
        self.state = "landed"
        self._deadline = 0.0
        self._alt = 0.0
        self._ground_since = None
        self._not_offboard_since = None
        # PX4 좌표 원점이 Gazebo 원점과 다를 때의 차이 (세계 좌표 = PX4 좌표 + offset).
        # 처음엔 같지만, 위치 추정(EKF)을 다시 시작하면 그 자리가 PX4의 새 원점이 된다
        self.offset = (0.0, 0.0, 0.0)

    @property
    def flying(self):
        """Offboard 명령을 받는 중 (이륙·착륙 도중은 아님)."""
        return self.state == "flying"

    @property
    def armed(self):
        """시동이 걸려 있다고 보는 중 (이륙, 비행, 착륙 도중)."""
        return self.state != "landed"

    def ready(self):
        health = self.telemetry.health()
        # is_armable: PX4가 시동을 허락하는 상태 (켜진 직후 몇 초는 아직 아니다)
        return health.is_global_position_ok and health.is_home_position_ok and health.is_armable

    def start_take_off(self, altitude_m):
        """그 자리에서 altitude_m만큼 이륙을 시작한다. 못 하면 이유(글)를, 시작했으면 None."""
        if self.state != "landed":
            return "이미 날고 있음"
        if not self.ready():
            return "GPS 준비 중. 잠시 뒤 다시 T"
        if self.tilt_deg() > MAX_TAKEOFF_TILT_DEG:
            return "드론이 넘어져 있음. X로 먼저 세우세요"
        try:
            self.action.set_takeoff_altitude(altitude_m)  # 이륙하는 자리 기준 (지붕 위면 지붕에서)
            self.action.arm()
            self.action.takeoff()
        except Exception as e:
            self._disarm()
            return f"이륙 실패 ({_reason(e)}). 잠시 뒤 다시 T"
        self.state = "taking_off"
        self._alt = altitude_m
        self._deadline = time.monotonic() + TAKEOFF_TIMEOUT_S
        return None

    def start_land(self):
        """그 자리에 착륙을 시작한다. 못 하면 이유(글), 시작했으면 None."""
        if self.state not in ("flying", "taking_off"):
            return None
        try:
            self.offboard.set_velocity_body(STOP)
            self.offboard.stop()
        except Exception:
            pass  # Offboard가 이미 꺼져 있어도 착륙은 한다
        try:
            self.action.land()
        except Exception as e:
            self.kill()
            return f"착륙 명령 실패 ({_reason(e)}) → 모터를 껐습니다"
        self.state = "landing"
        self._deadline = time.monotonic() + LAND_TIMEOUT_S
        self._ground_since = None
        return None

    def update(self):
        """조종 루프에서 매번 부른다. 이륙·착륙을 다음 단계로 넘긴다. 알릴 일이 있으면 글을 돌려준다."""
        now = time.monotonic()
        if self.state == "taking_off":
            # relative_altitude는 시동을 건 자리(홈) 기준이라 지붕 위에서 떠도 맞다
            if self.telemetry.position().relative_altitude_m >= self._alt - 0.3:
                try:
                    # Offboard는 시작 전에 목표를 하나 보내둬야 켜진다
                    self.offboard.set_velocity_body(STOP)
                    self.offboard.start()
                except Exception as e:
                    self.state = "flying"  # start_land가 받아 주게
                    return f"Offboard 시작 실패 ({_reason(e)}) → " + (self.start_land() or "착륙")
                self.state = "flying"
                self._not_offboard_since = None
            elif now > self._deadline:
                return "이륙이 끝나지 않아서 착륙 → " + (self.start_land() or "착륙 중")
        elif self.state == "landing":
            if not self.telemetry.armed():
                self.state = "landed"
                return "착륙 완료"
            if not self.telemetry.in_air():
                self._ground_since = self._ground_since or now
                if now - self._ground_since > DISARM_WAIT_S:
                    self._disarm()  # 바닥에 닿았는데 PX4가 시동을 안 끄면 직접 끈다
            if now > self._deadline:
                self.kill()
                return "착륙이 끝나지 않아서 모터를 껐습니다"
        elif self.state == "flying":
            if not self.telemetry.armed():
                self.state = "landed"
                return "시동이 꺼졌습니다"
            # PX4가 Offboard에서 빠져나가 있으면(명령이 잠깐 끊긴 경우 등) 다시 켠다
            if self.telemetry.flight_mode() != FlightMode.OFFBOARD:
                self._not_offboard_since = self._not_offboard_since or now
                if now - self._not_offboard_since > OFFBOARD_RETRY_S:
                    self._not_offboard_since = None
                    try:
                        self.offboard.set_velocity_body(STOP)
                        self.offboard.start()
                        return "Offboard가 꺼져 있어서 다시 켰습니다"
                    except Exception as e:
                        return f"Offboard 다시 켜기 실패 ({_reason(e)})"
            else:
                self._not_offboard_since = None
        return None

    def land_and_wait(self, timeout_s=LAND_TIMEOUT_S):
        """착륙이 끝날 때까지 기다린다 (프로그램을 끌 때)."""
        message = self.start_land()
        if message:
            print(message)
        deadline = time.monotonic() + timeout_s
        while self.state != "landed" and time.monotonic() < deadline:
            message = self.update()
            if message:
                print(message)
            time.sleep(0.2)
        if self.state != "landed":
            self.kill()

    def _disarm(self):
        try:
            self.action.disarm()
        except Exception:
            pass

    def kill(self):
        """모터를 바로 끈다 (부딪혀서 넘어질 때). 공중이면 그대로 떨어진다."""
        try:
            self.offboard.stop()
        except Exception:
            pass  # 이미 Offboard가 꺼졌어도 모터는 꺼야 한다
        try:
            self.action.kill()
        except Exception:
            self._disarm()
        self.state = "landed"

    def tilt_deg(self):
        """드론이 기울어진 정도 (앞뒤, 좌우 중 큰 쪽, 도)."""
        a = self.telemetry.attitude_euler()
        return max(abs(a.roll_deg), abs(a.pitch_deg))

    def send(self, command):
        """Offboard 명령 하나를 보낸다. 날고 있을 때만 보낸다."""
        if not self.flying:
            return
        try:
            if isinstance(command, PositionNedYaw):
                # 위치 명령은 세계 좌표로 만든 것이라 PX4 좌표로 바꿔서 보낸다
                ox, oy, oz = self.offset
                x, y, z = course.ned_to_world(command.north_m, command.east_m, command.down_m)
                north, east, down = course.world_to_ned(x - ox, y - oy, z - oz)
                self.offboard.set_position_ned(PositionNedYaw(north, east, down, command.yaw_deg))
            elif isinstance(command, VelocityNedYaw):
                self.offboard.set_velocity_ned(command)
            else:
                self.offboard.set_velocity_body(command)
        except Exception:
            pass  # 한 번 못 보내도 다음 차례(1/30초 뒤)에 또 보낸다

    def _px4_position(self):
        p = self.telemetry.position_velocity_ned().position
        return course.ned_to_world(p.north_m, p.east_m, p.down_m)

    def position(self):
        """현재 위치 (Gazebo 세계 좌표 x, y, z)."""
        x, y, z = self._px4_position()
        ox, oy, oz = self.offset
        return x + ox, y + oy, z + oz

    def calibrate(self, x, y, z):
        """진짜 위치(Gazebo)가 (x, y, z)일 때 PX4 좌표와의 차이를 다시 잰다."""
        px, py, pz = self._px4_position()
        self.offset = (x - px, y - py, z - pz)

    def speed(self):
        """현재 수평 속도 (m/s)."""
        v = self.telemetry.position_velocity_ned().velocity
        return math.hypot(v.north_m_s, v.east_m_s)

    def yaw(self):
        """현재 방향 (세계 좌표 yaw, 라디안. 동쪽이 0, 반시계가 +)."""
        return math.radians(90.0 - self.telemetry.heading().heading_deg)
