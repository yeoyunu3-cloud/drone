"""충돌 방지: 드론이 건물, 게이트 틀, 사람, 땅에 부딪히지 않게 속도 명령을 줄인다.

실제 드론의 "충돌 방지(collision prevention)"와 같은 원리다.
장애물 쪽으로 가는 속도만 줄이고, 옆으로 비켜가거나 멀어지는 속도는 그대로 둔다.
  - 장애물에서 STOP_DIST(m) 앞에서는 그쪽으로 더 못 간다
  - 그보다 멀면 거리에 비례한 속도까지만 허용한다 (가까울수록 천천히)
장애물 위치는 course.py가 알려준다.
위치 명령(PositionNedYaw)도 그 방향으로 POSITION_CHECK_SPEED로 간다고 보고 검사해서,
막히면 줄인 속도 명령으로 바꾼다 (자동 모드 경로는 미리 검사하지만, 혹시 몰라서 한 번 더).
"""

import math

from mavsdk.plugins.offboard import PositionNedYaw, VelocityBodyYawspeed, VelocityNedYaw

import course

STOP_DIST = 1.0  # 드론 한가운데에서 장애물 표면까지 이 거리에서 멈춘다 (드론 반지름 약 0.4m)
# 허용 속도 = (거리 - STOP_DIST) × 이 값. 드론은 브레이크를 밟아도 조금 더 밀려가서
# (시험해보니 1.2일 때 0.4m 더 감) 일찍부터 천천히 줄인다
SLOW_GAIN = 0.8
CHECK_DIST = 4.0  # 이보다 먼 장애물은 신경 쓰지 않는다
MIN_ALT = 0.7  # 이 높이 아래로는 내려가지 않는다 (착륙은 L 키로)
POSITION_CHECK_SPEED = 2.0  # 위치 명령을 검사할 때 이 속도(m/s)로 간다고 본다

OBSTACLES = course.obstacles()


def closest_point(solid, x, y, z):
    """직육면체 위에서 점 (x, y, z)에 가장 가까운 점."""
    c, s = math.cos(solid.yaw), math.sin(solid.yaw)
    dx, dy = x - solid.x, y - solid.y
    # 직육면체 기준 좌표로 바꿔서 각 축 범위 안으로 잘라낸 다음 다시 세계 좌표로
    lx = max(-solid.size_x / 2, min(solid.size_x / 2, dx * c + dy * s))
    ly = max(-solid.size_y / 2, min(solid.size_y / 2, -dx * s + dy * c))
    lz = max(solid.z - solid.size_z / 2, min(solid.z + solid.size_z / 2, z))
    return solid.x + lx * c - ly * s, solid.y + lx * s + ly * c, lz


def limit_velocity(x, y, z, vx, vy, vz):
    """세계 좌표 속도 (vx, vy, vz)를 장애물에 부딪히지 않게 줄여서 돌려준다.
    두 번째 값은 장애물 때문에 속도를 줄였는지."""
    v = [vx, vy, vz]
    limited = False
    for solid in OBSTACLES:
        px, py, pz = closest_point(solid, x, y, z)
        d = math.dist((x, y, z), (px, py, pz))
        if d > CHECK_DIST or d < 1e-6:
            continue  # 멀거나, 이미 안에 들어가 있음
        n = ((px - x) / d, (py - y) / d, (pz - z) / d)  # 장애물 쪽 방향
        toward = v[0] * n[0] + v[1] * n[1] + v[2] * n[2]
        allowed = max(0.0, d - STOP_DIST) * SLOW_GAIN
        if toward > allowed:
            excess = toward - allowed
            v = [v[i] - excess * n[i] for i in range(3)]
            limited = True

    # 땅: 아래로 가는 속도를 높이에 맞춰 줄인다
    allowed_down = max(0.0, z - MIN_ALT) * SLOW_GAIN
    if -v[2] > allowed_down:
        v[2] = -allowed_down
        limited = True
    return v, limited


def filter_command(command, x, y, z, yaw):
    """Offboard 속도 명령을 충돌 방지에 맞게 고친다. yaw는 드론 방향(세계 좌표, 라디안)."""
    c, s = math.cos(yaw), math.sin(yaw)
    if isinstance(command, VelocityBodyYawspeed):
        # 드론 기준 (앞, 오른쪽) → 세계 좌표. 오른쪽 방향 = (sin yaw, -cos yaw)
        f, r = command.forward_m_s, command.right_m_s
        (vx, vy, vz), limited = limit_velocity(x, y, z, f * c + r * s, f * s - r * c, -command.down_m_s)
        if not limited:
            return command
        return VelocityBodyYawspeed(vx * c + vy * s, vx * s - vy * c, -vz, command.yawspeed_deg_s)
    if isinstance(command, VelocityNedYaw):
        (vx, vy, vz), limited = limit_velocity(x, y, z, command.east_m_s, command.north_m_s,
                                                -command.down_m_s)
        if not limited:
            return command
        return VelocityNedYaw(vy, vx, -vz, command.yaw_deg)
    if isinstance(command, PositionNedYaw):
        tx, ty, tz = course.ned_to_world(command.north_m, command.east_m, command.down_m)
        d = math.dist((x, y, z), (tx, ty, tz))
        if d < 0.05:
            return command
        speed = min(d, POSITION_CHECK_SPEED) / d
        (vx, vy, vz), limited = limit_velocity(x, y, z, (tx - x) * speed, (ty - y) * speed, (tz - z) * speed)
        if not limited:
            return command
        return VelocityNedYaw(vy, vx, -vz, command.yaw_deg)
    return command
