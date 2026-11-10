"""시뮬레이터 드론이 고도 5m에서 한 변 10m짜리 사각형을 그리며 날고 착륙한다.

먼저 시뮬레이터(run_sim.sh)를 켜두고 실행한다.
MAVSDK v4 (native API) 기준.

좌표는 NED: north(북, +), east(동, +), down(아래, +) 단위 m.
그래서 고도 5m는 down = -5.
"""

import math
import time

from mavsdk import ComponentType, Configuration, Mavsdk
from mavsdk.plugins.action import Action
from mavsdk.plugins.offboard import Offboard, PositionNedYaw
from mavsdk.plugins.telemetry import Telemetry

ALTITUDE_M = 5.0
SIDE_M = 10.0
ARRIVE_RADIUS_M = 0.5  # 목표 지점에서 이만큼 안에 들어오면 도착으로 본다
CORNER_TIMEOUT_S = 30.0


def connect():
    config = Configuration.create_with_component_type(ComponentType.GROUND_STATION)
    mavsdk = Mavsdk(config)
    mavsdk.add_any_connection("udpin://0.0.0.0:14540")

    print("드론 연결 기다리는 중...")
    drone = mavsdk.first_autopilot(10.0)
    if drone is None:
        raise SystemExit("드론을 찾지 못함. 시뮬레이터가 켜져 있는지 확인")
    print("연결됨")
    # mavsdk 객체가 사라지면 연결도 끊기므로 같이 돌려준다
    return mavsdk, drone


def wait_until_ready(telemetry):
    print("GPS 준비 기다리는 중...")
    while True:
        health = telemetry.health()
        if health.is_global_position_ok and health.is_home_position_ok:
            break
        time.sleep(1)
    print("준비 완료")


def fly_to(offboard, telemetry, north, east, yaw_deg):
    """목표 지점으로 가라고 명령하고, 도착할 때까지 기다린다."""
    offboard.set_position_ned(PositionNedYaw(north, east, -ALTITUDE_M, yaw_deg))

    start = time.time()
    while time.time() - start < CORNER_TIMEOUT_S:
        pos = telemetry.position_velocity_ned().position
        distance = math.hypot(pos.north_m - north, pos.east_m - east)
        print(f"  북 {pos.north_m:5.1f}  동 {pos.east_m:5.1f}  남은 거리 {distance:4.1f} m")
        if distance < ARRIVE_RADIUS_M:
            return
        time.sleep(1)
    print("  시간 초과, 다음 지점으로")


def main():
    mavsdk, drone = connect()
    action = Action(drone)
    telemetry = Telemetry(drone)
    offboard = Offboard(drone)

    wait_until_ready(telemetry)

    print("시동, 이륙")
    action.arm()
    action.set_takeoff_altitude(ALTITUDE_M)
    action.takeoff()

    while telemetry.position().relative_altitude_m < ALTITUDE_M - 0.5:
        print(f"  고도: {telemetry.position().relative_altitude_m:.1f} m")
        time.sleep(1)

    # 지금 위치를 사각형의 시작점으로 쓴다
    home = telemetry.position_velocity_ned().position
    n0, e0 = home.north_m, home.east_m

    # Offboard는 시작 전에 목표를 하나 보내둬야 켜진다
    offboard.set_position_ned(PositionNedYaw(n0, e0, -ALTITUDE_M, 0.0))
    offboard.start()
    print("Offboard 시작")

    # (북쪽, 동쪽, 바라볼 방향) 순서로 네 모서리를 돌고 출발점으로 돌아온다
    corners = [
        (n0 + SIDE_M, e0, 0.0),
        (n0 + SIDE_M, e0 + SIDE_M, 90.0),
        (n0, e0 + SIDE_M, 180.0),
        (n0, e0, 270.0),
    ]
    for i, (north, east, yaw) in enumerate(corners, start=1):
        print(f"{i}번 지점으로 이동")
        fly_to(offboard, telemetry, north, east, yaw)

    offboard.stop()
    print("Offboard 종료, 착륙")
    action.land()

    while telemetry.in_air():
        time.sleep(1)
    print("착륙 완료")


if __name__ == "__main__":
    main()
