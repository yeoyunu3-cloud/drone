"""시뮬레이터 드론을 5m 이륙시키고 10초 뒤 착륙시킨다.

먼저 시뮬레이터(make px4_sitl gz_x500)를 켜두고 실행한다.
MAVSDK v4 (native API) 기준.
"""

import time

from mavsdk import ComponentType, Configuration, Mavsdk
from mavsdk.plugins.action import Action
from mavsdk.plugins.telemetry import Telemetry


def main():
    config = Configuration.create_with_component_type(ComponentType.GROUND_STATION)
    mavsdk = Mavsdk(config)
    mavsdk.add_any_connection("udpin://0.0.0.0:14540")

    print("드론 연결 기다리는 중...")
    drone = mavsdk.first_autopilot(10.0)
    if drone is None:
        raise SystemExit("드론을 찾지 못함. 시뮬레이터가 켜져 있는지 확인")
    print("연결됨")

    action = Action(drone)
    telemetry = Telemetry(drone)

    print("GPS 준비 기다리는 중...")
    while True:
        health = telemetry.health()
        if health.is_global_position_ok and health.is_home_position_ok:
            break
        time.sleep(1)
    print("준비 완료")

    print("시동")
    action.arm()

    print("이륙 (5m)")
    action.set_takeoff_altitude(5.0)
    action.takeoff()

    # 이륙이 끝나기 전에 착륙 명령을 보내면 PX4가 이륙을 마친 뒤 제자리 비행(Hold)으로 바뀌어 버린다
    while telemetry.position().relative_altitude_m < 4.5:
        time.sleep(0.5)

    for _ in range(10):
        print(f"  고도: {telemetry.position().relative_altitude_m:.1f} m")
        time.sleep(1)

    print("착륙")
    action.land()

    while telemetry.in_air():
        print(f"  고도: {telemetry.position().relative_altitude_m:.1f} m")
        time.sleep(1)
    print("착륙 완료")


if __name__ == "__main__":
    main()
