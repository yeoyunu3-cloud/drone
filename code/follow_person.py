"""드론 카메라로 사람을 찾아서 따라간다. (Phase 5 최종 목표)

먼저 카메라 드론 시뮬레이터를 켜고 사람 모델을 놓아둔다 (계획 파일 Phase 4 참고).
실행은 ~/drone/venv 환경에서 한다.

조종 방법 (비례 제어):
  - 사람이 화면 오른쪽에 있으면(error_x > 0) 오른쪽으로 돈다. 벗어난 만큼 빨리 돈다.
  - 사람이 작게 보이면(멀면) 앞으로, 크게 보이면(가까우면) 뒤로 간다.
  - 사람을 놓치면 제자리에서 멈춘다.

q 키(영상 창) 또는 Ctrl+C로 끝내면 착륙한다.
python follow_person.py --headless 로 실행하면 영상 창 없이 숫자만 출력한다.
"""

import argparse
import time

import cv2
from mavsdk import ComponentType, Configuration, Mavsdk
from mavsdk.plugins.action import Action
from mavsdk.plugins.offboard import Offboard, VelocityBodyYawspeed
from mavsdk.plugins.telemetry import Telemetry
from ultralytics import YOLO

from drone_camera import DroneCamera
from vision import MODEL, find_target

ALTITUDE_M = 2.5

# 회전: error_x(-1 ~ 1)에 곱해서 회전 속도(도/초)로 바꾼다
YAW_GAIN = 40.0
MAX_YAW_SPEED = 30.0

# 전진: 사람 박스 높이가 TARGET_HEIGHT가 되도록 앞뒤로 움직인다 (약 4m 거리)
TARGET_HEIGHT = 0.24
FORWARD_GAIN = 6.0
MAX_FORWARD_SPEED = 1.5
MAX_BACKWARD_SPEED = 0.5

DEAD_ZONE_X = 0.05  # 이 정도 벗어난 건 무시 (제자리에서 떨지 않게)
DEAD_ZONE_HEIGHT = 0.02


def clamp(value, low, high):
    return max(low, min(high, value))


def compute_velocity(target):
    """목표가 화면 어디에 있는지를 드론 속도 명령으로 바꾼다."""
    if target is None:
        return VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)

    error_x = 0.0 if abs(target.error_x) < DEAD_ZONE_X else target.error_x
    yaw_speed = clamp(YAW_GAIN * error_x, -MAX_YAW_SPEED, MAX_YAW_SPEED)

    error_h = TARGET_HEIGHT - target.height
    if abs(error_h) < DEAD_ZONE_HEIGHT:
        error_h = 0.0
    forward = clamp(FORWARD_GAIN * error_h, -MAX_BACKWARD_SPEED, MAX_FORWARD_SPEED)

    return VelocityBodyYawspeed(forward, 0.0, 0.0, yaw_speed)


def connect():
    config = Configuration.create_with_component_type(ComponentType.GROUND_STATION)
    mavsdk = Mavsdk(config)
    mavsdk.add_any_connection("udpin://0.0.0.0:14540")

    print("드론 연결 기다리는 중...")
    drone = mavsdk.first_autopilot(10.0)
    if drone is None:
        raise SystemExit("드론을 찾지 못함. 시뮬레이터가 켜져 있는지 확인")
    print("연결됨")
    return mavsdk, drone


def take_off(action, telemetry):
    print("GPS 준비 기다리는 중...")
    while True:
        health = telemetry.health()
        if health.is_global_position_ok and health.is_home_position_ok:
            break
        time.sleep(1)

    print(f"시동, 이륙 ({ALTITUDE_M}m)")
    action.arm()
    action.set_takeoff_altitude(ALTITUDE_M)
    action.takeoff()
    while telemetry.position().relative_altitude_m < ALTITUDE_M - 0.3:
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="영상 창 없이 실행")
    parser.add_argument("--duration", type=float, default=0, help="이 시간(초)이 지나면 착륙. 0이면 계속")
    args = parser.parse_args()

    model = YOLO(MODEL)
    camera = DroneCamera()
    mavsdk, drone = connect()
    action = Action(drone)
    telemetry = Telemetry(drone)
    offboard = Offboard(drone)

    take_off(action, telemetry)

    # Offboard는 시작 전에 목표를 하나 보내둬야 켜진다
    offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    offboard.start()
    print("따라가기 시작 (q 또는 Ctrl+C로 착륙)")

    start = time.time()
    last_print = 0.0
    try:
        while not args.duration or time.time() - start < args.duration:
            frame = camera.read()
            if frame is None:
                continue

            target = find_target(model, frame)
            velocity = compute_velocity(target)
            offboard.set_velocity_body(velocity)

            if time.time() - last_print > 0.5:
                last_print = time.time()
                if target:
                    print(f"  error_x {target.error_x:+.2f}  height {target.height:.2f}  "
                          f"→ 전진 {velocity.forward_m_s:+.2f} m/s  회전 {velocity.yawspeed_deg_s:+.1f} °/s")
                else:
                    print("  사람 없음 → 정지")

            if not args.headless:
                cv2.imshow("Follow person (q: land)", cv2.resize(frame, (960, 720)))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass

    print("착륙")
    offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    offboard.stop()
    action.land()
    while telemetry.in_air():
        time.sleep(1)
    print("착륙 완료")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
