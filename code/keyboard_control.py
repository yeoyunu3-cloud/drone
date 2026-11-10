"""키보드로 드론을 직접 조종한다. 드론 카메라 영상 창에 키를 누르면 움직인다.

먼저 카메라 드론 시뮬레이터를 켜둔다: zsh ~/drone/run_sim.sh gz_x500_mono_cam
(카메라 없는 gz_x500으로 켜도 조종은 된다. 영상 창이 검은 화면일 뿐)
실행은 ~/drone/venv 환경에서 한다.

조종 방법 (영상 창을 클릭해서 선택한 상태에서 키를 누른다):
  T 이륙 / L 착륙 / Esc 착륙 후 종료
  W S 앞뒤   A D 왼쪽 오른쪽   ↑ ↓ 위 아래   ← → 왼쪽 오른쪽으로 돌기 (R F, Q E도 된다)
  Space 바로 멈춤

키를 누르고 있는 동안만 움직이고, 떼면 잠시 뒤 멈춘다.
"""

import time

import cv2
import numpy as np
from mavsdk import ComponentType, Configuration, Mavsdk
from mavsdk.plugins.action import Action
from mavsdk.plugins.offboard import Offboard, VelocityBodyYawspeed
from mavsdk.plugins.telemetry import Telemetry

from drone_camera import DroneCamera

ALTITUDE_M = 2.5
SPEED = 1.5  # 앞뒤, 좌우 속도 (m/s)
CLIMB_SPEED = 0.8  # 위아래 속도 (m/s)
YAW_SPEED = 45.0  # 회전 속도 (도/초)

# 키를 누르고 있으면 macOS가 같은 키를 반복해서 보내 준다.
# 반복이 시작되기까지 0.5초쯤 걸리므로, 마지막 키 입력 후 이 시간까지는 계속 움직인다.
KEY_HOLD_S = 0.6

# 화살표 키 (macOS OpenCV 창의 waitKeyEx 값)
UP, DOWN, LEFT, RIGHT = 0xF700, 0xF701, 0xF702, 0xF703

# 키 → (앞, 오른쪽, 아래, 회전) 방향. 아래(down)가 +라서 위로 가려면 -
MOVES = {
    ord("w"): (1, 0, 0, 0),
    ord("s"): (-1, 0, 0, 0),
    ord("a"): (0, -1, 0, 0),
    ord("d"): (0, 1, 0, 0),
    ord("r"): (0, 0, -1, 0),
    ord("f"): (0, 0, 1, 0),
    ord("q"): (0, 0, 0, -1),
    ord("e"): (0, 0, 0, 1),
    UP: (0, 0, -1, 0),
    DOWN: (0, 0, 1, 0),
    LEFT: (0, 0, 0, -1),
    RIGHT: (0, 0, 0, 1),
}
# 입력기가 한글이면 T가 ㅅ으로 들어온다. 두벌식 자판 위치대로 영문 키로 바꿔준다
KOREAN = {"ㅅ": "t", "ㅣ": "l", "ㅈ": "w", "ㄴ": "s", "ㅁ": "a", "ㅇ": "d",
          "ㄱ": "r", "ㄹ": "f", "ㅂ": "q", "ㄷ": "e"}
ESC = 27
STOP = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
WINDOW = "Keyboard control"
HELP = [
    "T takeoff   L land   Esc quit",
    "W/S fwd/back   A/D left/right",
    "Up/Down climb   Left/Right turn   Space stop",
]


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


def take_off(action, telemetry, offboard):
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

    # Offboard는 시작 전에 목표를 하나 보내둬야 켜진다
    offboard.set_velocity_body(STOP)
    offboard.start()
    print("조종 시작")


def land(action, telemetry, offboard):
    print("착륙")
    offboard.set_velocity_body(STOP)
    offboard.stop()
    action.land()
    while telemetry.in_air():
        time.sleep(1)
    print("착륙 완료")


def draw(frame, flying, velocity, altitude):
    """영상 위에 키 안내와 현재 상태를 적는다."""
    for i, line in enumerate(HELP):
        cv2.putText(frame, line, (10, 30 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    state = "FLYING" if flying else "LANDED (T: takeoff)"
    status = (f"{state}  alt {altitude:.1f}m  fwd {velocity.forward_m_s:+.1f}  "
              f"right {velocity.right_m_s:+.1f}  up {-velocity.down_m_s:+.1f}  "
              f"turn {velocity.yawspeed_deg_s:+.0f}")
    cv2.putText(frame, status, (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)


def main():
    camera = DroneCamera()
    mavsdk, drone = connect()
    action = Action(drone)
    telemetry = Telemetry(drone)
    offboard = Offboard(drone)

    print("영상 창을 클릭한 뒤 T를 눌러 이륙")
    flying = False
    velocity = STOP
    last_key_time = 0.0

    while True:
        frame = camera.read()
        if frame is None:
            frame = np.zeros((720, 960, 3), np.uint8)  # 카메라가 없으면 검은 화면
        frame = cv2.resize(frame, (960, 720))
        draw(frame, flying, velocity, telemetry.position().relative_altitude_m)
        cv2.imshow(WINDOW, frame)

        key = cv2.waitKeyEx(50)  # 50ms마다 = 1초에 20번 속도 명령. 안 누르면 -1
        if 0 <= key < 0x110000:
            char = KOREAN.get(chr(key), chr(key))
            if "A" <= char <= "Z":
                char = char.lower()  # Caps Lock이 켜져 있어도 소문자로
            key = ord(char)
        known = key in MOVES or key in (ESC, ord(" "), ord("t"), ord("l"))
        if key != -1 and not known:
            print(f"  조종 키가 아님: {key}")  # 키가 안 먹을 때 무엇이 들어오는지 확인용

        if key == ESC:
            break
        if key == ord("t") and not flying:
            take_off(action, telemetry, offboard)
            flying = True
        elif key == ord("l") and flying:
            land(action, telemetry, offboard)
            flying = False
            velocity = STOP
        elif key == ord(" "):
            velocity = STOP
        elif key in MOVES:
            forward, right, down, yaw = MOVES[key]
            velocity = VelocityBodyYawspeed(forward * SPEED, right * SPEED,
                                            down * CLIMB_SPEED, yaw * YAW_SPEED)
            last_key_time = time.time()

        if time.time() - last_key_time > KEY_HOLD_S:
            velocity = STOP

        # Offboard 중에는 명령을 계속 보내야 한다. 끊기면 드론이 안전모드로 바뀐다
        if flying:
            offboard.set_velocity_body(velocity)

    if flying:
        land(action, telemetry, offboard)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
