"""AI 드론 시뮬레이터를 명령 하나로 실행한다. (Phase 7)

    cd ~/drone/code && source ../venv/bin/activate && python app.py

1. 도시 맵(course.py)으로 시뮬레이터를 켜고  (1분쯤 걸린다. 기록은 ~/drone/sim.log)
2. 드론 카메라 창을 띄운 다음
3. 키로 모드를 바꿔가며 날린다. 창을 끄면(Esc) 시뮬레이터도 같이 꺼진다.

키 (영상 창을 클릭해서 선택한 상태에서. 한글 입력기여도 된다):
  T 이륙   L 착륙 (지붕 가장자리·기둥 위처럼 걸치는 자리면 가까운 평평한 곳으로 옮겨서. 한 번 더 L이면 그 자리에)
  Esc 착륙 후 종료
  1 직접 조종   2 사람 따라가기   3 코스 자동 통과 (착륙 상태면 알아서 이륙)
  C 코스 기록 초기화 (게이트 1번부터 다시)
  V 화면 시점 바꾸기: 드론 카메라 → 추격 시점 → 위에서 보기
  X 넘어지거나 걸린 드론을 똑바로 세우기 (날고 있으면 모터를 끄고. 지붕 위면 지붕에, 가장자리·벽 속·막대 밑이면 가까운 넓은 곳에)
  직접 조종: W/S 앞뒤  A/D 좌우  ↑/↓ 위아래  ←/→ 회전  Space 멈춤 (R/F, Q/E도 된다)

안전장치: 장애물 0.8m 앞, 땅 0.7m 위에서 자동으로 멈추고(safety.py),
그래도 부딪혀서 60도 넘게 기울면(이륙·착륙 중에도) 모터를 꺼서 날뛰지 않게 한다.
이륙·착륙은 기다리지 않아서 창이 멈추지 않고, PX4가 거절하면 프로그램이 꺼지지 않고 이유를 알려 준다.

다른 실행 방법:
  python app.py --no-sim             이미 켜 둔 시뮬레이터에 붙는다 (PX4_GZ_WORLD=city 로 켠 것)
  python app.py --auto --headless    창 없이 코스 자동 통과만 하고 끝낸다 (테스트용)
"""

import argparse
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

import course
import gate_vision
import landing
import make_world
import safety
from drone_camera import DroneCamera, camera_topic
from drone_link import STOP, DroneLink, open_mavsdk, wait_for_drone
from held_keys import HeldKeys
from keyboard_control import KOREAN
from modes import FollowMode, ManualMode, RaceMode, angle_diff
from view_camera import ViewCamera
from vision import MODEL

HOME = Path.home() / "drone"
RUN_SIM = HOME / "run_sim.sh"
SIM_LOG = HOME / "sim.log"
WORLD = make_world.WORLD_NAME
VEHICLE = "gz_x500_mono_cam"
DRONE_MODEL = "x500_mono_cam_0"  # Gazebo 속 드론 이름
CRASH_TILT_DEG = 60  # 이보다 기울면 부딪혀서 넘어지는 중으로 보고 모터를 끈다
UPRIGHT_LIFT = 0.3  # X로 세울 때 바닥에서 이만큼 띄워 놓는다 (떨어지며 자리 잡게)
CALIBRATE_WAIT_S = 3  # 위치 추정을 다시 시작하고 준비된 뒤 이만큼 더 기다렸다가 좌표 차이를 잰다
UPRIGHT_HEADROOM = 3.5  # 세운 자리 위로 이만큼 비어 있어야 다시 이륙할 수 있다 (이륙 높이 2.5m + 여유)

CONNECT_TIMEOUT_S = 150  # 시뮬레이터가 켜지고 드론이 나타나기까지 기다리는 시간
ALTITUDE_M = 2.5
LOOP_HZ = 30  # 1초에 몇 번 명령을 보내고 화면을 그리나 (드론 카메라도 1초에 30장)
KEY_WAIT_MIN_S = 0.012  # macOS에서 waitKeyEx는 1ms를 달라고 해도 11ms쯤 기다린다
KEY_WAIT_EXTRA_MS = 4  # 그리고 달라는 시간보다 3~4ms 더 기다린다
ESC = 27
WINDOW = "AI drone (Esc: quit)"
VIEW_SIZE = (960, 720)
VIEWS = ["drone", "chase", "top"]  # V 키로 돌아가며 바꾸는 화면 시점
PIP_SIZE = (320, 240)  # 다른 시점일 때 구석에 작게 띄우는 드론 카메라 크기
YELLOW, RED, WHITE = (0, 255, 255), (0, 0, 255), (255, 255, 255)

# 한글 입력기일 때: 조종 키(keyboard_control.KOREAN) + 이 프로그램에서 더 쓰는 키
APP_KOREAN = {**KOREAN, "ㅊ": "c", "ㅍ": "v", "ㅌ": "x"}


class Simulator:
    """PX4 + Gazebo 시뮬레이터를 도시 맵으로 켜고 끈다."""

    def __init__(self):
        self.process = None

    def start(self):
        # 전에 비정상으로 끝나서 남은 시뮬레이터가 있으면 정리한다 (Gazebo만 죽고 PX4가 남는 경우 등)
        if subprocess.run(["pgrep", "-f", "px4_sitl_default/bin/px4"], capture_output=True).returncode == 0:
            print("남아 있던 시뮬레이터 정리하는 중...")
            kill_leftovers()
        print(f"시뮬레이터 켜는 중... (1분쯤 걸림, 기록: {SIM_LOG})")
        log = open(SIM_LOG, "w")
        # 끌 때 한꺼번에 끄려고 새 프로세스 그룹으로 띄운다
        self.process = subprocess.Popen(
            ["zsh", str(RUN_SIM), VEHICLE],
            env=dict(os.environ, PX4_GZ_WORLD=WORLD),
            stdin=subprocess.PIPE,  # PX4 셸(pxh>)이 입력이 끊겼다고 꺼지지 않게 열어만 둔다
            stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def px4_command(self, line):
        """PX4 콘솔(pxh>)에 명령을 넣는다. 이 프로그램이 켠 시뮬레이터일 때만 된다."""
        if self.process is None or self.process.poll() is not None:
            return False
        try:
            self.process.stdin.write((line + "\n").encode())
            self.process.stdin.flush()
            return True
        except (BrokenPipeError, OSError):
            return False

    def stop(self):
        if self.process is None:
            return
        print("시뮬레이터 끄는 중...")
        try:
            os.killpg(self.process.pid, signal.SIGINT)
            self.process.wait(timeout=15)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        kill_leftovers()  # 남은 Gazebo 창까지 확실히
        self.process = None


def kill_leftovers():
    for pattern in ("make px4_sitl", "px4_sitl_default/bin/px4", "gz sim"):
        subprocess.run(["pkill", "-9", "-f", pattern])
    time.sleep(1)


def sim_log_tail(lines=15):
    try:
        return "\n".join(SIM_LOG.read_text(errors="replace").splitlines()[-lines:])
    except FileNotFoundError:
        return "(기록 없음)"


def find_drone(mavsdk, sim):
    """드론을 기다린다. 시뮬레이터가 켜지다 멈추는 일이 가끔 있어서, 그러면 한 번 다시 켠다."""
    print("드론 연결 기다리는 중...")
    drone = wait_for_drone(mavsdk, CONNECT_TIMEOUT_S)
    if drone is None and sim is not None:
        print("시뮬레이터가 제대로 안 켜져서 한 번 다시 켬")
        sim.stop()
        sim.start()
        drone = wait_for_drone(mavsdk, CONNECT_TIMEOUT_S)
    if drone is None:
        raise SystemExit(f"드론을 찾지 못함. 시뮬레이터 기록({SIM_LOG}) 마지막 부분:\n{sim_log_tail()}")
    return drone


def normalize_key(key):
    """한글 입력기, Caps Lock이어도 같은 영문 소문자 키로."""
    if 0 <= key < 0x110000:
        char = APP_KOREAN.get(chr(key), chr(key))
        return ord(char.lower()) if len(char.lower()) == 1 else key
    return key


class CourseClock:
    """게이트 통과 기록 시간."""

    def __init__(self):
        self.start = None
        self.finish = None

    def reset(self, running):
        self.start = time.time() if running else None
        self.finish = None

    def elapsed(self):
        if self.start is None:
            return 0.0
        return (self.finish or time.time()) - self.start


def next_gate_hint(tracker, link, x, y):
    if tracker.done:
        return "course done"
    gate = tracker.gates[tracker.next]
    turn = math.degrees(angle_diff(math.atan2(gate.y - y, gate.x - x), link.yaw()))
    distance = math.hypot(gate.x - x, gate.y - y)
    if abs(turn) < 5:
        return f"next gate {distance:.0f}m ahead"
    side = "left" if turn > 0 else "right"
    return f"next gate {distance:.0f}m, turn {side} {abs(turn):.0f}deg"


def compose_view(frame, view_name, view_cam):
    """창에 띄울 화면. 드론 카메라가 아닌 시점이면 구석에 드론 카메라를 작게 붙인다."""
    if view_name == "drone":
        return cv2.resize(frame, VIEW_SIZE)
    main = view_cam.read()
    if main is None:
        main = np.zeros((VIEW_SIZE[1], VIEW_SIZE[0], 3), np.uint8)
        cv2.putText(main, "loading view...", (380, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.9, WHITE, 2)
    view = cv2.resize(main, VIEW_SIZE)
    pip = cv2.resize(frame, PIP_SIZE)
    x0, y0 = VIEW_SIZE[0] - PIP_SIZE[0] - 10, 110  # 오른쪽 위, 안내 글자 아래
    view[y0:y0 + PIP_SIZE[1], x0:x0 + PIP_SIZE[0]] = pip
    cv2.rectangle(view, (x0, y0), (x0 + PIP_SIZE[0], y0 + PIP_SIZE[1]), WHITE, 2)
    return view


def status_line(mode, link, lander):
    if lander.active:
        return "moving to a flat landing spot..."
    return {
        "flying": mode.status(),
        "taking_off": "taking off...",
        "landing": "landing...",
    }.get(link.state, "LANDED - T: takeoff (fell over? X first)")


def take_off_height(x, y, z):
    """이륙 높이. 위가 막혀 있으면(게이트 막대 밑 등) 그 아래까지만. 못 뜨면 0."""
    room = landing.headroom(x, y, z) - 1.0  # 위의 물체에서 1m 아래까지만
    return 0.0 if room < 1.0 else min(ALTITUDE_M, room)


def upright_spot(x, y, z):
    """넘어진 드론을 세울 자리 (x, y, 바닥 높이). 가장자리, 벽 속, 막대 밑이면 가까운 넓은 곳으로."""
    return landing.find_spot(x, y, z + 0.3, check_path=False, min_headroom=UPRIGHT_HEADROOM)


def draw_hud(view, mode, link, lander, tracker, clock, x, y, z, view_name="drone"):
    lines = [
        f"MODE {mode.name}   [1] manual [2] follow [3] auto   VIEW {view_name}",
        "T takeoff  L land  C reset course  V view  X upright  Esc quit",
        status_line(mode, link, lander),
    ]
    for i, line in enumerate(lines):
        cv2.putText(view, line, (10, 30 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, YELLOW, 2)
    bottom = (f"alt {z:.1f}m   gates {tracker.next}/{len(tracker.gates)}   "
              f"time {clock.elapsed():.1f}s   {next_gate_hint(tracker, link, x, y)}")
    cv2.putText(view, bottom, (10, view.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, RED, 2)


def run(args, sim):
    model = YOLO(MODEL)
    camera = DroneCamera(camera_topic(WORLD))
    view_cam = ViewCamera(WORLD)
    view_name = VIEWS[0]
    mavsdk = open_mavsdk()  # 이 객체가 살아 있어야 연결이 유지된다
    link = DroneLink(find_drone(mavsdk, sim))
    print("연결됨")

    tracker = course.GateTracker()
    clock = CourseClock()
    race = RaceMode(tracker)
    modes = {ord("1"): ManualMode(HeldKeys()), ord("2"): FollowMode(model), ord("3"): race}
    mode = modes[ord("1")]
    lander = landing.Lander()
    calibrate_after = None  # 위치 추정을 다시 시작했으면, 이 시각 뒤에 PX4 좌표와 세계 좌표를 다시 맞춘다

    def take_off():
        """이륙을 시작한다. 시작했으면 True."""
        if calibrate_after is not None:
            print("위치 추정 다시 맞추는 중. 잠시 뒤 T")
            return False
        # 위가 막혔는지는 Gazebo 진짜 위치로 본다 (PX4 위치는 넘어지거나 옮겨지면 틀릴 수 있다)
        pose = view_cam.true_pose(DRONE_MODEL)
        x, y, z = pose[:3] if pose else link.position()
        height = take_off_height(x, y, z)
        if height == 0.0:
            print("위가 막혀서 이륙할 수 없음. X로 넓은 곳에 세우세요")
            return False
        message = link.start_take_off(height)
        if message:
            print(message)
            return False
        print(f"이륙 ({height:.1f}m)")
        if tracker.next == 0:
            clock.reset(running=True)  # 기록은 이륙하는 순간부터 잰다
        return True

    def start_race():
        nonlocal mode
        tracker.reset()
        if not link.armed and not take_off():
            return
        clock.reset(running=True)
        mode = race
        mode.start(link)
        print("코스 자동 통과 시작")

    if args.auto:
        start_race()
    print("영상 창을 클릭하고 키를 누르세요 (T 이륙, 1/2/3 모드, Esc 종료)")

    started = time.time()
    last_print = 0.0
    period = 1 / LOOP_HZ
    while True:
        tick = time.time()
        frame = camera.read()
        if frame is None:
            frame = np.zeros((960, 1280, 3), np.uint8)  # 카메라 영상이 아직 없으면 검은 화면
        x, y, z = link.position()

        if tracker.update(x, y, z):
            print(f"  게이트 {tracker.next}/{len(tracker.gates)} 통과  {clock.elapsed():.1f}초")
            if tracker.done:
                clock.finish = time.time()
                print(f"코스 완주! 기록 {clock.elapsed():.1f}초")

        if calibrate_after is not None and link.ready() and time.time() > calibrate_after:
            pose = view_cam.true_pose(DRONE_MODEL)
            if pose is not None:
                link.calibrate(*pose[:3])
                calibrate_after = None
                print("위치 추정 준비 완료. T로 이륙하세요")
            x, y, z = link.position()
        message = link.update()  # 이륙·착륙 다음 단계
        if message:
            print(message)
        # 시동이 걸려 있는 동안(이륙·비행·착륙 중) 부딪혀서 넘어지면 모터를 꺼서 날뛰지 않게
        tilt = link.tilt_deg() if link.armed else 0.0
        if tilt > CRASH_TILT_DEG:
            link.kill()
            lander.cancel()
            print(f"충돌! ({tilt:.0f}도 기울어짐, 위치 {x:.1f}, {y:.1f}, {z:.1f}) "
                  "모터를 껐습니다. X로 드론을 세우고 T로 다시 이륙하세요")

        command, message = lander.step(link, x, y)  # 착륙 자리로 옮기는 중이면 그게 먼저
        if message:
            print(message)
        if command is None:
            command = mode.step(frame, link) if link.flying else STOP
            command = safety.filter_command(command, x, y, z, link.yaw())
        link.send(command)
        if mode is race:
            gate_vision.draw_gate(frame, race.view)

        if time.time() - last_print > 1.0:
            last_print = time.time()
            print(f"  [{mode.name}] {mode.status()}  위치 ({x:.1f}, {y:.1f}, {z:.1f})")

        keys = []
        if args.headless:
            time.sleep(max(0.0, period - (time.time() - tick)))
        else:
            if view_name != "drone":
                view_cam.move(view_name, x, y, z, link.yaw())
            view = compose_view(frame, view_name, view_cam)
            draw_hud(view, mode, link, lander, tracker, clock, x, y, z, view_name)
            cv2.imshow(WINDOW, view)
            # 다음 차례까지 남은 시간 동안 키를 모두 받는다. 키를 누르고 있으면 macOS가 1초에 30번 넘게
            # 보내는데, 한 번에 하나씩만 꺼내면 쌓여서 떼고 나서도 한참 움직인다
            while True:
                wait_ms = int((period - (time.time() - tick)) * 1000) - KEY_WAIT_EXTRA_MS
                key = cv2.waitKeyEx(max(1, wait_ms))
                if key != -1 and normalize_key(key) not in keys:
                    keys.append(normalize_key(key))
                if period - (time.time() - tick) < KEY_WAIT_MIN_S:
                    break
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                keys = [ESC]  # 창의 빨간 버튼으로 닫아도 종료

        if args.auto and race.phase == "home" and math.hypot(x, y) < 0.5:
            break  # 테스트: 완주하고 출발점에 돌아왔으면 끝
        if args.duration and time.time() - started > args.duration:
            break

        if ESC in keys:
            break
        for key in keys:
            if key == ord("t"):
                if not link.armed:
                    take_off()
            elif key == ord("l"):
                if lander.active:
                    lander.cancel()  # 옮기는 중에 L을 또 누르면 그 자리에 바로 내린다
                    print(link.start_land() or "그 자리에 착륙")
                elif link.state == "taking_off":
                    print(link.start_land() or "이륙 취소, 착륙")
                elif link.flying:
                    print(lander.request(link, x, y, z))
            elif key == ord("c"):
                tracker.reset()
                clock.reset(running=link.flying)
                race.start(link)
                print("코스 기록 초기화")
            elif key == ord("x"):
                if link.armed:
                    # 부딪혀서 기울지 않고 걸려 버린 경우 등: 모터를 끄고 세운다
                    link.kill()
                    lander.cancel()
                    print("모터를 끄고 세웁니다")
                # 넘어진 드론을 똑바로 세운다. 지붕 위면 지붕 위에, 가장자리면 안쪽에 (살짝 띄워 놓는다).
                # 뒤집히면 PX4가 아는 위치·자세가 엉망이라 Gazebo의 진짜 위치로 자리를 고른다
                pose = view_cam.true_pose(DRONE_MODEL, timeout_s=3.0)
                if pose is None:
                    print("Gazebo 위치를 못 받아서 PX4가 아는 위치로 세웁니다")
                    pose = (x, y, z, link.yaw())
                tx, ty, tz, tyaw = pose
                spot = upright_spot(tx, ty, tz)
                if spot is None:
                    print("근처에 세울 자리가 없음")
                    continue
                sx, sy, floor = spot
                placed = any(view_cam.set_pose(DRONE_MODEL, sx, sy, floor + UPRIGHT_LIFT, 0.0, tyaw, timeout_ms=1000)
                             for _ in range(2))  # 한 번 안 되면 한 번 더
                if not placed:
                    print("세우기 실패 (Gazebo 응답 없음). 다시 X")
                    continue
                # PX4 위치 추정(EKF)을 새로 시작해서 '아직 뒤집혀 있다'는 착각을 지운다
                if sim is not None and sim.px4_command("ekf2 stop") and sim.px4_command("ekf2 start"):
                    calibrate_after = time.time() + CALIBRATE_WAIT_S
                    print("드론을 세웠습니다. 위치 추정을 다시 시작함 (몇 초 뒤 T로 이륙)")
                else:
                    print("드론을 세웠습니다. PX4 위치 추정이 돌아오면 T로 이륙하세요")
            elif key == ord("v"):
                view_name = VIEWS[(VIEWS.index(view_name) + 1) % len(VIEWS)]
                view_cam.watch(view_name != "drone")
                print(f"시점: {view_name}")
            elif key == ord("3"):
                start_race()
            elif key in modes:
                lander.cancel()
                mode = modes[key]
                mode.start(link)
                print(f"모드: {mode.name}")
            elif key != -1:
                mode.on_key(key)

    if link.armed:
        print("착륙")
        link.land_and_wait()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-sim", action="store_true", help="이미 켜 둔 시뮬레이터에 붙는다")
    parser.add_argument("--auto", action="store_true", help="바로 코스 자동 통과를 시작한다")
    parser.add_argument("--headless", action="store_true", help="영상 창 없이 실행")
    parser.add_argument("--duration", type=float, default=0, help="이 시간(초)이 지나면 착륙하고 끝냄")
    args = parser.parse_args()

    sim = None
    if not args.no_sim:
        make_world.main()  # course.py를 바꿨을 수도 있으니 맵을 다시 만든다
        sim = Simulator()
        sim.start()
    try:
        run(args, sim)
    except KeyboardInterrupt:
        pass
    finally:
        if sim:
            sim.stop()


if __name__ == "__main__":
    main()
