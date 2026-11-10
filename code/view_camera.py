"""조종 창의 다른 시점(추격 시점, 위에서 보기)을 만드는 관찰용 카메라.

도시 맵에 모양 없는 카메라(make_world.py의 view_camera)가 하나 있다.
드론이 움직일 때마다 그 카메라를 드론 뒤나 위로 옮기고, 찍힌 영상을 받는다.
영상은 그 시점을 보고 있을 때만 받는다 (안 받으면 Gazebo가 그리지 않아서 Mac이 덜 바쁘다).

카메라 옮기기(Gazebo에 요청하고 답을 기다림)는 한 번에 20~140ms 걸려서, 조종 루프에서 하면
드론 명령까지 같이 끊긴다. 스레드로 빼도 기다리는 동안 파이썬 전체가 멈춰서(GIL) 소용없다.
그래서 이 파일을 따로 실행한 프로세스(mover)가 1초에 30번 옮긴다. 목표 자리는 한 줄씩 글자로 보낸다.
카메라는 드론에 딱 붙지 않고 부드럽게 쫓아간다 (진짜 촬영 드론처럼).
"""

import math
import os
import subprocess
import sys
import threading
import time

os.environ.setdefault("GZ_IP", "127.0.0.1")  # run_sim.sh와 같은 통신 설정

import cv2
import numpy as np
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node, SubscribeOptions

from make_world import VIEW_CAMERA_NAME, VIEW_CAMERA_TOPIC

CHASE_BACK = 5.0  # 추격 시점: 드론 뒤로 이만큼 (m)
CHASE_UP = 2.5  # 추격 시점: 드론 위로 이만큼 (m)
TOP_UP = 15.0  # 위에서 보기: 드론 위로 이만큼 (m)
MOVE_HZ = 30  # 카메라를 1초에 몇 번 옮기나
FOLLOW_S = 0.25  # 카메라가 드론을 쫓아가는 느슨함 (초). 작을수록 딱 붙는다
POSES_PER_SEC = 5  # Gazebo 진짜 위치를 1초에 몇 번 받아 두나 (많이 받으면 조종 루프가 느려진다)


def angle_diff(a, b):
    """두 각도(라디안)의 차이를 -pi ~ pi로."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def camera_pose(view, x, y, z, yaw):
    """시점 이름과 드론 위치(세계 좌표)로 카메라 위치와 기울기(pitch, 아래가 +)를 정한다."""
    if view == "chase":
        cx = x - CHASE_BACK * math.cos(yaw)
        cy = y - CHASE_BACK * math.sin(yaw)
        return cx, cy, z + CHASE_UP, math.atan2(CHASE_UP, CHASE_BACK), yaw
    # top: 바로 위에서 수직으로 내려다본다. 화면 위쪽이 드론의 앞쪽
    return x, y, z + TOP_UP, math.pi / 2, yaw


def set_pose(node, service, name, x, y, z, pitch, yaw, timeout_ms):
    """Gazebo 속 모델(name)을 그 위치·방향으로 옮긴다."""
    req = Pose()
    req.name = name
    req.position.x, req.position.y, req.position.z = x, y, z
    # 회전(roll 0, pitch, yaw)을 쿼터니언으로
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cyaw, syaw = math.cos(yaw / 2), math.sin(yaw / 2)
    req.orientation.w = cp * cyaw
    req.orientation.x = -sp * syaw
    req.orientation.y = sp * cyaw
    req.orientation.z = cp * syaw
    ok, _ = node.request(service, req, Pose, Boolean, timeout_ms)
    return ok


class ViewCamera:
    def __init__(self, world):
        self._node = Node()
        self._service = f"/world/{world}/set_pose"
        self._lock = threading.Lock()
        self._frame = None
        self._watching = False
        self._mover = None
        # Gazebo 진짜 위치는 처음부터 계속 받아 둔다. 필요할 때 구독하면 첫 메시지가 몇 초 늦게 온다
        self._poses = None
        self._poses_time = 0.0
        options = SubscribeOptions()
        options.msgs_per_sec = POSES_PER_SEC
        self._node.subscribe(Pose_V, f"/world/{world}/pose/info", self._on_poses, options)

    def watch(self, on):
        """영상을 받기 시작하거나 그만둔다. 그만두면 카메라 옮기는 프로세스도 쉬게 한다."""
        if on and not self._watching:
            self._node.subscribe(Image, VIEW_CAMERA_TOPIC, self._on_image)
        elif not on and self._watching:
            self._node.unsubscribe(VIEW_CAMERA_TOPIC)
            self._send("pause")
            with self._lock:
                self._frame = None
        self._watching = on

    def _on_image(self, msg):
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        with self._lock:
            self._frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def read(self):
        """가장 최근 영상 (BGR). 아직 없으면 None."""
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def move(self, view, x, y, z, yaw):
        """카메라가 쫓아갈 자리를 정한다 (바로 돌아온다. 실제로 옮기는 건 mover 프로세스)."""
        self._send(" ".join(f"{v:.4f}" for v in camera_pose(view, x, y, z, yaw)) + f" {view}")

    def _send(self, line):
        if self._mover is None:
            if line == "pause":
                return
            # 처음 쓸 때 한 번만 켠다. 이 프로그램이 끝나면 입력이 끊겨서 같이 끝난다
            self._mover = subprocess.Popen(
                [sys.executable, os.path.abspath(__file__), self._service],
                stdin=subprocess.PIPE, text=True, bufsize=1)
        try:
            self._mover.stdin.write(line + "\n")
        except (BrokenPipeError, OSError):
            self._mover = None  # 죽었으면 다음에 다시 켠다

    def set_pose(self, name, x, y, z, pitch, yaw, timeout_ms=100):
        """Gazebo 속 모델(name)을 그 위치·방향으로 옮긴다. 드론을 다시 세울 때도 쓴다."""
        return set_pose(self._node, self._service, name, x, y, z, pitch, yaw, timeout_ms)

    def _on_poses(self, msg):
        with self._lock:
            self._poses = msg
            self._poses_time = time.monotonic()

    def true_pose(self, name, max_age_s=1.0, timeout_s=2.0):
        """Gazebo가 아는 모델(name)의 진짜 위치 (x, y, z, yaw). 못 받으면 None.
        드론이 뒤집히거나 옮겨지면 PX4의 위치 추정이 틀리므로, 세우기·이륙 검사는 이걸 쓴다.
        max_age_s보다 오래된 것밖에 없으면 새 것이 올 때까지 timeout_s 동안 기다린다."""
        deadline = time.monotonic() + timeout_s
        while True:
            with self._lock:
                msg, age = self._poses, time.monotonic() - self._poses_time
            if msg is not None and age <= max_age_s:
                break
            if time.monotonic() > deadline:
                return None
            time.sleep(0.05)
        for p in msg.pose:
            if p.name == name:
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                return p.position.x, p.position.y, p.position.z, yaw
        return None


def mover(service):
    """따로 도는 프로세스: 받은 목표 자리로 관찰용 카메라를 부드럽게 쫓아가게 옮긴다."""
    lock = threading.Lock()
    latest = {"goal": None, "open": True}

    def read_lines():
        for line in sys.stdin:
            parts = line.split()
            with lock:
                latest["goal"] = None if parts[0] == "pause" else ([float(v) for v in parts[:5]], parts[5])
        latest["open"] = False  # 앱이 끝나서 입력이 끊겼다

    threading.Thread(target=read_lines, daemon=True).start()
    node = Node()
    pose, view = None, None
    last = time.time()
    while latest["open"]:
        now = time.time()
        dt, last = now - last, now
        with lock:
            goal = latest["goal"]
        if goal is None:
            pose = None
        else:
            target, target_view = goal
            if pose is None or target_view != view:
                pose, view = list(target), target_view  # 시점을 바꾸면 바로 그 자리로
            else:
                k = 1 - math.exp(-dt / FOLLOW_S)
                for i in range(4):
                    pose[i] += (target[i] - pose[i]) * k
                pose[4] += angle_diff(target[4], pose[4]) * k
            set_pose(node, service, VIEW_CAMERA_NAME, *pose, timeout_ms=100)
        time.sleep(max(0.0, 1 / MOVE_HZ - (time.time() - now)))


if __name__ == "__main__":
    mover(sys.argv[1])
