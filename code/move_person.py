"""Gazebo 속 사람 모델("person")을 사각형 경로로 걷게 한다. 드론 따라가기 테스트용.

먼저 사람 모델을 놓아둔다 (계획 파일 Phase 4 참고).
실행은 ~/drone/venv 환경에서 한다. Ctrl+C로 멈춘다.

좌표는 Gazebo 세계 좌표 (x, y), 단위 m. 드론은 (0, 0)에서 +x 쪽을 보고 시작한다.
"""

import argparse
import math
import os
import time

os.environ.setdefault("GZ_IP", "127.0.0.1")  # run_sim.sh와 같은 통신 설정

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node

SERVICE = "/world/default/set_pose"
NAME = "person"

# 사람이 처음 서 있는 (6, 0)에서 출발해서 사각형을 돈다
PATH = [(6.0, 0.0), (6.0, 6.0), (12.0, 6.0), (12.0, 0.0)]
UPDATE_HZ = 20


def set_pose(node, x, y, yaw):
    req = Pose()
    req.name = NAME
    req.position.x = x
    req.position.y = y
    req.position.z = 0.0
    # yaw(라디안)만 있는 회전을 쿼터니언으로
    req.orientation.z = math.sin(yaw / 2)
    req.orientation.w = math.cos(yaw / 2)
    ok, _ = node.request(SERVICE, req, Pose, Boolean, 1000)
    return ok


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=0.6, help="걷는 속도 (m/s)")
    parser.add_argument("--wait", type=float, default=0, help="이 시간(초)만큼 기다렸다가 걷기 시작")
    parser.add_argument("--duration", type=float, default=0, help="이 시간(초)만큼 걷고 멈춤. 0이면 계속")
    args = parser.parse_args()

    node = Node()
    time.sleep(args.wait)
    print(f"사람 걷기 시작 (속도 {args.speed} m/s)")

    start = time.time()
    i = 0
    x, y = PATH[0]
    while not args.duration or time.time() - start < args.duration:
        goal_x, goal_y = PATH[(i + 1) % len(PATH)]
        dx, dy = goal_x - x, goal_y - y
        distance = math.hypot(dx, dy)
        step = args.speed / UPDATE_HZ

        if distance <= step:
            x, y = goal_x, goal_y
            i += 1
            print(f"  {i % len(PATH)}번 모서리 도착 ({x:.0f}, {y:.0f})")
        else:
            x += dx / distance * step
            y += dy / distance * step

        if not set_pose(node, x, y, math.atan2(dy, dx)):
            raise SystemExit("사람 위치 변경 실패. 시뮬레이터와 사람 모델이 있는지 확인")
        time.sleep(1 / UPDATE_HZ)

    print("사람 멈춤")


if __name__ == "__main__":
    main()
