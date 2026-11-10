"""Gazebo 속 드론 카메라 영상을 받아 YOLO로 사람을 찾는다.

먼저 카메라 드론으로 시뮬레이터를 켜둔다: zsh ~/drone/run_sim.sh gz_x500_mono_cam
실행은 ~/drone/venv 환경에서 한다 (Gazebo Python 모듈이 거기에만 있다).
q 키를 누르면 끝난다.
"""

import os
import threading
import time

os.environ.setdefault("GZ_IP", "127.0.0.1")  # run_sim.sh와 같은 통신 설정

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from ultralytics import YOLO

from vision import MODEL, find_target

def camera_topic(world="default"):
    """맵(world) 이름에 따라 드론 카메라 토픽 이름이 바뀐다."""
    return f"/world/{world}/model/x500_mono_cam_0/link/camera_link/sensor/camera/image"


CAMERA_TOPIC = camera_topic()


class DroneCamera:
    """Gazebo 카메라 토픽을 구독해서 가장 최근 프레임 하나만 들고 있는다."""

    def __init__(self, topic=CAMERA_TOPIC):
        self._lock = threading.Lock()
        self._frame = None
        self._node = Node()  # 이 객체가 살아 있어야 구독이 유지된다
        if not self._node.subscribe(Image, topic, self._on_image):
            raise SystemExit(f"카메라 토픽 구독 실패: {topic}")

    def _on_image(self, msg):
        # Gazebo는 RGB, OpenCV는 BGR 순서라서 바꿔준다
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        with self._lock:
            self._frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def read(self):
        """가장 최근 프레임(BGR numpy 배열). 아직 없으면 None."""
        with self._lock:
            return None if self._frame is None else self._frame.copy()


def main():
    model = YOLO(MODEL)
    camera = DroneCamera()

    print("카메라 영상 기다리는 중...")
    while camera.read() is None:
        time.sleep(0.1)
    print("영상 수신 시작")

    while True:
        frame = camera.read()
        target = find_target(model, frame)
        if target:
            print(f"  error_x {target.error_x:+.2f}  size {target.size:.2f}")

        cv2.imshow("Drone camera (q: quit)", cv2.resize(frame, (960, 720)))
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
