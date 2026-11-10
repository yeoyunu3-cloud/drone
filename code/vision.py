"""YOLO로 영상에서 따라갈 사람(목표)을 찾는 공통 코드.

웹캠(yolo_webcam.py)과 드론 카메라(drone_camera.py)가 같이 쓴다.
"""

from dataclasses import dataclass

import cv2

MODEL = "yolo26n.pt"  # 가장 작고 빠른 모델, 처음 실행 때 자동 다운로드
PERSON = 0  # COCO 데이터셋에서 0번 클래스가 person
MIN_CONFIDENCE = 0.5


@dataclass
class Target:
    error_x: float  # 목표가 화면 중앙에서 좌우로 벗어난 정도 (-1 ~ 1, 오른쪽이 +)
    size: float  # 목표 박스가 화면에서 차지하는 비율 (가까울수록 크다)
    height: float  # 목표 박스 높이 / 화면 높이 (0 ~ 1). 거리에 반비례해서 거리 맞추기에 쓴다


def find_target(model, frame):
    """frame에서 사람을 찾아 박스를 그리고, 가장 큰 사람을 Target으로 돌려준다. 없으면 None."""
    h, w = frame.shape[:2]
    center_x, center_y = w // 2, h // 2
    cv2.drawMarker(frame, (center_x, center_y), (255, 255, 255), cv2.MARKER_CROSS, 30, 2)

    result = model.predict(frame, classes=[PERSON], conf=MIN_CONFIDENCE, verbose=False)[0]

    # 가장 큰 사람 = 가장 가까운 사람을 목표로 삼는다
    boxes = sorted(result.boxes, key=lambda b: float(b.xywh[0][2] * b.xywh[0][3]), reverse=True)
    for i, box in enumerate(boxes):
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        color = (0, 0, 255) if i == 0 else (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, f"person {float(box.conf):.2f}", (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    if not boxes:
        cv2.putText(frame, "people: 0", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        return None

    x1, y1, x2, y2 = map(int, boxes[0].xyxy[0])
    target_x, target_y = (x1 + x2) // 2, (y1 + y2) // 2
    cv2.line(frame, (center_x, center_y), (target_x, target_y), (0, 0, 255), 2)

    target = Target(
        error_x=(target_x - center_x) / center_x,
        size=(x2 - x1) * (y2 - y1) / (w * h),
        height=(y2 - y1) / h,
    )
    info = (f"people: {len(boxes)}  error_x: {target.error_x:+.2f}  "
            f"size: {target.size:.2f}  height: {target.height:.2f}")
    cv2.putText(frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    return target
