"""Mac 웹캠 영상에서 YOLO로 사람을 찾아 박스를 그린다.

q 키를 누르면 끝난다.
나중에 드론이 따라갈 때 쓸 값(사람 박스의 중심, 화면 중앙과의 차이)도 화면에 표시한다.
"""

import cv2
from ultralytics import YOLO

from vision import MODEL, find_target


def main():
    model = YOLO(MODEL)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise SystemExit("웹캠을 열 수 없음. 카메라 권한을 확인")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        find_target(model, frame)

        cv2.imshow("YOLO webcam (q: quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
