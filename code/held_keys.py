"""지금 눌려 있는 키를 macOS에 직접 물어본다. (app.py의 직접 조종이 쓴다)

OpenCV 창은 '키를 눌렀다'만 알려 주고 '뗐다'는 알려 주지 않는다. 그래서 키 반복에 기대면
처음 0.2초쯤 반복이 시작되기 전에 멈칫하고, 떼도 조금 더 가고, 두 키(W+D)를 같이 누를 수 없다.
macOS(Quartz)에 키 상태를 직접 물어보면 누른 동안만, 여러 키를 동시에 움직일 수 있다.

키보드의 '자리'로 물어보기 때문에 한글 입력기여도 된다.
이 프로그램 창이 앞에 있을 때만 키를 읽는다 (다른 창에서 타자 친 게 조종되지 않게).
"""

import os

from keyboard_control import DOWN, LEFT, RIGHT, UP

try:
    import Quartz
    from AppKit import NSWorkspace
except ImportError:  # pyobjc가 없으면 예전처럼 키 반복으로 조종한다
    Quartz = None

# 조종 키(keyboard_control.MOVES의 키) → macOS 키보드 자리 번호 (영문 자판 기준)
KEY_CODES = {ord("w"): 13, ord("s"): 1, ord("a"): 0, ord("d"): 2,
             ord("r"): 15, ord("f"): 3, ord("q"): 12, ord("e"): 14,
             UP: 126, DOWN: 125, LEFT: 123, RIGHT: 124}

# 키 이벤트가 이만큼 들어왔는데 키 상태가 한 번도 '눌림'이 아니면 읽을 수 없는 것으로 본다
CHECK_EVENTS = 5


class HeldKeys:
    def __init__(self):
        self.available = Quartz is not None
        self._pid = os.getpid()
        self._events = 0
        self._seen_down = False

    def _front(self):
        app = NSWorkspace.sharedWorkspace().frontmostApplication()
        return app is not None and app.processIdentifier() == self._pid

    def held(self):
        """지금 눌려 있는 조종 키들 (MOVES의 키, 예: {ord("w"), UP})."""
        if not self.available or not self._front():
            return set()
        state = Quartz.kCGEventSourceStateCombinedSessionState
        down = {k for k, code in KEY_CODES.items() if Quartz.CGEventSourceKeyState(state, code)}
        if down:
            self._seen_down = True
        return down

    def saw_key_event(self, key):
        """창에 조종 키 이벤트가 들어왔을 때 부른다. 키 상태가 계속 비어 있으면 이 방법을 그만 쓴다."""
        if not self.available or self._seen_down or key not in KEY_CODES:
            return
        if self.held():
            return
        self._events += 1
        if self._events >= CHECK_EVENTS:
            self.available = False
            print("키 상태를 읽을 수 없어서 키 반복 방식으로 조종합니다")
