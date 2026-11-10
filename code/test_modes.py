"""ManualMode가 키 상태대로 부드럽게 움직이는지 확인한다. 실행: ../venv/bin/python test_modes.py"""

import math
import time

from keyboard_control import LEFT, UP
from modes import ManualMode

W, D, Q, R = (ord(c) for c in "wdqr")


class FakeKeys:
    available = True

    def __init__(self):
        self.down = set()

    def held(self):
        return self.down

    def saw_key_event(self, char):
        pass


def run(mode, seconds, dt=1 / 30):
    """시간을 dt씩 흘려가며 step을 부른다."""
    command = None
    for _ in range(int(seconds / dt)):
        mode.last_step -= dt  # step이 dt만큼 지난 것으로 보게
        command = mode.step(None, None)
    return command


def test_diagonal_same_speed():
    keys = FakeKeys()
    mode = ManualMode(keys)
    keys.down = {W, D}
    c = run(mode, 3)
    assert math.isclose(math.hypot(c.forward_m_s, c.right_m_s), 1.5, abs_tol=0.01)
    assert c.forward_m_s > 1.0 and c.right_m_s > 1.0


def test_ramps_up_not_jump():
    keys = FakeKeys()
    mode = ManualMode(keys)
    keys.down = {W}
    c = run(mode, 0.1)
    assert 0.1 < c.forward_m_s < 0.4  # 0.1초 만에 1.5로 튀지 않는다


def test_release_stops_smoothly():
    keys = FakeKeys()
    mode = ManualMode(keys)
    keys.down = {W}
    run(mode, 2)
    keys.down = set()
    c = run(mode, 0.2)
    assert 0.5 < c.forward_m_s < 1.5  # 떼자마자 0이 되지 않고
    c = run(mode, 2)
    assert c.forward_m_s < 0.01  # 곧 선다


def test_turn_and_climb_together():
    keys = FakeKeys()
    mode = ManualMode(keys)
    keys.down = {Q, R}
    c = run(mode, 3)
    assert c.yawspeed_deg_s < -40 and c.down_m_s < -0.7


def test_arrow_keys_turn_and_climb():
    keys = FakeKeys()
    mode = ManualMode(keys)
    keys.down = {LEFT, UP}
    c = run(mode, 3)
    assert c.yawspeed_deg_s < -40 and c.down_m_s < -0.7


def test_arrow_repeat_fallback():
    mode = ManualMode(None)
    mode.on_key(UP)
    c = run(mode, 0.3)
    assert c.down_m_s < -0.2


def test_repeat_fallback():
    mode = ManualMode(None)  # 키 상태를 못 읽으면 키 이벤트로
    mode.on_key(ord("w"))
    c = run(mode, 0.3)
    assert c.forward_m_s > 0.5


if __name__ == "__main__":
    for name, test in list(globals().items()):
        if name.startswith("test_"):
            test()
            print("ok", name)
