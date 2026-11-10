"""착륙 자리 고르기(landing.py)와 세우기·이륙 높이(app.py) 확인. 실행: ../venv/bin/python test_landing.py"""

import math
import unittest

import app
import course
import landing

B6 = next(b for b in course.BUILDINGS if b.name == "building_6")  # (-6, -14) 6x6, 높이 8
P1 = next(b for b in course.BUILDINGS if b.name == "pillar_1")  # (33, -9) 1x1, 높이 10
GATE = course.GATES[0]  # (12, 0, 2.5)


def fully_flat(x, y, floor):
    return landing.spot_floor(x, y, floor + 0.3) == floor


class LandingTest(unittest.TestCase):
    def test_roof_center_lands_here(self):
        self.assertEqual(landing.find_spot(B6.x, B6.y, 10.0), (B6.x, B6.y, 8.0))

    def test_roof_edge_moves_inward(self):
        edge = B6.x + B6.size_x / 2
        x, y, floor = landing.find_spot(edge - 0.1, B6.y, 10.0)
        self.assertEqual(floor, 8.0)  # 지붕 안쪽이 더 가깝다
        self.assertLessEqual(x, edge - landing.FOOTPRINT)
        self.assertTrue(fully_flat(x, y, floor))

    def test_just_off_roof_edge_moves_out(self):
        edge = B6.x + B6.size_x / 2
        x, y, floor = landing.find_spot(edge + 0.2, B6.y, 10.0)
        self.assertEqual(floor, 0.0)
        self.assertGreaterEqual(x, edge + landing.FOOTPRINT)

    def test_pillar_top_too_small(self):
        x, y, floor = landing.find_spot(P1.x, P1.y, 12.0)
        self.assertEqual(floor, 0.0)
        self.assertGreater(math.hypot(x - P1.x, y - P1.y), P1.size_x / 2)

    def test_gate_top_bar(self):
        x, y, floor = landing.find_spot(GATE.x, GATE.y, 5.0)
        self.assertTrue(fully_flat(x, y, floor))
        self.assertIsNotNone(landing.spot_floor(x, y, 5.0))

    def test_person_head(self):
        px, py = course.PERSON
        x, y, floor = landing.find_spot(px, py, 3.0)
        self.assertEqual(floor, 0.0)
        self.assertGreater(math.hypot(x - px, y - py), 0.3 + landing.FOOTPRINT - 0.01)

    def test_next_to_wall_moves_away(self):
        wall = B6.x + B6.size_x / 2
        x, y, floor = landing.find_spot(wall + 0.3, B6.y, 1.0)
        self.assertEqual(floor, 0.0)
        self.assertGreaterEqual(x, wall + landing.FOOTPRINT)

    def test_open_ground_lands_here(self):
        self.assertEqual(landing.find_spot(0.0, 0.0, 2.5), (0.0, 0.0, 0.0))


class UprightTest(unittest.TestCase):
    def test_fallen_on_roof_stays_on_roof(self):
        self.assertEqual(app.upright_spot(B6.x, B6.y, 8.1), (B6.x, B6.y, 8.0))

    def test_fallen_on_roof_edge_moves_inward(self):
        edge = B6.x + B6.size_x / 2
        x, y, floor = app.upright_spot(edge - 0.1, B6.y, 8.05)
        self.assertEqual(floor, 8.0)
        self.assertTrue(fully_flat(x, y, floor))

    def test_stuck_inside_building_goes_outside(self):
        x, y, floor = app.upright_spot(B6.x, B6.y, 4.0)
        self.assertEqual(floor, 0.0)
        self.assertFalse(landing._inside(B6.solid(), x, y, landing.FOOTPRINT - 0.01))

    def test_fallen_under_gate_bar_moves_out(self):
        x, y, floor = app.upright_spot(GATE.x, GATE.y, 0.1)
        self.assertEqual(floor, 0.0)
        self.assertGreaterEqual(landing.headroom(x, y, floor), app.UPRIGHT_HEADROOM)


class TakeOffHeightTest(unittest.TestCase):
    def test_open_ground(self):
        self.assertEqual(app.take_off_height(0.0, 0.0, 0.1), app.ALTITUDE_M)

    def test_roof(self):
        self.assertEqual(app.take_off_height(B6.x, B6.y, 8.1), app.ALTITUDE_M)

    def test_next_to_pillar_is_not_blocked(self):
        # 기둥 옆에 내려앉은 드론: 옆의 기둥은 '위가 막힌 것'이 아니다
        self.assertEqual(app.take_off_height(P1.x - 0.95, P1.y, 0.0), app.ALTITUDE_M)

    def test_under_gate_bottom_bar(self):
        self.assertEqual(app.take_off_height(GATE.x, GATE.y, 0.1), 0.0)


if __name__ == "__main__":
    unittest.main()
