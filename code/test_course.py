"""course.py 테스트. 실행: python -m unittest test_course.py"""

import math
import unittest

import course
from course import Box, Gate, GateTracker


class TestCourse(unittest.TestCase):
    def test_auto_path_is_clear(self):
        self.assertEqual(course.check_course(), [])

    def test_segment_hits_box(self):
        box = Box("b", 0, 0, 2, 2, 5)
        self.assertTrue(course.segment_hits_box((-5, 0), (5, 0), box))
        self.assertFalse(course.segment_hits_box((-5, 3), (5, 3), box))
        self.assertTrue(course.segment_hits_box((-5, 3), (5, 3), box, margin=2.5))
        self.assertFalse(course.segment_hits_box((-5, 0), (-2, 0), box))

    def test_segment_hits_rotated_box(self):
        # 45도 돌린 2x2 상자는 모서리가 (1.41, 0)까지 나온다
        box = Box("b", 0, 0, 2, 2, 5, yaw=math.pi / 4)
        self.assertTrue(course.segment_hits_box((1.3, -5), (1.3, 5), box))
        self.assertFalse(course.segment_hits_box((1.5, -5), (1.5, 5), box))

    def test_tracker_passes_through_opening(self):
        gate = Gate("g", 10, 0, 3, 0.0)
        tracker = GateTracker([gate])
        self.assertFalse(tracker.update(8, 0, 3))
        self.assertTrue(tracker.update(10.2, 0.5, 3.4))
        self.assertTrue(tracker.done)

    def test_tracker_ignores_going_around(self):
        gate = Gate("g", 10, 0, 3, 0.0)
        tracker = GateTracker([gate])
        tracker.update(8, 3, 3)  # 옆으로 비켜서 지나감
        self.assertFalse(tracker.update(11, 3, 3))
        tracker.update(8, 0, 6)  # 위로 지나감
        self.assertFalse(tracker.update(11, 0, 6))
        self.assertFalse(tracker.done)

    def test_tracker_counts_in_order(self):
        tracker = GateTracker()
        for g in course.GATES:
            tracker.update(*g.point(-1), g.z)
            self.assertTrue(tracker.update(*g.point(1), g.z))
        self.assertTrue(tracker.done)

    def test_gates_face_previous_gate(self):
        g1, g2 = course.GATES[0], course.GATES[1]
        along, side, _ = g2.offset(g1.x, g1.y, g2.z)
        self.assertLess(along, 0)
        self.assertAlmostEqual(side, 0, places=6)

    def test_ned_conversion(self):
        self.assertEqual(course.world_to_ned(1, 2, 3), (2, 1, -3))
        self.assertEqual(course.ned_to_world(*course.world_to_ned(1, 2, 3)), (1, 2, 3))
        self.assertAlmostEqual(course.world_yaw_to_ned_deg(0), 90)  # 동쪽
        self.assertAlmostEqual(course.world_yaw_to_ned_deg(math.pi / 2), 0)  # 북쪽
        self.assertAlmostEqual(course.world_yaw_to_ned_deg(math.pi), -90)  # 서쪽

    def test_route_over_building(self):
        # 건물 1(20, -6, 높이 12)을 가로지르는 길은 그 위(12 + 2m)로 넘어간다
        self.assertEqual(course.route_height((20.0, -12.0), (20.0, 0.0)), 12.0 + course.ROUTE_CLEARANCE)

    def test_course_route_is_open(self):
        # 정해진 코스 경로(게이트 뒤 → 다음 게이트 앞)는 넘어갈 필요가 없다
        points = course.auto_waypoints()
        for (kind_a, _, xa, ya, _), (kind_b, _, xb, yb, _) in zip(points, points[1:]):
            if kind_a == "exit":
                self.assertIsNone(course.route_height((xa, ya), (xb, yb)), f"{kind_a}->{kind_b}")

    def test_race_mode_climbs_when_blocked(self):
        from modes import RaceMode
        race = RaceMode(course.GateTracker())
        # 건물 1 바로 뒤(남쪽)에서 북쪽 목표로: 먼저 그 자리에서 14m로 올라간다
        cmd = race._fly_to(20.0, -12.0, 3.0, 20.0, 0.0, 3.0, 0.0)
        self.assertEqual(course.ned_to_world(cmd.north_m, cmd.east_m, cmd.down_m), (20.0, -12.0, 14.0))
        # 다 올라갔으면 그 높이로 목표 위까지
        cmd = race._fly_to(20.0, -12.0, 14.0, 20.0, 0.0, 3.0, 0.0)
        self.assertEqual(course.ned_to_world(cmd.north_m, cmd.east_m, cmd.down_m), (20.0, 0.0, 14.0))
        # 막힌 게 없으면 곧장
        cmd = race._fly_to(0.0, 0.0, 3.0, 5.0, 0.0, 3.0, 0.0)
        self.assertEqual(course.ned_to_world(cmd.north_m, cmd.east_m, cmd.down_m), (5.0, 0.0, 3.0))


if __name__ == "__main__":
    unittest.main()
