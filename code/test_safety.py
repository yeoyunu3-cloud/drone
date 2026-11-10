"""safety.py 테스트. 실행: python -m unittest test_safety.py"""

import math
import unittest

from mavsdk.plugins.offboard import PositionNedYaw, VelocityBodyYawspeed, VelocityNedYaw

import safety
from course import Solid

WALL = Solid(10, 0, 5, 2, 20, 10)  # x=9~11에 서 있는 벽


class TestSafety(unittest.TestCase):
    def setUp(self):
        self.saved = safety.OBSTACLES
        safety.OBSTACLES = [WALL]

    def tearDown(self):
        safety.OBSTACLES = self.saved

    def test_far_from_wall_unchanged(self):
        v, limited = safety.limit_velocity(0, 0, 3, 1.5, 0, 0)
        self.assertEqual(v, [1.5, 0, 0])
        self.assertFalse(limited)

    def test_stops_in_front_of_wall(self):
        # 벽 표면(x=9)에서 0.8m 앞
        v, limited = safety.limit_velocity(9 - safety.STOP_DIST, 0, 3, 1.5, 0, 0)
        self.assertAlmostEqual(v[0], 0)
        self.assertTrue(limited)

    def test_slows_near_wall(self):
        v, _ = safety.limit_velocity(7.5, 0, 3, 1.5, 0, 0)  # 벽 표면에서 1.5m 앞
        self.assertAlmostEqual(v[0], (1.5 - safety.STOP_DIST) * safety.SLOW_GAIN)

    def test_can_move_away_and_along_wall(self):
        v, _ = safety.limit_velocity(9 - safety.STOP_DIST, 0, 3, -1.0, 1.5, 0)
        self.assertEqual(v, [-1.0, 1.5, 0])

    def test_ground(self):
        v, limited = safety.limit_velocity(0, 0, safety.MIN_ALT, 0, 0, -1.0)
        self.assertAlmostEqual(v[2], 0)
        self.assertTrue(limited)
        v, _ = safety.limit_velocity(0, 0, 5, 0, 0, -1.0)
        self.assertEqual(v[2], -1.0)

    def test_body_command_toward_wall(self):
        # 북쪽(yaw 90도)을 보고 오른쪽(= 동쪽 = 벽 쪽)으로 가면 막힌다
        cmd = VelocityBodyYawspeed(0.0, 1.5, 0.0, 10.0)
        out = safety.filter_command(cmd, 9 - safety.STOP_DIST, 0, 3, math.pi / 2)
        self.assertAlmostEqual(out.right_m_s, 0, places=6)
        self.assertAlmostEqual(out.forward_m_s, 0, places=6)
        self.assertEqual(out.yawspeed_deg_s, 10.0)

    def test_ned_command_toward_wall(self):
        cmd = VelocityNedYaw(0.5, 1.5, 0.0, 90.0)  # 동쪽(벽 쪽) 1.5, 북쪽 0.5
        out = safety.filter_command(cmd, 9 - safety.STOP_DIST, 0, 3, 0)
        self.assertAlmostEqual(out.east_m_s, 0, places=6)
        self.assertAlmostEqual(out.north_m_s, 0.5, places=6)

    def test_position_command_toward_wall_is_stopped(self):
        # 벽 너머 목표로 가는 위치 명령: 벽 쪽으로는 더 못 가는 속도 명령으로 바뀐다
        cmd = PositionNedYaw(0.0, 20.0, -3.0, 90.0)
        out = safety.filter_command(cmd, 9 - safety.STOP_DIST, 0, 3, 0)
        self.assertIsInstance(out, VelocityNedYaw)
        self.assertAlmostEqual(out.east_m_s, 0, places=6)
        self.assertEqual(out.yaw_deg, 90.0)

    def test_position_command_away_unchanged(self):
        cmd = PositionNedYaw(0.0, 0.0, -3.0, 90.0)  # 벽 반대쪽으로
        self.assertIs(safety.filter_command(cmd, 8.5, 0, 3, 0), cmd)


if __name__ == "__main__":
    unittest.main()
