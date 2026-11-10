import unittest
from stop_evidence_monitor.lidar_motion import FixedWallLidarMotion


class FixedWallLidarTests(unittest.TestCase):
    def setUp(self):
        self.e = FixedWallLidarMotion()

    def observe(self, t, x):
        return self.e.observe(t, 100 + t, [x + 0.005, x, x + 0.005], 0.15, 12.0)

    def test_approaching_wall(self):
        self.assertIsNone(self.observe(1, 5))
        s = self.observe(1.2, 4.96)
        self.assertAlmostEqual(s.forward_speed_mps, 0.2, places=5)

    def test_stationary_wall(self):
        self.observe(1, 5)
        self.assertEqual(self.observe(1.1, 5).forward_speed_mps, 0)

    def test_stale_sensor_stamp_rejected(self):
        self.observe(1, 5)
        self.assertIsNone(self.observe(1, 5))
        self.assertIsNone(self.observe(.8, 5))

    def test_missing_lidar_return_resets_pair(self):
        self.observe(1, 5)
        self.assertIsNone(self.e.observe(1.1, 101.1, [float('inf')]*3, 0.15, 12))
        self.assertIsNone(self.observe(1.2, 4.9))

    def test_scan_gap_must_rebuild_pair(self):
        self.observe(1, 5)
        self.assertIsNone(self.observe(2, 4.9))
        self.assertAlmostEqual(self.observe(2.1, 4.89).forward_speed_mps, .1)

    def test_spread_rejected(self):
        self.assertIsNone(self.e.observe(1, 101, [4, 5, 6], .15, 12))

    def test_max_speed_rejected(self):
        self.observe(1, 5)
        self.assertIsNone(self.observe(1.1, 4))

if __name__ == '__main__':
    unittest.main()
