"""M5 adversarial tests: synthetic evidence, not hardware."""
import unittest

from stop_evidence_monitor.core import (
    Config, Status, StopEvidenceMonitor
)
from stop_evidence_monitor.lidar_motion import FixedWallLidarMotion
from stop_evidence_monitor.verdict_lease import VerdictLease


def monitor():
    m = StopEvidenceMonitor(Config(
        require_veto=True, veto_source="wheel_odom"
    ))
    m.request_stop(0.0)
    m.record_downstream_command(.70, 0, 0)
    for t in (.15, .35, .55, .75):
        m.record_motion(t, 0, 0, "independent_motion")
    return m


class M5Tests(unittest.TestCase):

    def test_normal_stop(self):
        m = monitor()
        m.record_veto_motion(.75, 0, 0, "wheel_odom")
        self.assertEqual(
            m.evaluate(.77).status, Status.STOP_EVIDENCED
        )

    def test_missing_wheel_evidence(self):
        m = monitor()
        self.assertEqual(m.evaluate(.77).status, Status.STOP_PENDING)
        self.assertEqual(m.evaluate(1.55).status, Status.UNVERIFIABLE)

    def test_lidar_false_zero_wheel_moving(self):
        m = monitor()
        m.record_veto_motion(.75, .2, 0, "wheel_odom")
        self.assertEqual(m.evaluate(.77).status, Status.STOP_PENDING)
        m.record_downstream_command(1.54, 0, 0)
        m.record_motion(1.54, 0, 0, "independent_motion")
        m.record_veto_motion(1.54, .2, 0, "wheel_odom")
        d = m.evaluate(1.55)
        self.assertEqual(d.status, Status.UNVERIFIABLE)
        self.assertIn("conflicts", d.reason)

    def test_stale_wheel_evidence(self):
        m = monitor()
        m.record_veto_motion(.30, 0, 0, "wheel_odom")
        self.assertEqual(m.evaluate(.77).status, Status.STOP_PENDING)

    def test_wheel_false_zero_lidar_moving(self):
        m = monitor()
        m.record_downstream_command(1.54, 0, 0)
        m.record_motion(1.54, .2, 0, "independent_motion")
        m.record_veto_motion(1.54, 0, 0, "wheel_odom")
        self.assertEqual(m.evaluate(1.55).status, Status.STOP_FAILED)

    def test_continued_downstream_command(self):
        m = monitor()
        m.record_downstream_command(1.54, .2, 0)
        m.record_veto_motion(1.54, 0, 0, "wheel_odom")
        self.assertEqual(m.evaluate(1.55).status, Status.STOP_FAILED)

    def test_motion_resumes_after_confirmation(self):
        m = monitor()
        m.record_veto_motion(.75, 0, 0, "wheel_odom")
        self.assertEqual(m.evaluate(.77).status, Status.STOP_EVIDENCED)
        m.record_motion(.83, .2, 0, "independent_motion")
        self.assertEqual(m.evaluate(.84).status, Status.STOP_FAILED)

    def test_lidar_dropout_after_confirmation(self):
        m = monitor()
        m.record_veto_motion(.75, 0, 0, "wheel_odom")
        self.assertEqual(m.evaluate(.77).status, Status.STOP_EVIDENCED)
        self.assertEqual(m.evaluate(1.10).status, Status.UNVERIFIABLE)

    def test_replayed_sensor_timestamps(self):
        lidar = FixedWallLidarMotion()
        ranges = [5.002, 5.0, 5.002]
        for stamp, receipt in (
            (10.0, 100.0),
            (10.0, 100.1),
            (9.9, 100.2)
        ):
            self.assertIsNone(
                lidar.observe(stamp, receipt, ranges, .15, 12)
            )
        self.assertEqual(lidar.speed_samples, 0)

    def test_known_correlated_spoof_limitation(self):
        # External truth: robot moving, both sensors falsely report zero.
        # The monitor does not receive external truth.
        m = StopEvidenceMonitor(Config(
            require_veto=True, veto_source="wheel_odom"
        ))
        m.request_stop(0.0)
        m.record_downstream_command(.70, 0, 0)
        lidar = FixedWallLidarMotion()

        for t in (.15, .35, .55, .75):
            r = lidar.observe(
                t, t, [5.002, 5.0, 5.002], .15, 12
            )
            if r:
                m.record_motion(
                    t, r.forward_speed_mps, 0,
                    "independent_motion"
                )

        m.record_veto_motion(.75, 0, 0, "wheel_odom")
        self.assertEqual(
            m.evaluate(.77).status, Status.STOP_EVIDENCED
        )
        # Known vulnerability demonstrated, NOT a safety pass.

    def test_consumer_verdict_expiration(self):
        lease = VerdictLease(ttl_s=.25)
        self.assertFalse(lease.is_current_stop(1.0))
        self.assertTrue(
            lease.receive(Status.STOP_EVIDENCED, 1.0)
        )
        self.assertTrue(lease.is_current_stop(1.2))
        self.assertFalse(lease.is_current_stop(1.26))
        self.assertFalse(lease.is_current_stop(.9))
        self.assertFalse(
            lease.receive(Status.STOP_EVIDENCED, .8)
        )
        self.assertTrue(
            lease.receive(Status.UNVERIFIABLE, 1.3)
        )
        self.assertFalse(lease.is_current_stop(1.31))


if __name__ == "__main__":
    unittest.main()
