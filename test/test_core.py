import math
import unittest
from stop_evidence_monitor.core import Config, Status, StopEvidenceMonitor


class MonitorTests(unittest.TestCase):
    def setUp(self):
        self.m = StopEvidenceMonitor()
        self.m.request_stop(0.0)

    def _evidenced_stop(self):
        self.m.record_downstream_command(0.1, 0, 0)
        for t in (0.15, 0.35, 0.55):
            self.m.record_motion(t, 0, 0, "independent_motion")
        self.assertEqual(self.m.evaluate(0.57).status, Status.STOP_EVIDENCED)

    def test_normal_stop_evidenced(self):
        self._evidenced_stop()

    def test_idle(self):
        self.assertEqual(StopEvidenceMonitor().evaluate(1).status, Status.IDLE)

    def test_disconnected_stop_motion_continues(self):
        self.m.record_motion(1.55, .3, 0, "independent_motion")
        self.assertEqual(self.m.evaluate(1.6).status, Status.STOP_FAILED)

    def test_downstream_nonzero_is_failed(self):
        self.m.record_downstream_command(1.55, .2, 0)
        self.assertEqual(self.m.evaluate(1.6).status, Status.STOP_FAILED)

    def test_missing_feedback_is_unverifiable(self):
        self.m.record_downstream_command(1.4, 0, 0)
        self.assertEqual(self.m.evaluate(1.55).status, Status.UNVERIFIABLE)

    def test_frozen_odom_cannot_confirm(self):
        self.m.record_downstream_command(1.4, 0, 0)
        for t in (.2, .4, .6, 1.54):
            self.assertFalse(self.m.record_motion(t, 0, 0, "odom"))
        self.assertEqual(self.m.evaluate(1.55).status, Status.UNVERIFIABLE)

    def test_frozen_odom_with_independent_motion(self):
        self.m.record_downstream_command(1.4, 0, 0)
        self.m.record_motion(1.54, .2, 0, "independent_motion")
        self.assertEqual(self.m.evaluate(1.55).status, Status.STOP_FAILED)

    def test_rebound_command_after_evidence(self):
        self._evidenced_stop()
        self.m.record_downstream_command(.62, .3, 0)
        self.assertEqual(self.m.evaluate(.63).status, Status.STOP_FAILED)
        self.assertEqual(self.m.evaluate(1.8).status, Status.STOP_FAILED)

    def test_rebound_motion_after_evidence(self):
        self._evidenced_stop()
        self.m.record_motion(.62, .2, 0, "independent_motion")
        self.assertEqual(self.m.evaluate(.63).status, Status.STOP_FAILED)

    def test_stale_evidence_degrades(self):
        self._evidenced_stop()
        self.assertEqual(self.m.evaluate(0.9).status, Status.UNVERIFIABLE)

    def test_old_samples_rejected_after_new_request(self):
        self._evidenced_stop()
        self.m.request_stop(2.0)
        self.assertFalse(self.m.record_motion(.55, 0, 0, "independent_motion"))
        self.assertFalse(self.m.record_downstream_command(.1, 0, 0))
        self.assertEqual(self.m.evaluate(3.6).status, Status.UNVERIFIABLE)

    def test_non_finite_samples_rejected(self):
        self.assertFalse(self.m.record_motion(.1, math.nan, 0, "independent_motion"))
        self.assertFalse(self.m.record_downstream_command(.1, math.inf, 0))

    def test_out_of_order_samples_rejected(self):
        self.assertTrue(self.m.record_motion(.4, 0, 0, "independent_motion"))
        self.assertFalse(self.m.record_motion(.2, 0, 0, "independent_motion"))
        self.assertTrue(self.m.record_downstream_command(.4, 0, 0))
        self.assertFalse(self.m.record_downstream_command(.2, 0, 0))

    def test_no_short_hold_false_confirmation(self):
        self.m.record_downstream_command(.1, 0, 0)
        for t in (.35, .4, .45):
            self.m.record_motion(t, 0, 0, "independent_motion")
        self.assertEqual(self.m.evaluate(.46).status, Status.STOP_PENDING)

    def test_gap_in_hold_resets(self):
        self.m.record_downstream_command(.1, 0, 0)
        for t in (.1, .3, .7):
            self.m.record_motion(t, 0, 0, "independent_motion")
        self.assertEqual(self.m.evaluate(.71).status, Status.STOP_PENDING)

    def test_rotation_counts_as_motion(self):
        self.m.record_motion(1.54, 0, .2, "independent_motion")
        self.assertEqual(self.m.evaluate(1.55).status, Status.STOP_FAILED)

    def test_threshold_boundary_treated_as_stopped(self):
        self.m.record_downstream_command(.1, 0, 0)
        for t in (.15, .35, .55):
            self.m.record_motion(t, .02, .03, "independent_motion")
        self.assertEqual(self.m.evaluate(.57).status, Status.STOP_EVIDENCED)

    def test_stop_only_after_deadline_is_unverifiable(self):
        self.m.record_downstream_command(1.6, 0, 0)
        for t in (1.6, 1.8, 2.0):
            self.m.record_motion(t, 0, 0, "independent_motion")
        self.assertEqual(self.m.evaluate(2.01).status, Status.UNVERIFIABLE)

    def test_zero_command_without_receiving_a_stop_request_does_not_count(self):
        other = StopEvidenceMonitor()
        self.assertFalse(other.record_downstream_command(0.2, 0, 0))
        self.assertEqual(other.evaluate(1).status, Status.IDLE)

    def test_reject_odom_config(self):
        with self.assertRaises(ValueError):
            Config(independent_source="odom")


if __name__ == "__main__":
    unittest.main()
