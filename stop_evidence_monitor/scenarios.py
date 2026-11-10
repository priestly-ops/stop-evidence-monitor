"""Deterministic scenarios. These are synthetic cases, not Gazebo evidence."""
import json
import sys
from .core import StopEvidenceMonitor


def run_scenarios():
    results = {}

    m = StopEvidenceMonitor()
    m.request_stop(0.0)
    m.record_downstream_command(0.1, 0.0, 0.0)
    for t in [0.15, 0.35, 0.55]:
        m.record_motion(t, 0.0, 0.0, "independent_motion")
    results["normal_stop"] = m.evaluate(0.57).status.value

    m = StopEvidenceMonitor()
    m.request_stop(0.0)
    m.record_motion(1.55, 0.3, 0.0, "independent_motion")
    results["disconnected_stop_while_moving"] = m.evaluate(1.6).status.value

    m = StopEvidenceMonitor()
    m.request_stop(0.0)
    m.record_downstream_command(0.1, 0.0, 0.0)
    for t in [0.2, 0.4, 0.6]:
        m.record_motion(t, 0.0, 0.0, "odom")  # rejected by source policy
    m.record_motion(1.52, 0.25, 0.0, "independent_motion")
    results["frozen_odom_but_motion_continues"] = m.evaluate(1.55).status.value

    m = StopEvidenceMonitor()
    m.request_stop(0.0)
    m.record_downstream_command(1.45, 0.0, 0.0)
    results["no_independent_motion"] = m.evaluate(1.55).status.value

    m = StopEvidenceMonitor()
    m.request_stop(0.0)
    m.record_downstream_command(0.1, 0.0, 0.0)
    for t in [0.15, 0.35, 0.55]:
        m.record_motion(t, 0.0, 0.0, "independent_motion")
    first = m.evaluate(0.57).status.value
    m.record_downstream_command(0.61, 0.3, 0.0)
    m.record_motion(0.62, 0.2, 0.0, "independent_motion")
    results["rebound_after_stop_evidenced"] = {"before": first, "after": m.evaluate(0.64).status.value}

    expected = {
        "normal_stop": "STOP_EVIDENCED",
        "disconnected_stop_while_moving": "STOP_FAILED",
        "frozen_odom_but_motion_continues": "STOP_FAILED",
        "no_independent_motion": "UNVERIFIABLE",
        "rebound_after_stop_evidenced": {"before": "STOP_EVIDENCED", "after": "STOP_FAILED"},
    }
    if results != expected:
        print(json.dumps(results, indent=2))
        raise SystemExit("Unexpected result: scenario contract failed")
    print(json.dumps({"synthetic_contract_passed": True, "results": results}, indent=2))


def main():
    run_scenarios()


if __name__ == "__main__":
    main()
