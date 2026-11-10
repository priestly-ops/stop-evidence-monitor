
#!/usr/bin/env python3
"""Simulation-only: verify a misrouted stop through observed command traffic."""
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stop_evidence_monitor.core import (
    Config, Status, StopEvidenceMonitor
)
from reference_stop_route_trial import (
    WORLD, REAL_TOPIC, BROKEN_TOPIC, SPEED_MPS,
    available_worlds, call, pose, publish, distance,
)

OUT = Path.home() / 'stop_evidence_results/reference_live_monitor_trial.json'


def run():
    if available_worlds(call('gz', 'service', '-l')) != [WORLD]:
        print('ABORT: Run only the diff_drive Gazebo world')
        return 2

    pose()

    if not os.getenv('ROS_DISTRO'):
        print('ABORT: Source your ROS 2 environment first')
        return 2

    spec = f'{REAL_TOPIC}@geometry_msgs/msg/Twist[gz.msgs.Twist'

    bridge = subprocess.Popen(
        ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', spec],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )

    rclpy.init()
    node = rclpy.create_node('sem_reference_cmd_observer')
    monitor = StopEvidenceMonitor(
        Config(deadline_s=2.5, command_fresh_s=1.2)
    )

    observed = []
    stop_requested_at = [None]
    command_violation_at = [None]
    command_grace_s = 0.5
    cancel = threading.Event()
    errors = []
    sent = [0]
    worker = None

    result = {
        'experiment': 'misrouted_stop_live_command_observer',
        'result': 'INCONCLUSIVE',
        'motion_source': 'none: Gazebo pose used only for evaluation',
    }

    def on_cmd(msg):
        t = time.monotonic()
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)
        observed.append((t, vx))
        monitor.record_downstream_command(t, vx, wz)

        if (stop_requested_at[0] is not None
                and t - stop_requested_at[0] >= command_grace_s
                and (abs(vx) > 0.03 or abs(wz) > 0.03)
                and command_violation_at[0] is None):
            command_violation_at[0] = t

    node.create_subscription(Twist, REAL_TOPIC, on_cmd, 10)

    def forward():
        start = time.monotonic()
        while not cancel.is_set() and time.monotonic() - start < 10:
            try:
                publish(REAL_TOPIC, SPEED_MPS)
                sent[0] += 1
            except Exception as exc:
                errors.append(str(exc))
                cancel.set()
                break
            cancel.wait(0.1)

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        until = time.monotonic() + 10
        while node.count_publishers(REAL_TOPIC) == 0 and time.monotonic() < until:
            if bridge.poll() is not None:
                raise RuntimeError('ROS-Gazebo bridge exited')
            spin(0.1)

        if node.count_publishers(REAL_TOPIC) == 0:
            raise RuntimeError('No ROS command-topic observer publisher')

        start_pose = pose()
        worker = threading.Thread(target=forward, daemon=True)
        worker.start()

        until = time.monotonic() + 4
        while time.monotonic() < until:
            spin(0.1)
            if sum(abs(v) > 0.03 for _, v in observed) >= 2:
                break

        moving_pose = pose()
        if (
            errors
            or sum(abs(v) > 0.03 for _, v in observed) < 2
            or distance(start_pose, moving_pose) < 0.03
        ):
            raise RuntimeError(
                'Preflight failed: command observation or motion'
            )

        requested = time.monotonic()
        monitor.request_stop(requested)
        stop_requested_at[0] = requested

        # Inject a stop routed to the wrong topic.
        publish(BROKEN_TOPIC, 0.0)
        fault_pose = pose()

        spin(3.1)
        final_pose = pose()
        decision = monitor.evaluate(time.monotonic())

        nonzero = sum(
            t >= requested and abs(v) > 0.03
            for t, v in observed
        )
        moved = distance(fault_pose, final_pose)

        path_failed = command_violation_at[0] is not None
        result.update(
            monitor_status=decision.status.value,
            monitor_reason=decision.reason,
            command_path_status=(
                "COMMAND_PATH_FAILED"
                if path_failed else "NOT_DEMONSTRATED"
            ),
            command_grace_s=command_grace_s,
            first_command_violation_after_request_s=(
                round(command_violation_at[0] - requested, 3)
                if path_failed else None
            ),
            observed_nonzero_after_stop=nonzero,
            post_fault_displacement_m=round(moved, 5),
            forward_commands_sent=sent[0],
        )

        if path_failed and moved >= 0.03 and not errors:
            result["result"] = (
                "COMMAND_PATH_FAILURE_DETECTED_AND_ORACLE_MOVING"
            )
        else:
            result["result"] = "INCONCLUSIVE"

    except Exception as exc:
        result.update(result='ABORTED', error=repr(exc))

    finally:
        cancel.set()

        if worker is not None:
            worker.join(timeout=5)
            if worker.is_alive():
                result['cleanup_warning'] = (
                    'Forward publisher did not exit'
                )

        # Best-effort simulation-only stop commands.
        for _ in range(5):
            try:
                publish(REAL_TOPIC, 0.0)
            except Exception as exc:
                errors.append(f'Stop cleanup: {exc}')
            time.sleep(0.1)

        if errors:
            result['command_errors'] = errors

        node.destroy_node()
        rclpy.shutdown()

        if bridge.poll() is None:
            os.killpg(bridge.pid, signal.SIGTERM)
            try:
                bridge.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(bridge.pid, signal.SIGKILL)
                bridge.wait(timeout=3)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2) + '\n')

    print(json.dumps(result, indent=2))
    print('Saved:', OUT)

    return (
        0
        if result['result'] ==
        'COMMAND_PATH_FAILURE_DETECTED_AND_ORACLE_MOVING'
        else 1
    )


if __name__ == '__main__':
    raise SystemExit(run())
