#!/usr/bin/env python3
"""M4 straight-line LiDAR wall-distance experiment; simulation ONLY.

The robot's model pose is evaluated as an oracle, NEVER fed to the monitor.
Single fixed wall makes LiDAR range-rate a *conditional* independent motion
proxy for longitudinal motion only. Neither rotation nor sideways motion can
be certified by this experiment.
"""
from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from stop_evidence_monitor.core import Config, Status, StopEvidenceMonitor
from stop_evidence_monitor.lidar_motion import FixedWallLidarMotion
from reference_stop_route_trial import call, parse_pose, distance

WORLD = 'sem_m4'
CMD = '/model/vehicle_blue/cmd_vel'
BAD = '/model/vehicle_blue/unconnected_stop'
SCAN = '/sem/m4_scan'
SPEED = 0.12
OUT = Path.home() / 'stop_evidence_results/m4_live_trial.json'
MAX_FORWARD_S = 6.0


def publish(topic, speed):
    call('gz', 'topic', '-t', topic, '-m', 'gz.msgs.Twist', '-p',
         f'linear: {{x: {speed}}}, angular: {{z: 0.0}}', timeout=5)


def pose():
    text = call('gz', 'model', '-m', 'vehicle_blue', '--pose', timeout=12)
    if f'world [{WORLD}]' not in text or 'Name: vehicle_blue' not in text:
        raise RuntimeError('Wrong world/model was selected: ' + text[-250:])
    return parse_pose(text)


def stop_process(proc):
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=4)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=4)


class Observer:
    def __init__(self):
        self.node = rclpy.create_node('sem_m4_fixed_wall_observer')
        self.motion = FixedWallLidarMotion()
        self.monitor = StopEvidenceMonitor(Config(
            deadline_s=3.5, hold_s=0.4, min_stopped_samples=3,
            motion_fresh_s=1.0, command_fresh_s=1.4, max_motion_gap_s=0.65,
            linear_stop_mps=0.035, independent_source='sem_fixed_wall_lidar',
        ))
        self.drop_scan = False
        self.scans = 0
        self.velocities = []
        self.command_count = 0
        self.last_decision = None
        self.node.create_subscription(Twist, CMD, self.on_cmd, 20)
        self.node.create_subscription(LaserScan, SCAN, self.on_scan, 20)

    def on_cmd(self, msg):
        now = time.monotonic()
        self.command_count += 1
        self.monitor.record_downstream_command(now, msg.linear.x, msg.angular.z)

    def on_scan(self, msg):
        self.scans += 1
        if self.drop_scan:  # simulated sensor loss; sensor itself continues publishing
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        receipt = time.monotonic()
        result = self.motion.observe(stamp, receipt, msg.ranges,
                                     msg.range_min, msg.range_max)
        if result:
            self.velocities.append((result.receipt_s, result.forward_speed_mps))
            self.monitor.record_motion(result.receipt_s, result.forward_speed_mps, 0,
                                       'sem_fixed_wall_lidar')

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=min(0.06, end - time.monotonic()))
            if self.monitor._requested_s is not None:
                self.last_decision = self.monitor.evaluate(time.monotonic())


class Writer:
    def __init__(self, topic=CMD, speed=SPEED, max_s=MAX_FORWARD_S):
        self.topic, self.speed, self.max_s = topic, speed, max_s
        self.cancel = threading.Event()
        self.errors = []
        self.sent = 0
        self.thread = threading.Thread(target=self._worker, daemon=True)

    def _worker(self):
        start = time.monotonic()
        while not self.cancel.is_set() and time.monotonic() - start < self.max_s:
            try:
                publish(self.topic, self.speed)
                self.sent += 1
            except Exception as ex:
                self.errors.append(str(ex))
                self.cancel.set()
                return
            self.cancel.wait(0.10)

    def start(self):
        self.thread.start()

    def stop(self):
        self.cancel.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=6)
        if self.thread.is_alive():
            raise RuntimeError('command writer did not terminate')
        if self.errors:
            raise RuntimeError('command writer error: ' + repr(self.errors))


def run():
    if not os.getenv('ROS_DISTRO'):
        print('ABORT: ROS environment not sourced')
        return 2
    services = call('gz', 'service', '-l', timeout=10)
    worlds = sorted(set(x.split('/')[2] for x in services.splitlines()
                        if x.startswith('/world/') and x.endswith('/state')))
    if worlds != [WORLD]:
        print(f'ABORT: expected only {WORLD}; found {worlds}')
        return 2
    pose()  # preflight the world selection before any motor command

    specs = [f'{CMD}@geometry_msgs/msg/Twist[gz.msgs.Twist',
             f'{SCAN}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']
    bridge = subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', *specs],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                              start_new_session=True)
    rclpy.init()
    observer = Observer()
    writers = []
    findings = {}
    result = {
        'experiment': 'sem_m4_fixed_wall_lidar', 'world': WORLD,
        'independent_motion_input': 'fresh timestamped 1-D LiDAR range-rate to fixed wall',
        'odometry_input': 'NONE',
        'oracle': 'gz model pose: scored only, never supplied to classifier',
        'classification_scope': 'longitudinal motion toward a static wall only',
        'result': 'INCONCLUSIVE',
    }

    def send_zero(seconds=1.5):
        # Keep ROS callbacks running while Gazebo receives stop commands.
        zero_writer = Writer(topic=CMD, speed=0.0, max_s=seconds)
        writers.append(zero_writer)
        zero_writer.start()
        try:
            observer.spin(seconds)
        finally:
            zero_writer.stop()

    def preflight_motion(max_forward_s=MAX_FORWARD_S):
        observer.monitor = StopEvidenceMonitor(observer.monitor.cfg)
        observer.last_decision = None
        before = pose()
        writer = Writer(max_s=max_forward_s)
        writers.append(writer)
        writer.start()
        observer.spin(2.6)
        after = pose()
        recent_speeds = [abs(v) for t, v in observer.velocities
                         if time.monotonic() - t < 3.5]
        if (writer.sent < 2 or distance(before, after) < 0.03 or
                not recent_speeds or max(recent_speeds) < 0.04):
            raise RuntimeError('Preflight: no verified travel and LiDAR wall-relative motion')
        return writer, before, after

    try:
        observer.spin(3.0)
        if bridge.poll() is not None:
            raise RuntimeError('one-way sensor bridge exited early')
        if observer.scans < 4 or observer.motion.speed_samples < 2:
            raise RuntimeError('LiDAR preflight failed: insufficient valid range-rate; check Gazebo sensor log')
        print('LIDAR_PREFLIGHT_PASS: scans=', observer.scans,
              'speed_samples=', observer.motion.speed_samples)

        # Positive control: a real stop with ongoing zero velocity commands.
        writer, start_xy, moving_xy = preflight_motion()
        writer.stop()
        observer.monitor.request_stop(time.monotonic())
        observer.last_decision = None
        send_zero(4.2)
        normal = observer.last_decision.status if observer.last_decision else Status.IDLE
        end_xy = pose()
        findings['normal'] = {
            'status': normal.value,
            'pre_stop_displacement_m': round(distance(start_xy, moving_xy), 5),
            'post_stop_displacement_m': round(distance(moving_xy, end_xy), 5),
        }
        print('NORMAL_STOP:', findings['normal'])
        if normal != Status.STOP_EVIDENCED:
            raise RuntimeError('Normal stop was not evidenced; no later tests claimed')

        # Fault: stop routed to a topic the drivetrain does not consume;
        # command producer continues. The LiDAR observer does not see odom.
        writer, start_xy, moving_xy = preflight_motion(max_forward_s=14.0)
        observer.monitor.request_stop(time.monotonic())
        observer.last_decision = None
        publish(BAD, 0.0)
        at_bad = pose()
        observer.spin(3.9)
        after_bad = pose()
        failed = observer.last_decision.status if observer.last_decision else Status.IDLE
        findings['misrouted'] = {
            'status': failed.value,
            'displacement_after_bad_stop_m': round(distance(at_bad, after_bad), 5),
            'observed_lidar_motion_samples': observer.motion.speed_samples,
        }
        print('MISROUTED_STOP:', findings['misrouted'])
        writer.stop()
        send_zero(1.5)
        if failed != Status.STOP_FAILED or distance(at_bad, after_bad) < 0.03:
            raise RuntimeError('Fault detection or oracle movement gate failed')

        # Missing motion evidence must NOT produce a stop success.
        observer.drop_scan = True
        observer.monitor.request_stop(time.monotonic())
        observer.last_decision = None
        send_zero(3.9)
        missing = observer.last_decision.status if observer.last_decision else Status.IDLE
        findings['missing_lidar'] = {'status': missing.value}
        print('MISSING_LIDAR:', findings['missing_lidar'])
        if missing != Status.UNVERIFIABLE:
            raise RuntimeError('Missing LiDAR wrongly confirmed stopping')
        result['result'] = 'M4_STRAIGHT_LINE_GATES_PASSED'
    except Exception as ex:
        result['result'] = 'ABORTED'
        result['first_failure'] = str(ex)
    finally:
        for writer in writers:
            try:
                writer.stop()
            except Exception as ex:
                result.setdefault('cleanup_errors', []).append(str(ex))
        try:
            for _ in range(5):
                publish(CMD, 0.0)
                time.sleep(.1)
        except Exception as ex:
            result.setdefault('cleanup_errors', []).append('zero: ' + str(ex))
        result['scans'] = observer.scans
        result['accepted_scans'] = observer.motion.accepted_scans
        result['valid_speed_samples'] = observer.motion.speed_samples
        result['findings'] = findings
        observer.node.destroy_node()
        rclpy.shutdown()
        stop_process(bridge)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    print('Saved:', OUT)
    return 0 if result['result'] == 'M4_STRAIGHT_LINE_GATES_PASSED' else 1


if __name__ == '__main__':
    sys.exit(run())
