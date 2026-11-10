#!/usr/bin/env python3
"""Gazebo-only, bounded STOP-ROUTING FAULT demonstration.

Uses Gazebo model pose as *experiment oracle* only. Does not prove that a
robot-side Stop Evidence Monitor has independent physical stop evidence.

Prerequisite: ONLY the official `diff_drive` Gazebo world is running.
Never connect this script to real robot actuators.
"""
from __future__ import annotations

import json
import math
import re
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

WORLD = "diff_drive"
MODEL = "vehicle_blue"
REAL_TOPIC = "/model/vehicle_blue/cmd_vel"
BROKEN_TOPIC = "/model/vehicle_blue/unconnected_stop"
SPEED_MPS = 0.12
MAX_DRIVE_S = 6.0


def call(*args: str, timeout: float = 8.0) -> str:
    p = subprocess.run(args, capture_output=True, text=True, timeout=timeout, check=False)
    if p.returncode:
        raise RuntimeError(f"command failed ({p.returncode}): {' '.join(args)}\n{p.stderr.strip()}")
    return p.stdout


def available_worlds(service_list: str) -> list[str]:
    return sorted(set(re.findall(r"^/world/([^/]+)/state$", service_list, flags=re.M)))


def parse_pose(output: str) -> tuple[float, float]:
    # Match the x/y row directly after the pose header, not earlier coordinates.
    pattern = (r"Pose\s*\[\s*XYZ\s*\(m\)\s*\]\s*\[\s*RPY\s*\(rad\)\s*\]\s*:"
               r"\s*\[\s*([-+\deE.]+)\s+([-+\deE.]+)\s+([-+\deE.]+)\s*\]")
    match = re.search(pattern, output)
    if not match:
        raise RuntimeError(f"Could not parse pose: {output[-600:]!r}")
    return float(match.group(1)), float(match.group(2))


def pose() -> tuple[float, float]:
    out = call("gz", "model", "-m", MODEL, "--pose", timeout=12.0)
    if f"Name: {MODEL}" not in out and f"Name: [{MODEL}]" not in out:
        raise RuntimeError(f"Incorrect or missing model: {out[-500:]}")
    if f"world [{WORLD}]" not in out:
        raise RuntimeError(f"Gazebo world selection changed: {out[-500:]}")
    return parse_pose(out)


def publish(topic: str, speed: float) -> None:
    call("gz", "topic", "-t", topic, "-m", "gz.msgs.Twist", "-p",
         f"linear: {{x: {speed}}}, angular: {{z: 0.0}}", timeout=4.0)


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(b[0]-a[0], b[1]-a[1])


def run() -> int:
    worlds = available_worlds(call("gz", "service", "-l"))
    if worlds != [WORLD]:
        print(f"ABORT: expected only [{WORLD}] but found {worlds}; stop other Gazebo worlds first")
        return 2

    _ = pose()  # World and robot validated before any motion commands.
    errors: list[str] = []
    stop_writer = threading.Event()
    count = [0]

    def forward_writer() -> None:
        began = time.monotonic()
        while not stop_writer.is_set() and time.monotonic() - began < MAX_DRIVE_S:
            try:
                publish(REAL_TOPIC, SPEED_MPS)
                count[0] += 1
            except Exception as exc:
                errors.append(str(exc))
                stop_writer.set()
                break
            stop_writer.wait(0.1)

    def settle() -> tuple[tuple[float, float], tuple[float, float]]:
        # The command producer has already ceased before sending true zeros.
        for _ in range(5):
            try:
                publish(REAL_TOPIC, 0.0)
            except Exception as exc:
                errors.append(f"zero command: {exc}")
            time.sleep(0.1)
        at_stop = pose()
        time.sleep(2)
        return at_stop, pose()

    result: dict[str, object] = {
        "experiment": "intentionally_misrouted_stop_while_forward_source_continues",
        "world": WORLD, "model": MODEL, "command_topic": REAL_TOPIC,
        "bad_stop_topic": BROKEN_TOPIC, "speed_mps": SPEED_MPS,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "note": "Gazebo pose is an evaluation oracle, not deployed monitor evidence",
    }
    thread = threading.Thread(target=forward_writer, daemon=True)
    try:
        before = pose()
        thread.start()
        time.sleep(1.2)
        just_before_bad_stop = pose()
        if errors:
            raise RuntimeError(f"publisher failed: {errors}")

        # DELIBERATE FAULT: publish the apparent stop to the WRONG topic.
        # It has no connection to the DiffDrive input, while the forward
        # publisher continues on the real command topic.
        publish(BROKEN_TOPIC, 0.0)
        at_bad_stop = pose()
        time.sleep(1.5)
        after_bad_stop = pose()

        stop_writer.set()
        thread.join(timeout=5)
        if thread.is_alive():
            raise RuntimeError("forward publisher did not terminate")
        at_real_stop, settled = settle()
        result.update({
            "forward_commands_sent": count[0],
            "before_xy": before,
            "before_bad_stop_xy": just_before_bad_stop,
            "bad_stop_xy": at_bad_stop,
            "after_bad_stop_xy": after_bad_stop,
            "at_real_stop_xy": at_real_stop,
            "after_settle_xy": settled,
            "pre_fault_displacement_m": round(distance(before, just_before_bad_stop), 5),
            "post_bad_stop_displacement_m": round(distance(at_bad_stop, after_bad_stop), 5),
            "settle_window_displacement_m": round(distance(at_real_stop, settled), 5),
        })
        if count[0] < 3 or distance(before, just_before_bad_stop) < 0.03:
            result["result"] = "INCONCLUSIVE_NO_VERIFIED_PREFAULT_MOTION"
        elif distance(at_bad_stop, after_bad_stop) >= 0.03:
            result["result"] = "MISROUTED_STOP_MOTION_CONTINUED"
        else:
            result["result"] = "INCONCLUSIVE_NO_POSTFAULT_MOTION"
    except Exception as exc:
        result["result"] = "ABORTED"
        result["error"] = str(exc)
    finally:
        stop_writer.set()
        if thread.is_alive():
            thread.join(timeout=5)
        # Best-effort zero commands, even if an earlier phase failed.
        for _ in range(5):
            try:
                publish(REAL_TOPIC, 0.0)
            except Exception as exc:
                errors.append(f"cleanup zero: {exc}")
            time.sleep(0.1)
        if errors:
            result["command_errors"] = errors

    output_dir = Path.home() / "stop_evidence_results"
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / "reference_stop_route_trial.json"
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    print(f"Saved: {output}")
    return 0 if result.get("result") == "MISROUTED_STOP_MOTION_CONTINUED" else 1


if __name__ == "__main__":
    raise SystemExit(run())
