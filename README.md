# Stop Evidence Monitor (SEM)

**ROS 2 research prototype | Simulation-validated | Not safety-certified**

SEM assesses whether a requested robot stop has supporting command
and motion evidence, detects selected failed-stop scenarios, and
reports uncertainty when evidence is insufficient.

## Critical known failure — NOT SOLVED

**Correlated sensor spoofing can cause false stop confirmation.**

If a robot continues moving while:
- LiDAR reports constant ranges with plausible fresh timestamps,
- wheel odometry falsely reports zero velocity,
- downstream commands appear to be zero,

SEM can return `STOP_EVIDENCED` even though the robot is moving.

This vulnerability is reproduced in a synthetic test. It has
NOT been resolved or validated against real hardware.

See docs/known-failures.md.

## What SEM does

- Observes downstream motion commands.
- Evaluates timestamped motion evidence.
- Detects selected continued-motion and command-path failures.
- Reports STOP_EVIDENCED, STOP_FAILED, or UNVERIFIABLE.
- Rejects insufficient or stale evidence under its configured rules.

STOP_EVIDENCED means evidence supports standstill under the
experiment's assumptions. It is not proof of physical safety.

## Experimental results

### Gazebo simulation (M4)

| Scenario | Result |
|---|---|
| Normal stop | STOP_EVIDENCED |
| Misrouted stop | STOP_FAILED |
| Missing LiDAR | UNVERIFIABLE |

Misrouted-stop trial: **0.5046 m displacement after the bad stop**, measured in **one** recorded M4 trial, not an average. The archived M4 run contains **one trial per scenario** (normal, misrouted stop, and missing LiDAR); it does not establish a repeated-run success rate.

**Baseline distinction:** A command-only check that equates successful stop publication with physical standstill would miss this injected wrong-topic failure. This is an **illustrative baseline assumption**, not a separately implemented or benchmarked competing controller.

The experiment uses a fixed-wall LiDAR range-rate estimator.
Only longitudinal motion toward a static wall is assessed.
Gazebo model pose is used only as evaluation ground truth.

**Implementation context:** ROS 2 Lyrical; Gazebo Sim differential-drive model with fixed-wall LiDAR; Python (3.14.4 in M6 manifest). The archived manifest does not pin the Gazebo executable version.

### Automated tests (M5)

41 tests passed:
- 40 tests asserting expected software behavior.
- 1 test documenting a known false-confirmation vulnerability.

The documented known-failure test is `test/test_m5_final.py::test_known_correlated_spoof_limitation`.
A passing known-failure test DOES NOT mean the vulnerability
has been corrected.

Live wheel-odometry veto integration and a live consumer
freshness lease remain unvalidated.

## Assumptions

A1. Motion estimates are meaningful only in observable directions
    and under documented environmental conditions.

A2. Relevant sensor dependencies and failure assumptions are known.
    Sensor agreement alone does not establish physical truth.

A3. Evidence must be fresh and continuously available. Missing
    verdicts must not be interpreted as successful stops.

## Reproduce

Run software tests:

    python3 -m unittest discover -s test -q

Generate the isolated Gazebo test world:

    python3 scripts/m4_make_world.py

Start the world in a separate terminal:

    gz sim -s -r ~/stop_evidence_results/sem_m4.sdf

With ROS 2 Lyrical sourced, run:

    python3 scripts/m4_live_trial.py

Only the sem_m4 world should be active.
Do not connect these fault-injection experiments to a real robot.

## Safety and deployment limits

SEM is advisory research software.

It does not replace:
- Hardware emergency stops
- Safety PLCs
- Certified drive safety functions
- Safety-rated motor-controller watchdogs

No hardware safety or arbitrary-environment reliability claims
are made. Correlated faults and unobservable motion remain open.

## Release

v0.1.0-sim — controlled simulation research prototype.
