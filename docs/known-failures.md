# Known limitation: correlated false standstill evidence

## Status
OPEN — NOT SOLVED

## Problem
SEM can produce STOP_EVIDENCED while a robot is moving if
multiple evidence sources simultaneously provide convincing
but incorrect stationary readings.

## Reproduction
The synthetic test
test/test_m5_final.py::test_known_correlated_spoof_limitation
constructs the following situation:

1. The real robot is assumed to continue moving.
2. LiDAR returns constant wall distances with increasing timestamps.
3. Wheel odometry reports zero velocity.
4. The observed command is zero.
5. SEM returns STOP_EVIDENCED.

Physical motion is an external assumption in this test, not a
live Gazebo fault-injection measurement.

## Why it matters
Sensor agreement alone cannot establish physical standstill.
A shared or correlated fault can defeat this evidence contract.

## Scope
This is an unresolved limitation, not a certified safety defect
or proof of failure frequency on physical robots.

## Future work
Study sensor provenance, common-cause failures, and independent
physical motion evidence. No general solution is claimed.
