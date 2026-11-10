# Experiment contract (v0.1)

## Question
Can an observation-only verifier avoid falsely reporting stop success when the command path or motion-feedback path is broken?

## Null and alternative
- H0: Simple command-only and odometry-only checkers make no additional false-success claims compared with evidence-based checking in the defined fault scenarios.
- H1: At least one defined injected fault causes the simple checker to wrongly declare success while the evidence-based monitor correctly reports STOP_FAILED or UNVERIFIABLE.

## Decision semantics
- STOP_PENDING: stop requested; observation window still open.
- STOP_EVIDENCED: fresh downstream **zero command** and multiple fresh near-zero samples from a separately justified motion-observation source, over a dwell interval. Means **evidence**, not guaranteed physical standstill.
- STOP_FAILED: fresh downstream nonzero command or fresh motion after the deadline; also motion or nonzero command after STOP_EVIDENCED.
- UNVERIFIABLE: deadline reached without enough evidence, or evidence expires after initial STOP_EVIDENCED.

## Validity limitations
- A ROS topic with a plausible name is not evidence of independent sensing. Don't feed odometry derived from the failing sensor or merely command-derived velocity into `/sem/independent_motion`.
- Wheel encoder measurements cannot prove zero body translation if the robot is sliding or being pushed; IMU gyroscope alone cannot prove absence of linear motion.
- In simulation, Gazebo ground-truth model pose is RESERVED for scorer/evaluation and must not be subscribed to by the runtime verifier.
- Receipt timestamps are local `time.monotonic()`; they do not prove sender timestamps or device freshness. Add sensor timestamp/freshness and QoS/dropout checks before hardware use.
- v0.1 has no authenticated controller acknowledgement, safety arbiter ownership, stop authority, or hardware verification.
- The node never sends motor commands. It cannot enforce braking, prevent restart, or make functional-safety claims.
- Any `STOP_FAILED`/`UNVERIFIABLE` are latched until a new stop request. `STOP_EVIDENCED` remains conditional on fresh data and can downgrade.

## First experiments
1. Pure Python synthetic scenarios and unit tests (required before integration).
2. Observe a real simulated robot downstream command path and independent feedback; validate provenance before connection.
3. Create a *separate* experimental fault injector and scorer. Never put fault injection on a physical robot.
4. Predeclare baseline simple checker, inject faults with fixed seeds, score from Gazebo ground truth only in benchmark.

## Preliminary thresholds (demonstration defaults; not validated safety limits)
- stop deadline 1.5 s
- motion freshness 0.25 s
- command freshness 0.5 s
- standstill hold 0.3 s, >=3 samples, <=0.25 s gap
- speed thresholds 0.02 m/s and 0.03 rad/s

## Metrics for later simulation campaign
False-confirmation rate, failure-detection delay, unverifiable fraction, true-stop evidence latency, stop distance, processing overhead. Include sample counts and 95% confidence intervals. Do not call a zero-collision run safety certification.
