# px4-offboard-smc

Sliding-mode control for a T2 Cruza tiltrotor VTOL, precision-landing on an
AprilTag platform in PX4 offboard mode. Master's thesis work.

Two control laws are implemented — classical SMC and super-twisting SMC
(`controller_type:=smc|stsmc`), with stsmc as the primary controller. Landing
runs a 4-phase state machine (approach, vision descent, commit, touchdown)
driven by a relative-state EKF fused from AprilTag detections, with an
external-force observer for wind rejection.

## Layout

- `src/`, `include/` — controller node and the two control laws
- `src/TrajectoryPublishers/` — trajectory generation and the landing state machine
- `config/` — controller gains and vehicle parameters
- `launch/` — ROS 2 launch files
- `scripts/visualizer.py` — plotting helper

## Build

Standard ROS 2 / colcon package (`px4_offboard_lowlevel`), built inside a
workspace alongside PX4's `px4_msgs` and `apriltag_msgs`.

```
colcon build --packages-select px4_offboard_lowlevel
```
