# Scout Minimum Detection Distance Report

Status: pending physical Scout measurement

## Setup

- Platform: Scout 2.0 by default; record `is_scout_mini` if changed.
- Sensor chain: Robosense RS16 -> `rs_to_velodyne` -> SBG -> FAST-LIO.
- Radial filters: Robosense `min_distance=0.2 m`, FAST-LIO `blind=0.2 m`.
- Body returns: filtered by the FAST-LIO self-filter box.
- Target: soft cylinder or person surrogate outside the robot envelope.

## Procedure

Run the real chain without wheel motion:

```bash
roslaunch swarm_test scout_secbf_real.launch \
  start_base:=false cmd_vel_topic:=/cmd_vel_secbf_dryrun
```

At each clearance `0.2, 0.5, 1.0, 1.5, 2.0 m`, hold the target still for at
least 10 seconds and run:

```bash
rosrun swarm_test check_scout_real_topics.py \
  --clearance-m 0.5 \
  --duration-sec 10 \
  --rosbag-path /path/to/scout_min_distance_0p5m.bag \
  --output minimum_detection_distance_report.md
```

Repeat the command with the actual clearance. Record the rosbag path and add
manual observations below.

## Acceptance

- `0.5-2.0 m`: every required topic has messages and positive target output.
- `0.2 m`: raw point cloud remains visible; this is not an automatic-motion distance.
- No persistent self-body cluster or predicted obstacle is present.
- Removing the target produces an empty typed snapshot after the configured timeout.

## Rosbag And Notes

- Rosbag: pending
- Operator: pending
- Scout model: pending
- Robosense serial: pending
- SBG serial: pending
- Notes: pending

## Measurements

The checker appends measurement rows below this section.
