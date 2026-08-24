# PedRefTrack

PedRefTrack is a pure-Python 3D pedestrian tracker with a general ROS2 Humble adapter. The node consumes prepared `vision_msgs/msg/Detection3DArray` bounding boxes, runs normal detector-only tracking, and publishes both `pedestrian_tracking_msgs/msg/TrackedPedestrianArray` and optional tracked `Detection3DArray` boxes.

This repository has no dependency on `m_detector` or a particular point-cloud detector. A separate `dynout_pedestrian_detector` package can convert `/m_detector/dyn_out` into the standard input message when needed.

The benchmark-compatible implementation is also bundled in [Draxran/tracker_eval](https://github.com/Draxran/tracker_eval) behind `pedreftrack_adapter.py`. The core in `pedreftrack/core.py` is ROS-independent and functionally shared between both repositories.

## Platform and dependencies

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10
- `vision_msgs`, `tf2_ros`, NumPy and SciPy
- the existing `pedestrian_tracking_msgs` package in the same colcon workspace

`pedestrian_tracking_msgs` is intentionally not duplicated here.

## Build

Place this repository beside your existing message package:

```text
ros2_ws/src/
├── PedRefTrack/
└── pedestrian_tracking_msgs/
```

Then build normally:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select pedestrian_tracking_msgs pedreftrack
source install/setup.bash
```

## Run

```bash
ros2 launch pedreftrack pedreftrack.launch.py
```

Default topics:

| Direction | Topic | Type |
|---|---|---|
| input | `/pedestrian_detections_3d` | `vision_msgs/Detection3DArray` |
| output | `/tracked_pedestrians` | `pedestrian_tracking_msgs/TrackedPedestrianArray` |
| optional output | `/pedreftrack/tracked_detections_3d` | `vision_msgs/Detection3DArray` |

Override topics or frames with parameters:

```bash
ros2 run pedreftrack pedreftrack_node --ros-args \
  -p input_topic:=/my_detector/detections_3d \
  -p tracking_frame:=map \
  -p output_topic:=/tracked_pedestrians
```

If `tracking_frame` is empty, boxes are tracked in the incoming message frame. If it is set, the node looks up a timestamped TF transform and publishes in that frame. The input header frame must be non-empty.

## Detection mapping

For every input `Detection3D`:

- `bbox.center.position` becomes `(cx, cy, cz)`;
- `bbox.size.{x,y,z}` becomes `(length, width, height)`;
- bbox quaternion yaw becomes `rot_z`;
- the highest-scoring hypothesis supplies confidence and class;
- `pedestrian_class_id` filters non-pedestrian hypotheses when set.

Tracked box IDs are written to `Detection3D.id`. The compact custom output contains ID, XY position, EMA velocity and configured radius. Velocity is an adapter output estimate; it does not change PedRefTrack association.

## Tracker parameters

`config/pedreftrack.yaml` exposes every PedRefTrack parameter present in the tracker-evaluation CLI, with identical defaults:

| Parameter | Default |
|---|---:|
| `tracker.fps` | 15.0 |
| `tracker.T_reid_base_s` | 2.5 |
| `tracker.T_reid_static_s` | 5.0 |
| `tracker.confirmation_target_s` | 0.25 |
| `tracker.confirmation_one_hit_score` | 0.95 |
| `tracker.confirmation_min_score` | 0.50 |
| `tracker.tentative_max_gap_s` | 0.50 |
| `tracker.motion_robustness_history_s` | 1.00 |
| `tracker.motion_robustness_immediate_history_s` | 0.25 |
| `tracker.motion_error_free_m` | 0.05 |
| `tracker.motion_error_half_decay_m` | 0.038 |
| `tracker.T_out_min_s` | 0.50 |
| `tracker.T_out_max_s` | 2.0 |
| `tracker.assoc_iou_first_pass_thr` | 0.33 |
| `tracker.dist_gate_m` | 0.4 |
| `tracker.z_gate_m` | 0.5 |
| `tracker.kf_max_gate_m` | 1.0 |

The ROS node deliberately does not expose or implement GT-assisted mode. That diagnostic mode remains available only in `tracker_eval`.

## Pure-Python use

The core accepts lightweight `Detection` objects without ROS:

```python
from pedreftrack import Box3D, Detection, PedRefTrack

tracker = PedRefTrack()
tracks = tracker.step(
    "0",
    [Detection("0", -1, Box3D(2.0, 0.0, 0.85, 0.5, 0.5, 1.7, 0.0), 0.9)],
    timestamp=0.0,
)
```

MIT licensed.
