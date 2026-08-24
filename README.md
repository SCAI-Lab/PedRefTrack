# PedRefTrack

PedRefTrack is a pure-Python 3D pedestrian tracker with a detector-agnostic ROS 2 Humble adapter. The ROS node consumes prepared [`vision_msgs/msg/Detection3DArray`](https://docs.ros.org/en/humble/p/vision_msgs/msg/Detection3DArray.html) bounding boxes, performs detector-only tracking, and publishes pedestrian trajectories as `pedestrian_tracking_msgs/msg/TrackedPedestrianArray`. It can also publish the tracked bounding boxes as a `Detection3DArray`.

The benchmark-compatible implementation is also bundled in [SCAI-Lab/tracker_eval](https://github.com/SCAI-Lab/tracker_eval) behind `pedreftrack_adapter.py`. The ROS-independent core in `ros2/pedreftrack/pedreftrack/core.py` is maintained consistently with the evaluation implementation.

## Repository structure

This repository contains two ROS 2 packages:

```text
PedRefTrack/
├── README.md
├── LICENSE
└── ros2/
    ├── pedreftrack/
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── config/
    │   ├── launch/
    │   ├── resource/
    │   └── pedreftrack/
    └── pedestrian_tracking_msgs/
        ├── package.xml
        ├── CMakeLists.txt
        └── msg/
            ├── TrackedPedestrian.msg
            └── TrackedPedestrianArray.msg
```

`pedestrian_tracking_msgs` remains an independent ROS 2 interface package even though it is distributed in the same Git repository. Once the workspace is built and sourced, its generated message types are available to any ROS 2 package in that environment.

## Platform and dependencies

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- `rclpy`
- `vision_msgs`
- `geometry_msgs`
- `std_msgs`
- `tf2_ros`
- NumPy and SciPy
- the bundled `pedestrian_tracking_msgs` package

## Build

Clone this repository anywhere below the `src` directory of a ROS 2 workspace:

```text
ros2_ws/
└── src/
    └── PedRefTrack/
        └── ros2/
            ├── pedreftrack/
            └── pedestrian_tracking_msgs/
```

Install the dependencies and build both packages:

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -r -y

colcon build \
  --symlink-install \
  --packages-select \
    pedestrian_tracking_msgs \
    pedreftrack

source install/setup.bash
```

Verify the installation:

```bash
ros2 pkg prefix pedestrian_tracking_msgs
ros2 pkg prefix pedreftrack

ros2 interface show \
  pedestrian_tracking_msgs/msg/TrackedPedestrianArray
```

## Run

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch pedreftrack pedreftrack.launch.py
```

Default interfaces:

| Direction | Topic | Type |
|---|---|---|
| Input | `/pedestrian_detections_3d` | `vision_msgs/msg/Detection3DArray` |
| Output | `/tracked_pedestrians` | `pedestrian_tracking_msgs/msg/TrackedPedestrianArray` |
| Optional output | `/pedreftrack/tracked_detections_3d` | `vision_msgs/msg/Detection3DArray` |

Override topics or frames with ROS parameters:

```bash
ros2 run pedreftrack pedreftrack_node --ros-args \
  -p input_topic:=/my_detector/detections_3d \
  -p tracking_frame:=map \
  -p output_topic:=/tracked_pedestrians
```

If `tracking_frame` is empty, boxes are tracked in the frame specified by the incoming message. If it is set, the node looks up a timestamped TF transform and tracks and publishes in that frame. `Detection3DArray.header.frame_id` must not be empty.

## Detection mapping

For every input `Detection3D`:

- `bbox.center.position` becomes `(cx, cy, cz)`;
- `bbox.size.{x,y,z}` becomes `(length, width, height)`;
- the bounding-box quaternion yaw becomes `rot_z`;
- the highest-scoring hypothesis supplies the confidence and class;
- `pedestrian_class_id` filters non-pedestrian hypotheses when set.

Tracked box IDs are written to `Detection3D.id`. The compact custom output contains the track ID, XY position, EMA-smoothed velocity, and configured pedestrian radius. The velocity is estimated by the ROS adapter for publication and does not affect PedRefTrack association.

## Custom tracking messages

`pedestrian_tracking_msgs/msg/TrackedPedestrian` contains:

```text
uint32 track_id
float32 x
float32 y
float32 vx
float32 vy
float32 radius
```

`pedestrian_tracking_msgs/msg/TrackedPedestrianArray` contains:

```text
std_msgs/Header header
TrackedPedestrian[] pedestrians
```

## Configuration

The default ROS configuration is stored in `ros2/pedreftrack/config/pedreftrack.yaml`. It exposes every PedRefTrack parameter present in the tracker-evaluation CLI, using the same defaults:

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

The ROS node deliberately implements detector-only tracking and does not expose GT-assisted mode. GT-assisted operation is a diagnostic evaluation mode available only in [`tracker_eval`](https://github.com/SCAI-Lab/tracker_eval).

## Pure-Python use

The tracker core accepts lightweight `Detection` objects and does not depend on ROS:

```python
from pedreftrack import Box3D, Detection, PedRefTrack

tracker = PedRefTrack()
tracks = tracker.step(
    "0",
    [
        Detection(
            "0",
            -1,
            Box3D(2.0, 0.0, 0.85, 0.5, 0.5, 1.7, 0.0),
            0.9,
        )
    ],
    timestamp=0.0,
)
```

## License

Both ROS 2 packages in this repository are licensed under the MIT License.
