"""General ROS2 adapter for PedRefTrack using vision_msgs detections."""

from __future__ import annotations

import math
from typing import Dict, Iterable, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion, Vector3
from pedestrian_tracking_msgs.msg import TrackedPedestrian, TrackedPedestrianArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from .core import PedRefTrack, PedRefTrackConfig
from .types import Box3D, Detection


EXPOSED_TRACKER_PARAMETERS = (
    "fps",
    "T_reid_base_s",
    "T_reid_static_s",
    "confirmation_target_s",
    "confirmation_one_hit_score",
    "confirmation_min_score",
    "tentative_max_gap_s",
    "motion_robustness_history_s",
    "motion_robustness_immediate_history_s",
    "motion_error_free_m",
    "motion_error_half_decay_m",
    "T_out_min_s",
    "T_out_max_s",
    "assoc_iou_first_pass_thr",
    "dist_gate_m",
    "z_gate_m",
    "kf_max_gate_m",
)


def _quat_array(quaternion) -> np.ndarray:
    values = np.asarray(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
        dtype=float,
    )
    norm = float(np.linalg.norm(values))
    return values / norm if norm > 1e-12 else np.asarray([0.0, 0.0, 0.0, 1.0])


def _quat_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.asarray(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ],
        dtype=float,
    )


def _quat_rotation(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _yaw(quaternion: np.ndarray) -> float:
    x, y, z, w = quaternion
    return float(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _yaw_quaternion(yaw: float) -> Quaternion:
    output = Quaternion()
    output.z = math.sin(0.5 * yaw)
    output.w = math.cos(0.5 * yaw)
    return output


class PedRefTrackNode(Node):
    """Track prepared 3D bounding boxes; this deployment node never consumes GT."""

    def __init__(self) -> None:
        super().__init__("pedreftrack")
        defaults = PedRefTrackConfig(mode="no_gt")

        self.input_topic = str(self._declare("input_topic", "/pedestrian_detections_3d"))
        self.output_topic = str(self._declare("output_topic", "/tracked_pedestrians"))
        self.tracked_detections_topic = str(
            self._declare("tracked_detections_topic", "/pedreftrack/tracked_detections_3d")
        )
        self.tracking_frame = str(self._declare("tracking_frame", "")).strip()
        self.pedestrian_class_id = str(self._declare("pedestrian_class_id", "pedestrian"))
        self.publish_tracked_detections = bool(self._declare("publish_tracked_detections", True))
        self.radius_m = float(self._declare("radius_m", 0.25))
        self.velocity_ema_alpha = float(self._declare("velocity_ema_alpha", 0.5))

        config_values = {
            name: self._declare(f"tracker.{name}", getattr(defaults, name))
            for name in EXPOSED_TRACKER_PARAMETERS
        }
        self.tracker = PedRefTrack(
            cfg=PedRefTrackConfig(mode="no_gt", **config_values)
        )
        self.tracker.reset_sequence("ros2")
        self._velocity_state: Dict[int, Tuple[float, float, float, float, float]] = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            Detection3DArray,
            self.input_topic,
            self._on_detections,
            qos_profile_sensor_data,
        )
        self.track_publisher = self.create_publisher(
            TrackedPedestrianArray, self.output_topic, 10
        )
        self.box_publisher = (
            self.create_publisher(Detection3DArray, self.tracked_detections_topic, 10)
            if self.publish_tracked_detections
            else None
        )
        frame_text = self.tracking_frame or "incoming message frame"
        self.get_logger().info(
            f"PedRefTrack: {self.input_topic} -> {self.output_topic}; tracking frame={frame_text}"
        )

    def _declare(self, name: str, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _transform(self, message: Detection3DArray):
        source_frame = str(message.header.frame_id)
        target_frame = self.tracking_frame or source_frame
        if not source_frame:
            raise ValueError("Detection3DArray.header.frame_id must not be empty")
        if target_frame == source_frame:
            return np.eye(3), np.zeros(3), np.asarray([0.0, 0.0, 0.0, 1.0]), target_frame
        stamped = self.tf_buffer.lookup_transform(
            target_frame, source_frame, Time.from_msg(message.header.stamp)
        )
        rotation = _quat_array(stamped.transform.rotation)
        translation = np.asarray(
            [
                stamped.transform.translation.x,
                stamped.transform.translation.y,
                stamped.transform.translation.z,
            ],
            dtype=float,
        )
        return _quat_rotation(rotation), translation, rotation, target_frame

    def _score_and_class(self, detection: Detection3D) -> Tuple[float, str]:
        if not detection.results:
            return 1.0, ""
        result = max(detection.results, key=lambda item: float(item.hypothesis.score))
        return float(result.hypothesis.score), str(result.hypothesis.class_id)

    def _to_core(
        self,
        detections: Iterable[Detection3D],
        frame_id: str,
        rotation: np.ndarray,
        translation: np.ndarray,
        transform_quaternion: np.ndarray,
    ) -> list[Detection]:
        output = []
        for detection in detections:
            score, class_id = self._score_and_class(detection)
            if self.pedestrian_class_id and class_id and class_id != self.pedestrian_class_id:
                continue
            center = detection.bbox.center.position
            point = rotation @ np.asarray([center.x, center.y, center.z]) + translation
            orientation = _quat_multiply(
                transform_quaternion,
                _quat_array(detection.bbox.center.orientation),
            )
            output.append(
                Detection(
                    frame_id=frame_id,
                    track_id=-1,
                    box=Box3D(
                        cx=float(point[0]),
                        cy=float(point[1]),
                        cz=float(point[2]),
                        l=float(detection.bbox.size.x),
                        w=float(detection.bbox.size.y),
                        h=float(detection.bbox.size.z),
                        rot_z=_yaw(orientation),
                    ),
                    score=score,
                    label=class_id or self.pedestrian_class_id or "pedestrian",
                )
            )
        return output

    def _velocities(self, tracks, timestamp: float) -> Dict[int, Tuple[float, float]]:
        alpha = min(1.0, max(0.0, self.velocity_ema_alpha))
        current: Dict[int, Tuple[float, float]] = {}
        active = set()
        for detection in tracks:
            track_id = int(detection.track_id)
            active.add(track_id)
            old = self._velocity_state.get(track_id)
            vx = vy = 0.0
            if old is not None and timestamp > old[2]:
                dt = timestamp - old[2]
                raw_vx = (float(detection.box.cx) - old[0]) / dt
                raw_vy = (float(detection.box.cy) - old[1]) / dt
                vx = alpha * raw_vx + (1.0 - alpha) * old[3]
                vy = alpha * raw_vy + (1.0 - alpha) * old[4]
            self._velocity_state[track_id] = (
                float(detection.box.cx), float(detection.box.cy), timestamp, vx, vy
            )
            current[track_id] = (vx, vy)
        self._velocity_state = {
            track_id: state
            for track_id, state in self._velocity_state.items()
            if track_id in active
        }
        return current

    def _track_messages(self, header: Header, tracks, velocities) -> TrackedPedestrianArray:
        output = TrackedPedestrianArray()
        output.header = header
        for detection in tracks:
            message = TrackedPedestrian()
            message.track_id = int(detection.track_id)
            message.x = float(detection.box.cx)
            message.y = float(detection.box.cy)
            vx, vy = velocities.get(int(detection.track_id), (0.0, 0.0))
            message.vx = float(vx)
            message.vy = float(vy)
            message.radius = float(self.radius_m)
            output.pedestrians.append(message)
        return output

    def _box_messages(self, header: Header, tracks) -> Detection3DArray:
        output = Detection3DArray()
        output.header = header
        for track in tracks:
            detection = Detection3D()
            detection.header = header
            detection.id = str(track.track_id)
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = "pedestrian"
            result.hypothesis.score = float(track.score or 1.0)
            detection.results.append(result)
            detection.bbox.center.position = Point(
                x=float(track.box.cx), y=float(track.box.cy), z=float(track.box.cz)
            )
            detection.bbox.center.orientation = _yaw_quaternion(float(track.box.rot_z))
            detection.bbox.size = Vector3(
                x=float(track.box.l), y=float(track.box.w), z=float(track.box.h)
            )
            output.detections.append(detection)
        return output

    def _on_detections(self, message: Detection3DArray) -> None:
        timestamp = float(message.header.stamp.sec) + 1e-9 * float(message.header.stamp.nanosec)
        frame_id = str(message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec)
        try:
            rotation, translation, quaternion, output_frame = self._transform(message)
            detections = self._to_core(
                message.detections, frame_id, rotation, translation, quaternion
            )
            tracks = self.tracker.step(frame_id, detections, timestamp=timestamp).dets
        except (TransformException, ValueError) as error:
            self.get_logger().warn(f"Skipping detections: {error}")
            return
        except Exception as error:
            self.get_logger().error(f"PedRefTrack failed: {error}")
            return

        header = Header()
        header.stamp = message.header.stamp
        header.frame_id = output_frame
        velocities = self._velocities(tracks, timestamp)
        self.track_publisher.publish(self._track_messages(header, tracks, velocities))
        if self.box_publisher is not None:
            self.box_publisher.publish(self._box_messages(header, tracks))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PedRefTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
