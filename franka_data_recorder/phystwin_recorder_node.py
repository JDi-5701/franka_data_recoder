"""PhysTwin recording entry point -- subclasses RecorderNode; ZERO changes to existing code.

Two ways to tell it which cameras to record:
  * AUTO (default): scan the ROS graph for every camera that publishes the required triplet
    (`<ns>/color/image_raw` + `<ns>/aligned_depth_to_color/image_raw` + `<ns>/color/camera_info`)
    and record ALL of them. Plug in N cameras -> record N cameras. No per-camera config.
    Camera index (0..N-1) = namespaces sorted alphabetically (stable across runs).
  * STATIC: if the config lists `observation.images.*` features, those are used verbatim and
    auto-discovery is skipped. Set param `auto_discover:=false` to force static.

Run it:
    ros2 run franka_data_recorder phystwin_recorder \
        --ros-args -p config_file:=<...>/config/recorder_phystwin_auto.yaml \
                   -p dataset_name:=microwave_door_ep01 \
                   -p calibrate_pkl:=<...>/calibrate.pkl \
                   -p gravity_json:=<...>/gravity.json
(or use launch/phystwin_recorder.launch.py / phystwin_all.launch.py).

Everything else is inherited from RecorderNode: the fps-tick synchroniser and its
"all-configured-topics-present -> emit ONE frame with a monotonic index" logic, which is
exactly PhysTwin's requirement of synchronised, shared, contiguous frame indices across the
cameras. Only the writer is swapped (PhysTwinWriter) and camera_info is read for intrinsics.
"""
import numpy as np
import rclpy
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)
from sensor_msgs.msg import CameraInfo

from .recorder_node import RecorderNode, _Feature, _resolve_type
from . import depth_extractor  # noqa: F401  registers `depth_raw` into the extractor registry

_COLOR_SUFFIX = "/color/image_raw"
_DEPTH_SUFFIX = "/aligned_depth_to_color/image_raw"
_INFO_SUFFIX = "/color/camera_info"


class PhysTwinRecorderNode(RecorderNode):
    def __init__(self):
        super().__init__()                       # builds subs/timer/services from the config

        auto = self.declare_parameter("auto_discover", True).value
        self._discover_sec = float(self.declare_parameter("discover_sec", 4.0).value)
        has_static_imgs = any(f.name.startswith("observation.images.") for f in self.features)
        if auto and not has_static_imgs:
            cams = self._discover_cameras()
            if cams:
                self.get_logger().info(
                    f"[phystwin] auto-discovered {len(cams)} camera(s): {cams}")
                self._add_dynamic_features(cams)
            else:
                self.get_logger().error(
                    "[phystwin] auto_discover found NO cameras (need color + "
                    "aligned_depth_to_color + camera_info). Nothing to record.")
        elif has_static_imgs:
            self.get_logger().info("[phystwin] using cameras from config (auto_discover off).")

        self._cam_meta = {}                       # cam_idx -> {"K", "W", "H", "serial"}
        self._num_cam = sum(1 for f in self.features
                            if f.name.startswith("observation.images."))
        self._subscribe_camera_info()
        # one-time calibration artifacts embedded into every case folder (cameras are static)
        self._calib_pkl = self.declare_parameter("calibrate_pkl", "").value or None
        self._gravity_json = self.declare_parameter("gravity_json", "").value or None
        self.get_logger().info(
            f"PhysTwin recorder ready (writer=phystwin, {self._num_cam} cameras).")

    # ---- auto-discovery ------------------------------------------------------
    def _discover_cameras(self):
        """Spin briefly, then return camera namespaces that publish the full triplet, sorted."""
        import time
        deadline = time.time() + self._discover_sec
        self.get_logger().info(
            f"[phystwin] discovering cameras for {self._discover_sec:.0f}s ...")
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        topics = dict(self.get_topic_names_and_types())
        cams = []
        for topic, types in topics.items():
            if not topic.endswith(_COLOR_SUFFIX):
                continue
            if "sensor_msgs/msg/Image" not in types:
                continue
            ns = topic[: -len(_COLOR_SUFFIX)]
            if (ns + _DEPTH_SUFFIX) in topics and (ns + _INFO_SUFFIX) in topics:
                cams.append(ns)
            else:
                self.get_logger().warn(
                    f"[phystwin] skip {ns}: missing aligned_depth_to_color or camera_info "
                    f"(enable align_depth.enable:=true)")
        return sorted(cams)

    def _add_dynamic_features(self, cams):
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        existing = {src.topic for f in self.features for src in f.sources}
        for i, ns in enumerate(cams):
            specs = [
                (f"observation.images.{i}",
                 {"topic": ns + _COLOR_SUFFIX, "type": "Image",
                  "extractor": "image_rgb", "shape": [0, 0, 3]}),
                (f"observation.depth.{i}",
                 {"topic": ns + _DEPTH_SUFFIX, "type": "Image",
                  "extractor": "depth_raw", "shape": [0, 0]}),
            ]
            for name, spec in specs:
                feat = _Feature(name, spec)
                self.features.append(feat)
                for src in feat.sources:
                    if src.topic in existing:
                        continue
                    existing.add(src.topic)
                    self.create_subscription(
                        _resolve_type(src.type_str), src.topic,
                        lambda m, t=src.topic: self._latest.__setitem__(t, m), qos)
                    self.get_logger().info(f"[phystwin] subscribed {src.topic}")

    # ---- intrinsics from camera_info -----------------------------------------
    def _subscribe_camera_info(self):
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)
        for feat in self.features:
            if not feat.name.startswith("observation.images."):
                continue
            idx = int(feat.name.rsplit(".", 1)[1])
            color_topic = feat.sources[0].topic                       # .../color/image_raw
            info_topic = color_topic.rsplit("/", 1)[0] + "/camera_info"
            self.create_subscription(
                CameraInfo, info_topic,
                lambda m, i=idx: self._on_camera_info(i, m), qos)
            self.get_logger().info(f"[phystwin] cam {idx} intrinsics <- {info_topic}")

    def _on_camera_info(self, idx, msg):
        self._cam_meta[idx] = {
            "K": np.array(msg.k, dtype=float).reshape(3, 3).tolist(),
            "W": int(msg.width),
            "H": int(msg.height),
            "serial": msg.header.frame_id or f"cam_{idx}",
        }

    def _meta_provider(self):
        """Assemble metadata.json fields in camera-index order (0..N-1)."""
        intrinsics, serials, WH = [], [], None
        for i in range(self._num_cam):
            m = self._cam_meta.get(i)
            if m is None:
                self.get_logger().warn(
                    f"[phystwin] no camera_info received for cam {i}; intrinsics will be zero")
                intrinsics.append([[0, 0, 0], [0, 0, 0], [0, 0, 1]])
                serials.append(f"cam_{i}")
                continue
            intrinsics.append(m["K"])
            serials.append(m["serial"])
            WH = [m["W"], m["H"]]
        return {"intrinsics": intrinsics, "serial_numbers": serials, "WH": WH}

    # ---- swap in the PhysTwin writer -----------------------------------------
    def _ensure_writer(self):
        if self._writer is not None:
            return
        from .phystwin_writer import PhysTwinWriter
        self._writer = PhysTwinWriter(
            case_root=self._ds_cfg["root"], num_cam=self._num_cam,
            meta_provider=self._meta_provider, fps=self.fps,
            calibrate_src=self._calib_pkl, gravity_src=self._gravity_json,
            logger=self.get_logger())


def main():
    rclpy.init()
    node = PhysTwinRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
