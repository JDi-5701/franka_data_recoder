"""One-time multi-camera extrinsic calibration for PhysTwin (ROS2 + Charuco).

Grabs one color frame + camera_info per camera, detects a Charuco board, estimates each
camera's pose w.r.t. the board, and writes (into --out-dir):
    calibrate.pkl   list of camera->world 4x4 matrices, in camera-index order (cam 0..N-1)
    gravity.json    gravity vector expressed in the world frame

WORLD FRAME = the Charuco board frame (all cameras see the same board -> consistent relative
extrinsics). Point the phystwin recorder at this calibrate.pkl (cameras are static, so you
calibrate ONCE and reuse it for every case).

GRAVITY -- three ways (pick with --gravity):
  * board   (default): assumes the board lies FLAT on floor/table, so its +Z is vertical ->
             gravity_world = [0, 0, -9.81]. Simplest; no extra input.
  * base:    world is taken to coincide with a level robot base (base +Z up) -> also
             [0, 0, -9.81]. Use this if you calibrated cameras against the robot base and the
             robot is mounted level. (Same numbers as `board`; kept explicit for clarity.)
  * x,y,z:   pass an explicit vector, e.g. --gravity "-0.1,0,-9.79", if you computed the
             down-direction yourself (e.g. rotated [0,0,-9.81] from the robot base into the
             board frame using your known camera<->base extrinsic).

Board defaults MATCH PhysTwin's camera_system.calibrate(): DICT_4X4_50, (4,5) squares,
squareLength=0.05 m, markerLength=0.037 m. Override with flags if your board differs.

NO cv_bridge: color frames are decoded with this repo's `image_rgb` extractor (NumPy). cv2 is
used only for aruco (same as gui_node uses cv2). cv2.aruco changed API around OpenCV 4.7; this
mirrors PhysTwin's (older) calls -- if your OpenCV is newer and a call errors, adjust the three
aruco lines flagged below.
"""
import argparse
import json
import os
import pickle

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo

from .extractors import get_extractor
from . import depth_extractor  # noqa: F401  (not used here, but keeps registry imports uniform)

_image_rgb = get_extractor("image_rgb")


class _Grabber(Node):
    """Grabs one (color, K) pair per camera namespace."""

    def __init__(self, namespaces):
        super().__init__("phystwin_calibrate")
        self.color = {}
        self.K = {}
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)
        for i, ns in enumerate(namespaces):
            self.create_subscription(Image, f"{ns}/color/image_raw",
                                     lambda m, k=i: self._on_color(k, m), qos)
            self.create_subscription(CameraInfo, f"{ns}/color/camera_info",
                                     lambda m, k=i: self._on_info(k, m), qos)
            self.get_logger().info(f"cam {i}: {ns}/color/image_raw (+ camera_info)")

    def _on_color(self, i, msg):
        rgb = _image_rgb(msg)                       # HxWx3 uint8 RGB (no cv_bridge)
        self.color[i] = np.ascontiguousarray(rgb[:, :, ::-1])   # -> BGR for cv2.aruco

    def _on_info(self, i, msg):
        self.K[i] = np.array(msg.k, dtype=float).reshape(3, 3)

    def ready(self, n):
        return len(self.color) >= n and len(self.K) >= n


def estimate_c2w(color_bgr, K, board, dictionary):
    """Charuco pose of one camera w.r.t. the board -> 4x4 camera->world. None on failure."""
    # --- aruco calls (adjust for OpenCV >=4.7 if needed) ---------------------
    corners, ids, _ = cv2.aruco.detectMarkers(color_bgr, dictionary)
    if ids is None or len(ids) == 0:
        return None, 0
    _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
        markerCorners=corners, markerIds=ids, image=color_bgr, board=board, cameraMatrix=K)
    if ch_corners is None or len(ch_corners) < 6:
        return None, 0 if ch_corners is None else len(ch_corners)
    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        ch_corners, ch_ids, board, K, None, None, None)
    if not ok:
        return None, len(ch_corners)
    # reprojection error
    reproj, _ = cv2.projectPoints(board.getChessboardCorners()[ch_ids, :], rvec, tvec, K, None)
    err = float(np.sqrt(((reproj.reshape(-1, 2) - ch_corners.reshape(-1, 2)) ** 2)
                        .sum(axis=1)).mean())
    w2c = np.eye(4)
    w2c[:3, :3] = cv2.Rodrigues(rvec)[0]
    w2c[:3, 3] = tvec[:, 0]
    return np.linalg.inv(w2c), err, len(ch_corners)


def parse_gravity(spec):
    if spec in ("board", "base"):
        return [0.0, 0.0, -9.81], spec
    try:
        v = [float(x) for x in spec.split(",")]
        assert len(v) == 3
        return v, "manual"
    except Exception:
        raise SystemExit(f"--gravity must be 'board', 'base', or 'x,y,z'; got: {spec!r}")


def discover_cameras(node, secs=4.0):
    """Same discovery + sort as the recorder -> identical camera-index order."""
    import time
    end = time.time() + secs
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    topics = dict(node.get_topic_names_and_types())
    cams = []
    for t, types in topics.items():
        if t.endswith("/color/image_raw") and "sensor_msgs/msg/Image" in types:
            ns = t[: -len("/color/image_raw")]
            if (ns + "/aligned_depth_to_color/image_raw") in topics \
                    and (ns + "/color/camera_info") in topics:
                cams.append(ns)
    return sorted(cams)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cameras", nargs="+", default=None,
                    help="camera namespaces in index order (e.g. /d405/camera /d435/camera "
                         "/d455/camera). OMIT to auto-discover (same order as the recorder).")
    ap.add_argument("--discover-sec", type=float, default=4.0)
    ap.add_argument("--out-dir", default=".", help="where to write calibrate.pkl / gravity.json")
    ap.add_argument("--gravity", default="board", help="'board' | 'base' | 'x,y,z'")
    ap.add_argument("--squares", nargs=2, type=int, default=[4, 5], help="board (cols rows)")
    ap.add_argument("--square-len", type=float, default=0.05)
    ap.add_argument("--marker-len", type=float, default=0.037)
    ap.add_argument("--max-reproj", type=float, default=0.5, help="reject a cam above this px err")
    args = ap.parse_args()

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard(tuple(args.squares), squareLength=args.square_len,
                                   markerLength=args.marker_len, dictionary=dictionary)
    gravity_vec, gravity_src = parse_gravity(args.gravity)

    rclpy.init()
    cameras = args.cameras
    if not cameras:
        disc = rclpy.create_node("phystwin_calib_discovery")
        disc.get_logger().info(f"auto-discovering cameras for {args.discover_sec:.0f}s ...")
        cameras = discover_cameras(disc, args.discover_sec)
        disc.destroy_node()
        if not cameras:
            rclpy.shutdown()
            raise SystemExit("no cameras discovered (need color + aligned_depth_to_color + "
                             "camera_info). Start the cameras first, or pass --cameras.")
        print(f"[calibrate] discovered cameras: {cameras}")
    n = len(cameras)

    node = _Grabber(cameras)
    node.get_logger().info("waiting for one frame + camera_info from every camera...")
    while rclpy.ok() and not node.ready(n):
        rclpy.spin_once(node, timeout_sec=0.5)

    c2ws = []
    ok_all = True
    for i in range(n):
        res = estimate_c2w(node.color[i], node.K[i], board, dictionary)
        if res[0] is None:
            node.get_logger().error(f"cam {i}: board NOT detected (corners={res[-1]}). "
                                    f"Ensure the board is fully visible from this camera.")
            ok_all = False
            c2ws.append(None)
            continue
        c2w, err, ncorners = res
        node.get_logger().info(f"cam {i}: reproj err={err:.3f}px, corners={ncorners}")
        if err > args.max_reproj:
            node.get_logger().error(f"cam {i}: reproj err {err:.3f} > {args.max_reproj}; reject.")
            ok_all = False
        c2ws.append(c2w)

    os.makedirs(args.out_dir, exist_ok=True)
    if not ok_all:
        node.get_logger().error("calibration incomplete -- NOT writing calibrate.pkl. "
                                "Fix camera views / board placement and retry.")
    else:
        with open(os.path.join(args.out_dir, "calibrate.pkl"), "wb") as f:
            pickle.dump(c2ws, f)
        node.get_logger().info(f"wrote {os.path.join(args.out_dir, 'calibrate.pkl')} "
                               f"({n} cameras, world = board frame)")

    up = [-g for g in gravity_vec]
    with open(os.path.join(args.out_dir, "gravity.json"), "w") as f:
        json.dump({"gravity_world": gravity_vec, "up_world": up,
                   "source": gravity_src, "world_frame": "charuco_board"}, f, indent=2)
    node.get_logger().info(f"wrote gravity.json (gravity_world={gravity_vec}, src={gravity_src})")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
