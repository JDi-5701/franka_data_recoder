"""Depth extractor for the PhysTwin recording backend.

ADDITIVE: does NOT modify extractors.py. Registers a `depth_raw` extractor into the SAME
registry (via the shared `@extractor` decorator), so a config can say `extractor: depth_raw`.
Importing this module once (the phystwin recorder node does) runs the registration.

NO cv_bridge -- decodes the depth Image straight from the message buffer with NumPy, matching
this repo's ros_ml/system-ROS workaround (see the `image_rgb` rewrite in extractors.py). PhysTwin
needs LOSSLESS depth in uint16 millimetres (its pipeline does `depth / 1000.0` -> metres).
"""
import numpy as np

from .extractors import extractor


@extractor("depth_raw")
def depth_raw(msg):
    """sensor_msgs/Image (depth) -> HxW uint16 millimetres (lossless). No cv_bridge.

    - RealSense `aligned_depth_to_color` is 16UC1 (== mono16) in millimetres -> passthrough.
    - 32FC1 depth is in metres -> converted to uint16 mm.
    `msg.step` (row stride in bytes) is respected in case of row padding. Assumes native
    (little-endian) byte order, which RealSense uses; msg.is_bigendian is expected to be 0.
    """
    if msg.__class__.__name__ == "CompressedImage":
        raise ValueError("depth_raw needs a raw sensor_msgs/Image "
                         "(aligned_depth_to_color/image_raw), not a compressed topic")
    enc = msg.encoding.lower()
    h, w = msg.height, msg.width

    if enc in ("16uc1", "mono16"):
        raw = np.frombuffer(msg.data, dtype=np.uint16)
        row = msg.step // 2                                   # bytes -> uint16 per row
        arr = raw.reshape(h, row)[:, :w]
        return np.ascontiguousarray(arr, dtype=np.uint16)     # already millimetres

    if enc == "32fc1":
        raw = np.frombuffer(msg.data, dtype=np.float32)
        row = msg.step // 4                                   # bytes -> float32 per row
        arr = raw.reshape(h, row)[:, :w]
        return np.ascontiguousarray(np.round(arr * 1000.0), dtype=np.uint16)  # metres -> mm

    raise ValueError(f"unsupported depth encoding: {msg.encoding!r} "
                     f"(expected 16UC1/mono16 or 32FC1)")
