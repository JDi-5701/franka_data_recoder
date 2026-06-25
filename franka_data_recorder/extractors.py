"""Per-message-type extractors: ROS msg -> numpy array (low-dim) or HxWx3 uint8 (image).

To support a NEW message type: write a function `f(msg) -> np.ndarray`, decorate it with
`@extractor("name")`, and reference "name" from the config. That is the only change needed
(the "incrementally add new msg type" requirement).
"""
import numpy as np

_REGISTRY = {}
_BRIDGE = None  # lazily created cv_bridge


def extractor(name):
    def deco(fn):
        _REGISTRY[name] = fn
        return fn
    return deco


def get_extractor(name):
    if name not in _REGISTRY:
        raise KeyError(f"unknown extractor '{name}'; available: {sorted(_REGISTRY)}")
    return _REGISTRY[name]


@extractor("pose_7d")
def pose_7d(msg):
    """geometry_msgs/PoseStamped -> [x,y,z, qx,qy,qz,qw]."""
    p, o = msg.pose.position, msg.pose.orientation
    return np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w], dtype=np.float32)


@extractor("wrench_6d")
def wrench_6d(msg):
    """geometry_msgs/WrenchStamped -> [fx,fy,fz, tx,ty,tz]."""
    f, t = msg.wrench.force, msg.wrench.torque
    return np.array([f.x, f.y, f.z, t.x, t.y, t.z], dtype=np.float32)


@extractor("twist_6d")
def twist_6d(msg):
    """geometry_msgs/Twist -> [vx,vy,vz, wx,wy,wz]."""
    l, a = msg.linear, msg.angular
    return np.array([l.x, l.y, l.z, a.x, a.y, a.z], dtype=np.float32)


@extractor("jointstate_pos")
def jointstate_pos(msg):
    """sensor_msgs/JointState -> position vector."""
    return np.asarray(msg.position, dtype=np.float32)


@extractor("jointstate_vel")
def jointstate_vel(msg):
    return np.asarray(msg.velocity, dtype=np.float32)


@extractor("jointstate_effort")
def jointstate_effort(msg):
    return np.asarray(msg.effort, dtype=np.float32)


@extractor("scalar")
def scalar(msg):
    """std_msgs/Float64 (or any .data scalar) -> [v]."""
    return np.array([float(msg.data)], dtype=np.float32)


@extractor("int_array")
def int_array(msg):
    """std_msgs/Int8MultiArray (or any .data sequence) -> float vector."""
    return np.asarray(msg.data, dtype=np.float32)


@extractor("image_rgb")
def image_rgb(msg):
    """sensor_msgs/Image (or CompressedImage) -> HxWx3 uint8 RGB. Needs cv_bridge."""
    global _BRIDGE
    if _BRIDGE is None:
        from cv_bridge import CvBridge
        _BRIDGE = CvBridge()
    if msg.__class__.__name__ == 'CompressedImage':
        img = _BRIDGE.compressed_imgmsg_to_cv2(msg, desired_encoding='rgb8')
    else:
        img = _BRIDGE.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    return np.ascontiguousarray(img, dtype=np.uint8)
