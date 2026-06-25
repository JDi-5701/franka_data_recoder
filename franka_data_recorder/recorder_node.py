"""Config-driven LeRobot recorder.

Subscribes to the topics named in a YAML config, samples them at a fixed `fps`, and writes
one LeRobot episode per recording (start/stop via std_srvs/Trigger services so a CLI, a
SpaceMouse button, or a web GUI can all drive it).

Services (under this node's namespace):
  ~/start_recording   (std_srvs/Trigger)  begin a new episode
  ~/stop_recording    (std_srvs/Trigger)  finish + write the episode
  ~/discard_episode   (std_srvs/Trigger)  drop the in-progress episode

Params:
  config_file (str)  path to the YAML config (defaults to the installed config/recorder.yaml)
  task        (str)  language instruction for the next episode (overrides config default)
"""
import importlib
import os

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory

from .extractors import get_extractor

try:
    from franka_cartesian_impedance_node.srv import GoToPose
except Exception:  # noqa - recorder still records without the reset feature
    GoToPose = None

# short name -> (module, class). Also accepts the full "pkg_msgs/Type" form.
_TYPE_MAP = {
    'PoseStamped': ('geometry_msgs.msg', 'PoseStamped'),
    'WrenchStamped': ('geometry_msgs.msg', 'WrenchStamped'),
    'Twist': ('geometry_msgs.msg', 'Twist'),
    'TwistStamped': ('geometry_msgs.msg', 'TwistStamped'),
    'JointState': ('sensor_msgs.msg', 'JointState'),
    'Image': ('sensor_msgs.msg', 'Image'),
    'CompressedImage': ('sensor_msgs.msg', 'CompressedImage'),
    'Float64': ('std_msgs.msg', 'Float64'),
    'Int8MultiArray': ('std_msgs.msg', 'Int8MultiArray'),
}


def _resolve_type(type_str):
    if '/' in type_str:                       # "geometry_msgs/PoseStamped"
        pkg, cls = type_str.split('/')[-2:]
        mod = importlib.import_module(pkg + '.msg' if not pkg.endswith('.msg') else pkg)
        return getattr(mod, cls)
    mod_name, cls = _TYPE_MAP[type_str]
    return getattr(importlib.import_module(mod_name), cls)


class _Source:
    __slots__ = ('topic', 'type_str', 'extractor')

    def __init__(self, spec):
        self.topic = spec['topic']
        self.type_str = spec['type']
        self.extractor = get_extractor(spec['extractor'])


class _Feature:
    """One LeRobot feature, built from one or more concatenated sources."""

    def __init__(self, name, spec):
        self.name = name
        self.is_image = name.startswith('observation.images.')
        if 'concat' in spec:
            self.sources = [_Source(s) for s in spec['concat']]
            self.shape = [int(sum(s.get('dim', 0) for s in spec['concat']))]
        else:
            self.sources = [_Source(spec)]
            self.shape = list(spec['shape']) if 'shape' in spec else [int(spec.get('dim', 0))]


class RecorderNode(Node):
    def __init__(self):
        super().__init__('franka_data_recorder')
        default_cfg = os.path.join(
            get_package_share_directory('franka_data_recorder'), 'config', 'recorder.yaml')
        cfg_path = self.declare_parameter('config_file', default_cfg).value
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
        self.get_logger().info(f'loaded config: {cfg_path}')

        ds = cfg['dataset']
        self.fps = float(ds.get('fps', 30))
        self.task_default = self.declare_parameter(
            'task', cfg.get('task', {}).get('default', 'franka teleop demo')).value

        self.features = [_Feature(name, spec) for name, spec in cfg['features'].items()]

        # one subscription per unique topic; cache the latest message
        self._latest = {}
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        seen = {}
        for feat in self.features:
            for src in feat.sources:
                if src.topic in seen:
                    continue
                seen[src.topic] = True
                msg_cls = _resolve_type(src.type_str)
                self.create_subscription(
                    msg_cls, src.topic,
                    lambda m, t=src.topic: self._latest.__setitem__(t, m), qos)
                self.get_logger().info(f'subscribed {src.topic} ({src.type_str})')

        # lazy writer (created on first start so a missing lerobot fails loudly only then)
        self._ds_cfg = ds
        self._writer = None
        self._recording = False
        self._n_frames = 0
        self._skipped = 0

        self.create_service(Trigger, '~/start_recording', self._on_start)
        self.create_service(Trigger, '~/stop_recording', self._on_stop)
        self.create_service(Trigger, '~/discard_episode', self._on_discard)

        # reset / go-home: drive the robot to a configured pose via the controller's go_home
        # service (no topic conflict), then poke /reset_teleop so teleop re-latches.
        rcfg = cfg.get('reset', {})
        self._reset_pose = rcfg.get('pose')
        self._reset_vel = float(rcfg.get('max_velocity', 0.0))
        self._reset_teleop_pub = self.create_publisher(
            Bool, rcfg.get('reset_teleop_topic', '/reset_teleop'), 10)
        self._resume_timer = None
        self._gohome_cli = None
        if GoToPose is not None and self._reset_pose is not None:
            self._gohome_cli = self.create_client(
                GoToPose, rcfg.get('go_home_service', '/cartesian_impedance_node/go_home'))
            self.create_service(Trigger, '~/reset', self._on_reset)

        self.create_timer(1.0 / self.fps, self._tick)
        self.get_logger().info(f'recorder ready @ {self.fps} Hz. call ~/start_recording to begin.')

    # ---- frame sampling -------------------------------------------------
    def _build_frame(self):
        frame = {}
        for feat in self.features:
            parts = []
            for src in feat.sources:
                msg = self._latest.get(src.topic)
                if msg is None:
                    return None                      # not all sources ready yet
                parts.append(src.extractor(msg))
            frame[feat.name] = parts[0] if feat.is_image else np.concatenate(parts)
        return frame

    def _tick(self):
        if not self._recording:
            return
        try:
            frame = self._build_frame()
            if frame is None:
                self._skipped += 1
                if self._skipped % int(self.fps) == 1:
                    self.get_logger().warn('waiting for all configured topics to publish...')
                return
            frame['task'] = self._task
            self._writer.add_frame(frame)
            self._n_frames += 1
        except Exception as e:  # noqa - never let one bad frame kill the node
            self.get_logger().error(f'record frame failed: {e}', throttle_duration_sec=2.0)

    # ---- services -------------------------------------------------------
    def _ensure_writer(self):
        if self._writer is None:
            from .lerobot_writer import LeRobotWriter
            features_meta = {f.name: {'dtype': 'video' if f.is_image else 'float32',
                                      'shape': f.shape} for f in self.features}
            self._writer = LeRobotWriter(
                repo_id=self._ds_cfg['repo_id'], root=self._ds_cfg['root'],
                fps=self.fps, robot_type=self._ds_cfg.get('robot_type', 'franka_fr3'),
                features=features_meta, logger=self.get_logger())

    def _on_start(self, req, resp):
        if self._recording:
            resp.success, resp.message = False, 'already recording'
            return resp
        try:
            self._ensure_writer()
            self._task = self.get_parameter('task').value or self.task_default
            self._writer.start_episode(self._task)
        except Exception as e:  # noqa
            resp.success, resp.message = False, f'start failed: {e}'
            self.get_logger().error(resp.message)
            return resp
        self._recording, self._n_frames, self._skipped = True, 0, 0
        resp.success, resp.message = True, f'recording started (task="{self._task}")'
        self.get_logger().info(resp.message)
        return resp

    def _on_stop(self, req, resp):
        if not self._recording:
            resp.success, resp.message = False, 'not recording'
            return resp
        self._recording = False
        try:
            self._writer.save_episode()
        except Exception as e:  # noqa
            resp.success, resp.message = False, f'save failed: {e}'
            self.get_logger().error(resp.message)
            return resp
        resp.success, resp.message = True, f'episode saved ({self._n_frames} frames)'
        self.get_logger().info(resp.message)
        return resp

    def _on_discard(self, req, resp):
        self._recording = False
        if self._writer is not None:
            self._writer.discard_episode()
        resp.success, resp.message = True, f'episode discarded ({self._n_frames} frames)'
        self.get_logger().info(resp.message)
        return resp

    # ---- reset / go-home (non-blocking: homes the robot, then pokes /reset_teleop) ------
    def _on_reset(self, req, resp):
        if self._recording:
            resp.success, resp.message = False, 'stop recording before reset'
            return resp
        if not self._gohome_cli.wait_for_service(timeout_sec=2.0):
            resp.success, resp.message = False, 'go_home service unavailable'
            return resp
        goal = GoToPose.Request()
        p, o = self._reset_pose['position'], self._reset_pose['orientation']
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = map(float, p)
        (goal.pose.orientation.x, goal.pose.orientation.y,
         goal.pose.orientation.z, goal.pose.orientation.w) = map(float, o)
        goal.max_velocity = self._reset_vel
        # 1) tell teleop to STOP publishing so it does not fight the homing / overwrite the
        #    controller target the instant homing finishes.
        self._reset_teleop_pub.publish(Bool(data=True))
        self._gohome_cli.call_async(goal).add_done_callback(self._after_home)
        resp.success, resp.message = True, 'reset (homing) started'
        self.get_logger().info(resp.message)
        return resp

    def _after_home(self, future):
        try:
            res = future.result()
        except Exception as e:  # noqa
            self.get_logger().error(f'go_home call failed: {e}')
            res = None
        if res is not None and not res.success:
            self.get_logger().warn(f'go_home: {res.message}')
        # 2) robot is now at home (or wherever it stopped). Re-latch teleop's equilibrium to
        #    the current pose (True), then resume publishing shortly after (False). The small
        #    delay guarantees teleop processes the re-latch (with a fresh current_pose) first.
        self._reset_teleop_pub.publish(Bool(data=True))
        if self._resume_timer is not None:
            self._resume_timer.cancel()
        self._resume_timer = self.create_timer(0.3, self._resume_teleop)

    def _resume_teleop(self):
        self._resume_timer.cancel()
        self._resume_timer = None
        self._reset_teleop_pub.publish(Bool(data=False))  # teleop resumes from the new pose
        self.get_logger().info('reset done -> teleop re-latched + resumed')


def main():
    rclpy.init()
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
