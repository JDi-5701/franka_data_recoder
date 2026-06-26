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
from datetime import datetime

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

from .extractors import get_extractor, default_names

try:
    # GoToPose lives in the interface-only package (no libfranka) so this imports on the
    # operator PC too. (Older layout had it under franka_cartesian_impedance_node.)
    from franka_cartesian_impedance_msgs.srv import GoToPose
except Exception:  # noqa - recorder still records without the reset feature
    try:
        from franka_cartesian_impedance_node.srv import GoToPose
    except Exception:  # noqa
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


def _source_repo_dir(cfg_path):
    """Directory that relative dataset roots anchor to: the package *source* repo (config
    lives in <repo>/config/, so repo = parent of the config dir).

    When the package is run from a colcon install space the config is a copy at
    <ws>/install/<pkg>/share/<pkg>/config/...; recordings would land in throwaway install
    space (wiped by `colcon build`). Redirect to the source checkout under <ws>/src so data
    stays in the real repo. With `--symlink-install` realpath already points at source and
    this is a no-op."""
    real = os.path.realpath(cfg_path)
    repo = os.path.dirname(os.path.dirname(real))          # <...>/config/x.yaml -> <...>
    parts = repo.split(os.sep)
    if 'install' in parts:
        ws = os.sep.join(parts[:parts.index('install')]) or os.sep
        src = os.path.join(ws, 'src')
        if os.path.isdir(src):
            for dirpath, _dirs, _files in os.walk(src):
                if os.path.basename(dirpath) == 'franka_data_recorder' \
                        and os.path.isdir(os.path.join(dirpath, 'config')):
                    return dirpath
    return repo


def resolve_data_root(root, cfg_path):
    """Relative dataset roots resolve to <source-repo>/<root> (git-ignored), so recordings
    stay inside the package folder and survive `colcon build`. Absolute paths/~ kept."""
    root = os.path.expanduser(str(root or 'data/dataset'))
    if os.path.isabs(root):
        return root
    return os.path.join(_source_repo_dir(cfg_path), root)


def _slug(name):
    """Filesystem-safe folder name from a free-text run/task name (spaces -> _)."""
    keep = [(c if (c.isalnum() or c in '-_') else '_') for c in str(name).strip()]
    return ''.join(keep).strip('_') or 'dataset'


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
        specs = spec['concat'] if 'concat' in spec else [spec]
        self.sources = [_Source(s) for s in specs]
        if 'concat' in spec:
            self.shape = [int(sum(s.get('dim', 0) for s in specs))]
        else:
            self.shape = list(spec['shape']) if 'shape' in spec else [int(spec.get('dim', 0))]
        # per-dimension names for low-dim features (None for images: writer fills h/w/c).
        # Each source may set explicit `names: [...]`, else they are auto-derived from the
        # extractor (canonical dims, optionally prefixed by the source's `name` label).
        if self.is_image:
            self.names = None
        else:
            self.names = []
            for s in specs:
                dim = int(s.get('dim', 0))
                nm = s.get('names') or default_names(s['extractor'], dim, s.get('name'))
                self.names.extend(nm)


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

        # lazy writer (created on first start so a missing lerobot fails loudly only then).
        # Each process run gets its own timestamped dataset dir so a restart never collides
        # with an existing one (cross-restart append is a separate TODO). Episodes recorded
        # within one run still accumulate into this single dataset.
        #
        # `dataset_name` param overrides the folder/repo_id base (e.g. a task name like
        # "pick_cube") -> data/<dataset_name>_<timestamp>; otherwise the config root is used.
        self._ds_cfg = dict(ds)
        base_root = resolve_data_root(ds.get('root'), cfg_path)
        run_name = (self.declare_parameter('dataset_name', '').value or '').strip()
        if run_name:
            run_name = _slug(run_name)
            base_root = os.path.join(os.path.dirname(base_root), run_name)
            self._ds_cfg['repo_id'] = run_name
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._ds_cfg['root'] = f'{base_root}_{stamp}'
        self.get_logger().info(f"dataset root: {self._ds_cfg['root']}")
        self._writer = None
        self._recording = False
        self._n_frames = 0
        self._skipped = 0

        self.create_service(Trigger, '~/start_recording', self._on_start)
        self.create_service(Trigger, '~/stop_recording', self._on_stop)
        self.create_service(Trigger, '~/discard_episode', self._on_discard)

        # Homing services (go_home / go_pose) are thin forwarders to the controller. The
        # controller OWNS the lock: it ignores /target_pose while homing and re-takes control
        # via its own GUARD afterwards (teleop respects ~/control_state). So the recorder just
        # calls the controller service -- no /reset_teleop, no teleop coordination here.

        # --- go_home button: forwards to the controller's ~/go_home (std_srvs/Trigger), which
        #     drives to the controller's OWN fixed home_pose. By design this takes NO pose param
        #     (home lives in the controller). Trigger -> callable even where GoToPose is absent.
        ghcfg = cfg.get('go_home', {})
        self._gohome_trigger_cli = self.create_client(
            Trigger, ghcfg.get('service', '/cartesian_impedance_node/go_home'))
        self.create_service(Trigger, '~/go_home', self._on_go_home)

        # --- go_pose button: drive to a CONFIGURED pose (editable in recorder.yaml `go_pose:`)
        #     via the controller's ~/go_pose (GoToPose), separate from the reset pose.
        gpcfg = cfg.get('go_pose', {})
        self._go_pose = gpcfg.get('pose')
        self._go_pose_vel = float(gpcfg.get('max_velocity', 0.0))
        self._gopose_cli = None
        if GoToPose is not None and self._go_pose is not None:
            self._gopose_cli = self.create_client(
                GoToPose, gpcfg.get('service', '/cartesian_impedance_node/go_pose'))
            self.create_service(Trigger, '~/go_pose', self._on_go_pose)

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
                                      'shape': f.shape, 'names': f.names}
                             for f in self.features}
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

    # ---- homing services (thin forwarders; the controller owns the lock + hand-over) ------
    def _discard_if_recording(self):
        """Homing mid-recording would corrupt the demo -> drop the in-progress episode."""
        if not self._recording:
            return False
        self._recording = False
        if self._writer is not None:
            try:
                self._writer.discard_episode()
            except Exception as e:  # noqa
                self.get_logger().warn(f'discard failed: {e}')
        self.get_logger().warn(
            f'homing during recording -> discarded in-progress episode ({self._n_frames} frames)')
        return True

    def _on_go_home(self, req, resp):
        """Forward to the controller's ~/go_home (Trigger) -> its fixed home_pose."""
        discarded = self._discard_if_recording()
        if not self._gohome_trigger_cli.wait_for_service(timeout_sec=2.0):
            resp.success, resp.message = False, 'go_home service unavailable'
            return resp
        self._gohome_trigger_cli.call_async(
            Trigger.Request()).add_done_callback(lambda f: self._log_homing('go_home', f))
        resp.success, resp.message = True, \
            ('discarded in-progress episode; ' if discarded else '') + 'go_home started'
        self.get_logger().info(resp.message)
        return resp

    def _on_go_pose(self, req, resp):
        """Drive to the configured `go_pose.pose` (recorder.yaml) via the controller go_pose."""
        if self._gopose_cli is None:
            resp.success, resp.message = (
                False, 'go_pose unavailable (no GoToPose msg or no pose configured)')
            return resp
        discarded = self._discard_if_recording()
        if not self._gopose_cli.wait_for_service(timeout_sec=2.0):
            resp.success, resp.message = False, 'go_pose service unavailable'
            return resp
        goal = GoToPose.Request()
        p, o = self._go_pose['position'], self._go_pose['orientation']
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = map(float, p)
        (goal.pose.orientation.x, goal.pose.orientation.y,
         goal.pose.orientation.z, goal.pose.orientation.w) = map(float, o)
        goal.max_velocity = self._go_pose_vel
        self._gopose_cli.call_async(goal).add_done_callback(lambda f: self._log_homing('go_pose', f))
        resp.success, resp.message = True, \
            ('discarded in-progress episode; ' if discarded else '') + 'go_pose started'
        self.get_logger().info(resp.message)
        return resp

    def _log_homing(self, what, future):
        try:
            res = future.result()
        except Exception as e:  # noqa
            self.get_logger().error(f'{what} call failed: {e}')
            return
        if res is not None and not res.success:
            self.get_logger().warn(f'{what}: {res.message}')
        else:
            self.get_logger().info(f'{what} done (controller re-takes control via GUARD)')


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
