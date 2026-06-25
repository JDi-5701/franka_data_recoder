"""LeRobot dataset writer (isolated so the rest of the recorder is lerobot-version agnostic).

This is the ONLY file that talks to the `lerobot` library. If your installed lerobot has a
different API (import path / add_frame / save_episode signature), adjust it here only.
Tested against the lerobot 0.x `LeRobotDataset` API.
"""
import os

# Record locally without any HuggingFace Hub access (else LeRobotDataset.create tries to
# fetch the repo refs and 401s for a local-only dataset). Set BEFORE importing lerobot.
os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")

import numpy as np


def _import_lerobot_dataset():
    # the import path moved between lerobot versions; try both
    for path in ('lerobot.datasets.lerobot_dataset',
                 'lerobot.common.datasets.lerobot_dataset'):
        try:
            mod = __import__(path, fromlist=['LeRobotDataset'])
            return mod.LeRobotDataset
        except Exception:  # noqa
            continue
    raise ImportError(
        "could not import LeRobotDataset. Install lerobot in the recorder's python env:\n"
        "  pip install lerobot   (or: pip install 'lerobot[pi0]')")


class LeRobotWriter:
    def __init__(self, repo_id, root, fps, robot_type, features, logger=None):
        self._log = logger
        LeRobotDataset = _import_lerobot_dataset()

        # build the LeRobot feature schema from our config-derived metadata.
        # NOTE: shape MUST be a tuple, not a list. lerobot's per-frame validator compares
        # `value.shape` (always a numpy tuple, e.g. (7,)) against `feature["shape"]` with a
        # plain `!=`; a list [7] never equals the tuple (7,), so every frame would be
        # rejected ("does not have the expected shape").
        ds_features = {}
        for name, meta in features.items():
            if meta['dtype'] == 'video':
                ds_features[name] = {'dtype': 'video', 'shape': tuple(meta['shape']),
                                     'names': ['height', 'width', 'channels']}
            else:
                ds_features[name] = {'dtype': 'float32', 'shape': tuple(meta['shape']),
                                     'names': None}

        # Always CREATE a fresh local dataset. (Re-opening an existing one via
        # LeRobotDataset(repo_id, root) makes lerobot query the HF Hub for the dataset
        # version -> 401/offline for a local-only dataset. Appending across recorder
        # restarts needs a Hub-free local load -- TODO.)
        root_str = str(root)
        if os.path.exists(os.path.join(root_str, 'meta', 'info.json')):
            raise RuntimeError(
                f"a dataset already exists at {root_str}. Delete it (rm -rf) or change "
                f"dataset.root / repo_id to record a fresh one. Cross-restart append is TODO.")
        self.ds = LeRobotDataset.create(
            repo_id=repo_id, fps=int(fps), root=root, robot_type=robot_type,
            features=ds_features, use_videos=True)
        self._info('created LeRobot dataset at %s' % root)
        self._task = None

    def _info(self, m):
        if self._log:
            self._log.info(m)

    def start_episode(self, task):
        self._task = task

    def add_frame(self, frame):
        # frame already contains feature arrays + 'task'; ensure float32 for low-dim
        out = {}
        for k, v in frame.items():
            if k == 'task':
                out[k] = v
            elif isinstance(v, np.ndarray) and v.dtype != np.uint8:
                out[k] = v.astype(np.float32)
            else:
                out[k] = v
        try:
            self.ds.add_frame(out)                     # newer API: task inside the frame
        except TypeError:
            task = out.pop('task', self._task)
            self.ds.add_frame(out, task=task)          # older API: task kwarg

    def save_episode(self):
        try:
            self.ds.save_episode()
        except TypeError:
            self.ds.save_episode(task=self._task)      # older API wanted task here

    def discard_episode(self):
        for attr in ('clear_episode_buffer', 'clear_episode'):
            if hasattr(self.ds, attr):
                getattr(self.ds, attr)()
                return
