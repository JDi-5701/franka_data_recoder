"""PhysTwin dataset writer -- same interface as LeRobotWriter, different on-disk layout.

ADDITIVE: does not import or modify lerobot_writer.py. The phystwin recorder node picks this
writer instead of LeRobotWriter; everything else in RecorderNode is reused unchanged.

Produces exactly what the PhysTwin pipeline (data_process/*) consumes for one case:

    {case}/color/{cam}/{frame}.png    BGR png via cv2 (lossless; PhysTwin reads it back and
                                      does cv2 BGR->RGB, so we store BGR)
    {case}/depth/{cam}/{frame}.npy    uint16 millimetres (PhysTwin does depth/1000 -> metres)
    {case}/metadata.json              intrinsics / WH / fps / frame_num / serial_numbers
    {case}/calibrate.pkl              copied from the one-time calibration (cameras are static)
    {case}/gravity.json               copied from the one-time calibration

Frame routing is by feature-key suffix (set in recorder_phystwin.yaml):
    observation.images.<i>  -> color cam i   (HxWx3 uint8 RGB, from image_rgb)
    observation.depth.<i>   -> depth cam i   (HxW uint16 mm, from depth_raw)

One recorder start/stop == one PhysTwin case folder. Extra start/stops in the same run get an
auto-incremented suffix so nothing is overwritten.

NO cv_bridge (matches this repo's ros_ml/system-ROS workaround). cv2 is used only to encode the
PNG (same as gui_node.py); the RGB->BGR swap is a plain NumPy slice, and cv2 is imported lazily.
"""
import json
import os
import shutil

import numpy as np

_CV2 = None  # lazily imported, like gui_node.py


def _cv2():
    global _CV2
    if _CV2 is None:
        import cv2
        _CV2 = cv2
    return _CV2


class PhysTwinWriter:
    def __init__(self, case_root, num_cam, meta_provider, fps,
                 calibrate_src=None, gravity_src=None, logger=None):
        self._log = logger
        self.num_cam = int(num_cam)
        self.fps = int(fps)
        self.meta_provider = meta_provider          # callable -> {intrinsics, WH, serial_numbers}
        self.calibrate_src = calibrate_src
        self.gravity_src = gravity_src
        self._base_root = str(case_root)
        self._case = None
        self._frame = 0
        self._episode = 0

    def _info(self, m):
        if self._log:
            self._log.info(m)

    def _new_case_dir(self):
        root = self._base_root
        if os.path.exists(root):                    # never clobber an existing case
            i = 1
            while os.path.exists(f"{self._base_root}_{i}"):
                i += 1
            root = f"{self._base_root}_{i}"
        for i in range(self.num_cam):
            os.makedirs(f"{root}/color/{i}", exist_ok=True)
            os.makedirs(f"{root}/depth/{i}", exist_ok=True)
        return root

    # ---- LeRobotWriter-compatible interface --------------------------------
    def start_episode(self, task=None):
        self._case = self._new_case_dir()
        self._frame = 0
        self._info(f"[phystwin] recording case -> {self._case}")

    def add_frame(self, frame):
        f = self._frame
        for key, val in frame.items():
            if key.startswith("observation.images."):
                idx = int(key.rsplit(".", 1)[1])
                rgb = np.asarray(val, dtype=np.uint8)
                bgr = np.ascontiguousarray(rgb[:, :, ::-1])  # RGB->BGR (PhysTwin reads BGR)
                _cv2().imwrite(f"{self._case}/color/{idx}/{f}.png", bgr)
            elif key.startswith("observation.depth."):
                idx = int(key.rsplit(".", 1)[1])
                d = np.asarray(val)
                if d.dtype != np.uint16:
                    d = (np.round(d * 1000.0).astype(np.uint16)
                         if np.issubdtype(d.dtype, np.floating) else d.astype(np.uint16))
                np.save(f"{self._case}/depth/{idx}/{f}.npy", d)
            # any other keys (e.g. 'task', proprio) are ignored by PhysTwin
        self._frame += 1

    def save_episode(self):
        if self._case is None:
            return
        meta = dict(self.meta_provider() or {})
        meta.setdefault("fps", self.fps)
        meta["frame_num"] = self._frame
        with open(f"{self._case}/metadata.json", "w") as fp:
            json.dump(meta, fp, indent=2)
        # cameras are static -> copy the one-time calibration artifacts into the case folder
        for src, name in ((self.calibrate_src, "calibrate.pkl"),
                          (self.gravity_src, "gravity.json")):
            if not src:
                continue
            if os.path.isfile(src):
                shutil.copyfile(src, f"{self._case}/{name}")
            else:
                self._info(f"[phystwin] WARNING: {name} source not found: {src} "
                           f"(run calibrate_phystwin first)")
        self._info(f"[phystwin] saved case {self._case} ({self._frame} frames)")
        self._episode += 1
        self._case = None

    def discard_episode(self):
        if self._case and os.path.isdir(self._case):
            shutil.rmtree(self._case, ignore_errors=True)
            self._info(f"[phystwin] discarded case {self._case}")
        self._case = None

    def close(self):
        pass  # nothing buffered; every frame is flushed to disk immediately
