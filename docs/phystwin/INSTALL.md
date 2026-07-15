# PhysTwin recording add-on for `franka_data_recorder`

Adds a **PhysTwin-format** recording path to the existing LeRobot recorder. All files here are
**additive** — your existing `recorder_node.py`, `lerobot_writer.py`, `extractors.py`,
`recorder.yaml`, and the LeRobot → π0.5 pipeline are **unchanged**. LeRobot mode keeps working
exactly as before (`ros2 run franka_data_recorder recorder`).

## What each file is / where it goes

Copy these into your `franka_data_recorder` package, mirroring the layout:

| File (here) | Destination in the package | Role |
|---|---|---|
| `franka_data_recorder/depth_extractor.py` | `franka_data_recorder/depth_extractor.py` | registers `depth_raw` (uint16 mm, lossless) into the shared extractor registry |
| `franka_data_recorder/phystwin_writer.py` | `franka_data_recorder/phystwin_writer.py` | writes `color/*.png` + `depth/*.npy` + `metadata.json` (PhysTwin layout) |
| `franka_data_recorder/phystwin_recorder_node.py` | `franka_data_recorder/phystwin_recorder_node.py` | new entry point; subclass of `RecorderNode`, swaps the writer + reads `camera_info` |
| `franka_data_recorder/calibrate_phystwin.py` | `franka_data_recorder/calibrate_phystwin.py` | one-time Charuco extrinsics → `calibrate.pkl` + `gravity.json` |
| `config/recorder_phystwin_auto.yaml` | `config/recorder_phystwin_auto.yaml` | **auto-discover** mode: records every connected camera (no topics to edit) |
| `config/recorder_phystwin.yaml` | `config/recorder_phystwin.yaml` | static mode: explicit 3× color + 3× depth features (edit namespaces) |
| `launch/realsense_all.launch.py` | `launch/realsense_all.launch.py` | auto-launch EVERY connected RealSense (enumerates serials via pyrealsense2) |
| `launch/phystwin_all.launch.py` | `launch/phystwin_all.launch.py` | one-shot: all cameras + recorder (auto-discover) |
| `launch/phystwin_recorder.launch.py` | `launch/phystwin_recorder.launch.py` | recorder only (RMW + /usr/bin/python3, like recorder.launch.py) |
| `launch/phystwin_calibrate.launch.py` | `launch/phystwin_calibrate.launch.py` | one-time calibration under system python |

> **ros_ml compatible:** all add-on nodes are **cv_bridge-free** (depth/color decoded with
> NumPy straight from the message buffer, matching your `image_rgb` rewrite). The launch files
> pin `rmw_cyclonedds_cpp` and run through `/usr/bin/python3`, same as `recorder.launch.py`.
> Prefer the launch files over bare `ros2 run` so those settings apply.

## The ONE edit to an existing file: `setup.py`

Register the two new entry points (adds lines, changes nothing existing):

```python
entry_points={
    'console_scripts': [
        'recorder = franka_data_recorder.recorder_node:main',
        'gui = franka_data_recorder.gui_node:main',
        'fake = franka_data_recorder.fake_publisher:main',
        # --- PhysTwin add-on ---
        'phystwin_recorder = franka_data_recorder.phystwin_recorder_node:main',
        'phystwin_calibrate = franka_data_recorder.calibrate_phystwin:main',
    ],
},
```

Then rebuild: `colcon build --packages-select franka_data_recorder --symlink-install`.

## Usage — AUTO mode (recommended: plug in N cameras, record N cameras)

No serials, no topic names, no config edits. `realsense_all.launch.py` enumerates every
connected RealSense and launches it (align_depth on, common resolution); the recorder
auto-discovers all camera topics and records them. Camera index = namespaces sorted
alphabetically (e.g. d405=0, d435=1, d455=2).

```bash
# find D455 serial etc. is NOT needed in auto mode, but to sanity-check what's connected:
rs-enumerate-devices -s

# 1) calibrate once (see step 1 below) -> ~/phystwin_calib/{calibrate.pkl,gravity.json}
#    NOTE: calibrate's --cameras order must match the recorder's index order (sorted names).

# 2) launch all cameras + recorder together
ros2 launch franka_data_recorder phystwin_all.launch.py \
    dataset_name:=microwave_door_ep01 \
    calibrate_pkl:=$HOME/phystwin_calib/calibrate.pkl \
    gravity_json:=$HOME/phystwin_calib/gravity.json
# (resolution override: width:=1280 height:=720 fps:=30)

# 3) drive recording with the usual Trigger services (or the GUI)
ros2 service call /franka_data_recorder/start_recording std_srvs/srv/Trigger
ros2 service call /franka_data_recorder/stop_recording  std_srvs/srv/Trigger
```

Prefer to launch cameras yourself? Start them any way you like, then just:
`ros2 run franka_data_recorder phystwin_recorder --ros-args -p config_file:=<...>/recorder_phystwin_auto.yaml -p calibrate_pkl:=... -p gravity_json:=...`

## Usage — STATIC mode (pin an explicit camera set)

### 0. Edit the config
In `config/recorder_phystwin.yaml`, set the camera namespaces to match `ros2 topic list`
(e.g. `/d405`, `/d435`, `/d455`) and run `phystwin_recorder` with `auto_discover:=false`.
Depth must be `aligned_depth_to_color` (enable `align_depth.enable:=true`).

### 1. Calibrate once (cameras are static → reuse for all cases)
Place a Charuco board visible from all 3 cameras. Lay it FLAT on the floor/table if you want
gravity for free.
```bash
ros2 run franka_data_recorder phystwin_calibrate \
    --cameras /camera_0 /camera_1 /camera_2 \
    --out-dir ~/phystwin_calib \
    --gravity board          # 'board' (flat board) | 'base' | 'x,y,z'
# -> ~/phystwin_calib/calibrate.pkl  and  gravity.json
```
Gravity note: if you instead know the **camera↔robot-base** extrinsic and the robot is mounted
level, the down-direction is the base −Z rotated into the world frame — pass it as
`--gravity "gx,gy,gz"`. With a flat board (or world = level base), it's just `[0,0,-9.81]`.

### 2. Record (one start/stop = one PhysTwin case folder)
```bash
ros2 launch franka_data_recorder phystwin_recorder.launch.py \
    dataset_name:=microwave_door_ep01 \
    calibrate_pkl:=$HOME/phystwin_calib/calibrate.pkl \
    gravity_json:=$HOME/phystwin_calib/gravity.json
# in another shell — same Trigger services as the LeRobot recorder (CLI / GUI both work):
ros2 service call /franka_data_recorder/start_recording std_srvs/srv/Trigger
# ... perform: static pre-roll -> excite -> release -> free motion -> settle ...
ros2 service call /franka_data_recorder/stop_recording  std_srvs/srv/Trigger
```
Output case folder (under the config `root`, timestamped per run) contains:
```
color/{0,1,2}/{frame}.png    depth/{0,1,2}/{frame}.npy
metadata.json  calibrate.pkl  gravity.json
```

### 3. Feed PhysTwin
Point PhysTwin's `process_data.py` at it:
```bash
python process_data.py --base_path <parent-of-case-folder> --case_name <case-folder-name> \
    --category microwave   # object noun for Grounded-SAM2
```

## Verified / not verified
- ✅ Layout, metadata.json keys, depth units (uint16 mm), BGR png — matched against PhysTwin
  source (`data_process/data_process_pcd.py`, `qqtt/env/camera/camera_system.py`).
- ⚠️ **NOT hardware-tested** (no ROS2/RealSense here). First run, check: (a) all 6 topics
  publish so frames aren't skipped, (b) `metadata.json` intrinsics are non-zero (camera_info
  arrived), (c) a depth `.npy` loads as uint16 in millimetres, (d) `data_process_pcd.py`
  renders a sane point cloud.
- ⚠️ `cv2.aruco` API differs across OpenCV versions (calibration script mirrors PhysTwin's
  older API); adjust the 3 flagged aruco lines if your OpenCV ≥4.7 errors.

## Design notes
- **Synchronisation is inherited, not reinvented.** `RecorderNode._tick` emits a frame only
  when *every* configured topic has a message, with a monotonic index — that IS PhysTwin's
  "synchronised, shared, contiguous frame index across cameras" requirement.
- **Robot state (pose/wrench)** is not recorded here (PhysTwin ignores it). To keep it for
  later system identification, add those features back into `recorder_phystwin.yaml`; the
  PhysTwin writer ignores any non-image/non-depth keys.
