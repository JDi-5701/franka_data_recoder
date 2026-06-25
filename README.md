# franka_data_recorder

A ROS 2 package that records teleoperation episodes from the 2-PC Franka setup directly
into the **[LeRobot](https://github.com/huggingface/lerobot) dataset format**, so the data
can fine-tune **π0.5 (pi0.5)** out of the box and, later, feed world models (JEPA, Cosmos).

GitHub: `https://github.com/JDi-5701/franka_data_recoder`

> **This README is the requirements + design spec.** Implementation follows once it's agreed.
> Status: 🟦 requirements drafted from the goals below.

---

## 1. Goals

1. **Primary:** record teleop demonstrations → **LeRobot v2.1 dataset** → fine-tune **π0.5**
   with no conversion step.
2. **Future:** the same recordings should be usable by world models / video models
   (**JEPA**, **NVIDIA Cosmos**) — i.e. keep raw-ish, richly-observed data (RGB, optionally
   depth, proprio, actions, language).
3. **Open, config-driven interface:** *what* gets recorded is defined in a YAML config, not
   in code. Adding/removing a stream is a config edit.
4. **Incrementally extensible:** new ROS message types can be supported by adding a small
   "extractor" plug-in, without touching the core recorder.

5. **Web GUI** (planned): one page to **visualize** everything (robot state + camera images)
   and to **control** the session — reset, start/stop recording, mark success.

Non-goals (for now): training code, dataset upload/versioning, online inference.

---

## 2. Output format — LeRobot v2.1

One dataset = many episodes. On disk (what π0.5 / `lerobot` expects):
```
<dataset>/
  meta/info.json        # feature schema, fps, robot_type, counts, codebase version
  meta/episodes.jsonl   # per-episode: length, task(s)
  meta/tasks.jsonl      # task_index -> language instruction
  meta/stats.json       # per-feature mean/std/min/max (normalization)
  data/chunk-000/episode_000000.parquet   # low-dim timeseries (state, action, bookkeeping)
  videos/chunk-000/observation.images.<cam>/episode_000000.mp4  # one mp4 per camera
```
Canonical feature keys (π0.5 / VLA convention):
- `observation.state` — proprioception vector (see §4)
- `observation.images.<name>` — one RGB stream per camera
- `action` — the commanded action vector
- `task` — natural-language instruction (**required** for π0.5; language-conditioned)
- bookkeeping (auto): `timestamp`, `frame_index`, `episode_index`, `index`, `task_index`

Implementation may use the `lerobot` `LeRobotDataset` writer if available, else a
self-contained writer producing the same layout.

---

## 3. Architecture — the open interface

```
ROS topics ──► [per-msg-type EXTRACTOR] ──► numpy / image ──► [SYNCHRONIZER @ fps] ──► [LeRobot WRITER]
                       ▲                                              ▲
                 registry (add new                            config.yaml decides
                 msg types here)                              features + topics + fps
```

- **config.yaml** is the single source of truth: it lists the dataset `fps`, the
  `task`/language source, and every **feature** with its source topic, message type,
  extractor, field spec, dtype and shape.
- **Extractor registry:** one small function per ROS message type that converts a message
  into an array (or an image). Supporting a new message type = register one extractor +
  reference it in config. This is the "incrementally add new msg type" requirement.
- **Synchronizer:** picks a master rate (`fps`, usually a camera) and aligns every feature
  to each frame by nearest-timestamp (poses may be interpolated). Decouples the 1 kHz / 100
  Hz source rates from the dataset rate.
- **Writer:** accumulates a frame buffer per episode and flushes a LeRobot episode on stop.

---

## 4. Important ROS message types for robot learning

These are the types worth supporting; ✅ = available now in our setup, ➕ = needs adding.
Each maps to a LeRobot feature via an extractor.

| ROS message type | Carries | LeRobot feature | Status |
|------------------|---------|-----------------|--------|
| `sensor_msgs/JointState` | arm q / dq / effort, gripper width | part of `observation.state` (proprio) | ➕ arm joints NOT published yet by `cartesian_impedance_node` (see §6); gripper via `/franka_gripper/joint_states` ✅ |
| `geometry_msgs/PoseStamped` | TCP pose (current = state, target = action) | `observation.state` / `action` | ✅ `current_pose`, `target_pose` |
| `sensor_msgs/Image` / `CompressedImage` | RGB camera | `observation.images.<cam>` | ➕ no camera connected yet |
| `sensor_msgs/CameraInfo` | intrinsics | stored once in `meta` | ➕ |
| `geometry_msgs/WrenchStamped` | external F/T | `observation.state` (contact tasks) | ✅ `ext_wrench` |
| `geometry_msgs/Twist`/`TwistStamped` | velocity command | alt `action` | ✅ `/spacemouse/raw_command` |
| gripper command / width (`JointState` / `Float64*` / `franka_msgs`) | gripper open/close | part of `action` | ✅ via gripper server |
| `std_msgs/String` | language instruction | `task` | ➕ need a per-episode source |
| depth `sensor_msgs/Image` (16UC1) / `sensor_msgs/PointCloud2` | depth / 3D | extra `observation.images.depth` / state | ➕ future (Cosmos/JEPA) |
| `tf2_msgs/TFMessage` | frame transforms / cam extrinsics | meta (optional) | ✅ if needed |
| `std_msgs/Int8MultiArray` | SpaceMouse buttons | **control/trigger**, not data | ✅ |

A typical **π0.5 episode** therefore needs at minimum: `observation.images.*` (≥1 camera),
`observation.state` (proprio: joints+EE+gripper), `action` (target EE pose + gripper), and a
`task` string.

---

## 5. Recording behaviour — **? still to decide**

- **Episode model:** start/stop **per demonstration** (one episode = one demo). _assumed_
- **Trigger:** **? DECIDE** — SpaceMouse button vs ROS service vs keyboard. (Buttons are read
  on `/spacemouse/buttons`; a button-driven start/stop/mark-success is convenient hands-free.)
- **fps:** **? DECIDE** — e.g. 30 (camera-driven) for π0.5; proprio/action resampled to it.
- **task/language:** **? DECIDE** — set per episode (config field, CLI arg, or a topic/service).
- **Episode end metadata:** success/fail flag, notes — **? DECIDE** how to mark.
- **Storage path / naming** on `prs`: **? DECIDE**.

---

## 6. Known gaps / dependencies (record these as work items)

1. **Arm joint states are not published.** `cartesian_impedance_node` currently publishes only
   `current_pose`, `target_pose`, `ext_wrench`. Proper proprioception for π0.5/IL needs joint
   `q/dq/tau_J` (libfranka `RobotState` already has them). → add a `joint_states` publisher to
   the controller (low rate, decimated).
2. **No camera yet.** Need a camera driver (RealSense / USB) publishing `Image`. Decide model,
   mount (wrist/3rd-person), resolution, fps.
3. **Language/task source** for π0.5 — decide how each episode gets its instruction.

---

## 7. Config sketch (illustrative — the "open interface")

```yaml
dataset:
  repo_id: franka_pickplace
  root: /data/lerobot/franka_pickplace
  fps: 30
  robot_type: franka_fr3
task:
  source: cli            # cli | topic:/task_text | config
  default: "pick up the cube and place it in the bowl"
trigger:
  start_stop: spacemouse_button   # spacemouse_button | service | keyboard
features:
  observation.images.wrist:
    topic: /camera/wrist/color/image_raw
    type: sensor_msgs/Image
    extractor: image_rgb
    shape: [480, 640, 3]
  observation.state:
    concat:                         # build one vector from several sources
      - { topic: /franka/joint_states, type: sensor_msgs/JointState, extractor: jointstate_pos, dim: 7 }
      - { topic: /cartesian_impedance_node/current_pose, type: geometry_msgs/PoseStamped, extractor: pose_7d, dim: 7 }
      - { topic: /franka_gripper/joint_states, type: sensor_msgs/JointState, extractor: jointstate_pos, dim: 1 }
  action:
    concat:
      - { topic: /cartesian_impedance_node/target_pose, type: geometry_msgs/PoseStamped, extractor: pose_7d, dim: 7 }
      - { topic: /gripper_cmd, type: std_msgs/Float64, extractor: scalar, dim: 1 }
```

---

## 8. Control & GUI architecture (no topic conflict)

The robot is commanded by **one** publisher on `target_pose` (teleop OR a policy). The GUI
and the recorder must therefore **never publish poses** — they are pure clients that call
services. This avoids fighting the teleop stream.

```
 SpaceMouse teleop ─┐
 (or policy)        ├─► /cartesian_impedance_node/target_pose ─► robot   (ONE active driver)
                    │
 Web GUI ───────────┼─► (service) ~/reset / go_home  on the controller  ── homing, ignores target_pose
                    ├─► (topic)   /reset_teleop       re-latch teleop equilibrium after homing
                    └─► (service) recorder ~/start_recording / ~/stop_recording / ~/discard_episode
 Web GUI  ◄──────────── subscribes current_pose / ext_wrench / images / joint_states  (visualize)
```

- **Reset / go-home = a service ON the controller** (`cartesian_impedance_node`) — IMPLEMENTED:
  `~/go_home` (srv `franka_cartesian_impedance_node/GoToPose`, in-package). It creeps the
  controller's own equilibrium to the requested pose at a slow cap and **ignores `target_pose`
  while homing** → single owner, no conflict; blocks until reached.
- **`~/reset` on this recorder** — IMPLEMENTED: reads the per-task home pose from
  `config/recorder.yaml` (`reset:` section), calls the controller's `go_home`, then publishes
  `/reset_teleop` so `teleop_interface` re-latches its equilibrium to the new pose (else it
  yanks the arm back). So the GUI just calls `recorder ~/reset` — it never publishes poses.
- **Record control = services on this recorder** (see §10).
- The GUI is a thin web client (e.g. **rosbridge + a web page**, or **Foxglove**) that
  subscribes for visualization and calls these services for control.

> The general version is still a command **mux** (teleop / policy / reset arbitrated onto
> `target_pose`); the go_home service is the minimal slice and enough for record + reset.

---

## 9. Recording behaviour decisions (current)
- Episode = one demonstration; **start/stop via Trigger services** (CLI, button, or GUI).
- `fps` from config (default 30); topics are latest-sampled to that rate.
- `task` (language) from the `task` param or the config default.
- **? still open:** success/fail marking, SpaceMouse-button → service binding, storage naming.

---

## 10. Run the minimal recorder (v0)

Implemented now: config-driven recorder of the available low-dim streams
(`current_pose`, `target_pose`, `ext_wrench`, gripper `joint_states`) → LeRobot dataset.
(Cameras + arm joints are config-ready but commented out until they exist — see §6.)

```bash
# deps (GPU operator PC): lerobot in the same python env as ROS
pip install lerobot --break-system-packages      # or 'lerobot[pi0]'

# build + source (on the GPU PC)
cd ~/franka_ws && colcon build --packages-select franka_data_recorder --symlink-install
source install/setup.bash

# run (edit config/recorder.yaml first: dataset root, repo_id)
ros2 launch franka_data_recorder recorder.launch.py task:="pick up the cube"

# name the output folder by task (-> data/pick_cube_<timestamp>/). task = language label,
# dataset_name = folder/repo_id base. Each run still gets a fresh timestamped dir.
ros2 launch franka_data_recorder recorder.launch.py task:="pick up the cube" dataset_name:=pick_cube
#   or on a bare node:  ros2 run franka_data_recorder recorder --ros-args -p dataset_name:=pick_cube

# control it (CLI now; GUI/button later call the same services)
ros2 service call /franka_data_recorder/start_recording std_srvs/srv/Trigger
ros2 service call /franka_data_recorder/stop_recording  std_srvs/srv/Trigger
ros2 service call /franka_data_recorder/discard_episode std_srvs/srv/Trigger
ros2 service call /franka_data_recorder/reset           std_srvs/srv/Trigger   # home + re-latch teleop
```

The reset home pose lives in `config/recorder.yaml` under `reset:` (per-task). The
controller's homing service can also be called directly:
```bash
ros2 service call /cartesian_impedance_node/go_home franka_cartesian_impedance_node/srv/GoToPose \
  "{pose: {position: {x: 0.4, y: 0.0, z: 0.4}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}, max_velocity: 0.08}"
```
Edit `config/recorder.yaml` to change **what** is recorded — no code change. Add a new
message type by adding an extractor in `franka_data_recorder/extractors.py` and referencing
it from the config.

> ⚠️ v0 is written but **not yet hardware-tested**; the `lerobot` writer targets the 0.x
> `LeRobotDataset` API — if your installed version differs, only `lerobot_writer.py` needs a
> tweak (import path / `add_frame` / `save_episode` signature).

### Web GUI (start/stop/discard/reset buttons)
A minimal self-contained web page (no rosbridge) whose buttons call the recorder services.
Runs on the GPU; view it locally or SSH-tunnel the port to your laptop.
```bash
# on the GPU (recorder must also be running)
ros2 run franka_data_recorder gui            # serves http://localhost:8088
# optional: ros2 run franka_data_recorder gui --ros-args -p port:=8088 -p recorder_node:=/franka_data_recorder
```
View it:
- **At the GPU:** open `http://localhost:8088` in a browser.
- **From your own PC (remote):** tunnel the port, then open it locally:
  ```bash
  ssh -L 8088:localhost:8088 prs@<gpu-tailscale-ip>     # keep open
  # then browse http://localhost:8088 on your PC
  ```
Buttons: **Start / Stop / Discard** call the record services; **Reset robot** calls `~/reset`
(go_home + re-latch teleop). The page is the place to add live state + camera views later.

---

## 11. Roadmap
1. ✅ v0 recorder (low-dim streams → LeRobot) — **test on hardware**, confirm a dataset loads
   in `lerobot`.
2. ✅ `~/go_home` service + homing mode in `cartesian_impedance_node`, recorder `~/reset`
   wired (§8) — **build + test on hardware**.
3. Add arm `joint_states` publisher to the controller (§6.1).
4. Web GUI: rosbridge/Foxglove page — visualize state+images, buttons for reset/record.
5. Add a camera; record `observation.images.*`; first π0.5 fine-tune smoke test.
6. Add depth / extra cameras for world-model (JEPA/Cosmos) compatibility.

---

## 12. Adaptivity — what's config-driven today, and what's not (TODO)

### ✅ What adapts right now (edit `recorder.yaml`, no code change)
The recorder, the GUI dashboard, and the data player are all built dynamically from the
`features:` block of the config — so the same package handles very different robot setups by
config alone:

| Scenario | How |
|---|---|
| **Multiple robots** | namespace the topics (`/robot1/...`, `/robot2/...`) and add each as a feature/source |
| **Custom topic** | add one `{topic, type, extractor, dim}` line under a feature (or in a `concat`) |
| **Joint-only vs TCP-only** | keep a separate config per setup; list only the sources that setup publishes |
| **Multiple cameras** | add several `observation.images.<name>` features → GUI shows N panels automatically |
| **Naming a run** | `dataset_name:=<task>` → `data/<task>_<timestamp>/` (folder + repo_id) |

The **GUI** reads the same config and adapts its camera panels + curve plots; the **player**
lists every dataset under `data/` and replays a chosen episode through those same panels.

### ⚠️ Current limits (carry these as TODO)
1. **Cross-config replay is not self-describing.** The player slices a recorded feature
   vector (e.g. `observation.state`, 22-dim) back onto per-topic curves using the **config the
   GUI was launched with** — *not* the dataset's own schema. Replaying a dataset recorded under
   a *different* config (different dims / topics / camera count) than the running GUI will
   mis-slice or mismatch. Root cause: LeRobot `meta/info.json` stores feature names + shapes
   but **not** our `concat → topic` breakdown.
   **TODO:** write the topic/concat mapping into the dataset at record time (a sidecar next to
   `meta/`), and have the player rebuild panels + slicing per-dataset from *its own* schema, so
   any dataset replays correctly regardless of the GUI's launch config.
2. **All configured sources are mandatory.** `_build_frame` drops a frame until *every* listed
   topic has published, so one config cannot "record whatever happens to be live" — joint-only
   vs tcp-only is handled by swapping configs, not auto-detection.
   **TODO:** an `optional: true` flag per source — skip a missing optional source instead of
   dropping the whole frame, so one config can adapt to "joints if present, TCP if present".
3. **Novel message types need code.** Known types (PoseStamped / JointState / WrenchStamped /
   Image …) work from config; a brand-new message type needs a small extractor in
   `extractors.py` + a `_TYPE_MAP` entry.
   **TODO:** document the extractor-plugin pattern (and ship ready-made multi-arm / multi-cam /
   joint-only / tcp-only example configs).
