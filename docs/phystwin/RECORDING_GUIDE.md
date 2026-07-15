# Recording guide — one package, two data formats

The `franka_data_recorder` package records into **two independent formats**. You pick ONE per
recording session (they are different datasets for different purposes). The start/stop/discard
Trigger services and the GUI are identical for both.

| | **LeRobot mode** (robot policy training) | **PhysTwin mode** (digital-twin data) |
|---|---|---|
| Entry point | `recorder` (existing) | `phystwin_recorder` (add-on) |
| Config | `recorder.yaml` | `recorder_phystwin_auto.yaml` |
| Records | RGB (mp4) + proprio + action + task | per-cam RGB (png) + **depth (npy)** + calib |
| Output | LeRobot dataset (parquet + videos) | PhysTwin case folder (color/ depth/ meta) |
| Downstream | π0.5 / VLA finetune | PhysTwin `process_data.py` |
| Cameras | whatever `recorder.yaml` lists | all connected RealSense (auto-discovered) |
| Calibration | not needed | one-time Charuco (`calibrate.pkl`) |

> They are separate runs → a given recording is ONE format. (Recording both from a single
> pass would need a dual-writer; not built.)

---

## 0. One-time build
```bash
cd ~/franka_ws && colcon build --packages-select franka_data_recorder --symlink-install
source install/setup.bash
```

## Mode A — LeRobot (robot training)  [unchanged, existing pipeline]
```bash
ros2 launch franka_data_recorder recorder.launch.py \
    task:="pick up the cube" dataset_name:=pick_cube
# control (CLI or GUI):
ros2 service call /franka_data_recorder/start_recording std_srvs/srv/Trigger
ros2 service call /franka_data_recorder/stop_recording  std_srvs/srv/Trigger
```
→ LeRobot dataset under `data/<dataset_name>_<timestamp>/`. Feed your existing π0.5 pipeline.

## Mode B — PhysTwin (digital-twin data)

### B1. Launch the 3 RealSense cameras (848x480x30, aligned depth)
```bash
ros2 launch franka_data_recorder realsense_all.launch.py width:=848 height:=480 fps:=30
# verify all 3 have the triplet:
ros2 topic list | grep -E "color/image_raw$|aligned_depth_to_color/image_raw$|color/camera_info$"
```

> ros_ml note: use the **launch files** below (not bare `ros2 run`). Like `recorder.launch.py`,
> they pin `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and run through `/usr/bin/python3`, so ROS
> libs come from `/opt/ros`, not the Conda env. All these nodes are cv_bridge-free.

### B2. Calibrate ONCE (cameras static → reuse for all cases)
Charuco board flat on the table, visible to all 3 cameras. Calibration AUTO-DISCOVERS the
cameras in the SAME order the recorder uses (d405=0, d435=1, d455=2).
```bash
ros2 launch franka_data_recorder phystwin_calibrate.launch.py \
    out_dir:=$HOME/phystwin_calib gravity:=board
# -> ~/phystwin_calib/{calibrate.pkl, gravity.json}   (reproj err should be < 0.5 px)
```

### B3. Record (one start/stop = one PhysTwin case)
```bash
ros2 launch franka_data_recorder phystwin_recorder.launch.py \
    dataset_name:=microwave_door_ep01 \
    calibrate_pkl:=$HOME/phystwin_calib/calibrate.pkl \
    gravity_json:=$HOME/phystwin_calib/gravity.json
# log should say: auto-discovered 3 camera(s): ['/d405/camera','/d435/camera','/d455/camera']

# episode protocol: static pre-roll (1-2s) -> apply initial velocity by hand -> hand leaves
# frame -> free motion -> settle (post-roll 1-2s). START before you touch the object.
ros2 service call /franka_data_recorder/start_recording std_srvs/srv/Trigger
ros2 service call /franka_data_recorder/stop_recording  std_srvs/srv/Trigger
```
→ case folder under `data/different_types/microwave_door_ep01_<timestamp>/` with
`color/{0,1,2}/*.png`, `depth/{0,1,2}/*.npy`, `metadata.json`, `calibrate.pkl`, `gravity.json`.

### B4. Feed PhysTwin
```bash
python process_data.py --base_path <parent-of-case> --case_name <case-folder> --category microwave
```

### B5. Sanity checks after the first case
```bash
# depth is uint16 millimetres?
python3 -c "import numpy as np; d=np.load('<case>/depth/0/0.npy'); print(d.dtype, int(d.max()))"  # uint16, hundreds..few-thousand
# intrinsics captured (non-zero)?
python3 -c "import json; print(json.load(open('<case>/metadata.json'))['intrinsics'][0])"
# frame_num > 0 and equals #files
python3 -c "import json; print(json.load(open('<case>/metadata.json'))['frame_num'])"
```

---

## Tips
- **One recording session = one format.** For robot training run `recorder`; for twin data run
  `phystwin_recorder`. Don't run both at once (they'd both grab the cameras).
- **Recalibrate** only when a camera moves. Otherwise reuse `~/phystwin_calib/*`.
- Want robot state stored alongside PhysTwin data (future system-ID)? Add those proprio/action
  features into `recorder_phystwin_auto.yaml`; the PhysTwin writer ignores non-image/depth keys.
- GUI works for both: `ros2 run franka_data_recorder gui` → http://localhost:8088.
