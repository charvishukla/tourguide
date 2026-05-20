Qualcomm Tourguide Project 

1. Development Commands: https://docs.google.com/document/d/1W1xiDhx1SPBUB9cWZxoI5_jqmf44flyGU7YIr4dX07Q/edit?usp=sharing
2. Documentation: https://docs.google.com/document/d/16fBF933qUPQ54OrpnyQiFzLinThJnUOtlMAH7NGwwjQ/edit?usp=sharing

----
# Plan: Raise Lidar and Camera Height on ROSbot Pro 3

## Context

The user physically raised the lidar and camera on their ROSbot Pro 3 and needs the URDF/config to reflect the new positions. Without this, TF frames (sensor → base_link) will be wrong, which breaks odometry fusion, costmap obstacle placement, and point-cloud-based mapping.

---

## Q1: What Exactly Needs to Be Updated?

There are **two files** to change — no URDF xacro rewrite needed.

### File 1 — Component positions (primary change)
**`rosbot_description/config/rosbot/basic.yaml`**

```yaml
components:
  - type: LDR02            # ← Lidar
    parent_link: cover_link
    xyz: 0.02 0.0 0.0      # ← z=0.0 means flush with cover_link
    rpy: 0.0 0.0 0.0

  - type: CAM11            # ← Camera
    parent_link: camera_mount_link
    xyz: -0.01 0.0 0.02    # ← z=0.02 above camera_mount_link
    rpy: 0.0 0.0 0.0
```

The `z` value in each `xyz:` line is the **offset from the parent link** in metres.

- To raise the lidar by Δ m → change LDR02's `xyz: 0.02 0.0 Δ`
- To raise the camera by Δ m → change CAM11's `xyz: -0.01 0.0 ${0.02+Δ}`

### File 2 — Camera mount joint (only if the physical mount pole height changed)
**`rosbot_description/urdf/rosbot/body.urdf.xacro` line 75**


body_to_camera_joint (z = 0.125 m) is the distance from the body_link origin up to camera_mount_link. Those are two specific coordinate frames, not the physical surfaces you'd measure with a ruler.

Here's what body_link origin actually is — it sits at wheel axle height above the ground (set by base_to_body_joint at z = wheel_radius ≈ 0.0425 m). It's not the top or bottom of the chassis — it's an internal reference point.

```
[ground]
   + 0.0425 m  (wheel_radius)  →  body_link origin
   + 0.0603 m                  →  cover_link  (top of chassis plate)
   + 0.0647 m  (0.125−0.0603)  →  camera_mount_link  (tip of the pole)
   + 0.0200 m  (CAM11 z)       →  camera optical frame
```



```xml
<joint name="body_to_camera_joint" type="fixed">
  <origin xyz="-0.0141 0.0 0.125" rpy="0.0 0.0 0.0" />   ← z=0.125 m above body_link
```

If you changed the mount arm/pole height (not just the camera bracket), also increase `0.125` here by the same physical delta. If only the camera itself moved on the existing mount, changing `basic.yaml` alone is sufficient.

### Reference frame chain (for understanding)

```
base_link
  └─ body_link         (z = wheel_radius above ground)
       ├─ cover_link   (z = +0.0603 m)  ← lidar sits here
       └─ camera_mount_link  (z = +0.125 m)  ← camera sits here
```

Measure heights from the **parent link**, not from the ground.
 
---

## Q2: How Are the Changes Applied?

The `basic.yaml` values are read at launch time via xacro — they are not compiled into a binary. The flow is:

1. `rosbot.urdf.xacro` reads `components_config` arg → defaults to `basic.yaml`
2. `husarion_components_description/urdf/components.urdf.xacro` instantiates the sensor macros (LDR02, CAM11) with your xyz/rpy offsets
3. The resolved URDF is published on `/robot_description`
4. `robot_state_publisher` broadcasts TF frames from it

**For Python/YAML-only changes** (editing `basic.yaml` only):
- With `--symlink-install` (the standard build mode here), the file is symlinked — **no rebuild needed**. Just restart the launch.
- Without symlink-install: run `colcon build --packages-select rosbot_description`.

**For xacro changes** (`body.urdf.xacro`):
- Same: symlink-install means the xacro is read live, so just restart the launch.

To verify the TF is correct after the change:
```bash
ros2 run tf2_tools view_frames   # generates frames.pdf
ros2 topic echo /robot_description | head -200   # check the resolved URDF
```

---

## Q3: How to Get Changes onto the Robot

These packages are currently cloned locally, not on the robot. Three options:

### Option A — Push to git, pull on robot (recommended for teams)
```bash
# Local machine
git checkout -b feat/raise-sensors
# edit the files
git commit -am "Update lidar/camera height in URDF"
git push origin feat/raise-sensors

# On the robot (SSH in)
cd ~/ros2_ws/src/rosbot_ros
git fetch && git checkout feat/raise-sensors
colcon build --symlink-install --packages-select rosbot_description
source ~/ros2_ws/install/setup.bash
ros2 launch rosbot_bringup rosbot.yaml
```

### Option B — Copy files directly via scp (fastest for testing)
```bash
# From your local machine
scp rosbot_description/config/rosbot/basic.yaml husarion@<robot-ip>:~/ros2_ws/src/rosbot_ros/rosbot_description/config/rosbot/basic.yaml
# If you also edited body.urdf.xacro:
scp rosbot_description/urdf/rosbot/body.urdf.xacro husarion@<robot-ip>:~/ros2_ws/src/rosbot_ros/rosbot_description/urdf/rosbot/body.urdf.xacro
```
Then restart the launch on the robot — no rebuild needed (symlink-install).

### Option C — Docker / snap
If the robot runs the Husarion Docker image or snap, rebuild the image with your changes and redeploy (see `docker/Dockerfile.hardware`).

---

## Critical Files

| File | Purpose | Lines to change |
|---|---|---|
| `rosbot_description/config/rosbot/basic.yaml` | Sensor xyz offsets from parent link | LDR02 z, CAM11 z |
| `rosbot_description/urdf/rosbot/body.urdf.xacro` | camera_mount_link joint origin | Line 75 (z=0.125) — only if mount pole height changed |

## Verification

1. Edit `basic.yaml`, change LDR02 z and CAM11 z to match physical measurements (metres from parent link)
2. Restart `ros2 launch rosbot_bringup rosbot.yaml`
3. Open RViz → add TF display → confirm `rplidar_link` and `camera_link` are at the right visual height above the robot body
4. Run `ros2 run tf2_tools view_frames` and check the z offsets in the TF tree PDF



---
Yes, it's fine. Here's why.

What actually matters for navigation, SLAM, and sensor fusion is where the final camera sensor frame (camera_link) ends up relative to base_link. Both approaches produce the same result:


Proper way (build from source):
  body_link + 0.205 (body_to_camera_joint) + 0.02 (CAM11 z) = 0.225 m

Snap workaround:
  body_link + 0.125 (body_to_camera_joint) + 0.10 (CAM11 z) = 0.225 m
Same final height. ✓

The only thing "wrong" with the workaround is that camera_mount_link stays at the old position — but that's harmless because:

It has no visual mesh in the URDF — it's just a bare <link name="camera_mount_link" /> with no geometry, so nothing looks wrong in RViz
Nothing in nav2, SLAM, or any sensor pipeline uses camera_mount_link directly — it's just a stepping stone to get to camera_link
The intermediate frame being "wrong" only matters if you had a physical component with a visual attached to camera_mount_link. You don't.

So: change CAM11 z from 0.02 to 0.10 in basic.yaml, and you're done.
