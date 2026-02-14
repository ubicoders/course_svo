# vrobots ROS2 Workspace

## Prerequisites

If conda is active, make the system use default python.
```bash
bash use_system_python.bash
source ~/.bashrc
```

## Building

### Build all packages
```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install
```


### Build a specific package
```bash
colcon build --packages-select vrobots_ros2_controller --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --cmake-args -DPYTHON_EXECUTABLE=/miniconda/bin/python
```

### Source the workspace
```bash
source install/setup.bash
```

## Running

After building and sourcing, run the following steps in order:

### Step 1: RUN SVO

Choose the camera parameters based on your dataset:

**KITTI (default):**
```bash
ros2 run ubicoders_svo ubicoders_svo
# or explicitly:
ros2 run ubicoders_svo ubicoders_svo --ros-args -p dataset:=kitti
```

**EuROC:**
```bash
ros2 run ubicoders_svo ubicoders_svo --ros-args -p dataset:=euroc
```

**Custom Dataset:**
```bash
ros2 run ubicoders_svo ubicoders_svo --ros-args -p dataset:=custom
```

The node will automatically load the correct camera parameters (fx, fy, cx, cy, baseline) based on the dataset parameter.

### Step 2: RUN RVIZ2
```bash
rviz2
```
change the Global Options/Fix Frame to "ubicoders_svo"

### Step 3: Feed Dataset

#### KITTI Dataset
Simple usage (uses default sequence "00"):
```bash
ros2 run kitti_image_publisher kitti_image_publisher_node
```

Select a specific sequence (e.g., sequence 05):
```bash
ros2 run kitti_image_publisher kitti_image_publisher_node 05
```

All available parameters:
```bash
ros2 run kitti_image_publisher kitti_image_publisher_node --ros-args \
  -p sequence:=05 \
  -p dataset_root:=/path/to/KITTI/odom/dataset \
  -p publish_rate:=10.0 \
  -p start_index:=100 \
  -p repeat:=true
```

**Note:** Make sure to run SVO with `dataset:=kitti` (default) when using KITTI.

#### EuROC Dataset
Simple usage with full sequence name:
```bash
ros2 run euroc_image_publisher euroc_image_publisher_node MH_01_easy
```

Or use short codes (m1, m2, m3, m4, m5, v1, v1_02, v1_03, v2, v2_02):
```bash
ros2 run euroc_image_publisher euroc_image_publisher_node m1
```

All available parameters:
```bash
ros2 run euroc_image_publisher euroc_image_publisher_node --ros-args \
  -p sequence:=MH_03_medium \
  -p dataset_root:=/home/ubuntu/euroc \
  -p publish_rate:=20.0
```

**Available EuROC sequences:**
- `m1` → MH_01_easy
- `m2` → MH_02_easy
- `m3` → MH_03_medium
- `m4` → MH_04_difficult
- `m5` → MH_05_difficult
- `v1` → V1_01_easy
- `v1_02` → V1_02_medium
- `v1_03` → V1_03_difficult
- `v2` → V2_01_easy
- `v2_02` → V2_02_medium

**Note:** Make sure to run SVO with `dataset:=euroc` when using EuROC.

---

## Complete Workflow Examples

### Example 1: Running with KITTI
```bash
# Terminal 1: Start SVO with KITTI parameters
ros2 run ubicoders_svo ubicoders_svo

# Terminal 2: Start RViz2
rviz2

# Terminal 3: Feed KITTI sequence 00
ros2 run kitti_image_publisher kitti_image_publisher_node 00
```

### Example 2: Running with EuROC
```bash
# Terminal 1: Start SVO with EuROC parameters
ros2 run ubicoders_svo ubicoders_svo --ros-args -p dataset:=euroc

# Terminal 2: Start RViz2
rviz2

# Terminal 3: Feed EuROC MH_01_easy sequence
ros2 run euroc_image_publisher euroc_image_publisher_node m1
```

---

### Step 4: Run Rerun publisher
```bash
ros2 run ubicoders_svo_rerun rerun_node
```

> **Note:** `sys_id` defaults to `0` if omitted. Use `sys_id:=1` (or another ID) to target a specific system.

## vrobots Sim

```bash
./virtual_robots/virtual_robots.x86_64
```
