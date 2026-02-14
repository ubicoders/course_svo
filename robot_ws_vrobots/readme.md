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

### Step 1: Start Rerun visualization
```bash
rerun
```

### Step 2: Launch the bridge
```bash
ros2 launch vrobots_ros2_bridge bridge.launch.py sys_id:=1
```

### Step 3: Run SVO
```bash
ros2 run ubicoders_svo ubicoders_svo --ros-args -p sys_id:=1
```

### Step 4: Run Rerun publisher
```bash
ros2 run ubicoders_svo_rerun rerun_node
```

> **Note:** `sys_id` defaults to `0` if omitted. Use `sys_id:=1` (or another ID) to target a specific system.

## vrobots Sim

```bash
./virtual_robots/virtual_robots.x86_64
```
