# concert_nav2

Nav2 + SLAM Toolbox stack for the CONCERT robot in Gazebo simulation.

## How to run the simulation
<!-- Why do i need to do so on my pc? Need to investigate >.<
```bash

export GZ_SIM_RESOURCE_PATH=$SRC/concert_description/concert_gazebo/models:$SRC/concert_description:$SRC/velodyne_simulator:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=$INSTALL/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
``` -->

### 1. Gazebo + xbot2 + RViz
```bash
ros2 launch concert_gazebo modular.launch.xml \
    world_file:=$(ros2 pkg prefix concert_gazebo)/share/concert_gazebo/world/small_warehouse.sdf \
    velodyne:=true gui:=true rviz:=true xbot2_gui:=false
```

### 2. PointCloud2 → LaserScan
```bash
ros2 launch concert_nav2 pc2laser.launch.py \
    cloud_topic:=/VLP16_lidar_front/points scan_topic:=/scan
```

### 3. Odometry (base_estimation)
```bash
ros2 launch concert_odometry_ros2 concert_odometry.launch.py
```

### 4a. SLAM (online mapping)
```bash
ros2 launch concert_nav2 slam.launch.py
```

<!-- ### 4b. Localization (alternative, with a pre-saved map)
```bash
ros2 launch concert_nav2 localization.launch.py \
    map:=<forest_ws>/src/concert_description/concert_nav2/maps/warehouse.yaml
# bring lifecycle nodes up:
ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate
ros2 lifecycle set /amcl configure && ros2 lifecycle set /amcl activate
```
 -->
### 5. Nav2 (controller + planner + BT + behaviors + smoother)
```bash
ros2 launch concert_nav2 nav2.launch.py
```
<!-- 
## Save the map
```bash
mkdir -p <forest_ws>/src/concert_description/concert_nav2/maps
ros2 run nav2_map_server map_saver_cli -f \
    <forest_ws>/src/concert_description/concert_nav2/maps/warehouse
``` -->

### Manual teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/omnisteering/cmd_vel
```
