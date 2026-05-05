# Edubot Autonomy

ROS2 Python package for Edubot lane following, mapping, cone recognition, and sign handling.

## Build

```bash
cd /home/rix/ros2_ws
colcon build --packages-select edubot_autonomy
source install/setup.bash
```

## Run

```bash
ros2 launch edubot_autonomy edubot_autonomy_launch.py
```

## Topics

- Publishes `/cmd_vel` for robot motion
- Publishes `/map` as `nav_msgs/OccupancyGrid`
- Publishes `lane_markers` and `cone_markers` for RViz visualization

## Notes

- Uses a downward camera topic and front camera topic for lane/sign detection
- Uses `/scan` and `/odom` for obstacle avoidance and local mapping
- Re-initializes mapping at startup
