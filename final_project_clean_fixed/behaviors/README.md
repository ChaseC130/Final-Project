# Final Project ROS2 Package

Package name: `behaviors`

## Correct folder structure

Put this folder at:

```bash
~/ros2_ws/src/behaviors
```

The structure should be:

```text
behaviors/
├── behaviors/
│   ├── __init__.py
│   ├── autonomous_mapper.py
│   ├── cone_detector.py
│   ├── lane_detector.py
│   ├── lane_map_publisher.py
│   ├── road_navigator.py
│   └── sign_detector.py
├── launch/
│   ├── final_project.launch.py
│   └── mapping.launch.py
├── resource/
│   └── behaviors
├── package.xml
├── setup.cfg
└── setup.py
```

Do not place `build`, `install`, or `log` inside this package.

## Build

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select behaviors
source install/setup.bash
```

## Run

```bash
ros2 launch behaviors final_project.launch.py use_sim_time:=false image_topic:=/camera_1/image_raw scan_topic:=/scan
```

## RViz topics

Add Marker displays for:

- `/lane/map_markers`
- `/cones/markers`

Add Image display for:

- `/lane/debug_image`
