# Package Setup Guide

## Recommended Package Structure

```
your_package_name/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── your_package_name
├── your_package_name/
│   ├── __init__.py
│   └── scripts/
│       ├── lane_follower.py
│       └── camera_calibrator.py
├── launch/
│   └── lane_follower_launch.py
├── config/
│   └── lane_follower_params.yaml
└── README.md
```

## Step-by-Step Setup

### 1. Create or Navigate to Your Package

If you don't have a package yet:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python your_package_name \
    --dependencies rclpy sensor_msgs geometry_msgs cv_bridge
```

If you already have a package:
```bash
cd ~/ros2_ws/src/your_package_name
```

### 2. Create Directory Structure

```bash
mkdir -p your_package_name/scripts
mkdir -p launch
mkdir -p config
```

### 3. Copy Files

```bash
# Copy Python scripts
cp /path/to/lane_follower.py your_package_name/scripts/
cp /path/to/camera_calibrator.py your_package_name/scripts/
chmod +x your_package_name/scripts/*.py

# Copy launch file
cp /path/to/lane_follower_launch.py launch/

# Copy config
cp /path/to/lane_follower_params.yaml config/

# Copy README
cp /path/to/README.md .
```

### 4. Update package.xml

Edit `package.xml` and ensure you have these dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>your_package_name</name>
  <version>0.0.1</version>
  <description>Autonomous navigation package for Edubot</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>std_msgs</depend>
  
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 5. Update setup.py

Edit `setup.py`:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'your_package_name'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Autonomous navigation package for Edubot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_follower = your_package_name.scripts.lane_follower:main',
            'camera_calibrator = your_package_name.scripts.camera_calibrator:main',
        ],
    },
)
```

### 6. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select your_package_name
source install/setup.bash
```

### 7. Verify Installation

```bash
# Check if executables are available
ros2 pkg executables your_package_name

# Should show:
# your_package_name lane_follower
# your_package_name camera_calibrator

# Check if launch files are installed
ros2 launch your_package_name lane_follower_launch.py --show-args
```

## Testing Your Setup

### 1. First, Calibrate Your Camera

```bash
# Make sure your camera is running and robot is on the lane
ros2 run your_package_name camera_calibrator --ros-args -p camera_topic:=/your/camera/topic

# Adjust the trackbars until you see good detection
# Press 'Q' when satisfied - it will print optimal values
# Copy those values into lane_follower.py
```

### 2. Test Lane Following

```bash
# Start with slow speed for safety
ros2 launch your_package_name lane_follower_launch.py

# In another terminal, view the debug output
ros2 run rqt_image_view rqt_image_view /lane_debug
```

### 3. Monitor Performance

```bash
# View velocity commands
ros2 topic echo /cmd_vel

# Check node parameters
ros2 param list /lane_follower

# Adjust parameters on the fly
ros2 param set /lane_follower linear_speed 0.15
ros2 param set /lane_follower kp 0.008
```

## Integration with Other Nodes

Your final project will likely have multiple nodes. Here's a recommended architecture:

```
Main Navigation Node
├── Lane Follower (this node)
├── Obstacle Detector
├── Intersection Handler
├── Sign Recognition
└── SLAM/Mapping
```

Example main navigation logic:

```python
class MainNavigator(Node):
    def __init__(self):
        super().__init__('main_navigator')
        
        # Subscribe to sensor inputs
        self.obstacle_sub = self.create_subscription(...)
        self.sign_sub = self.create_subscription(...)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lane_follower_enable = self.create_publisher(Bool, '/lane_follower/enable', 10)
        
    def decision_loop(self):
        # Priority-based decision making
        if self.sign_detected == 'STOP':
            self.stop_robot()
            self.disable_lane_follower()
        elif self.obstacle_imminent:
            self.avoid_obstacle()
            self.disable_lane_follower()
        elif self.at_intersection:
            self.handle_intersection()
            self.disable_lane_follower()
        else:
            # Normal lane following
            self.enable_lane_follower()
```

## Troubleshooting Common Issues

### Build Errors

**Error: "No module named 'cv_bridge'"**
```bash
sudo apt-get install ros-<ros-distro>-cv-bridge
# Replace <ros-distro> with your ROS version (humble, foxy, etc.)
```

**Error: "Package not found"**
```bash
# Make sure you sourced the workspace
source ~/ros2_ws/install/setup.bash
# Add to ~/.bashrc to make permanent
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Runtime Errors

**Error: "No images received"**
- Check camera topic: `ros2 topic list | grep image`
- Verify camera is running: `ros2 topic hz /camera/image_raw`
- Check topic name in launch file matches your camera

**Error: "ImportError: No module named 'cv2'"**
```bash
pip3 install opencv-python
# Or for ROS
sudo apt-get install python3-opencv
```

**Warning: "No lane detected"**
- Run camera calibrator to tune color detection
- Check that robot is actually on the lane
- Verify lighting conditions
- Ensure camera is pointing at floor

## Tips for Final Demo

1. **Test early and often** - Don't wait until the last minute
2. **Keep backups** - Use Git to track changes
3. **Document parameters** - Write down working values for each test scenario
4. **Separate workspaces** - As mentioned in project requirements
5. **Practice demos** - Run through the full course multiple times
6. **Have fallbacks** - If lane following fails, have manual control ready
7. **Battery management** - Fully charge before demo
8. **Lighting check** - Test in the actual demo room lighting

## Git Repository Setup

```bash
cd ~/ros2_ws/src/your_package_name
git init
git add .
git commit -m "Initial commit: Lane following behavior"
git remote add origin <your-repo-url>
git push -u origin main
```

## Additional Resources

- ROS2 Documentation: https://docs.ros.org/
- OpenCV Python Tutorials: https://docs.opencv.org/master/d6/d00/tutorial_py_root.html
- PID Controller Tuning: https://en.wikipedia.org/wiki/PID_controller#Loop_tuning

## Contact

For team coordination and questions, make sure all team members have:
- Access to the Git repository
- Tested the code on the actual robot
- Understood the tuning parameters
- Documented their contributions

Good luck with your final project! 🚗🤖
