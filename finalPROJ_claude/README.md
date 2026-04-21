# Lane Following Behavior for Edubot

This ROS2 node implements lane following behavior that keeps your Edubot centered in a lane marked by solid white lines (right boundary) and dashed yellow lines (left boundary).

## Features

- ✅ Detects solid white lines and dashed yellow lines using color filtering
- ✅ Maintains position in lane center
- ✅ Never crosses solid white line (with safety margin)
- ✅ Can cross dashed yellow line if needed
- ✅ PID-based steering control
- ✅ Real-time debug visualization
- ✅ Configurable parameters for tuning

## Installation

### 1. Add to Your ROS2 Package

```bash
cd ~/your_workspace/src/your_package
mkdir -p scripts launch config
cp lane_follower.py scripts/
cp lane_follower_launch.py launch/
cp lane_follower_params.yaml config/
chmod +x scripts/lane_follower.py
```

### 2. Update package.xml

Add these dependencies:
```xml
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>cv_bridge</depend>
<depend>opencv2</depend>
```

### 3. Update setup.py

Add entry point:
```python
entry_points={
    'console_scripts': [
        'lane_follower = your_package.scripts.lane_follower:main',
    ],
},
```

Also add data files:
```python
data_files=[
    # ... existing entries ...
    (os.path.join('share', package_name, 'launch'), 
     glob('launch/*.py')),
    (os.path.join('share', package_name, 'config'), 
     glob('config/*.yaml')),
],
```

### 4. Build

```bash
cd ~/your_workspace
colcon build --packages-select your_package_name
source install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch your_package_name lane_follower_launch.py
```

### With Custom Camera Topic

```bash
ros2 launch your_package_name lane_follower_launch.py camera_topic:=/your/camera/topic
```

### With Parameter File

```bash
ros2 run your_package_name lane_follower --ros-args --params-file config/lane_follower_params.yaml
```

### View Debug Visualization

```bash
# In another terminal
ros2 run rqt_image_view rqt_image_view /lane_debug
```

## Configuration & Tuning

### Key Parameters

**Speed Settings:**
- `linear_speed`: Forward speed (m/s). Start at 0.15-0.2, increase gradually
- `max_angular_speed`: Maximum turning speed (rad/s). Typical: 0.4-0.6

**PID Controller:**
- `kp`: Proportional gain. Controls immediate response. Start: 0.005
- `ki`: Integral gain. Eliminates steady-state error. Start: 0.0001 (keep small!)
- `kd`: Derivative gain. Reduces oscillations. Start: 0.001

**Vision:**
- `roi_height`: Portion of image to analyze (0.3 = bottom 30%)
- `min_line_length`: Minimum pixels for valid line (adjust for camera resolution)

### Tuning Guide

**Problem: Robot oscillates/weaves**
- Reduce `kp` by 20-30%
- Increase `kd` by 50-100%
- Check if lanes are clearly visible

**Problem: Robot responds too slowly to curves**
- Increase `kp` by 20-30%
- Reduce `kd` slightly
- Increase `max_angular_speed`

**Problem: Robot drifts to one side consistently**
- Increase `ki` slightly (0.0001 → 0.0002)
- Check camera calibration
- Verify lane markings are symmetric

**Problem: Robot loses lane detection**
- Adjust color ranges in code (HSV values)
- Increase `roi_height` to see more of the lane
- Improve lighting conditions
- Reduce `min_line_length` for broken lines

**Problem: Robot gets too close to solid line**
- The code has a built-in safety margin (adjust `min_distance` in code)
- Increase `kp` for faster corrections
- Check that solid line is being detected correctly

## How It Works

### 1. Vision Processing
- Captures images from downward-facing camera
- Extracts Region of Interest (ROI) from bottom portion
- Applies color filtering:
  - **White detection**: HSV (0-180, 0-30, 200-255)
  - **Yellow detection**: HSV (20-30, 100-255, 100-255)
- Uses Canny edge detection + Hough line transform
- Calculates line positions

### 2. Lane Center Calculation
- **Both lines detected**: Center = (solid_x + dashed_x) / 2
- **Only solid line**: Assumes lane width, stays left of solid
- **Only dashed line**: Assumes lane width, stays right of dashed
- **No lines**: Stops (safety behavior)

### 3. Control
- Calculates error = lane_center - robot_position
- PID controller adjusts angular velocity
- Safety check prevents crossing solid line
- Publishes Twist message to move robot

## Debug & Monitoring

### View Debug Image
The `/lane_debug` topic shows:
- **Blue lines**: Detected white (solid) lines
- **Green lines**: Detected yellow (dashed) lines
- **Yellow line**: Calculated lane center
- **Red line**: Robot's current position
- Text overlay with line positions

### Monitor Performance
```bash
# See node logs
ros2 run your_package_name lane_follower

# Check topics
ros2 topic list | grep lane
ros2 topic echo /cmd_vel

# View parameters
ros2 param list /lane_follower
ros2 param get /lane_follower kp
```

## Troubleshooting

### No lane detection
1. Check camera topic: `ros2 topic echo /camera/image_raw`
2. Verify camera is pointing down at floor
3. Check lighting - need good contrast
4. View raw camera: `ros2 run rqt_image_view rqt_image_view`
5. Adjust HSV color ranges in code if tape colors differ

### Robot doesn't move
1. Check if cmd_vel is being published: `ros2 topic echo /cmd_vel`
2. Verify correct topic name for your robot
3. Check that lane is detected (view debug image)
4. Ensure robot motors are enabled

### Erratic behavior
1. Reduce speed: lower `linear_speed` to 0.1 m/s
2. Check for vibrations affecting camera
3. Ensure tape lines are straight and continuous
4. Tune PID parameters (start by reducing kp by half)

### Robot crosses solid line
1. Check solid line detection in debug image
2. Increase safety margin (`min_distance` in code)
3. Verify white color detection (may need HSV adjustment)
4. Reduce speed and increase responsiveness (higher kp)

## Integration with Other Behaviors

This lane follower can be integrated with:
- **Obstacle avoidance**: Temporarily override control when obstacle detected
- **Intersection handling**: Disable lane following at intersections
- **Sign detection**: Modify behavior based on traffic signs
- **SLAM mapping**: Run concurrently to build map while following lane

Example integration pattern:
```python
# In your main navigation node
if obstacle_detected:
    # Stop lane follower, use obstacle avoidance
    pass
elif at_intersection:
    # Stop lane follower, use intersection behavior
    pass
else:
    # Let lane follower control robot
    pass
```

## Camera Setup Requirements

- **Mounting**: Camera must face downward toward floor
- **Angle**: Approximately 45-60 degrees from vertical
- **Height**: High enough to see 1-2 feet ahead
- **Focus**: Set to infinity or fixed focus
- **Resolution**: 640x480 or higher recommended
- **Frame rate**: 15-30 fps sufficient

## Color Calibration

If your lane markings have different colors or lighting conditions vary:

1. Capture sample images: `ros2 run image_view image_saver image:=/camera/image_raw`
2. Open in image editor and check HSV values
3. Adjust ranges in code:
   ```python
   # For white lines
   lower_white = np.array([H_min, S_min, V_min])
   upper_white = np.array([H_max, S_max, V_max])
   
   # For yellow lines
   lower_yellow = np.array([H_min, S_min, V_min])
   upper_yellow = np.array([H_max, S_max, V_max])
   ```

## Testing Checklist

- [ ] Robot stays in lane on straight sections
- [ ] Robot follows gentle curves smoothly
- [ ] Robot follows sharp turns without crossing solid line
- [ ] Robot never crosses solid white line
- [ ] Robot can cross dashed yellow line if needed
- [ ] Robot stops when lane markings are lost
- [ ] Debug visualization shows correct line detection
- [ ] Behavior works at different speeds (0.1 - 0.3 m/s)
- [ ] Behavior works under room lighting conditions
- [ ] Robot recovers if temporarily loses one line

## Performance Tips

1. **Start slow**: Begin with 0.15 m/s and gradually increase
2. **Tune incrementally**: Change one parameter at a time
3. **Use debug view**: Always verify vision is working correctly
4. **Test systematically**: Straight → gentle curves → sharp turns
5. **Document settings**: Keep track of working parameter values

## Files Overview

- `lane_follower.py` - Main ROS2 node implementation
- `lane_follower_launch.py` - Launch file
- `lane_follower_params.yaml` - Parameter configuration
- `README.md` - This file

## License

This code is provided for educational purposes as part of the autonomous robotics course project.

## Support

For issues or questions:
1. Check debug visualization first
2. Review troubleshooting section
3. Verify parameter values are reasonable
4. Test vision processing in isolation
5. Consult with team members or instructor

Good luck with your project! 🤖
