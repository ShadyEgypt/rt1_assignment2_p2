# Robot Controller Node

## Overview
This ROS2 Python node (`robot_controller`) integrates a publisher and a subscriber to control a robot's movement based on its odometry data. The node listens to `/odom` for the robot's position and publishes velocity commands to `/cmd_vel` to control its linear and angular velocities.

### Node Functionality
1. **Subscriber:**
   - Subscribes to the `/odom` topic to get the robot's current x-position.
   - Logs the x-position to monitor movement.

2. **Publisher:**
   - Publishes velocity commands to `/cmd_vel`.
   - Adjusts linear and angular velocities based on the robot's x-position.

### Behavior
- **x > 7.0:** Turn Left (Linear: 1.0, Angular: 1.0)
- **x < -7.0:** Turn Right (Linear: 1.0, Angular: -1.0)
- **-7.0 <= x <= 7.0:** Move Straight (Linear: 1.0, Angular: 0.0)

## Launch File
To launch the node, the following launch file was created (`launch.py`):
This launch file launches the urdf_robot package which is a dependency for this node to run correctly.

### Launch Command
Run the launch file with:
```bash
ros2 launch rt1_assignment2_p2 launch.py
```
If this command didn't work as expected, then you need to run:
```bash
cd src/rt1_assignment2_p2/launch
ros2 launch launch.py
```
## Common Errors & Solutions

### Error: urdf_robot package failed to start
```bash
Unable to create the rendering window when running gazebo
```
- **Issue:** The launch file was not working due to problems with the graphics driver, causing Gazebo to fail to render the environment.
- **Solution:** The graphics driver was updated/reconfigured to ensure compatibility with Gazebo.

## How to Run the Node Manually
1. Build the package:
   ```bash
   colcon build
   ```
2. Source the workspace:
   ```bash
   source install/setup.bash
   ```
3. Run the node directly:
   ```bash
   ros2 run rt1_assignment2_p2 controller_node
   ```
   
## Author
- **Shady Abdelmalek**
