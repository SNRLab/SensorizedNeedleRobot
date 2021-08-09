# Needle Path Control
Contains the configuration file for the controllers for the joints of the virtual robot. Each joint is controlled by a JointTrajectoryController, using the gazebo_ros2_control package. The following launch file loads up each joint's controller.
```bash
ros2 launch needle_path_control needle_position_launch.py
```