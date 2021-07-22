# Sensorized Needle Robot Project
Shared repository for the sensorized needle robot collaboration between BWH and JHU.

## Installation
Clone repository into the src folder of a colcon workspace, and then run rosdep to install dependencies:
```bash
rosdep install --from-paths src
```
Then build:
```bash
colcon build --symlink-install
```

## Usage
Launch the adaptive_guide_launch.py file
```bash
ros2 launch adaptive_guide adaptive_guide_launch.py sim_level:=<0,1,2,3>
```
The *sim_level* argument controls whether Gazebo simulation, physical hardware, or both will be used:
- *sim_level:=0* : Emulated (dummy nodes) stage and sensors only
- *sim_level:=1* : Virtual stage and sensors, simulated in Gazebo (Not yet fully implemented)
- *sim_level:=2* : Physical stage and sensors (Depth and rotation sensors currently only emulated)
- *sim_level:=3* : Both virtual and physical sensors

## Motorized Stage Control - John & Hannah
Packages to provide actions to control the motorized stage hardware and virtual copy, as well as provide topics for stage position and needle pose.

Relevant packages:
- **adaptive_guide**: provides launch file to bring-up full system
- **adaptive_guide_description**: description files for virtual robot
- **adaptive_guide_gazebo**: launches robot in Gazebo simulation
- **needle_path_control**: provides controllers for virtual robot
- **needle_pose_sensors**: connects to needle pose sensors and publishes needle pose
- **stage_control**: provides action server to control motorized stage
- **stage_control_interfaces**: message, service, and action definitions for stage_control

### Interfacing
#### Motorized Stage Control for Compensation Algorithm
The stage control node exposes the */move_stage* action which takes a */stage_control_interfaces/action/MoveStage* action message of the format:
```
float64 x
float64 z
float64 eps
---
float64 x
float64 z
float64 time
float64 error
int32 error_code
---
float64 x
float64 z
float64 error
float64 time
```
**Goal**: *x* and *z* in the goal section represent the target position in millimeters for the motorized stage with respect to the home position of the controller. *eps* is the error in position in millimeters that the stage will try to achieve before stopping.\
**Result**: *x* and *z* in the result section represents the achieved *x* and *z* position. *time* is the number of seconds it took to reach the position. *error* is the final positional error. *error_code* is 0 if the movement succeeded. \
**Feedback**: *x* and *z* in the feedback section represents the current position at the time the message was published. *error* is the current error in position. *time* is the elapsed time in seconds.

#### Pose Topics
The following topics are also exposed:
- */needle/state/pose*: A geometry_msgs/msg/PoseStamped message that includes needle depth and rotation.
- */stage/state/pose*: A geometry_msgs/msg/PoseStamped message that includes the x and z position of the needle guide.
- */stage/state/needle_pose*: A geometry_msgs/msg/PoseStamped message that combines the needle and stage pose into one pose message.

#### Emulated Needle Control
The following topics can be published to in order to change the pose of the emulated needle.
- */needle/emulated/y*: A std_msgs/msg/Float64 topic to change the y (insertion) position of the needle.
- */needle/emulated/z*: A std_msgs/msg/Float64 topic to change the rotation of the needle about the y-axis.

### TO-DO
- Finish implementing virtual robot controls
- Ensure virtual and physical robot are consistent

