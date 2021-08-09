# Stage Control
Package containing nodes for controlling the emulated, virtual, and physical robot. Control of the robot is implemented through the ```/move_stage``` action server.\
The stage control nodes can be launched using the following command:
```bash
ros2 launch stage_control stage_control_launch.py sim_level:=<0,1,2,3>
```

The *sim_level* argument controls whether Gazebo simulation, physical hardware, or both will be used:
- *sim_level:=0* : Emulated (dummy nodes) stage and sensors only
- *sim_level:=1* : Virtual stage and sensors, simulated in Gazebo (Not yet fully implemented)
- *sim_level:=2* : Physical stage and sensors (Depth and rotation sensors currently only emulated)
- *sim_level:=3* : Both virtual and physical sensors

## Parameters
The *stage_config.yaml* has the following parameters that are loaded when launched:
- **simulated_latency**: A delay in milliseconds between when the hardware and the virtual stage acts upon a movement action. The hardware has a communication delay when sending commands to the controller, so a simulated lag is applied to the virtual stage to keep the two systems synchronized when the *sim_level* argument is 3.
- **sync_vel_compensation**: An adjustment, in meters per second, applied to the virtual stage's velocity when a global velocity is set. The hardware stage tends to move slightly slower than the virtual stage likely due to some frictional effects unaccounted for in Gazebo. This adjustment is applied when *sim_level* is 3 to keep the two systems synchronized.

## Topics
- */stage/global_velocity*: A std_msgs/msg/Float64 message to set the velocity of both the hardware and virtual stage, or whichever one is active.


## Physical Stage

### Installation
To be able to connect to the NSC-A2L motor controller with a Linux system, some setup is required. Connecting to the controller via USB requires elevated user privileges. Run these commands in the terminal to enable USB communications with the controller:
1. Add a new user group: ```addgroup --system usb```
2. Add user to group: ```sudo adduser $USER usb```
3. Navigate to permission rules folder: ```cd /etc/udev/rules.d```
4. Create new rules file: ```sudo gedit 70-usb-arcus.rules```
5. Add this line to the new file: *SUBSYSTEM=="usb", ATTR{idVendor}=="1589", ATTR{idProduct}=="a101", GROUP="usb", MODE="0666"*
6. If necessary, ensure hex values match by running command: ```lsusb```
7. Reload rules: ```sudo udevadm control --reload-rules```
8. ```sudo udevadm trigger```

### Usage
The stage should be primarily controlled via the ```/move_stage``` action server, however the ```/stage/controller/command``` service is available for more specific commands. Using this service, commands can be sent directly to the controller following the communication protocol in Section 9 of the NSC-2AL user manual.

### Parameters
The hardware_config.yaml has the following parameters that are loaded when launched:
- **velocity**: The default x-z velocity, in meters per second, set to the stage when it is launched. It can further be changed either via the global velocity topic, or its own velocity topic.

### Topics
- */stage/velocity*: A std_msgs/msg/Float64 message to set the velocity of the hardware stage in meters per second.

## Virtual Stage
The virtual stage is a simulation of the physical stage done in the Gazebo physics environment.

### Parameters
The virtual_config.yaml has the following parameters that are loaded when launched:
- **velocity**: The default x-z velocity, in meters per second, set to the stage when it is launched. It can further be changed either via the global velocity topic, or its own velocity topic.
 - **insertion_vel**: The default needle insertion velocity in meters per second. Controls the default velocity of the needle in the y direction.
 - **rotation_vel**: The default needle rotation velocity in radians per second. controls the default angular velocity of the needle about the y axis.

### Topics
- */virtual_stage/velocity*: A std_msgs/msg/Float64 message to set the velocity of the virtual stage in meters per second.
- */virtual_stage/insertion_velocity*: A std_msgs/msg/Float64 message to set the needle insertion velocity of the virtual stage in meters per second.
- */virtual_stage/rotation_velocity*: A std_msgs/msg/Float64 message to set the needle rotation velocity of the virtual stage in radians per second.
- */virtual_stage/joint_states/<x,y,z,theta>*: A std_msgs/msg/Float64 topic publishing the current joint states of the virtual robot.
- */virtual_stage/y_position_controller/command*: A std_msgs/msg/Float64 topic to change the y (insertion) position of the needle in meters.
- */virtual_Stage/theta_position_controller/command*: A std_msgs/msg/Float64 topic to change the rotation of the needle about the y-axis in radians.