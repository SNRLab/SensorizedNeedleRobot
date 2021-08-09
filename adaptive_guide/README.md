# Adaptive Guide
Top level package for starting up the full system.

Launch the adaptive_guide_launch.py file
```bash
ros2 launch adaptive_guide adaptive_guide_launch.py sim_level:=<0,1,2,3>
```
The *sim_level* argument controls whether Gazebo simulation, physical hardware, or both will be used:
- *sim_level:=0* : Emulated (dummy nodes) stage and sensors only
- *sim_level:=1* : Virtual stage and sensors, simulated in Gazebo (Not yet fully implemented)
- *sim_level:=2* : Physical stage and sensors (Depth and rotation sensors currently only emulated)
- *sim_level:=3* : Both virtual and physical sensors