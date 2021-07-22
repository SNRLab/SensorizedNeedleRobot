from launch import LaunchDescription
from launch_ros.actions import Node

# Launch needle pose sensors
# Including needle depth and rotation

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="needle_pose_sensors",
            executable="emulate_sensors_node",
            name="needle_pose_sensors_node",
            output="screen"
        )
    ])