import os
from posixpath import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression

pkg_stage_control = get_package_share_directory('stage_control')
pkg_needle_pose_sensors = get_package_share_directory('needle_pose_sensors')

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "sim_level",
            default_value="0",
            description="Simulation level: 0 - Emulation only, " +
                "1 - virtual only, 2 - hardware only, 3 - virtual and hardware"
        ),
        actions.LogInfo(msg=["Launching with sim level: ", LaunchConfiguration('sim_level')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_stage_control, 'launch', 'stage_control_launch.py')
                )
            , launch_arguments={'sim_level': LaunchConfiguration('sim_level')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_needle_pose_sensors, 'launch', 'needle_pose_sensors_launch.py')
                )
            ),
        Node(
            package="adaptive_guide",
            executable="stage_state_builder",
            name="stage_state_builder_node",
            output="screen"
        )
    ])