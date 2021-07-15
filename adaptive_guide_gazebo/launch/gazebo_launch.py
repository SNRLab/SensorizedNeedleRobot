import os
from posixpath import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  urdf_file_name = 'models/adaptive_guide/adaptive_guide.urdf'

  print("urdf_file_name : {}".format(urdf_file_name))

  urdf = os.path.join(
      get_package_share_directory('adaptive_guide_description'),
      urdf_file_name)

  pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

  return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        #ExecuteProcess(
        #    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        #    output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
                )
            ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
            ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "cam_bot"])
  ])