import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'calixto-ros-bot'

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.world'),
        description='Path to the Gazebo world file'
    )

    # 1. Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Gazebo (Permanently passing libgazebo_ros_factory.so)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={
            'extra_gazebo_args': '-s libgazebo_ros_factory.so',
            'world': LaunchConfiguration('world')
        }.items()
    )

    # 3. Spawn Entity (Wrapped in a Timer to wait 5 seconds for Gazebo to load)
    spawn_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'calixto_bot','-z', '0.1'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        world_arg,
        rsp,
        gazebo,
        spawn_entity,
    ])
