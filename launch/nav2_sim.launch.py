import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction # Added TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'calixto-ros-bot'

    # Define paths
    nav2_params_path = os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml')
    map_path = os.path.join(get_package_share_directory(package_name), 'maps', 'my_map.yaml')
    world_path = os.path.join(get_package_share_directory(package_name), 'gazebo', 'maze.world')
    rviz_config_path = os.path.join(get_package_share_directory(package_name), 'rviz', 'rviz_slam.rviz')
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')

    # 1. Launch the Simulation (RSP, Gazebo, and Spawn)
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'launch_sim.launch.py'
        )]), launch_arguments={'world': world_path}.items()
    )

    # 2. Twist Mux Node
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # 3. Nav2 Bringup (Includes Localization, Planning, and Control)
    # Note: bringup_launch.py includes localization_launch.py internally
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'
        )]), launch_arguments={
            'map': map_path,
            'use_sim_time': 'true',
            'params_file': nav2_params_path,
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    # 4. RViz2 with a slight delay to ensure Nav2 is up
    rviz2 = TimerAction(
    period=5.0,
    actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ]
)

    return LaunchDescription([
        launch_sim,
        twist_mux,
        nav2_bringup,
        rviz2
    ])