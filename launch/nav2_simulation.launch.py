import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    calixto_dir  = get_package_share_directory("calixto-ros-bot")
    slam_dir     = get_package_share_directory("slam_toolbox")
    nav2_dir     = get_package_share_directory("nav2_bringup")

    step1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(calixto_dir, "launch", "launch_sim.launch.py")
        ),
        launch_arguments={
            "world": "src/calixto-ros-bot/gazebo/maze.world"
        }.items(),
    )

    step2 = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_dir, "launch", "online_async_launch.py")
                ),
                launch_arguments={
                    "slam_params_file": "src/calixto-ros-bot/config/mapper_params_online_async.yaml",
                    "use_sim_time": "true",
                }.items(),
            )
        ],
    )

    step3 = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_dir, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                }.items(),
            )
        ],
    )

    return [step1, step2, step3]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
