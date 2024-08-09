from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description for a iris quadcopter."""
    pkg_project_bringup = get_package_share_directory("drone_mission")
    pkg_project_bringup_ai_copter = get_package_share_directory('ai_copter')
    # Iris.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ai_copter"),
                        "launch",
                        "drone_camera.launch.py",
                    ]
                ),
            ]
        )
    )
    detection = LaunchDescription([
        Node(
            package='drone_mission', 
            executable='detection', 
            name='detection',
            output='screen',  
        ),
    ])
    camera = LaunchDescription([
        Node(
            package='drone_mission', 
            executable='camera', 
            name='camera',
            output='screen',  
        ),
    ])
    return LaunchDescription(
        [
            sim,
            detection,
            camera,
        ]
    )
