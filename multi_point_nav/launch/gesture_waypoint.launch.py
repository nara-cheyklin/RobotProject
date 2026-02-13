from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Path to turtlebot3_navigation2 launch file
    tb3_nav2_launch = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'launch',
        'navigation2.launch.py'
    )

    # Map file (edit if needed)
    map_yaml = os.path.expanduser('~/jjjmap.yaml')

    return LaunchDescription([

        # ─────────────── Nav2 ───────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_nav2_launch),
            launch_arguments={
                'map': map_yaml
            }.items()
        ),

        # ─────────────── Waypoint logic ───────────────
        Node(
            package='multi_point_nav',
            executable='multi_point_nav_node',
            name='multi_point_nav_node',
            output='screen',
        ),

        # ─────────────── Mediapipe gesture node ───────────────
        Node(
            package='multi_point_nav',
            executable='hand_gesture_node',
            name='hand_gesture_node',
            output='screen',
        ),
    ])
