import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for area coverage (Task 3).

    Prerequisites:
      1. Gazebo must be running (gazebo.launch.py)
      2. A saved map must exist from SLAM (Task 2)

    Usage:
      ros2 launch robot_description coverage.launch.py map:=/path/to/map.yaml
    """
    pkg_share = get_package_share_directory('robot_description')

    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )
    declare_map = DeclareLaunchArgument(
        'map', description='Full path to the map yaml file'
    )

    # Nav2 navigation stack
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'map': map_file,
            'autostart': 'true',
        }.items(),
    )

    # Coverage planner node
    coverage_node = Node(
        package='coverage_planner',
        executable='boustrophedon_planner',
        name='boustrophedon_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'sweep_spacing': 0.3,
            'robot_radius': 0.22,
        }],
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        nav2_launch,
        coverage_node,
        rviz_node,
    ])
