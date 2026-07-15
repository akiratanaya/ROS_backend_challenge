import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')

    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # ---- Simple Mapper (Bypassing SLAM Toolbox issue) ----
    slam_toolbox_node = Node(
        package='coverage_planner',
        executable='simple_mapper',
        name='simple_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )

    # ---- Teleop Twist Keyboard ----
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',   # opens in new terminal for keyboard input
        parameters=[{
            'use_sim_time': True,
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ],
    )

    # ---- RViz2 with SLAM config ----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        slam_toolbox_node,
        teleop_node,
        rviz_node,
    ])
