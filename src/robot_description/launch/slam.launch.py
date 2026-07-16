import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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

    # ---- SLAM Toolbox (Lifecycle Node in Jazzy) ----
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': True},
        ],
    )

    # ---- Lifecycle Manager ----
    # In ROS 2 Jazzy, slam_toolbox is a lifecycle node.
    # It starts in 'unconfigured' state and must be explicitly
    # configured + activated before it subscribes to /scan.
    # nav2_lifecycle_manager handles this automatically.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 0.0,  # Disable bond for standalone SLAM
        }],
    )

    # ---- Teleop Twist Keyboard ----
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
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
        lifecycle_manager,
        teleop_node,
        rviz_node,
    ])
