import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')

    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    mapping_mode = LaunchConfiguration('mapping_mode', default='slam')

    slam_mode = IfCondition(PythonExpression(["'", mapping_mode, "' == 'slam'"]))
    simple_mode = IfCondition(PythonExpression(["'", mapping_mode, "' == 'simple'"]))

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_mapping_mode = DeclareLaunchArgument(
        'mapping_mode',
        default_value='slam',
        description='Mapping mode to use: slam or simple'
    )

    # ---- SLAM Toolbox (Lifecycle Node in Jazzy) ----
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=slam_mode,
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
        condition=slam_mode,
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 0.0,  # Disable bond for standalone SLAM
        }],
    )

    # ---- Simple Mapper ----
    simple_mapper_node = Node(
        package='coverage_planner',
        executable='simple_mapper',
        name='simple_mapper',
        output='screen',
        condition=simple_mode,
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
        declare_mapping_mode,
        slam_toolbox_node,
        lifecycle_manager,
        simple_mapper_node,
        teleop_node,
        rviz_node,
    ])
