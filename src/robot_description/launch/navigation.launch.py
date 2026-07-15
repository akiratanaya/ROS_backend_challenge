import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        description='Full path to the map yaml file to load'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    # Rewrite params with map file and sim time
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file,
        },
        convert_types=True,
    )

    # Nav2 bringup (includes all nav2 nodes)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
        }.items(),
    )

    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
    )

    # Lifecycle manager for map server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server'],
        }],
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_autostart,
        map_server,
        lifecycle_manager,
        nav2_bringup,
        rviz_node,
    ])
