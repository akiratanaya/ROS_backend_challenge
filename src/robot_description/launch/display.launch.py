import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    
    # Lokasi file URDF/Xacro dan RViz config
    xacro_file = os.path.join(pkg_share, 'urdf', 'turtlebot_custom.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'robot_description.rviz')

    # Konversi Xacro ke XML mentah
    robot_description_content = Command(['xacro ', xacro_file])

    # Node 1: Robot State Publisher (Penyebar struktur robot ke sistem)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }]
    )

    # Node 2: Joint State Publisher GUI (Memunculkan slider kecil untuk menggerakkan roda secara manual)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Node 3: RViz2 (Visualisasi 3D)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])
