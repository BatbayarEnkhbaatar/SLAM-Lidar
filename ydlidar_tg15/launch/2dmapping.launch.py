from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'ydlidar_tg15'
    config_dir = os.path.join(get_package_share_directory(package_name), 'config')
    rviz_config_file = os.path.join(config_dir, 'ydlidar.rviz')  # Correct path to the RViz config file

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            output='screen',
            executable='cartographer_node',
            name='cr_node',
            arguments=['-configuration_directory', config_dir, '-configuration_basename', 'ydlidar_mapping2d.lua']
        ),
        Node(
            package='cartographer_ros',
            output='screen',
            executable='occupancy_grid_node',
            name='cr_oc_node',
            parameters=[{'resolution': 0.05, 'publish_period_sec': 1.0}]
        )
    ])
