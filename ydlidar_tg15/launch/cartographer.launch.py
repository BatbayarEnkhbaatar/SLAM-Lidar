import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():

    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    rviz_config_file = os.path.join(share_dir, 'config', 'ydlidar.rviz')

    # LiDAR parameters
    ydlidar_tg15_param = LaunchConfiguration(
        "ydlidar_tg15_param",
        default=os.path.join(
            get_package_share_directory("ydlidar_tg15"),
            "param",
            "ydlidar_tg15_param.yaml",
        ),
    )

    # LiDAR driver node
    lidar_driver_node = LifecycleNode(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[ydlidar_tg15_param],
    )

    # Static transform publisher for LiDAR TF
    lidar_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_laser",
        arguments=["0", "0", "0.02", "0", "0", "0", "1", "base_link", "laser_frame"],
    )

    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(get_package_share_directory('ydlidar_tg15'), 'config')
    )

    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='mapping2d.lua'
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename],
        remappings=[('/scan', '/scan')],
    )

    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
        
    return LaunchDescription([lidar_driver_node, lidar_tf_node, cartographer_node, rviz2_node])
