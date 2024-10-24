from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
    laser_dir = get_package_share_directory('ros2_laser_scan_merger')
    merge_dir = os.path.join(laser_dir, 'launch')

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )
    wheeltec_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_lidar.launch.py')),
    )

    return LaunchDescription([
        wheeltec_robot,wheeltec_lidar,
        launch_ros.actions.Node(
        	parameters=[
        		get_package_share_directory("wheeltec_slam_toolbox") + '/config/mapper_params_online_async.yaml'
        	],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            remappings=[('odom', 'odom_combined'), ('scan', '/lidar_merge_scan')]

        )
    ])
