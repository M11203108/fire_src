import os
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    Lslidar_dir = get_package_share_directory('lslidar_driver')
    Lslidar_launch_dir = os.path.join(Lslidar_dir, 'launch')
    
    rplidar_dir = get_package_share_directory('rplidar_ros')
    rplidar_launch_dir = os.path.join(rplidar_dir, 'launch')

    merge_dir = get_package_share_directory('ros2_laser_scan_merger')
    merge_launch_dir = os.path.join(merge_dir,'launch')

    a2m12_dir = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rplidar_launch_dir, 'rplidar_two_s2_launch.py'))

    )

    # merge_lidar = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(merge_launch_dir,'merge_2_scan.launch.py'))
    # )
       

    # Create the launch description and populate
    ld = LaunchDescription()
    '''
    Please select your lidar here, options include:
    Lsm10_m10p_uart、Lsm10_m10p_net、Lsm10_m10_uart、Lsm10_m10_net、Lsn10、Lsn10p,ld14、Ld06.
    1.If you are using LS* lidar (including lsn10, lsm10*), please don't forget to 
    modify the tf conversion parameters of robot_mode_description.launch.py
    according to the user guide file.
    2.If you are using m10 lidar, please pay attention to distinguish whether it is m10p or not.
    '''
    ld.add_action(a2m12_dir)
    # ld.add_action(merge_lidar)

    return ld

