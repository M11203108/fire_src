import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

#     imu_pkg_dir = get_package_share_directory('realsense-ros')
#     imu_bringup_dir = get_package_share_directory(imu_pkg_dir, 'realsense2_camera')
#     imu_launch_dir = os.path.join(imu_bringup_dir, 'launch')

        
    ekf_config = Path(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'ekf.yaml')

    
    carto_slam = LaunchConfiguration('carto_slam', default='true')
    
    carto_slam_dec = DeclareLaunchArgument('carto_slam',default_value='false')
            

    choose_car = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
    )

#     open_imu = IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(imu_launch_dir, 'rs_launch.py')),
#     )
        
    robot_ekf = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_ekf.launch.py')),
            launch_arguments={'carto_slam':carto_slam}.items(),            
    )
                                                            
    base_to_link = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_link',
            arguments=['0', '0', '0','0', '0','0','base_footprint','base_link'],
    )
    base_to_gyro = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_gyro',
            arguments=['0', '0', '0','0', '0','0','base_footprint','gyro_link'],
    )
    
    
                           
    joint_state_publisher_node = launch_ros.actions.Node(
            package='joint_state_publisher', 
            executable='joint_state_publisher', 
            name='joint_state_publisher',
    )
    fire_base = launch_ros.actions.Node(
                package='turn_on_wheeltec_robot', 
                executable='odom_vel_to_dis.py', 
                name="odom_vel_to_dis",
                output='screen',
                parameters=[{
                        'usart_port_name_0': '/dev/ttyUSB0',
                        'usart_port_name_1': '/dev/ttyUSB1',
                        'serial_baud_rate': 115200,
                        'robot_frame_id': 'base_footprint',
                        'odom_frame_id': 'odom_combined',
                        'cmd_vel': 'cmd_vel',
                }],
                remappings=[('/cmd_vel', 'cmd_vel')],
                )


    ld = LaunchDescription()

    ld.add_action(choose_car)
    ld.add_action(carto_slam_dec)
#     ld.add_action(wheeltec_robot)
    ld.add_action(base_to_link)
    ld.add_action(base_to_gyro)
    ld.add_action(joint_state_publisher_node)
#     ld.add_action(open_imu)    
    ld.add_action(robot_ekf)
    ld.add_action(fire_base)

    return ld

