import os
from pathlib import Path
import launch_ros.actions
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, LogInfo,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
import xacro

def generate_robot_node(robot_urdf):
    pkg_path = os.path.join(get_package_share_directory('fire_robot_urdf'))
    xacro_file = os.path.join(pkg_path, 'description', robot_urdf)
    
    # 使用 xacro.process_file 來處理 Xacro 文件
    doc = xacro.process_file(xacro_file)
    robot_description = doc.toprettyxml(indent='  ')
    
    return launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

def generate_static_transform_publisher_node(translation, rotation, parent, child):
    return launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'base_to_{child}',
        arguments=[translation[0], translation[1], translation[2], rotation[0], rotation[1], rotation[2], parent, child],
    )
    
def generate_launch_description():
    mini_4wd = GroupAction([
            generate_robot_node('fire_robot.urdf.xacro'),
            generate_static_transform_publisher_node(['0.048', '0', '0.155'], ['0', '0', '0'], 'base_footprint', 'laser'),
            # generate_static_transform_publisher_node(['0.0', '0', '0.25'], ['0', '0', '0'], 'base_footprint', 'camera_link'),
    ])            

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(mini_4wd)
    return ld
