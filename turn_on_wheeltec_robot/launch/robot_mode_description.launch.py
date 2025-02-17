import os
from pathlib import Path
import launch_ros.actions
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,LogInfo,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration


def generate_robot_node(robot_urdf):
    return launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'), 'urdf', robot_urdf)],
    )

def generate_static_transform_publisher_node(translation, rotation, parent, child):
    return launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'base_to_{child}',
        arguments=[translation[0], translation[1], translation[2], rotation[0], rotation[1], rotation[2], parent, child],
    )
    
def generate_launch_description():
    


    
    # mini_4wd = GroupAction([
    #         generate_robot_node('mini_mec_robot.urdf'),
    #         generate_static_transform_publisher_node(['0.435', '-0.3285', '0.2'], ['-1.5707', '3.1415926', '0'], 'base_footprint', 'laser'),
    #         generate_static_transform_publisher_node(['0.0', '0', '0.25'], ['0', '0', '0'], 'base_footprint', 'camera_link'),
    # ])
    mini_4wd_two_laser = GroupAction([
            generate_robot_node('mini_mec_robot.urdf'),
            generate_static_transform_publisher_node(['0.435', '-0.3285', '0.2'], ['0', '0', '0'], 'base_footprint', 'laser_2'),#front
            # generate_static_transform_publisher_node(['-0.435', '0.3285', '0.2'], ['1.5707', '3.1415926', '0'], 'base_footprint', 'laser_1'),#back
            
        #     generate_static_transform_publisher_node(['0.0', '0', '0.25'], ['0', '0', '0'], 'base_footprint', 'camera_link'),
    ])                


    # Create the launch description and populate
    ld = LaunchDescription()
    # ld.add_action(mini_4wd)
    ld.add_action(mini_4wd_two_laser)
    return ld

