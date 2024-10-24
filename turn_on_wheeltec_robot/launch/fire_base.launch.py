from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
import launch_ros.actions

#def launch(launch_descriptor, argv):
def generate_launch_description():


    
            
#the default mode is not akm
    launch_ros.actions.Node(
        package='turn_on_wheeltec_robot', 
        executable='odom_vel_to_dis.py', 
        output='screen',
        )

