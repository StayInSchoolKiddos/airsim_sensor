from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import numpy as np
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    
    """
    
    launch_file_name = 'launch_single_camera_info.launch.py'
    cam1_info = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), 
                                       '/'+launch_file_name]),
        launch_arguments={
            'file_path': '/home/justin/Documents/Unreal Projects/TestUE5/Source/camera1_info.txt',
            'timer_period': '0.1',
            'info_topic': '/camera1/camera_info',
            'node_name': 'camera_info_publisher_1'
        }.items()
    )
    
    cam2_info = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), 
                                       '/'+launch_file_name]),
        launch_arguments={
            'file_path': '/home/justin/Documents/Unreal Projects/TestUE5/Source/camera2_info.txt',
            'timer_period': '0.1',
            'info_topic': '/camera2/camera_info',
            'node_name': 'camera_info_publisher_2'
        }.items()
    )
    
    
    return LaunchDescription([
        cam1_info,
        cam2_info
    ])