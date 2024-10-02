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
    airsim_pkg_name = 'airsim_ros_pkgs'
    airsim_launch_file_name = 'airsim_node.launch.py'
    #launch airsim_camera_frame_node
    airsim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(airsim_pkg_name),
                'launch',
                airsim_launch_file_name
            ])
        )
    )
    
    launch_file_name = 'launch_single_camera_frame.launch.py'
    base_frame = 'SimpleFlight/odom_local'
    static_cam_frame_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), 
                                       '/'+launch_file_name]),
        launch_arguments={
            'x': '0.1',
            'y': '0.0',
            'z': '0.0',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
            'parent_frame': base_frame,
            'child_frame': 'camera_frame_1',
            'node_name': 'static_transform_publisher_1'
        }.items()
    )
    
    pitch_rot = str(np.deg2rad(90.0))
    static_cam_frame_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), 
                                       '/'+launch_file_name]),
        launch_arguments={
            'x': '0.0',
            'y': '0.0',
            'z': '0.5',
            'roll': '0.0',
            'pitch': pitch_rot,
            'yaw': '0.0',
            'parent_frame': base_frame,
            'child_frame': 'camera_frame_2',
            'node_name': 'static_transform_publisher_2'
        }.items()
    )
    
    
    return LaunchDescription([
        static_cam_frame_1,
        static_cam_frame_2,
        airsim_launch
    ])