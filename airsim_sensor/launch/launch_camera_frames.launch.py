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
    
    launch_file_name = 'single_camera_frame.launch.py'
    camera_position_launch_file_name = 'single_camera_position.launch.py'
    
    
    base_frame = 'SimpleFlight/odom_local'
    pawn_1 = 'AirsimMovePawn_1'
    x_offset = str(0.5)
    static_cam_frame_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), 
                                       '/'+launch_file_name]),
        launch_arguments={
            'x': '0.0',
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
    position_cam_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), 
                                       '/' + camera_position_launch_file_name]),
        launch_arguments={
            'airsim_camera_frame': 'camera_frame_1',
            'airsim_move_pawn_name': pawn_1,
            'world_frame': 'world',
            'dt': '0.01',
            'ned_offset_x': x_offset,
            'ned_offset_y': '0.0',
            'ned_offset_z': '0.0',
            'node_name': 'camera_position_publisher_1'
        }.items()
    )
    
    
    pitch_rot = str(np.deg2rad(90.0))
    z = str(0.5)
    static_cam_frame_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), 
                                       '/'+launch_file_name]),
        launch_arguments={
            'x': '0.0',
            'y': '0.0',
            'z': z,
            'roll': '0.0',
            'pitch': pitch_rot,
            'yaw': '0.0',
            'parent_frame': base_frame,
            'child_frame': 'camera_frame_2',
            'node_name': 'static_transform_publisher_2'
        }.items()
    )
    
    position_cam_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), 
                                       '/' + camera_position_launch_file_name]),
        launch_arguments={
            'airsim_camera_frame': 'camera_frame_2',
            'airsim_move_pawn_name': 'AirsimMovePawn_2',
            'world_frame': 'world',
            'dt': '0.01',
            'ned_offset_x': '0.0',
            'ned_offset_y': '0.0',
            'ned_offset_z': z,
            'node_name': 'camera_position_publisher_2',
            'use_pitch_rot': 'True',
            'pitch_rot_deg': '-90.0'
        }.items()
    )
    
    
    return LaunchDescription([
        static_cam_frame_1,
        static_cam_frame_2,
        position_cam_1,
        position_cam_2,
        airsim_launch
    ])