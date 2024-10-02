from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare command-line arguments for file_path, project_id, and topic_name
        DeclareLaunchArgument(
            'airsim_camera_frame',
            default_value='camera_frame_1',
            description='Camera Frame'
        ),
        DeclareLaunchArgument(
            'airsim_move_pawn_name',
            default_value='AirsimMovePawn_1',
            description='AirsimMovePawn Unreal Object Name'
        ),
        DeclareLaunchArgument(
            'world_frame',
            default_value='world_frame',
            description='Info topic for published camera info'
        ),
        DeclareLaunchArgument(
            'dt',
            default_value='0.01',
            description='Frequency of the camera position publisher'
        ),
        DeclareLaunchArgument(
            'ned_offset_x',
            default_value='0.0',
            description='X offset of the camera frame in NED frame'
        ),
        DeclareLaunchArgument(
            'ned_offset_y',
            default_value='0.0',
            description='Y offset of the camera frame in NED frame'
        ),
        DeclareLaunchArgument(
            'ned_offset_z',
            default_value='0.0',
            description='Z offset of the camera frame in NED frame'
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='camera_position_publisher',
            description='Name of the node'
        ),
        # DeclareLaunchArgument(
        #     'topic_name',
        #     default_value='/camera/position',
        #     description='Name of the topic'
        # ),
        DeclareLaunchArgument(
            'use_pitch_rot',
            default_value='False',
            description='Use pitch rotation'
        ),
        DeclareLaunchArgument(
            'pitch_rot_deg',
            default_value='0.0',
            description='Pitch rotation in degrees'
        ),

        # Launch the node with parameters and topic remapping
        Node(
            package='airsim_sensor',  # Replace with your package name
            executable='airsim_camera_location_node.py',  # Replace with your node executable
            name=LaunchConfiguration('node_name'),  # Node name remapping            output='screen',
            parameters=[
                {'airsim_camera_frame': LaunchConfiguration('airsim_camera_frame')},
                {'airsim_move_pawn_name': LaunchConfiguration('airsim_move_pawn_name')},
                {'world_frame': LaunchConfiguration('world_frame')},
                {'dt': LaunchConfiguration('dt')},
                {'ned_offset_x': LaunchConfiguration('ned_offset_x')},
                {'ned_offset_y': LaunchConfiguration('ned_offset_y')},
                {'ned_offset_z': LaunchConfiguration('ned_offset_z')},
                {'use_pitch_rot': LaunchConfiguration('use_pitch_rot')},
                {'pitch_rot_deg': LaunchConfiguration('pitch_rot_deg')},
            ],
            # Remap the topic from the default '/camera/image_raw' to a user-defined topic
            
        ),
    ])
