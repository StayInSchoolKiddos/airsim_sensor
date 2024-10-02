from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare command-line arguments for file_path, project_id, and topic_name
        DeclareLaunchArgument(
            'file_path',
            default_value='/home/justin/Documents/Unreal Projects/TestUE5/Source/camera1_info.txt',
            description='Path to the camera info file'
        ),
        DeclareLaunchArgument(
            'timer_period',
            default_value='0.1',
            description='Publish rate of the camera info'
        ),
        DeclareLaunchArgument(
            'info_topic',
            default_value='/camera1/camera_info',
            description='Info topic for published camera info'
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='camera_info_publisher',
            description='Name of the node'
        ),


        # Launch the node with parameters and topic remapping
        Node(
            package='airsim_sensor',  # Replace with your package name
            executable='airsim_camera_info_pub_node.py',  # Replace with your node executable
            name=LaunchConfiguration('node_name'),  # Node name remapping            output='screen',
            parameters=[
                {'file_path': LaunchConfiguration('file_path')},
                {'timer_period': LaunchConfiguration('timer_period')},
                {'info_topic': LaunchConfiguration('info_topic')},
            ],
            # Remap the topic from the default '/camera/image_raw' to a user-defined topic
            remappings=[
                ('/camera_info', LaunchConfiguration('info_topic'))
            ]
        ),
    ])
