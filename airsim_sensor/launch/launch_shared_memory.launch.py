from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare command-line arguments for file_path, project_id, and topic_name
        DeclareLaunchArgument(
            'file_path',
            default_value='/home/justin/Documents/Unreal Projects/TestUE5/Source/data.conf',
            description='Path to the shared memory config file'
        ),
        DeclareLaunchArgument(
            'project_id',
            default_value='1',
            description='Project ID (token number) for shared memory'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='Topic name for published images'
        ),

        # Launch the node with parameters and topic remapping
        Node(
            package='airsim_sensor',  # Replace with your package name
            executable='shared_image_reader',  # Replace with your node executable
            name=LaunchConfiguration('node_name'),  # Node name remapping            output='screen',
            parameters=[
                {'file_path': LaunchConfiguration('file_path')},
                {'project_id': LaunchConfiguration('project_id')},
            ],
            # Remap the topic from the default '/camera/image_raw' to a user-defined topic
            remappings=[
                ('/camera/image_raw', LaunchConfiguration('image_topic'))
            ]
        ),
    ])
