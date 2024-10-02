# parametric_static_transform_launch_with_node_name.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    """
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument('x', default_value='1.0', description='X coordinate of the transform'),
        DeclareLaunchArgument('y', default_value='0.0', description='Y coordinate of the transform'),
        DeclareLaunchArgument('z', default_value='0.0', description='Z coordinate of the transform'),
        DeclareLaunchArgument('roll', default_value='0.0', description='Roll angle (in radians)'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='Pitch angle (in radians)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw angle (in radians)'),
        DeclareLaunchArgument('parent_frame', default_value='world', description='Parent frame id'),
        DeclareLaunchArgument('child_frame', default_value='robot_base', description='Child frame id'),
        DeclareLaunchArgument('node_name', default_value='static_transform_publisher', description='Name of the node'),

        # Define the node and pass the parameters as arguments
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=LaunchConfiguration('node_name'),  # Set the node name dynamically
            output='screen',
            arguments=[
                LaunchConfiguration('x'),
                LaunchConfiguration('y'),
                LaunchConfiguration('z'),
                LaunchConfiguration('roll'),
                LaunchConfiguration('pitch'),
                LaunchConfiguration('yaw'),
                LaunchConfiguration('parent_frame'),
                LaunchConfiguration('child_frame')
            ]
        ),
    ])
