from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    config_file = "/home/justin/Documents/Unreal Projects/TestUE5/Source/data.conf"
    # ===========================
    # Node 1 Argument Declarations
    # ===========================
    file_path_1_arg = DeclareLaunchArgument(
        'file_path_1',
        default_value=config_file,
        description='Shared memory config file path for Node 1'
    )
    
    project_id_1_arg = DeclareLaunchArgument(
        'project_id_1',
        default_value='1',
        description='Project ID for Node 1'
    )
    
    image_topic_1_arg = DeclareLaunchArgument(
        'image_topic_1',
        default_value='/camera/image_raw_1',
        description='Image topic for Node 1'
    )
    
    # ===========================
    # Node 2 Argument Declarations
    # ===========================
    file_path_2_arg = DeclareLaunchArgument(
        'file_path_2',
        default_value=config_file,
        description='Shared memory config file path for Node 2'
    )
    
    project_id_2_arg = DeclareLaunchArgument(
        'project_id_2',
        default_value='2',
        description='Project ID for Node 2'
    )
    
    image_topic_2_arg = DeclareLaunchArgument(
        'image_topic_2',
        default_value='/camera/image_raw_2',
        description='Image topic for Node 2'
    )
    
    # ===========================
    # Node 1 Launch Configuration
    # ===========================
    node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/launch_shared_memory.launch.py']),
        launch_arguments={
            'file_path': LaunchConfiguration('file_path_1'),
            'project_id': LaunchConfiguration('project_id_1'),
            'image_topic': LaunchConfiguration('image_topic_1'),
            'node_name': 'ue_image_publisher_1'
        }.items()
    )
    
    # ===========================
    # Node 2 Launch Configuration
    # ===========================
    node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/launch_shared_memory.launch.py']),
        launch_arguments={
            'file_path': LaunchConfiguration('file_path_2'),
            'project_id': LaunchConfiguration('project_id_2'),
            'image_topic': LaunchConfiguration('image_topic_2'),
            'node_name': 'ue_image_publisher_2'
        }.items()
    )
    
    # ===========================
    # Return Full Launch Description
    # ===========================
    return LaunchDescription([
        # Arguments for Node 1
        file_path_1_arg,
        project_id_1_arg,
        image_topic_1_arg,
        
        # Arguments for Node 2
        file_path_2_arg,
        project_id_2_arg,
        image_topic_2_arg,
        
        # Launch Node 1 and Node 2
        node_1,
        node_2
    ])
