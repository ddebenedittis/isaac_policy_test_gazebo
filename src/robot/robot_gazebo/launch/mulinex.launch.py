import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
        
    terrain = LaunchConfiguration('terrain', default='rigid')
    
    # ======================================================================== #
    
    launch_arguments = {
        'robot_name': 'mulinex',
        'package_name': 'mulinex_description',
        'robot_file_path': os.path.join('urdf', 'mulinex.xacro'),
        'world_file_path': os.path.join('worlds', 'mulinex.world'),
        'height': '0.05',
        'terrain': terrain,
    }.items()


    return LaunchDescription([
        
        DeclareLaunchArgument('terrain', default_value='rigid'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_gazebo'), 'launch', 'robot.launch.py')
            ),
            launch_arguments = launch_arguments,
        ),
    ])
