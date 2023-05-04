from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    simulator_share = get_package_share_directory('rb1_gazebo')
    rb1_tracker_share = get_package_share_directory('rb1_tracker_bringup')
            
    # Launch
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rb1_tracker_share, 'launch', 'yolo.launch.py')
        )
    )
    
    tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rb1_tracker_share, 'launch', 'tracking.launch.py')
        ),
        launch_arguments={
            'exec_name': 'rb1_tracker_noyolo',
            'image_topic': '/camera/image_raw'
        }.items()
    )
    
    
    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulator_share, 'launch', 'granny.launch.py')
        )
    )
    
    
    ld = LaunchDescription()
    
    ld.add_action(simulator_launch)
    # ld.add_action(yolo_launch)
    ld.add_action(tracking_launch)
    
    return ld
        