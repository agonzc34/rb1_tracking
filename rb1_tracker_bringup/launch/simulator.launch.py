from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    simulator_share = get_package_share_directory('rb1_gazebo')
    rb1_tracker_share = get_package_share_directory('rb1_tracker')
    yolo_share = get_package_share_directory('ros_yolov8')
            
    # Launch
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_share, 'launch', 'noyolo.launch.py')
        )
    )
    
    tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rb1_tracker_share, 'launch', 'simulator.launch.py')
        )
    )
    
    
    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulator_share, 'launch', 'granny.launch.py')
        )
    )
    
    
    ld = LaunchDescription()
    
    ld.add_action(simulator_launch)
    ld.add_action(yolo_launch)
    ld.add_action(tracking_launch)
    
    return ld
        