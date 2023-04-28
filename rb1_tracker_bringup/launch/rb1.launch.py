from launch import LaunchDescription
from launch.actions import Node, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bridge_share = get_package_share_directory('tiago_camera')
    nav2_share = get_package_share_directory('tiago_navigation')
    rb1_tracker_share = get_package_share_directory('rb1_tracker')
        
    # Arguments
    
    # Launch
    tracker_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rb1_tracker_share, 'launch', 'tracking.launch.py')
        ),
        launch_arguments={
            'exec_name': 'rb1_tracker',
            'image_topic': '/xtion/rgb/image_raw/repub',
            'cloud_point_topic': '/xtion/depth/points/repub',
            'camera_frame': 'xtion_link'
        }.items()
    )
    
    yolo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rb1_tracker_share, 'launch', 'yolo.launch.py')
        ), 
        launch_arguments={
            'image_topic': '/xtion/rgb/image_raw/repub',
        }.items()
    )
    
    repub_images_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, 'launch', 'republish.launch.py')
        )
    )
    
    repub_pc_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, 'launch', 'pc.launch.py')
        )
    )
    
    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'tiago_nav2.launch.py') # Necesitaría poder cambiar el archivo de configuración
        )
    )
    
    
    ld = LaunchDescription()
    
    ld.add_action(repub_images_launch_cmd)
    ld.add_action(repub_pc_launch_cmd)
    ld.add_action(navigation_launch_cmd)
    ld.add_action(yolo_launch_cmd)
    ld.add_action(tracker_launch_cmd)
    
    return ld
        