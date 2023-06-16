from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rb1_tracker_share = get_package_share_directory('rb1_tracker')
    yolo_share = get_package_share_directory('ros_yolov8')
    #asus_xtion_share = get_package_share_directory('asus_xtion')
    nav2_share = get_package_share_directory('tiago_navigation')
         
    # Node
    # asus_camera_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0.0', '0.0', '0.05', '0.0', '0.0', '0.0', 'xtion_link', 'asus_xtion_link'],
    # )
   
    # Launch
    tracker_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rb1_tracker_share, 'launch', 'tracker.launch.py')
        ),
        launch_arguments={
            'exec_name': 'rb1_tracker',
            'image_topic': '/camera/rgb/image_raw',
            'cloud_point_topic': '/camera/depth/points',
            'camera_frame': 'camera_rgb_optical_frame'
        }.items()
    )
    
    yolo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_share, 'launch', 'yolov8.launch.py')
        ), 
        launch_arguments={
            'image_topic': '/camera/rgb/image_raw',
            'yolo_weights': os.path.join(rb1_tracker_share, 'net_props', 'yolov8s.pt'),
        }.items()
    )
    
    # asus_xtion_launch_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(asus_xtion_share, 'launch', 'asus_xtion.launch.py')
    #     )
    # )
    
    # navigation_launch_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav2_share, 'launch', 'tiago_nav2.launch.py')
    #     )
    # )
    
    
    ld = LaunchDescription()
    
    #ld.add_action(asus_xtion_launch_cmd)
    ld.add_action(yolo_launch_cmd)
    ld.add_action(tracker_launch_cmd)
    #ld.add_action(asus_camera_tf_node)
    #ld.add_action(navigation_launch_cmd)
    
    return ld
        