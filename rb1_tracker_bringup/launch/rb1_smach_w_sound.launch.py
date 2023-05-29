from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rb1_tracker_bu_share = get_package_share_directory('rb1_tracker_bringup')
    nav2_share = get_package_share_directory('tiago_navigation')
    asus_xtion_share = get_package_share_directory('asus_xtion')
    
    # Node
    asus_camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.05', '0.0', '0.0', '0.0', 'xtion_link', 'asus_xtion_link'],
    )
    
    audio_tts_node = Node(
        package='audio_common',
        executable='tts_node'
    )
    
    audio_player_node = Node(
        package='audio_common',
        executable='audio_player_node'
    )
    
    state_machine_node = Node(
        package='rb1_tracker_smach',
        executable='tracking_smach.py'
    )
    
    # Launch
    yolo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rb1_tracker_bu_share, 'launch', 'yolo.launch.py')
        ),
        launch_arguments={
            'image_topic': '/camera/rgb/image_raw',
            'yolo_weights': os.path.join(rb1_tracker_bu_share, 'net_props', 'yolov8s.pt'),
        }.items()
    )
    
    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'tiago_nav2.launch.py')
        )
    )
    
    asus_xtion_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(asus_xtion_share, 'launch', 'asus_xtion.launch.py')
        )
    )
    
    ld = LaunchDescription()
    
    ld.add_action(asus_xtion_launch_cmd)
    ld.add_action(yolo_launch_cmd)
    ld.add_action(navigation_launch_cmd)
    
    ld.add_action(asus_camera_tf_node)
    ld.add_action(audio_tts_node)
    ld.add_action(audio_player_node)
    ld.add_action(state_machine_node)
    
    return ld