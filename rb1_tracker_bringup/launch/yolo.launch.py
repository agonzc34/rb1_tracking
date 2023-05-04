from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    yolo_package_share = get_package_share_directory('ros_yolov8')
    
    yolo_config_files = os.path.join(yolo_package_share, 'net_props')
    
    # Arguments
    debug_yolo = LaunchConfiguration('debug')
    debug_yolo_cmd = DeclareLaunchArgument(
        'debug',
        default_value='True',
        description='Enable debug mode'
    )
    
    yolo_weights = LaunchConfiguration('yolo_weights')
    yolo_weights_cmd = DeclareLaunchArgument(
        'yolo_weights',
        default_value=os.path.join(yolo_config_files, 'yolov8m.pt'),
        description='YOLO model to use'
    )
        
    
    tracker_yaml = LaunchConfiguration('tracker_yaml')
    tracker_yaml_cmd = DeclareLaunchArgument(
        'tracker_yaml',
        default_value=os.path.join(yolo_config_files, 'bytetrack.yaml'),
        description='Tracker YAML file'
    )
    
    image_topic = LaunchConfiguration('image_topic')
    image_topic_cmd = DeclareLaunchArgument(
        'image_topic',
        default_value='/head_front_camera/rgb/image_raw',
        description='Image topic'
    )
    
    yolo_params = [{'debug': debug_yolo}, {'tracker_yaml': tracker_yaml}, {'yolo_weights': yolo_weights}, {'image_topic': image_topic}]
    
    # Nodes
    yolo_node = Node(
        name='yolov8_node',
        package='ros_yolov8',
        executable='ros_yolov8',
        parameters=yolo_params
    )
    
    ld = LaunchDescription()
    
    ld.add_action(yolo_weights_cmd)
    ld.add_action(debug_yolo_cmd)
    ld.add_action(tracker_yaml_cmd)
    ld.add_action(image_topic_cmd)
    
    ld.add_action(yolo_node)
    
    return ld
        