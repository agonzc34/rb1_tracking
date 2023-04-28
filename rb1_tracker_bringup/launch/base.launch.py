from launch import LaunchDescription
from launch.actions import Node, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    yolo_package_share = get_package_share_directory('ros_yolov8')
    
    yolo_config_files = os.path.join(yolo_package_share, 'net_props')
    
    # Arguments
    yolo_weights = LaunchConfiguration('yolo_weights')
    yolo_weights_cmd = DeclareLaunchArgument(
        'yolo_weights',
        default_value=os.path.join(yolo_config_files, 'yolov8m.pt'),
        description='YOLO model to use'
    )
    
    debug_yolo = LaunchConfiguration('debug')
    debug_yolo_cmd = DeclareLaunchArgument(
        'debug',
        default_value='True',
        description='Enable debug mode'
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
        default_value='/camera/image_raw',
        description='Image topic'
    )
    
    cloud_point_topic = LaunchConfiguration('cloud_point_topic')
    cloud_point_topic_cmd = DeclareLaunchArgument(
        'cloud_point_topic',
        default_value='/camera/points',
        description='Cloud point topic'
    )
    
    camera_frame = LaunchConfiguration('camera_frame')
    camera_frame_cmd = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_link',
        description='Camera frame'
    )
    
    # Nodes
    yolo_node_params = [('yolo_weights', yolo_weights), ('debug', debug_yolo), ('tracker_yaml', tracker_yaml), ('image_topic', image_topic)]
    yolo_node = Node(
        name='yolov8_node',
        package='ros_yolov8',
        executable='ros_yolov8',
        arguments=yolo_node_params,
    )
    
    tracking_node_params = [('image_topic', image_topic), ('cloud_point_topic', cloud_point_topic), ('camera_frame', camera_frame)]
    tracking_node = Node(
        name='rb1_tracker_node',
        package='rb1_tracker',
        executable='rb1_tracker',
        arguments=tracking_node_params
    )
    

    
    ld = LaunchDescription()
    
    ld.add_action(yolo_weights_cmd)
    ld.add_action(debug_yolo_cmd)
    ld.add_action(tracker_yaml_cmd)
    ld.add_action(image_topic_cmd)
    ld.add_action(cloud_point_topic_cmd)
    ld.add_action(camera_frame_cmd)
    
    ld.add_action(yolo_node)
    ld.add_action(tracking_node)
    
    return ld
        