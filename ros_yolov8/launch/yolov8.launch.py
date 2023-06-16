
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("ros_yolov8")
    
    yolo_weights = LaunchConfiguration('yolo_weights')
    image_topic = LaunchConfiguration('image_topic')
    tracker_yaml = LaunchConfiguration('tracker_yaml')
    debug = LaunchConfiguration('debug')
    
    yolo_weights_cmd = DeclareLaunchArgument(
        'yolo_weights',
        default_value=os.path.join(pkg_dir, 'net_props', 'yolov8s.pt'),
        description='Yolo weights file'
    )
    
    image_topic_cmd = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/rgb/image_raw',
        description='Image topic'
    )
    
    tracker_yaml_cmd = DeclareLaunchArgument(
        'tracker_yaml',
        default_value='bytetrack.yaml',
        description='Tracker YAML file'
    )
    
    debug_cmd = DeclareLaunchArgument(
        'debug',
        default_value='False',
        description='Debug mode'
    )

    #
    # NODES
    #
    detector_node_cmd = Node(
        package="ros_yolov8",
        executable="ros_yolov8",
        name="ros_yolov8",
        parameters=[{
                'yolo_weights': yolo_weights,
                'image_topic': image_topic,
                'tracker_yaml': tracker_yaml,
                'debug': debug
            }]
        )

    ld = LaunchDescription()

    ld.add_action(debug_cmd)
    ld.add_action(yolo_weights_cmd)
    ld.add_action(image_topic_cmd)
    ld.add_action(tracker_yaml_cmd)
    
    ld.add_action(detector_node_cmd)

    return ld