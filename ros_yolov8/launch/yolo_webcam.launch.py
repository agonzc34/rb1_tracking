
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory("ros_yolov8")
    
    #
    # NODES
    #
    webcam_node_cmd = Node(
        package="ros_yolov8",
        executable="webcam",
        name="webcam"
    )
    
    #
    # Launch
    #
    yolo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'yolov8.launch.py'),
        ),
        launch_arguments={
            "model": 'yolov8m.pt',
            "device": 'cuda:0',
            "debug": 'True',
        }
    )

    ld = LaunchDescription()

    ld.add_action(yolo_launch_cmd)
    ld.add_action(webcam_node_cmd)

    return ld