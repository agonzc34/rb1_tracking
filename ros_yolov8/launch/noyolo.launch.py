
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    
    image_topic_cmd = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Image topic'
    )

    #
    # NODES
    #
    detector_node_cmd = Node(
        package="ros_yolov8",
        executable="ros_noyolo",
        name="ros_yolov8",
        parameters=[{
                'image_topic': image_topic
            }]
        )

    ld = LaunchDescription()

    ld.add_action(image_topic_cmd)
    
    ld.add_action(detector_node_cmd)

    return ld