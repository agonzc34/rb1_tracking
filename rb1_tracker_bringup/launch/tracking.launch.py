from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
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
    
    exec_name = LaunchConfiguration('exec_name')
    exec_name_cmd = DeclareLaunchArgument(
        'exec_name',
        default_value='rb1_tracker_noyolo',
        description='Executable name'
    )
    
    # Nodes
    tracking_params = [{'image_topic': image_topic}, {'cloud_point_topic': cloud_point_topic}, {'camera_frame': camera_frame}]
    
    tracking_node = Node(
        name='rb1_tracker_node',
        package='rb1_tracker',
        executable=exec_name,
        parameters=tracking_params
    )
    

    
    ld = LaunchDescription()
    
    ld.add_action(image_topic_cmd)
    ld.add_action(cloud_point_topic_cmd)
    ld.add_action(camera_frame_cmd)
    ld.add_action(exec_name_cmd)
    
    ld.add_action(tracking_node)
    
    return ld
        