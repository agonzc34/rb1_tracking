from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    cloud_point_topic = LaunchConfiguration('point_cloud_topic')
    camera_frame = LaunchConfiguration('camera_frame')
    
    cloud_point_topic_cmd = DeclareLaunchArgument(
        'point_cloud_topic',
        default_value='/camera/points',
        description='Cloud point topic'
    )
    
    camera_frame_cmd = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_link',
        description='Camera frame'
    )
    
    # Nodes
    tracking_params = [{'point_cloud_topic': cloud_point_topic}, {'camera_frame': camera_frame}]
    
    tracking_node = Node(
        name='rb1_tracker_node',
        package='rb1_tracker',
        executable='rb1_tracker',
        parameters=tracking_params
    )
    

    ld = LaunchDescription()
    
    ld.add_action(cloud_point_topic_cmd)
    ld.add_action(camera_frame_cmd)
    
    ld.add_action(tracking_node)
    
    return ld
        