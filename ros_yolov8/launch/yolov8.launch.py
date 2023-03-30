
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    model = LaunchConfiguration("model")
    model_cmd = DeclareLaunchArgument(
        "model",
        default_value="yolov8m.pt",
        description="Model name or path")

    device = LaunchConfiguration("device")
    device_cmd = DeclareLaunchArgument(
        "device",
        default_value="cuda:0",
        description="Device to use (GPU/CPU)")

    debug = LaunchConfiguration("debug")
    debug_cmd = DeclareLaunchArgument(
        "debug",
        default_value="True",
        description="Enable debug mode")

    #
    # NODES
    #
    detector_node_cmd = Node(
        package="ros_yolov8",
        executable="ros_yolov8",
        name="ros_yolov8",
        parameters=[{"model": model,
                     "device": device,
                     "debug": debug,
                    }],)

    ld = LaunchDescription()

    ld.add_action(model_cmd)
    ld.add_action(device_cmd)
    ld.add_action(debug_cmd)
    
    ld.add_action(detector_node_cmd)

    return ld