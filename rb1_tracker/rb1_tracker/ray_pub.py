import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Transform, Pose, TransformStamped, Point
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Duration
import numpy as np
import quaternion


class RayPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, '/visualization_marker', 10)
        self.robot_position = self.create_subscription(TFMessage, '/tf', self.robot_position_callback, 10)
        
    def robot_position_callback(self, msg: TFMessage):
        for obj in msg.transforms:
            if obj.child_frame_id == 'base_footprint':
                robot_transform = obj.transform
                
                marker_line = Marker()
                marker_line.id = 1
                
                
                marker_line.header.frame_id = 'base_footprint'
                marker_line.header.stamp = self.get_clock().now().to_msg()
                marker_line.ns = 'ray'
                marker_line.action = Marker.ADD
                marker_line.type = Marker.LINE_STRIP
                
                marker_line.scale.x = 0.1
                
                marker_line.color.a = 1.0
                marker_line.color.r = 1.0
                
                marker_line.frame_locked = True
                
                robot_point = Point()
                
                robot_point.x = 0.0
                robot_point.y = 0.0
                robot_point.z = 0.0
                
                distance = 10
                
                end_point = Point()
                
                robot_orientation = quaternion.from_float_array([robot_transform.rotation.x, robot_transform.rotation.y, robot_transform.rotation.z, robot_transform.rotation.w])
                
                # Quaternion to Euler
                robot_euler = quaternion.as_euler_angles(robot_orientation)
                self.get_logger().info('I heard: "%s"' % robot_euler)
                end_point.x = distance * np.cos(robot_euler[1])
                end_point.y = distance * np.sin(robot_euler[1])
                end_point.z = 0.0
                                
                marker_line.points.append(robot_point)
                marker_line.points.append(end_point)
                
                marker_array = MarkerArray()
                marker_array.markers.append(marker_line)                
                self.publisher_.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    ray_pub = RayPublisher()

    rclpy.spin(ray_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ray_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()