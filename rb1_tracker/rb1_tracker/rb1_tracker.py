#!/bin/python3
import cv_bridge
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

import rclpy.logging
import struct
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
from yolo_msgs.msg import BoundingBoxes
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from tf2_geometry_msgs import PointStamped as PointStamped_tf, PoseStamped
from builtin_interfaces.msg import Duration

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TrackingNode(Node):
    def __init__(self, *args):
        super().__init__('tracking_node')
        
        # Parameters
        self.pc_topic = ""
        self.camera_frame = ""
        
        if (len(args) > 0):
            self.pc_topic = args[0]
            self.camera_frame = args[1]
        else:
            self.declare_parameter('point_cloud_topic', '/camera/depth_registered/points')
            self.declare_parameter('camera_frame', 'xtion_link')
            
            self.pc_topic = self.get_parameter('point_cloud_topic').get_parameter_value().string_value
            self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        
        self.get_logger().info("Point cloud topic: " + self.pc_topic)
        self.get_logger().info("Camera frame: " + self.camera_frame)


        # Variables
        self.cv_bridge = cv_bridge.CvBridge()
        self.init_tracker = False
        self.id = 0
        self.marker_array = MarkerArray()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)   


        # Publishers
        self.marker_pub_ = self.create_publisher(MarkerArray, '/visualization_marker', 10)
        self.goal_pose_pub_ = self.create_publisher(PoseStamped, '/goal_pose', 1)


        # Subscribers
        self.point_cloud_sub_ = Subscriber(self, PointCloud2, self.pc_topic, qos_profile=qos_profile_sensor_data)
        self.bounding_box_sub_ = Subscriber(self, BoundingBoxes, "/obj_rec/bounding_boxes", qos_profile=qos_profile_sensor_data)
        
        self.topic_sync = ApproximateTimeSynchronizer([self.point_cloud_sub_, self.bounding_box_sub_], 100, 0.5)
        self.topic_sync.registerCallback(self.track_callback)
        
                
                    
    def track_callback(self, point_cloud: PointCloud2, bounding_boxes: BoundingBoxes):
        self.get_logger().info("track_callback ready")
        
        if len(bounding_boxes.bounding_boxes) == 0:
            self.get_logger().warn("No bounding boxes")
            return
        
        bounding_box = bounding_boxes.bounding_boxes[0]

        x = bounding_box.xmin
        y = bounding_box.ymin
        w = bounding_box.xmax - bounding_box.xmin
        h = bounding_box.ymax - bounding_box.ymin
        
        obj_center_in_img = (x + w/2, y + h/2)
          
        #Obtener la posición del objeto en el espacio gracias a la nube de puntos
        rel_estimated_point = self.get_point_from_cloud(point_cloud, obj_center_in_img[0], obj_center_in_img[1])
        
        horizontal_pos = -rel_estimated_point[0]
        vertical_pos = -rel_estimated_point[1]
        distance = rel_estimated_point[2]
        
        spatial_point_debug = self.debug_point(self.camera_frame, point_cloud.header.stamp, 'spatial_point', 1, distance, horizontal_pos, vertical_pos, r=1.0, g=0.0, b=0.0, a=1.0, scale=0.1, seconds=1)
        self.marker_array.markers.append(spatial_point_debug)
        
        # Transformar el punto seleccionado a coordenadas del mapa
        try:
            #add selected point to transform
            selected_point_transformable = PoseStamped()            
            selected_point_transformable.header.frame_id = point_cloud.header.frame_id
            selected_point_transformable.header.stamp = point_cloud.header.stamp
            selected_point_transformable.pose.position.x = rel_estimated_point[0] 
            selected_point_transformable.pose.position.y = rel_estimated_point[1]
            selected_point_transformable.pose.position.z = rel_estimated_point[2]
            selected_point_transformable.pose.orientation.x = 0.0
            selected_point_transformable.pose.orientation.y = 0.0
            selected_point_transformable.pose.orientation.z = 0.0
            selected_point_transformable.pose.orientation.w = 1.0
            

            transform = self.tf_buffer.lookup_transform(
                "map",
                point_cloud.header.frame_id,
                rclpy.time.Time())
            do_transform = self.tf_buffer.registration.get(type(selected_point_transformable))
            abs_point:PoseStamped = do_transform(selected_point_transformable, transform)

            abs_point.pose.orientation.x = 0.0
            abs_point.pose.orientation.y = 0.0
            abs_point.pose.orientation.z = 0.0
            abs_point.pose.orientation.w = 1.0

            self.get_logger().info("selected_point abs: {}".format(abs_point))
            
            destination_point_abs_debug = self.debug_point('map', point_cloud.header.stamp, 'destination_point_abs', 2, abs_point.pose.position.x, abs_point.pose.position.y, abs_point.pose.position.z, 0.0, 1.0, 0.0, 1.0, 0.1, 5)
            self.marker_array.markers.append(destination_point_abs_debug)
            
            self.goal_pose_pub_.publish(abs_point)
            
        except Exception as e:
            self.get_logger().warning("No se ha podido transformar el punto seleccionado a coordenadas del mapa")
            self.get_logger().warning(str(e))
        
        self.id += 1
        self.marker_pub_.publish(self.marker_array)


    def get_point_from_cloud(self, pointcloud: PointCloud2, x: int, y: int):
        x = int(x)
        y = int(y)
        is_bigendian = pointcloud.is_bigendian
        point_size = pointcloud.point_step
        row_step = pointcloud.row_step
        
        if pointcloud.height == 1:
            row_step = 720 * point_size
        
        data_format = ""
        
        if is_bigendian:
            data_format = ">f"
        else:
            data_format = "<f"
            
        xp = struct.unpack_from(data_format, pointcloud.data, (y * row_step) + (x * point_size))[0]
        yp = struct.unpack_from(data_format, pointcloud.data, (y * row_step) + (x * point_size) + 4)[0]
        zp = struct.unpack_from(data_format, pointcloud.data, (y * row_step) + (x * point_size) + 8)[0]
        
        return [xp, yp, zp]
    
    def debug_line(self, frame_id, stamp, ns, id, x1, y1, z1, x2, y2, z2, r, g, b, a, scale, seconds):
        marker_line = Marker()
        marker_line.header.frame_id = frame_id
        marker_line.header.stamp = stamp
        marker_line.ns = ns
        marker_line.id = id
        
        marker_line.type = Marker.LINE_STRIP
        marker_line.action = Marker.ADD
        marker_line.frame_locked = True
        
        marker_line.color.a = a
        marker_line.scale.x = scale
        marker_line.color.r = r
        marker_line.color.g = g
        marker_line.color.b = b
        
        marker_line.lifetime = Duration(sec=seconds, nanosec=0)
        
        robot_point = Point()
        end_point = Point()
        
        robot_point.x = x1
        robot_point.y = y1
        robot_point.z = z1
        
        end_point.x = x2 # distance
        end_point.y = y2 # horizontal
        end_point.z = z2 # vertical
        
        marker_line.points.append(robot_point)
        marker_line.points.append(end_point)
        
        return marker_line

        
    def debug_point(self, frame_id, stamp, ns, id, x, y, z, r, g, b, a, scale, seconds):
        marker_point = Marker()
        marker_point.header.frame_id = frame_id
        marker_point.header.stamp = stamp
        marker_point.ns = ns
        marker_point.id = id
        
        marker_point.type = Marker.SPHERE
        marker_point.action = Marker.ADD
        marker_point.frame_locked = True
        marker_point.lifetime = Duration(sec=seconds, nanosec=0)
        
        marker_point.color.a = a
        marker_point.color.b = b
        marker_point.color.g = g
        marker_point.color.r = r
        
        marker_point.scale.x = scale
        marker_point.scale.y = scale
        marker_point.scale.z = scale
        
        marker_point.pose.position.x = x
        marker_point.pose.position.y = y
        marker_point.pose.position.z = z
        
        return marker_point        

def main(args=None):
    rclpy.init(args=args)

    yolo_publisher = TrackingNode()

    rclpy.spin(yolo_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    yolo_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()