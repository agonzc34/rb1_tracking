#!/bin/python3

import cv2 as cv
import cv_bridge
import time
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

import rclpy.logging
import struct
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, PointCloud2
from yolo_msgs.msg import BoundingBoxes
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from tf2_geometry_msgs import PointStamped as PointStamped_tf
from builtin_interfaces.msg import Duration, Time

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TrackingPublisher(Node):
    def __init__(self):
        super().__init__('tracking_node')

        self.declare_parameter('image_topic', '/xtion/rgb/image_raw/repub')
        self.declare_parameter('point_cloud_topic', '/xtion/depth_registered/points')
        self.declare_parameter('camera_frame', 'xtion_link')
        
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.pc_topic = self.get_parameter('point_cloud_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        
                
        self.image_sub_ = Subscriber(self, Image, self.image_topic, qos_profile=qos_profile_sensor_data)
        self.point_cloud_sub_ = Subscriber(self, PointCloud2, self.pc_topic, qos_profile=qos_profile_sensor_data)
        self.bounding_box_sub_ = Subscriber(self, BoundingBoxes, "/obj_rec/bounding_boxes", qos_profile=qos_profile_sensor_data)
        
        self.camera_debug_ = self.create_publisher(Image, '/camera_debug', 10)

        self.topic_sync = ApproximateTimeSynchronizer([self.point_cloud_sub_, self.bounding_box_sub_], 100, 0.5)
        self.topic_sync.registerCallback(self.track_callback)

        self.marker_pub_ = self.create_publisher(MarkerArray, '/visualization_marker', 10)
        self.goal_pose_pub_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.cv_bridge = cv_bridge.CvBridge()
        self.init_tracker = False
        
        self.last_frame_time = time.time()
        self.curr_frame_time = time.time()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
                
        self.marker_array = MarkerArray()
                
        self.id = 0
                
                    
    def track_callback(self, point_cloud: PointCloud2, bounding_boxes: BoundingBoxes):
        self.get_logger().info("track_callback ready 2")
        self.curr_frame_time = time.time()
        
        bounding_box = bounding_boxes.bounding_boxes[0]

        x = bounding_box.xmin
        y = bounding_box.ymin
        w = bounding_box.xmax - bounding_box.xmin
        h = bounding_box.ymax - bounding_box.ymin
        
        # cv_image = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")
        obj_center_in_img = (x + w/2, y + h/2)

        # cv.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
        # cv.circle(cv_image, (int(obj_center_in_img[0]), int(obj_center_in_img[1])), 5, (0, 0, 255), -1)
          
        #Obtener la posición del objeto en el espacio gracias a la nube de puntos
        estimated_point = self.get_point_from_cloud(point_cloud, obj_center_in_img[0], obj_center_in_img[1])
        
        horizontal_pos = -estimated_point[0]
        vertical_pos = -estimated_point[1]
        distance = estimated_point[2]
        
        # self.get_logger().info("estimated: {}, y: {}, z: {}".format(horizontal_pos, vertical_pos, distance))
        
        spatial_point_debug = self.debug_point(self.camera_frame, point_cloud.header.stamp, 'spatial_point', 1, distance, horizontal_pos, vertical_pos, r=1.0, g=0.0, b=0.0, a=1.0, scale=0.1, seconds=1)
        self.marker_array.markers.append(spatial_point_debug)
        
        
        # Transformar el punto seleccionado a coordenadas del mapa
        try:
            # transform = self.tf_buffer.lookup_transform('camera_link', 'map', rclpy.time.Time(), rclpy.duration.Duration(seconds=2))
            # print(transform)
        
            #add selected point to transform
            selected_point_transformable = PointStamped_tf()

            
            selected_point_transformable.header.frame_id = self.camera_frame
            selected_point_transformable.header.stamp = point_cloud.header.stamp
            selected_point_transformable.point.x = distance 
            selected_point_transformable.point.y = horizontal_pos 
            selected_point_transformable.point.z = vertical_pos
            
            selected_point_t = self.tf_buffer.transform(selected_point_transformable, 'map', new_type=PointStamped_tf)
            selected_point = (selected_point_t.point.x, selected_point_t.point.y, selected_point_t.point.z)
            
            # print('selected_point abs: {}'.format(selected_point))
            self.get_logger().info("selected_point abs: {}".format(selected_point))
            
            destination_point_abs_debug = self.debug_point('map', point_cloud.header.stamp, 'destination_point_abs', 2, selected_point[0], selected_point[1], selected_point[2], 0.0, 1.0, 0.0, 1.0, 0.1, 5)
            self.marker_array.markers.append(destination_point_abs_debug)
            
            destionation_point = PoseStamped()
            destionation_point.header.frame_id = 'map'
            destionation_point.header.stamp = point_cloud.header.stamp
            destionation_point.pose.position.x = selected_point[0]
            destionation_point.pose.position.y = selected_point[1]
            destionation_point.pose.position.z = selected_point[2]
            
            destionation_point.pose.orientation.x = 0.0
            destionation_point.pose.orientation.y = 0.0
            destionation_point.pose.orientation.z = 0.0
            destionation_point.pose.orientation.w = 1.0
            
            self.goal_pose_pub_.publish(destionation_point)
            
        except Exception as e:
            # print(e)
            self.get_logger().warning("No se ha podido transformar el punto seleccionado a coordenadas del mapa")
            self.get_logger().warning(e)
            
        fps = 1 / (self.curr_frame_time - self.last_frame_time)
        # cv.putText(cv_image, "FPS: {:.2f}".format(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        # cv.imshow('Tracking', cv_image)
        # cv.waitKey(1)
        
        # image_debug = self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
        # self.camera_debug_.publish(image_debug)
        
        self.id += 1
        self.last_frame_time = self.curr_frame_time
        self.marker_pub_.publish(self.marker_array)


    def get_point_from_cloud(self, pointcloud: PointCloud2, x: int, y: int):
        x = int(x)
        y = int(y)
        is_bigendian = pointcloud.is_bigendian
        point_size = pointcloud.point_step
        row_step = pointcloud.row_step
        
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
        
        # top_left_point = self.draw_point(1, image, 2.3, -1.0, 3.0)
        # self.marker_array.markers.append(top_left_point)
        
        # alternative_point = self.draw_point(2, image, horizontal_pos_img, vertical_pos_img, distance_cloud)
        # self.marker_array.markers.append(alternative_point)
        
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

    yolo_publisher = TrackingPublisher()

    rclpy.spin(yolo_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    yolo_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()