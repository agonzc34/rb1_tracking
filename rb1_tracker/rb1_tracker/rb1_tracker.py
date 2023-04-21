#!/bin/python3

import cv2 as cv
import cv_bridge
import numpy as np
import rclpy
from rclpy.time import Time as rclpy_time
from rclpy.node import Node
from message_filters import Subscriber, TimeSynchronizer

import rclpy.logging
import struct

from sensor_msgs.msg import Image, PointCloud2
from yolo_msgs.msg import BoundingBoxes, BoundingBox
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from tf2_geometry_msgs import PointStamped as PointStamped_tf
from builtin_interfaces.msg import Duration, Time

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TrackingPublisher(Node):
    def __init__(self):
        super().__init__('tracking_node')
                
        self.image_sub_ = Subscriber(self, Image, "/camera/image_raw")
        self.point_cloud_sub_ = Subscriber(self, PointCloud2, "/camera/points")
        self.bounding_box_sub_ = Subscriber(self, BoundingBoxes, "/obj_rec/bounding_boxes")

        self.topic_sync = TimeSynchronizer([self.image_sub_, self.point_cloud_sub_, self.bounding_box_sub_], 10)
        self.topic_sync.registerCallback(self.track_callback)

        self.marker_pub_ = self.create_publisher(MarkerArray, '/visualization_marker', 10)
        self.goal_pose_pub_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.tracker = cv.TrackerCSRT.create()
        self.cv_bridge = cv_bridge.CvBridge()
        self.init_tracker = False
        
        self.camera = {
            'horizontal_fov': 1.3439,
            'near_clip': 0.05,
            'far_clip': 3.0,
            'image_width': 720,
            'image_height': 480
        }
        
        self.camera['image_ratio'] = self.camera['image_width'] / self.camera['image_height']
        self.camera['vertical_fov'] = self.camera['horizontal_fov'] / self.camera['image_ratio']
        
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self)
                
        self.marker_array = MarkerArray()
        
        self.id = 0
                
                    
    def track_callback(self, image: Image, point_cloud: PointCloud2, bounding_boxes: BoundingBoxes):
        bounding_box = bounding_boxes.bounding_boxes[0]

        x = bounding_box.xmin
        y = bounding_box.ymin
        w = bounding_box.xmax - bounding_box.xmin
        h = bounding_box.ymax - bounding_box.ymin
        
        cv_image = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")
        
        cv.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
        
        obj_center_in_img = (x + w/2, y + h/2)
        obj_center_to_img_center = (obj_center_in_img[0] - self.camera['image_width']/2, obj_center_in_img[1] - self.camera['image_height']/2)
        
        cv.circle(cv_image, (int(obj_center_in_img[0]), int(obj_center_in_img[1])), 5, (0, 0, 255), -1)
        
        obj_to_center_prop = (-obj_center_to_img_center[0] / self.camera['image_width'], -obj_center_to_img_center[1] / self.camera['image_height']) # -0.5 a 0.5
        
        max_horizontal = np.tan(self.camera['horizontal_fov'] / 2) * self.camera['far_clip']
        max_vertical = np.tan(self.camera['vertical_fov'] / 2) * self.camera['far_clip']
        
        distance_img = self.camera['far_clip']
        horizontal_img = max_horizontal * obj_to_center_prop[0] * 2
        vertical_img = max_vertical * obj_to_center_prop[1] * 2
        
        image_line_debug = self.debug_line('camera_link', image.header.stamp, 'image_line', 0, 0.0, 0.0, 0.0, distance_img, horizontal_img, vertical_img, 0.0, 0.0, 0.0, 1.0, 0.01, 1)
        self.marker_array.markers.append(image_line_debug)
                    
        horizontal_angle = np.arctan(horizontal_img / distance_img)
        vertical_angle = np.arctan(vertical_img / distance_img)
                    
        point_cloud = self.transfromPointCloud(point_cloud)
        estimated_point = None
        
        min_err = 1
        
        for point in point_cloud:
            if abs(point[2] - self.camera['far_clip']) < 0.0001:
                continue
            err_x = abs(point[2] * np.tan(horizontal_angle) + point[0])
            err_y = abs(point[2] * np.tan(vertical_angle) + point[1])
            err = pow(err_x, 2) + pow(err_y, 2)
                            
            if err < min_err:  # Se puede cambiar por un umbral y detener el bucle para mejorar rendimiento. Ademas, hay que gestionar si no se encuentra el punto dentro de la nube de puntos
                min_err = err
                estimated_point = point
                # break
        
        horizontal_pos = -estimated_point[0]
        vertical_pos = -estimated_point[1]
        distance = estimated_point[2]
                    
        print("estimated: {}, y: {}, z: {}".format(horizontal_pos, vertical_pos, distance))
        
        selected_point = (horizontal_img, vertical_img, distance)
        spatial_point_debug = self.debug_point('camera_link', image.header.stamp, 'spatial_point', 1, distance, horizontal_pos, vertical_pos, r=1.0, g=0.0, b=0.0, a=1.0, scale=0.1, seconds=1)
        self.marker_array.markers.append(spatial_point_debug)
        
        try:
            # transform = self.tf_buffer.lookup_transform('camera_link', 'map', rclpy.time.Time(), rclpy.duration.Duration(seconds=2))
            # print(transform)
        
            #add selected point to transform
            selected_point_transformable = PointStamped_tf()

            
            selected_point_transformable.header.frame_id = 'camera_link'
            selected_point_transformable.header.stamp = Time(sec=0)
            selected_point_transformable.point.x = distance 
            selected_point_transformable.point.y = horizontal_pos 
            selected_point_transformable.point.z = vertical_pos
            
            selected_point_t = self.tf_buffer.transform(selected_point_transformable, 'map', new_type=PointStamped_tf)
            selected_point = (selected_point_t.point.x, selected_point_t.point.y, selected_point_t.point.z)
            
            print('selected_point abs: {}'.format(selected_point))
            
            destination_point_abs_debug = self.debug_point('map', image.header.stamp, 'destination_point_abs', 2, selected_point[0], selected_point[1], selected_point[2], 0.0, 1.0, 0.0, 1.0, 0.1, 1)
            self.marker_array.markers.append(destination_point_abs_debug)
            
            destionation_point = PoseStamped()
            destionation_point.header.frame_id = 'map'
            destionation_point.header.stamp = image.header.stamp
            destionation_point.pose.position.x = selected_point[0]
            destionation_point.pose.position.y = selected_point[1]
            destionation_point.pose.position.z = selected_point[2]
            
            destionation_point.pose.orientation.x = 0.0
            destionation_point.pose.orientation.y = 0.0
            destionation_point.pose.orientation.z = 0.0
            destionation_point.pose.orientation.w = 1.0
            
            self.goal_pose_pub_.publish(destionation_point)
            
        except Exception as e:
            print(e)
            

        cv.imshow('Tracking', cv_image)
        cv.waitKey(1)
        
        self.id += 1
        self.marker_pub_.publish(self.marker_array)


    def transfromPointCloud(self, pointcloud: PointCloud2):
        point_list = []
        
        is_bigendian = pointcloud.is_bigendian
        point_size = pointcloud.point_step
        
        data_format = ""
        
        if is_bigendian:
            data_format = ">f"
        else:
            data_format = "<f"
        
        for i in range(0, len(pointcloud.data), point_size):
            x = struct.unpack_from(data_format, pointcloud.data, i)[0]
            y = struct.unpack_from(data_format, pointcloud.data, i + 4)[0]
            z = struct.unpack_from(data_format, pointcloud.data, i + 8)[0]
            rbg = struct.unpack_from(data_format, pointcloud.data, i + 16)[0]
            
            if z == 3.0:
                continue
            
            point_list.append([x, y, z])
            # print('x: {}, y: {}, z: {}'.format(x, y, z))
        
        # max_x = max(point_list, key=lambda x: x[0])[0]
        # max_y = max(point_list, key=lambda x: x[1])[1]
        # min_x = min(point_list, key=lambda x: x[0])[0]
        # min_y = min(point_list, key=lambda x: x[1])[1]
        
        # print('Maximos y minimos')
        # print(max_x, max_y, min_x, min_y)
        
        return point_list
    
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