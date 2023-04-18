#!/bin/python3

import cv2 as cv
import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber, TimeSynchronizer

import rclpy.logging
import struct

from sensor_msgs.msg import Image, PointCloud2
from yolo_msgs.msg import BoundingBoxes, BoundingBox
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from builtin_interfaces.msg import Duration


class TrackingPublisher(Node):
    
    def __init__(self):
        super().__init__('tracking_node')
                
        self.image_sub_ = Subscriber(self, Image, "/camera/image_raw")
        self.point_cloud_sub_ = Subscriber(self, PointCloud2, "/camera/points")

        self.topic_sync = TimeSynchronizer([self.image_sub_, self.point_cloud_sub_], 10)
        self.topic_sync.registerCallback(self.track_callback)

        self.marker_pub_ = self.create_publisher(MarkerArray, '/visualization_marker', 10)
        self.goal_pose_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

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
        
        self.id = 0
                
                    
    def track_callback(self, image: Image, point_cloud: PointCloud2):
        cv_image = self.cv_bridge.imgmsg_to_cv2(image)
        
        if self.init_tracker is False:
            roi = cv.selectROI('Tracking', cv_image)
            self.tracker.init(cv_image, roi)
            self.init_tracker = True
        
        
        success, box = self.tracker.update(cv_image)

        if success:
            x, y, w, h = [int(v) for v in box]
            cv.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            
            obj_center_in_img = (x + w/2, y + h/2)
            obj_center_to_img_center = (obj_center_in_img[0] - self.camera['image_width']/2, obj_center_in_img[1] - self.camera['image_height']/2)
            
            cv.circle(cv_image, (int(obj_center_in_img[0]), int(obj_center_in_img[1])), 5, (0, 0, 255), -1)
            
            obj_to_center_prop = (-obj_center_to_img_center[0] / self.camera['image_width'], -obj_center_to_img_center[1] / self.camera['image_height']) # -0.5 a 0.5
            
            max_horizontal = np.tan(self.camera['horizontal_fov'] / 2) * self.camera['far_clip']
            max_vertical = np.tan(self.camera['vertical_fov'] / 2) * self.camera['far_clip']
                        
            horizontal_pos = None
            vertical_pos = None
            distance = None
            
            distance_img = self.camera['far_clip']
            horizontal_img = max_horizontal * obj_to_center_prop[0] * 2
            vertical_img = max_vertical * obj_to_center_prop[1] * 2
                        
            horizontal_angle = np.arctan(horizontal_img / distance_img)
            vertical_angle = np.arctan(vertical_img / distance_img)
            
            print('horizontal_angle: {}, vertical_angle: {}'.format(horizontal_angle, vertical_angle))
            print('selected point: {}, {}, {}'.format(horizontal_img, vertical_img, distance_img))
            
            point_cloud = self.transfromPointCloud(point_cloud)
            estimated_point = None
            
            min_err = 1
            
            for point in point_cloud:
                if abs(point[2] - self.camera['far_clip']) < 0.0001:
                    continue
                err_x = abs(point[2] * np.tan(horizontal_angle) + point[0])
                err_y = abs(point[2] * np.tan(vertical_angle) + point[1])
                err = pow(err_x, 2) + pow(err_y, 2)
                
                # print('err_x: {}, err_y: {}'.format(err_x, err_y))
                
                if err < min_err:
                    min_err = err
                    estimated_point = point
                    print('found possible point')
                    # break
            
            if estimated_point is not None:
                horizontal_pos = -estimated_point[0]
                vertical_pos = -estimated_point[1]
                distance = estimated_point[2]
                           
                print("estimated: {}, y: {}, z: {}".format(horizontal_pos, vertical_pos, distance))
                self.debug_line(image, horizontal_img, vertical_img, distance_img, horizontal_pos, vertical_pos, distance)
                
                selected_point = (horizontal_img, vertical_img, distance)
                
                
                

        cv.imshow('Tracking', cv_image)
        cv.waitKey(1)
        
        self.id += 1


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
            
            # if rbg == 0.0:
            #     pass
            point_list.append([x, y, z, rbg])
            print('x: {}, y: {}, z: {}, rbg: {}'.format(x, y, z, rbg))
        
        # max_x = max(point_list, key=lambda x: x[0])[0]
        # max_y = max(point_list, key=lambda x: x[1])[1]
        # min_x = min(point_list, key=lambda x: x[0])[0]
        # min_y = min(point_list, key=lambda x: x[1])[1]
        
        # print('Maximos y minimos')
        # print(max_x, max_y, min_x, min_y)
        
        return point_list
    
    def debug_line(self, image: Image, horizontal_pos_img: float, vertical_pos_img: float, distance_img: float, horizontal_pos_cloud: float, vertical_pos_cloud: float, distance_cloud: float):
        marker_array = MarkerArray()
        marker_array.markers = []
        
        marker_line = Marker()
        marker_line.header.frame_id = 'camera_link'
        marker_line.header.stamp = image.header.stamp
        marker_line.ns = 'rb1_debug_tracker'
        
        marker_line.type = Marker.LINE_STRIP
        marker_line.action = Marker.ADD
        marker_line.frame_locked = True
        
        marker_line.color.a = 1.0
        marker_line.scale.x = 0.01
        marker_line.lifetime = Duration(sec=1, nanosec=0)
        
        robot_point = Point()
        end_point = Point()
        
        robot_point.x = 0.0
        robot_point.y = 0.0
        robot_point.z = 0.0
        
        end_point.x = distance_img
        end_point.y = horizontal_pos_img
        end_point.z = vertical_pos_img
        
        marker_line.points.append(robot_point)
        marker_line.points.append(end_point)
        
        marker_array.markers.append(marker_line)
        
        marker_point = Marker()
        marker_point.header.frame_id = 'camera_link'
        marker_point.header.stamp = image.header.stamp
        marker_point.ns = 'rb1_debug_tracker_p'
        
        marker_point.type = Marker.SPHERE
        marker_point.action = Marker.ADD
        marker_point.frame_locked = True
        
        marker_point.color.a = 1.0
        marker_point.color.r = 1.0
        
        marker_point.scale.x = 0.1
        marker_point.scale.y = 0.1
        marker_point.scale.z = 0.1
        
        marker_point.pose.position.x = distance_cloud
        marker_point.pose.position.y = horizontal_pos_cloud
        marker_point.pose.position.z = vertical_pos_cloud
        
        marker_array.markers.append(marker_point)
        
        # top_left_point = self.draw_point(1, image, 2.3, -1.0, 3.0)
        # marker_array.markers.append(top_left_point)
        
        # alternative_point = self.draw_point(2, image, horizontal_pos_img, vertical_pos_img, distance_cloud)
        # marker_array.markers.append(alternative_point)
        
        self.marker_pub_.publish(marker_array)
        
    def draw_point(self, id, image: Image, horizontal_pos: float, vertical_pos: float, distance: float):
        marker_point = Marker()
        marker_point.header.frame_id = 'camera_link'
        marker_point.header.stamp = image.header.stamp
        marker_point.ns = 'rb1_debug_tracker ' + str(id)
        
        marker_point.type = Marker.SPHERE
        marker_point.action = Marker.ADD
        marker_point.frame_locked = True
        marker_point.lifetime = Duration(sec=1, nanosec=0)
        
        marker_point.color.a = 1.0
        marker_point.color.b = 1.0
        
        marker_point.scale.x = 0.1
        marker_point.scale.y = 0.1
        marker_point.scale.z = 0.1
        
        marker_point.pose.position.x = distance
        marker_point.pose.position.y = -horizontal_pos
        marker_point.pose.position.z = -vertical_pos
        
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