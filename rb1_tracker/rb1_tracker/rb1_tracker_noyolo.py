#!/bin/python3

import cv2 as cv
import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
import rclpy.logging


from sensor_msgs.msg import Image
from yolo_msgs.msg import BoundingBoxes, BoundingBox


class TrackingPublisher(Node):

    def __init__(self):
        super().__init__('tracking_node')
                
        self.image_sub_ = self.create_subscription(Image, "/camera/image_raw", self.track_callback, 10)
        self.marker_pub_ = self.create_publisher(Marker, '/visualization_marker', 10)

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
        
        self.id = 0
                
                    
    def track_callback(self, image: Image):
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
            distance = 2.0
            
            cv.circle(cv_image, (int(obj_center_in_img[0]), int(obj_center_in_img[1])), 5, (0, 0, 255), -1)
            
            horizontal_angle = obj_center_to_img_center[0] / self.camera['image_width'] * self.camera['horizontal_fov']
            end_x = distance * np.sin(horizontal_angle)
            
            
            vertical_angle = obj_center_to_img_center[1] / self.camera['image_height'] * self.camera['horizontal_fov'] * self.camera['image_height'] / self.camera['image_width']
            end_y = distance * np.sin(vertical_angle)
            
                        
            marker = Marker()
            marker.header.frame_id = 'camera_link'
            marker.header.stamp = image.header.stamp
            marker.ns = 'rb1_debug_tracker'
            
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.frame_locked = True
            
            marker.color.a = 1.0
            marker.scale.x = 0.01
            marker.lifetime = Duration(sec=1, nanosec=0)
            
            robot_point = Point()
            end_point = Point()
            
            robot_point.x = 0.0
            robot_point.y = 0.0
            robot_point.z = 0.0
            
            end_point.x = distance
            end_point.y = -end_x
            end_point.z = -end_y
            
            marker.points.append(robot_point)
            marker.points.append(end_point)
            
            self.marker_pub_.publish(marker)            

        cv.imshow('Tracking', cv_image)
        cv.waitKey(1)
        
        self.id += 1


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