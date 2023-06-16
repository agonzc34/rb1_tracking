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
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Duration, Time

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class NoYoloNode(Node):
    
    def __init__(self):
        super().__init__('noyolo_node')
        
        self.declare_parameter('image_topic', '/head_front_camera/rgb/image_raw')
        
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.get_logger().info("Image topic: " + self.image_topic)
        
        self.bounding_boxes_pub_ = self.create_publisher(BoundingBoxes, "/obj_rec/bounding_boxes", qos_profile_sensor_data)
        self.yolo_debug_pub_ = self.create_publisher(Image, "/obj_rec/yolo", qos_profile_sensor_data)
        
        self.camera_read_sub_ = self.create_subscription(Image, self.image_topic, self.camera_read_callback, qos_profile_sensor_data)
        
        self.cv_bridge = cv_bridge.CvBridge()
        
        self.last_person_id = None
        self.frame_id = 0
        self.init_tracker = False
        
        self.tracker = cv.TrackerCSRT_create()
                
                    
    def camera_read_callback(self, image: Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(image)
        bounding_boxes = BoundingBoxes()
        
        bounding_boxes.header.frame_id = str(self.frame_id)
        bounding_boxes.header.stamp = image.header.stamp
        
        bounding_box = BoundingBox()
        
        if self.init_tracker is False:
            roi = cv.selectROI('Tracking', cv_image)
            self.tracker.init(cv_image, roi)
            self.init_tracker = True
        
        success, box = self.tracker.update(cv_image)

        if success:
            x, y, w, h = [int(v) for v in box]
            cv.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            bounding_box.xmin = x
            bounding_box.ymin = y
            bounding_box.xmax = x + w
            bounding_box.ymax = y + h
            bounding_box.probability = 1.0
            bounding_box.class_name = 'person'
            bounding_box.id = 0
            
            bounding_boxes.bounding_boxes.append(bounding_box)

        cv.imshow('Tracking', cv_image)
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Quit')
            self.init_tracker = False
        
        self.bounding_boxes_pub_.publish(bounding_boxes)
        self.yolo_debug_pub_.publish(self.cv_bridge.cv2_to_imgmsg(cv_image))
        
        self.frame_id += 1

      

def main(args=None):
    rclpy.init(args=args)

    yolo_publisher = NoYoloNode()

    rclpy.spin(yolo_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    yolo_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()