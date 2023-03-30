import cv2
import numpy as np
import ultralytics
import rclpy
from rclpy.node import Node
import cv_bridge
import os
import ament_index_python.packages

from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator

from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from yolo_msgs.msg import BoundingBoxes, BoundingBox
from std_srvs.srv import SetBool


class YoloPublisher(Node):
    
    def __init__(self):
        super().__init__('yolo_node') # type: ignore
        
        self.set_parameters([
            self.declare_parameter('yolo_weights', 'yolov8m.pt'),
            self.declare_parameter('debug', True)
        ])
        
        yolo_weights = self.get_parameter('yolo_weights').get_parameter_value().string_value
        yolo_weights = os.path.join(ament_index_python.packages.get_package_share_directory('ros_yolov8'), 'net_props', yolo_weights)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        
        self.detecteion_image_pub_ = self.create_publisher(Image, '/obj_rec/detection_image', 10)
        self.counted_objects_pub_ = self.create_publisher(Int8, "/obj_rec/objects_counted", 10)
        self.bounding_boxes_pub_ = self.create_publisher(BoundingBoxes, "/obj_rec/bounding_boxes", 10)
        
        self.camera_read_sub_ = self.create_subscription(Image, "/head_front_camera/rgb/image_raw", self.camera_read_callback, 10)
        
        self.cv_bridge = cv_bridge.CvBridge()
        
        self.yolo = YOLO(yolo_weights)
        self.yolo.fuse()
        
        self.enabled = True
        
        self.create_service(SetBool, 'yolo_enable', self.enable_callback)
        
        self.frame_id = 0
    
    
    def enable_callback(self, request, response):
        self.enabled = request.data
        response.success = True
        return response
    
    
    def camera_read_callback(self, image):
        if self.enabled:
            counted_objects = Int8()
            output_image = Image()
            bounding_boxes = BoundingBoxes()
            
            counted_objects.data = 0
            
            bounding_boxes.header = image.header
            bounding_boxes.header.frame_id = str(self.frame_id)
            bounding_boxes.bounding_boxes = []
            
            cv_image = self.cv_bridge.imgmsg_to_cv2(image)
            results = self.yolo.predict(source=cv_image)
            
            for result in results:
                
                annotator = Annotator(cv_image)
                
                boxes = result.boxes
                
                for box in boxes:
                    box: ultralytics.yolo.engine.results.Boxes = box
                    
                    cls = box.cls[0]
                    xyxy = box.xyxy[0]
                    conf = box.conf[0]
                    
                    print(str.format("{} {} {}", cls, xyxy, conf))

                    annotator.box_label(xyxy, self.yolo.names[int(cls)])

                    # if conf < 0.5:
                    label = self.yolo.names[int(cls)]
                    # if label == 'person':
                    counted_objects.data += 1
                    bounding_box = BoundingBox()
                    bounding_box.class_name = label
                    bounding_box.probability = float(conf)
                    bounding_box.xmin = int(xyxy[0])
                    bounding_box.ymin = int(xyxy[1])
                    bounding_box.xmax = int(xyxy[2])
                    bounding_box.ymax = int(xyxy[3])
                    bounding_boxes.bounding_boxes.append(bounding_box)
                    self.get_logger().info("Found " + label + " with confidence " + str(conf))
            
            
            cv_image = annotator.result()
            # cv_image = results.render()
            
            output_image = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.get_logger().info("Frame " + str(self.frame_id) + ". Counted " + str(counted_objects.data) + " objects")   
            
            output_image.header = image.header
            output_image.header.frame_id = str(self.frame_id)
            
            self.counted_objects_pub_.publish(counted_objects)
            self.bounding_boxes_pub_.publish(bounding_boxes)
            self.detecteion_image_pub_.publish(output_image)
            
            if self.debug:
                cv2.namedWindow("Yolo Debug", cv2.WINDOW_NORMAL)
                cv2.imshow("Yolo Debug", cv_image)
                cv2.waitKey(1)
            
            self.frame_id += 1
            

def main(args=None):
    rclpy.init(args=args)
    
    yolo_publisher = YoloPublisher()
    rclpy.spin(yolo_publisher)
    
    yolo_publisher.destroy_node()
    rclpy.shutdown()
    
         
if __name__ == '__main__':
    main()   