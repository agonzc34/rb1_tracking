import cv2
import torch
import numpy as np
import rclpy
from rclpy.node import Node
import cv_bridge
import os
import ament_index_python.packages

from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator
from ultralytics.yolo.utils import IterableSimpleNamespace, yaml_load
from ultralytics.yolo.utils.checks import check_yaml
from ultralytics.tracker.trackers import BYTETracker
from rclpy.qos import qos_profile_sensor_data
from ultralytics.yolo.engine.results import Boxes

from yolo_msgs.msg import BoundingBoxes, BoundingBox
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image


class YoloPublisher(Node):
    
    def __init__(self):
        super().__init__('yolo_node') # type: ignore
        
        self.declare_parameter('debug', False)
        self.declare_parameter('yolo_weights', os.path.join(ament_index_python.packages.get_package_share_directory('ros_yolov8'), 'net_props', 'yolov8s.pt'))
        self.declare_parameter('image_topic', '/camera/rgb/image_raw')
        self.declare_parameter('tracker_yaml', os.path.join(ament_index_python.packages.get_package_share_directory('ros_yolov8'), 'net_props', 'bytetrack.yaml'))
        
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        
        yolo_weights = self.get_parameter('yolo_weights').get_parameter_value().string_value
        # yolo_weights = os.path.join(ament_index_python.packages.get_package_share_directory('ros_yolov8'), 'net_props', yolo_weights)
        
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        self.tracker_yaml = self.get_parameter('tracker_yaml').get_parameter_value().string_value
        self.tracker_yaml = os.path.join(ament_index_python.packages.get_package_share_directory('ros_yolov8'), 'net_props', self.tracker_yaml)
        
        self.bounding_boxes_pub_ = self.create_publisher(BoundingBoxes, "/obj_rec/bounding_boxes", qos_profile_sensor_data)
        
        self.camera_read_sub_ = self.create_subscription(Image, self.image_topic, self.camera_read_callback, qos_profile_sensor_data)
        
        self.cv_bridge = cv_bridge.CvBridge()
        
        self.yolo = YOLO(yolo_weights)
        self.yolo.fuse()
        
        self.enabled = True
        
        self.create_service(SetBool, 'yolo_enable', self.enable_callback)
        self.create_tracker()
        
        self.last_person_id = None
        self.frame_id = 0
    
    
    def enable_callback(self, request, response):
        self.enabled = request.data
        response.success = True
        return response
    
    
    def camera_read_callback(self, image):
        if self.enabled:
            bounding_boxes = BoundingBoxes()
                        
            bounding_boxes.header.frame_id = str(self.frame_id)
            bounding_boxes.header.stamp = image.header.stamp
            
            bounding_box = BoundingBox()
            
            cv_image = self.cv_bridge.imgmsg_to_cv2(image)
            results = self.yolo.predict(source=cv_image, mode='track', classes=[0,1], verbose=self.debug)
            
            det = results[0].boxes.cpu().numpy()

            if len(det) > 0:
                im0s = self.yolo.predictor.batch[2]
                im0s = im0s if isinstance(im0s, list) else [im0s]

                tracks = self.tracker.update(det, im0s[0])
                if len(tracks) > 0:
                    results[0].update(boxes=torch.as_tensor(tracks[:, :-1]))
            
            person_boxes = []
                        
            result = results[0]
            annotator = Annotator(cv_image)
            
            boxes = result.boxes
            
            for box in boxes:
                box: Boxes = box
                                
                cls = box.cls[0]
                xyxy = box.xyxy[0]
                conf = box.conf[0]
                
                # annotator.box_label(xyxy, self.yolo.names[int(cls)])

                if conf > 0.5 and self.yolo.names[int(cls)] == 'person':
                    track_id = ""
                    
                    if not box.id is None:
                        track_id = str(int(box.id))
                    
                    person_boxes.append((xyxy, track_id, cls, conf))
                    
                    annotator.box_label(xyxy, track_id)

            if len(person_boxes) == 0:
                self.last_person = None
            else:
                self.last_person = min(person_boxes, key=lambda x: x[0][0])
                
                bounding_box.class_name = self.yolo.names[int(self.last_person[2])]
                bounding_box.probability = float(self.last_person[3])
                bounding_box.xmin = int(self.last_person[0][0])
                bounding_box.ymin = int(self.last_person[0][1])
                bounding_box.xmax = int(self.last_person[0][2])
                bounding_box.ymax = int(self.last_person[0][3])
                if self.last_person[1] != '':
                    bounding_box.id = int(self.last_person[1])
                else:
                    bounding_box.id = 0
                
                self.get_logger().info("Person detected with id: " + str(bounding_box.id))
                
                bounding_boxes.bounding_boxes.append(bounding_box)
            
            
            cv_image = annotator.result()
            # cv_image = results.render()
            
            if len(bounding_boxes.bounding_boxes) > 0:          
                self.bounding_boxes_pub_.publish(bounding_boxes)
            
            if self.debug:
                cv2.namedWindow("Yolo Debug", cv2.WINDOW_NORMAL)
                cv2.imshow("Yolo Debug", cv_image)
                cv2.waitKey(1)
            
            self.frame_id += 1
            
    
    def create_tracker(self):
        tracker = check_yaml(self.tracker_yaml)
        cfg = IterableSimpleNamespace(**yaml_load(tracker))
        
        self.tracker = BYTETracker(args=cfg, frame_rate=30)
    
            

def main(args=None):
    rclpy.init(args=args)
    
    yolo_publisher = YoloPublisher()
    rclpy.spin(yolo_publisher)
    
    yolo_publisher.destroy_node()
    rclpy.shutdown()
    
         
if __name__ == '__main__':
    main()   