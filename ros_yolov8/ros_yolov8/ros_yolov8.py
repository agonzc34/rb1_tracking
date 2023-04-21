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
from ultralytics.yolo.engine.results import Boxes

from yolo_msgs.msg import BoundingBoxes, BoundingBox
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image


class YoloPublisher(Node):
    
    def __init__(self):
        super().__init__('yolo_node') # type: ignore
        
        self.set_parameters([
            self.declare_parameter('yolo_weights', 'yolov8m.pt'),
            self.declare_parameter('debug', True),
            self.declare_parameter('tracker_yaml', 'bytetrack.yaml')
        ])
        
        yolo_weights = self.get_parameter('yolo_weights').get_parameter_value().string_value
        yolo_weights = os.path.join(ament_index_python.packages.get_package_share_directory('ros_yolov8'), 'net_props', yolo_weights)
        
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        print(self.debug)
        
        self.tracker_yaml = self.get_parameter('tracker_yaml').get_parameter_value().string_value
        self.tracker_yaml = os.path.join(ament_index_python.packages.get_package_share_directory('ros_yolov8'), 'net_props', self.tracker_yaml)
        
        self.bounding_boxes_pub_ = self.create_publisher(BoundingBox, "/obj_rec/bounding_box", 10)
        
        self.camera_read_sub_ = self.create_subscription(Image, "/head_front_camera/rgb/image_raw", self.camera_read_callback, 10)
        
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
            results = self.yolo.predict(source=cv_image, mode='track')
            
            det = results[0].boxes.cpu().numpy()

            if len(det) > 0:
                im0s = self.yolo.predictor.batch[2]
                im0s = im0s if isinstance(im0s, list) else [im0s]

                tracks = self.tracker.update(det, im0s[0])
                if len(tracks) > 0:
                    results[0].update(boxes=torch.as_tensor(tracks[:, :-1]))
            
            person_boxes = []
            
            print(str('Numero resultados: {}, {}'.format(len(results), len(results[0].boxes))))
            
            result = results[0]
            annotator = Annotator(cv_image)
            
            boxes = result.boxes
            
            for box in boxes:
                box: Boxes = box
                                
                cls = box.cls[0]
                xyxy = box.xyxy[0]
                conf = box.conf[0]
                
                print(str.format("{} {} {}", cls, xyxy, conf))

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
                bounding_box.probability = self.last_person[3]
                bounding_box.xmin = self.last_person[0][0]
                bounding_box.ymin = self.last_person[0][1]
                bounding_box.xmax = self.last_person[0][2]
                bounding_box.ymax = self.last_person[0][3]
                bounding_box.id = self.last_person[1]
                
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