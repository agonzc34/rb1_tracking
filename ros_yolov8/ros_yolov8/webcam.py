import cv2 as cv
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
import rclpy.logging

from sensor_msgs.msg import Image


class WebcamPublisher(Node):

    def __init__(self):
        super().__init__('yolo_node')
        self.webcam_pub_ = self.create_publisher(Image, '/head_front_camera/rgb/image_raw', 10)
        timer_period = 1.0 / 30.0

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.capture = cv.VideoCapture(0)
        self.cv_bridge = CvBridge()

        self.i = 0

    
    def timer_callback(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'webcam'

        ret, frame = self.capture.read()
        img_msg = self.cv_bridge.cv2_to_imgmsg(frame)

        self.webcam_pub_.publish(img_msg)

        self.i = (self.i + 1) % 200

        if self.i == 1:
            self.get_logger().info('Publishing: "%s"' % msg.header.frame_id)



def main(args=None):
    rclpy.init(args=args)

    webcam_pub = WebcamPublisher()
    rclpy.spin(webcam_pub)

    webcam_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()