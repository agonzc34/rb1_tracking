#!/bin/python3
import rclpy
from rclpy.node import Node
from time import sleep

from nav2_msgs.srv import ClearCostmapAroundRobot


class ClearCostmaps(Node):
    def __init__(self, *args):
        super().__init__('clear_costmaps_node')
        
        # args
        self.sleep_time = 2.0
        
        # clients
        self.clear_global_client = self.create_client(ClearCostmapAroundRobot, "/global_costmap/clear_around_global_costmap")
        self.clear_local_client = self.create_client(ClearCostmapAroundRobot, "/local_costmap/clear_around_local_costmap")
    
        self.get_logger().info("Clear node initialized")
        
        
    def clear_call(self):
        self.clear_global_map()
        self.clear_local_map()
        
        sleep(self.sleep_time)
        
        
    def clear_global_map(self):
        self.get_logger().info("Clearing global costmap")
        
        while not self.clear_global_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Clear global costmap service not available, waiting again...")
        
        request = ClearCostmapAroundRobot.Request()
        request.reset_distance = 5.0
        
        future = self.clear_global_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        
    def clear_local_map(self):
        self.get_logger().info("Clearing local costmap")
        
        while not self.clear_local_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Clear local costmap service not available, waiting again...")
            
        request = ClearCostmapAroundRobot.Request()
        request.reset_distance = 2.0
        
        future = self.clear_local_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
      

def main(args=None):
    rclpy.init(args=args)

    yolo_publisher = ClearCostmaps()

    while True:
        yolo_publisher.clear_call()

if __name__ == '__main__':
    main()