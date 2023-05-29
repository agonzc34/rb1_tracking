import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from audio_common_msgs.action import TTS

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.action_client = ActionClient(self, TTS, '/tts')
        self.action_done = False
                
    def talk(self, message) -> rclpy.task.Future:
        msg = TTS.Goal()
        msg.text = message
        msg.volume = 0.5
        msg.rate = 0.5
        
        self.action_client.wait_for_server()
        
        future = self.action_client.send_goal_async(msg)
        
        future.add_done_callback(self.done_callback)
        
    def done_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self.future_result = goal_handle.get_result_async()
        self.future_result.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        self.action_done = True