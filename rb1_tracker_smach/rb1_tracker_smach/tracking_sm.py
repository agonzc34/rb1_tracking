import rclpy
import smach
from pynput import keyboard
from rb1_tracker_smach.talking_node import TTSNode
from rb1_tracker.rb1_tracker import TrackingNode

class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.init_node = TTSNode()
        
    def execute(self, userdata):
        self.init_node.talk("Hello, I am RB1. I am ready to track you. Press space to start tracking. Press escape to stop tracking.")
        
        while not self.init_node.action_done:
            rclpy.spin_once(self.init_node)
        
        self.init_node.action_done = False
        return 'outcome1'


        
class WaitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.key_pressed = False
        
    def on_press(self, key):
        if key == keyboard.Key.space:
            return False
        
    def execute(self, userdata):
        keyboard_listener = keyboard.Listener(on_press=self.on_press)
        keyboard_listener.start()
        
        while not self.key_pressed:
            pass
        
        self.key_pressed = False
        return 'outcome1'
        
        
        
class TrackingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.track_node = TrackingNode('/camera/depth/points', 'camera_rgb_optical_frame')
        self.key_pressed = False
        
    def on_press(self, key):
        if key == keyboard.Key.esc:
            return False
        
    def execute(self, ud):
        keyboard_listener = keyboard.Listener(on_press=self.on_press)
        keyboard_listener.start()
        
        while self.key_pressed:
            rclpy.spin_once(self.track_node)
        
        self.key_pressed = False
        return 'outcome1'
        
        

def main():
    rclpy.init()
    
    sm = smach.StateMachine(outcomes=['finish_outcome'])
    
    with sm:
        sm.add('INIT', InitState(), transitions={'outcome1':'WAIT'})
        sm.add('WAIT', WaitState(), transitions={'outcome1':'TRACKING'})
        sm.add('TRACKING', TrackingState(), transitions={'outcome1':'WAIT'})
        
    outcome = sm.execute()
    
    print("outcome: ", outcome)
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()