from std_msgs.msg import String
from std_srvs.srv import Trigger

class DaisyAPI:
    def __init__(self, node):
        self.node = node
        # Publishers
        self.mode_pub = node.create_publisher(String, '/robot_head/control_mode', 10)
        self.face_pub = node.create_publisher(String, '/robot_head/face', 10)
        self.gesture_pub = node.create_publisher(String, '/robot_head/gesture', 10)
        self.text_pub = node.create_publisher(String, '/robot_head/text', 10)
        
        # Service Clients
        self.oopsie_client = node.create_client(Trigger, '/robot_head/oopsie_daisy_service')
        self.shake_client = node.create_client(Trigger, '/robot_head/shake_service')
        self.nod_client = node.create_client(Trigger, '/robot_head/nod_service')
        self.dance_client = node.create_client(Trigger, '/robot_head/dance_service')
        self.daisy_dance_client = node.create_client(Trigger, '/robot_head/daisy_dance_service')
        self.thinking_client = node.create_client(Trigger, '/robot_head/thinking_service')
        self.look_around_client = node.create_client(Trigger, '/robot_head/look_around_service')
        
    def take_control(self):
        """Set head control mode to ROS."""
        self.mode_pub.publish(String(data="ROS"))
        
    def be_sad(self):
        self.face_pub.publish(String(data="SAD"))
        
    def be_happy(self):
        self.face_pub.publish(String(data="HAPPY"))
        
    def say_oops(self):
        """Call the oopsie daisy service (face and gesture)."""
        if self.oopsie_client.wait_for_service(timeout_sec=0.5):
            self.oopsie_client.call_async(Trigger.Request())
        else:
            self.node.get_logger().warn("Daisy oopsie_daisy_service not available")
        
    def shake_head(self):
        """Shake head gesture."""
        if self.shake_client.wait_for_service(timeout_sec=0.5):
            self.shake_client.call_async(Trigger.Request())
            
    def nod_head(self):
        """Nod head gesture."""
        if self.nod_client.wait_for_service(timeout_sec=0.5):
            self.nod_client.call_async(Trigger.Request())
            
    def daisy_dance(self):
        """Daisy dance gesture."""
        if self.daisy_dance_client.wait_for_service(timeout_sec=0.5):
            self.daisy_dance_client.call_async(Trigger.Request())
            
    def look_around(self):
        """Look around gesture."""
        if self.look_around_client.wait_for_service(timeout_sec=0.5):
            self.look_around_client.call_async(Trigger.Request())
            
    def show_thinking(self):
        """Show thinking face."""
        if self.thinking_client.wait_for_service(timeout_sec=0.5):
            self.thinking_client.call_async(Trigger.Request())
        
    def show_text(self, text, hold_ms=3000):
        """Show text on the screen."""
        self.text_pub.publish(String(data=f"{hold_ms}|false|{text}"))
