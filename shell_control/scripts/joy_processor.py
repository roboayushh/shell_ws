#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from shell_messages.msg import Control3DOF

class JoyProcessor(Node):
    def __init__(self):
        super().__init__('joy_processor')
        
        # Subscribe to raw joystick data
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        
        # Publisher for our custom AUV control message
        self.publisher = self.create_publisher(Control3DOF, '/control_topic', 10)
        
        self.get_logger().info("Joy Processor Node Started. Listening to /joy...")

    def joy_callback(self, msg):
        control_msg = Control3DOF()
        
        # MAP YOUR JOYSTICK AXES HERE
        # Example: Left Stick Up/Down = Surge, Left Stick Left/Right = Yaw
        # Right Stick Up/Down = Pitch
        try:
            control_msg.surge = msg.axes[1]  # Forward/Back
            control_msg.yaw   = msg.axes[3]  # Left/Right
            control_msg.pitch = msg.axes[4]  # Tilt Up/Down
            
            self.publisher.publish(control_msg)
        except IndexError:
            self.get_logger().warn("Joystick axis index out of range! Check your controller mapping.")

def main(args=None):
    rclpy.init(args=args)
    node = JoyProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()