#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from shell_messages.msg import Control3DOF, ThrusterSignals, HardwareStatus

class MixerPidNode(Node):
    def __init__(self):
        super().__init__('mixer_pid_node')

        # 1. State Variables
        self.is_armed = False
        
        # 2. Mixing Matrix (4 Thrusters x 3 DOF)
        # Row format: [Surge, Pitch, Yaw]
        # Based on your "Bowl" configuration
        self.mixing_matrix = np.array([
            [1.0, -1.0,  1.0],  # Thruster 1
            [1.0, -1.0, -1.0],  # Thruster 2
            [1.0,  1.0,  1.0],  # Thruster 3
            [1.0,  1.0, -1.0]   # Thruster 4
        ])

        # 3. Subscribers
        self.control_sub = self.create_subscription(
            Control3DOF, '/control_topic', self.control_callback, 10)
        
        self.status_sub = self.create_subscription(
            HardwareStatus, '/hw_status', self.status_callback, 10)

        # 4. Publisher
        self.thruster_pub = self.create_publisher(ThrusterSignals, '/thruster_signals', 10)

        self.get_logger().info("Mixer PID Node initialized and waiting for ARM status...")

    def status_callback(self, msg):
        """Updates the internal arming state from Arduino feedback."""
        self.is_armed = msg.is_armed

    def control_callback(self, msg):
        """Processes joystick input into PWM signals."""
        thruster_msg = ThrusterSignals()

        if not self.is_armed:
            # SAFETY: If not armed, always output Neutral
            thruster_msg.pwm_values = [1500, 1500, 1500, 1500]
        else:
            # 1. Create Input Vector V
            V = np.array([msg.surge, msg.pitch, msg.yaw])

            # 2. Matrix Multiplication T = M x V
            # Resulting T is an array of 4 values between -1.0 and 1.0
            T = np.dot(self.mixing_matrix, V)

            # 3. Convert Normalized thrust (-1 to 1) to PWM (1100 to 1900)
            # Formula: 1500 + (T * 400)
            pwm_outputs = []
            for thrust_value in T:
                # Clip value to ensure we don't exceed -1.0 to 1.0
                thrust_clamped = max(-1.0, min(1.0, thrust_value))
                pwm = int(1500 + (thrust_clamped * 400))
                pwm_outputs.append(pwm)
            
            thruster_msg.pwm_values = pwm_outputs

        self.thruster_pub.publish(thruster_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MixerPidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()