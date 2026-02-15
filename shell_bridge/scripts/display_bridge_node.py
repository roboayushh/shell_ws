#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
import threading
import time
from shell_messages.msg import SensorTelemetry

# --- CONFIG ---
# Protocol: Header (2 bytes) + 6 Floats (24 bytes) = 26 Bytes Total
HEADER = b'\xAA\xBB'
CMD_START_STREAM = b'\xA5' # Simple 1-byte command to tell Arduino "I am here"

class DisplayBridgeNode(Node):
    def __init__(self):
        super().__init__('display_bridge_node')

        # Port for the DISPLAY Arduino (NOT the Thruster Arduino)
        self.declare_parameter('display_port', '/dev/lcd_arduino') 
        self.port = self.get_parameter('display_port').value
        self.baud_rate = 115200

        self.ser = None
        self.connected = False
        self.lock = threading.Lock()

        # Subscribe to the sensor data coming from the Thruster Arduino
        self.subscription = self.create_subscription(
            SensorTelemetry,
            '/sensors/telemetry',
            self.listener_callback,
            10)

        # Start Connection Thread
        self.connect_thread = threading.Thread(target=self.connection_loop, daemon=True)
        self.connect_thread.start()

    def connection_loop(self):
        """ Handles the Handshake: Wait for BOOT_OK -> Send Start Command """
        while rclpy.ok():
            if self.ser is None:
                try:
                    self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
                    self.get_logger().info(f"Display Port Opened: {self.port}")
                    self.connected = False
                except serial.SerialException:
                    self.get_logger().warn(f"Cannot open Display Port: {self.port}")
                    time.sleep(2)
                    continue

            # Handshake Logic
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if "BOOT_OK" in line:
                        self.get_logger().info("Display Arduino Found! Sending Start Command...")
                        self.ser.write(CMD_START_STREAM)
                        self.connected = True
            except Exception as e:
                self.ser = None
                self.connected = False
            
            time.sleep(0.1)

    def listener_callback(self, msg):
        """ Received Sensor Data from ROS -> Pack Binary -> Send to Display """
        if not self.connected or self.ser is None:
            return

        try:
            # 1. Pack data into binary (Same format as Arduino struct)
            # < = Little Endian, ffffff = 6 floats
            payload = struct.pack('<ffffff', 
                                  msg.roll, msg.pitch, msg.yaw, 
                                  msg.temperature, msg.humidity, msg.voltage)

            # 2. Send Header + Payload
            with self.lock:
                self.ser.write(HEADER)
                self.ser.write(payload)

        except Exception as e:
            self.get_logger().error(f"Serial Write Failed: {e}")
            self.connected = False
            if self.ser: self.ser.close()
            self.ser = None

def main(args=None):
    rclpy.init(args=args)
    node = DisplayBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()