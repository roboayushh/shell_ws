#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
import threading
import time
import os  #
from shell_messages.msg import SensorTelemetry

# --- CONFIG ---
HEADER = b'\xAA\xBB'
CMD_START_STREAM = b'\xA5'

class DisplayBridgeNode(Node):
    def __init__(self):
        super().__init__('display_bridge_node')
        
        self.declare_parameter('display_port', '/dev/lcd_arduino') 
        self.port = self.get_parameter('display_port').value
        self.baud_rate = 115200

        self.ser = None
        self.connected = False
        self.lock = threading.Lock()

        self.subscription = self.create_subscription(
            SensorTelemetry,
            '/sensors/telemetry',
            self.listener_callback,
            10)

        self.connect_thread = threading.Thread(target=self.connection_loop, daemon=True)
        self.connect_thread.start()

    def connection_loop(self):
        while rclpy.ok():
            # 1. Connect if not connected
            if self.ser is None:
                try:
                    self.ser = serial.Serial(self.port, self.baud_rate, timeout=0.1)
                    self.get_logger().info(f"Display Port Opened: {self.port}")
                    self.connected = False
                except serial.SerialException:
                    self.get_logger().warn(f"Cannot open Display Port: {self.port}")
                    time.sleep(2)
                    continue

            # 2. Read Serial Data (Handshake + Commands)
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Handshake
                    if "BOOT_OK" in line:
                        self.get_logger().info("Handshake Received. Sending Start Command...")
                        self.ser.write(CMD_START_STREAM)
                        self.connected = True
                    
                    # --- KILL SWITCH LOGIC ---
                    elif "KILL_SYSTEM" in line:
                        self.get_logger().fatal("!!! KILL SWITCH ACTIVATED - SHUTTING DOWN !!!")
                        # Kills all ROS nodes and shuts down the Pi
                        os.system("shutdown -h now") 
            
            except Exception as e:
                self.ser = None
                self.connected = False
            
            time.sleep(0.05)

    def listener_callback(self, msg):
        if not self.connected or self.ser is None:
            return

        try:
            payload = struct.pack('<ffffff', 
                                  msg.roll, msg.pitch, msg.yaw, 
                                  msg.temperature, msg.humidity, msg.voltage)
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