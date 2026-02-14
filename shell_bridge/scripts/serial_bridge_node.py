#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import threading
import time
from shell_messages.msg import ThrusterSignals, HardwareStatus

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # 1. Serial Config
        self.declare_parameter('port', '/dev/sensors/arduino')
        self.port = self.get_parameter('port').value
        self.baud = 115200
        self.ser = None

        # 2. State & Buffering (Single-Slot LIFO)
        self.latest_pwm = [1500, 1500, 1500, 1500]
        self.data_lock = threading.Lock()
        self.is_arduino_online = False

        # 3. ROS Comms
        self.thruster_sub = self.create_subscription(
            ThrusterSignals, '/thruster_signals', self.thruster_callback, 10)
        self.status_pub = self.create_publisher(HardwareStatus, '/hw_status', 10)

        # 4. Background Serial Thread
        self.running = True
        self.serial_thread = threading.Thread(target=self.serial_handler, daemon=True)
        self.serial_thread.start()

        self.get_logger().info(f"Bridge started on {self.port} at {self.baud}")

    def thruster_callback(self, msg):
        """Single-slot buffer: always overwrite with the newest data."""
        with self.data_lock:
            self.latest_pwm = list(msg.pwm_values)

    def cobs_encode(self, data):
        """Consistent Overhead Byte Stuffing to remove 0x00."""
        res = bytearray([0])
        ptr = 0
        for i, b in enumerate(data):
            if b == 0:
                res[ptr] = i - ptr + 1
                ptr = len(res)
                res.append(0)
            else:
                res.append(b)
        res[ptr] = len(res) - ptr
        res.append(0) # The delimiter
        return res

    def serial_handler(self):
        last_read_time = time.time()
        
        while self.running:
            if self.ser is None or not self.ser.is_open:
                self.is_arduino_online = False
                try:
                    self.ser = serial.Serial(self.port, self.baud, timeout=0) # Timeout 0 for non-blocking
                    self.get_logger().info("Serial Port Opened.")
                except Exception as e:
                    time.sleep(1.0)
                    continue

            # --- WRITE STEP ---
            with self.data_lock:
                raw_payload = bytearray()
                for val in self.latest_pwm:
                    raw_payload.extend(val.to_bytes(2, 'little'))
                
                checksum = sum(raw_payload) % 256
                raw_payload.append(checksum)
                encoded_packet = self.cobs_encode(raw_payload)
                
                try:
                    self.ser.write(encoded_packet)
                except serial.SerialException:
                    self.ser = None # Trigger reconnect
                    continue

            # --- READ STEP (Non-blocking) ---
            try:
                if self.ser.in_waiting > 0:
                    # Read what's available without waiting for a newline
                    data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    for line in data.split('\r\n'):
                        if line:
                            self.process_arduino_feedback(line)
                            last_read_time = time.time()
                            self.is_arduino_online = True
            except Exception as e:
                self.get_logger().warn(f"Read error: {e}")

            # --- HEARTBEAT CHECK ---
            # If we haven't heard from Arduino in 2s, tell ROS it's offline
            if time.time() - last_read_time > 2.0:
                if self.is_arduino_online:
                    self.get_logger().error("Arduino Heartbeat Lost!")
                    self.is_arduino_online = False
                msg = HardwareStatus()
                msg.is_armed = False
                msg.last_error = "Hardware Offline"
                self.status_pub.publish(msg)

            time.sleep(0.02) # Rigid 50Hz

    def process_arduino_feedback(self, line):
        msg = HardwareStatus()
        if "STATUS_ARMED" in line:
            msg.is_armed = True
        elif "STATUS_DISARMED" in line:
            msg.is_armed = False
        
        # In future, parse battery from string like "VOLT:12.4"
        if "VOLT:" in line:
            try:
                msg.battery_voltage = float(line.split(":")[1])
            except: pass
            
        self.status_pub.publish(msg)

    def stop(self):
        self.running = False
        if self.ser: self.ser.close()

def main():
    rclpy.init()
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()