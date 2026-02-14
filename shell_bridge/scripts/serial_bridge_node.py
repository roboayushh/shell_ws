#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import threading
import time
import struct
from shell_messages.msg import ThrusterSignals, HardwareStatus

# --- CONFIG ---
# This matches the Magic Bytes in Arduino
CMD_ARM_SYSTEM = b'\xAA\x55\xAA\x55' 
BAUD_RATE = 115200

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.port = self.get_parameter('port').value
        
        # Internal State
        self.latest_pwm = [1500, 1500, 1500, 1500]
        self.lock = threading.Lock()
        self.ser = None
        self.hardware_state = "DISCONNECTED" # States: DISCONNECTED, WAITING_HANDSHAKE, ARMED
        self.running = True

        # ROS Setup
        self.sub = self.create_subscription(
            ThrusterSignals, '/thruster_signals', self.pwm_callback, 10)
        self.pub_status = self.create_publisher(HardwareStatus, '/hw_status', 10)

        # Start the Serial Thread
        self.io_thread = threading.Thread(target=self.serial_io_loop, daemon=True)
        self.io_thread.start()

        self.get_logger().info(f"Initialized. Waiting for Arduino on {self.port}...")

    def pwm_callback(self, msg):
        with self.lock:
            self.latest_pwm = list(msg.pwm_values)[:4]

    def cobs_encode(self, data):
        output = bytearray()
        idx = 0
        while idx < len(data):
            next_zero = -1
            for i in range(idx, min(idx + 254, len(data))):
                if data[i] == 0:
                    next_zero = i
                    break
            
            if next_zero == -1:
                code = min(254, len(data) - idx) + 1
                output.append(code)
                output.extend(data[idx:idx + code - 1])
                idx += code - 1
            else:
                code = next_zero - idx + 1
                output.append(code)
                output.extend(data[idx:next_zero])
                idx = next_zero + 1
        output.append(0)
        return output

    def serial_io_loop(self):
        while self.running:
            # --- 1. Connection Handling ---
            if self.ser is None:
                try:
                    self.ser = serial.Serial(self.port, BAUD_RATE, timeout=0.05)
                    self.get_logger().info(f"Port Opened: {self.port}")
                    self.hardware_state = "WAITING_HANDSHAKE"
                    self.ser.reset_input_buffer()
                except serial.SerialException:
                    self.publish_status("NO_PORT")
                    time.sleep(1.0)
                    continue

            # --- 2. Read Incoming Data (Handshakes) ---
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.handle_arduino_message(line)
            except Exception as e:
                self.get_logger().error(f"Read Error: {e}")
                self.close_serial()
                continue
            
            # --- 3. Write Outgoing Data (PWM) ---
            # We ONLY send data if we are fully ARMED.
            if self.hardware_state == "ARMED":
                try:
                    with self.lock:
                        current_pwm = self.latest_pwm
                    
                    # Pack 4x uint16 (Little Endian)
                    payload = bytearray(struct.pack('<4H', *current_pwm))
                    
                    # Calculate Checksum (Sum % 256)
                    checksum = sum(payload) % 256
                    payload.append(checksum)
                    
                    # Encode and Send
                    packet = self.cobs_encode(payload)
                    self.ser.write(packet)
                    
                except Exception as e:
                    self.get_logger().error(f"Write Error: {e}")
                    self.close_serial()

            time.sleep(0.02) # Run at 50Hz

    def handle_arduino_message(self, line):
        # LOGIC:
        # If we see BOOT_OK, Arduino is begging for config. Send ARM command.
        # This works even if we *thought* we were ARMED (self-healing).
        
        if "BOOT_OK" in line:
            if self.hardware_state != "WAITING_HANDSHAKE":
                self.get_logger().warn("Arduino Reset Detected! Re-sending ARM command.")
            
            # Send Arm Command (COBS Encoded)
            encoded_cmd = self.cobs_encode(CMD_ARM_SYSTEM)
            self.ser.write(encoded_cmd)
            self.hardware_state = "WAITING_HANDSHAKE"

        elif "ARM_CONFIRMED" in line:
            self.get_logger().info("Arduino Armed! Starting Data Stream.")
            self.hardware_state = "ARMED"

        elif "STATUS_DISARMED" in line:
            self.get_logger().warn("Arduino reports DISARMED. Waiting for BOOT_OK...")
            self.hardware_state = "WAITING_HANDSHAKE"

        # Publish status to ROS network
        self.publish_status(line)

    def publish_status(self, msg_text):
        msg = HardwareStatus()
        msg.is_armed = (self.hardware_state == "ARMED")
        msg.last_error = msg_text
        self.pub_status.publish(msg)

    def close_serial(self):
        if self.ser:
            self.ser.close()
        self.ser = None
        self.hardware_state = "DISCONNECTED"
        self.publish_status("DISCONNECTED")

    def stop(self):
        self.running = False
        if self.ser: self.ser.close()

def main():
    rclpy.init()
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()