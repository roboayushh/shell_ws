#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import threading
import time
import struct
from shell_messages.msg import ThrusterSignals, HardwareStatus, SensorTelemetry

# --- CONFIG ---
CMD_ARM_SYSTEM = b'\xAA\x55\xAA\x55' 
BAUD_RATE = 115200

# Binary Packet Config (Must match Arduino Struct)
# '<' = Little Endian, 'fff' = 3 floats
STRUCT_FORMAT = '<fff'
PACKET_SIZE = 12  # 3 floats * 4 bytes
HEADER_SIZE = 2   # 0xAA 0xBB
TOTAL_MSG_SIZE = PACKET_SIZE + HEADER_SIZE

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('port', '/dev/control_arduino')
        self.port = self.get_parameter('port').value
        
        # Internal State
        self.latest_pwm = [1500, 1500, 1500, 1500]
        self.lock = threading.Lock()
        self.ser = None
        self.hardware_state = "DISCONNECTED"
        self.running = True

        # --- BUFFER FOR HYBRID PARSING ---
        self.rx_buffer = bytearray() 

        # ROS Setup
        self.sub = self.create_subscription(
            ThrusterSignals, '/thruster_signals', self.pwm_callback, 10)
        self.pub_status = self.create_publisher(HardwareStatus, '/hw_status', 10)
        self.pub_sensors = self.create_publisher(SensorTelemetry, '/sensors/telemetry', 10)

        # Start Serial Thread
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
                    self.rx_buffer.clear() # Clear buffer on reconnect
                except serial.SerialException:
                    self.publish_status("NO_PORT")
                    time.sleep(1.0)
                    continue

            # --- 2. HYBRID READ (Text + Binary) ---
            try:
                if self.ser.in_waiting:
                    chunk = self.ser.read(self.ser.in_waiting)
                    self.rx_buffer.extend(chunk)
                    self.process_buffer()

            except Exception as e:
                self.get_logger().error(f"Read Error: {e}")
                self.close_serial()
                continue
            
            # --- 3. Write Outgoing Data (PWM) ---
            if self.hardware_state == "ARMED":
                try:
                    with self.lock:
                        current_pwm = self.latest_pwm
                    
                    payload = bytearray(struct.pack('<4H', *current_pwm))
                    checksum = sum(payload) % 256
                    payload.append(checksum)
                    
                    packet = self.cobs_encode(payload)
                    self.ser.write(packet)
                    
                except Exception as e:
                    self.get_logger().error(f"Write Error: {e}")
                    self.close_serial()

            time.sleep(0.02) # 50Hz Loop

    # --- BUFFER PROCESSOR ---
    def process_buffer(self):
        while len(self.rx_buffer) > 0:
            if len(self.rx_buffer) >= 2 and self.rx_buffer[0] == 0xAA and self.rx_buffer[1] == 0xBB:
                if len(self.rx_buffer) >= TOTAL_MSG_SIZE:
                    packet_bytes = self.rx_buffer[2:TOTAL_MSG_SIZE] 
                    self.parse_binary_sensor(packet_bytes)
                    del self.rx_buffer[0:TOTAL_MSG_SIZE]
                    continue
                else:
                    break

            try:
                nl_idx = self.rx_buffer.find(b'\n')
                if nl_idx != -1:
                    line_bytes = self.rx_buffer[:nl_idx]
                    line_str = line_bytes.decode('utf-8', errors='ignore').strip()
                    if line_str:
                        self.handle_arduino_text(line_str)
                    del self.rx_buffer[0:nl_idx+1]
                    continue
            except Exception:
                pass

            if self.rx_buffer[0] != 0xAA:
                del self.rx_buffer[0]
            else:
                break

    def parse_binary_sensor(self, data_bytes):
        try:
            # Unpack 3 floats
            temp, hum, volt = struct.unpack(STRUCT_FORMAT, data_bytes)
            
            msg = SensorTelemetry()
            msg.temperature = temp
            msg.humidity = hum
            msg.voltage = volt
            
            self.pub_sensors.publish(msg)
            
        except struct.error as e:
            self.get_logger().warn(f"Struct Unpack Error: {e}")

    def handle_arduino_text(self, line):
        if "BOOT_OK" in line:
            if self.hardware_state != "WAITING_HANDSHAKE":
                self.get_logger().warn("Arduino Reset Detected! Re-sending ARM command.")
            encoded_cmd = self.cobs_encode(CMD_ARM_SYSTEM)
            self.ser.write(encoded_cmd)
            self.hardware_state = "WAITING_HANDSHAKE"

        elif "ARM_CONFIRMED" in line:
            self.get_logger().info("Arduino Armed! Starting Data Stream.")
            self.hardware_state = "ARMED"

        elif "STATUS_DISARMED" in line:
            self.get_logger().warn("Arduino reports DISARMED.")
            self.hardware_state = "WAITING_HANDSHAKE"

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