from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='shell_control',
            executable='joy_processor.py',
            name='joy_processor'
        ),
        Node(
            package='shell_control',
            executable='mixer_pid_node.py',
            name='mixer_pid'
        ),
        # 1. Serial Bridge
        Node(
            package='shell_bridge',
            executable='serial_bridge_node.py',
            name='serial_bridge',
            parameters=[{'port': '/dev/sensors/arduino'}]
        ),
        # 2. Mixer Node

        # 3. Joy Processor (Optional to run on Pi if Joystick is on Laptop)

    ])