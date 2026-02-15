import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Start Joystick Driver (Standard ROS 2 Node)
        # Reads from physical controller (e.g., /dev/input/js0)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.05, 'autorepeat_rate': 20.0}]
        ),

        # 2. Start Joy Processor
        # Converts /joy -> /control_topic (Control3DOF)
        Node(
            package='shell_control',
            executable='joy_processor.py',
            name='joy_processor',
            output='screen'
        ),

        # 3. Start PID Mixer
        # Converts /control_topic -> /thruster_signals
        Node(
            package='shell_control',
            executable='mixer_pid_node.py',
            name='mixer_pid_node',
            output='screen'
        ),

        # 4. Start Serial Bridge (Control Arduino)
        # Handles Thrusters and Sensors
        Node(
            package='shell_bridge',
            executable='serial_bridge_node.py',
            name='serial_bridge_node',
            parameters=[{'port': '/dev/control_arduino'}], # Ensure this matches your UDEV rule or path
            output='screen'
        ),

        # 5. Start Display Bridge (LCD Arduino)
        # Displays telemetry on the LCD
        Node(
            package='shell_bridge',
            executable='display_bridge_node.py',
            name='display_bridge_node',
            parameters=[{'display_port': '/dev/lcd_arduino'}], # Ensure this matches your UDEV rule or path
            output='screen'
        )
    ])