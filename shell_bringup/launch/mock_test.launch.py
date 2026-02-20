from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1. MOCK Joystick Publisher
        # Publishes data to /joy topic at 10Hz
        # Format: axes: [0.0, 0.1 (SURGE), 0.0, 0.0, 0.0 (PITCH), 0.0]
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-r', '10', 
                '/joy', 'sensor_msgs/msg/Joy', 
                '{axes: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0], buttons: [0,0,0,0,0,0,0,0]}'
            ],
            output='log'
        ),

        # 2. Start Joy Processor
        # Will receive the mock data and output Control3DOF(surge=0.1)
        Node(
            package='shell_control',
            executable='joy_processor.py',
            name='joy_processor',
            output='screen'
        ),

        # 3. Start PID Mixer
        Node(
            package='shell_control',
            executable='mixer_pid_node.py',
            name='mixer_pid_node',
            output='screen'
        ),

        # 4. Start Serial Bridge (Control Arduino)
        Node(
            package='shell_bridge',
            executable='serial_bridge_node.py',
            name='serial_bridge_node',
            parameters=[{'port': '/dev/control_arduino'}],
            output='screen'
        ),

        # 5. Start Display Bridge (LCD Arduino)
        # Node(
        #     package='shell_bridge',
        #     executable='display_bridge_node.py',
        #     name='display_bridge_node',
        #     parameters=[{'display_port': '/dev/lcd_arduino'}],
        #     output='screen'
        # )
    ])