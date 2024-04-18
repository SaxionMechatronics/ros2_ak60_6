from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros2_ak60_6",
            executable="ak60_6",
            name="ak60_6_1",
            output="screen",
            namespace='motor1',
            parameters=[
                {"motor_can_id": 1},
                {"motor_stiffness": 1.0},
                {"motor_dampening": 0.5}
            ]
        ),
        Node(
            package="ros2_ak60_6",
            executable="ak60_6",
            name="ak60_6_2",
            output="screen",
            namespace='motor2',
            parameters=[
                {"motor_can_id": 2},
                {"motor_stiffness": 1.0},
                {"motor_dampening": 0.5}
            ]
        )        
    ])
