from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='iiwa_mujoco_ros2',
            executable='iiwa_joint_publisher',
            name='iiwa_joint_publisher'
        ),
        Node(
            package='iiwa_mujoco_viewer',
            executable='viewer',
            name='mujoco_viewer'
        )
    ])
