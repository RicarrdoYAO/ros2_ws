from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from datetime import datetime

def generate_launch_description():

    # --- 自动生成带日期时间的 rosbag 名称 ---
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = f"iiwa_bag_{timestamp}"

    # 1) MuJoCo 仿真节点
    mujoco_node = Node(
        package="iiwa_mujoco",
        executable="iiwa_mujoco_node",
        output="screen"
    )

    # 2) 控制器节点
    controller_node = Node(
        package="iiwa_mujoco",
        executable="iiwa_controller_node",
        output="screen"
    )

    # 3) rosbag2 record
    bag_record = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "/iiwa/joint_states",
            "-o", bag_name
        ],
        shell=True,
        output="screen"
    )

    return LaunchDescription([
        mujoco_node,
        controller_node,
        bag_record
    ])
