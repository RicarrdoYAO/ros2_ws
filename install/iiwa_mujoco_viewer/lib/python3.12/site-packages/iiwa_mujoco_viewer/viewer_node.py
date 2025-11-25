import os
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import mujoco
import mujoco.viewer

from ament_index_python.packages import get_package_share_directory


class IiwaMujocoViewer(Node):
    def __init__(self):
        super().__init__('iiwa_mujoco_viewer')

        # 找到 iiwa_mujoco_ros2 包里 models/scene.xml 的路径
        pkg_share = get_package_share_directory('iiwa_mujoco_ros2')
        model_path = os.path.join(pkg_share, 'models', 'scene.xml')
        self.get_logger().info(f'Loading MuJoCo model from: {model_path}')

        # 加载 MuJoCo 模型
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # 关节名 -> 索引的映射
        self.joint_name_to_qpos = {}
        for j_id in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j_id)
            if name is not None:
                qpos_adr = self.model.jnt_qposadr[j_id]
                self.joint_name_to_qpos[name] = qpos_adr

        self.get_logger().info(f'Joint mapping: {self.joint_name_to_qpos}')

        # 保存最新的关节角（长度 = nq）
        self.latest_qpos = np.zeros(self.model.nq)

        # 订阅 C++ 侧发布的关节状态
        self.create_subscription(
            JointState,
            'iiwa/joint_states',
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg: JointState):
        # 根据关节名字，把 ROS 的位置写入最新的 qpos
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_name_to_qpos:
                q_idx = self.joint_name_to_qpos[name]
                self.latest_qpos[q_idx] = pos


def main(args=None):
    rclpy.init(args=args)
    node = IiwaMujocoViewer()

    # 打开 MuJoCo 的被动 viewer（我们自己调用 mj_step）
    with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
        # 主循环：既要处理 ROS2 回调，又要跑仿真和渲染
        while rclpy.ok() and viewer.is_running():
            # 处理一次 ROS2 回调（非阻塞）
            rclpy.spin_once(node, timeout_sec=0.0)

            # 将最新接受到的 qpos 写入 MuJoCo data
            node.data.qpos[:] = node.latest_qpos

            # 迈一步仿真，并刷新 viewer
            mujoco.mj_step(node.model, node.data)
            viewer.sync()

    node.destroy_node()
    rclpy.shutdown()
