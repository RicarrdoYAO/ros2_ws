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


        pkg_share = get_package_share_directory('iiwa_mujoco_ros2')
        model_path = os.path.join(pkg_share, 'models', 'scene.xml')
        self.get_logger().info(f'Loading MuJoCo model from: {model_path}')

        # 加载 MuJoCo 
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)


        self.joint_name_to_qpos = {}
        for j_id in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j_id)
            if name is not None:
                qpos_adr = self.model.jnt_qposadr[j_id]
                self.joint_name_to_qpos[name] = qpos_adr

        self.get_logger().info(f'Joint mapping: {self.joint_name_to_qpos}')

        # 保存最新关节角
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

    with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
        # 主循环：处理 ROS2 + 仿真和渲染
        while rclpy.ok() and viewer.is_running():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.data.qpos[:] = node.latest_qpos
            mujoco.mj_step(node.model, node.data)
            viewer.sync()

    node.destroy_node()
    rclpy.shutdown()
