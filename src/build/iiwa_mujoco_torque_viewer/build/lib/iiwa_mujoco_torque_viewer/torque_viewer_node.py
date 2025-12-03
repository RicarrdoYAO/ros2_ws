import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import mujoco
import mujoco.viewer
from ament_index_python.packages import get_package_share_directory


class IiwaTorqueViewer(Node):
    def __init__(self):
        super().__init__('iiwa_torque_viewer')

        pkg_share = get_package_share_directory('iiwa_mujoco_torque_viewer')
        model_path = os.path.join(pkg_share, 'models', 'scene.xml')
        self.get_logger().info(f'Loading MuJoCo model from: {model_path}')

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # joint name -> qpos index
        self.joint_name_to_qpos = {}
        for j_id in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j_id)
            if name:
                self.joint_name_to_qpos[name] = self.model.jnt_qposadr[j_id]

        # latest torque
        self.latest_torque = np.zeros(self.model.nu)

        # read joint angles
        self.create_subscription(
            JointState,
            'iiwa/joint_states',
            self.joint_state_callback,
            10
        )

        # read torque
        self.create_subscription(
            Float64MultiArray,
            'iiwa/joint_torque_cmd',
            self.torque_callback,
            10
        )

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_name_to_qpos:
                idx = self.joint_name_to_qpos[name]
                self.data.qpos[idx] = pos

    def torque_callback(self, msg):
        for i, tau in enumerate(msg.data):
            self.data.ctrl[i] = tau


def main(args=None):
    rclpy.init(args=args)
    node = IiwaTorqueViewer()

    with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
        while rclpy.ok() and viewer.is_running():
            rclpy.spin_once(node, timeout_sec=0)
            mujoco.mj_step(node.model, node.data)
            viewer.sync()

    node.destroy_node()
    rclpy.shutdown()
