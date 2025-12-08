import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

bag_path = "/home/yaohouyu/ros2_mujoco_ws/iiwa_bag_20251208_205906"

# 读取 MCAP
reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
converter_options = rosbag2_py.ConverterOptions("", "")
reader.open(storage_options, converter_options)

# 存储
times, q_list, dq_list, tau_list = [], [], [], []

while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic != "/iiwa/joint_states":
        continue

    msg = deserialize_message(data, JointState)

    # 使用仿真时间（绝对时间，不归零）
    t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    times.append(t_sec)

    q_list.append(msg.position)
    dq_list.append(msg.velocity)
    tau_list.append(msg.effort)

# 转数组
times = np.array(times)
q = np.array(q_list)
dq = np.array(dq_list)
tau = np.array(tau_list)

# ---------------------------
#  正弦参考轨迹（与控制器完全一致）
# ---------------------------

amp = np.array([0.5] * 7)
freq = np.array([4.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1])

# 参考位置
qd = amp * np.sin(freq * times[:, None])

# 参考速度（直接解析形式，比 gradient 更平滑）
dqd = amp * freq * np.cos(freq * times[:, None])

# 数值求导实际速度（验证用）
dq_num = np.gradient(q, axis=0) / np.gradient(times)[:, None]

# ---------------------------
#  Plot: 单关节速度检查
# ---------------------------

plt.figure()
plt.plot(times, dq[:, 0], label="dq(topic)")
plt.plot(times, dqd[:, 0], "--", label="dq_ref")
plt.legend()
plt.title("Velocity Check (Joint 1)")
plt.show()

# ---------------------------
# 7x3 子图（位置，速度，力矩）
# ---------------------------

fig, axes = plt.subplots(7, 3, figsize=(16, 18))
for i in range(7):
    # 位置对比
    axes[i, 0].plot(times, q[:, i], label="real")
    axes[i, 0].plot(times, qd[:, i], "--", label="ref")
    axes[i, 0].grid()
    
    # 速度对比
    axes[i, 1].plot(times, dq[:, i], label="real")
    axes[i, 1].plot(times, dqd[:, i], "--", label="ref")
    axes[i, 1].grid()

    # 力矩
    axes[i, 2].plot(times, tau[:, i], label="tau")
    axes[i, 2].grid()

plt.tight_layout()
plt.show()

# ---------------------------
# 误差图
# ---------------------------
mask = times > 1.0
fig, axes = plt.subplots(7, 1, figsize=(10, 14))
for i in range(7):
    error = q[:, i] - qd[:, i]
    axes[i].plot(times[mask], error[mask], label=f"Joint {i+1} Error")
    axes[i].set_ylabel(f"Joint {i+1} Error (rad)")
    axes[i].grid()
    axes[i].legend()

axes[-1].set_xlabel("Simulation Time (s)")
fig.suptitle("Position Tracking Errors", fontsize=14)
plt.tight_layout()
plt.show()

# ---------------------------
# 速度比例检查（可选）
# ---------------------------

mask = np.abs(dq[:, 0]) > 0.1
if np.any(mask):
    ratio = np.mean(dq[mask, 0] / dq_num[mask, 0])
    print(f"Velocity Ratio (Topic / Numeric): {ratio:.4f}")
