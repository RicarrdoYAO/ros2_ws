import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

bag_path = "/home/yaohouyu/ros2_mujoco_ws/iiwa_bag_20251208_151447"   

reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
converter_options = rosbag2_py.ConverterOptions("", "")
reader.open(storage_options, converter_options)

times, q_list, dq_list, tau_list = [], [], [], []

start_time = None

while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic != "/iiwa/joint_states":
        continue

    msg = deserialize_message(data, JointState)

    t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    if start_time is None:
        start_time = t_sec

    times.append(t_sec - start_time)
    q_list.append(msg.position)
    dq_list.append(msg.velocity)
    tau_list.append(msg.effort)

times = np.array(times)
q = np.array(q_list)
dq = np.array(dq_list)
tau = np.array(tau_list)

# 使用 joint_states.velocity 作为真实速度
dq_real = dq  # 因为我们从 bag 里读出来的 dq 就是 msg.velocity

plt.figure()
plt.plot(times, dq_real[:,0], label="dq(topic)")
plt.plot(times, dq[:,0], "--", label="dq_ref")
plt.legend()
plt.title("Velocity Check (Joint 1)")
plt.show()

dt = 0.001  # 你的 MuJoCo timestep

dq_idx = np.zeros_like(q)
dq_idx[1:, :] = (q[1:, :] - q[:-1, :]) / dt

plt.figure()
plt.plot(times, dq[:,0], label="dq(topic)")   # 来自 joint_states.velocity
plt.plot(times, dq_idx[:,0], "--", label="dq_from_q_fixed_dt")
plt.legend()
plt.grid()
plt.show()



# 正弦轨迹
amp = np.array([0.5]*7)
freq = np.array([0.5,0.6,0.7,0.8,0.9,1.0,1.1])

qd  = amp * np.sin(freq * times[:,None])
# 对参考位置求导得到参考速度
dqd = np.zeros_like(qd)
for i in range(7):
    dqd[:,i] = np.gradient(qd[:,i], times)

# 对实际位置求导得到数值速度
dq_num = np.zeros_like(q)
for i in range(7):
    dq_num[:,i] = np.gradient(q[:,i], times)

plt.figure()
plt.plot(times, dq[:,0], label="dq(topic)")
plt.plot(times, dq_num[:,0], "--", label="dq_num")
plt.legend()
plt.title("Velocity Check (Joint 1)")
plt.show()

# 7x3 plot
fig, axes = plt.subplots(7, 3, figsize=(16,18))
for i in range(7):
    axes[i,0].plot(times, q[:,i], label="real")
    axes[i,0].plot(times, qd[:,i], "--", label="ref")
    axes[i,0].grid()

   # 第二列：速度
    axes[i,1].plot(times, dq_real[:,i], label="real (topic)")
    axes[i,1].plot(times, dqd[:,i], "--", label="ref")
    axes[i,1].grid()


    axes[i,2].plot(times, tau[:,i], label="tau")
    axes[i,2].grid()

plt.tight_layout()
plt.show()


fig, axes = plt.subplots(7, 1, figsize=(10, 14))
for i in range(7):
    error = q[:,i] - qd[:,i]
    axes[i].plot(times, error, label=f"Joint {i+1} Error")
    axes[i].set_ylabel(f"Joint {i+1} Error (rad)")
    axes[i].grid()
    axes[i].legend()

axes[-1].set_xlabel("Time (s)")
fig.suptitle("Position Tracking Errors", fontsize=14)
plt.tight_layout()
plt.show()