import time
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
import mujoco
import mujoco.viewer

MODEL_XML = "/home/yaohouyu/ros2_mujoco_ws/src/iiwa_mujoco_ros2/models/scene.xml"
BAG_PATH  = "/home/yaohouyu/ros2_mujoco_ws/iiwa_bag_20251208_194604"
TOPIC     = "/iiwa/joint_states"

def load_bag(path, topic):
    reader = rosbag2_py.SequentialReader()
    storage_options   = rosbag2_py.StorageOptions(uri=path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    times, q_list = [], []
    while reader.has_next():
        tpc, data, _ = reader.read_next()
        if tpc != topic:
            continue
        msg = deserialize_message(data, JointState)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        times.append(t)
        q_list.append(list(msg.position))
    return np.array(times), np.array(q_list)

def main():
    times, q = load_bag(BAG_PATH, TOPIC)
    if len(times) == 0:
        print("bag 中未找到数据")
        return

    # 以第一帧时间为零点
    times = times - times[0]

    m = mujoco.MjModel.from_xml_path(MODEL_XML)
    d = mujoco.MjData(m)

    with mujoco.viewer.launch_passive(m, d) as v:
        start_wall = time.time()
        for i in range(len(times)):
            # 根据记录时间与墙钟同步播放
            target_wall = start_wall + times[i]
            now = time.time()
            if target_wall > now:
                time.sleep(target_wall - now)

            # 设置关节位置并前向计算
            d.qpos[:q.shape[1]] = q[i]
            d.qvel[:] = 0
            mujoco.mj_forward(m, d)

            v.sync()  # 渲染当前状态

        # 播放结束后停留窗口，按 ESC 退出
        while v.is_running():
            time.sleep(0.01)

if __name__ == "__main__":
    main()