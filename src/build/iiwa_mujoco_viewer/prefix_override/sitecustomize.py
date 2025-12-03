import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yaohouyu/ros2_ws/src/install/iiwa_mujoco_viewer'
