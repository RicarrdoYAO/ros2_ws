注意，使用c++的mujoco需要注意的是
1. cmake list的配置
   需要https://github.com/google-deepmind/mujoco/releases下载（到主目录下）3.3.7版本


2. 配置cmake list

3. 设置库路径export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/yaohouyu/mujoco-3.3.7-linux-x86_64/mujoco-3.3.7/lib
可以直接配置把这行配置追加到 .bashrc 文件末尾
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/yaohouyu/mujoco-3.3.7-linux-x86_64/mujoco-3.3.7/lib' >> ~/.bashrc
后
source ~/.bashrc
4. ros2 run iiwa_mujoco iiwa_mujoco_node
5. ros2 run iiwa_mujoco_torque torque_controller 
6. ros2 topic echo /iiwa/joint_states
   ros2 topic echo /iiwa/joint_states --flow-style > joint_states.json
   ros2 launch iiwa_mujoco iiwa_sim_ctrl_bag.launch.py