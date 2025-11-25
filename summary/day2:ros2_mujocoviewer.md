# Day 2: ROS2 与 MuJoCo Viewer 集成

## 系统架构

```
ROS2 C++（发布 JointState）
    ↓
ROS2 Topic /iiwa/joint_states
    ↓
Python Node（订阅 JointState）
    ↓
MuJoCo（更新 qpos）
    ↓
MuJoCo Viewer 实时显示机械臂
```

## 1. 创建 ROS2 C++ 节点：发布关节角 JointState

创建文件：`ros2_ws/src/iiwa_mujoco_ros2/src/joint_publisher.cpp`

## 2. 修改 package.xml，添加依赖

```xml
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
```

这个目前好像没什么影响

## 3. 修改 CMakeLists.txt

### 编译 joint_publisher 并安装模型文件

```cmake
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(iiwa_joint_publisher src/joint_publisher.cpp)
ament_target_dependencies(iiwa_joint_publisher rclcpp sensor_msgs)

install(TARGETS
  hello
  iiwa_joint_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)
```

**作用**：
- 编译 `joint_publisher` 可执行文件
- 安装 `models/` 目录到 install 路径
- 让 viewer 能找到 `scene.xml`

## 4. 创建 ROS2 Python 包

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python iiwa_mujoco_viewer --dependencies rclpy sensor_msgs
```

## 5. 编写 Python Viewer 节点

创建文件：`ros2_ws/src/iiwa_mujoco_viewer/iiwa_mujoco_viewer/viewer_node.py`

## 6. 修改 setup.py 注册入口点

```python
entry_points={
    'console_scripts': [
        'viewer = iiwa_mujoco_viewer.viewer_node:main',
    ],
},
```

## 7. 编译和运行

### 编译工作空间

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 运行流程（需要两个终端）

**终端 1 - 启动关节发布节点：**

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run iiwa_mujoco_ros2 iiwa_joint_publisher
```

**终端 2 - 启动 MuJoCo Viewer：**

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run iiwa_mujoco_viewer viewer
```

### 验证话题数据

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /iiwa/joint_states
```

