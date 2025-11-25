# Day 1: ROS2 与 MuJoCo 初始化配置

## 1. 创建 ROS2 Jazzy 工作空间

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

## 2. 创建 C++ ROS2 包

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake iiwa_mujoco_ros2
```
笔记：
ros2 pkg create                 # 创建新的 ROS2 包
--build-type ament_cmake        # 用 CMake 构建，如python就用ament_python
iiwa_mujoco_ros2                # 包名





## 3. 配置 CMakeLists.txt

在 `CMakeLists.txt` 中添加

```cmake
find_package(rclcpp REQUIRED)

add_executable(hello src/hello.cpp)
ament_target_dependencies(hello rclcpp)

install(TARGETS
  hello
  DESTINATION lib/${PROJECT_NAME}
)
```

先find package 找到ros2的c++api
add_executable(可执行文件名字 源代码路径),把cpp编译成一个可执行文件，名叫hello
ament_target_dependencies(hello rclcpp)  可执行文件hello依赖rclcpp库，自动链接

install(TARGETS
  hello
  DESTINATION lib/${PROJECT_NAME}
)相当于把hello 这个可执行程序安装到install/lib/iiwa_mujoco_ros2/hello，可以直接ros2 run



## 4. 将 MuJoCo 模型加入 ROS2 包

直接复制粘贴打包到src下的包中的models文件夹中即可


## 5. 修复 Conda 与 ROS2 的 Python 冲突

### 修改 `.bashrc` 配置

将 `.bashrc` 中的 conda 部分替换为:

```bash
# >>> conda initialize (manual activation mode) >>>
# Conda will only load when you explicitly set CONDA_AUTOLOAD=1
if [ "$CONDA_AUTOLOAD" = "1" ]; then
    __conda_setup="$('/home/yaohouyu/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
    if [ $? -eq 0 ]; then
        eval "$__conda_setup"
    elif [ -f "/home/yaohouyu/anaconda3/etc/profile.d/conda.sh" ]; then
        . "/home/yaohouyu/anaconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/yaohouyu/anaconda3/bin:$PATH"
    fi
    unset __conda_setup
fi
# <<< conda initialize <<<
```

### 启用 Conda 环境

```bash
export CONDA_AUTOLOAD=1
source ~/.bashrc
conda activate kuka_simulation
```

### 清理被 Conda 污染的构建文件

```bash
cd ~/ros2_ws
rm -rf build install log
```

## 6. 第一个 ROS2 C++ 节点

创建 `src/hello.cpp` 文件并编写 hello_node 节点。

## 7. 编译和运行

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run iiwa_mujoco_ros2 hello
```

## 8. 修复 VSCode 配置

### 解决 "无法打开 rclcpp/rclcpp.hpp" 错误

1. 按 `Ctrl+Shift+P`
2. 选择 "C/C++: Edit Configurations (JSON)"
3. 添加以下配置:

```json
{
    "configurations": [
        {
            "name": "ROS2",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/jazzy/include/**",
                "/usr/include/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/g++",
            "cStandard": "gnu17",
            "cppStandard": "gnu++17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```

