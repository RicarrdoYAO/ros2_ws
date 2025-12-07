cmake_minimum_required(VERSION 3.8)
project(iiwa_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies (ROS2 packages)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(glfw3 REQUIRED)
# -------------------------------------------------------------
# 2. MuJoCo Configuration (Extra setup for external library)
# -------------------------------------------------------------
# 设置 MuJoCo 的根目录
set(MUJOCO_PATH "/home/yaohouyu/mujoco-3.3.7-linux-x86_64/mujoco-3.3.7")

# 告诉编译器去哪里找 mujoco.h
include_directories(${MUJOCO_PATH}/include)

# 告诉链接器去哪里找 libmujoco.so
link_directories(${MUJOCO_PATH}/lib)
# -------------------------------------------------------------


# 3. Define Executable
add_executable(iiwa_mujoco_node src/iiwa_mujoco_node.cpp)

# 4. Link ROS dependencies (This fixes rclcpp.hpp error)
# 这行代码会自动处理 rclcpp 和 sensor_msgs 的头文件路径
ament_target_dependencies(iiwa_mujoco_node rclcpp sensor_msgs)

add_executable(iiwa_controller_node src/iiwa_controller_node.cpp)
ament_target_dependencies(iiwa_controller_node rclcpp sensor_msgs)

install(TARGETS
  iiwa_controller_node
  DESTINATION lib/${PROJECT_NAME})


# 5. Link MuJoCo library
# 链接 libmujoco.so
target_link_libraries(iiwa_mujoco_node mujoco)
target_link_libraries(iiwa_controller_node mujoco)


target_link_libraries(iiwa_mujoco_node
  ${MUJOCO_LIBRARIES}
  glfw
  GL
)


# 6. Install Rules
install(TARGETS
  iiwa_mujoco_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(iiwa_pd_hold_node src/iiwa_pd_hold_node.cpp)
ament_target_dependencies(iiwa_pd_hold_node rclcpp sensor_msgs)
target_link_libraries(iiwa_pd_hold_node mujoco)

install(TARGETS iiwa_pd_hold_node DESTINATION lib/${PROJECT_NAME})


# 如果你有 launch 文件夹或 models 文件夹需要安装，取消下面的注释
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()