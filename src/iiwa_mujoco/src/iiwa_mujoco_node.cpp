#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>

using namespace std::chrono_literals;

class IiwaMujocoSim : public rclcpp::Node
{
public:
    IiwaMujocoSim()
    : Node("iiwa_mujoco_sim")
    {
        std::string xml_path = "/home/yaohouyu/ros2_mujoco_ws/src/iiwa_mujoco_ros2/models/scene.xml";
        
        char error[1000] = "Could not load binary model";

        m_ = mj_loadXML(xml_path.c_str(), 0, error, 1000);
        if (!m_) {
            RCLCPP_ERROR(this->get_logger(), "MuJoCo Load Error: %s", error);
            throw std::runtime_error("Failed to load MuJoCo model");
        }

        d_ = mj_makeData(m_);

        //设置机械臂初始位置
        int key_id = mj_name2id(m_, mjOBJ_KEY, "home");
        if (key_id >= 0) {
            mj_resetDataKeyframe(m_, d_, key_id);
        }
        
        RCLCPP_INFO(this->get_logger(), "MuJoCo Model Loaded: %s", xml_path.c_str());
        RCLCPP_INFO(this->get_logger(), "MuJoCo timestep: %.6f, n_steps per 10ms: %d", 
    m_->opt.timestep, static_cast<int>(0.01 / m_->opt.timestep));

        
        // ROS 通信
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
        pub_state_ = this->create_publisher<sensor_msgs::msg::JointState>("iiwa/joint_states", 10);
        sub_torque_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "iiwa/joint_torque", 10,
            std::bind(&IiwaMujocoSim::torque_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(1ms, std::bind(&IiwaMujocoSim::sim_step, this));
    }

    ~IiwaMujocoSim()
    {
        

        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);
        RCLCPP_INFO(this->get_logger(), "MuJoCo resources cleaned up.");
    }

private:
    void torque_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->effort.size() == 0) return;
        int num_controls = std::min((int)msg->effort.size(), m_->nu);
        for (int i = 0; i < num_controls; ++i) {
            d_->ctrl[i] = msg->effort[i];
        }
    }

    void sim_step()
    {
      
        // 物理仿真
        int n_steps = static_cast<int>(0.001 / m_->opt.timestep);
        for (int i = 0; i < n_steps; ++i) {
            mj_step(m_, d_);
        }

        //验证仿真时间和控制时间是否对齐
        // RCLCPP_INFO(this->get_logger(),
        //     "sim_time = %.6f", d_->time);

        // 发布关节状态
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = rclcpp::Time(d_->time * 1e9);

        msg.name = joint_names_;
        
        int num_joints = std::min((int)joint_names_.size(), m_->nq);
        msg.position.resize(num_joints);
        msg.velocity.resize(num_joints);
        msg.effort.resize(num_joints);

        for (int i = 0; i < num_joints; ++i) {
            msg.position[i] = d_->qpos[i];
            msg.velocity[i] = d_->qvel[i];
            if (i < m_->nu) {
                msg.effort[i] = d_->ctrl[i]; 
            }
        }
        pub_state_->publish(msg);
    }
    

    mjModel* m_ = nullptr;
    mjData* d_ = nullptr;
    GLFWwindow* window_ = nullptr;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_state_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_torque_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> joint_names_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<IiwaMujocoSim>());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}