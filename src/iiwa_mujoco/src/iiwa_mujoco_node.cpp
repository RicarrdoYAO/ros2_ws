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
        std::string xml_path =
            "/home/yaohouyu/ros2_mujoco_ws/src/iiwa_mujoco_ros2/models/scene.xml";
        
        char error[1000] = "Could not load binary model";

        m_ = mj_loadXML(xml_path.c_str(), 0, error, 1000);
        if (!m_) {
            RCLCPP_ERROR(this->get_logger(), "MuJoCo Load Error: %s", error);
            throw std::runtime_error("Failed to load MuJoCo model");
        }

        d_ = mj_makeData(m_);

        // 设置机械臂初始位置（keyframe = home）
        int key_id = mj_name2id(m_, mjOBJ_KEY, "home");
        if (key_id >= 0) {
            mj_resetDataKeyframe(m_, d_, key_id);
        }

        // 打印 timestep
        RCLCPP_INFO(
            this->get_logger(),
            "MuJoCo loaded. Timestep = %.6f sec (expected 0.001)",
            m_->opt.timestep
        );

        // ============ 初始化可视化 ============
        if (!glfwInit()) {
            throw std::runtime_error("GLFW init failed");
        }

        window_ = glfwCreateWindow(1200, 900, "IIWA MuJoCo Simulation", NULL, NULL);
        if (!window_) {
            glfwTerminate();
            throw std::runtime_error("GLFW window creation failed");
        }

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);   // VSYNC

        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);

        mjv_makeScene(m_, &scn_, 2000);
        mjr_makeContext(m_, &con_, mjFONTSCALE_150);

        // 固定相机视角
        cam_.lookat[0] = 0.0;
        cam_.lookat[1] = 0.0;
        cam_.lookat[2] = 0.5;
        cam_.distance   = 3.0;
        cam_.azimuth    = 135;
        cam_.elevation  = -20;

        // ROS topic 设置
        joint_names_ = {"joint1","joint2","joint3","joint4","joint5","joint6","joint7"};

        pub_state_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "iiwa/joint_states",
            10
        );

        sub_torque_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "iiwa/joint_torque",
            10,
            std::bind(&IiwaMujocoSim::torque_callback, this, std::placeholders::_1)
        );

        // ============ 关键修改：1ms 控制周期，与 MuJoCo timestep 完全同步 ============
        timer_ = this->create_wall_timer(
            1ms,
            std::bind(&IiwaMujocoSim::sim_step, this)
        );

        // 渲染 decimation（每 10 次 sim 才渲染一次）
        render_decimation_ = 10;
        cycle_count_ = 0;
    }

    ~IiwaMujocoSim()
    {
        mjv_freeScene(&scn_);
        mjr_freeContext(&con_);
        glfwDestroyWindow(window_);
        glfwTerminate();

        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);
    }

private:

    // ================= torque 输入（CTC 输出） =================
    void torque_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->effort.size() == 0) return;

        int N = std::min((int)msg->effort.size(), m_->nu);
        for (int i = 0; i < N; i++)
            d_->ctrl[i] = msg->effort[i];
    }

    // ================= 主仿真 loop（1ms） =================
    void sim_step()
    {
        if (glfwWindowShouldClose(window_)) {
            rclcpp::shutdown();
            return;
        }

        // ========= 真正关键：每次 timer 推进一步（1ms）仿真 =========
        mj_step(m_, d_);

        cycle_count_++;

        // ========= 降低渲染频率（每 10 步渲染一次） =========
        if (cycle_count_ % render_decimation_ == 0)
        {
            int width, height;
            glfwGetFramebufferSize(window_, &width, &height);

            mjv_updateScene(m_, d_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
            mjrRect viewport = {0, 0, width, height};
            mjr_render(viewport, &scn_, &con_);

            glfwSwapBuffers(window_);
            glfwPollEvents();
        }

        // ========= 发布 joint_states（1kHz） =========
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name = joint_names_;

        int J = m_->nq;
        msg.position.resize(J);
        msg.velocity.resize(J);
        msg.effort.resize(J);

        for (int i = 0; i < J; i++) {
            msg.position[i] = d_->qpos[i];
            msg.velocity[i] = d_->qvel[i];
            if (i < m_->nu)
                msg.effort[i] = d_->ctrl[i];
        }

        pub_state_->publish(msg);
    }

    // ================= 数据成员 =================
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

    int render_decimation_;
    int cycle_count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IiwaMujocoSim>());
    rclcpp::shutdown();
    return 0;
}
