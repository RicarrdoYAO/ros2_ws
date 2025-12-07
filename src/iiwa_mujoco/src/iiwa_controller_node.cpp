#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <chrono>
#include <cmath>
#include "mujoco/mujoco.h"

using namespace std::chrono_literals;

class IiwaPDHoldGController : public rclcpp::Node
{
public:
    IiwaPDHoldGController()
    : Node("iiwa_pd_hold_gravity_controller")
    {
        sub_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "iiwa/joint_states", 10,
            std::bind(&IiwaPDHoldGController::state_callback, this, std::placeholders::_1));

        pub_torque_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "iiwa/joint_torque", 10);

        timer_ = this->create_wall_timer(
            5ms, std::bind(&IiwaPDHoldGController::control_loop, this));

        // ---------------- PD 增益 ----------------
        Kp_ = {10, 10, 10, 10, 10, 10, 5};
        Kd_ = { 1,  1,  1,  1,  1,  1,  0.5};

        q_.resize(7, 0.0);
        dq_.resize(7, 0.0);
        qd_.resize(7, 0.0);
        dqd_.resize(7, 0.0);

        initialized_ = false;

        // ---------------- 正弦轨迹参数 ----------------
        amp_.assign(7, 0.5);   // 每关节最大幅值 0.5 rad
        freq_ = {0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 0.1};

        start_time_ = this->now();

        // ---------------- 力矩限幅（iiwa7 实际参数） ----------------
        tau_limit_ = {200, 500, 150, 120, 80, 50, 40};

        // ----------- 加载 MuJoCo 模型用于动力学计算 -----------
        char error[1000];
        m_ = mj_loadXML(
            "/home/yaohouyu/ros2_mujoco_ws/src/iiwa_mujoco_ros2/models/scene.xml",
            nullptr, error, 1000);

        if (!m_) {
            RCLCPP_ERROR(this->get_logger(), "MuJoCo load error: %s", error);
            throw std::runtime_error("Failed to load MuJoCo model");
        }

        d_ = mj_makeData(m_);
    }

    ~IiwaPDHoldGController()
    {
        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);
    }

private:

    // ====================== 接收 joint_states ======================
    void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 7) return;

        q_  = msg->position;
        dq_ = msg->velocity;

        if (!initialized_) {
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Gravity PD + sine tracking initialized.");
        }
    }

    // ============================ 控制循环 ============================
    void control_loop()
    {
        if (!initialized_) return;

        double t = (this->now() - start_time_).seconds();

        // ----------- 1. 正弦轨迹生成 -----------
        for (int i = 0; i < 7; ++i)
        {
            double A = amp_[i];
            double w = freq_[i];

            qd_[i]  = A * sin(w * t);
            dqd_[i] = A * w * cos(w * t);
        }

        // ----------- 2. 计算重力补偿 g(q) -----------
        for (int i = 0; i < 7; ++i) {
            d_->qpos[i] = q_[i];
            d_->qvel[i] = 0;   // 只要重力项，不要科氏力
        }

        mj_forward(m_, d_); // 更新动力学

        std::vector<double> g(7);
        for (int i = 0; i < 7; ++i)
            g[i] = d_->qfrc_bias[i];

        // ----------- 3. PD + 力矩限幅 -----------
        std::vector<double> tau(7);

        for (int i = 0; i < 7; ++i)
        {
            double e  = qd_[i]  - q_[i];
            double ed = dqd_[i] - dq_[i];

            double out = g[i] + Kp_[i] * e + Kd_[i] * ed;

            // 关节独立限幅（方案 C）
            if (out >  tau_limit_[i]) out =  tau_limit_[i];
            if (out < -tau_limit_[i]) out = -tau_limit_[i];

            tau[i] = out;
        }

        // ----------- 4. 发布力矩 -----------
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.effort = tau;

        pub_torque_->publish(msg);
    }

    // ------------------- ROS 成员变量 -------------------
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_state_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    pub_torque_;
    rclcpp::TimerBase::SharedPtr                                  timer_;

    std::vector<double> q_, dq_, qd_, dqd_;
    std::vector<double> Kp_, Kd_;
    bool initialized_;

    // 正弦轨迹
    std::vector<double> amp_;
    std::vector<double> freq_;
    rclcpp::Time start_time_;

    // 力矩限幅
    std::vector<double> tau_limit_;

    // MuJoCo
    mjModel* m_;
    mjData*  d_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IiwaPDHoldGController>());
    rclcpp::shutdown();
    return 0;
}
