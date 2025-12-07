#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <chrono>
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

        // PD 参数
        Kp_ = {50, 50, 50, 50, 50, 50, 50};
        Kd_ = {1, 10, 1, 5, 1, 1, 1};

        q_.resize(7, 0.0);
        dq_.resize(7, 0.0);
        qd_.resize(7, 0.0);

        initialized_ = false;

        // ========== 加载 MuJoCo 模型用于重力补偿 ==========
        char error[1000];
        m_ = mj_loadXML("/home/yaohouyu/ros2_mujoco_ws/src/iiwa_mujoco_ros2/models/scene.xml",
                        nullptr, error, 1000);
        if (!m_) {
            RCLCPP_ERROR(this->get_logger(), "MuJoCo load error: %s", error);
            throw std::runtime_error("Failed to load MuJoCo");
        }
        d_ = mj_makeData(m_);
    }

    ~IiwaPDHoldGController()
    {
        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);
    }

private:
    void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 7) return;

        q_  = msg->position;
        dq_ = msg->velocity;

        if (!initialized_) {
            qd_ = q_;
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Gravity PD hold initialized.");
        }
    }

    void control_loop()
    {
        if (!initialized_) return;

        std::vector<double> tau(7, 0.0);

        // ===== 1. 求重力补偿 g(q) =====
        mj_resetData(m_, d_);

        for (int i = 0; i < 7; ++i) {
            d_->qpos[i] = q_[i];
            d_->qvel[i] = 0.0;  // 重力项只需要 q
        }

        mj_forward(m_, d_);

        std::vector<double> g(7);
        for (int i = 0; i < 7; ++i)
            g[i] = d_->qfrc_bias[i];

        // ===== 2. PD 部分 =====
        for (int i = 0; i < 7; ++i)
        {
            double e  = qd_[i] - q_[i];
            double ed = - dq_[i];
            tau[i] = g[i] + Kp_[i] * e + Kd_[i] * ed;
        }

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.effort = tau;
        pub_torque_->publish(msg);
    }

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_state_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_torque_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 状态
    std::vector<double> q_, dq_, qd_;
    std::vector<double> Kp_, Kd_;
    bool initialized_;

    // MuJoCo 动力学模型
    mjModel* m_;
    mjData* d_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IiwaPDHoldGController>());
    rclcpp::shutdown();
    return 0;
}
