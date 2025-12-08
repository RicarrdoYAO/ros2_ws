#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <chrono>
#include <cmath>
#include "mujoco/mujoco.h"

using namespace std::chrono_literals;

// CTC 控制器：用 MuJoCo 逆动力学 + 正弦期望轨迹
class IiwaCTCController : public rclcpp::Node
{
public:
    IiwaCTCController()
    : Node("iiwa_ctc_controller")
    {
        sub_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "iiwa/joint_states", 10,
            std::bind(&IiwaCTCController::state_callback, this, std::placeholders::_1));

        pub_torque_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "iiwa/joint_torque", 10);

        // 1kHz 控制周期
        timer_ = this->create_wall_timer(
            1ms, std::bind(&IiwaCTCController::control_loop, this));

        // PD 参数
        Kp_ = {300.0, 200.0, 400.0, 400.0, 500.0, 500.0, 500.0};
        Kd_ = { 30.0,  30.0,  36.0,  36.0,  40.0,  40.0,  40.0};

        q_.resize(7, 0.0);
        dq_.resize(7, 0.0);
        qd_.resize(7, 0.0);
        dqd_.resize(7, 0.0);
        ddqd_.resize(7, 0.0);

        initialized_ = false;
        has_time_    = false;

        // 正弦轨迹参数
        amp_.assign(7, 0.5);   // 幅值 0.5 rad
        freq_ = {4.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1};

        // 关节力矩限幅（随便给的比较大的值，主要防炸）
        tau_limit_ = {2000, 5000, 1500, 1200, 800, 500, 400};

        // 加载 MuJoCo 模型用于逆动力学
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

    ~IiwaCTCController()
    {
        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);
    }

private:
    // ========== 订阅 iiwa/joint_states ==========
    void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 7) return;

        q_  = msg->position;
        dq_ = msg->velocity;

        // 初始化：记录时间零点 = 第一帧 joint_states 时间
        if (!initialized_) {
            initialized_ = true;
            first_state_stamp_ = msg->header.stamp;
            last_state_stamp_  = msg->header.stamp;
            has_time_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "CTC Controller (Inverse Dynamics) initialized. t0 = %.6f",
                        rclcpp::Time(first_state_stamp_).seconds());
        } else {
            last_state_stamp_ = msg->header.stamp;
            has_time_ = true;
        }
    }

    // ========== 1kHz 控制循环 ==========
    void control_loop()
    {
        if (!initialized_ || !has_time_) return;

        // ★ 关键：用 joint_states 的时间戳来做 t，保证和仿真以及画图完全对齐
        double t = last_state_stamp_.seconds();

        //验证时间是否对齐

        // RCLCPP_INFO(this->get_logger(), 
        //     "controller_t = %.6f", t);



        

        // -------- 1. 正弦期望轨迹：q_d, dq_d, ddq_d --------
        for (int i = 0; i < 7; ++i) {
            double A = amp_[i];
            double w = freq_[i];

            qd_[i]   = A * std::sin(w * t);
            dqd_[i]  = A * w * std::cos(w * t);
            ddqd_[i] = -A * w * w * std::sin(w * t);
        }

        // -------- 2. 准备 MuJoCo 逆动力学输入 --------
        // tau = M(q)*(ddqd + Kp*e + Kd*ed) + C(q,dq)*dq + g(q)
        for (int i = 0; i < 7; ++i) {
            d_->qpos[i] = q_[i];
            d_->qvel[i] = dq_[i];  // 必须是真实速度

            double e  = qd_[i]  - q_[i];
            double ed = dqd_[i] - dq_[i];

            double v = ddqd_[i] + Kp_[i] * e + Kd_[i] * ed;

            d_->qacc[i] = v;  // 目标加速度
        }

        // -------- 3. MuJoCo 逆动力学 --------
        mj_inverse(m_, d_);   // 结果在 d_->qfrc_inverse

        // -------- 4. 限幅并发布 --------
        std::vector<double> tau(7);
        for (int i = 0; i < 7; ++i) {
            double out = d_->qfrc_inverse[i];
            if (out >  tau_limit_[i]) out =  tau_limit_[i];
            if (out < -tau_limit_[i]) out = -tau_limit_[i];
            tau[i] = out;
        }

        

        sensor_msgs::msg::JointState out_msg;
        out_msg.header.stamp = last_state_stamp_;  // 用状态时间戳更严谨
        out_msg.effort = tau;
        pub_torque_->publish(out_msg);
    }

    // ROS 相关
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_state_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    pub_torque_;
    rclcpp::TimerBase::SharedPtr                                  timer_;

    // 状态 / 期望
    std::vector<double> q_, dq_, qd_, dqd_, ddqd_;
    std::vector<double> Kp_, Kd_;
    bool initialized_;

    // 时间对齐用
    bool         has_time_;
    rclcpp::Time first_state_stamp_;
    rclcpp::Time last_state_stamp_;

    // 期望轨迹
    std::vector<double> amp_;
    std::vector<double> freq_;

    // 力矩限幅
    std::vector<double> tau_limit_;

    // MuJoCo 逆动力学模型
    mjModel* m_;
    mjData*  d_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IiwaCTCController>());
    rclcpp::shutdown();
    return 0;
}
