#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <chrono>
#include <cmath>
#include "mujoco/mujoco.h"

using namespace std::chrono_literals;

// 类名改为了 CTCController 以区分
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

        timer_ = this->create_wall_timer(
            1ms, std::bind(&IiwaCTCController::control_loop, this));



        Kp_ = {300.0, 200.0, 400.0, 400.0, 500.0, 500.0, 500.0};
        Kd_ = { 30.0,  30.0,  36.0,  36.0,  40.0,  40.0,  40.0};



        q_.resize(7, 0.0);
        dq_.resize(7, 0.0);
        qd_.resize(7, 0.0);
        dqd_.resize(7, 0.0);
        ddqd_.resize(7, 0.0); // [新增] 期望加速度

        initialized_ = false;

        // ---------------- 正弦轨迹参数 ----------------
        amp_.assign(7, 0.5);   // 每关节最大幅值 0.5 rad
        freq_ = {0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1};

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

    ~IiwaCTCController()
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
            RCLCPP_INFO(this->get_logger(), "CTC Controller (Inverse Dynamics) initialized.");
        }
    }

    // ============================ 控制循环 (CTC 核心) ============================
    void control_loop()
    {
        if (!initialized_) return;

        double t = (this->now() - start_time_).seconds();

        // ----------- 1. 正弦轨迹生成 (Pos, Vel, Acc) -----------
        for (int i = 0; i < 7; ++i)
        {
            double A = amp_[i];
            double w = freq_[i];

            // 位置 q_d
            qd_[i]  = A * sin(w * t);
            
            // 速度 dq_d = d/dt(Pos)
            dqd_[i] = A * w * cos(w * t);
            
            // 加速度 ddq_d = d/dt(Vel) [新增]
            ddqd_[i] = -A * w * w * sin(w * t); 
        }

        // ----------- 2. 准备逆动力学输入 -----------
        // 公式：tau = M(q) * (ddqd + Kp*e + Kd*ed) + C(q,dq)*dq + g(q)
        // 在 MuJoCo 中，我们只需要把 "ddqd + Kp*e + Kd*ed" 填入 d_->qacc
        // 并把当前真实状态 q, dq 填入 d_->qpos, d_->qvel
        // mj_inverse 会自动完成上述公式计算

        for (int i = 0; i < 7; ++i) {
            // 2.1 填入真实状态
            d_->qpos[i] = q_[i];
            d_->qvel[i] = dq_[i]; // [关键修改] 这里必须是真实速度，用于计算科氏力/离心力

            // 2.2 计算误差
            double e  = qd_[i]  - q_[i];
            double ed = dqd_[i] - dq_[i];

            // 2.3 计算虚拟控制量 (目标加速度)
            double v = ddqd_[i] + Kp_[i] * e + Kd_[i] * ed;

            // 2.4 填入 MuJoCo 加速度槽
            d_->qacc[i] = v;
        }

        // ----------- 3. 调用 MuJoCo 逆动力学 -----------
        // 这步计算完成后，结果存储在 d_->qfrc_inverse 中
        mj_inverse(m_, d_); 

        // ----------- 4. 获取力矩并限幅 -----------
        std::vector<double> tau(7);

        for (int i = 0; i < 7; ++i)
        {
            // 从 qfrc_inverse 读取结果
            double out = d_->qfrc_inverse[i];

            // 关节独立限幅
            if (out >  tau_limit_[i]) out =  tau_limit_[i];
            if (out < -tau_limit_[i]) out = -tau_limit_[i];

            tau[i] = out;
        }

        // ----------- 5. 发布力矩 -----------
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.effort = tau;

        pub_torque_->publish(msg);
    }

    // ------------------- ROS 成员变量 -------------------
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_state_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    pub_torque_;
    rclcpp::TimerBase::SharedPtr                                  timer_;

    std::vector<double> q_, dq_, qd_, dqd_, ddqd_; // [修改] 增加了 ddqd_
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
    mjData* d_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IiwaCTCController>());
    rclcpp::shutdown();
    return 0;
}