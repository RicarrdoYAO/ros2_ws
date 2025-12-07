#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

class TorqueController : public rclcpp::Node
{
public:
    TorqueController()
    : Node("iiwa_torque_controller"),
      start_time_(this->get_clock()->now())
    {
        
        joint_names_ = {
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6", "joint7"
        };

        pub_ = this->create_publisher<sensor_msgs::msg::JointState>("iiwa/joint_torque", 10);
        timer_ = this->create_wall_timer(10ms,std::bind(&TorqueController::update, this));
    }

private:
    void update()
    {
        rclcpp::Time now = this->get_clock()->now();
        double t = (now - start_time_).seconds();
        
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = now;
        msg.name = joint_names_;

        msg.effort.resize(7, 0.0);

        msg.effort[0] = 100.0 * std::sin(1*t);
        msg.effort[1] = 2000.0 * std::sin(1.5*t);
        msg.effort[2] = 150.0 * std::sin(2*t);
        msg.effort[3] = 1150.0 * std::sin(2.5*t);
        msg.effort[4] = 150.0 * std::sin(3*t);
        msg.effort[5] = 1150.0 * std::sin(3.5*t);
        msg.effort[6] = 50.0 * std::sin(4*t);

        pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    std::vector<std::string> joint_names_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TorqueController>());
    rclcpp::shutdown();
    return 0;
}
