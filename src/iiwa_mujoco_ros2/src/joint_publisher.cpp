#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <vector>
#include <string>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// 发布关节角度的节点
class IiwaJointPublisher : public rclcpp::Node
{
public:
    IiwaJointPublisher()
    : Node("iiwa_joint_publisher"),
      start_time_(this->now())
    {
        // 关节
        joint_names_ = {
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6", 
            "joint7"
        };

        // 创建一个发布器，话题名 iiwa/joint_states
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("iiwa/joint_states", 10);

        // 创建一个 10 ms 触发一次的定时器 100 Hz
        timer_ = this->create_wall_timer(10ms, std::bind(&IiwaJointPublisher::on_timer, this));
    }

private:
    void on_timer()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->get_clock()->now();
        msg.name = joint_names_;

        
        double t = (this->now() - start_time_).seconds();

        // 正弦关节运动
        msg.position.resize(joint_names_.size());
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            msg.position[i] = 0.1* i * std::sin(10*t + i * 0.5);  
        }

        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    std::vector<std::string> joint_names_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IiwaJointPublisher>());
    rclcpp::shutdown();
    return 0;
}
