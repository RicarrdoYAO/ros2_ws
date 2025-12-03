#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>

class TorqueController : public rclcpp::Node
{
public:
    TorqueController()
    : Node("iiwa_torque_controller"),
      start_time_(this->get_clock()->now())
    {
        pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "iiwa/joint_torque_cmd", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TorqueController::update, this));
    }

private:
    void update()
    {
        rclcpp::Time now = this->get_clock()->now();
        double t = (now - start_time_).seconds();

        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(7);

        msg.data[0] = 50.0 * std::sin(t);
        for(int i=1; i<7; i++)
            msg.data[i] = 0.0;

        pub_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TorqueController>());
    rclcpp::shutdown();
    return 0;
}
