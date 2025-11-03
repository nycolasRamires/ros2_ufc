#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class WheelRefPublisher : public rclcpp::Node {
public:
    WheelRefPublisher() : Node("wheel_ref_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_position_controller/commands", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&WheelRefPublisher::publish_reference, this));

        direction_ = 1.0;
        RCLCPP_INFO(this->get_logger(), "Wheel reference publisher started.");
    }

private:
    void publish_reference() {
        auto msg = std_msgs::msg::Float64MultiArray();

        // Alterna entre +0.5 rad e -0.5 rad
        msg.data = {direction_ * 0.5, -direction_ * 0.5};

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
                    "Published wheel references: [%.2f, %.2f]",
                    msg.data[0], msg.data[1]);

        direction_ *= -1.0;  // inverte direção a cada pub
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double direction_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelRefPublisher>());
    rclcpp::shutdown();
    return 0;
}
