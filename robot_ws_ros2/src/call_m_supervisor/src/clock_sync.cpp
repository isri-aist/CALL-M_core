#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class ClockSync : public rclcpp::Node {
public:
    ClockSync() : Node("clock_publisher") {
        publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ClockSync::publishTime, this));
        clock = this->get_clock();
    }

private:
    void publishTime() {
        auto message = rosgraph_msgs::msg::Clock();
        message.clock = clock->now();
        publisher_->publish(message);
    }

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr clock; 
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClockSync>());
    rclcpp::shutdown();
    return 0;
}
