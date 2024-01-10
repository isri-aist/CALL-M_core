#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class SimulatedClockPublisher : public rclcpp::Node {
public:
  SimulatedClockPublisher() : Node("simulated_clock_publisher") {
    publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&SimulatedClockPublisher::publishClock, this));
  }

private:
  void publishClock() {
    auto message = rosgraph_msgs::msg::Clock();
    message.clock = this->now();
    publisher_->publish(message);
  }

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatedClockPublisher>());
  rclcpp::shutdown();
  return 0;
}
