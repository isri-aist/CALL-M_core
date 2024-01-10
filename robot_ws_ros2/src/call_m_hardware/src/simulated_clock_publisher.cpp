#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class SimulatedClockPublisher : public rclcpp::Node {
public:
  SimulatedClockPublisher() : Node("simulated_clock_publisher") {
    publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&SimulatedClockPublisher::publishClock, this));

    clock = this->get_clock();
    t0 = clock->now();

  }

private:
  void publishClock() {
    auto message = rosgraph_msgs::msg::Clock();
    tf = clock->now();

    // Calculate the time difference as a duration
    rclcpp::Duration duration = tf - t0;

    // Assign the duration to the message.clock field
    message.clock.sec = duration.seconds();
    message.clock.nanosec = duration.nanoseconds() % 1000000000;

    publisher_->publish(message);
  }

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock::SharedPtr clock;
  rclcpp::Time t0;
  rclcpp::Time tf;

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatedClockPublisher>());
  rclcpp::shutdown();
  return 0;
}
