#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class ClockSync : public rclcpp::Node {
public:
    ClockSync() : Node("clock_sync_node") {
        initialize_params();
        refresh_params();
        if(!this->sim_time){
            publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ClockSync::publishTime, this));
            clock = this->get_clock();
            RCLCPP_INFO(this->get_logger(), "/clock to sync other computers published.");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "No need to publish /clock to sync other computers, simulator already manage it.");
        }
    }

private:
    void publishTime() {
        auto message = rosgraph_msgs::msg::Clock();
        message.clock = clock->now();
        publisher_->publish(message);
    }

    void initialize_params(){
        this->declare_parameter("sim_time",false);
    }

    void refresh_params(){
        get_parameter("sim_time",sim_time);
    }

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr clock; 
    bool sim_time;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClockSync>());
    rclcpp::shutdown();
    return 0;
}
