#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "c_pkg/msg/state_vector.hpp"

using namespace std::chrono_literals;

class Keyboard_control:public rclcpp::Node
{
    public:
        Keyboard_control():Node("keyboard_control_node")
        {
            //create pubisher that will publish message of type [vx,vy,w]
            publisher_ = this->create_publisher<c_pkg::msg::StateVector>("bot_command", 10);
            //create timer that will call repetitively the function timer_callback
            timer_ = this->create_wall_timer(500ms, std::bind(&Keyboard_control::timer_callback, this));
            RCLCPP_INFO(this->get_logger(),"\nkeyboard_control_node started...");
        } 

    private:
        //global variables    
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<c_pkg::msg::StateVector>::SharedPtr publisher_;
        
        //Functions
        void timer_callback()
        {   
            auto commands=c_pkg::msg::StateVector();

            float vx=0.0;
            float vy=0.0;
            float w=0.0;

            //filling StateVector Message
            commands.vx = vx;
            commands.vy = vy;
            commands.w = w;

            //publish commands
            publisher_->publish(commands);
        }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Keyboard_control>());
    rclcpp::shutdown();
    return 0;
}