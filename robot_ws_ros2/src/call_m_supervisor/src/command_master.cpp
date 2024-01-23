#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

bool is_command(geometry_msgs::msg::Twist msg){
    bool result = msg.linear.x != 0 || msg.linear.y != 0 || msg.angular.z != 0;
    return result;
}

bool is_same(geometry_msgs::msg::Twist prev_msg,geometry_msgs::msg::Twist new_msg){
    bool result = prev_msg.linear.x == new_msg.linear.x && prev_msg.linear.y == new_msg.linear.y && prev_msg.angular.z == new_msg.angular.z;
    return result;
}

void init_twist(geometry_msgs::msg::Twist &msg){
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
}

class CommandMasterNode : public rclcpp::Node
{
public:
  CommandMasterNode() : Node("command_master_node")
  {
    // Initialize subscribers
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    sub_teleop_key = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_teleop_key", sensor_qos, std::bind(&CommandMasterNode::twistCallback_key, this, std::placeholders::_1));

    sub_teleop_joy = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_teleop_joy", sensor_qos, std::bind(&CommandMasterNode::twistCallback_joy, this, std::placeholders::_1));

    sub_nav = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_nav", sensor_qos, std::bind(&CommandMasterNode::twistCallback_nav, this, std::placeholders::_1));

    // Initialize publisher
    pub_command = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_apply", default_qos); //QOS to reliable

    // Initialize variables to store received twist messages
    init_twist(twist_teleop_key);
    init_twist(twist_teleop_joy);
    init_twist(twist_nav);

    // Set a timer to publish the selected twist message periodically
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&CommandMasterNode::publishTwist, this));
  }

private:
  void twistCallback_key(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    //MAIN CODE to get commands from keyboard
    if(!is_same(twist_teleop_key,*msg)){twist_teleop_key = *msg;}
    teleop_key_active=0;
  }

  void twistCallback_joy(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    //MAIN CODE to get commands from joystick
    if(!is_same(twist_teleop_joy,*msg)){twist_teleop_joy= *msg;}
    teleop_joy_active=0;
    //RCLCPP_INFO(this->get_logger(),"command_joystick_received...");
  }
  
  void twistCallback_nav(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    //MAIN CODE to get commands from nav2 plugin
    if(!is_same(twist_nav,*msg)){twist_nav = *msg;} 
    nav_active =0;  
  }

  void publishTwist()
  {
    geometry_msgs::msg::Twist selected_twist;

    //check if a subscriber is active
    if(teleop_key_active >= iteration_to_reset){init_twist(twist_teleop_key);}
    if(teleop_joy_active >= iteration_to_reset){init_twist(twist_teleop_joy);}
    if(nav_active >= iteration_to_reset){init_twist(twist_nav);}

    teleop_key_active ++; //to avoid int memory limit
    teleop_joy_active ++;
    nav_active ++;

    //choice of linear commands to apply
    if(is_command(twist_teleop_key)){
      selected_twist=twist_teleop_key;
      //RCLCPP_INFO(this->get_logger(),"Keyboard controlling...");
      }
    else if(is_command(twist_teleop_joy)){
      selected_twist=twist_teleop_joy;
      //RCLCPP_INFO(this->get_logger(),"Joystick controlling...");
      }
    else if(is_command(twist_nav)){
      selected_twist=twist_nav;
      //RCLCPP_INFO(this->get_logger(),"Nav2 controlling...");
      }
    else{
      init_twist(selected_twist);
      //RCLCPP_INFO(this->get_logger(),"Waiting commands...");
      }

    // Publish the selected twist message
    pub_command->publish(selected_twist);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_teleop_key; 
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_teleop_joy;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_command;

  geometry_msgs::msg::Twist twist_teleop_key;
  geometry_msgs::msg::Twist twist_teleop_joy;
  geometry_msgs::msg::Twist twist_nav;

// Flags to indicate if new messages have been received on each topic, if no message have been received for an amount of time, we reset the command.
  int teleop_key_active = 0;
  int teleop_joy_active = 0;
  int nav_active = 0;
  int iteration_to_reset = 100; //= 1 sec if publisher frequence is 10ms

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandMasterNode>());
  rclcpp::shutdown();
  return 0;
}
