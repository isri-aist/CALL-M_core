#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

double clamp_angle(double angle){
  //put angle between 0 and 2pi
  angle = std::fmod(angle, 2 * M_PI);
  if (angle < 0) {
      angle += 2 * M_PI;
  }
  return angle;
}

int angle_to_index(double alpha, int resolution){
    //return index of angle alpha, in a table with 'resolution' values placed from 0 to 360 angles.
    // Normalize the angle to the range [0, 2*M_PI)
    alpha = clamp_angle(alpha);
    // Calculate the index
    return static_cast<int>(round((alpha * resolution) / (2 * M_PI)));
}

double index_to_angle(int ind, int resolution){
    return (ind*(2*M_PI))/resolution;
}

std::vector<double> subpoints(sensor_msgs::msg::LaserScan data,int i,int y){
  std::vector<double> tempo = {};
  for (int ind = i; ind <= y; ++ind){
    tempo.push_back(data.ranges[ind]);
  }
  return tempo;
}

double get_points(sensor_msgs::msg::LaserScan data,double start_angle,double end_angle,double r1,double r2,double field_of_view){
  //DEBUG START
  /*double li_deb = 0.0;
  double di_deb = 9999.0;
  int ind_deb = 0;
  double alpha_deb = 0.0;
  double point_deb = 0.0;
  int count_deb = 0;*/
  //DEBUG END
  double di_min = INFINITY;
  
  std::vector<double> points = {};
  int resolution = data.ranges.size();
  double start_ind = angle_to_index(start_angle,resolution);
  double end_ind = angle_to_index(end_angle,resolution);
  //RCLCPP_INFO(this->get_logger(), "cmd_index_start: %f",start_ind);
  //RCLCPP_INFO(this->get_logger(), "cmd_index_end: %f",end_ind);
  if(start_ind > end_ind){
    std::vector<double> tempo = subpoints(data,0,end_ind);
    points.insert(points.begin(),tempo.begin(),tempo.end());
    std::vector<double> tempo2 = subpoints(data,start_ind,resolution-1);
    points.insert(points.begin(),tempo2.begin(),tempo2.end());
  }
  else{
    std::vector<double> tempo = subpoints(data,start_ind,end_ind);
    points.insert(points.begin(),tempo.begin(),tempo.end());
  }
  //filter points and throwing those that are not in the rectangle
  int reso2 = round((2*M_PI*float(points.size()))/field_of_view); //because alpha local should not have values from 0 to 'field of view'
  //RCLCPP_INFO(this->get_logger(), "reso2: %f",float(resolution)/float(points.size()));
  //RCLCPP_INFO(this->get_logger(), "reso2: %d",reso2);
  for (int ind = 0; ind < points.size(); ++ind){
    double alpha_local = index_to_angle(ind,reso2);
    //RCLCPP_INFO(this->get_logger(), "alpha_local: %f",alpha_local);
    double di = points[ind];
    double li = r2; //limit of rectangle area
    if(alpha_local != M_PI/2 - field_of_view/2){
      li = std::min(sqrt(pow(r1,2)+pow(r2,2)),r1/abs(cos(alpha_local+field_of_view/2)));
    }
    if(di > li){
      points[ind] = INFINITY;
    }

    if(points[ind] < di_min){
      di_min = points[ind];
    }

    //DEBUG START
    /*if(di <= di_deb){
      li_deb = li;
      di_deb = di;
      ind_deb = ind;
      alpha_deb = alpha_local;
      point_deb = points[ind];
    }
    count_deb +=1;*/
    //DEBUG END
  }

  //RCLCPP_INFO(this->get_logger(), "li_deb: %f",li_deb);
  //RCLCPP_INFO(this->get_logger(), "di_deb: %f",di_deb);
  //RCLCPP_INFO(this->get_logger(), "ind_deb: %d",ind_deb);
  //RCLCPP_INFO(this->get_logger(), "alpha_deb: %f",alpha_deb*180/M_PI);
  //RCLCPP_INFO(this->get_logger(), "point_deb: %f",point_deb);
  //RCLCPP_INFO(this->get_logger(), "count_deb: %d",count_deb);
  
  return di_min;
  //return points;
}


bool proximity(sensor_msgs::msg::LaserScan data, double distance){
  for (int ind = 0; ind < data.ranges.size(); ++ind){
    if(data.ranges[ind] < distance){
      return true;
    }
  }
  return false;
}

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
      "cmd_vel_teleop", sensor_qos, std::bind(&CommandMasterNode::twistCallback_joy, this, std::placeholders::_1));

    sub_nav = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_nav", sensor_qos, std::bind(&CommandMasterNode::twistCallback_nav, this, std::placeholders::_1));

    sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", sensor_qos, std::bind(&CommandMasterNode::scanCallback, this, std::placeholders::_1));

    // Initialize publisher
    pub_command = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_apply", default_qos); //QOS to reliable

    // Initialize variables to store received twist messages
    init_twist(twist_teleop_key);
    init_twist(twist_teleop_joy);
    init_twist(twist_nav);

    // Set a timer to publish the selected twist message periodically
    timer_ = create_wall_timer(std::chrono::milliseconds(33), std::bind(&CommandMasterNode::publishTwist, this));
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
    if(this->llv_secu){
      pub_command->publish(clamp_cmd(selected_twist));
    }
    else{
      pub_command->publish(selected_twist);
    }
  }

  geometry_msgs::msg::Twist clamp_cmd(geometry_msgs::msg::Twist twist){
    geometry_msgs::msg::Twist new_twist = twist;
    
    if (scan_data_ != nullptr){
      double absolute_speed = sqrt(pow(new_twist.linear.x,2)+pow(new_twist.linear.y,2))*0.5; //(0 to 1) * max linear speed (not needed if subscribed to odom)
      r_secu_2 = std::max(r_secu_2_min,r_secu_1+pow(absolute_speed,2));
      //cmd direction from 0 to 2pi
      double cmd_angle = atan2(new_twist.linear.y,new_twist.linear.x) + angle_offset;
      //area limit
      double cmd_angle_start = clamp_angle(cmd_angle-field_of_view/2);
      double cmd_angle_end = clamp_angle(cmd_angle+field_of_view/2);
      //all points in the rectangular area in front of the direction
      //std::vector<double> points = get_points(*scan_data_,cmd_angle_start,cmd_angle_end,r_secu_1,r_secu_2,field_of_view);
      //nearest obstacle
      double dmin = get_points(*scan_data_,cmd_angle_start,cmd_angle_end,r_secu_1,r_secu_2,field_of_view); //*min_element(points.begin(),points.end());
      //percentage of speed autorized
      double clamp = std::max(0.0,std::min(1.0,(dmin-r_secu_1)/(r_secu_2-r_secu_1)));
      //clamp speeds
      new_twist.linear.y = std::min(clamp*new_twist.linear.y,1.0);
      new_twist.linear.x = std::min(clamp*new_twist.linear.x,1.0);
      
      //RCLCPP_INFO(this->get_logger(), "------------");
      //RCLCPP_INFO(this->get_logger(), "r2: %f",r_secu_2);
      //RCLCPP_INFO(this->get_logger(), "cmd_angle: %f",cmd_angle*180/M_PI);
      //RCLCPP_INFO(this->get_logger(), "cmd_angle_start: %f",cmd_angle_start*180/M_PI);
      //RCLCPP_INFO(this->get_logger(), "cmd_angle_end: %f",cmd_angle_end*180/M_PI);
      //RCLCPP_INFO(this->get_logger(), "clamp1: %f",(dmin-r_secu_1)/(r_secu_2-r_secu_1));
      //RCLCPP_INFO(this->get_logger(), "clamp2: %f",clamp);
      //RCLCPP_INFO(this->get_logger(), "dmin: %f",dmin);
      //RCLCPP_INFO(this->get_logger(), "////////////");
      //rotation speed clamp
      if (proximity(*scan_data_,r_secu_1-0.06)){
        new_twist.angular.z = 0.0;
      }
    }

    return new_twist;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    scan_data_ = msg;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_teleop_key; 
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_teleop_joy;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_command;

  geometry_msgs::msg::Twist twist_teleop_key;
  geometry_msgs::msg::Twist twist_teleop_joy;
  geometry_msgs::msg::Twist twist_nav;
  sensor_msgs::msg::LaserScan::SharedPtr scan_data_ = nullptr;

  //scan security parameters
  double r_secu_1 = 0.45; //define rectangle area width and distance limit to put speed to 0
  double r_secu_2_min = 0.5; //define rectangle area lenght and distance limit to start decrease the speed
  double r_secu_2 = 0.5;
  double angle_offset = 0.0; //offset of angle between commands vector and lidars datas
  double field_of_view = M_PI/2; //should be in ]0,pi]
  bool llv_secu = false; //activate or not the low level collision avoidance

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
