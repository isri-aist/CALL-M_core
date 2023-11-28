#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include <cmath>
#include <string>

int angle_to_index(float alpha, int resolution){
    //return index of angle alpha, in a table with 'resolution' values placed from 0 to 360 angles.
    return (alpha*resolution)/(2*M_PI);
}

int index_to_angle(int ind, int resolution){
    return (ind*(2*M_PI))/resolution;
}

bool consider_val(int current_ind, int start_ind, int end_ind){
    // return true if current_ind is between start_ind and end_ind according to a circle reference.
    if(start_ind>end_ind){ //if interval pass throught the origin of the circle, we test considering the split into 2 intervals
        if(current_ind>=start_ind || current_ind<=start_ind){
            return true;
        }
        else{
            return false;
        }
    }
    else{ // if values are equal, or classical ,we test as classic interval
        if(current_ind>=start_ind && current_ind<=end_ind){
            return true;
        }
        else{
            return false;
        }
    }
}

class LaserScanFusionNode : public rclcpp::Node {
public:
    LaserScanFusionNode()
    : Node("laser_scan_fusion_node") {
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic_lid1, default_qos, std::bind(&LaserScanFusionNode::scan1Callback, this, std::placeholders::_1));
        subscription2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic_lid2, default_qos, std::bind(&LaserScanFusionNode::scan2Callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_out, rclcpp::SystemDefaultsQoS());

        RCLCPP_INFO(this->get_logger(), "Merger Started: Lidar Scan messages need to be same length and same angles correspondance on 360 elongation.");
    }

private:
    void scan1Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan1_data_ = msg;
        //RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", msg->ranges[0],msg->ranges[100]);
        fuseAndPublish();
    }

    void scan2Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        //convert lidar2 data in lidar1 frame
        // Lookup the transform between the source and target frames

        geometry_msgs::msg::TransformStamped transform =tf_buffer_->lookupTransform(frame_lid1, frame_lid2, tf2::TimePointZero);

        //get translation vector
        geometry_msgs::msg::Vector3 translation_vector = transform.transform.translation;
        // Now you can access the x, y, and z components of the translation vector
        double x_offset = translation_vector.x;
        double y_offset = translation_vector.y;

        // Transform the LaserScan message
        sensor_msgs::msg::LaserScan transformed_scan;
        for (size_t i = 0; i < msg->ranges.size(); ++i) { //copy
            transformed_scan.ranges.push_back(msg->ranges[i]);
        }
        
        //loop variables
        int resolution = msg->ranges.size();
        double init_x = 0;
        double init_y = 0;
        double init_val = 0;
        double init_angle = 0;
        double new_x = 0;
        double new_y = 0;
        double new_val = 0;
        double new_angle = 0;
        double new_index = 0;
        for (size_t i = 0; i < msg->ranges.size(); ++i) { //transform
            int ind = static_cast<int>(i);
            init_angle = index_to_angle(ind,resolution);
            init_val = msg->ranges[ind];
            init_x = init_val*cos(init_angle);
            init_y = init_val*sin(init_angle);
            new_x = init_x - x_offset;
            new_y = init_y - y_offset;
            new_val = sqrt(pow(new_x,2)+pow(new_y,2));
            new_angle = atan2(new_y,new_x)+M_PI;
            new_index = angle_to_index(new_angle,resolution);
            transformed_scan.ranges[new_index] = new_val;
        }

        scan2_data_ = std::make_shared<sensor_msgs::msg::LaserScan>(transformed_scan);
        //RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", msg->ranges[0],msg->ranges[100]);
        fuseAndPublish();

    }

    void fuseAndPublish() {
        if (scan1_data_ != nullptr && scan2_data_ != nullptr) {

            // Implemented fusion logic here 
            auto fused_data = fuseScans(scan1_data_, scan2_data_);
            //RCLCPP_INFO(this->get_logger(), "I heard merged: '%f' '%f'", fused_data.ranges[0],fused_data.ranges[100]);
            // Publish the fused laser scan
            publisher_->publish(fused_data);
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Lidar merger need the topics: '%s' '%s' and the frames: '%s' '%s' to work.", topic_lid1.c_str(),topic_lid2.c_str(),frame_lid1.c_str(), frame_lid2.c_str());
        }
    }

    sensor_msgs::msg::LaserScan fuseScans(
        const sensor_msgs::msg::LaserScan::SharedPtr& scan1,
        const sensor_msgs::msg::LaserScan::SharedPtr& scan2) {

        sensor_msgs::msg::LaserScan fused_data = *scan1; // Assuming headers are the same
        int resolution = scan1->ranges.size(); //assuming length are the same
        //compute index to consider for each lidars according to min/max angles wanted
        int lidar1_start_index = angle_to_index(lidar1_start_angle, resolution);
        int lidar1_end_index = angle_to_index(lidar1_end_angle, resolution);
        int lidar2_start_index = angle_to_index(lidar2_start_angle, resolution);
        int lidar2_end_index = angle_to_index(lidar2_end_angle, resolution);
        int ind = 0;

        for (size_t i = 0; i < scan1->ranges.size(); ++i) {
            ind = static_cast<int>(i); //just to avoid comparison warning between size_t and int values
            fused_data.ranges[i] = scan2->ranges[i];
            /*if (consider_val(ind, lidar1_start_index, lidar1_end_index) && consider_val(ind, lidar2_start_index, lidar2_end_index)){ //if we have to consider both lidars and fuse values
                fused_data.ranges[i] = (scan1->ranges[i] + scan2->ranges[i])/2.0;
            }
            else if(consider_val(ind, lidar1_start_index, lidar1_end_index)){ //if only lidar1 values
                fused_data.ranges[i] = scan1->ranges[i];
            }
            else if(consider_val(ind, lidar2_start_index, lidar2_end_index)){ //if only lidar2 values
                fused_data.ranges[i] = scan2->ranges[i];
            }
            else{ //if none lidar should be considered in this area
                fused_data.ranges[i] = INFINITY;
            }*/
        }

        return fused_data;
    }

    sensor_msgs::msg::LaserScan::SharedPtr scan1_data_;
    sensor_msgs::msg::LaserScan::SharedPtr scan2_data_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription2_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    //Lidar configurations
    float lidar1_start_angle = (3/4)*M_PI; //start angle to consider for lidar1 (0 to 2pi circle)
    float lidar1_end_angle = M_PI; //endangle to consider for lidar1 (0 to 2pi circle)
    float lidar2_start_angle = 0.5*M_PI;
    float lidar2_end_angle = 2.0*M_PI;
    std::string topic_lid1 = "lidar1_scan";
    std::string topic_lid2 = "lidar2_scan";
    std::string topic_out = "scan";
    std::string frame_lid1 = "lidar1_link";
    std::string frame_lid2 = "lidar2_link";

    //for transform considerations
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScanFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
