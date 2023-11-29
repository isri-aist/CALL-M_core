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

float index_to_angle(int ind, int resolution){
    return (ind*(2*M_PI))/resolution;
}

void get_pos(double &x, double &y,float alpha,float val,float x_off,float y_off){
    x = val*cos(alpha)+x_off;
    y = val*sin(alpha)+y_off;
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

sensor_msgs::msg::LaserScan transform_data(sensor_msgs::msg::LaserScan::SharedPtr msg, double off_vect_x,double off_vect_y){
    /*
    msg: current message to transform
    transformed_scan: message in which datas will be stored
    off_vect_x: vector new_origin=>lidar
    off_vect_y: vector new_origin=>lidar
    */

    //RCLCPP_INFO(this->get_logger(), "transform 1");
    sensor_msgs::msg::LaserScan transformed_scan;
    transformed_scan.ranges.clear();
    int resolution = msg->ranges.size();

    // Transform the LaserScan message
    for (int i = 0; i < resolution; ++i) { //initiate
        transformed_scan.ranges.push_back(INFINITY);
    }
    //RCLCPP_INFO(this->get_logger(), "transform 2");
    //loop variables
    double init_x = 0;
    double init_y = 0;
    double init_val = 0;
    double init_angle = 0;
    double new_x = 0;
    double new_y = 0;
    float new_val = 0;
    double new_angle = 0;
    double new_index = 0;
    for (int ind = 0; ind < resolution; ++ind) { //transform
        init_angle = index_to_angle(ind,resolution);
        init_val = msg->ranges[ind];
        if (!isinf(init_val)){ //if it is infinity, the output will be infinity whatever the new frame
            get_pos(init_x, init_y,init_angle,init_val,0.0,0.0); //position in inital frame, vector: lidar=>point
            new_x = off_vect_x + init_x;
            new_y = off_vect_y + init_y;
            new_val = sqrt(pow(new_x,2)+pow(new_y,2));
            new_angle = atan2(new_y,new_x);
            if (new_angle<0){
                new_angle=2*M_PI+new_angle;
            }
            if (new_angle == 2*M_PI){
                new_angle=0;
            }
            new_index = angle_to_index(new_angle,resolution);
            transformed_scan.ranges[new_index] = std::min(transformed_scan.ranges[new_index],new_val); //in case some area are no more accessible from the new origin
        }
    }
    //RCLCPP_INFO(this->get_logger(), "transform 3");
    return transformed_scan;
}

class LaserScanFusionNode : public rclcpp::Node {
public:
    LaserScanFusionNode()
    : Node("laser_scan_merger_node") {

        initialize_params();
        refresh_params();
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic_lid1, default_qos, std::bind(&LaserScanFusionNode::scan1Callback, this, std::placeholders::_1));
        subscription2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic_lid2, default_qos, std::bind(&LaserScanFusionNode::scan2Callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_out, rclcpp::SystemDefaultsQoS());

        RCLCPP_INFO(this->get_logger(), "Merger Started: Lidar Scan messages need to be same length and same angles correspondance on 360 elongation.");
        RCLCPP_INFO(this->get_logger(), "Waiting for topics '%s' and '%s' ...",topic_lid1.c_str(),topic_lid2.c_str());
    }

private:
    void scan1Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        //RCLCPP_INFO(this->get_logger(), "Callback1");
        //get frame
        if (frame_lid1 != msg->header.frame_id){
            frame_lid1 = msg->header.frame_id;
        }
        refresh_params();
        if (frames_got){
            //RCLCPP_INFO(this->get_logger(), "Lid1_run");
            //transform data and configure new message
            sensor_msgs::msg::LaserScan tempo = transform_data(msg,vector_newframe_lid1.x,vector_newframe_lid1.y);
            scan1_data_ = std::make_shared<sensor_msgs::msg::LaserScan>(tempo);
            //RCLCPP_INFO(this->get_logger(), "Lid1_transformed");
            scan1_data_->header=msg->header;
            scan1_data_->header.frame_id=new_frame;
            scan1_data_->angle_min=msg->angle_min; 
            scan1_data_->angle_max=msg->angle_max; 
            scan1_data_->angle_increment=msg->angle_increment; 
            scan1_data_->time_increment=msg->time_increment; 
            scan1_data_->scan_time=msg->scan_time; 
            scan1_data_->range_min=msg->range_min; 
            scan1_data_->range_max=msg->range_max; 
            scan1_data_->intensities=msg->intensities; 
            //RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", msg->ranges[0],msg->ranges[100]);
            //RCLCPP_INFO(this->get_logger(), "Lid1_configured");
            fuseAndPublish();
            //RCLCPP_INFO(this->get_logger(), "Lid1_end");
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Trying to get frames");
            try {
                //get translation vector new_frame=>lidar1
                geometry_msgs::msg::TransformStamped transform =tf_buffer_->lookupTransform(new_frame, frame_lid1, tf2::TimePointZero);
                vector_newframe_lid1 = transform.transform.translation;
                //get translation vector new_frame=>lidar2
                transform =tf_buffer_->lookupTransform(new_frame, frame_lid2, tf2::TimePointZero);
                vector_newframe_lid2 = transform.transform.translation;
                frames_got=true;
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "%s (Ignore this error if node is launched correctly after)", e.what());
                frames_got=false;
                active_notif=true;
            }
        }

    }

    void scan2Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        //RCLCPP_INFO(this->get_logger(), "Callback2");
        //get frame
        if (frame_lid2 != msg->header.frame_id){
            frame_lid2 = msg->header.frame_id;
        }
        refresh_params();
        if (frames_got){
            //RCLCPP_INFO(this->get_logger(), "Lid2_run");
            //transform data and configure new message
            sensor_msgs::msg::LaserScan tempo = transform_data(msg,vector_newframe_lid2.x,vector_newframe_lid2.y);
            scan2_data_ = std::make_shared<sensor_msgs::msg::LaserScan>(tempo);
            //RCLCPP_INFO(this->get_logger(), "Lid2_transformed");
            scan2_data_->header=msg->header;
            scan2_data_->header.frame_id=new_frame;
            scan2_data_->angle_min=msg->angle_min; 
            scan2_data_->angle_max=msg->angle_max; 
            scan2_data_->angle_increment=msg->angle_increment; 
            scan2_data_->time_increment=msg->time_increment; 
            scan2_data_->scan_time=msg->scan_time; 
            scan2_data_->range_min=msg->range_min; 
            scan2_data_->range_max=msg->range_max; 
            scan2_data_->intensities=msg->intensities; 
            //RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", msg->ranges[0],msg->ranges[100]);
            //RCLCPP_INFO(this->get_logger(), "Lid2_configured");
            fuseAndPublish();
            //RCLCPP_INFO(this->get_logger(), "Lid2_end");
        }
    }

    void fuseAndPublish() {
        if (scan1_data_ != nullptr && scan2_data_ != nullptr) {
            // Implemented fusion logic here 
            //RCLCPP_INFO(this->get_logger(), "fuse");
            auto fused_data = fuseScans(scan1_data_, scan2_data_);
            //RCLCPP_INFO(this->get_logger(), "fused");
            //RCLCPP_INFO(this->get_logger(), "I heard merged: '%f' '%f'", fused_data.ranges[0],fused_data.ranges[100]);
            
            // Publish the fused laser scan
            publisher_->publish(fused_data);
            if(active_notif){
                RCLCPP_INFO(this->get_logger(), "Merging '%s' and '%s'",topic_lid1.c_str(),topic_lid2.c_str());
                RCLCPP_INFO(this->get_logger(), "Target frame: '%s'",new_frame.c_str());
                RCLCPP_INFO(this->get_logger(), "Merged Scan published on topic: '%s'",topic_out.c_str());
                active_notif=false;
            }
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Lidar merger need the topics: '%s' '%s' and the frames: '%s' '%s' '%s' to work.", topic_lid1.c_str(),topic_lid2.c_str(),frame_lid1.c_str(), frame_lid2.c_str(), new_frame.c_str());
            active_notif=true;
        }
    }

    sensor_msgs::msg::LaserScan fuseScans(const sensor_msgs::msg::LaserScan::SharedPtr scan1,const sensor_msgs::msg::LaserScan::SharedPtr scan2) {
        int resolution = (scan1->ranges.size())*1; //assuming length are the same, we are using double resolution for more accurate result
        sensor_msgs::msg::LaserScan fused_data = *scan1; // Assuming headers are the same, the scan1 have already been transform in new_frame
        
        //init ranges and add inf values according to the resolution
        fused_data.ranges = {};
        for (int i = 0; i < resolution; ++i){
            fused_data.ranges.push_back(INFINITY);
        }
        fused_data.angle_increment = fused_data.angle_increment*(scan1->ranges.size()/resolution); //adjust new increment depending on output resolution chosen
        
        //compute index to consider for each lidars according to min/max angles wanted
        int lidar1_start_index = angle_to_index(lidar1_start_angle, resolution);
        int lidar1_end_index = angle_to_index(lidar1_end_angle, resolution);
        int lidar2_start_index = angle_to_index(lidar2_start_angle, resolution);
        int lidar2_end_index = angle_to_index(lidar2_end_angle, resolution);
        int ind = 0;

        //fuse datas
        for (size_t i = 0; i < fused_data.ranges.size(); ++i) { 
            ind = static_cast<int>(i); //just to avoid comparison warning between size_t and int values
            if (consider_val(ind, lidar1_start_index, lidar1_end_index) && consider_val(ind, lidar2_start_index, lidar2_end_index)){ //if we have to consider both lidars and fuse values
                fused_data.ranges[i] = std::min(scan1->ranges[i],scan2->ranges[i]); //we take the closed signal
            }
            else if(consider_val(ind, lidar1_start_index, lidar1_end_index)){ //if only lidar1 values
                fused_data.ranges[i] = scan1->ranges[i];
            }
            else if(consider_val(ind, lidar2_start_index, lidar2_end_index)){ //if only lidar2 values
                fused_data.ranges[i] = scan2->ranges[i];
            }
            else{ //if none lidar should be considered in this area
                fused_data.ranges[i] = INFINITY;
            }
        }

        return fused_data;
    }

    void initialize_params(){
        
        this->declare_parameter("lidar1_start_angle"); 
        this->declare_parameter("lidar1_end_angle"); 
        this->declare_parameter("idar2_start_angle"); 
        this->declare_parameter("lidar2_end_angle"); 
        this->declare_parameter("topic_lid1");
        this->declare_parameter("topic_lid2"); 
        this->declare_parameter("topic_out"); 
        this->declare_parameter("new_frame");

    }

    void refresh_params(){
        this->get_parameter_or<float>("lidar1_start_angle", lidar1_start_angle, 0.0);
        this->get_parameter_or<float>("lidar1_end_angle", lidar1_end_angle, 2*M_PI);
        this->get_parameter_or<float>("lidar2_start_angle", lidar2_start_angle, 0.0);
        this->get_parameter_or<float>("lidar2_end_angle", lidar2_end_angle, 2*M_PI);
        this->get_parameter_or<std::string>("topic_lid1",topic_lid1, "lidar1_scan");
        this->get_parameter_or<std::string>("topic_lid2",topic_lid2,"lidar2_scan");
        this->get_parameter_or<std::string>("topic_out",topic_out,"scan");
        this->get_parameter_or<std::string>("new_frame",new_frame,"base2_link");
    }

    sensor_msgs::msg::LaserScan::SharedPtr scan1_data_;
    sensor_msgs::msg::LaserScan::SharedPtr scan2_data_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription2_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    //Lidar configurations
    float lidar1_start_angle; //start angle to consider for lidar1 (0 to 2pi circle)
    float lidar1_end_angle; //endangle to consider for lidar1 (0 to 2pi circle)
    float lidar2_start_angle;
    float lidar2_end_angle;
    std::string topic_lid1;
    std::string topic_lid2;
    std::string topic_out;
    std::string frame_lid1;
    std::string frame_lid2;
    std::string new_frame;
    geometry_msgs::msg::Vector3 vector_newframe_lid1;
    geometry_msgs::msg::Vector3 vector_newframe_lid2;
    bool frames_got = false;
    bool active_notif = true;

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
