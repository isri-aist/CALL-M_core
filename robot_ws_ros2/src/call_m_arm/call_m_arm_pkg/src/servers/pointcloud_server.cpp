#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <filesystem>
#include "call_m_custom_msgs/srv/save_point_cloud.hpp"

namespace fs = std::filesystem;

class PCLSaveNode : public rclcpp::Node {
public:
    PCLSaveNode() : Node("pcl_save_node"), save_request_(false) {
        // Create a service to save the point cloud
        service_ = this->create_service<call_m_custom_msgs::srv::SavePointCloud>(
            "/save_point_cloud",
            std::bind(&PCLSaveNode::savePointCloudServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Initialize tf2 listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Point cloud save node ready.");
    }

private:
    void savePointCloudServiceCallback(const std::shared_ptr<call_m_custom_msgs::srv::SavePointCloud::Request> request,
                                       std::shared_ptr<call_m_custom_msgs::srv::SavePointCloud::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Received request to save point cloud from topic: %s to path: %s",
                    request->topic.c_str(), request->path.c_str());

        if (!fs::exists(fs::path(request->path).parent_path())) {
            RCLCPP_ERROR(this->get_logger(), "Invalid save path: %s", request->path.c_str());
            response->success = false;
            //response->message = "Invalid save path.";
            return;
        }

        save_path_ = request->path;
        save_request_ = true;

        // Subscribe to the given topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            request->topic, 10,
            std::bind(&PCLSaveNode::pointCloudCallback, this, std::placeholders::_1)
        );

        response->success = true;
        //response->message = "Save request accepted. Waiting for point cloud...";
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!save_request_) {
            return;  // Only process if a save request is active
        }

        // Transform point cloud to the base frame
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_->lookupTransform(
                "base",                // Target frame
                msg->header.frame_id,  // Source frame
                rclcpp::Time(0),       // Get the latest transform
                rclcpp::Duration::from_seconds(1.0)
            );
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform error from %s to base: %s", msg->header.frame_id.c_str(), ex.what());
            return;
        }

        // Convert ROS point cloud to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Apply transformation
        for (auto &point : cloud->points) {
            geometry_msgs::msg::PointStamped point_in, point_out;
            point_in.header.frame_id = msg->header.frame_id;
            point_in.point.x = point.x;
            point_in.point.y = point.y;
            point_in.point.z = point.z;

            tf2::doTransform(point_in, point_out, transform_stamped);

            point.x = point_out.point.x;
            point.y = point_out.point.y;
            point.z = point_out.point.z;
        }

        // Remove NaN points
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered, indices);

        // Filter out the floor plane
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.03);

        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (!inliers->indices.empty()) {
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(true);

            pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*final_cloud);

            pcl::removeNaNFromPointCloud(*final_cloud, *final_cloud, indices);

            if (pcl::io::savePCDFileASCII(save_path_, *final_cloud) == 0) {
                RCLCPP_INFO(this->get_logger(), "Filtered point cloud saved to %s", save_path_.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to save filtered point cloud.");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "No plane detected. Saving original filtered cloud.");
            pcl::io::savePCDFileASCII(save_path_, *cloud_filtered);
        }

        // Clean up
        save_request_ = false;
        subscription_.reset();
    }

    // Member variables
    bool save_request_;
    std::string save_path_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Service<call_m_custom_msgs::srv::SavePointCloud>::SharedPtr service_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCLSaveNode>());
    rclcpp::shutdown();
    return 0;
}
