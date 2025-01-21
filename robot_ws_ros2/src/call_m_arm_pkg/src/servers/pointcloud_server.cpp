#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <algorithm>
#include "std_srvs/srv/empty.hpp" // Header for the Empty service
#include "std_srvs/srv/trigger.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <filesystem>
#include "call_m_custom_msgs/srv/save_point_cloud.hpp"
#include "call_m_custom_msgs/srv/concatenate_point_cloud.hpp"
#include "call_m_custom_msgs/srv/filter_point_cloud.hpp"
#include "chrono"
#include "rclcpp/executor.hpp"
#include <vector>
#include <memory>
#include <limits>

namespace fs = std::filesystem;
std::chrono::milliseconds sleep_duration(1000); // 1000ms = 1 second
class PCLSaveNode : public rclcpp::Node
{
public:
    PCLSaveNode() : Node("pcl_save_node"), save_request_(false)
    {
        // Create a service to save the point cloud
        save_point_cloid_service_ = this->create_service<call_m_custom_msgs::srv::SavePointCloud>(
            "/save_point_cloud",
            std::bind(&PCLSaveNode::savePointCloudServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

        filterPointCloud_ = this->create_service<call_m_custom_msgs::srv::FilterPointCloud>(
            "/filter_point_cloud",
            std::bind(&PCLSaveNode::filterPointCloudCallback, this, std::placeholders::_1, std::placeholders::_2));

        concatenatePointCloud_ = this->create_service<call_m_custom_msgs::srv::ConcatenatePointCloud>(
            "/add_point_cloud",
            std::bind(&PCLSaveNode::addAndPublishPointCloud, this, std::placeholders::_1, std::placeholders::_2));

        reset_point_clouds_ = this->create_service<std_srvs::srv::Empty>(
            "/reset_point_clouds",
            std::bind(&PCLSaveNode::resetPointClouds, this, std::placeholders::_1, std::placeholders::_2));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        interest_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/saved_pointcloud", 10);

        detected_objects_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/detected_objects", 10);

        selected_cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/selected_cluster", 10);

        clicked_cluster_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&PCLSaveNode::identifyAndSaveCluster, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Point cloud save node ready.");
    }

private:

    void resetPointClouds(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        const std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        // Create an empty point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Update internal cloud storage
        selected_cluster_ = empty_cloud;
        accumulated_cloud_ = empty_cloud;
        detected_clusters_ = empty_cloud;

        // Convert to ROS message
        sensor_msgs::msg::PointCloud2 empty_cloud_msg;
        pcl::toROSMsg(*empty_cloud, empty_cloud_msg);

        // Set the header values
        empty_cloud_msg.header.frame_id = "base"; // Replace with the correct frame if necessary
        empty_cloud_msg.header.stamp = this->get_clock()->now();

        // Publish the empty cloud
        interest_pointcloud_publisher_->publish(empty_cloud_msg);
        detected_objects_publisher_->publish(empty_cloud_msg);
        selected_cluster_publisher_->publish(empty_cloud_msg);

        RCLCPP_INFO(this->get_logger(), "Reset and published empty point clouds.");
    }

    void identifyAndSaveCluster(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received clicked point at (%f, %f, %f)",
                    msg->point.x, msg->point.y, msg->point.z);

        if (detected_clusters_->points.empty() || cluster_indices_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No clusters available to identify.");
            return;
        }

        // Convert clicked point to Eigen
        Eigen::Vector3f clicked_point(msg->point.x, msg->point.y, msg->point.z);

        float min_distance = std::numeric_limits<float>::max();
        size_t nearest_cluster_idx = 0;

        // Variables for brute force comparison
        int nearest_point_idx = -1;

        for (size_t i = 0; i < cluster_indices_.size(); ++i)
        {
            for (const auto &point_idx : cluster_indices_[i].indices)
            {
                const auto &point = detected_clusters_->points[point_idx];
                Eigen::Vector3f cluster_point(point.x, point.y, point.z);

                // Calculate Euclidean distance
                float distance = (clicked_point - cluster_point).norm();

                if (distance < min_distance)
                {
                    min_distance = distance;
                    nearest_cluster_idx = i;
                    nearest_point_idx = point_idx;
                }
            }
        }

        RCLCPP_INFO(this->get_logger(),
                    "Nearest point is at index %d, part of cluster %lu, at a distance of %f meters.",
                    nearest_point_idx, nearest_cluster_idx, min_distance);

        // Extract the nearest cluster
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr nearest_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto &idx : cluster_indices_[nearest_cluster_idx].indices)
        {
            nearest_cluster->points.push_back(detected_clusters_->points[idx]);
        }

        // Update selected cluster
        *selected_cluster_ = *nearest_cluster;

        // Publish the nearest cluster
        sensor_msgs::msg::PointCloud2 cluster_msg;
        pcl::toROSMsg(*nearest_cluster, cluster_msg);
        cluster_msg.header.frame_id = "base";
        cluster_msg.header.stamp = this->get_clock()->now();
        selected_cluster_publisher_->publish(cluster_msg);

        RCLCPP_INFO(this->get_logger(), "Published nearest cluster.");

        // Save the nearest cluster to a PCD file
        selected_cluster_->width = selected_cluster_->points.size();
        selected_cluster_->height = 1;

        std::string filename = "/workspace/ros2-workspace/call_m_arm_ws/src/call_m_arm/gpd/tutorials/output.pcd"; // Modify the file path if needed
        if (pcl::io::savePCDFileASCII(filename, *selected_cluster_) == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Saved nearest cluster to %s", filename.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save nearest cluster.");
        }
    }

    void filterPointCloudCallback(
        const std::shared_ptr<call_m_custom_msgs::srv::FilterPointCloud::Request> request,
        std::shared_ptr<call_m_custom_msgs::srv::FilterPointCloud::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to filter point cloud");

        // Convert the accumulated pcl::PointCloud to a sensor_msgs::PointCloud2 message
        sensor_msgs::msg::PointCloud2 raw_pointcloud_msg;
        pcl::toROSMsg(*accumulated_cloud_, raw_pointcloud_msg);

        if (raw_pointcloud_msg.data.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received an empty point cloud. Skipping processing.");
            response->pointcloud = raw_pointcloud_msg;
            return;
        }

        // Convert the received PointCloud2 message to PCL format
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(raw_pointcloud_msg, *input_cloud);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ransac_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        //applyRANSACFiltering(input_cloud,ransac_cloud);
        *ransac_cloud = *input_cloud;
        // Apply height filter with custom ranges
        std::vector<std::pair<double, double>> ignored_height_ranges = {
            {-0.8, -0.15},
            {0.20, 0.24},
            {0.6, 0.64}};

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr height_filtered_cloud = applyHeightFilter(input_cloud, ignored_height_ranges);

        if (height_filtered_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "No points remain after height filtering. Returning the RANSAC-filtered cloud.");
            pcl::toROSMsg(*ransac_cloud, raw_pointcloud_msg);
            response->pointcloud = raw_pointcloud_msg;
            return;
        }

        // Perform clustering on the height-filtered cloud
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(height_filtered_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(0.02); // 2cm
        ec.setMinClusterSize(80);
        ec.setMaxClusterSize(2000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(height_filtered_cloud);
        ec.extract(cluster_indices);

        if (cluster_indices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No clusters detected in the filtered cloud.");
            pcl::toROSMsg(*height_filtered_cloud, raw_pointcloud_msg);
            response->pointcloud = raw_pointcloud_msg;
            return;
        }

        // Update class variables for cluster indices and detected clusters
        cluster_indices_ = cluster_indices;
        *detected_clusters_ = *height_filtered_cloud;

        RCLCPP_INFO(this->get_logger(), "Detected %lu clusters.", cluster_indices_.size());

        // Process and publish clusters
        for (size_t i = 0; i < cluster_indices_.size(); ++i)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
            for (const auto &idx : cluster_indices_[i].indices)
            {
                cluster->points.push_back(detected_clusters_->points[idx]);
            }

            RCLCPP_INFO(this->get_logger(), "Cluster %lu has %lu points.", i, cluster->points.size());

            // Optionally convert each cluster to a ROS message and publish it
            sensor_msgs::msg::PointCloud2 cluster_msg;
            pcl::toROSMsg(*cluster, cluster_msg);
            cluster_msg.header = raw_pointcloud_msg.header;
            cluster_msg.header.frame_id = "base";
            cluster_msg.header.stamp = this->get_clock()->now();
            detected_objects_publisher_->publish(cluster_msg);
            rclcpp::sleep_for(sleep_duration); // Add a delay (e.g., 1 second)
        }

        // Send final filtered point cloud as response
        pcl::toROSMsg(*height_filtered_cloud, raw_pointcloud_msg);
        response->pointcloud = raw_pointcloud_msg;
    }

    void addAndPublishPointCloud(const std::shared_ptr<call_m_custom_msgs::srv::ConcatenatePointCloud::Request> request,
                                 std::shared_ptr<call_m_custom_msgs::srv::ConcatenatePointCloud::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Activating service to add point cloud from topic: %s", request->topic.c_str());

        // Reuse the subscription logic
        subscribeToPointCloudTopic(
            request->topic,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr new_cloud_msg)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::fromROSMsg(*new_cloud_msg, *new_cloud);

                if (new_cloud->empty())
                {
                    RCLCPP_WARN(this->get_logger(), "Received an empty point cloud. Skipping processing.");
                    return;
                }

                // Transform the point cloud to the base frame
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                try
                {
                    geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                        "base", new_cloud_msg->header.frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));

                    Eigen::Matrix4f transform_matrix = tf2::transformToEigen(transform_stamped).matrix().cast<float>();
                    pcl::transformPointCloud(*new_cloud, *transformed_cloud, transform_matrix);
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
                    return;
                }

                // Apply RANSAC filtering to remove planar elements (e.g., floor)
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr ransac_filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                *ransac_filtered_cloud = *transformed_cloud;
                //applyRANSACFiltering(transformed_cloud, ransac_filtered_cloud);

                // Accumulate the filtered point cloud
                {
                    std::lock_guard<std::mutex> lock(accumulated_cloud_mutex_);
                    *accumulated_cloud_ += *ransac_filtered_cloud;

                    // Apply voxel grid filtering to optimize accumulated cloud
                    applyVoxelGridFilter(accumulated_cloud_);
                }

                // Convert accumulated point cloud back to ROS message
                sensor_msgs::msg::PointCloud2 accumulated_msg;
                pcl::toROSMsg(*accumulated_cloud_, accumulated_msg);
                accumulated_msg.header.frame_id = "base";
                accumulated_msg.header.stamp = this->get_clock()->now();

                // Publish the accumulated point cloud
                interest_pointcloud_publisher_->publish(accumulated_msg);
                RCLCPP_INFO(this->get_logger(), "Published accumulated point cloud with %lu points",
                            accumulated_cloud_->points.size());
            });
    }

    // Function to apply RANSAC filtering
    void applyRANSACFiltering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.03); // Adjust this threshold based on your environment

        seg.setInputCloud(input_cloud);
        seg.segment(*inliers, *coefficients);

        // If no plane is detected, copy the input cloud to the output
        if (inliers->indices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No planar surface detected in the point cloud.");
            *output_cloud = *input_cloud;
            return;
        }

        // Extract non-planar points
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(input_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true); // Keep points that are not part of the plane
        extract.filter(*output_cloud);
    }

    void subscribeToPointCloudTopic(
        const std::string &topic,
        std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)> callback)
    {
        if (topic.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot subscribe to an empty topic.");
            return;
        }

        if (is_subscribed_)
        {
            RCLCPP_WARN(this->get_logger(), "Already subscribed to a topic. Skipping new subscription.");
            return;
        }

        // Set the flag to indicate the subscription is active
        is_subscribed_ = true;

        // Create a temporary subscription
        temp_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, 10,
            [this, callback](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
            {
                if (!is_subscribed_)
                {
                    return; // Ignore if no active subscription
                }

                callback(msg);

                // Clean up the subscription after the callback
                is_subscribed_ = false;
                temp_subscription_.reset();
                RCLCPP_INFO(this->get_logger(), "Unsubscribed from topic: %s", msg->header.frame_id.c_str());
            });

        RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", topic.c_str());
    }

    void savePointCloudServiceCallback(const std::shared_ptr<call_m_custom_msgs::srv::SavePointCloud::Request> request,
                                       std::shared_ptr<call_m_custom_msgs::srv::SavePointCloud::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to save point cloud from topic: %s to path: %s",
                    request->topic.c_str(), request->path.c_str());

        if (!fs::exists(fs::path(request->path).parent_path()))
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid save path: %s", request->path.c_str());
            response->success = false;
            // response->message = "Invalid save path.";
            return;
        }

        save_path_ = request->path;
        save_request_ = true;

        // Subscribe to the given topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            request->topic, 10,
            std::bind(&PCLSaveNode::pointCloudCallback, this, std::placeholders::_1));

        response->success = true;
        // response->message = "Save request accepted. Waiting for point cloud...";
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!save_request_)
        {
            return; // Only process if a save request is active
        }

        // Transform point cloud to the base frame
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform(
                "base",               // Target frame
                msg->header.frame_id, // Source frame
                rclcpp::Time(0),      // Get the latest transform
                rclcpp::Duration::from_seconds(1.0));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform error from %s to base: %s", msg->header.frame_id.c_str(), ex.what());
            return;
        }

        // Convert ROS point cloud to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Apply transformation
        for (auto &point : cloud->points)
        {
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

        if (!inliers->indices.empty())
        {
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(true);

            pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*final_cloud);

            pcl::removeNaNFromPointCloud(*final_cloud, *final_cloud, indices);

            if (pcl::io::savePCDFileASCII(save_path_, *final_cloud) == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Filtered point cloud saved to %s", save_path_.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to save filtered point cloud.");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No plane detected. Saving original filtered cloud.");
            pcl::io::savePCDFileASCII(save_path_, *cloud_filtered);
        }

        // Clean up
        save_request_ = false;
        subscription_.reset();
    }

    // Assuming your point cloud is of type pcl::PointCloud<pcl::PointXYZ>
    void applyVoxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setInputCloud(cloud);

        // Set the leaf size (voxel resolution)
        // Smaller values retain more details but reduce less data; adjust as needed
        float leaf_size = 0.01f; // Example: 1 cm voxels
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

        // Create a new cloud to hold the filtered data
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        voxel_filter.filter(*filtered_cloud);

        // Replace the input cloud with the filtered cloud
        *cloud = *filtered_cloud;
    }


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyHeightFilter(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
        const std::vector<std::pair<double, double>> &ignored_height_ranges)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (const auto &point : input_cloud->points)
        {
            bool ignore_point = false;
            for (const auto &range : ignored_height_ranges)
            {
                if (point.z >= range.first && point.z <= range.second)
                {
                    ignore_point = true;
                    break;
                }
            }

            if (!ignore_point)
            {
                filtered_cloud->points.push_back(point);
            }
        }

        return filtered_cloud;
    }
    
    // Member variables
    bool save_request_;
    bool is_subscribed_;
    // std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> detected_clusters_;
    std::vector<pcl::PointIndices> cluster_indices_;
    std::string save_path_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<octomap::ColorOcTree> accumulated_octree_;
    std::mutex accumulated_cloud_mutex_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_cluster_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr temp_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detected_objects_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr interest_pointcloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr selected_cluster_publisher_;
    rclcpp::Service<call_m_custom_msgs::srv::SavePointCloud>::SharedPtr save_point_cloid_service_;
    rclcpp::Service<call_m_custom_msgs::srv::ConcatenatePointCloud>::SharedPtr concatenatePointCloud_;
    rclcpp::Service<call_m_custom_msgs::srv::FilterPointCloud>::SharedPtr filterPointCloud_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_cluster_pcd_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_point_clouds_;
    // This point instance of point cloud should exist outside the function so it does not reset when the function ends
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected_cluster_{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr detected_clusters_{new pcl::PointCloud<pcl::PointXYZRGB>};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCLSaveNode>());
    rclcpp::shutdown();
    return 0;
}
