/*
 * This sample code extract planes and objects from a pointcloud.
 * It it mainly based on ROS-industrial's "Building a Perception Pipeline" tutorial.
 * https://industrial-training-master.readthedocs.io/en/humble/_source/session5/Building-a-Perception-Pipeline.html#building-a-perception-pipeline
 *
 */

#include <string>
#include <memory>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>

// Using PCL functions
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

class PerceptionNode : public rclcpp::Node
{
  public:
    PerceptionNode()
    : Node(
          "perception_node", rclcpp::NodeOptions()
                                 .allow_undeclared_parameters(true)
                                 .automatically_declare_parameters_from_overrides(true))
    {
        /*
         * SET UP PUBLISHERS
         */
        RCLCPP_INFO(this->get_logger(), "Setting up publishers");

        voxel_grid_publisher_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_cluster", 1);

        passthrough_publisher_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("cropped_cluster", 1);

        segmentation_plane_publisher_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("segmented_plane", 1);

        segmentation_obj_publisher_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("segmented_obj", 1);

        /*
         * SET UP PARAMETERS
         */
        rclcpp::Parameter cloud_topic_param, world_frame_param, camera_frame_param,
            voxel_leaf_size_param, x_filter_min_param, x_filter_max_param, y_filter_min_param,
            y_filter_max_param, z_filter_min_param, z_filter_max_param, plane_max_iter_param,
            plane_dist_thresh_param;

        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        this->get_parameter_or(
            "cloud_topic", cloud_topic_param, rclcpp::Parameter("", "/depth/points"));
        this->get_parameter_or(
            "world_frame", world_frame_param, rclcpp::Parameter("", "kinect_link"));
        this->get_parameter_or(
            "camera_frame", camera_frame_param, rclcpp::Parameter("", "kinect_link"));
        this->get_parameter_or(
            "voxel_leaf_size", voxel_leaf_size_param, rclcpp::Parameter("", 0.002));
        this->get_parameter_or("x_filter_min", x_filter_min_param, rclcpp::Parameter("", -2.5));
        this->get_parameter_or("x_filter_max", x_filter_max_param, rclcpp::Parameter("", 2.5));
        this->get_parameter_or("y_filter_min", y_filter_min_param, rclcpp::Parameter("", -2.5));
        this->get_parameter_or("y_filter_max", y_filter_max_param, rclcpp::Parameter("", 2.5));
        this->get_parameter_or("z_filter_min", z_filter_min_param, rclcpp::Parameter("", -2.5));
        this->get_parameter_or("z_filter_max", z_filter_max_param, rclcpp::Parameter("", 2.5));
        this->get_parameter_or(
            "plane_max_iterations", plane_max_iter_param, rclcpp::Parameter("", 50));
        this->get_parameter_or(
            "plane_distance_threshold", plane_dist_thresh_param, rclcpp::Parameter("", 0.05));

        cloud_topic = cloud_topic_param.as_string();
        world_frame = world_frame_param.as_string();
        camera_frame = camera_frame_param.as_string();
        voxel_leaf_size = static_cast<float>(voxel_leaf_size_param.as_double());
        x_filter_min = x_filter_min_param.as_double();
        x_filter_max = x_filter_max_param.as_double();
        y_filter_min = y_filter_min_param.as_double();
        y_filter_max = y_filter_max_param.as_double();
        z_filter_min = z_filter_min_param.as_double();
        z_filter_max = z_filter_max_param.as_double();
        plane_max_iter = plane_max_iter_param.as_int();
        plane_dist_thresh = plane_dist_thresh_param.as_double();

        /*
         * SET UP SUBSCRIBER
         */
        RCLCPP_INFO(this->get_logger(), "Setting up subscriber");

        cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, 1,
            std::bind(&PerceptionNode::cloud_callback, this, std::placeholders::_1));

        /*
         * SET UP TF
         */
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

  private:
    /*
     * LISTEN FOR PointCloud2
     */
    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud)
    {
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "Cloud service called; getting a PointCloud2 on topic " + cloud_topic);

        /*
         * TRANSFORM PointCloud2 FROM CAMERA FRAME TO WORLD FRAME
         */
        geometry_msgs::msg::TransformStamped stransform;

        try {
            stransform = tf_buffer_->lookupTransform(
                world_frame, recent_cloud->header.frame_id, tf2::TimePointZero,
                tf2::durationFromSec(3));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        }

        sensor_msgs::msg::PointCloud2 transformed_cloud;
        pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud, transformed_cloud);

        /*
         * CONVERT PointCloud2 ROS->PCL
         */
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(transformed_cloud, cloud);

        update_parameters();

        /*
         * VoxelGrid
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud_ptr);
        voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_filter.filter(*cloud_voxel_filtered);

        /*
         * Crop box
         */
        pcl::PointCloud<pcl::PointXYZ> xyz_filtered_cloud;
        pcl::CropBox<pcl::PointXYZ> crop;
        crop.setInputCloud(cloud_voxel_filtered);
        Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
        Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
        crop.setMin(min_point);
        crop.setMax(max_point);
        crop.filter(xyz_filtered_cloud);

        this->publishPointCloud(voxel_grid_publisher_, *cloud_voxel_filtered);
        this->publishPointCloud(passthrough_publisher_, xyz_filtered_cloud);

        /*
         * Plane segmentation
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(
            new pcl::PointCloud<pcl::PointXYZ>(xyz_filtered_cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(plane_max_iter);
        seg.setDistanceThreshold(plane_dist_thresh);

        seg.setInputCloud(cropped_cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            RCLCPP_WARN(
                this->get_logger(), "Could not estimate a planar model for the given dataset.");
            return;
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cropped_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);

        extract.setNegative(true);
        extract.filter(*cloud_f);

        this->publishPointCloud(segmentation_plane_publisher_, *cloud_plane);
        this->publishPointCloud(segmentation_obj_publisher_, *cloud_f);
    }

    void publishPointCloud(
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
        pcl::PointCloud<pcl::PointXYZ> point_cloud)
    {
        sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(point_cloud, *pc2_cloud);
        pc2_cloud->header.frame_id = world_frame;
        pc2_cloud->header.stamp = this->get_clock()->now();
        publisher->publish(*pc2_cloud);
    }

    void update_parameters()
    {
        voxel_leaf_size = static_cast<float>(this->get_parameter("voxel_leaf_size").as_double());
        x_filter_min = this->get_parameter("x_filter_min").as_double();
        x_filter_max = this->get_parameter("x_filter_max").as_double();
        y_filter_min = this->get_parameter("y_filter_min").as_double();
        y_filter_max = this->get_parameter("y_filter_max").as_double();
        z_filter_min = this->get_parameter("z_filter_min").as_double();
        z_filter_max = this->get_parameter("z_filter_max").as_double();
        plane_max_iter = this->get_parameter("plane_max_iterations").as_int();
        plane_dist_thresh = this->get_parameter("plane_distance_threshold").as_double();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;

    /*
     * Publishers
     */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_grid_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passthrough_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr segmentation_plane_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr segmentation_obj_publisher_;

    /*
     * Parameters
     */
    std::string cloud_topic;
    std::string world_frame;
    std::string camera_frame;
    float voxel_leaf_size;
    float x_filter_min;
    float x_filter_max;
    float y_filter_min;
    float y_filter_max;
    float z_filter_min;
    float z_filter_max;
    int plane_max_iter;
    float plane_dist_thresh;

    /*
     * TF
     */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
    /*
     * INITIALIZE ROS NODE
     */
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
