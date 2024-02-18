#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "pcl_ros/transforms.hpp"

class PointCloudTransformer : public rclcpp::Node {
public:
  PointCloudTransformer()
      : Node("point_cloud_transformer"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    this->declare_parameter("input_topic", "/vlp16/velodyne_points");
    this->declare_parameter("output_topic", "/hdl64e/vlp16_points");
    this->declare_parameter("output_frame", "hdl64e");

    output_frame_ = this->get_parameter("output_frame").get_value<std::string>();
    input_topic_ = this->get_parameter("input_topic").get_value<std::string>();
    output_topic_ = this->get_parameter("output_topic").get_value<std::string>();

    //Create Pub Sub
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
    point_cloud_subscriber_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Initialized");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "PointCloud Callback...");
    auto transformed_point_cloud = transformPointCloud(msg);
    // Publish the transformed point cloud
    if (transformed_point_cloud) {
      publisher_->publish(*transformed_point_cloud);
    }
  }

  std::unique_ptr<sensor_msgs::msg::PointCloud2> transformPointCloud(
            const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg) {
        // RCLCPP_INFO(this->get_logger(), "Transforming point cloud...");
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
                    output_frame_, point_cloud_msg->header.frame_id, rclcpp::Time(), rclcpp::Duration::from_seconds(1.0));

            sensor_msgs::msg::PointCloud2 transformed_point_cloud;
            pcl_ros::transformPointCloud(output_frame_, *point_cloud_msg, transformed_point_cloud, tf_buffer_);
            return std::make_unique<sensor_msgs::msg::PointCloud2>(transformed_point_cloud);

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            return nullptr;
        }
    }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string output_frame_;
  std::string input_topic_;
  std::string output_topic_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto point_cloud_transformer = std::make_shared<PointCloudTransformer>();
  rclcpp::spin(point_cloud_transformer);
  rclcpp::shutdown();
  return 0;
}
