#include "composable/rslidar_managed.hpp"

namespace robosense
{
namespace lidar
{

PointCloudLFNode::PointCloudLFNode(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("trossen_rslidar", "", options)
{
    declare_parameter("ros_frame_id", "rslidar");
    declare_parameter("ros_send_point_cloud_topic", "rslidar_points");
    declare_parameter("ros_send_by_rows", false);
    declare_parameter("dense_points", false);
}

PointCloudLFNode::~PointCloudLFNode()
{
}

CallbackReturn PointCloudLFNode::on_configure(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_DEBUG(get_logger(), "Configuring...");

  point_cloud_topic_ = "points";
  frame_id_ = "lidar_front";
  send_by_rows_ = false;
  dense_points_ = false;

  RCLCPP_INFO(get_logger(), "Configuration complete.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PointCloudLFNode::on_activate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_DEBUG(this->get_logger(), "Activating...");

  pub_pointcloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    this->point_cloud_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  RCLCPP_INFO(get_logger(), "Activation complete.");
  return CallbackReturn::SUCCESS;
}

void PointCloudLFNode::sendPointCloud(const LidarPointCloudMsg& msg)
{
  pub_pointcloud_->publish(robosense::lidar::toRosMsg(msg, frame_id_, send_by_rows_));
}

CallbackReturn PointCloudLFNode::on_deactivate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_DEBUG(get_logger(), "Deactivating...");
  RCLCPP_INFO(get_logger(), "Deactivation complete");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PointCloudLFNode::on_cleanup(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_DEBUG(get_logger(), "Cleaning up...");
  RCLCPP_INFO(get_logger(), "Cleanup complete.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PointCloudLFNode::on_shutdown(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_DEBUG(get_logger(), "Shutting down...");
  RCLCPP_INFO(get_logger(), "Shut down complete.");
  return CallbackReturn::SUCCESS;
}

}
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robosense::lidar::PointCloudLFNode)
