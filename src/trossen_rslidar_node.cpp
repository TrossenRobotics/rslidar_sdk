/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

Modified by: Trossen Robotics
Copyright (c) 2024 Trossen Robotics

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/


#include <trossen_rslidar_node.hpp>

namespace robosense
{
namespace lidar
{

PointCloudLFNode::PointCloudLFNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("trossen_rslidar", "", options),
  diagnostic_updater_(this)
{
  declare_parameter<std::string>("ros_frame_id", "rslidar");
  declare_parameter<std::string>("ros_send_point_cloud_topic", "points");
  declare_parameter<bool>("ros_send_by_rows", false);
  declare_parameter<bool>("show_driver_config", false);

  // input related
  declare_parameter<uint16_t>("msop_port", 6699);
  declare_parameter<uint16_t>("difop_port", 7788);
  declare_parameter<std::string>("host_address", "0.0.0.0");
  declare_parameter<std::string>("group_address", "0.0.0.0");
  declare_parameter<bool>("use_vlan", false);
  declare_parameter<std::string>("pcap_path", "");
  declare_parameter<float>("pcap_rate", 1.0);
  declare_parameter<bool>("pcap_repeat", true);
  declare_parameter<uint16_t>("user_layer_bytes", 0);
  declare_parameter<uint16_t>("tail_layer_bytes", 0);

  declare_parameter<std::string>("lidar_type", "RSHELIOS_16P");

  // decoder
  declare_parameter<bool>("wait_for_difop", true);
  declare_parameter<bool>("use_lidar_clock", false);
  declare_parameter<float>("min_distance", 0.2);
  declare_parameter<float>("max_distance", 200.0);
  declare_parameter<float>("start_angle", 0.0);
  declare_parameter<float>("end_angle", 360.0);
  declare_parameter<bool>("dense_points", false);
  declare_parameter<bool>("ts_first_point", false);

  // mechanical decoder
  declare_parameter<bool>("config_from_file", false);
  declare_parameter<std::string>("angle_path", "");

  declare_parameter<uint16_t>("split_frame_mode", 1);

  declare_parameter<float>("split_angle", 0.0);
  declare_parameter<uint16_t>("num_blks_split", 0);

  // transform
  declare_parameter<float>("x", 0.0);
  declare_parameter<float>("y", 0.0);
  declare_parameter<float>("z", 0.0);
  declare_parameter<float>("roll", 0.0);
  declare_parameter<float>("pitch", 0.0);
  declare_parameter<float>("yaw", 0.0);

  // diagnostic parameters
  declare_parameter<std::string>("lidar_diagnostic_name", "lidar");
  declare_parameter<double>("minimum_frequency", 10.0);
  declare_parameter<double>("maximum_frequency", 10.0);
  declare_parameter<double>("frequency_tolerance", 0.1);
  declare_parameter<int>("frequency_window", 5);

  get_parameter<std::string>("ros_frame_id", this->frame_id_);
  driver_parameters_.frame_id = this->frame_id_;

  get_parameter<std::string>("ros_send_point_cloud_topic", this->point_cloud_topic_);
  get_parameter<bool>("ros_send_by_rows", this->send_by_rows_);
  get_parameter<bool>("show_driver_config", this->show_driver_config_);

  // input related
  get_parameter<uint16_t>("msop_port", driver_parameters_.input_param.msop_port);
  get_parameter<uint16_t>("difop_port", driver_parameters_.input_param.difop_port);
  get_parameter<std::string>("host_address", driver_parameters_.input_param.host_address);
  get_parameter<std::string>("group_address", driver_parameters_.input_param.group_address);
  get_parameter<bool>("use_vlan", driver_parameters_.input_param.use_vlan);
  get_parameter<std::string>("pcap_path", driver_parameters_.input_param.pcap_path);
  get_parameter<float>("pcap_rate", driver_parameters_.input_param.pcap_rate);
  get_parameter<bool>("pcap_repeat", driver_parameters_.input_param.pcap_repeat);
  get_parameter<uint16_t>("user_layer_bytes", driver_parameters_.input_param.user_layer_bytes);
  get_parameter<uint16_t>("tail_layer_bytes", driver_parameters_.input_param.tail_layer_bytes);

  std::string lidar_type;
  get_parameter<std::string>("lidar_type", lidar_type);
  driver_parameters_.lidar_type = strToLidarType(lidar_type);

  // decoder
  get_parameter<bool>("wait_for_difop", driver_parameters_.decoder_param.wait_for_difop);
  get_parameter<bool>("use_lidar_clock", driver_parameters_.decoder_param.use_lidar_clock);
  get_parameter<float>("min_distance", driver_parameters_.decoder_param.min_distance);
  get_parameter<float>("max_distance", driver_parameters_.decoder_param.max_distance);
  get_parameter<float>("start_angle", driver_parameters_.decoder_param.start_angle);
  get_parameter<float>("end_angle", driver_parameters_.decoder_param.end_angle);
  get_parameter<bool>("dense_points", driver_parameters_.decoder_param.dense_points);
  get_parameter<bool>("ts_first_point", driver_parameters_.decoder_param.ts_first_point);

  // mechanical decoder
  get_parameter<bool>("config_from_file", driver_parameters_.decoder_param.config_from_file);
  get_parameter<std::string>("angle_path", driver_parameters_.decoder_param.angle_path);

  uint16_t split_frame_mode;
  get_parameter<uint16_t>("split_frame_mode", split_frame_mode);
  driver_parameters_.decoder_param.split_frame_mode = SplitFrameMode(split_frame_mode);

  get_parameter<float>("split_angle", driver_parameters_.decoder_param.split_angle);
  get_parameter<uint16_t>("num_blks_split", driver_parameters_.decoder_param.num_blks_split);

  // transform
  get_parameter<float>("x", driver_parameters_.decoder_param.transform_param.x);
  get_parameter<float>("y", driver_parameters_.decoder_param.transform_param.y);
  get_parameter<float>("z", driver_parameters_.decoder_param.transform_param.z);
  get_parameter<float>("roll", driver_parameters_.decoder_param.transform_param.roll);
  get_parameter<float>("pitch", driver_parameters_.decoder_param.transform_param.pitch);
  get_parameter<float>("yaw", driver_parameters_.decoder_param.transform_param.yaw);

  // diagnostic parameters
  std::string lidar_diagnostic_name;
  get_parameter<std::string>("lidar_diagnostic_name", lidar_diagnostic_name);
  get_parameter<double>("minimum_frequency", minimum_frequency_);
  get_parameter<double>("maximum_frequency", maximum_frequency_);
  get_parameter<double>("frequency_tolerance", frequency_tolerance_);
  get_parameter<int>("frequency_window", frequency_window_);

  RCLCPP_INFO(
    get_logger(),
    "Creating diagnostic publisher for lidar '%s' with minimum frequency %f, maximum frequency %f",
    lidar_diagnostic_name.c_str(),
    minimum_frequency_,
    maximum_frequency_);

  diagnostic_updater_.setHardwareID(lidar_diagnostic_name);
  pub_pointcloud_diagnostic_ = std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
    point_cloud_topic_,
    diagnostic_updater_,
    diagnostic_updater::FrequencyStatusParam(
      &minimum_frequency_,
      &maximum_frequency_,
      frequency_tolerance_,
      frequency_window_));

  driver_parameters_.input_type = InputType::ONLINE_LIDAR;

  if (this->show_driver_config_)
  {
    driver_parameters_.print();
  }

  driver_ptr_.reset(new lidar::LidarDriver<LidarPointCloudMsg>());
  driver_ptr_->regPointCloudCallback(std::bind(&PointCloudLFNode::getPointCloud, this),
      std::bind(&PointCloudLFNode::putPointCloud, this, std::placeholders::_1));
  driver_ptr_->regExceptionCallback(
      std::bind(&PointCloudLFNode::putException, this, std::placeholders::_1));

  if (!driver_ptr_->init(driver_parameters_))
  {
    RCLCPP_FATAL(get_logger(), "Failed to bring up RSLIDAR SDK Node: Driver Initialize Error");
    exit(EXIT_FAILURE);
  }

  driver_ptr_->start();

  to_exit_process_ = false;
  point_cloud_process_thread_ = std::thread(std::bind(&PointCloudLFNode::processPointCloud, this));

  pub_pointcloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    this->point_cloud_topic_,
    rclcpp::SensorDataQoS());

  RCLCPP_INFO(get_logger(), "RSLIDAR SDK node is up.");
}

PointCloudLFNode::~PointCloudLFNode()
{
  driver_ptr_->stop();
  free_point_cloud_queue_.clear();
  point_cloud_queue_.clear();

  to_exit_process_ = true;
  point_cloud_process_thread_.join();
  driver_ptr_.reset();
  pub_pointcloud_.reset();
  pub_pointcloud_diagnostic_.reset();
}

void PointCloudLFNode::sendPointCloud(const LidarPointCloudMsg& msg)
{
  pub_pointcloud_->publish(robosense::lidar::toRosMsg(msg, frame_id_, send_by_rows_));
  pub_pointcloud_diagnostic_->tick();
}

std::shared_ptr<LidarPointCloudMsg> PointCloudLFNode::getPointCloud(void)
{
  std::shared_ptr<LidarPointCloudMsg> point_cloud = free_point_cloud_queue_.pop();

  if (point_cloud.get() != NULL)
  {
    return point_cloud;
  }

  return std::make_shared<LidarPointCloudMsg>();
}

void PointCloudLFNode::putPointCloud(std::shared_ptr<LidarPointCloudMsg> msg)
{
  point_cloud_queue_.push(msg);
}

void PointCloudLFNode::processPointCloud()
{
  while (!to_exit_process_ && rclcpp::ok())
  {
    std::shared_ptr<LidarPointCloudMsg> msg = point_cloud_queue_.popWait(1000);
    if (msg.get() == NULL)
    {
      continue;
    }

    sendPointCloud(*msg);

    free_point_cloud_queue_.push(msg);
  }
}

void PointCloudLFNode::putException(const lidar::Error& msg)
{
  switch (msg.error_code_type)
  {
    case lidar::ErrCodeType::INFO_CODE:
      RS_INFO << msg.toString() << RS_REND;
      break;
    case lidar::ErrCodeType::WARNING_CODE:
      RS_WARNING << msg.toString() << RS_REND;
      break;
    case lidar::ErrCodeType::ERROR_CODE:
      RS_ERROR << msg.toString() << RS_REND;
      break;
  }
}

}  // namespace lidar
}  // namespace robosense

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robosense::lidar::PointCloudLFNode)
