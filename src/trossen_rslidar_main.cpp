/**
 * Copyright 2024 Trossen Robotics - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <rs_driver/macro/version.hpp>
#include <trossen_rslidar_node.hpp>

int main(int argc, char ** argv)
{
  RS_TITLE << "********************************************************" << RS_REND;
  RS_TITLE << "**********    RSLidar_SDK Version: v" << RSLIDAR_VERSION_MAJOR
           << "." << RSLIDAR_VERSION_MINOR
           << "." << RSLIDAR_VERSION_PATCH << "     **********" << RS_REND;
  RS_TITLE << "********************************************************" << RS_REND;

  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<robosense::lidar::PointCloudLFNode>(options);

  node->configure();
  node->activate();
  RS_MSG << "RoboSense-LiDAR-Driver is running....." << RS_REND;

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
