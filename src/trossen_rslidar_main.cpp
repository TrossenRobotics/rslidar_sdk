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

// static void sigHandler(int sig)
// {
//   RS_MSG << "RoboSense-LiDAR-Driver is stopping....." << RS_REND;
//   g_cv.notify_all();
// }

// void show_usage(std::string name)
// {
//   std::cerr << "Usage: " << name << " <option(s)> SOURCES"
//             << "Options:\n"
//             << "\t--config-path\t\tPass the config file absolute path\n"
//             << std::endl;
// }

int main(int argc, char ** argv)
{
//   signal(SIGINT, sigHandler);  //< bind ctrl+c signal with the sigHandler function

  RS_TITLE << "********************************************************" << RS_REND;
  RS_TITLE << "**********    RSLidar_SDK Version: v" << RSLIDAR_VERSION_MAJOR
           << "." << RSLIDAR_VERSION_MINOR
           << "." << RSLIDAR_VERSION_PATCH << "     **********" << RS_REND;
  RS_TITLE << "********************************************************" << RS_REND;

//   std::unique_lock<std::mutex> lck(g_mtx);
//   g_cv.wait(lck);

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
