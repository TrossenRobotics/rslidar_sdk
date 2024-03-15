/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

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

#include <signal.h>

#include <manager/node_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rs_driver/macro/version.hpp>
#include <composable/source_driver.hpp>
#include <composable/rslidar_managed.hpp>

using NodeManager = robosense::lidar::NodeManager;
using namespace robosense::lidar;
using SourceDriver = robosense::lidar::SourceDriver;

std::mutex g_mtx;
std::condition_variable g_cv;

static void sigHandler(int sig)
{
  RS_MSG << "RoboSense-LiDAR-Driver is stopping....." << RS_REND;
  g_cv.notify_all();
}

void show_usage(std::string name)
{
  std::cerr << "Usage: " << name << " <option(s)> SOURCES"
            << "Options:\n"
            << "\t--config-path\t\tPass the config file absolute path\n"
            << std::endl;
}

std::string arg_parser(int argc, char * argv[])
{
  std::string path = "";
  if (argc > 1) {
    for (int i = 1; i < argc; i++) {
      std::string arg = argv[i];
      if (arg == "--config_filepath") {
        path = argv[i+1];
        return path;
      }
      i++;
    }
  } else {
    std::cerr << "No input given. Resorting to default parameters\n";
    show_usage(argv[0]);
  }
  return path;
}

int main(int argc, char ** argv)
{
  signal(SIGINT, sigHandler);  //< bind ctrl+c signal with the sigHandler function

  RS_TITLE << "********************************************************" << RS_REND;
  RS_TITLE << "**********    RSLidar_SDK Version: v" << RSLIDAR_VERSION_MAJOR
           << "." << RSLIDAR_VERSION_MINOR
           << "." << RSLIDAR_VERSION_PATCH << "     **********" << RS_REND;
  RS_TITLE << "********************************************************" << RS_REND;

  rclcpp::init(argc, argv);

  // Get the YAML Config file path
  std::string config_path;
  config_path = arg_parser(argc, argv);
  if (config_path == "") {
    config_path = static_cast<std::string>(PROJECT_PATH);
    config_path += "/config/config.yaml";
  }

  // Read the YAML file
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path);
  } catch (...) {
    RS_ERROR << "The format of config file " << config_path
             << " is wrong. Please check (e.g. indentation)." << RS_REND;
    return -1;
  }

//   std::shared_ptr<NodeManager> demo_ptr = std::make_shared<NodeManager>();
//   demo_ptr->init(config);
//   demo_ptr->start();


  YAML::Node config;
  std::shared_ptr<Source> source_;
  YAML::Node lidar_config = yamlSubNodeAbort(config, "lidar");
  source_ = std::make_shared<robosense::lidar::SourceDriver>(SourceType::MSG_FROM_LIDAR);
  source_->init(lidar_config[0]);

  auto options = rclcpp::NodeOptions();
  std::shared_ptr<trossen_rslidar::DestinationPointCloud> node = std::make_shared<trossen_rslidar::DestinationPointCloud>(options);

  node->configure();
  node->activate();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  source_->regPointCloudCallback(node);
  source_->start();


  RS_MSG << "RoboSense-LiDAR-Driver is running....." << RS_REND;

  std::unique_lock<std::mutex> lck(g_mtx);
  g_cv.wait(lck);

  return 0;
}
