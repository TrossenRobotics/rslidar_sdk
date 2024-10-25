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


#ifndef TROSSEN_RSLIDAR_NODE_HPP_
#define TROSSEN_RSLIDAR_NODE_HPP_

#include <memory>
#include <sstream>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "diagnostic_updater/update_functions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rs_driver/api/lidar_driver.hpp"
#include "rs_driver/utility/sync_queue.hpp"
#include "rslidar_helper.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "utility/yaml_reader.hpp"

namespace robosense
{
namespace lidar
{

class PointCloudLFNode : public rclcpp::Node
{
    private:
    // ROS topic for publishing point cloud
    std::string point_cloud_topic_;

    // Publisher to publish PointCloud2 message
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pub_pointcloud_;

    // Frame ID for the LiDAR device
    std::string frame_id_;

    // TODO(aniketmpatil): What does this control?
    bool send_by_rows_;

    // Publish dense point cloud
    bool dense_points_;  // TODO(aniketmpatil): Analyze performance

    // Flag to show lidar driver configuration on setup
    bool show_driver_config_;

    // Driver Parameters to be used by RS Driver
    lidar::RSDriverParam driver_parameters_;

    // Robosense Driver object
    std::shared_ptr<lidar::LidarDriver<LidarPointCloudMsg>> driver_ptr_;

    // PointCloud queue to store LiDAR PointCloud message
    SyncQueue<std::shared_ptr<LidarPointCloudMsg>> free_point_cloud_queue_;

    // PointCloud queue to store LiDAR PointCloud message
    SyncQueue<std::shared_ptr<LidarPointCloudMsg>> point_cloud_queue_;

    // Thread to run the Process PointCloud function
    std::thread point_cloud_process_thread_;

    // Flag to terminate the Process PointCloud thread
    bool to_exit_process_;

    // Diagnostic updater - used to report status of this node and related hardware
    diagnostic_updater::Updater diagnostic_updater_;

    // Diagnostic for the PointCloud2 publisher's frequency
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> pub_pointcloud_diagnostic_;

    public:
    // Constructor for LifeCycle Node class
    explicit PointCloudLFNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    // Desctructor for Lifecycle Node class
    ~PointCloudLFNode();

    /**
     * @brief Function to publish PointCloud2 message
     * @param msg LidarPointCloud message to be published
     * @return void
     */
    void sendPointCloud(const LidarPointCloudMsg& msg);

    /**
     * @brief Function to pop a LidarPointCloud message from the free_point_cloud queue
     * @return pointer to the popped LidarPointCloud Message
     */
    std::shared_ptr<LidarPointCloudMsg> getPointCloud(void);

    /**
     * @brief Function to pop a LidarPointCloud message from the point_cloud queue
     * @param msg LidarPointCloud message to be pushed to queue
     * @return void
     */
    void putPointCloud(std::shared_ptr<LidarPointCloudMsg> msg);

    /**
     * @brief Function to get and send a LidarPointCloud message
     * @return void
     */
    void processPointCloud();

    /**
     * @brief Function to handle exceptions using the RS Driver Error codes
     * @param msg Error Message to display
     * @return void
     */
    void putException(const lidar::Error& msg);
};
}  // namespace lidar
}  // namespace robosense

#endif  // TROSSEN_RSLIDAR_NODE_HPP_
