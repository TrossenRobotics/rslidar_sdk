/**
 * Copyright 2024 Trossen Robotics - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef TROSSEN_RSLIDAR_NODE_HPP_
#define TROSSEN_RSLIDAR_NODE_HPP_

#include <memory>
#include <sstream>
#include <string>

#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/utility/sync_queue.hpp>
#include <rslidar_helper.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <utility/yaml_reader.hpp>

namespace robosense
{
namespace lidar
{

using CallbackReturn = nav2_util::CallbackReturn;

class PointCloudLFNode : public nav2_util::LifecycleNode
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

    public:
    // Constructor for LifeCycle Node class
    explicit PointCloudLFNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    // Desctructor for Lifecycle Node class
    ~PointCloudLFNode();

    /**
     * @brief Configure
     * @return SUCCESS or FAILURE
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Activate
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Deactivate
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Cleanup
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Shut down
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

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
