#ifndef TROSSEN_RSLIDAR_HPP_
#define TROSSEN_RSLIDAR_HPP_

#include "nav2_util/lifecycle_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include "source.hpp"
#include "source_driver.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <rslidar_helper.hpp>
#include "utility/yaml_reader.hpp"

namespace trossen_rslidar
{

using CallbackReturn = nav2_util::CallbackReturn;
using robosense::lidar::Source;
using robosense::lidar::SourceType;
using robosense::lidar::SourceDriver;
using namespace robosense::lidar;

class DestinationPointCloud : public nav2_util::LifecycleNode
{
    private:

    std::string point_cloud_topic_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pub_pointcloud_;

    std::string frame_id_;

    bool send_by_rows_;

    bool dense_points_;

    std::shared_ptr<Source> source_;

    public:
    DestinationPointCloud(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~DestinationPointCloud();

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

    void sendPointCloud(const LidarPointCloudMsg& msg);


};

}  // namespace trossen_rslidar
#endif  // TROSSENRS_LIDAR_HPP_