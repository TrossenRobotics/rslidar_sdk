#ifndef TROSSEN_RSLIDAR_HPP_
#define TROSSEN_RSLIDAR_HPP_

#include <sstream>

#include <composable/rslidar_helper.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
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

    std::string point_cloud_topic_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pub_pointcloud_;

    std::string frame_id_;

    bool send_by_rows_;

    bool dense_points_;

    public:
    PointCloudLFNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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

    void sendPointCloud(const LidarPointCloudMsg& msg);


};
}
}
#endif  // TROSSENRS_LIDAR_HPP_