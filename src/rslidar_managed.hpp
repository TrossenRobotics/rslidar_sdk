#ifndef TROSSEN_RSLIDAR_HPP_
#define TROSSEN_RSLIDAR_HPP_

#include <sstream>

#include <rslidar_helper.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <utility/yaml_reader.hpp>

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/utility/sync_queue.hpp>

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

    lidar::RSDriverParam driver_parameters_;

    std::shared_ptr<lidar::LidarDriver<LidarPointCloudMsg>> driver_ptr_;
    SyncQueue<std::shared_ptr<LidarPointCloudMsg>> free_point_cloud_queue_;
    SyncQueue<std::shared_ptr<LidarPointCloudMsg>> point_cloud_queue_;
    std::thread point_cloud_process_thread_;
    bool to_exit_process_;

    std::mutex g_mtx_;
    std::condition_variable g_cv_;

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

    std::shared_ptr<LidarPointCloudMsg> getPointCloud(void);

    void putPointCloud(std::shared_ptr<LidarPointCloudMsg> msg);

    void processPointCloud();

    void putException(const lidar::Error& msg);

};
}
}
#endif  // TROSSENRS_LIDAR_HPP_