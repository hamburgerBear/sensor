#ifndef POINTCLOUD_UNDISTORTION__POINTCLOUD_UNDISTORTION_NODE_HPP_
#define POINTCLOUD_UNDISTORTION__POINTCLOUD_UNDISTORTION_NODE_HPP_

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <mutex>
#include <vector>

#include "pointcloud_undistortion/preprocess.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pointcloud_undistortion
{

class ImuProcess;

struct MeasureGroup
{
  MeasureGroup()
  : lidar_beg_time(0.0),
    lidar_end_time(0.0),
    lidar(std::make_shared<PointCloudXYZI>())
  {
  }

  double lidar_beg_time;
  double lidar_end_time;
  std::string lidar_frame_id;
  PointCloudXYZI::Ptr lidar;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu;
};

class PointcloudUndistortionNode : public rclcpp::Node
{
public:
  PointcloudUndistortionNode();
  ~PointcloudUndistortionNode() override;

private:
  bool syncPackages(MeasureGroup & meas);
  void processingLoop();
  void loadParameters();
  void handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void handleImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult handleParameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_publisher_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::string input_topic_;
  std::string output_topic_;
  std::string imu_topic_;
  std::string output_frame_;
  std::vector<double> extrinsic_t_;
  std::vector<double> extrinsic_r_;
  int scan_line_{32};
  int scan_rate_{10};
  int point_filter_num_{1};
  double blind_threshold_{3.0};
  double distance_threshold_{0.0};
  bool use_imu_{true};
  bool running_{true};
  std::thread processing_thread_;
  Preprocess preprocess_;
  std::unique_ptr<ImuProcess> imu_process_;
  bool flg_first_scan_{true};
  double first_lidar_time_{0.0};
  PointCloudXYZI::Ptr feats_undistort_{std::make_shared<PointCloudXYZI>()};
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
  double lidar_end_time_{0.0};
  double lidar_mean_scantime_{0.0};
  int scan_num_{0};
  bool lidar_pushed_{false};
  std::mutex lidar_mutex_;
  std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
  std::deque<std::string> frame_id_buffer_;
  std::deque<double> time_buffer_;
  std::mutex imu_mutex_;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;
  double last_lidar_end_time_{0.0};
  double last_timestamp_imu_{-1.0};
  std::optional<rclcpp::Time> latest_imu_stamp_;
  std::uint64_t imu_count_{0U};
};

}  // namespace pointcloud_undistortion

#endif  // POINTCLOUD_UNDISTORTION__POINTCLOUD_UNDISTORTION_NODE_HPP_
