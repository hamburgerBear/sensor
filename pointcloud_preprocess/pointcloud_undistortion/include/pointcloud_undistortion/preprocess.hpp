#ifndef POINTCLOUD_UNDISTORTION__PREPROCESS_HPP_
#define POINTCLOUD_UNDISTORTION__PREPROCESS_HPP_

#include <cstdint>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rslidar_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring{0U};
  double timestamp{0.0};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace rslidar_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(
  rslidar_ros::Point,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (double, timestamp, timestamp))

namespace pointcloud_undistortion
{

using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;

class Preprocess
{
public:
  Preprocess();

  void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg, PointCloudXYZI::Ptr & pcl_out);
  void setScanLine(int scan_line);
  void setScanRate(int scan_rate);
  void setBlind(double blind);
  void setDistanceThreshold(double distance_threshold);
  void setPointFilterNum(int point_filter_num);
  void setLogger(const rclcpp::Logger & logger);

private:
  void rsHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  PointCloudXYZI pl_surf_;
  int scan_line_{32};
  int scan_rate_{10};
  int point_filter_num_{1};
  double blind_{3.0};
  double distance_threshold_{0.0};
  bool given_offset_time_{false};
  rclcpp::Logger logger_{rclcpp::get_logger("pointcloud_undistortion.preprocess")};
  rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};
};

}  // namespace pointcloud_undistortion

#endif  // POINTCLOUD_UNDISTORTION__PREPROCESS_HPP_
