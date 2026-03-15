#include "pointcloud_undistortion/preprocess.hpp"

#include <cmath>

#include <pcl_conversions/pcl_conversions.h>

namespace pointcloud_undistortion
{

Preprocess::Preprocess() = default;

void Preprocess::process(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg, PointCloudXYZI::Ptr & pcl_out)
{
  rsHandler(msg);
  *pcl_out = pl_surf_;
}

void Preprocess::setScanLine(int scan_line)
{
  scan_line_ = scan_line;
}

void Preprocess::setScanRate(int scan_rate)
{
  scan_rate_ = scan_rate;
}

void Preprocess::setBlind(double blind)
{
  blind_ = blind;
}

void Preprocess::setDistanceThreshold(double distance_threshold)
{
  distance_threshold_ = distance_threshold;
}

void Preprocess::setPointFilterNum(int point_filter_num)
{
  point_filter_num_ = point_filter_num;
}

void Preprocess::setLogger(const rclcpp::Logger & logger)
{
  logger_ = logger;
}

void Preprocess::rsHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  pl_surf_.clear();

  pcl::PointCloud<rslidar_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  const int plsize = static_cast<int>(pl_orig.points.size());
  if (plsize == 0) {
    return;
  }

  pl_surf_.reserve(plsize);

  const double omega_l = 0.361 * static_cast<double>(scan_rate_);
  std::vector<bool> is_first(static_cast<std::size_t>(scan_line_), true);
  std::vector<double> yaw_fp(static_cast<std::size_t>(scan_line_), 0.0);
  std::vector<float> yaw_last(static_cast<std::size_t>(scan_line_), 0.0F);
  std::vector<float> time_last(static_cast<std::size_t>(scan_line_), 0.0F);

  given_offset_time_ = pl_orig.points[static_cast<std::size_t>(plsize - 1)].timestamp > 0.0;

  for (int i = 0; i < plsize; ++i) {
    PointType added_pt;
    added_pt.normal_x = 0.0F;
    added_pt.normal_y = 0.0F;
    added_pt.normal_z = 0.0F;
    added_pt.x = pl_orig.points[static_cast<std::size_t>(i)].x;
    added_pt.y = pl_orig.points[static_cast<std::size_t>(i)].y;
    added_pt.z = pl_orig.points[static_cast<std::size_t>(i)].z;
    added_pt.intensity = static_cast<float>(pl_orig.points[static_cast<std::size_t>(i)].intensity);
    added_pt.curvature =
      static_cast<float>(
      (pl_orig.points[static_cast<std::size_t>(i)].timestamp - pl_orig.points.front().timestamp) *
      1000.0);

    if (!given_offset_time_) {
      const int layer = static_cast<int>(pl_orig.points[static_cast<std::size_t>(i)].ring);
      if (layer >= scan_line_ || layer < 0) {
        continue;
      }

      const double yaw_angle = std::atan2(added_pt.y, added_pt.x) * 57.2957;

      if (is_first[static_cast<std::size_t>(layer)]) {
        yaw_fp[static_cast<std::size_t>(layer)] = yaw_angle;
        is_first[static_cast<std::size_t>(layer)] = false;
        added_pt.curvature = 0.0F;
        yaw_last[static_cast<std::size_t>(layer)] = static_cast<float>(yaw_angle);
        time_last[static_cast<std::size_t>(layer)] = added_pt.curvature;
        continue;
      }

      if (yaw_angle <= yaw_fp[static_cast<std::size_t>(layer)]) {
        added_pt.curvature = static_cast<float>(
          (yaw_fp[static_cast<std::size_t>(layer)] - yaw_angle) / omega_l);
      } else {
        added_pt.curvature = static_cast<float>(
          (yaw_fp[static_cast<std::size_t>(layer)] - yaw_angle + 360.0) / omega_l);
      }

      if (added_pt.curvature < time_last[static_cast<std::size_t>(layer)]) {
        added_pt.curvature += static_cast<float>(360.0 / omega_l);
      }

      yaw_last[static_cast<std::size_t>(layer)] = static_cast<float>(yaw_angle);
      time_last[static_cast<std::size_t>(layer)] = added_pt.curvature;
    }

    if (i % point_filter_num_ == 0) {
      const double range_sq =
        static_cast<double>(added_pt.x) * added_pt.x +
        static_cast<double>(added_pt.y) * added_pt.y +
        static_cast<double>(added_pt.z) * added_pt.z;
      const bool outside_blind = range_sq > (blind_ * blind_);
      const bool inside_distance_threshold =
        distance_threshold_ <= 0.0 || range_sq < (distance_threshold_ * distance_threshold_);
      if (outside_blind && inside_distance_threshold) {
        pl_surf_.push_back(added_pt);
      }
    }
  }

  RCLCPP_INFO_THROTTLE(
    logger_,
    *clock_,
    2000,
    "preprocess rs_lidar: input_points=%d output_points=%zu given_offset_time=%s point_filter_num=%d "
    "blind=%.3f distance_threshold=%.3f",
    plsize,
    pl_surf_.size(),
    given_offset_time_ ? "true" : "false",
    point_filter_num_,
    blind_,
    distance_threshold_);
}

}  // namespace pointcloud_undistortion
