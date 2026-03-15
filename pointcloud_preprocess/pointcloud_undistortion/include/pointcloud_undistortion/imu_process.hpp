#ifndef POINTCLOUD_UNDISTORTION__IMU_PROCESS_HPP_
#define POINTCLOUD_UNDISTORTION__IMU_PROCESS_HPP_

#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "pointcloud_undistortion/ikfom.hpp"
#include "pointcloud_undistortion/pointcloud_undistortion_node.hpp"

namespace pointcloud_undistortion
{

struct Pose6D
{
  double offset_time{0.0};
  double acc[3]{0.0, 0.0, 0.0};
  double gyr[3]{0.0, 0.0, 0.0};
  double vel[3]{0.0, 0.0, 0.0};
  double pos[3]{0.0, 0.0, 0.0};
  double rot[9]{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
};

class ImuProcess
{
public:
  double first_lidar_time{0.0};

  ImuProcess();

  void setExtrinsic(const Eigen::Vector3d & transl, const Eigen::Matrix3d & rot);
  void setUseImuTranslationCompensation(bool enabled);
  void Process(const MeasureGroup & meas, PointCloudXYZI::Ptr & pcl_un);
  void setLogger(const rclcpp::Logger & logger);

private:
  void reset();
  void imuInit(const MeasureGroup & meas, EsEkf & kf_state, int & n);
  void undistortPcl(const MeasureGroup & meas, EsEkf & kf_state, PointCloudXYZI & pcl_out);

  Eigen::Matrix<double, 12, 12> q_;
  Eigen::Vector3d cov_acc_;
  Eigen::Vector3d cov_gyr_;
  Eigen::Vector3d cov_acc_scale_;
  Eigen::Vector3d cov_gyr_scale_;
  Eigen::Vector3d cov_bias_gyr_;
  Eigen::Vector3d cov_bias_acc_;
  PointCloudXYZI::Ptr cur_pcl_un_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
  std::vector<Pose6D> imu_pose_;
  Eigen::Matrix3d lidar_r_wrt_imu_;
  Eigen::Vector3d lidar_t_wrt_imu_;
  Eigen::Vector3d mean_acc_;
  Eigen::Vector3d mean_gyr_;
  Eigen::Vector3d angvel_last_;
  Eigen::Vector3d acc_s_last_;
  double start_timestamp_{-1.0};
  double last_lidar_end_time_{0.0};
  int init_iter_num_{500};
  bool first_frame_{true};
  bool imu_need_init_{true};
  bool use_imu_translation_compensation_{false};
  EsEkf kf_state_;
  rclcpp::Logger logger_{rclcpp::get_logger("pointcloud_undistortion.imu_process")};
};

}  // namespace pointcloud_undistortion

#endif  // POINTCLOUD_UNDISTORTION__IMU_PROCESS_HPP_
