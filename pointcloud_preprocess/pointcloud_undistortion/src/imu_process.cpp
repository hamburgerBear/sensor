#include "pointcloud_undistortion/imu_process.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace pointcloud_undistortion
{

namespace
{

constexpr int kMaxIniCount = 10;
constexpr double kTimingLogIntervalMs = 2000.0;

bool timeList(const PointType & left, const PointType & right)
{
  return left.curvature < right.curvature;
}

Pose6D setPose6D(
  double t, const Eigen::Vector3d & acc, const Eigen::Vector3d & gyr,
  const Eigen::Vector3d & vel, const Eigen::Vector3d & pos, const Eigen::Matrix3d & rot)
{
  Pose6D pose;
  pose.offset_time = t;
  for (int i = 0; i < 3; ++i) {
    pose.acc[i] = acc(i);
    pose.gyr[i] = gyr(i);
    pose.vel[i] = vel(i);
    pose.pos[i] = pos(i);
    for (int j = 0; j < 3; ++j) {
      pose.rot[i * 3 + j] = rot(i, j);
    }
  }
  return pose;
}

Eigen::Vector3d vecFromArray(const double values[3])
{
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

Eigen::Matrix3d matFromArray(const double values[9])
{
  Eigen::Matrix3d matrix;
  matrix <<
    values[0], values[1], values[2],
    values[3], values[4], values[5],
    values[6], values[7], values[8];
  return matrix;
}

}  // namespace

ImuProcess::ImuProcess()
: q_(processNoiseCov()),
  cov_acc_(0.1, 0.1, 0.1),
  cov_gyr_(0.1, 0.1, 0.1),
  cov_acc_scale_(0.1, 0.1, 0.1),
  cov_gyr_scale_(0.1, 0.1, 0.1),
  cov_bias_gyr_(0.0001, 0.0001, 0.0001),
  cov_bias_acc_(0.0001, 0.0001, 0.0001),
  cur_pcl_un_(std::make_shared<PointCloudXYZI>()),
  lidar_r_wrt_imu_(Eigen::Matrix3d::Identity()),
  lidar_t_wrt_imu_(Eigen::Vector3d::Zero()),
  mean_acc_(0.0, 0.0, -1.0),
  mean_gyr_(0.0, 0.0, 0.0),
  angvel_last_(Eigen::Vector3d::Zero()),
  acc_s_last_(Eigen::Vector3d::Zero())
{
}

void ImuProcess::setLogger(const rclcpp::Logger & logger)
{
  logger_ = logger;
}

void ImuProcess::setExtrinsic(const Eigen::Vector3d & transl, const Eigen::Matrix3d & rot)
{
  lidar_t_wrt_imu_ = transl;
  lidar_r_wrt_imu_ = rot;
}

void ImuProcess::setUseImuTranslationCompensation(bool enabled)
{
  use_imu_translation_compensation_ = enabled;
}

void ImuProcess::reset()
{
  mean_acc_ = Eigen::Vector3d(0.0, 0.0, -1.0);
  mean_gyr_ = Eigen::Vector3d::Zero();
  angvel_last_ = Eigen::Vector3d::Zero();
  imu_need_init_ = true;
  start_timestamp_ = -1.0;
  init_iter_num_ = 500;
  imu_pose_.clear();
  last_imu_.reset();
  cur_pcl_un_ = std::make_shared<PointCloudXYZI>();
}

void ImuProcess::imuInit(const MeasureGroup & meas, EsEkf & kf_state, int & n)
{
  Eigen::Vector3d cur_acc;
  Eigen::Vector3d cur_gyr;

  if (first_frame_) {
    reset();
    n = 1;
    first_frame_ = false;
    const auto & imu_acc = meas.imu.front()->linear_acceleration;
    const auto & gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc_ << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr_ << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
  }

  for (const auto & imu : meas.imu) {
    const auto & imu_acc = imu->linear_acceleration;
    const auto & gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc_ += (cur_acc - mean_acc_) / static_cast<double>(n);
    mean_gyr_ += (cur_gyr - mean_gyr_) / static_cast<double>(n);

    cov_acc_ =
      cov_acc_ * (static_cast<double>(n) - 1.0) / static_cast<double>(n) +
      (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) / static_cast<double>(n);
    cov_gyr_ =
      cov_gyr_ * (static_cast<double>(n) - 1.0) / static_cast<double>(n) +
      (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) / static_cast<double>(n) /
      static_cast<double>(n) * (static_cast<double>(n) - 1.0);
    ++n;
  }

  StateIkfom init_state = kf_state.getX();
  init_state.grav = -mean_acc_ / mean_acc_.norm() * kGravity;
  init_state.bg = mean_gyr_;
  init_state.offset_T_L_I = lidar_t_wrt_imu_;
  init_state.offset_R_L_I = SO3(lidar_r_wrt_imu_);
  kf_state.changeX(init_state);

  Eigen::Matrix<double, 24, 24> init_p = Eigen::Matrix<double, 24, 24>::Identity();
  init_p(6, 6) = init_p(7, 7) = init_p(8, 8) = 0.00001;
  init_p(9, 9) = init_p(10, 10) = init_p(11, 11) = 0.00001;
  init_p(15, 15) = init_p(16, 16) = init_p(17, 17) = 0.0001;
  init_p(18, 18) = init_p(19, 19) = init_p(20, 20) = 0.001;
  init_p(21, 21) = init_p(22, 22) = init_p(23, 23) = 0.00001;
  kf_state.changeP(init_p);
  last_imu_ = meas.imu.back();

  RCLCPP_INFO(
    logger_,
    "imu_init: iter=%d imu_count=%zu lidar=[%.6f, %.6f] mean_acc=[%.6f, %.6f, %.6f] "
    "mean_gyr=[%.6f, %.6f, %.6f]",
    n,
    meas.imu.size(),
    meas.lidar_beg_time,
    meas.lidar_end_time,
    mean_acc_.x(), mean_acc_.y(), mean_acc_.z(),
    mean_gyr_.x(), mean_gyr_.y(), mean_gyr_.z());
}

void ImuProcess::undistortPcl(const MeasureGroup & meas, EsEkf & kf_state, PointCloudXYZI & pcl_out)
{
  const auto total_start = std::chrono::steady_clock::now();
  static auto last_timing_log_time = std::chrono::steady_clock::time_point::min();
  const auto toMilliseconds =
    [](const std::chrono::steady_clock::time_point & start,
    const std::chrono::steady_clock::time_point & end) -> double
    {
      return std::chrono::duration<double, std::milli>(end - start).count();
    };
  const auto shouldLogTiming =
    [&](const std::chrono::steady_clock::time_point & now) -> bool
    {
      if (last_timing_log_time == std::chrono::steady_clock::time_point::min() ||
        toMilliseconds(last_timing_log_time, now) >= kTimingLogIntervalMs)
      {
        last_timing_log_time = now;
        return true;
      }
      return false;
    };

  const auto imu_prepare_start = std::chrono::steady_clock::now();
  auto v_imu = meas.imu;
  if (last_imu_) {
    v_imu.push_front(last_imu_);
  }
  if (v_imu.empty()) {
    pcl_out = *(meas.lidar);
    const auto total_end = std::chrono::steady_clock::now();
    if (shouldLogTiming(total_end)) {
      RCLCPP_INFO(
        logger_,
        "undistortPcl timing: total=%.3f ms imu_prepare=%.3f ms cloud_copy=0.000 ms "
        "imu_propagate=0.000 ms final_predict=0.000 ms pcl_compensate=0.000 ms "
        "imu_count=%zu point_count=%zu (empty imu path)",
        toMilliseconds(total_start, total_end),
        toMilliseconds(imu_prepare_start, total_end),
        v_imu.size(),
        meas.lidar ? meas.lidar->points.size() : 0UL);
    }
    return;
  }
  const auto imu_prepare_end = std::chrono::steady_clock::now();

  const double imu_end_time = rclcpp::Time(v_imu.back()->header.stamp).seconds();
  const double pcl_beg_time = meas.lidar_beg_time;
  const double pcl_end_time = meas.lidar_end_time;

  const auto cloud_copy_start = std::chrono::steady_clock::now();
  pcl_out = *(meas.lidar);
  std::sort(pcl_out.points.begin(), pcl_out.points.end(), timeList);
  const auto cloud_copy_end = std::chrono::steady_clock::now();

  StateIkfom imu_state = kf_state.getX();
  imu_pose_.clear();
  imu_pose_.push_back(
    setPose6D(0.0, acc_s_last_, angvel_last_, imu_state.vel, imu_state.pos, imu_state.rot.matrix()));

  Eigen::Vector3d angvel_avr;
  Eigen::Vector3d acc_avr;
  Eigen::Vector3d acc_imu;
  Eigen::Vector3d vel_imu;
  Eigen::Vector3d pos_imu;
  Eigen::Matrix3d r_imu = Eigen::Matrix3d::Identity();
  double dt = 0.0;
  InputIkfom in;

  const auto imu_propagate_start = std::chrono::steady_clock::now();
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); ++it_imu) {
    auto head = *it_imu;
    auto tail = *(it_imu + 1);
    if (rclcpp::Time(tail->header.stamp).seconds() < last_lidar_end_time_) {
      continue;
    }

    angvel_avr <<
      0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
      0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
      0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr <<
      0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
      0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
      0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
    acc_avr = acc_avr * kGravity / mean_acc_.norm();

    if (rclcpp::Time(head->header.stamp).seconds() < last_lidar_end_time_) {
      dt = rclcpp::Time(tail->header.stamp).seconds() - last_lidar_end_time_;
    } else {
      dt = rclcpp::Time(tail->header.stamp).seconds() - rclcpp::Time(head->header.stamp).seconds();
    }

    in.acc = acc_avr;
    in.gyro = angvel_avr;
    q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
    q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
    q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
    q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;

    kf_state.predict(dt, q_, in);

    imu_state = kf_state.getX();
    angvel_last_ <<
      tail->angular_velocity.x, tail->angular_velocity.y, tail->angular_velocity.z;
    angvel_last_ -= imu_state.bg;

    acc_s_last_ <<
      tail->linear_acceleration.x, tail->linear_acceleration.y, tail->linear_acceleration.z;
    acc_s_last_ = acc_s_last_ * kGravity / mean_acc_.norm();
    acc_s_last_ = imu_state.rot * (acc_s_last_ - imu_state.ba) + imu_state.grav;

    const double offs_t = rclcpp::Time(tail->header.stamp).seconds() - pcl_beg_time;
    imu_pose_.push_back(
      setPose6D(offs_t, acc_s_last_, angvel_last_, imu_state.vel, imu_state.pos, imu_state.rot.matrix()));
  }
  const auto imu_propagate_end = std::chrono::steady_clock::now();

  const auto final_predict_start = std::chrono::steady_clock::now();
  dt = std::abs(pcl_end_time - imu_end_time);
  kf_state.predict(dt, q_, in);
  imu_state = kf_state.getX();
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;
  const auto final_predict_end = std::chrono::steady_clock::now();

  if (pcl_out.points.empty()) {
    const auto total_end = std::chrono::steady_clock::now();
    if (shouldLogTiming(total_end)) {
      RCLCPP_INFO(
        logger_,
        "undistortPcl timing: total=%.3f ms imu_prepare=%.3f ms cloud_copy=%.3f ms "
        "imu_propagate=%.3f ms final_predict=%.3f ms pcl_compensate=0.000 ms "
        "imu_count=%zu point_count=%zu pose_count=%zu (empty cloud path)",
        toMilliseconds(total_start, total_end),
        toMilliseconds(imu_prepare_start, imu_prepare_end),
        toMilliseconds(cloud_copy_start, cloud_copy_end),
        toMilliseconds(imu_propagate_start, imu_propagate_end),
        toMilliseconds(final_predict_start, final_predict_end),
        v_imu.size(),
        pcl_out.points.size(),
        imu_pose_.size());
    }
    return;
  }

  const Eigen::Matrix3d r_li = imu_state.offset_R_L_I.matrix();
  const Eigen::Matrix3d r_il = r_li.transpose();
  const Eigen::Matrix3d r_wi_t = imu_state.rot.matrix().transpose();
  const Eigen::Vector3d t_li = imu_state.offset_T_L_I;
  const Eigen::Vector3d pos_end = imu_state.pos;

  const auto pcl_compensate_start = std::chrono::steady_clock::now();
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = imu_pose_.end() - 1; it_kp != imu_pose_.begin(); --it_kp) {
    auto head = it_kp - 1;
    auto tail = it_kp;
    r_imu = matFromArray(head->rot);
    vel_imu = vecFromArray(head->vel);
    pos_imu = vecFromArray(head->pos);
    acc_imu = vecFromArray(tail->acc);
    angvel_avr = vecFromArray(tail->gyr);

    for (; it_pcl->curvature / 1000.0 > head->offset_time; --it_pcl) {
      dt = it_pcl->curvature / 1000.0 - head->offset_time;
      const Eigen::Matrix3d r_i = r_imu * SO3::exp(angvel_avr * dt).matrix();

      const Eigen::Vector3d p_i(it_pcl->x, it_pcl->y, it_pcl->z);
      const Eigen::Vector3d p_lidar_in_imu = r_li * p_i + t_li;
      Eigen::Vector3d p_compensate;
      if (use_imu_translation_compensation_) {
        const Eigen::Vector3d t_ei = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - pos_end;
        p_compensate = r_il * (r_wi_t * (r_i * p_lidar_in_imu + t_ei) - t_li);
      } else {
        p_compensate = r_il * (r_wi_t * (r_i * p_lidar_in_imu) - t_li);
      }

      it_pcl->x = static_cast<float>(p_compensate.x());
      it_pcl->y = static_cast<float>(p_compensate.y());
      it_pcl->z = static_cast<float>(p_compensate.z());

      if (it_pcl == pcl_out.points.begin()) {
        break;
      }
    }
  }
  const auto pcl_compensate_end = std::chrono::steady_clock::now();
  const auto total_end = std::chrono::steady_clock::now();

  if (shouldLogTiming(total_end)) {
    RCLCPP_INFO(
      logger_,
      "undistortPcl timing: total=%.3f ms imu_prepare=%.3f ms cloud_copy=%.3f ms "
      "imu_propagate=%.3f ms final_predict=%.3f ms pcl_compensate=%.3f ms "
      "imu_count=%zu point_count=%zu pose_count=%zu",
      toMilliseconds(total_start, total_end),
      toMilliseconds(imu_prepare_start, imu_prepare_end),
      toMilliseconds(cloud_copy_start, cloud_copy_end),
      toMilliseconds(imu_propagate_start, imu_propagate_end),
      toMilliseconds(final_predict_start, final_predict_end),
      toMilliseconds(pcl_compensate_start, pcl_compensate_end),
      v_imu.size(),
      pcl_out.points.size(),
      imu_pose_.size());
  }
}

void ImuProcess::Process(const MeasureGroup & meas, PointCloudXYZI::Ptr & pcl_un)
{
  if (meas.imu.empty() || !meas.lidar) {
    return;
  }

  if (!pcl_un) {
    pcl_un = std::make_shared<PointCloudXYZI>();
  }

  if (imu_need_init_) {
    RCLCPP_INFO(
      logger_,
      "imu_process: imu_need_init=true init_iter_num=%d imu_count=%zu first_lidar_time=%.6f "
      "frame=[%.6f, %.6f]",
      init_iter_num_,
      meas.imu.size(),
      first_lidar_time,
      meas.lidar_beg_time,
      meas.lidar_end_time);

    imuInit(meas, kf_state_, init_iter_num_);
    imu_need_init_ = true;
    last_imu_ = meas.imu.back();

    if (init_iter_num_ > kMaxIniCount) {
      cov_acc_ *= std::pow(kGravity / mean_acc_.norm(), 2);
      imu_need_init_ = false;
      cov_acc_ = cov_acc_scale_;
      cov_gyr_ = cov_gyr_scale_;
      RCLCPP_INFO(
        logger_,
        "imu_process: initialization complete at iter=%d mean_acc_norm=%.6f",
        init_iter_num_,
        mean_acc_.norm());
    }
    return;
  }

  undistortPcl(meas, kf_state_, *pcl_un);
}

}  // namespace pointcloud_undistortion
