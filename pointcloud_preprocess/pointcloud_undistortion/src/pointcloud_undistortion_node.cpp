#include "pointcloud_undistortion/imu_process.hpp"
#include "pointcloud_undistortion/pointcloud_undistortion_node.hpp"

#include <chrono>
#include <utility>

#include <pcl_conversions/pcl_conversions.h>

namespace pointcloud_undistortion
{

namespace
{

Eigen::Vector3d vectorFromStdVector(const std::vector<double> & values)
{
  Eigen::Vector3d vector = Eigen::Vector3d::Zero();
  if (values.size() == 3U) {
    vector << values[0], values[1], values[2];
  }
  return vector;
}

Eigen::Matrix3d matrixFromStdVector(const std::vector<double> & values)
{
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
  if (values.size() == 9U) {
    matrix <<
      values[0], values[1], values[2],
      values[3], values[4], values[5],
      values[6], values[7], values[8];
  }
  return matrix;
}

}  // namespace

PointcloudUndistortionNode::PointcloudUndistortionNode()
: Node("pointcloud_undistortion")
{
  loadParameters();
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&PointcloudUndistortionNode::handleParameterUpdate, this, std::placeholders::_1));

  input_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PointcloudUndistortionNode::handlePointCloud, this, std::placeholders::_1));

  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PointcloudUndistortionNode::handleImu, this, std::placeholders::_1));

  output_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_, rclcpp::SensorDataQoS());

  RCLCPP_INFO(
    get_logger(),
    "pointcloud_undistortion config: input_topic='%s', output_topic='%s', imu_topic='%s', "
    "output_frame='%s', scan_line=%d, scan_rate=%d, point_filter_num=%d, blind_threshold=%.3f, "
    "distance_threshold=%.3f, extrinsic_t=[%.3f, %.3f, %.3f], use_imu=%s",
    input_topic_.c_str(), output_topic_.c_str(), imu_topic_.c_str(), output_frame_.c_str(),
    scan_line_, scan_rate_, point_filter_num_, blind_threshold_, distance_threshold_,
    extrinsic_t_[0], extrinsic_t_[1], extrinsic_t_[2],
    use_imu_ ? "true" : "false");

  imu_process_ = std::make_unique<ImuProcess>();
  imu_process_->setLogger(get_logger());
  preprocess_.setLogger(get_logger());
  imu_process_->setExtrinsic(
    vectorFromStdVector(extrinsic_t_), matrixFromStdVector(extrinsic_r_));
  processing_thread_ = std::thread(&PointcloudUndistortionNode::processingLoop, this);
}

PointcloudUndistortionNode::~PointcloudUndistortionNode()
{
  running_ = false;

  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
}

void PointcloudUndistortionNode::loadParameters()
{
  input_topic_ = declare_parameter<std::string>("input_topic", "/input/points");
  output_topic_ = declare_parameter<std::string>("output_topic", "/sensor/undistorted_points");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/input/imu");
  output_frame_ = declare_parameter<std::string>("output_frame", "");
  extrinsic_t_ = declare_parameter<std::vector<double>>(
    "extrinsic_t", std::vector<double>{0.0, 0.0, 0.0});
  extrinsic_r_ = declare_parameter<std::vector<double>>(
    "extrinsic_r", std::vector<double>{0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0});
  scan_line_ = declare_parameter<int>("scan_line", 32);
  scan_rate_ = declare_parameter<int>("scan_rate", 10);
  point_filter_num_ = declare_parameter<int>("point_filter_num", 1);
  blind_threshold_ = declare_parameter<double>("blind_threshold", 3.0);
  distance_threshold_ = declare_parameter<double>("distance_threshold", 0.0);
  use_imu_ = declare_parameter<bool>("use_imu", true);

  preprocess_.setScanLine(scan_line_);
  preprocess_.setScanRate(scan_rate_);
  preprocess_.setPointFilterNum(point_filter_num_);
  preprocess_.setBlind(blind_threshold_);
  preprocess_.setDistanceThreshold(distance_threshold_);
  preprocess_.setLogger(get_logger());
}

rcl_interfaces::msg::SetParametersResult PointcloudUndistortionNode::handleParameterUpdate(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();

    if (name == "input_topic" || name == "output_topic" || name == "imu_topic") {
      result.successful = false;
      result.reason = "input_topic/output_topic/imu_topic are not dynamically reconfigurable";
      return result;
    }

    if (name == "output_frame") {
      output_frame_ = parameter.as_string();
      continue;
    }
    if (name == "extrinsic_t") {
      extrinsic_t_ = parameter.as_double_array();
      imu_process_->setExtrinsic(
        vectorFromStdVector(extrinsic_t_), matrixFromStdVector(extrinsic_r_));
      continue;
    }
    if (name == "extrinsic_r") {
      extrinsic_r_ = parameter.as_double_array();
      imu_process_->setExtrinsic(
        vectorFromStdVector(extrinsic_t_), matrixFromStdVector(extrinsic_r_));
      continue;
    }
    if (name == "scan_line") {
      scan_line_ = parameter.as_int();
      preprocess_.setScanLine(scan_line_);
      continue;
    }
    if (name == "scan_rate") {
      scan_rate_ = parameter.as_int();
      preprocess_.setScanRate(scan_rate_);
      continue;
    }
    if (name == "point_filter_num") {
      point_filter_num_ = parameter.as_int();
      preprocess_.setPointFilterNum(point_filter_num_);
      continue;
    }
    if (name == "blind_threshold") {
      blind_threshold_ = parameter.as_double();
      preprocess_.setBlind(blind_threshold_);
      continue;
    }
    if (name == "distance_threshold") {
      distance_threshold_ = parameter.as_double();
      preprocess_.setDistanceThreshold(distance_threshold_);
      continue;
    }
    if (name == "use_imu") {
      use_imu_ = parameter.as_bool();
      continue;
    }
  }

  return result;
}

void PointcloudUndistortionNode::handleImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    const double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    if (timestamp < last_timestamp_imu_) {
      RCLCPP_WARN(get_logger(), "imu loop back, clear buffer");
      imu_buffer_.clear();
    }
    last_timestamp_imu_ = timestamp;
    imu_buffer_.push_back(msg);
  }

  latest_imu_stamp_ = rclcpp::Time(msg->header.stamp);
  ++imu_count_;
}

bool PointcloudUndistortionNode::syncPackages(MeasureGroup & meas)
{
  const auto start_time = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lidar_lock(lidar_mutex_);
  std::lock_guard<std::mutex> imu_lock(imu_mutex_);

  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }

  if (!lidar_pushed_) {
    meas.lidar = lidar_buffer_.front();
    meas.lidar_beg_time = time_buffer_.front();
    meas.lidar_frame_id = frame_id_buffer_.front();

    if (meas.lidar->points.size() <= 5U) {
      lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
      RCLCPP_WARN(get_logger(), "Too few input point cloud.");
    } else if (
      static_cast<double>(meas.lidar->points.back().curvature) / 1000.0 <
      0.5 * lidar_mean_scantime_)
    {
      lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
    } else {
      ++scan_num_;
      lidar_end_time_ =
        meas.lidar_beg_time + static_cast<double>(meas.lidar->points.back().curvature) / 1000.0;
      lidar_mean_scantime_ +=
        ((static_cast<double>(meas.lidar->points.back().curvature) / 1000.0) -
        lidar_mean_scantime_) /
        static_cast<double>(scan_num_);
    }

    meas.lidar_end_time = lidar_end_time_;
    lidar_pushed_ = true;
  }

  if (last_timestamp_imu_ < lidar_end_time_) {
    return false;
  }

  meas.imu.clear();
  while (!imu_buffer_.empty()) {
    const double imu_time = rclcpp::Time(imu_buffer_.front()->header.stamp).seconds();
    if (imu_time > lidar_end_time_) {
      break;
    }
    meas.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  lidar_buffer_.pop_front();
  frame_id_buffer_.pop_front();
  time_buffer_.pop_front();
  lidar_pushed_ = false;

  const auto end_time = std::chrono::steady_clock::now();
  const auto elapsed_us =
    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "sync_packages elapsed=%.3fms",
    static_cast<double>(elapsed_us) / 1000.0);
  return true;
}

void PointcloudUndistortionNode::processingLoop()
{
  while (rclcpp::ok() && running_) {
    const auto loop_start_time = std::chrono::steady_clock::now();
    MeasureGroup meas;
    if (!syncPackages(meas)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    const bool had_last_imu = static_cast<bool>(last_imu_);
    auto v_imu = meas.imu;
    if (had_last_imu) {
      v_imu.push_front(last_imu_);
    }

    if (!meas.imu.empty()) {
      last_imu_ = meas.imu.back();
    }
    last_lidar_end_time_ = meas.lidar_end_time;

    if (flg_first_scan_) {
      first_lidar_time_ = meas.lidar_beg_time;
      imu_process_->first_lidar_time = first_lidar_time_;
      flg_first_scan_ = false;

      RCLCPP_INFO(
        get_logger(),
        "first lidar scan captured: first_lidar_time=%.6f, skip undistortion for initialization",
        first_lidar_time_);
      continue;
    }

    const auto imu_process_start_time = std::chrono::steady_clock::now();
    imu_process_->Process(meas, feats_undistort_);
    const auto imu_process_end_time = std::chrono::steady_clock::now();
    const auto imu_process_elapsed_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
      imu_process_end_time - imu_process_start_time).count();

    const auto rosmsg_start_time = std::chrono::steady_clock::now();
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*feats_undistort_, output_msg);
    output_msg.header.stamp = rclcpp::Time(meas.lidar_end_time * 1.0e9);
    output_msg.header.frame_id = output_frame_.empty() ? meas.lidar_frame_id : output_frame_;
    output_publisher_->publish(output_msg);
    const auto rosmsg_end_time = std::chrono::steady_clock::now();
    const auto rosmsg_elapsed_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
      rosmsg_end_time - rosmsg_start_time).count();
    const auto loop_end_time = std::chrono::steady_clock::now();
    const auto loop_elapsed_us =
      std::chrono::duration_cast<std::chrono::microseconds>(loop_end_time - loop_start_time).count();

    const double meas_imu_front =
      meas.imu.empty() ? -1.0 : rclcpp::Time(meas.imu.front()->header.stamp).seconds();
    const double meas_imu_back =
      meas.imu.empty() ? -1.0 : rclcpp::Time(meas.imu.back()->header.stamp).seconds();
    const double v_imu_front =
      v_imu.empty() ? -1.0 : rclcpp::Time(v_imu.front()->header.stamp).seconds();
    const double v_imu_back =
      v_imu.empty() ? -1.0 : rclcpp::Time(v_imu.back()->header.stamp).seconds();

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "synced measure group: lidar=[%.6f, %.6f] meas_imu=[%.6f, %.6f] v_imu=[%.6f, %.6f] "
      "imu_count=%zu prepended=%s points=%zu",
      meas.lidar_beg_time,
      meas.lidar_end_time,
      meas_imu_front,
      meas_imu_back,
      v_imu_front,
      v_imu_back,
      v_imu.size(),
      had_last_imu ? "true" : "false",
      feats_undistort_->points.size());
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "processing timings: imu_process=%.3fms rosmsg_publish=%.3fms loop_total=%.3fms",
      static_cast<double>(imu_process_elapsed_us) / 1000.0,
      static_cast<double>(rosmsg_elapsed_us) / 1000.0,
      static_cast<double>(loop_elapsed_us) / 1000.0);
  }
}

void PointcloudUndistortionNode::handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto start_time = std::chrono::steady_clock::now();

  auto pcl_out = std::make_shared<PointCloudXYZI>();
  preprocess_.process(msg, pcl_out);
  if (pcl_out->empty()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    lidar_buffer_.push_back(pcl_out);
    frame_id_buffer_.push_back(msg->header.frame_id);
    time_buffer_.push_back(rclcpp::Time(msg->header.stamp).seconds());
  }

  const auto end_time = std::chrono::steady_clock::now();
  const auto elapsed_us =
    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  const bool imu_ready = !use_imu_ || latest_imu_stamp_.has_value();
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "use_imu=%s imu_ready=%s imu_count=%llu points=%zu elapsed=%.3fms",
    use_imu_ ? "true" : "false",
    imu_ready ? "true" : "false",
    static_cast<unsigned long long>(imu_count_),
    pcl_out->points.size(),
    static_cast<double>(elapsed_us) / 1000.0);
}

}  // namespace pointcloud_undistortion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pointcloud_undistortion::PointcloudUndistortionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
