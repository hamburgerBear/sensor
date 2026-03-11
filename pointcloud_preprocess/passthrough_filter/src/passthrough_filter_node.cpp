#include "passthrough_filter/passthrough_filter_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <vector>

#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace passthrough_filter
{

PassthroughFilterNode::PassthroughFilterNode()
: Node("passthrough_filter")
{
  loadParameters();
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&PassthroughFilterNode::handleParameterUpdate, this, std::placeholders::_1));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  input_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PassthroughFilterNode::handlePointCloud, this, std::placeholders::_1));

  output_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_, rclcpp::SensorDataQoS());

  RCLCPP_INFO(
    get_logger(),
    "passthrough_filter config: input_topic='%s', output_topic='%s', input_frame='%s', field='%s', limits=[%.3f, %.3f], negative=%s, use_pcl=%s",
    input_topic_.c_str(), output_topic_.c_str(), input_frame_.c_str(), filter_field_name_.c_str(),
    min_limit_, max_limit_, negative_ ? "true" : "false", use_pcl_ ? "true" : "false");
}

void PassthroughFilterNode::loadParameters()
{
  input_topic_ = declare_parameter<std::string>("input_topic", "/input/points");
  output_topic_ = declare_parameter<std::string>("output_topic", "/output/points");
  input_frame_ = declare_parameter<std::string>("input_frame", "");
  filter_field_name_ = declare_parameter<std::string>("filter_field_name", "z");
  min_limit_ = declare_parameter<double>("min_limit", -2.0);
  max_limit_ = declare_parameter<double>("max_limit", 2.0);
  negative_ = declare_parameter<bool>("negative", false);
  use_pcl_ = declare_parameter<bool>("use_pcl", true);
}

rcl_interfaces::msg::SetParametersResult PassthroughFilterNode::handleParameterUpdate(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();

    if (name == "input_topic" || name == "output_topic") {
      result.successful = false;
      result.reason = "input_topic/output_topic are not dynamically reconfigurable";
      return result;
    }

    if (name == "filter_field_name") {
      filter_field_name_ = parameter.as_string();
      continue;
    }
    if (name == "input_frame") {
      input_frame_ = parameter.as_string();
      continue;
    }
    if (name == "min_limit") {
      min_limit_ = parameter.as_double();
      continue;
    }
    if (name == "max_limit") {
      max_limit_ = parameter.as_double();
      continue;
    }
    if (name == "negative") {
      negative_ = parameter.as_bool();
      continue;
    }
    if (name == "use_pcl") {
      use_pcl_ = parameter.as_bool();
      RCLCPP_INFO(get_logger(), "passthrough_filter backend switched to: %s", use_pcl_ ? "pcl" : "native");
      continue;
    }
  }

  return result;
}

std::optional<PassthroughFilterNode::FieldInfo> PassthroughFilterNode::findFieldInfo(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & field_name) const
{
  const auto field_it = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField & field) {
      return field.name == field_name;
    });

  if (field_it == cloud.fields.end()) {
    return std::nullopt;
  }

  return FieldInfo{
    static_cast<std::size_t>(field_it->offset),
    field_it->datatype};
}

std::optional<std::size_t> PassthroughFilterNode::findFloat32FieldOffset(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & field_name) const
{
  const auto field_it = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField & field) {
      return field.name == field_name && field.datatype == sensor_msgs::msg::PointField::FLOAT32;
    });

  if (field_it == cloud.fields.end()) {
    return std::nullopt;
  }

  return static_cast<std::size_t>(field_it->offset);
}

std::optional<double> PassthroughFilterNode::readFieldValue(
  const sensor_msgs::msg::PointCloud2 & cloud, std::size_t base_offset,
  const FieldInfo & field_info) const
{
  const auto * data = cloud.data.data() + base_offset + field_info.offset;

  switch (field_info.datatype) {
    case sensor_msgs::msg::PointField::FLOAT32: {
      float value = 0.0F;
      std::memcpy(&value, data, sizeof(float));
      return static_cast<double>(value);
    }
    case sensor_msgs::msg::PointField::FLOAT64: {
      double value = 0.0;
      std::memcpy(&value, data, sizeof(double));
      return value;
    }
    default:
      return std::nullopt;
  }
}

bool PassthroughFilterNode::isCoordinateField() const
{
  return filter_field_name_ == "x" || filter_field_name_ == "y" || filter_field_name_ == "z";
}

bool PassthroughFilterNode::shouldUseTransform(const sensor_msgs::msg::PointCloud2 & cloud) const
{
  return !input_frame_.empty() && cloud.header.frame_id != input_frame_;
}

std::optional<double> PassthroughFilterNode::readFilterValue(
  const sensor_msgs::msg::PointCloud2 & cloud, std::size_t base_offset,
  const FieldInfo & field_info, const tf2::Transform * transform,
  const CoordinateOffsets * coordinate_offsets) const
{
  if (transform == nullptr) {
    return readFieldValue(cloud, base_offset, field_info);
  }

  if (coordinate_offsets == nullptr) {
    return std::nullopt;
  }

  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  std::memcpy(&x, cloud.data.data() + base_offset + coordinate_offsets->x, sizeof(float));
  std::memcpy(&y, cloud.data.data() + base_offset + coordinate_offsets->y, sizeof(float));
  std::memcpy(&z, cloud.data.data() + base_offset + coordinate_offsets->z, sizeof(float));

  const auto transformed_point = (*transform) * tf2::Vector3(x, y, z);
  if (filter_field_name_ == "x") {
    return transformed_point.x();
  }
  if (filter_field_name_ == "y") {
    return transformed_point.y();
  }
  return transformed_point.z();
}

bool PassthroughFilterNode::shouldKeepPoint(double value) const
{
  const bool within_limits = value >= min_limit_ && value <= max_limit_;
  return negative_ ? !within_limits : within_limits;
}

sensor_msgs::msg::PointCloud2 PassthroughFilterNode::filterWithPcl(
  const sensor_msgs::msg::PointCloud2 & cloud) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud, *input_cloud);

  pcl::PassThrough<pcl::PointXYZ> pass_through;
  pass_through.setInputCloud(input_cloud);
  pass_through.setFilterFieldName(filter_field_name_);
  pass_through.setFilterLimits(static_cast<float>(min_limit_), static_cast<float>(max_limit_));
  pass_through.setNegative(negative_);

  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  pass_through.filter(filtered_cloud);

  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(filtered_cloud, output);
  output.header = cloud.header;
  output.is_dense = filtered_cloud.is_dense;
  return output;
}

void PassthroughFilterNode::handlePointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto start_time = std::chrono::steady_clock::now();
  const auto & input_cloud = *msg;
  const std::size_t point_count = static_cast<std::size_t>(input_cloud.width) * input_cloud.height;
  const bool use_transform = shouldUseTransform(input_cloud);
  std::optional<tf2::Transform> transform;
  std::optional<CoordinateOffsets> coordinate_offsets;

  sensor_msgs::msg::PointCloud2 output;
  if (use_pcl_ && !use_transform) {
    output = filterWithPcl(input_cloud);
  } else {
    const auto field_info = findFieldInfo(input_cloud, filter_field_name_);
    if (!field_info) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input point cloud is missing field '%s' required by passthrough_filter.",
        filter_field_name_.c_str());
      return;
    }

    if (use_transform && !isCoordinateField()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "input_frame filtering only supports x/y/z fields. Current field='%s'.",
        filter_field_name_.c_str());
      return;
    }

    if (use_transform) {
      const auto x_offset = findFloat32FieldOffset(input_cloud, "x");
      const auto y_offset = findFloat32FieldOffset(input_cloud, "y");
      const auto z_offset = findFloat32FieldOffset(input_cloud, "z");
      if (!x_offset || !y_offset || !z_offset) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "input_frame filtering requires FLOAT32 x/y/z fields in the input cloud.");
        return;
      }

      coordinate_offsets = CoordinateOffsets{*x_offset, *y_offset, *z_offset};

      try {
        const auto transform_msg = tf_buffer_->lookupTransform(
          input_frame_, input_cloud.header.frame_id, input_cloud.header.stamp, tf2::durationFromSec(0.1));
        tf2::Transform tf_transform;
        tf2::fromMsg(transform_msg.transform, tf_transform);
        transform = tf_transform;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Failed to transform points from '%s' to '%s': %s",
          input_cloud.header.frame_id.c_str(), input_frame_.c_str(), ex.what());
        return;
      }
    }

    output = input_cloud;
    std::vector<std::uint8_t> filtered_data;
    filtered_data.reserve(input_cloud.data.size());

    for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
      const std::size_t base_offset = point_index * input_cloud.point_step;
      const auto value = readFilterValue(
        input_cloud, base_offset, *field_info, transform ? &(*transform) : nullptr,
        coordinate_offsets ? &(*coordinate_offsets) : nullptr);
      if (!value) {
        if (!use_transform) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Field '%s' is not FLOAT32/FLOAT64, native passthrough_filter cannot process it.",
            filter_field_name_.c_str());
        }
        return;
      }

      if (!shouldKeepPoint(*value)) {
        continue;
      }

      const auto output_offset = filtered_data.size();
      filtered_data.resize(output_offset + input_cloud.point_step);
      std::memcpy(
        filtered_data.data() + output_offset,
        input_cloud.data.data() + base_offset,
        input_cloud.point_step);
    }

    output.data = std::move(filtered_data);
    output.width = output.point_step == 0 ? 0U : static_cast<std::uint32_t>(output.data.size() / output.point_step);
    output.height = 1;
    output.row_step = output.width * output.point_step;
    output.is_dense = false;
  }

  output_publisher_->publish(output);

  const auto end_time = std::chrono::steady_clock::now();
  const auto elapsed_us =
    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "frame=%s field=%s limits=[%.3f, %.3f] in=%zu out=%u elapsed=%.3fms",
    input_frame_.empty() ? input_cloud.header.frame_id.c_str() : input_frame_.c_str(),
    filter_field_name_.c_str(),
    min_limit_,
    max_limit_,
    point_count,
    output.width,
    static_cast<double>(elapsed_us) / 1000.0);
}

}  // namespace passthrough_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<passthrough_filter::PassthroughFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
