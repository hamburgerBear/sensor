#include "crop_box_filter/crop_box_filter_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <limits>
#include <vector>

#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace crop_box_filter
{

CropBoxFilterNode::CropBoxFilterNode()
: Node("crop_box_filter")
{
  loadParameters();
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&CropBoxFilterNode::handleParameterUpdate, this, std::placeholders::_1));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  input_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&CropBoxFilterNode::handlePointCloud, this, std::placeholders::_1));

  output_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_, rclcpp::SensorDataQoS());

  updateMarkerPublisher();

  RCLCPP_INFO(
    get_logger(),
    "crop_box_filter config: input_topic='%s', output_topic='%s', input_frame='%s', visual=%s, use_pcl=%s",
    input_topic_.c_str(), output_topic_.c_str(), input_frame_.c_str(), visual_ ? "true" : "false",
    use_pcl_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "crop_box_filter backend: %s", use_pcl_ ? "pcl" : "native");
}

void CropBoxFilterNode::loadParameters()
{
  input_topic_ = declare_parameter<std::string>("input_topic", "/input/points");
  output_topic_ = declare_parameter<std::string>("output_topic", "/output/points");
  marker_topic_ = declare_parameter<std::string>("marker_topic", "/debug/crop_box_marker");
  input_frame_ = declare_parameter<std::string>("input_frame", "");
  min_x_ = declare_parameter<double>("min_x", -10.0);
  max_x_ = declare_parameter<double>("max_x", 10.0);
  min_y_ = declare_parameter<double>("min_y", -10.0);
  max_y_ = declare_parameter<double>("max_y", 10.0);
  min_z_ = declare_parameter<double>("min_z", -2.0);
  max_z_ = declare_parameter<double>("max_z", 2.0);
  padding_x_ = declare_parameter<double>("padding_x", 0.0);
  padding_y_ = declare_parameter<double>("padding_y", 0.0);
  padding_z_ = declare_parameter<double>("padding_z", 0.0);
  negative_ = declare_parameter<bool>("negative", false);
  visual_ = declare_parameter<bool>("visual", true);
  use_pcl_ = declare_parameter<bool>("use_pcl", true);
}

void CropBoxFilterNode::updateMarkerPublisher()
{
  if (visual_) {
    marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      marker_topic_, rclcpp::QoS(1).transient_local());
  } else {
    if (marker_publisher_) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = input_frame_.empty() ? "map" : input_frame_;
      marker.header.stamp = now();
      marker.ns = "crop_box_filter";
      marker.id = 0;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      marker_publisher_->publish(marker);
    }
    marker_publisher_.reset();
  }
}

rcl_interfaces::msg::SetParametersResult CropBoxFilterNode::handleParameterUpdate(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool recreate_marker_publisher = false;

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();

    if (name == "input_topic" || name == "output_topic") {
      result.successful = false;
      result.reason = "input_topic/output_topic are not dynamically reconfigurable";
      return result;
    }

    if (name == "marker_topic") {
      marker_topic_ = parameter.as_string();
      recreate_marker_publisher = true;
      continue;
    }

    if (name == "input_frame") {
      input_frame_ = parameter.as_string();
      continue;
    }

    if (name == "min_x") {
      min_x_ = parameter.as_double();
      continue;
    }
    if (name == "max_x") {
      max_x_ = parameter.as_double();
      continue;
    }
    if (name == "min_y") {
      min_y_ = parameter.as_double();
      continue;
    }
    if (name == "max_y") {
      max_y_ = parameter.as_double();
      continue;
    }
    if (name == "min_z") {
      min_z_ = parameter.as_double();
      continue;
    }
    if (name == "max_z") {
      max_z_ = parameter.as_double();
      continue;
    }
    if (name == "padding_x") {
      padding_x_ = parameter.as_double();
      continue;
    }
    if (name == "padding_y") {
      padding_y_ = parameter.as_double();
      continue;
    }
    if (name == "padding_z") {
      padding_z_ = parameter.as_double();
      continue;
    }
    if (name == "negative") {
      negative_ = parameter.as_bool();
      continue;
    }
    if (name == "visual") {
      visual_ = parameter.as_bool();
      recreate_marker_publisher = true;
      continue;
    }
    if (name == "use_pcl") {
      use_pcl_ = parameter.as_bool();
      RCLCPP_INFO(get_logger(), "crop_box_filter backend switched to: %s", use_pcl_ ? "pcl" : "native");
      continue;
    }
  }

  if (recreate_marker_publisher) {
    updateMarkerPublisher();
  }

  return result;
}

std::optional<std::size_t> CropBoxFilterNode::findFieldOffset(
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

std::optional<CropBoxFilterNode::Bounds> CropBoxFilterNode::computeBoundsInCloudFrame(
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  const Bounds local_bounds{
    min_x_ - padding_x_,
    max_x_ + padding_x_,
    min_y_ - padding_y_,
    max_y_ + padding_y_,
    min_z_ - padding_z_,
    max_z_ + padding_z_};

  if (input_frame_.empty() || cloud.header.frame_id == input_frame_) {
    return local_bounds;
  }

  try {
    const auto transform_msg = tf_buffer_->lookupTransform(
      cloud.header.frame_id, input_frame_, cloud.header.stamp, tf2::durationFromSec(0.1));
    tf2::Transform transform;
    tf2::fromMsg(transform_msg.transform, transform);

    const std::array<tf2::Vector3, 8> corners = {{
      {local_bounds.min_x, local_bounds.min_y, local_bounds.min_z},
      {local_bounds.min_x, local_bounds.min_y, local_bounds.max_z},
      {local_bounds.min_x, local_bounds.max_y, local_bounds.min_z},
      {local_bounds.min_x, local_bounds.max_y, local_bounds.max_z},
      {local_bounds.max_x, local_bounds.min_y, local_bounds.min_z},
      {local_bounds.max_x, local_bounds.min_y, local_bounds.max_z},
      {local_bounds.max_x, local_bounds.max_y, local_bounds.min_z},
      {local_bounds.max_x, local_bounds.max_y, local_bounds.max_z},
    }};

    Bounds transformed_bounds{
      std::numeric_limits<double>::max(),
      std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::max(),
      std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::max(),
      std::numeric_limits<double>::lowest()};

    for (const auto & corner : corners) {
      const auto transformed_corner = transform * corner;
      transformed_bounds.min_x = std::min(transformed_bounds.min_x, transformed_corner.x());
      transformed_bounds.max_x = std::max(transformed_bounds.max_x, transformed_corner.x());
      transformed_bounds.min_y = std::min(transformed_bounds.min_y, transformed_corner.y());
      transformed_bounds.max_y = std::max(transformed_bounds.max_y, transformed_corner.y());
      transformed_bounds.min_z = std::min(transformed_bounds.min_z, transformed_corner.z());
      transformed_bounds.max_z = std::max(transformed_bounds.max_z, transformed_corner.z());
    }

    return transformed_bounds;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Failed to transform crop box from '%s' to '%s': %s",
      input_frame_.c_str(), cloud.header.frame_id.c_str(), ex.what());
    return std::nullopt;
  }
}

sensor_msgs::msg::PointCloud2 CropBoxFilterNode::filterWithPcl(
  const sensor_msgs::msg::PointCloud2 & cloud, const Bounds & bounds) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud, *input_cloud);

  pcl::CropBox<pcl::PointXYZ> crop_box;
  crop_box.setInputCloud(input_cloud);
  crop_box.setMin(Eigen::Vector4f(bounds.min_x, bounds.min_y, bounds.min_z, 1.0F));
  crop_box.setMax(Eigen::Vector4f(bounds.max_x, bounds.max_y, bounds.max_z, 1.0F));
  crop_box.setNegative(negative_);

  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  crop_box.filter(filtered_cloud);

  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(filtered_cloud, output);
  output.header = cloud.header;
  output.is_dense = filtered_cloud.is_dense;
  return output;
}

void CropBoxFilterNode::publishCropBoxMarker(const std_msgs::msg::Header & header)
{
  if (!visual_ || !marker_publisher_) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header = header;
  if (!input_frame_.empty()) {
    marker.header.frame_id = input_frame_;
  }
  marker.ns = "crop_box_filter";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;

  const double effective_min_x = min_x_ - padding_x_;
  const double effective_max_x = max_x_ + padding_x_;
  const double effective_min_y = min_y_ - padding_y_;
  const double effective_max_y = max_y_ + padding_y_;
  const double effective_min_z = min_z_ - padding_z_;
  const double effective_max_z = max_z_ + padding_z_;

  marker.pose.position.x = (effective_min_x + effective_max_x) * 0.5;
  marker.pose.position.y = (effective_min_y + effective_max_y) * 0.5;
  marker.pose.position.z = (effective_min_z + effective_max_z) * 0.5;
  marker.scale.x = std::max(effective_max_x - effective_min_x, 0.001);
  marker.scale.y = std::max(effective_max_y - effective_min_y, 0.001);
  marker.scale.z = std::max(effective_max_z - effective_min_z, 0.001);
  marker.color.r = negative_ ? 1.0F : 0.1F;
  marker.color.g = negative_ ? 0.2F : 0.8F;
  marker.color.b = 0.1F;
  marker.color.a = 0.2F;

  marker_publisher_->publish(marker);
}

void CropBoxFilterNode::handlePointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto start_time = std::chrono::steady_clock::now();
  const auto bounds = computeBoundsInCloudFrame(*msg);
  if (!bounds) {
    return;
  }
  const auto & input_cloud = *msg;

  const auto x_offset = findFieldOffset(input_cloud, "x");
  const auto y_offset = findFieldOffset(input_cloud, "y");
  const auto z_offset = findFieldOffset(input_cloud, "z");
  if (!x_offset || !y_offset || !z_offset) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Input point cloud is missing FLOAT32 x/y/z fields required by crop_box_filter.");
    return;
  }

  const std::size_t point_count = static_cast<std::size_t>(input_cloud.width) * input_cloud.height;
  sensor_msgs::msg::PointCloud2 output;
  if (use_pcl_) {
    output = filterWithPcl(input_cloud, *bounds);
  } else {
    output = input_cloud;
    output.data.clear();
    output.data.reserve(input_cloud.data.size());

    std::vector<std::uint8_t> filtered_data;
    filtered_data.reserve(input_cloud.data.size());

    for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
      const std::size_t base_offset = point_index * input_cloud.point_step;

      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      std::memcpy(&x, input_cloud.data.data() + base_offset + *x_offset, sizeof(float));
      std::memcpy(&y, input_cloud.data.data() + base_offset + *y_offset, sizeof(float));
      std::memcpy(&z, input_cloud.data.data() + base_offset + *z_offset, sizeof(float));

      const bool inside_box =
        x >= bounds->min_x && x <= bounds->max_x &&
        y >= bounds->min_y && y <= bounds->max_y &&
        z >= bounds->min_z && z <= bounds->max_z;
      const bool keep_point = negative_ ? !inside_box : inside_box;

      if (keep_point) {
        const auto point_begin = input_cloud.data.begin() + static_cast<std::ptrdiff_t>(base_offset);
        filtered_data.insert(
          filtered_data.end(),
          point_begin,
          point_begin + static_cast<std::ptrdiff_t>(input_cloud.point_step));
      }
    }

    output.data = std::move(filtered_data);
    output.width = output.point_step == 0 ? 0U : static_cast<std::uint32_t>(output.data.size() / output.point_step);
    output.height = 1;
    output.row_step = output.width * output.point_step;
    output.is_dense = false;
  }

  if (visual_) {
    output_publisher_->publish(output);
    publishCropBoxMarker(output.header);
  }

  const auto end_time = std::chrono::steady_clock::now();
  const auto elapsed_us =
    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "in=%u.%09u out=%u.%09u elapsed=%.3fms points_in=%zu points_out=%u",
    msg->header.stamp.sec,
    msg->header.stamp.nanosec,
    output.header.stamp.sec,
    output.header.stamp.nanosec,
    static_cast<double>(elapsed_us) / 1000.0,
    point_count,
    output.width);
}

}  // namespace crop_box_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<crop_box_filter::CropBoxFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
