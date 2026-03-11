#ifndef PASSTHROUGH_FILTER__PASSTHROUGH_FILTER_NODE_HPP_
#define PASSTHROUGH_FILTER__PASSTHROUGH_FILTER_NODE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace passthrough_filter
{

class PassthroughFilterNode : public rclcpp::Node
{
public:
  PassthroughFilterNode();

private:
  struct FieldInfo
  {
    std::size_t offset;
    std::uint8_t datatype;
  };

  struct CoordinateOffsets
  {
    std::size_t x;
    std::size_t y;
    std::size_t z;
  };

  void loadParameters();
  void handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult handleParameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters);
  sensor_msgs::msg::PointCloud2 filterWithPcl(
    const sensor_msgs::msg::PointCloud2 & cloud) const;
  std::optional<FieldInfo> findFieldInfo(
    const sensor_msgs::msg::PointCloud2 & cloud, const std::string & field_name) const;
  std::optional<std::size_t> findFloat32FieldOffset(
    const sensor_msgs::msg::PointCloud2 & cloud, const std::string & field_name) const;
  std::optional<double> readFieldValue(
    const sensor_msgs::msg::PointCloud2 & cloud, std::size_t base_offset,
    const FieldInfo & field_info) const;
  std::optional<double> readFilterValue(
    const sensor_msgs::msg::PointCloud2 & cloud, std::size_t base_offset,
    const FieldInfo & field_info, const tf2::Transform * transform,
    const CoordinateOffsets * coordinate_offsets) const;
  bool shouldUseTransform(const sensor_msgs::msg::PointCloud2 & cloud) const;
  bool isCoordinateField() const;
  bool shouldKeepPoint(double value) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_publisher_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string input_topic_;
  std::string output_topic_;
  std::string input_frame_;
  std::string filter_field_name_;
  double min_limit_{0.0};
  double max_limit_{0.0};
  bool negative_{false};
  bool use_pcl_{false};
};

}  // namespace passthrough_filter

#endif  // PASSTHROUGH_FILTER__PASSTHROUGH_FILTER_NODE_HPP_
