#ifndef CROP_BOX_FILTER__CROP_BOX_FILTER_NODE_HPP_
#define CROP_BOX_FILTER__CROP_BOX_FILTER_NODE_HPP_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

namespace crop_box_filter
{

class CropBoxFilterNode : public rclcpp::Node
{
public:
  CropBoxFilterNode();

private:
  struct Bounds
  {
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double min_z;
    double max_z;
  };

  void handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadParameters();
  void updateMarkerPublisher();
  void publishCropBoxMarker(const std_msgs::msg::Header & header);
  rcl_interfaces::msg::SetParametersResult handleParameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters);
  std::optional<Bounds> computeBoundsInCloudFrame(
    const sensor_msgs::msg::PointCloud2 & cloud);
  sensor_msgs::msg::PointCloud2 filterWithPcl(
    const sensor_msgs::msg::PointCloud2 & cloud, const Bounds & bounds) const;
  std::optional<std::size_t> findFieldOffset(
    const sensor_msgs::msg::PointCloud2 & cloud, const std::string & field_name) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string input_topic_;
  std::string output_topic_;
  std::string marker_topic_;
  std::string input_frame_;
  double min_x_{0.0};
  double max_x_{0.0};
  double min_y_{0.0};
  double max_y_{0.0};
  double min_z_{0.0};
  double max_z_{0.0};
  double padding_x_{0.0};
  double padding_y_{0.0};
  double padding_z_{0.0};
  bool negative_{false};
  bool visual_{false};
  bool use_pcl_{false};
};

}  // namespace crop_box_filter

#endif  // CROP_BOX_FILTER__CROP_BOX_FILTER_NODE_HPP_
