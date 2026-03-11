# Passthrough Filter

ROS 2 Humble point cloud passthrough filter package.

## Parameters

- `input_topic`: input `sensor_msgs/msg/PointCloud2` topic
- `output_topic`: output filtered point cloud topic
- `input_frame`: optional filter reference frame for `x/y/z` filtering
- `filter_field_name`: filter axis or field name, typically `x`, `y`, or `z`
- `min_limit`: lower pass-through limit
- `max_limit`: upper pass-through limit
- `negative`: invert the filter result
- `use_pcl`: use PCL `PassThrough` backend or the native fallback

When `input_frame` is set and differs from the point cloud frame, filtering is evaluated in that
target frame. This mode supports `x`, `y`, and `z` only.
