# Crop Box Filter

`crop_box_filter` is a ROS 2 Humble `ament_cmake` package for point cloud ROI filtering.

It is intended for point cloud preprocessing in the `north` workspace and currently provides:

- crop box filtering for `sensor_msgs/msg/PointCloud2`
- optional PCL backend
- optional RViz visualization
- TF-aware crop-box handling when `input_frame` differs from the cloud frame
- runtime parameter updates for most crop settings

## Current Behavior

- input topic default: `/input/points`
- output topic default: `/sensor/crop_box_filted`
- marker topic default: `/debug/crop_box_marker`
- `visual=true` by default
- `use_pcl=true` by default

When `input_frame` is different from the incoming point cloud frame, the current implementation uses an approximate `2B` strategy:

- transform the crop box corners from `input_frame` into the cloud frame
- build the enclosing AABB in the cloud frame
- filter the cloud against that AABB

This is faster than transforming the full point cloud, but it is an approximation rather than a strict oriented-box test.

## Parameters

Static at startup:

- `input_topic`
- `output_topic`

Dynamic at runtime:

- `marker_topic`
- `input_frame`
- `min_x`, `max_x`
- `min_y`, `max_y`
- `min_z`, `max_z`
- `padding_x`, `padding_y`, `padding_z`
- `negative`
- `visual`
- `use_pcl`

Parameter notes:

- `visual=true`: publish filtered cloud and crop box marker
- `visual=false`: publish neither filtered cloud nor marker
- `use_pcl=true`: use the PCL `CropBox` backend
- `use_pcl=false`: use the native filtering backend

## Build

```bash
cd ~/workspace/north
source /opt/ros/humble/setup.bash
colcon build --packages-select crop_box_filter
```

## Launch

```bash
cd ~/workspace/north
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch crop_box_filter crop_box_filter.launch.py
```

The launch file starts:

- `crop_box_filter`
- `rviz2` when `use_rviz:=true`

Supported launch arguments:

- `input_topic`
- `output_topic`
- `input_frame`
- `marker_topic`
- `visual`
- `use_pcl`
- `use_rviz`

Example:

```bash
ros2 launch crop_box_filter crop_box_filter.launch.py \
  input_topic:=/rslidar_points_front \
  input_frame:=base_link \
  use_pcl:=true
```

## Runtime Logs

Startup logs include:

- current topic configuration
- selected backend: `pcl` or `native`

Runtime logs are throttled to once every 2 seconds and include:

- input timestamp
- output timestamp
- elapsed filter time
- input point count
- output point count

## Visualization

The packaged RViz config shows:

- filtered point cloud: `/sensor/crop_box_filted`
- crop box marker: `/debug/crop_box_marker`

If `visual` is switched to `false`, the node:

- stops publishing the filtered cloud
- stops publishing the marker
- sends a marker delete message to clear the crop box in RViz

## Limitations

- the approximate `2B` mode may keep extra points compared with a strict oriented crop box
- `input_topic` and `output_topic` are not dynamically reconfigurable
- the current PCL path is intended primarily for performance comparison and fast operation

## Next Stage Todo

1. Add a strict oriented-box mode to replace the current approximate AABB fallback when higher geometric accuracy is required.
2. Benchmark `native` and `pcl` backends with recorded bags and document latency, point retention, and field preservation.
3. Verify whether the PCL path preserves all required fields such as `intensity`, `ring`, and `time`; if not, add a field-preserving implementation.
4. Optimize the native backend with a preallocated write path instead of repeated byte-vector insertion.
5. Add unit tests and rosbag-based regression tests for frame transforms, `negative` mode, dynamic parameters, and `visual` gating.
6. Add optional QoS configuration to make RViz and downstream consumers easier to integrate.
7. Add a dedicated parameter or mode name to separate visualization control from output publishing if those behaviors need to diverge later.
