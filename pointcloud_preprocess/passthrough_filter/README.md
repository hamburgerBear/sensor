# Passthrough Filter

`passthrough_filter` is a ROS 2 Humble `ament_cmake` package for point cloud single-field range
filtering.

It is intended for point cloud preprocessing in the `north` workspace and currently provides:

- pass-through filtering for `sensor_msgs/msg/PointCloud2`
- optional PCL backend when filtering in the input cloud frame
- optional RViz visualization
- TF-aware filtering when `input_frame` differs from the cloud frame
- runtime parameter updates for most filter settings

## Current Behavior

- input topic default: `/input/points`
- output topic default: `/sensor/passthrough_filtered`
- `filter_field_name=z` by default
- `min_limit=-2.0` by default
- `max_limit=2.0` by default
- `negative=false` by default
- `use_pcl=true` by default

When `input_frame` is empty, filtering is evaluated directly in the input cloud frame.

When `input_frame` differs from the incoming point cloud frame, the current implementation:

- looks up the transform from the cloud frame to `input_frame`
- evaluates each point in the target frame
- filters only by `x`, `y`, or `z` in that mode

This keeps the original point data unchanged while applying the filtering condition in the target
frame.

## Parameters

Static at startup:

- `input_topic`
- `output_topic`

Dynamic at runtime:

- `input_frame`
- `filter_field_name`
- `min_limit`
- `max_limit`
- `negative`
- `use_pcl`

Parameter notes:

- `filter_field_name=x|y|z`: typical geometry filtering usage
- `filter_field_name=<other field>`: supported only when filtering in the input cloud frame
- `negative=true`: keep points outside the configured range
- `use_pcl=true`: use the PCL `PassThrough` backend when no target-frame transform is required
- `use_pcl=false`: use the native filtering backend

## Build

```bash
cd ~/workspace/north
source /opt/ros/humble/setup.bash
colcon build --packages-select passthrough_filter
```

## Launch

```bash
cd ~/workspace/north
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch passthrough_filter passthrough_filter.launch.py
```

The launch file starts:

- `passthrough_filter`
- `rviz2` when `use_rviz:=true`

Supported launch arguments:

- `input_topic`
- `output_topic`
- `input_frame`
- `filter_field_name`
- `min_limit`
- `max_limit`
- `negative`
- `use_pcl`
- `use_rviz`

Example:

```bash
ros2 launch passthrough_filter passthrough_filter.launch.py \
  input_topic:=/rslidar_points_front \
  input_frame:=base_link \
  filter_field_name:=z \
  min_limit:=-2.0 \
  max_limit:=2.0 \
  use_pcl:=true
```

## Runtime Logs

Startup logs include:

- current topic configuration
- selected filter field and range
- selected backend: `pcl` or `native`

Runtime logs are throttled to once every 2 seconds and include:

- active filter frame
- active filter field
- elapsed filter time
- input point count
- output point count

## Visualization

The packaged RViz config shows:

- input point cloud: `/input/points`
- filtered point cloud: `/sensor/passthrough_filtered`

If you change `input_topic` or `output_topic` in launch arguments, update the RViz display topics
accordingly or use the RViz topic selector manually.

## Limitations

- only one field can be filtered at a time
- `input_frame` mode currently supports only `x`, `y`, and `z`
- `input_topic` and `output_topic` are not dynamically reconfigurable
- when `input_frame` is used, the current implementation falls back to native per-point processing
- if you need simultaneous constraints on `x`, `y`, and `z`, use `crop_box_filter` instead

## Next Stage Todo

1. Benchmark `pcl` and `native` backends with recorded bags and document latency under both plain-frame and `input_frame` modes.
2. Add a whole-cloud transform path for `input_frame` mode and compare it with the current per-point evaluation path.
3. Verify whether the PCL path preserves all required fields such as `intensity`, `ring`, and `time`; if not, add a field-preserving implementation.
4. Add unit tests and rosbag-based regression tests for `x/y/z` filtering, `negative` mode, dynamic parameters, and TF-based frame filtering.
5. Add optional QoS configuration to make RViz and downstream consumers easier to integrate.
6. Consider separating filtering-frame evaluation mode from output-frame behavior if downstream modules need transformed output clouds.
7. Add launch examples or chained launch support for multi-axis filtering with multiple `passthrough_filter` nodes.
