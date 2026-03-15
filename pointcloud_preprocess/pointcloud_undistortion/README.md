# Pointcloud Undistortion

`pointcloud_undistortion` is a ROS 2 Humble `ament_cmake` package for RS-LiDAR style point cloud preprocessing and IMU-assisted motion undistortion.

## What It Does

The node performs these stages:

1. Subscribe to raw `sensor_msgs/msg/PointCloud2` and IMU messages.
2. Convert RS-LiDAR points to an internal `PointXYZI` cloud and estimate per-point relative time when needed.
3. Filter points by sampling ratio, blind range, and max distance.
4. Synchronize one lidar frame with the IMU messages covering that frame.
5. Run IMU initialization, state propagation, and point cloud undistortion.
6. Publish the processed cloud as `sensor_msgs/msg/PointCloud2`.

The undistortion code currently:

- uses lidar-to-IMU extrinsics
- supports IMU rotational compensation
- defaults to disabling IMU translation compensation in the point compensation formula
- keeps timing and synchronization logs throttled to once every 2 seconds

## Topics

- Input point cloud: `input_topic`, default `/input/points`
- Input IMU: `imu_topic`, default `/input/imu`
- Output point cloud: `output_topic`, default `/sensor/undistorted_points`

## Parameters

The node declares these parameters at startup.

| Name | Type | Default | Meaning |
| --- | --- | --- | --- |
| `input_topic` | `string` | `/input/points` | Input point cloud topic |
| `output_topic` | `string` | `/sensor/undistorted_points` | Output point cloud topic |
| `imu_topic` | `string` | `/input/imu` | Input IMU topic |
| `output_frame` | `string` | `""` | Override output `frame_id` when non-empty |
| `extrinsic_t` | `double[3]` | `[0.0, 0.0, 0.0]` | Lidar-to-IMU translation |
| `extrinsic_r` | `double[9]` | `[0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0]` | Lidar-to-IMU rotation matrix, row-major |
| `scan_line` | `int` | `32` | Number of scan lines |
| `scan_rate` | `int` | `10` | Lidar scan rate in Hz |
| `point_filter_num` | `int` | `1` | Keep one point every `N` points |
| `blind_threshold` | `double` | `0.5` | Minimum valid range in meters |
| `distance_threshold` | `double` | `6.0` | Maximum valid range in meters, `<= 0` disables the cap |
| `use_imu` | `bool` | `true` | Current code keeps this mainly for pipeline/log control; it does not fully disable IMU processing in `processingLoop()` |

## Runtime-Reconfigurable Parameters

These parameters are accepted by the parameter callback at runtime:

- `output_frame`
- `extrinsic_t`
- `extrinsic_r`
- `scan_line`
- `scan_rate`
- `point_filter_num`
- `blind_threshold`
- `distance_threshold`
- `use_imu`

These are not dynamically reconfigurable:

- `input_topic`
- `output_topic`
- `imu_topic`

## IMU Initialization

`ImuProcess` keeps state across frames. It is not re-initialized before every undistortion call.

Current initialization behavior:

- `init_iter_num_` starts from `500`
- initialization uses early IMU frames to estimate mean acceleration and gyro bias
- gravity direction is estimated from the measured acceleration mean, so IMU axes do not need to be aligned with world axes

Practical requirement:

- keep the sensor stationary during initialization if you want stable gravity and bias estimation

## Translation Compensation

The current code includes a switch inside `ImuProcess` for IMU translation compensation.

Default behavior in the current implementation:

- lidar-to-IMU extrinsic rotation and translation are used
- IMU motion translation compensation is disabled by default
- IMU motion rotation compensation remains enabled

If you change the code to enable translation compensation, the compensation formula will also use:

- pose
- velocity
- acceleration

from the cached IMU state history.

## Build

```bash
cd ~/workspace/north
source /opt/ros/humble/setup.bash
colcon build --packages-select pointcloud_undistortion
```

## Launch

```bash
cd ~/workspace/north
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pointcloud_undistortion pointcloud_undistortion.launch.py
```

Launch arguments:

- `input_topic`
- `output_topic`
- `imu_topic`
- `output_frame`
- `use_imu`
- `use_rviz`

Example:

```bash
ros2 launch pointcloud_undistortion pointcloud_undistortion.launch.py \
  input_topic:=/rslidar_points_front \
  imu_topic:=/imu/data \
  output_topic:=/sensor/points_undistorted \
  output_frame:=base_link
```

## Logs

Current throttled logs include:

- preprocess timing and point count summary
- synchronization timing
- measure group timestamps
- processing loop timing
- `undistortPcl timing` summary, limited to once every 2 seconds

## Notes And Limitations

- The preprocessing code is tailored to `rslidar_ros::Point` input.
- The point relative time is stored in `curvature` in milliseconds.
- The first lidar frame is skipped to bootstrap timing and IMU history.
- Runtime parameter updates for topics are intentionally rejected.
- `use_imu=false` is not a full bypass switch in the current code path.

## Main Files

- [`src/pointcloud_undistortion_node.cpp`](/home/csp/workspace/north/src/sensor/pointcloud_preprocess/pointcloud_undistortion/src/pointcloud_undistortion_node.cpp)
- [`src/imu_process.cpp`](/home/csp/workspace/north/src/sensor/pointcloud_preprocess/pointcloud_undistortion/src/imu_process.cpp)
- [`src/preprocess.cpp`](/home/csp/workspace/north/src/sensor/pointcloud_preprocess/pointcloud_undistortion/src/preprocess.cpp)
- [`params/pointcloud_undistortion.param.yaml`](/home/csp/workspace/north/src/sensor/pointcloud_preprocess/pointcloud_undistortion/params/pointcloud_undistortion.param.yaml)
- [`launch/pointcloud_undistortion.launch.py`](/home/csp/workspace/north/src/sensor/pointcloud_preprocess/pointcloud_undistortion/launch/pointcloud_undistortion.launch.py)
