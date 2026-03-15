# Pointcloud Preprocess

`pointcloud_preprocess` contains ROS 2 point cloud preprocessing packages used in the `north` workspace.

## Packages

- `pointcloud_undistortion`: IMU-assisted point cloud preprocessing and motion undistortion for RS-LiDAR style input.
- `crop_box_filter`: Crop-box filtering package.
- `passthrough_filter`: Pass-through filtering package.

## Build

```bash
cd ~/workspace/north
source /opt/ros/humble/setup.bash
colcon build --packages-select pointcloud_undistortion crop_box_filter passthrough_filter
```

## Package Docs

- [`pointcloud_undistortion/README.md`](/home/csp/workspace/north/src/sensor/pointcloud_preprocess/pointcloud_undistortion/README.md)
- [`crop_box_filter/README.md`](/home/csp/workspace/north/src/sensor/pointcloud_preprocess/crop_box_filter/README.md)
- [`passthrough_filter/README.md`](/home/csp/workspace/north/src/sensor/pointcloud_preprocess/passthrough_filter/README.md)
