from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("pointcloud_undistortion")
    params_file = PathJoinSubstitution([package_share, "params", "pointcloud_undistortion.param.yaml"])
    rviz_config = PathJoinSubstitution([package_share, "rviz", "pointcloud_undistortion.rviz"])

    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    output_frame = LaunchConfiguration("output_frame")
    use_imu = LaunchConfiguration("use_imu")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_input_topic = DeclareLaunchArgument(
        "input_topic",
        default_value="/input/points",
        description="Input PointCloud2 topic",
    )
    declare_output_topic = DeclareLaunchArgument(
        "output_topic",
        default_value="/sensor/undistorted_points",
        description="Output undistorted PointCloud2 topic",
    )
    declare_imu_topic = DeclareLaunchArgument(
        "imu_topic",
        default_value="/input/imu",
        description="Input IMU topic",
    )
    declare_output_frame = DeclareLaunchArgument(
        "output_frame",
        default_value="",
        description="Optional output frame_id override",
    )
    declare_use_imu = DeclareLaunchArgument(
        "use_imu",
        default_value="true",
        description="Enable IMU-assisted undistortion pipeline scaffolding",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz together with the node",
    )

    pointcloud_undistortion_node = Node(
        package="pointcloud_undistortion",
        executable="pointcloud_undistortion_node",
        name="pointcloud_undistortion",
        output="screen",
        parameters=[
            params_file,
            {
                "input_topic": input_topic,
                "output_topic": output_topic,
                "imu_topic": imu_topic,
                "output_frame": output_frame,
                "use_imu": use_imu,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            declare_input_topic,
            declare_output_topic,
            declare_imu_topic,
            declare_output_frame,
            declare_use_imu,
            declare_use_rviz,
            pointcloud_undistortion_node,
            rviz_node,
        ]
    )
