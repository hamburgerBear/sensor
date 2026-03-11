from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("crop_box_filter")
    params_file = PathJoinSubstitution([package_share, "params", "crop_box_filter.param.yaml"])
    rviz_config = PathJoinSubstitution([package_share, "rviz", "crop_box_filter.rviz"])

    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    input_frame = LaunchConfiguration("input_frame")
    marker_topic = LaunchConfiguration("marker_topic")
    visual = LaunchConfiguration("visual")
    use_pcl = LaunchConfiguration("use_pcl")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_input_topic = DeclareLaunchArgument(
        "input_topic",
        default_value="/input/points",
        description="Input PointCloud2 topic",
    )
    declare_output_topic = DeclareLaunchArgument(
        "output_topic",
        default_value="/sensor/crop_box_filted",
        description="Output filtered PointCloud2 topic",
    )
    declare_input_frame = DeclareLaunchArgument(
        "input_frame",
        default_value="",
        description="Expected input frame_id, empty means disabled",
    )
    declare_marker_topic = DeclareLaunchArgument(
        "marker_topic",
        default_value="/debug/crop_box_marker",
        description="Marker topic for crop box visualization",
    )
    declare_visual = DeclareLaunchArgument(
        "visual",
        default_value="true",
        description="Enable filtered cloud and crop box marker output",
    )
    declare_use_pcl = DeclareLaunchArgument(
        "use_pcl",
        default_value="true",
        description="Use PCL CropBox backend",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz together with the node",
    )

    crop_box_filter_node = Node(
        package="crop_box_filter",
        executable="crop_box_filter_node",
        name="crop_box_filter",
        output="screen",
        parameters=[
            params_file,
            {
                "input_topic": input_topic,
                "output_topic": output_topic,
                "input_frame": input_frame,
                "marker_topic": marker_topic,
                "visual": visual,
                "use_pcl": use_pcl,
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
            declare_input_frame,
            declare_marker_topic,
            declare_visual,
            declare_use_pcl,
            declare_use_rviz,
            crop_box_filter_node,
            rviz_node,
        ]
    )
