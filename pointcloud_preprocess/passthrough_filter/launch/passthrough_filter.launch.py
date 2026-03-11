from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("passthrough_filter")
    params_file = PathJoinSubstitution([package_share, "params", "passthrough_filter.param.yaml"])
    rviz_config = PathJoinSubstitution([package_share, "rviz", "passthrough_filter.rviz"])

    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    input_frame = LaunchConfiguration("input_frame")
    filter_field_name = LaunchConfiguration("filter_field_name")
    min_limit = LaunchConfiguration("min_limit")
    max_limit = LaunchConfiguration("max_limit")
    negative = LaunchConfiguration("negative")
    use_pcl = LaunchConfiguration("use_pcl")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_input_topic = DeclareLaunchArgument(
        "input_topic",
        default_value="/input/points",
        description="Input PointCloud2 topic",
    )
    declare_output_topic = DeclareLaunchArgument(
        "output_topic",
        default_value="/sensor/passthrough_filtered",
        description="Output filtered PointCloud2 topic",
    )
    declare_input_frame = DeclareLaunchArgument(
        "input_frame",
        default_value="",
        description="Filter reference frame, empty means use the cloud frame directly",
    )
    declare_filter_field_name = DeclareLaunchArgument(
        "filter_field_name",
        default_value="z",
        description="Field name used for passthrough filtering",
    )
    declare_min_limit = DeclareLaunchArgument(
        "min_limit",
        default_value="-2.0",
        description="Lower limit of the passthrough filter",
    )
    declare_max_limit = DeclareLaunchArgument(
        "max_limit",
        default_value="2.0",
        description="Upper limit of the passthrough filter",
    )
    declare_negative = DeclareLaunchArgument(
        "negative",
        default_value="false",
        description="Invert filter selection",
    )
    declare_use_pcl = DeclareLaunchArgument(
        "use_pcl",
        default_value="true",
        description="Use PCL PassThrough backend",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz together with the node",
    )

    passthrough_filter_node = Node(
        package="passthrough_filter",
        executable="passthrough_filter_node",
        name="passthrough_filter",
        output="screen",
        parameters=[
            params_file,
            {
                "input_topic": input_topic,
                "output_topic": output_topic,
                "input_frame": input_frame,
                "filter_field_name": filter_field_name,
                "min_limit": min_limit,
                "max_limit": max_limit,
                "negative": negative,
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
            declare_filter_field_name,
            declare_min_limit,
            declare_max_limit,
            declare_negative,
            declare_use_pcl,
            declare_use_rviz,
            passthrough_filter_node,
            rviz_node,
        ]
    )
