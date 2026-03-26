from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    uav_name = LaunchConfiguration("uav_name")
    global_frame = LaunchConfiguration("global_frame")
    standalone = LaunchConfiguration("standalone")
    container_name = LaunchConfiguration("container_name")
    config_file = PathJoinSubstitution(
        [FindPackageShare("filter_reflective_uavs"), "config", "filter_reflective_uavs.yaml"]
    )

    filter_node = ComposableNode(
        package="filter_reflective_uavs",
        plugin="filter_reflective_uavs::FilterReflectiveUavs",
        name="filter_reflective_uavs",
        namespace=uav_name,
        parameters=[
            {
                "config": config_file,
                "custom_config": "",
                "uav_name": uav_name,
                "global_frame": global_frame,
            },
        ],
        remappings=[
            ("~/lidar3d_in", "livox/points"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("uav_name", default_value="uav1"),
        DeclareLaunchArgument("global_frame", default_value=[uav_name, "/world_origin"]),
        DeclareLaunchArgument("standalone", default_value="true"),
        DeclareLaunchArgument("container_name", default_value=""),
        LoadComposableNodes(
            target_container=container_name,
            composable_node_descriptions=[filter_node],
            condition=UnlessCondition(standalone),
        ),
        ComposableNodeContainer(
            namespace=uav_name,
            name="filter_reflective_uavs_container",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[filter_node],
            condition=IfCondition(standalone),
        ),
    ])
