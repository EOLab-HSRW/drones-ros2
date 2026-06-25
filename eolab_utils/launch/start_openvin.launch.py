import launch
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description() -> launch.LaunchDescription:

    ld = LaunchDescription()

    config_path = PathJoinSubstitution([
        FindPackageShare("eolab_utils"),
        "config",
        "open_vins",
        "mono_cam",
        "estimator_config.yaml"
    ])

    ov_msckf = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        namespace="protoflyer",
        output='screen',
        parameters=[
            {"save_total_state": False},
            {"config_path": config_path},
        ],
    )
    ld.add_action(ov_msckf)

    return ld
