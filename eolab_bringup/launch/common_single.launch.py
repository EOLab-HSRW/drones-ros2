import xacro

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DESCRIPTION_PACKAGE = "eolab_description"


def launch_setup(context):
    drone = LaunchConfiguration("drone").perform(context)
    alias = LaunchConfiguration("alias").perform(context)

    xacro_file = (
        get_package_share_path(DESCRIPTION_PACKAGE)
        / "drones"
        / f"{drone}.urdf.xacro"
    )

    if not xacro_file.is_file():
        raise FileNotFoundError(
            f"Drone xacro file was not found: '{xacro_file}'"
        )

    robot_description = (
        xacro.process_file(str(xacro_file))
        .toxml()
        .replace("\n", "")
    )

    # Explicitly force the launch argument to become a ROS boolean parameter.
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"),
        value_type=bool,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=alias,
        exec_name=f"{alias}.robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_description,
        }],
    )

    px4_odom_tf = Node(
        package="eolab_utils",
        executable="px4_odom_tf",
        name="px4_odom_tf",
        namespace=alias,
        output="both",
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
    )

    odom_viz = Node(
        package="eolab_utils",
        executable="odom_viz",
        name="odom_viz",
        namespace=alias,
        output="both",
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
    )

    return [
        robot_state_publisher,
        px4_odom_tf,
        odom_viz,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "drone",
            description="Drone model used to select the URDF xacro file.",
        ),
        DeclareLaunchArgument(
            "alias",
            description="ROS namespace assigned to the drone.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            choices=["true", "false"],
            description="Use the simulation clock.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
