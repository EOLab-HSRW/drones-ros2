import xacro

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from eolab_bringup.commons import (
    BOOLEAN_CHOICES,
    SYSTEM_CHOICES,
    System,
    get_drone_xacro,
)


def launch_setup(context: LaunchContext):
    """Create the backend-independent nodes for one drone."""
    system = LaunchConfiguration("system").perform(context)
    drone = LaunchConfiguration("drone").perform(context)
    alias = LaunchConfiguration("alias").perform(context)
    instance = LaunchConfiguration("instance").perform(context)

    standalone = (
        LaunchConfiguration("standalone")
        .perform(context)
        .strip()
        .lower()
        == "true"
    )

    if system == System.GAZEBO.value:
        mode = "simulation"
        use_sim_time_value = True
    elif system == System.HARDWARE.value:
        mode = "hardware"
        use_sim_time_value = False
    else:
        # Normally prevented by DeclareLaunchArgument(choices=...).
        raise RuntimeError(
            f"Unsupported system '{system}'. "
            f"Expected '{System.GAZEBO.value}' or "
            f"'{System.HARDWARE.value}'."
        )

    xacro_file = get_drone_xacro(drone)

    robot_description = ParameterValue(
        xacro.process_file(str(xacro_file)).toxml(),
        value_type=str,
    )

    use_sim_time = ParameterValue(
        use_sim_time_value,
        value_type=bool,
    )

    actions = []

    if standalone:
        banner = f"""
 _____ ___  _          _
| ____/ _ \\| |    __ _| |__   System:   {system}
|  _|| | | | |   / _` | '_ \\  Drone:    {drone}
| |__| |_| | |__| (_| | |_) | Alias:    {alias}
|_____\\___/|_____\\__,_|_.__/  Instance: {instance}
"""

        actions.extend([
            LogInfo(msg=banner),
            LogInfo(msg=f"Running in {mode} mode."),
        ])

    actions.extend([
        Node(
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
        ),

        Node(
            package="eolab_utils",
            executable="px4_odom_tf",
            name="px4_odom_tf",
            namespace=alias,
            output="both",
            parameters=[{
                "use_sim_time": use_sim_time,
            }],
        ),

        Node(
            package="eolab_utils",
            executable="odom_viz",
            name="odom_viz",
            namespace=alias,
            output="both",
            parameters=[{
                "use_sim_time": use_sim_time,
            }],
        ),
    ])

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            name="system",
            choices=SYSTEM_CHOICES,
            description="Execution system.",
        ),

        DeclareLaunchArgument(
            name="drone",
            description="Drone model name.",
        ),

        DeclareLaunchArgument(
            name="alias",
            description="ROS namespace assigned to the drone.",
        ),

        DeclareLaunchArgument(
            name="instance",
            description="Drone instance number.",
        ),

        DeclareLaunchArgument(
            name="standalone",
            default_value="true",
            choices=BOOLEAN_CHOICES,
            description=(
                "Enable standalone single-drone presentation, including "
                "the startup banner."
            ),
        ),

        OpaqueFunction(function=launch_setup),
    ])
