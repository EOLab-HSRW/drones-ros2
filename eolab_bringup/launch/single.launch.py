"""
Launch the backend-independent processes required by a single drone instance.

This launch file provides the processes that are common to both simulation and
hardware deployments/

The file supports two deployment patterns:


## Standalone single-drone operation
---------------------------------
Run this file directly with ``standalone:=true``. In this mode, it launches the
common processes for one drone and prints a startup banner describing the
selected system, drone model, namespace alias, and instance number.

## Composable multi-drone operation
--------------------------------
Include this file once per drone with ``standalone:=false``.

In a multi-drone simulation, a parent launch file should include this launch
file for every simulated drone, assigning each instance a unique ``alias`` and
``instance`` value.

In a multi-drone hardware deployment, each drone normally runs one instance of
this launch file on its own companion computer. The same parent experiment
configuration can therefore be used to start the common per-drone processes in
both hardware and simulation, while system-specific processes are launched
separately.

The ``standalone`` argument only controls standalone presentation behavior,
such as the startup banner. It does not change the set of per-drone nodes
launched by this file.
"""

import xacro

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

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

    standalone = (LaunchConfiguration("standalone").perform(context) == "true")

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

    Running in {mode} mode.

"""
        print(banner)

    ros_domain_id = EnvironmentVariable(
        name="ROS_DOMAIN_ID",
        default_value="",
    ).perform(context)

    if ros_domain_id.isdigit():
        domain_id = ros_domain_id
        info_msg = f"ROS_DOMAIN_ID detected: {domain_id}"
    else:
        domain_id = ros_domain_id
        info_msg = "ROS_DOMAIN_ID is set, but value is not a non-negative integer"

    actions.extend(LogInfo(msg=info_msg))

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
