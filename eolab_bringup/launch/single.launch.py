from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


SYSTEM_GZ = "gz"
SYSTEM_HW = "hw"


def launch_setup(context):
    system = LaunchConfiguration("system").perform(context)
    drone = LaunchConfiguration("drone").perform(context)
    alias = LaunchConfiguration("alias").perform(context)
    instance = LaunchConfiguration("instance").perform(context)

    if system == SYSTEM_GZ:
        mode = "simulation"
        use_sim_time = "true"
    elif system == SYSTEM_HW:
        mode = "hardware"
        use_sim_time = "false"
    else:
        # Normally prevented by DeclareLaunchArgument(choices=...).
        raise RuntimeError(
            f"Unsupported system '{system}'. "
            f"Expected '{SYSTEM_GZ}' or '{SYSTEM_HW}'."
        )

    banner = f"""
 _____ ___  _          _
| ____/ _ \\| |    __ _| |__   System:   {system}
|  _|| | | | |   / _` | '_ \\  Drone:    {drone}
| |__| |_| | |__| (_| | |_) | Alias:    {alias}
|_____\\___/|_____\\__,_|_.__/  Instance: {instance}
"""

    common_single = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("eolab_bringup"),
                "launch",
                "common_single.launch.py",
            ])
        ),
        launch_arguments={
            "drone": drone,
            "alias": alias,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return [
        LogInfo(msg=banner),
        LogInfo(msg=f"Running in {mode} mode."),
        common_single,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "system",
            choices=[SYSTEM_GZ, SYSTEM_HW],
            description="Target system: Gazebo simulation or hardware.",
        ),
        DeclareLaunchArgument(
            "drone",
            description="Drone model name.",
        ),
        DeclareLaunchArgument(
            "alias",
            description="Runtime alias for the drone.",
        ),
        DeclareLaunchArgument(
            "instance",
            description="Drone instance number.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
