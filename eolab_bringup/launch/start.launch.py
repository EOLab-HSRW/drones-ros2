"""
Top-level single-drone launch entry point.

This file is intended to serve as the primary umbrella launch file for Gazebo
simulation (``system:=gz``), physical hardware (``system:=hw``), and potentially
other backend systems or custom simulators in the future.

It is designed exclusively for single-drone environments and must not be used
to launch or coordinate multi-drone setups.

The default argument values are configured to bring up a complete simulation
environment without additional configuration. For a standard hardware launch,
only ``system:=hw`` should normally be required; the internal defaults are
expected to provide the remaining configuration.
"""

import eolab_drones

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

from eolab_bringup.commons import (
    BOOLEAN_CHOICES,
    COMMON_BRINGUP_PACKAGE,
    DEFAULT_DRONE,
    DEFAULT_INSTANCE,
    DEFAULT_WORLD,
    HW_BRINGUP_PACKAGE,
    SIM_BRINGUP_PACKAGE,
    SYSTEM_CHOICES,
    System,
    get_launch_file,
    get_worlds,
)


def launch_setup(context: LaunchContext):
    """Select and include the common and system-specific bringup."""
    system = LaunchConfiguration("system").perform(context)

    if system == System.GAZEBO.value:
        bringup_package = SIM_BRINGUP_PACKAGE
        is_simulation = True
    elif system == System.HARDWARE.value:
        bringup_package = HW_BRINGUP_PACKAGE
        is_simulation = False
    else:
        # Normally prevented by DeclareLaunchArgument(choices=...).
        raise RuntimeError(
            f"Unsupported system '{system}'. "
            f"Expected '{System.GAZEBO.value}' or "
            f"'{System.HARDWARE.value}'."
        )

    single_launch_file = get_launch_file(
        COMMON_BRINGUP_PACKAGE,
        "single.launch.py",
    )

    bringup_launch_file = get_launch_file(
        bringup_package,
        "start.launch.py",
    )

    single = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(single_launch_file)),
        launch_arguments={
            "system": LaunchConfiguration("system"),
            "drone": LaunchConfiguration("drone"),
            "alias": LaunchConfiguration("alias"),
            "instance": LaunchConfiguration("instance"),
            "standalone": "true",
        }.items(),
    )

    if is_simulation:
        bringup_arguments = {
            "drone": LaunchConfiguration("drone"),
            "alias": LaunchConfiguration("alias"),
            "instance": LaunchConfiguration("instance"),

            # Map the backend-independent public interface to the current
            # simulation backend interface.
            "x": LaunchConfiguration("sim/pos_x"),
            "y": LaunchConfiguration("sim/pos_y"),
            "z": LaunchConfiguration("sim/pos_z"),

            "lat": LaunchConfiguration("sim/lat"),
            "lon": LaunchConfiguration("sim/lon"),
            "alt": LaunchConfiguration("sim/alt"),

            "world": LaunchConfiguration("sim/world"),
            "skip_world": LaunchConfiguration("sim/skip_world"),
            "camera_follow": LaunchConfiguration("sim/camera_follow"),

            "verbose": LaunchConfiguration("verbose"),
            "rviz": LaunchConfiguration("rviz"),
        }
    else:
        bringup_arguments = {
            "verbose": LaunchConfiguration("verbose"),
        }

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_launch_file)),
        launch_arguments=bringup_arguments.items(),
    )

    # Preserve the intended execution order:
    # 1. Common single-drone nodes.
    # 2. Simulation or hardware-specific bringup.
    return [
        single,
        bringup,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([

        DeclareLaunchArgument(
            name="system",
            default_value=EnvironmentVariable(
                "EOLAB_SYSTEM",
                default_value=System.GAZEBO.value,
            ),
            choices=SYSTEM_CHOICES,
            description="Select the simulation or hardware system.",
        ),

        DeclareLaunchArgument(
            name="drone",
            default_value=EnvironmentVariable(
                "EOLAB_DRONE",
                default_value=DEFAULT_DRONE,
            ),
            choices=tuple(sorted(eolab_drones.get_drones().keys())),
            description="Name of the drone to launch.",
        ),

        DeclareLaunchArgument(
            name="alias",
            default_value=LaunchConfiguration("drone"),
            description="Custom alias; defaults to the drone name.",
        ),

        DeclareLaunchArgument(
            name="instance",
            default_value=DEFAULT_INSTANCE,
            description="Drone instance number.",
        ),

        DeclareLaunchArgument(
            name="verbose",
            default_value="false",
            choices=BOOLEAN_CHOICES,
            description="Enable verbose launch output.",
        ),

        # Simulation arguments use the "sim/..." scope rather than "gz/...".
        # This keeps the public configuration independent from the current
        # simulation backend.
        DeclareLaunchArgument(
            name="sim/pos_x",
            default_value="0.0",
            description="Drone spawn X position.",
        ),

        DeclareLaunchArgument(
            name="sim/pos_y",
            default_value="0.0",
            description="Drone spawn Y position.",
        ),

        DeclareLaunchArgument(
            name="sim/pos_z",
            default_value="0.8",
            description="Drone spawn Z position.",
        ),

        DeclareLaunchArgument(
            name="sim/lat",
            default_value="51.497741558866004",
            description="WGS84 simulation latitude.",
        ),

        DeclareLaunchArgument(
            name="sim/lon",
            default_value="6.549182534441797",
            description="WGS84 simulation longitude.",
        ),

        DeclareLaunchArgument(
            name="sim/alt",
            default_value="26.54",
            description="WGS84 simulation altitude.",
        ),

        DeclareLaunchArgument(
            name="sim/world",
            default_value=DEFAULT_WORLD,
            choices=get_worlds(),
            description=(
                "Simulation world name without the '.sdf' extension."
            ),
        ),

        DeclareLaunchArgument(
            name="sim/skip_world",
            default_value="false",
            choices=BOOLEAN_CHOICES,
            description=(
                "Use an already-running simulator instead of starting "
                "the world from this launch file."
            ),
        ),

        DeclareLaunchArgument(
            name="sim/camera_follow",
            default_value="true",
            choices=BOOLEAN_CHOICES,
            description=(
                "Make the simulator GUI camera follow the spawned drone."
            ),
        ),

        DeclareLaunchArgument(
            name="rviz",
            default_value="true",
            choices=BOOLEAN_CHOICES,
            description="Start RViz.",
        ),

        OpaqueFunction(function=launch_setup),
    ])
