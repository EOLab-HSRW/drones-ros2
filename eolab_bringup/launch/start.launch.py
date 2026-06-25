from pathlib import Path
from typing import List

import eolab_drones
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    LogInfo
    # RaiseError, # No available in Humble
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


SYSTEM_GZ = "gz"
SYSTEM_HW = "hw"
DEFAULT_WORLD = "empty"

DESCRIPTION_PACKAGE = "eolab_description"
COMMON_BRINGUP_PACKAGE = "eolab_bringup"
SIM_BRINGUP_PACKAGE = "eolab_bringup_sim"
HW_BRINGUP_PACKAGE = "eolab_bringup_hw"


def _get_package_share(package_name: str) -> Path:
    """Return a package share directory with a useful error message."""
    try:
        return Path(get_package_share_directory(package_name))
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f"Required ROS 2 package '{package_name}' was not found. "
            "Build/install it and source the workspace before launching."
        ) from exc


def _get_launch_file(package_name: str, filename: str) -> Path:
    """Resolve and validate a Python launch file from a package."""
    launch_file = _get_package_share(package_name) / "launch" / filename

    if not launch_file.is_file():
        raise RuntimeError(
            f"Launch file '{filename}' was not found in package "
            f"'{package_name}' at '{launch_file}'."
        )

    return launch_file


def get_worlds() -> List[str]:
    """Return available SDF world names from eolab_description/worlds."""
    worlds_dir = _get_package_share(DESCRIPTION_PACKAGE) / "worlds"

    if not worlds_dir.is_dir():
        raise RuntimeError(
            f"World directory was not found at '{worlds_dir}'."
        )

    worlds = sorted(
        world_file.stem
        for world_file in worlds_dir.glob("*.sdf")
        if world_file.is_file()
    )

    if not worlds:
        raise RuntimeError(
            f"No '.sdf' world files were found in '{worlds_dir}'."
        )

    if DEFAULT_WORLD not in worlds:
        raise RuntimeError(
            f"Default world '{DEFAULT_WORLD}' is not available in "
            f"'{worlds_dir}'."
        )

    return worlds


def launch_setup(context: LaunchContext):
    """Select and include the system-specific bringup at launch time."""
    system = LaunchConfiguration("system").perform(context)

    if system not in (SYSTEM_GZ, SYSTEM_HW):
        return [
            LogInfo(
                msg=(
                    f"[ERROR] Unsupported system '{system}'. "
                    f"Expected '{SYSTEM_GZ}' or '{SYSTEM_HW}'."
                )
            )
        ]

    is_gz = system == SYSTEM_GZ
    bringup_package = (
        SIM_BRINGUP_PACKAGE if is_gz else HW_BRINGUP_PACKAGE
    )

    try:
        single_launch_file = _get_launch_file(
            COMMON_BRINGUP_PACKAGE,
            "single.launch.py",
        )
        bringup_launch_file = _get_launch_file(
            bringup_package,
            "bringup.launch.py",
        )
    except RuntimeError as exc:
        return [LogInfo(msg=str(exc))]

    single = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(single_launch_file)),
        launch_arguments={
            "system": LaunchConfiguration("system"),
            "drone": LaunchConfiguration("drone"),
            "alias": LaunchConfiguration("alias"),
            "instance": LaunchConfiguration("instance"),
        }.items(),
    )

    if is_gz:
        bringup_arguments = {
            "drone": LaunchConfiguration("drone"),
            "alias": LaunchConfiguration("alias"),
            "instance": LaunchConfiguration("instance"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "lon": LaunchConfiguration("lon"),
            "lat": LaunchConfiguration("lat"),
            "alt": LaunchConfiguration("alt"),
            "verbose": LaunchConfiguration("verbose"),
            "world": LaunchConfiguration("world"),
            "skip_world": LaunchConfiguration("skip_world"),
        }
    else:
        bringup_arguments = {
            "verbose": LaunchConfiguration("verbose"),
        }

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_launch_file)),
        launch_arguments=bringup_arguments.items(),
    )

    # Preserve the original execution order: common single-drone setup first,
    # then the selected simulation or hardware bringup.
    return [single, bringup]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            name="system",
            default_value=EnvironmentVariable(
                "EOLAB_SYSTEM",
                default_value=SYSTEM_GZ,
            ),
            choices=[SYSTEM_GZ, SYSTEM_HW],
            description="Select the simulation or hardware system.",
        ),
        DeclareLaunchArgument(
            name="drone",
            default_value=EnvironmentVariable(
                "EOLAB_DRONE",
                default_value="protoflyer",
            ),
            choices=list(eolab_drones.get_drones().keys()),
            description="Name of the drone to launch.",
        ),
        DeclareLaunchArgument(
            name="alias",
            default_value=LaunchConfiguration("drone"),
            description="Custom alias; defaults to the drone name.",
        ),
        DeclareLaunchArgument(
            name="instance",
            default_value="0",
            description="Drone instance number (simulation only).",
        ),
        DeclareLaunchArgument(
            name="x",
            default_value="0.0",
            description="Drone spawn X position (simulation only).",
        ),
        DeclareLaunchArgument(
            name="y",
            default_value="0.0",
            description="Drone spawn Y position (simulation only).",
        ),
        DeclareLaunchArgument(
            name="z",
            default_value="0.8",
            description="Drone spawn Z position (simulation only).",
        ),
        DeclareLaunchArgument(
            name="lat",
            default_value="51.497741558866004",
            description="WGS84 latitude (simulation only).",
        ),
        DeclareLaunchArgument(
            name="lon",
            default_value="6.549182534441797",
            description="WGS84 longitude (simulation only).",
        ),
        DeclareLaunchArgument(
            name="alt",
            default_value="26.54",
            description="WGS84 altitude (simulation only).",
        ),
        DeclareLaunchArgument(
            name="world",
            default_value=DEFAULT_WORLD,
            choices=get_worlds(),
            description=(
                "World name without the file extension. Available worlds "
                "are loaded from eolab_description/worlds."
            ),
        ),
        DeclareLaunchArgument(
            name="skip_world",
            default_value="false",
            description="Skip launching the simulation world.",
        ),
        DeclareLaunchArgument(
            "camera_follow",
            default_value="true",
            choices=["true", "false"],
            description=(
                "Enable Gazebo GUI camera follow for single-drone simulation."
            ),
        ),
        DeclareLaunchArgument(
            name="verbose",
            default_value="false",
            description="Enable verbose launch output.",
        ),
        DeclareLaunchArgument(
            name="rviz",
            default_value="true",
            description="Start RViz.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
