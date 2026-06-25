"""Shared constants and resource validation for the EOLab launch system."""

from enum import Enum
from functools import lru_cache
from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_path,
)


DESCRIPTION_PACKAGE = "eolab_description"
COMMON_BRINGUP_PACKAGE = "eolab_bringup"
SIM_BRINGUP_PACKAGE = "eolab_bringup_sim"
HW_BRINGUP_PACKAGE = "eolab_bringup_hw"

DEFAULT_DRONE = "protoflyer"
DEFAULT_INSTANCE = "1"
DEFAULT_WORLD = "empty"

BOOLEAN_CHOICES = ("true", "false")

class System(str, Enum):
    """Supported EOLab execution systems."""
    GAZEBO = "gz"
    HARDWARE = "hw"


SYSTEM_CHOICES = tuple(system.value for system in System)


@lru_cache(maxsize=None)
def get_package_share(package_name: str) -> Path:
    """Resolve an installed ROS 2 package share directory."""
    try:
        return get_package_share_path(package_name)
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f"Required ROS 2 package '{package_name}' was not found. "
            "Build and install it, then source the workspace."
        ) from exc


def get_package_file(
    package_name: str,
    *relative_parts: str,
) -> Path:
    """Resolve and validate a file inside an installed package share."""
    path = get_package_share(package_name).joinpath(*relative_parts)

    if not path.is_file():
        relative_path = "/".join(relative_parts)

        raise RuntimeError(
            f"Resource '{relative_path}' was not found in package "
            f"'{package_name}' at '{path}'."
        )

    return path


def get_launch_file(package_name: str, filename: str) -> Path:
    """Resolve and validate an installed Python launch file."""
    return get_package_file(
        package_name,
        "launch",
        filename,
    )


def get_drone_xacro(drone: str) -> Path:
    """Resolve and validate the xacro file for a drone model."""
    return get_package_file(
        DESCRIPTION_PACKAGE,
        "drones",
        f"{drone}.urdf.xacro",
    )


@lru_cache(maxsize=1)
def get_worlds() -> tuple[str, ...]:
    """Return available SDF world names."""
    worlds_directory = get_package_share(DESCRIPTION_PACKAGE) / "worlds"

    if not worlds_directory.is_dir():
        raise RuntimeError(
            f"World directory was not found at '{worlds_directory}'."
        )

    worlds = tuple(sorted(
        world_file.stem
        for world_file in worlds_directory.glob("*.sdf")
        if world_file.is_file()
    ))

    if not worlds:
        raise RuntimeError(
            f"No '.sdf' world files were found in "
            f"'{worlds_directory}'."
        )

    if DEFAULT_WORLD not in worlds:
        raise RuntimeError(
            f"Default world '{DEFAULT_WORLD}' was not found in "
            f"'{worlds_directory}'."
        )

    return worlds
