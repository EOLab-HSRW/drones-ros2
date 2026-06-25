import shlex
import shutil
import subprocess
from pathlib import Path
from typing import List, Optional, Tuple

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_path,
)
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


DESCRIPTION_PACKAGE = "eolab_description"
SIM_BRINGUP_PACKAGE = "eolab_bringup_sim"

REQUIRED_GZ_SIM_MAJOR = 8
DEFAULT_GZ_READY_TIMEOUT_S = "20.0"


def _query_gazebo_versions(
    gz_executable: str,
) -> Tuple[Optional[List[str]], Optional[str]]:
    """Return installed Gazebo Sim versions or an error message."""
    try:
        result = subprocess.run(
            [gz_executable, "sim", "--versions"],
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return None, "Timed out while running 'gz sim --versions'."
    except OSError as exc:
        return None, f"Failed to execute 'gz sim --versions': {exc}"

    if result.returncode != 0:
        details = "\n".join(
            output.strip()
            for output in (result.stdout, result.stderr)
            if output.strip()
        )

        message = (
            "The command 'gz sim --versions' failed with exit code "
            f"{result.returncode}."
        )

        if details:
            message += f"\n{details}"

        return None, message

    versions = [
        line.strip()
        for line in result.stdout.splitlines()
        if line.strip()
    ]

    return versions, None


def launch_setup(context: LaunchContext):
    world_name = LaunchConfiguration("world").perform(context)
    drone_name = LaunchConfiguration("drone").perform(context)
    verbose = (
        LaunchConfiguration("verbose").perform(context) == "true"
    )

    latitude = LaunchConfiguration("lat").perform(context)
    longitude = LaunchConfiguration("lon").perform(context)
    altitude = LaunchConfiguration("alt").perform(context)

    alias = LaunchConfiguration("alias")

    # ------------------------------------------------------------------
    # Gazebo installation and version checks
    # ------------------------------------------------------------------

    gz_executable = shutil.which("gz")

    if gz_executable is None:
        return [
            Shutdown(
                reason=(
                    "Gazebo is not available: the 'gz' executable was "
                    "not found in PATH. Gazebo Sim 8 is required."
                )
            )
        ]

    versions, version_error = _query_gazebo_versions(gz_executable)

    if version_error is not None:
        return [
            Shutdown(reason=version_error)
        ]

    assert versions is not None

    has_required_version = any(
        version == str(REQUIRED_GZ_SIM_MAJOR)
        or version.startswith(f"{REQUIRED_GZ_SIM_MAJOR}.")
        for version in versions
    )

    if not has_required_version:
        detected_versions = (
            ", ".join(versions)
            if versions
            else "(none)"
        )

        return [
            Shutdown(
                reason=(
                    f"Gazebo Sim {REQUIRED_GZ_SIM_MAJOR} is required. "
                    f"Detected versions: {detected_versions}."
                )
            )
        ]

    # ------------------------------------------------------------------
    # Resolve and validate package resources
    # ------------------------------------------------------------------

    try:
        description_share = get_package_share_path(
            DESCRIPTION_PACKAGE
        )
    except PackageNotFoundError:
        return [
            Shutdown(
                reason=(
                    f"Required package '{DESCRIPTION_PACKAGE}' "
                    "was not found."
                )
            )
        ]

    # World names are expected to be simple filenames, not paths.
    if Path(world_name).name != world_name:
        return [
            Shutdown(
                reason=f"Invalid world name '{world_name}'."
            )
        ]

    world_file = (
        description_share
        / "worlds"
        / f"{world_name}.sdf"
    )

    if not world_file.is_file():
        return [
            Shutdown(
                reason=(
                    "Gazebo world file was not found: "
                    f"'{world_file}'."
                )
            )
        ]

    gz_config_dir = description_share / "gz-configs"

    default_server_config = (
        gz_config_dir
        / "server.config"
    )

    gui_config = (
        gz_config_dir
        / (
            "debug-gui.config"
            if verbose
            else "minimal-gui.config"
        )
    )

    server_config_is_overridden = bool(
        context.environment.get(
            "GZ_SIM_SERVER_CONFIG_PATH"
        )
    )

    if (
        not server_config_is_overridden
        and not default_server_config.is_file()
    ):
        return [
            Shutdown(
                reason=(
                    "Default Gazebo server configuration was not found: "
                    f"'{default_server_config}'."
                )
            )
        ]

    if not gui_config.is_file():
        return [
            Shutdown(
                reason=(
                    "Gazebo GUI configuration was not found: "
                    f"'{gui_config}'."
                )
            )
        ]

    sitl_plugin_dir = Path(
        f"/opt/eolab-sitl-{drone_name}/gz_plugins"
    )

    if not sitl_plugin_dir.is_dir():
        return [
            Shutdown(
                reason=(
                    "PX4 SITL Gazebo plugin directory was not found: "
                    f"'{sitl_plugin_dir}'."
                )
            )
        ]

    # ------------------------------------------------------------------
    # Gazebo environment
    # ------------------------------------------------------------------

    environment_actions = [
        # Preserve the original parent-of-share behavior.
        # This supports resource URIs containing the package directory
        # name, for example model://eolab_description/...
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            str(description_share.parent),
        ),

        AppendEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            str(sitl_plugin_dir),
        ),
    ]

    # Respect a value provided by the calling environment.
    if not server_config_is_overridden:
        environment_actions.append(
            SetEnvironmentVariable(
                "GZ_SIM_SERVER_CONFIG_PATH",
                str(default_server_config),
            )
        )

    # ------------------------------------------------------------------
    # Optional WSL graphics configuration
    # ------------------------------------------------------------------

    wsl_actions = []

    if Path(
        "/proc/sys/fs/binfmt_misc/WSLInterop"
    ).exists():
        wsl_actions.append(
            LogInfo(
                msg=(
                    "[Gazebo] WSL detected; applying the configured "
                    "D3D12/NVIDIA Mesa defaults."
                )
            )
        )

        # Do not overwrite explicit user settings.
        if not context.environment.get("GALLIUM_DRIVER"):
            wsl_actions.append(
                SetEnvironmentVariable(
                    "GALLIUM_DRIVER",
                    "d3d12",
                )
            )

        if not context.environment.get(
            "MESA_D3D12_DEFAULT_ADAPTER_NAME"
        ):
            wsl_actions.append(
                SetEnvironmentVariable(
                    "MESA_D3D12_DEFAULT_ADAPTER_NAME",
                    "NVIDIA",
                )
            )

    # ------------------------------------------------------------------
    # Gazebo server
    # ------------------------------------------------------------------

    server_arguments = [
        "-r",
        "-s",
        str(world_file),
    ]

    if verbose:
        server_arguments.extend([
            "-v",
            "4",
        ])

    # ros_gz_sim expects gz_args as one command-line string.
    gz_args_server = " ".join(
        shlex.quote(argument)
        for argument in server_arguments
    )

    start_gz_server = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("ros_gz_sim"),
            "launch",
            "gz_sim.launch.py",
        ]),
        launch_arguments={
            "gz_args": gz_args_server,
            "gz_version": str(REQUIRED_GZ_SIM_MAJOR),
            "on_exit_shutdown": "True",
        }.items(),
    )

    # ------------------------------------------------------------------
    # Gazebo GUI
    # ------------------------------------------------------------------

    gz_args_client = [
        gz_executable,
        "sim",
        "-g",
        "--force-version",
        str(REQUIRED_GZ_SIM_MAJOR),
        "-v",
        "4" if verbose else "1",
        "--gui-config",
        str(gui_config),
    ]

    start_gz_gui = ExecuteProcess(
        name="gz_gui",
        cmd=gz_args_client,
        output="both",
    )

    # ------------------------------------------------------------------
    # ROS <-> Gazebo bridges
    # ------------------------------------------------------------------

    bridge_gz_topics = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="world_bridges",
        namespace=alias,
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            (
                f"/world/{world_name}/control"
                "@ros_gz_interfaces/srv/ControlWorld"
            ),
        ],
        output="screen",
    )

    # ------------------------------------------------------------------
    # Gazebo world readiness
    # ------------------------------------------------------------------

    wait_gz_ready = Node(
        package=SIM_BRINGUP_PACKAGE,
        executable="wait_gz_ready",
        name="wait_gz_ready",
        parameters=[{
            "timeout_s": ParameterValue(
                LaunchConfiguration(
                    "gz_ready_timeout_s"
                ),
                value_type=float,
            ),
            "world_name": world_name,
        }],
        output="screen",
    )

    # ------------------------------------------------------------------
    # Runtime spherical coordinates
    # ------------------------------------------------------------------

    set_gps_coordinates = ExecuteProcess(
        name="set_gps_coordinates",
        cmd=[
            gz_executable,
            "service",
            "-s",
            (
                f"/world/{world_name}"
                "/set_spherical_coordinates"
            ),
            "--reqtype",
            "gz.msgs.SphericalCoordinates",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            (
                "surface_model: EARTH_WGS84, "
                f"latitude_deg: {latitude}, "
                f"longitude_deg: {longitude}, "
                f"elevation: {altitude}"
            ),
        ],
        output="screen",
    )

    # ------------------------------------------------------------------
    # Ready -> GPS setup + GUI sequence
    # ------------------------------------------------------------------

    def handle_gz_ready_exit(
        event: ProcessExited,
        _context: LaunchContext,
    ):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg=(
                        "[Gazebo] World ready; setting spherical "
                        "coordinates and starting the GUI."
                    )
                ),
                set_gps_coordinates,
                start_gz_gui,
            ]

        return [
            LogInfo(
                msg=(
                    "[ERROR] Gazebo world readiness check failed "
                    f"with exit code {event.returncode}."
                )
            ),
            Shutdown(
                reason=(
                    "Gazebo world did not become ready; the GUI and "
                    "drone system will not be started."
                )
            ),
        ]

    on_gz_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_gz_ready,
            on_exit=handle_gz_ready_exit,
        )
    )

    return [
        # These actions must run before any Gazebo process starts.
        *wsl_actions,
        *environment_actions,

        # Register the event handler before its observed process.
        on_gz_ready,

        start_gz_server,
        bridge_gz_topics,
        wait_gz_ready,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "drone",
            description=(
                "Drone model used to select the PX4 SITL "
                "Gazebo plugins."
            ),
        ),
        DeclareLaunchArgument(
            "alias",
            default_value=LaunchConfiguration("drone"),
            description=(
                "ROS namespace used for simulation processes."
            ),
        ),
        DeclareLaunchArgument(
            "world",
            default_value="empty",
            description=(
                "World filename from eolab_description/worlds, "
                "without the .sdf extension."
            ),
        ),
        DeclareLaunchArgument(
            "lat",
            default_value="51.497741558866004",
            description="WGS84 latitude.",
        ),
        DeclareLaunchArgument(
            "lon",
            default_value="6.549182534441797",
            description="WGS84 longitude.",
        ),
        DeclareLaunchArgument(
            "alt",
            default_value="26.54",
            description="WGS84 elevation in metres.",
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="false",
            choices=["true", "false"],
            description=(
                "Enable verbose Gazebo server and GUI output."
            ),
        ),
        DeclareLaunchArgument(
            "gz_ready_timeout_s",
            default_value=DEFAULT_GZ_READY_TIMEOUT_S,
            description=(
                "Gazebo world readiness timeout in seconds."
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
