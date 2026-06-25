import shlex
import shutil
import subprocess
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import ProcessExited
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from eolab_bringup.commons import (
    BOOLEAN_CHOICES,
    DEFAULT_WORLD,
    DESCRIPTION_PACKAGE,
    SIM_BRINGUP_PACKAGE,
    get_launch_file,
    get_package_file,
    get_package_share,
    get_worlds,
)


ROS_GZ_SIM_PACKAGE = "ros_gz_sim"

REQUIRED_GZ_SIM_MAJOR = 8
DEFAULT_GZ_READY_TIMEOUT_S = "20.0"


def _query_gazebo_versions(
    gz_executable: str,
) -> tuple[list[str] | None, str | None]:
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
    """Validate and launch one Gazebo world."""
    world_name = LaunchConfiguration("world").perform(context)
    drone_name = LaunchConfiguration("drone").perform(context)

    verbose = (
        LaunchConfiguration("verbose")
        .perform(context)
        .strip()
        .lower()
        == "true"
    )

    latitude = LaunchConfiguration("lat").perform(context)
    longitude = LaunchConfiguration("lon").perform(context)
    altitude = LaunchConfiguration("alt").perform(context)

    alias = LaunchConfiguration("alias")
    gz_ready_timeout = LaunchConfiguration("gz_ready_timeout_s")

    # ------------------------------------------------------------------
    # Gazebo installation and version checks
    # ------------------------------------------------------------------

    gz_executable = shutil.which("gz")

    if gz_executable is None:
        return [
            EmitEvent(
                event=Shutdown(
                    reason=(
                        "Gazebo is not available: the 'gz' executable was "
                        "not found in PATH. Gazebo Sim 8 is required."
                    )
                )
            )
        ]

    versions, version_error = _query_gazebo_versions(gz_executable)

    if version_error is not None:
        return [
            EmitEvent(
                event=Shutdown(reason=version_error)
            )
        ]

    if versions is None:
        return [
            EmitEvent(
                event=Shutdown(
                    reason=(
                        "Gazebo version detection returned no result."
                    )
                )
            )
        ]

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
            EmitEvent(
                event=Shutdown(
                    reason=(
                        f"Gazebo Sim {REQUIRED_GZ_SIM_MAJOR} is required. "
                        f"Detected versions: {detected_versions}."
                    )
                )
            )
        ]

    # ------------------------------------------------------------------
    # Resolve and validate installed package resources
    # ------------------------------------------------------------------

    try:
        description_share = get_package_share(
            DESCRIPTION_PACKAGE
        )

        world_file = get_package_file(
            DESCRIPTION_PACKAGE,
            "worlds",
            f"{world_name}.sdf",
        )

        gui_config = get_package_file(
            DESCRIPTION_PACKAGE,
            "gz-configs",
            (
                "debug-gui.config"
                if verbose
                else "minimal-gui.config"
            ),
        )

        ros_gz_sim_launch_file = get_launch_file(
            ROS_GZ_SIM_PACKAGE,
            "gz_sim.launch.py",
        )
    except RuntimeError as exc:
        return [
            EmitEvent(
                event=Shutdown(reason=str(exc))
            )
        ]

    default_server_config = (
        description_share
        / "gz-configs"
        / "server.config"
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
            EmitEvent(
                event=Shutdown(
                    reason=(
                        "Default Gazebo server configuration was not "
                        f"found at '{default_server_config}'."
                    )
                )
            )
        ]

    sitl_plugin_dir = Path(
        f"/opt/eolab-sitl-{drone_name}/gz_plugins"
    )

    if not sitl_plugin_dir.is_dir():
        return [
            EmitEvent(
                event=Shutdown(
                    reason=(
                        "PX4 SITL Gazebo plugin directory was not found "
                        f"at '{sitl_plugin_dir}'."
                    )
                )
            )
        ]

    # ------------------------------------------------------------------
    # Gazebo environment
    # ------------------------------------------------------------------

    environment_actions = [
        # Use the parent of the package share so resource URIs containing
        # the package directory remain resolvable, for example:
        # model://eolab_description/...
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            str(description_share.parent),
        ),

        AppendEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            str(sitl_plugin_dir),
        ),
    ]

    # Preserve an explicit value inherited from the calling environment.
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

    if Path("/proc/sys/fs/binfmt_misc/WSLInterop").exists():
        wsl_actions.append(
            LogInfo(
                msg=(
                    "[Gazebo] WSL detected; applying the configured "
                    "D3D12/NVIDIA Mesa defaults."
                )
            )
        )

        # Preserve explicitly configured user values.
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
        PythonLaunchDescriptionSource(
            str(ros_gz_sim_launch_file)
        ),
        launch_arguments={
            "gz_args": gz_args_server,
            "gz_version": str(REQUIRED_GZ_SIM_MAJOR),
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ------------------------------------------------------------------
    # Gazebo GUI
    # ------------------------------------------------------------------

    start_gz_gui = ExecuteProcess(
        name="gz_gui",
        cmd=[
            gz_executable,
            "sim",
            "-g",
            "--force-version",
            str(REQUIRED_GZ_SIM_MAJOR),
            "-v",
            "4" if verbose else "1",
            "--gui-config",
            str(gui_config),
        ],
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
                gz_ready_timeout,
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
        """Start the post-readiness actions or terminate the launch."""
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
            EmitEvent(
                event=Shutdown(
                    reason=(
                        "Gazebo world did not become ready; the GUI and "
                        "drone system will not be started."
                    )
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
        # Environment changes must execute before Gazebo starts.
        *wsl_actions,
        *environment_actions,

        # Register the handler before starting the observed process.
        on_gz_ready,

        start_gz_server,
        bridge_gz_topics,
        wait_gz_ready,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            name="drone",
            description=(
                "Drone model used to select the PX4 SITL "
                "Gazebo plugins."
            ),
        ),

        DeclareLaunchArgument(
            name="alias",
            default_value=LaunchConfiguration("drone"),
            description=(
                "ROS namespace used for simulation processes."
            ),
        ),

        DeclareLaunchArgument(
            name="world",
            default_value=DEFAULT_WORLD,
            choices=get_worlds(),
            description=(
                "World filename from eolab_description/worlds, "
                "without the '.sdf' extension."
            ),
        ),

        DeclareLaunchArgument(
            name="lat",
            default_value="51.497741558866004",
            description="WGS84 latitude.",
        ),

        DeclareLaunchArgument(
            name="lon",
            default_value="6.549182534441797",
            description="WGS84 longitude.",
        ),

        DeclareLaunchArgument(
            name="alt",
            default_value="26.54",
            description="WGS84 elevation in metres.",
        ),

        DeclareLaunchArgument(
            name="verbose",
            default_value="false",
            choices=BOOLEAN_CHOICES,
            description=(
                "Enable verbose Gazebo server and GUI output."
            ),
        ),

        DeclareLaunchArgument(
            name="gz_ready_timeout_s",
            default_value=DEFAULT_GZ_READY_TIMEOUT_S,
            description=(
                "Gazebo world readiness timeout in seconds."
            ),
        ),

        OpaqueFunction(function=launch_setup),
    ])
