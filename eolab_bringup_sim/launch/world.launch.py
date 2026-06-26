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
DEFAULT_GZ_READY_TIMEOUT_S = "20"


def _shutdown(reason: str):
    return [EmitEvent(event=Shutdown(reason=reason))]


def _has_nvidia_gpu() -> bool:
    """Return whether an NVIDIA GPU is visible through nvidia-smi."""
    nvidia_smi = shutil.which("nvidia-smi")
    if nvidia_smi is None:
        wsl_nvidia_smi = Path("/usr/lib/wsl/lib/nvidia-smi")
        if wsl_nvidia_smi.is_file():
            nvidia_smi = str(wsl_nvidia_smi)

    if nvidia_smi is None:
        return False

    try:
        result = subprocess.run(
            [nvidia_smi, "-L"],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False

    return result.returncode == 0 and bool(result.stdout.strip())


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
    alias = LaunchConfiguration("alias")

    headless = (LaunchConfiguration("headless").perform(context) == "true")
    real_time_factor = float(LaunchConfiguration("real_time_factor").perform(context))
    verbose = (LaunchConfiguration("verbose").perform(context) == "true")

    gz_executable = shutil.which("gz")
    if gz_executable is None:
        return _shutdown(
            "Gazebo is not available: the 'gz' executable was not found "
            "in PATH. Gazebo Sim 8 is required."
        )

    versions, version_error = _query_gazebo_versions(gz_executable)
    if version_error is not None:
        return _shutdown(version_error)

    if versions is None:
        return _shutdown("Gazebo version detection returned no result.")

    has_required_version = any(
        version == str(REQUIRED_GZ_SIM_MAJOR)
        or version.startswith(f"{REQUIRED_GZ_SIM_MAJOR}.")
        for version in versions
    )


    if not has_required_version:
        detected_versions = ", ".join(versions) if versions else "(none)"
        return _shutdown(
            f"Gazebo Sim {REQUIRED_GZ_SIM_MAJOR} is required. "
            f"Detected versions: {detected_versions}."
        )

    try:
        description_share = get_package_share(DESCRIPTION_PACKAGE)
        world_file = get_package_file(
            DESCRIPTION_PACKAGE,
            "worlds",
            f"{world_name}.sdf",
        )
        ros_gz_sim_launch_file = get_launch_file(
            ROS_GZ_SIM_PACKAGE,
            "gz_sim.launch.py",
        )
        gui_config = None
        if not headless:
            gui_config = get_package_file(
                DESCRIPTION_PACKAGE,
                "gz-configs",
                "debug-gui.config" if verbose else "minimal-gui.config",
            )
    except RuntimeError as exc:
        return _shutdown(str(exc))

    default_server_config = description_share / "gz-configs" / "server.config"
    server_config_is_overridden = bool(
        context.environment.get("GZ_SIM_SERVER_CONFIG_PATH")
    )

    if (
        not server_config_is_overridden
        and not default_server_config.is_file()
    ):
        return _shutdown(
            "Default Gazebo server configuration was not found at "
            f"'{default_server_config}'."
        )

    sitl_plugin_dir = Path(f"/opt/eolab-sitl-{drone_name}/gz_plugins")
    if not sitl_plugin_dir.is_dir():
        return _shutdown(
            "PX4 SITL Gazebo plugin directory was not found at "
            f"'{sitl_plugin_dir}'."
        )

    environment_actions = [
        LogInfo(msg=
            f"Extending `GZ_SIM_RESOURCE_PATH` and `GZ_SIM_SYSTEM_PLUGIN_PATH`"
            f" with {DESCRIPTION_PACKAGE} specific assets."
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            str(description_share.parent),
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            str(sitl_plugin_dir),
        ),
    ]

    if not server_config_is_overridden:
        environment_actions.append(
            SetEnvironmentVariable(
                "GZ_SIM_SERVER_CONFIG_PATH",
                str(default_server_config),
            )
        )

    wsl_actions = []
    if Path("/proc/sys/fs/binfmt_misc/WSLInterop").exists():
        has_nvidia_gpu = _has_nvidia_gpu()
        wsl_actions.append(
            LogInfo(
                msg=(
                    "[Gazebo] WSL detected."
                )
            )
        )
        wsl_actions.append(
            LogInfo(
                msg=(
                    "[Gazebo] NVIDIA GPU detected in WSL."
                    "Trying to set `GALLIUM_DRIVER` and `MESA_D3D12_DEFAULT_ADAPTER_NAME`"
                    "to improve performance with GPU rendering."
                    if has_nvidia_gpu
                    else "[Gazebo] NVIDIA GPU not detected."
                )
            )
        )

        if not context.environment.get("GALLIUM_DRIVER"):
            wsl_actions.append(
                SetEnvironmentVariable("GALLIUM_DRIVER", "d3d12")
            )

        # Note: avoid overwriting user defined environment variables
        if (has_nvidia_gpu and not context.environment.get("MESA_D3D12_DEFAULT_ADAPTER_NAME")):
            wsl_actions.append(
                SetEnvironmentVariable(
                    "MESA_D3D12_DEFAULT_ADAPTER_NAME",
                    "NVIDIA",
                )
            )

    server_arguments = ["-r", "-s", str(world_file)]
    if verbose:
        server_arguments.extend(["-v", "4"])

    gz_args_server = " ".join(
        shlex.quote(argument)
        for argument in server_arguments
    )

    start_gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_sim_launch_file)),
        launch_arguments={
            "gz_args": gz_args_server,
            "gz_version": str(REQUIRED_GZ_SIM_MAJOR),
            "on_exit_shutdown": "true",
        }.items(),
    )

    start_gz_gui = None
    if not headless:
        assert gui_config is not None
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

    wait_gz_ready = Node(
        package=SIM_BRINGUP_PACKAGE,
        executable="wait_gz_ready",
        name="wait_gz_ready",
        parameters=[{
            "timeout_s": LaunchConfiguration("gz_ready_timeout_s"),
            "world_name": world_name,
        }],
        output="screen",
    )

    set_gps_coordinates = ExecuteProcess(
        name="set_gps_coordinates",
        cmd=[
            gz_executable,
            "service",
            "-s",
            f"/world/{world_name}/set_spherical_coordinates",
            "--reqtype",
            "gz.msgs.SphericalCoordinates",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            (
                "surface_model: EARTH_WGS84, "
                f"latitude_deg: {LaunchConfiguration('lat').perform(context)}, "
                f"longitude_deg: {LaunchConfiguration('lon').perform(context)}, "
                f"elevation: {LaunchConfiguration('alt').perform(context)}"
            ),
        ],
        output="screen",
    )

    set_physics = ExecuteProcess(
        name="set_physics",
        cmd=[
            gz_executable,
            "service",
            "-s",
            f"/world/{world_name}/set_physics/blocking",
            "--reqtype",
            "gz.msgs.Physics",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            (
                "max_step_size: 0.01, "
                f"real_time_factor: {real_time_factor}, "
                "gravity: {x: 0.0, y: 0.0, z: -9.80665}"
            ),
        ],
        output="screen",
    )

    def handle_gz_ready_exit(
        event: ProcessExited,
        _context: LaunchContext,
    ):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg=(
                        "[Gazebo] World ready; setting spherical "
                        "coordinates."
                    )
                ),
                set_gps_coordinates,
                set_physics
            ]


        return [
            LogInfo(
                msg=(
                    "[ERROR] Gazebo world readiness check failed "
                    f"with exit code {event.returncode}."
                )
            ),
            *_shutdown(
                "Gazebo world did not become ready; dependent simulation "
                "processes will not be started."
            ),
        ]

    def handle_gps_coordinates_exit(
        event: ProcessExited,
        _context: LaunchContext,
    ):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "[ERROR] Spherical-coordinate setup failed "
                        f"with exit code {event.returncode}."
                    )
                ),
                *_shutdown(
                    "Gazebo spherical-coordinate setup failed; dependent "
                    "simulation processes will not be started."
                ),
            ]

        if start_gz_gui is not None:
            return [
                LogInfo(
                    msg=(
                        "[Gazebo] Spherical coordinates set; starting "
                        "the GUI."
                    )
                ),
                start_gz_gui,
            ]

        return [
            LogInfo(
                msg=(
                    "[Gazebo] Spherical coordinates set; headless mode "
                    "is active."
                )
            )
        ]

    on_gz_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_gz_ready,
            on_exit=handle_gz_ready_exit,

        )
    )
    on_gps_coordinates = RegisterEventHandler(
        OnProcessExit(
            target_action=set_gps_coordinates,
            on_exit=handle_gps_coordinates_exit,
        )
    )

    return [
        *wsl_actions,
        *environment_actions,
        on_gz_ready,
        on_gps_coordinates,
        LogInfo(
            msg=(
                "[Gazebo] Starting in "
                + ("headless" if headless else "GUI")
                + " mode."
            )
        ),
        start_gz_server,
        bridge_gz_topics,
        wait_gz_ready,
    ]



def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            name="drone",
            description=(
                "Drone model used to select the PX4 SITL Gazebo plugins."
            ),
        ),
        DeclareLaunchArgument(
            name="alias",
            default_value=LaunchConfiguration("drone"),
            description="ROS namespace used for simulation processes.",
        ),
        DeclareLaunchArgument(
            name="world",
            default_value=DEFAULT_WORLD,
            choices=get_worlds(),
            description=(
                "World filename from eolab_description/worlds, without "
                "the '.sdf' extension."
            ),
        ),
        DeclareLaunchArgument(
            name="headless",
            default_value="false",
            choices=BOOLEAN_CHOICES,
            description="Run the simulator without a GUI.",
        ),
        DeclareLaunchArgument(
            name="real_time_factor",
            default_value="1.0",
            description="Target simulation speed relative to wall time."
        ),
        DeclareLaunchArgument(
            name="gz_ready_timeout_s",
            default_value=DEFAULT_GZ_READY_TIMEOUT_S,
            description="Gazebo world readiness timeout in seconds.",
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
            description="Enable verbose Gazebo server and GUI output.",
        ),
        OpaqueFunction(function=launch_setup),
    ])

