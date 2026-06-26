from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import ProcessExited
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from eolab_bringup.commons import (
    BOOLEAN_CHOICES,
    COMMON_BRINGUP_PACKAGE,
    SIM_BRINGUP_PACKAGE,
    get_launch_file,
    get_package_file,
    get_worlds,
)


DEFAULT_GZ_READY_TIMEOUT_S = "20.0"

def launch_setup(context: LaunchContext):
    drone = LaunchConfiguration("drone")
    alias = LaunchConfiguration("alias")
    instance = LaunchConfiguration("instance")

    position_x = LaunchConfiguration("x")
    position_y = LaunchConfiguration("y")
    position_z = LaunchConfiguration("z")

    latitude = LaunchConfiguration("lat")
    longitude = LaunchConfiguration("lon")
    altitude = LaunchConfiguration("alt")

    world = LaunchConfiguration("world")
    headless_config = LaunchConfiguration("headless")
    real_time_factor = LaunchConfiguration("real_time_factor")
    headless = headless_config.perform(context).strip().lower() == "true"
    camera_follow = LaunchConfiguration("camera_follow")
    verbose = LaunchConfiguration("verbose")
    rviz = LaunchConfiguration("rviz")

    gz_ready_timeout_s = LaunchConfiguration("gz_ready_timeout_s")

    world_launch_file = get_launch_file(
        SIM_BRINGUP_PACKAGE,
        "world.launch.py",
    )

    drone_spawn_launch_file = get_launch_file(
        SIM_BRINGUP_PACKAGE,
        "drone_spawn.launch.py",
    )

    rviz_config = get_package_file(
        COMMON_BRINGUP_PACKAGE,
        "rviz",
        "default.rviz",
    )

    start_gz_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(world_launch_file)),
        launch_arguments={
            "world": world,
            "drone": drone,
            "alias": alias,
            "lat": latitude,
            "lon": longitude,
            "alt": altitude,
            "headless": headless_config,
            "real_time_factor": real_time_factor,
            "verbose": verbose,
        }.items(),
    )

    # Headless mode only requires the Gazebo server. GUI mode waits for the
    # GUI camera topic, which is published once the GUI scene is initialized.
    if headless:
        readiness_target = "server"
        readiness_display = "Server"
        wait_gz_ready = Node(
            package=SIM_BRINGUP_PACKAGE,
            executable="wait_gz_ready",
            name="wait_gz_ready_for_spawn",
            parameters=[{
                "world_name": world,
                "timeout_s": ParameterValue(
                    gz_ready_timeout_s,
                    value_type=float,
                ),
            }],
            output="screen",
        )
    else:
        readiness_target = "GUI"
        readiness_display = "GUI"
        wait_gz_ready = Node(
            package=SIM_BRINGUP_PACKAGE,
            executable="wait_gz_gui_ready",
            name="wait_gz_gui_ready_for_spawn",
            parameters=[{
                "timeout_s": ParameterValue(
                    gz_ready_timeout_s,
                    value_type=float,
                ),
            }],
            output="screen",
        )

    drone_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(drone_spawn_launch_file)),
        launch_arguments={
            "world": world,
            "drone": drone,
            "alias": alias,
            "instance": instance,
            "x": position_x,
            "y": position_y,
            "z": position_z,
            "verbose": verbose,
            "headless": headless_config,
            "camera_follow": camera_follow,
        }.items(),
    )

    start_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(rviz),
        arguments=[
            "-d",
            str(rviz_config),
            "-t",
            "EOLab System - RViz2",
        ],
        output="screen",
    )

    def handle_gz_ready_exit(
        event: ProcessExited,
        _context: LaunchContext,
    ):
        """Spawn the drone only after Gazebo becomes ready."""
        if event.returncode == 0:
            return [
                LogInfo(
                    msg=(
                        f"[Gazebo] {readiness_display} ready; "
                        "spawning the drone system."
                    )
                ),
                drone_spawn,
            ]

        return [
            LogInfo(
                msg=(
                    "[ERROR] Gazebo readiness check failed with exit "
                    f"code {event.returncode}; the drone will not be "
                    "spawned."
                )
            ),
            EmitEvent(
                event=Shutdown(
                    reason=(
                        f"Gazebo {readiness_target} did not become ready."
                    )
                )
            ),
        ]

    on_gz_full_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_gz_ready,
            on_exit=handle_gz_ready_exit,
        )
    )

    return [
        on_gz_full_ready,
        start_gz_world,
        wait_gz_ready,
        start_rviz,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            name="drone",
            description="Drone model to start in simulation.",
        ),
        DeclareLaunchArgument(
            name="alias",
            description="Runtime alias and ROS namespace for the drone.",
        ),
        DeclareLaunchArgument(
            name="instance",
            description="Simulation instance number.",
        ),
        DeclareLaunchArgument(
            name="x",
            description="Drone spawn X position.",
        ),
        DeclareLaunchArgument(
            name="y",
            description="Drone spawn Y position.",
        ),
        DeclareLaunchArgument(
            name="z",
            description="Drone spawn Z position.",
        ),
        DeclareLaunchArgument(
            name="lon",
            description="WGS84 longitude.",
        ),
        DeclareLaunchArgument(
            name="lat",
            description="WGS84 latitude.",
        ),
        DeclareLaunchArgument(
            name="alt",
            description="WGS84 altitude.",
        ),
        DeclareLaunchArgument(
            name="verbose",
            choices=BOOLEAN_CHOICES,
            description="Enable verbose simulation output.",
        ),
        DeclareLaunchArgument(
            name="world",
            choices=get_worlds(),
            description="Gazebo world name without the file extension.",
        ),
        DeclareLaunchArgument(
            name="headless",
            default_value="false",
            choices=BOOLEAN_CHOICES,
            description="Run the simulator without a GUI"
        ),
        DeclareLaunchArgument(
            name="real_time_factor",
            default_value="1.0",
            description="Target simulation speed relative to wall time."
        ),
        DeclareLaunchArgument(
            name="camera_follow",
            default_value="true",
            choices=BOOLEAN_CHOICES,
            description=(
                "Make the Gazebo GUI camera follow the spawned drone."
            ),
        ),
        DeclareLaunchArgument(
            name="rviz",
            default_value="true",
            choices=BOOLEAN_CHOICES,
            description="Start RViz.",
        ),
        DeclareLaunchArgument(
            name="gz_ready_timeout_s",
            default_value=DEFAULT_GZ_READY_TIMEOUT_S,
            description="Gazebo readiness timeout in seconds.",
        ),
        OpaqueFunction(function=launch_setup)
    ])

