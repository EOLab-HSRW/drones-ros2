from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import ProcessExited
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


SIM_BRINGUP_PACKAGE = "eolab_bringup_sim"
COMMON_BRINGUP_PACKAGE = "eolab_bringup"

DEFAULT_GZ_READY_TIMEOUT_S = "20.0"


def generate_launch_description() -> LaunchDescription:
    drone = LaunchConfiguration("drone")
    alias = LaunchConfiguration("alias")
    instance = LaunchConfiguration("instance")

    world = LaunchConfiguration("world")
    skip_world = LaunchConfiguration("skip_world")
    verbose = LaunchConfiguration("verbose")
    rviz = LaunchConfiguration("rviz")

    start_gz_world = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("eolab_bringup_sim"),
            "launch",
            "world.launch.py",
            ]),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "drone": LaunchConfiguration("drone"),
            "alias": LaunchConfiguration("alias"),

            "lat": LaunchConfiguration("lat"),
            "lon": LaunchConfiguration("lon"),
            "alt": LaunchConfiguration("alt"),
            "verbose": LaunchConfiguration("verbose"),
            }.items(),
        condition=UnlessCondition(
            LaunchConfiguration("skip_world")
            ),
    )


    # Contract:
    #   exit code 0     -> Gazebo server and GUI are ready
    #   nonzero code    -> timeout or readiness failure
    wait_gz_full_ready = Node(
        package=SIM_BRINGUP_PACKAGE,
        executable="wait_gz_gui_ready",
        name="wait_gz_gui_ready",
        parameters=[{
            "timeout_s": ParameterValue(
                LaunchConfiguration("gz_ready_timeout_s"),
                value_type=float,
            ),
        }],
        output="screen",
    )

    drone_spawn = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("eolab_bringup_sim"),
            "launch",
            "drone_spawn.launch.py",
        ]),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "drone": LaunchConfiguration("drone"),
            "alias": LaunchConfiguration("alias"),
            "instance": LaunchConfiguration("instance"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "verbose": LaunchConfiguration("verbose"),
            "camera_follow": LaunchConfiguration("camera_follow"),
        }.items(),
    )

    start_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(rviz),
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare(COMMON_BRINGUP_PACKAGE),
                "rviz",
                "default.rviz",
            ]),
            "-t",
            "EOLab System - RViz2",
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
                    msg="[Gazebo] Ready; spawning the drone system."
                ),
                drone_spawn,
            ]

        return [
            LogInfo(
                msg=(
                    "[ERROR] Gazebo readiness check failed with exit "
                    f"code {event.returncode}; the drone will not be spawned."
                )
            ),
            EmitEvent(
                event=Shutdown(
                    reason=(
                        "Gazebo server and GUI did not become ready."
                    )
                )
            ),
        ]

    on_gz_full_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_gz_full_ready,
            on_exit=handle_gz_ready_exit,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "drone",
            description="Drone model to start in simulation.",
        ),
        DeclareLaunchArgument(
            "alias",
            description="Runtime alias and ROS namespace for the drone.",
        ),
        DeclareLaunchArgument(
            "instance",
            description="Simulation instance number.",
        ),
        DeclareLaunchArgument(
            "x",
            description="Drone spawn X position.",
        ),
        DeclareLaunchArgument(
            "y",
            description="Drone spawn Y position.",
        ),
        DeclareLaunchArgument(
            "z",
            description="Drone spawn Z position.",
        ),

        # Preserved for compatibility with the current parent interface.
        # They are not consumed by this launch file yet.
        DeclareLaunchArgument(
            "lon",
            description="WGS84 longitude.",
        ),
        DeclareLaunchArgument(
            "lat",
            description="WGS84 latitude.",
        ),
        DeclareLaunchArgument(
            "alt",
            description="WGS84 altitude.",
        ),

        DeclareLaunchArgument(
            "verbose",
            choices=["true", "false"],
            description="Enable verbose simulation output.",
        ),
        DeclareLaunchArgument(
            "world",
            description="Gazebo world name without the file extension.",
        ),
        DeclareLaunchArgument(
            "skip_world",
            default_value="false",
            choices=["true", "false"],
            description=(
                "Do not start Gazebo from this launch file. The readiness "
                "checker will still wait for an externally started Gazebo."
            ),
        ),
        DeclareLaunchArgument(
            "camera_follow",
            default_value="true",
            choices=["true", "false"],
            description=(
                "Make the Gazebo GUI camera follow the spawned drone."
            ),
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            choices=["true", "false"],
            description="Start RViz.",
        ),
        DeclareLaunchArgument(
            "gz_ready_timeout_s",
            default_value=DEFAULT_GZ_READY_TIMEOUT_S,
            description="Gazebo readiness timeout in seconds.",
        ),

        # Register the handler before starting the process it observes.
        on_gz_full_ready,

        # Start Gazebo first unless an external instance is requested.
        start_gz_world,

        # Poll until Gazebo is ready or the timeout expires.
        wait_gz_full_ready,

        # RViz is independent from Gazebo readiness. It is stopped
        # automatically if launch shuts down after a readiness failure.
        start_rviz,
    ])
