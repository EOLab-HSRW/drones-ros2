from typing import Optional, TypedDict

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class DroneConfig(TypedDict):
    """Configuration for one vehicle in the fleet."""
    alias: str
    instance: int
    x: float
    y: float
    z: float


# =============================================================================
# STEP 1 — Configure the fleet
# =============================================================================
#
# This example intentionally uses one drone model for the complete fleet.
#
# Your current world.launch.py selects one PX4 Gazebo plugin directory using
# the "drone" argument. Therefore, a homogeneous fleet is the safest setup
# until world.launch.py is extended to register plugins for multiple models.
#
DRONE_MODEL = "protoflyer"
WORLD = "aruco"
VERBOSE = "true"


# All PX4 SITL instances connect to one shared Micro XRCE-DDS Agent.
UXRCE_DDS_PORT = "8888"


# wait_gz_gui_ready must report success only when both the Gazebo server and
# GUI are available.
GZ_GUI_READY_TIMEOUT_S = 30.0


# -----------------------------------------------------------------------------
# Gazebo GUI camera follow
# -----------------------------------------------------------------------------
#
# Camera follow is disabled by default for a multi-drone launch.
#
# The Gazebo GUI has one active camera target. Enabling follow independently
# for every drone would cause multiple commands to compete for that target.
#
FOLLOW_ALIAS: Optional[str] = None

# To follow exactly one drone, replace the line above with:
#
# FOLLOW_ALIAS: Optional[str] = "harley"
#
# Only that drone will receive:
#
#     camera_follow := true
#
# Every other drone receives:
#
#     camera_follow := false


# -----------------------------------------------------------------------------
# Fleet definition
# -----------------------------------------------------------------------------
#
# Requirements:
#
# - Every alias must be unique.
# - Every PX4 instance number must be unique.
# - Instance numbers must be non-negative.
# - Spawn positions should be separated to avoid collisions.
#
# PX4 instance numbering normally starts at zero.
#
FLEET: list[DroneConfig] = [
    {
        "alias": "harley",
        "instance": 0,
        "x": -3.0,
        "y": 0.0,
        "z": 0.4,
    },
    {
        "alias": "jayaram",
        "instance": 1,
        "x": 0.0,
        "y": 0.0,
        "z": 0.4,
    },
    {
        "alias": "naofal",
        "instance": 2,
        "x": 3.0,
        "y": 0.0,
        "z": 0.4,
    },
]


def validate_fleet() -> None:
    """
    Validate the static tutorial configuration before launch starts.

    Configuration errors should fail immediately rather than creating a
    partially started simulation.
    """
    if not FLEET:
        raise RuntimeError(
            "FLEET must contain at least one drone."
        )

    aliases = [
        vehicle["alias"]
        for vehicle in FLEET
    ]

    instances = [
        vehicle["instance"]
        for vehicle in FLEET
    ]

    if len(aliases) != len(set(aliases)):
        raise RuntimeError(
            "Every drone alias must be unique. "
            f"Configured aliases: {aliases}"
        )

    if len(instances) != len(set(instances)):
        raise RuntimeError(
            "Every PX4 instance number must be unique. "
            f"Configured instances: {instances}"
        )

    if any(instance < 0 for instance in instances):
        raise RuntimeError(
            "PX4 instance numbers must be non-negative. "
            f"Configured instances: {instances}"
        )

    if (
        FOLLOW_ALIAS is not None
        and FOLLOW_ALIAS not in aliases
    ):
        raise RuntimeError(
            f"FOLLOW_ALIAS '{FOLLOW_ALIAS}' is not present "
            "in the configured fleet."
        )


def common_single_launch(
    vehicle: DroneConfig,
) -> IncludeLaunchDescription:
    """
    Create the common ROS 2 nodes for one drone.

    common_single.launch.py is expected to:

    - Publish /<alias>/robot_description.
    - Start robot_state_publisher in the alias namespace.
    - Start px4_odom_tf.
    - Start odom_viz.
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("eolab_bringup"),
                "launch",
                "common_single.launch.py",
            ])
        ),
        launch_arguments={
            "drone": DRONE_MODEL,
            "alias": vehicle["alias"],

            # This launcher is simulation-only.
            "use_sim_time": "true",
        }.items(),
    )


def drone_spawn_launch(
    vehicle: DroneConfig,
) -> IncludeLaunchDescription:
    """
    Create one deferred drone_spawn.launch.py include.

    The action is constructed here, but it is not added directly to the
    top-level LaunchDescription. It is returned by the Gazebo-ready callback.
    """
    camera_follow = (
        "true"
        if FOLLOW_ALIAS == vehicle["alias"]
        else "false"
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("eolab_bringup_sim"),
                "launch",
                "drone_spawn.launch.py",
            ])
        ),
        launch_arguments={
            "world": WORLD,
            "drone": DRONE_MODEL,
            "alias": vehicle["alias"],
            "instance": str(vehicle["instance"]),
            "x": str(vehicle["x"]),
            "y": str(vehicle["y"]),
            "z": str(vehicle["z"]),
            "verbose": VERBOSE,

            # The fleet launcher starts one shared agent.
            #
            # Starting one agent per drone on the same UDP port would cause
            # a port conflict.
            "start_agent": "false",
            "uxrce_dds_port": UXRCE_DDS_PORT,

            # Enable only if the selected drone model exposes the exact
            # camera topic paths configured in drone_spawn.launch.py.
            "bridge_camera": "false",

            # Disabled for all drones by default.
            #
            # When FOLLOW_ALIAS is configured, only the matching drone gets
            # camera_follow=true.
            "camera_follow": camera_follow,
        }.items(),
    )


def generate_launch_description() -> LaunchDescription:
    validate_fleet()

    # =========================================================================
    # STEP 2 — Create the common ROS 2 nodes for every drone
    # =========================================================================
    #
    # These nodes may start before Gazebo. Nodes using simulation time will
    # begin receiving time once the /clock bridge becomes available.
    #
    common_drone_actions = [
        common_single_launch(vehicle)
        for vehicle in FLEET
    ]

    # =========================================================================
    # STEP 3 — Start one shared Micro XRCE-DDS Agent
    # =========================================================================
    #
    # All PX4 instances use the same UDP agent.
    #
    # Separation is provided by:
    #
    # - A unique PX4 instance number.
    # - A unique DDS client key generated by PX4.
    # - PX4_UXRCE_DDS_NS, which drone_spawn.launch.py sets to the alias.
    #
    start_shared_agent = ExecuteProcess(
        name="fleet_micro_xrce_dds_agent",
        cmd=[
            "MicroXRCEAgent",
            "udp4",
            "-p",
            UXRCE_DDS_PORT,
            "-v",
            "4" if VERBOSE == "true" else "1",
        ],
        output="both",
    )

    # =========================================================================
    # STEP 4 — Start exactly one Gazebo world
    # =========================================================================
    #
    # world.launch.py owns:
    #
    # - Gazebo server startup.
    # - Gazebo GUI startup.
    # - Gazebo version checking.
    # - Gazebo resource and plugin paths.
    # - GPS spherical coordinates.
    # - World-level ROS <-> Gazebo bridges.
    #
    # Do not include world.launch.py once per drone.
    #
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("eolab_bringup_sim"),
                "launch",
                "world.launch.py",
            ])
        ),
        launch_arguments={
            "drone": DRONE_MODEL,

            # This alias is only used to namespace world-level ROS nodes.
            # It is not a drone entity name.
            "alias": "fleet",

            "world": WORLD,
            "verbose": VERBOSE,

            # Optional site coordinates can also be forwarded here:
            #
            # "lat": "51.497741558866004",
            # "lon": "6.549182534441797",
            # "alt": "26.54",
        }.items(),
    )

    # =========================================================================
    # STEP 5 — Wait until Gazebo is fully ready
    # =========================================================================
    #
    # This waiter is intentionally outside world.launch.py.
    #
    # world.launch.py is responsible for bringing up the world. This waiter
    # provides a fleet-level synchronization point before any drone is spawned.
    #
    # Do not replace this with a fixed TimerAction:
    #
    # - A slow machine may need more time.
    # - A fast machine should not wait unnecessarily.
    # - A timer cannot distinguish success from failure.
    #
    wait_gz_full_ready = Node(
        package="eolab_bringup_sim",
        executable="wait_gz_gui_ready",
        name="wait_gz_gui_ready_for_fleet",
        parameters=[{
            "timeout_s": GZ_GUI_READY_TIMEOUT_S,
        }],
        output="screen",
    )

    # =========================================================================
    # STEP 6 — Prepare all drone spawn actions
    # =========================================================================
    #
    # These actions are not started yet. They will be returned by the
    # readiness event callback only after Gazebo reports success.
    #
    spawn_actions = [
        drone_spawn_launch(vehicle)
        for vehicle in FLEET
    ]

    def handle_gz_ready_exit(
        event: ProcessExited,
        _context: LaunchContext,
    ):
        """
        Start the fleet only when wait_gz_gui_ready exits successfully.

        Required waiter contract:

            return code 0      Gazebo is ready
            nonzero code       timeout or failure
        """
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "[ERROR] Gazebo did not become fully ready. "
                        "The fleet will not be spawned. "
                        f"Readiness exit code: {event.returncode}."
                    )
                ),
                Shutdown(
                    reason=(
                        "Gazebo readiness failed; multi-drone "
                        "startup has been cancelled."
                    )
                ),
            ]

        return [
            LogInfo(
                msg=(
                    f"[Gazebo] Ready; spawning {len(FLEET)} drones "
                    f"in world '{WORLD}'."
                )
            ),

            # Each drone_spawn.launch.py:
            #
            # 1. Spawns the Gazebo entity.
            # 2. Checks the create process result.
            # 3. Starts its PX4 SITL process.
            # 4. Optionally starts camera bridges.
            # 5. Optionally enables GUI camera follow.
            #
            *spawn_actions,
        ]

    # ROS 2 Humble OnProcessExit accepts a callback with the ProcessExited
    # event and LaunchContext. ProcessExited exposes the process return code.
    on_gz_full_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_gz_full_ready,
            on_exit=handle_gz_ready_exit,
        )
    )

    return LaunchDescription([
        # Event handlers must be registered before the process they observe
        # can terminate.
        on_gz_full_ready,

        # The shared agent and common ROS nodes can initialize while Gazebo
        # starts.
        start_shared_agent,
        *common_drone_actions,

        # One Gazebo world for the complete fleet.
        start_world,

        # Drone spawning is released only after this process reports success.
        wait_gz_full_ready,
    ])
