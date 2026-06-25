import json
import shutil

import eolab_drones

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import ProcessExited
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from eolab_bringup.commons import (
    BOOLEAN_CHOICES,
    get_worlds,
)


DEFAULT_UXRCE_DDS_PORT = "8888"

# ros_gz_sim create reports successful server-side creation before the
# Gazebo GUI necessarily renders the new entity. This delay avoids racing
# the GUI camera-tracking plugin.
CAMERA_FOLLOW_DELAY_S = 10.0


def launch_setup(context: LaunchContext):
    """Validate and start one simulated drone."""
    world = LaunchConfiguration("world").perform(context)
    drone = LaunchConfiguration("drone").perform(context)
    alias = LaunchConfiguration("alias").perform(context)
    instance = LaunchConfiguration("instance").perform(context)

    verbose = (
        LaunchConfiguration("verbose")
        .perform(context)
        .strip()
        .lower()
        == "true"
    )

    start_agent_enabled = (
        LaunchConfiguration("start_agent")
        .perform(context)
        .strip()
        .lower()
        == "true"
    )

    bridge_camera_enabled = (
        LaunchConfiguration("bridge_camera")
        .perform(context)
        .strip()
        .lower()
        == "true"
    )

    camera_follow_enabled = (
        LaunchConfiguration("camera_follow")
        .perform(context)
        .strip()
        .lower()
        == "true"
    )

    uxrce_dds_port = (
        LaunchConfiguration("uxrce_dds_port")
        .perform(context)
    )

    position_x = LaunchConfiguration("x")
    position_y = LaunchConfiguration("y")
    position_z = LaunchConfiguration("z")

    # ------------------------------------------------------------------
    # Validate launch values
    # ------------------------------------------------------------------

    try:
        instance_number = int(instance)
    except ValueError:
        return [
            EmitEvent(
                event=Shutdown(
                    reason=(
                        f"Invalid PX4 instance '{instance}'. "
                        "The instance must be a non-negative integer."
                    )
                )
            )
        ]

    if instance_number < 0:
        return [
            EmitEvent(
                event=Shutdown(
                    reason=(
                        f"Invalid PX4 instance '{instance}'. "
                        "The instance must be non-negative."
                    )
                )
            )
        ]

    try:
        port_number = int(uxrce_dds_port)
    except ValueError:
        return [
            EmitEvent(
                event=Shutdown(
                    reason=(
                        f"Invalid uXRCE-DDS port '{uxrce_dds_port}'. "
                        "The port must be an integer."
                    )
                )
            )
        ]

    if not 1 <= port_number <= 65535:
        return [
            EmitEvent(
                event=Shutdown(
                    reason=(
                        f"Invalid uXRCE-DDS port '{uxrce_dds_port}'. "
                        "The port must be between 1 and 65535."
                    )
                )
            )
        ]

    # ------------------------------------------------------------------
    # Resolve required executables and drone metadata
    # ------------------------------------------------------------------

    sitl_executable_name = f"eolab-sitl-{drone}"
    sitl_executable = shutil.which(sitl_executable_name)

    if sitl_executable is None:
        return [
            EmitEvent(
                event=Shutdown(
                    reason=(
                        f"PX4 SITL executable '{sitl_executable_name}' "
                        "was not found in PATH."
                    )
                )
            )
        ]

    agent_executable = None

    if start_agent_enabled:
        agent_executable = shutil.which("MicroXRCEAgent")

        if agent_executable is None:
            return [
                EmitEvent(
                    event=Shutdown(
                        reason=(
                            "MicroXRCEAgent was requested, but its "
                            "executable was not found in PATH."
                        )
                    )
                )
            ]

    gz_executable = None

    if camera_follow_enabled:
        gz_executable = shutil.which("gz")

        if gz_executable is None:
            return [
                EmitEvent(
                    event=Shutdown(
                        reason=(
                            "Gazebo camera follow was requested, but the "
                            "'gz' executable was not found in PATH."
                        )
                    )
                )
            ]

    frame_id = str(eolab_drones.get_id(drone))

    # ------------------------------------------------------------------
    # Optional Micro XRCE-DDS Agent
    # ------------------------------------------------------------------

    start_agent = None

    if start_agent_enabled:
        # Validated above when start_agent_enabled is true.
        assert agent_executable is not None

        start_agent = ExecuteProcess(
            name="micro_xrce_dds_agent",
            cmd=[
                agent_executable,
                "udp4",
                "-p",
                uxrce_dds_port,
                "-v",
                "4" if verbose else "1",
            ],
            output="both",
        )

    # ------------------------------------------------------------------
    # Spawn the URDF published by robot_state_publisher
    # ------------------------------------------------------------------

    spawn_drone = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            alias,
            "-topic",
            f"/{alias}/robot_description",
            "-x",
            position_x,
            "-y",
            position_y,
            "-z",
            position_z,
            "-world",
            world,
        ],
        output="both",
    )

    # ------------------------------------------------------------------
    # PX4 SITL
    # ------------------------------------------------------------------

    start_px4 = ExecuteProcess(
        name=f"{alias}-{instance}",
        cmd=[
            sitl_executable,
            "-i",
            instance,
        ],
        additional_env={
            # Gazebo is already managed by world.launch.py.
            "PX4_GZ_STANDALONE": "1",

            # Prefix the PX4 ROS 2 interfaces with the drone alias.
            "PX4_UXRCE_DDS_NS": alias,
            "PX4_UXRCE_DDS_PORT": uxrce_dds_port,

            "PX4_SYS_AUTOSTART": frame_id,
            "PX4_SIMULATOR": "gz",
            "PX4_GZ_WORLD": world,

            # Connect PX4 to the Gazebo entity created above.
            "PX4_GZ_MODEL_NAME": alias,
        },
        output="both",
    )

    # ------------------------------------------------------------------
    # Optional camera sensor bridges
    # ------------------------------------------------------------------

    bridge_camera_topics = None

    if bridge_camera_enabled:
        gz_camera_image_topic = (
            f"/world/{world}/model/{alias}/link/base_link/"
            "sensor/rgb_camera_sensor/image"
        )

        gz_camera_info_topic = (
            f"/world/{world}/model/{alias}/link/base_link/"
            "sensor/rgb_camera_sensor/camera_info"
        )

        bridge_camera_topics = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="camera_bridges",
            namespace=alias,
            arguments=[
                (
                    f"{gz_camera_image_topic}"
                    "@sensor_msgs/msg/Image"
                    "[gz.msgs.Image"
                ),
                (
                    f"{gz_camera_info_topic}"
                    "@sensor_msgs/msg/CameraInfo"
                    "[gz.msgs.CameraInfo"
                ),
            ],
            remappings=[
                (
                    gz_camera_image_topic,
                    f"/{alias}/rgb_camera_sensor/image_raw",
                ),
                (
                    gz_camera_info_topic,
                    f"/{alias}/rgb_camera_sensor/camera_info",
                ),
            ],
            output="screen",
        )

    # ------------------------------------------------------------------
    # Optional Gazebo GUI camera follow
    # ------------------------------------------------------------------

    delayed_camera_follow = None

    if camera_follow_enabled:
        # Validated above when camera_follow_enabled is true.
        assert gz_executable is not None

        # The Gazebo entity is created with "-name alias", so the GUI must
        # follow the runtime alias rather than the catalog drone name.
        #
        # json.dumps() produces a correctly quoted and escaped protobuf
        # string value.
        camera_track_message = (
            "track_mode: FOLLOW, "
            f"follow_target: {{ name: {json.dumps(alias)} }}, "
            "follow_offset: { x: -5.0, y: 0.0, z: 2.0 }, "
            "follow_pgain: 3.0"
        )

        set_camera_follow = ExecuteProcess(
            name=f"camera_follow_{alias}",
            cmd=[
                gz_executable,
                "topic",
                "-t",
                "/gui/track",
                "-m",
                "gz.msgs.CameraTrack",
                "-p",
                camera_track_message,
            ],
            output="screen",
        )

        delayed_camera_follow = TimerAction(
            period=CAMERA_FOLLOW_DELAY_S,
            actions=[
                LogInfo(
                    msg=(
                        "[Gazebo] Enabling GUI camera follow for "
                        f"entity '{alias}'."
                    )
                ),
                set_camera_follow,
            ],
        )

    # ------------------------------------------------------------------
    # Spawn completion sequence
    # ------------------------------------------------------------------

    def handle_spawn_exit(
        event: ProcessExited,
        _context: LaunchContext,
    ):
        """Start drone processes only after successful entity creation."""
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        f"[ERROR] Failed to spawn drone '{alias}'. "
                        "ros_gz_sim create exited with code "
                        f"{event.returncode}."
                    )
                ),
                EmitEvent(
                    event=Shutdown(
                        reason=(
                            f"Drone '{alias}' could not be spawned; "
                            "PX4 SITL will not be started."
                        )
                    )
                ),
            ]

        actions = [
            LogInfo(
                msg=(
                    f"[Gazebo] Drone '{alias}' spawned successfully; "
                    "starting PX4 SITL and drone-specific bridges."
                )
            ),
            start_px4,
        ]

        if bridge_camera_topics is not None:
            actions.append(bridge_camera_topics)

        if delayed_camera_follow is not None:
            actions.append(delayed_camera_follow)

        return actions

    on_spawn_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_drone,
            on_exit=handle_spawn_exit,
        )
    )

    actions = [
        # Register before launching the process being observed.
        on_spawn_exit,
    ]

    # The agent can initialize while Gazebo creates the entity.
    if start_agent is not None:
        actions.append(start_agent)

    # PX4 and camera follow are triggered only after this succeeds.
    actions.append(spawn_drone)

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            name="world",
            choices=get_worlds(),
            description="Gazebo world containing the drone entity.",
        ),

        DeclareLaunchArgument(
            name="drone",
            choices=tuple(sorted(eolab_drones.get_drones().keys())),
            description="Drone model from the EOLab catalog.",
        ),

        DeclareLaunchArgument(
            name="alias",
            description=(
                "Unique Gazebo entity name, ROS namespace, and "
                "PX4 uXRCE-DDS namespace."
            ),
        ),

        DeclareLaunchArgument(
            name="instance",
            description="Non-negative PX4 SITL instance number.",
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
            name="verbose",
            default_value="false",
            choices=BOOLEAN_CHOICES,
            description="Enable verbose PX4 communication output.",
        ),

        DeclareLaunchArgument(
            name="start_agent",
            default_value="true",
            choices=BOOLEAN_CHOICES,
            description=(
                "Start a Micro XRCE-DDS Agent. Disable this when an "
                "agent is already managed by the system bringup."
            ),
        ),

        DeclareLaunchArgument(
            name="uxrce_dds_port",
            default_value=DEFAULT_UXRCE_DDS_PORT,
            description=(
                "UDP port shared by the PX4 uXRCE-DDS client and "
                "Micro XRCE-DDS Agent."
            ),
        ),

        DeclareLaunchArgument(
            name="bridge_camera",
            default_value="false",
            choices=BOOLEAN_CHOICES,
            description=(
                "Bridge the expected Gazebo RGB camera topics to ROS 2."
            ),
        ),

        DeclareLaunchArgument(
            name="camera_follow",
            default_value="true",
            choices=BOOLEAN_CHOICES,
            description=(
                "Make the Gazebo GUI camera follow this drone after "
                "the entity is spawned. Disable this in multi-drone "
                "setups, or enable it for only one selected drone."
            ),
        ),

        OpaqueFunction(function=launch_setup),
    ])
