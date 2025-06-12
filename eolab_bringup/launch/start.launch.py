import eolab_drones

from pathlib import Path

from launch import LaunchDescription
from launch.action import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler
from launch_testing.event_handlers import StdoutReadyListener
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def get_worlds():
    """
    Check the worlds directory under eolab_description/worlds
    and dynamically get the available world choices.
    """
    context = LaunchContext()

    worlds_dir = Path(
        PathJoinSubstitution([
            FindPackageShare("eolab_description"), "worlds"
        ]).perform(context)
    )

    return [world.stem for world in worlds_dir.glob("*.sdf")]


def launch_setup(context):


    gazebo_world = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("eolab_bringup"), "launch", "world.launch.py"]),
        launch_arguments=[
            ("drone", LaunchConfiguration("drone")), # we need this here to resolve the SITL plugins for this drone
            ("verbose", LaunchConfiguration("verbose"))
        ]
    )

    spawn_drone = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("eolab_bringup"), "launch", "drone_spawn.launch.py"]),
        launch_arguments=[
            ("world", LaunchConfiguration("world")),
            ("drone", LaunchConfiguration("drone")),
            ("alias", LaunchConfiguration("drone")),
            ("instance", LaunchConfiguration("instance")),
            ("x", LaunchConfiguration("x")),
            ("y", LaunchConfiguration("y")),
            ("z", LaunchConfiguration("z")),
        ]
    )

    agent_cmd = ["MicroXRCEAgent", "udp4", "-p", "8888"]

    if LaunchConfiguration("verbose").perform(context) == "true":
        agent_cmd.extend(["-v", "4"])
    else:
        agent_cmd.extend(["-v", "1"])

    start_agent = ExecuteProcess(
        cmd=agent_cmd,
        output="both"
    )

    # Note: Now that the logic to start the SITL is in a launch file
    # it is not possible to registed this RegisterEventHandler
    # wait_for_sitl_ready = RegisterEventHandler(
    #     StdoutReadyListener(
    #         target_action=spawn_drone,
    #         ready_txt="INFO  [init] Standalone PX4 launch, waiting for Gazebo",
    #         actions=[
    #             LogInfo(msg="Got SITL firmware running..."),
    #             gazebo_world,
    #             start_agent
    #         ]
    #     )
    # )


    return [
        spawn_drone,
        gazebo_world,
        start_agent,
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        name="drone",
        default_value="protoflyer",
        choices=list(eolab_drones.get_drones().keys()),
        description="Name of the drone to launch"
    ))

    ld.add_action(DeclareLaunchArgument(
        name="alias",
        default_value=LaunchConfiguration("drone"),
        description="Set a custom alias name different from the drone name"
    ))

    ld.add_action(DeclareLaunchArgument(
        name="instance",
        default_value="0",
        description="Drone instance."
    ))

    ld.add_action(DeclareLaunchArgument(
        name="x",
        default_value="0.0",
        description="X position to spawn the drone in sim"
    ))

    ld.add_action(DeclareLaunchArgument(
        name="y",
        default_value="0.0",
        description="Y position to spawn the drone in sim"
    ))

    ld.add_action(DeclareLaunchArgument(
        name="z",
        default_value="0.8",
        description="Z position to spawn the drone in sim"
    ))

    ld.add_action(DeclareLaunchArgument(
        name="lat",
        default_value="51.497741558866004",
        description="GPS Coordinate Latitude in WGS84"
    ))

    ld.add_action(DeclareLaunchArgument(
        name="lon",
        default_value="6.549182534441797",
        description="GPS Coordinate Longitude in WGS84"
    ))

    ld.add_action(DeclareLaunchArgument(
        name="alt",
        default_value="26.54",
        description="GPS Coordinate Altitude in WGS84"
    ))

    ld.add_action(DeclareLaunchArgument(
        name="world",
        default_value="empty",
        choices=get_worlds(),
        description="Name of the world to launch (without file extension). Only the ones under the `worlds` folder in the pkg `eolab_description`."
    ))

    ld.add_action(DeclareLaunchArgument(
        name="verbose",
        default_value="false",
        description="Verbose launch."
    ))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
