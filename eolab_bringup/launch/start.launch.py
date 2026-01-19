import eolab_drones

from pathlib import Path

from launch import LaunchDescription
from launch.action import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    GroupAction
)

from launch.conditions import IfCondition
from launch_testing.event_handlers import StdoutReadyListener
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable
)

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

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

    system = LaunchConfiguration("system").perform(context)
    drone = LaunchConfiguration("drone").perform(context)
    alias = LaunchConfiguration("alias").perform(context)
    instance = LaunchConfiguration("instance").perform(context)

    is_gz = "true" if system == "gz" else "false"

    print(f"""
 _____ ___  _          _
| ____/ _ \| |    __ _| |__   System:   {system}
|  _|| | | | |   / _` | '_ \  Drone:    {drone}
| |__| |_| | |__| (_| | |_) | Alias:    {alias}
|_____\___/|_____\__,_|_.__/  Instance: {instance}

""")
    if (is_gz):
        print("\tRunning in simulation mode.\n")

    start_gz_world = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("eolab_bringup"), "launch", "world.launch.py"]),
        launch_arguments=[
            ("drone", drone), # we need this here to resolve the SITL plugins for this drone
            ("verbose", LaunchConfiguration("verbose")),
            ("system", system)
            ],
        condition=IfCondition(is_gz)
    )

    # TODO: get the `robot_state_publisher` out of "drone_spawn.launch.py"
    drone_spawn = IncludeLaunchDescription( 
        PathJoinSubstitution([FindPackageShare("eolab_bringup"), "launch", "drone_spawn.launch.py"]),
        launch_arguments=[
            ("system", system),
            ("world", LaunchConfiguration("world")),
            ("drone", drone),
            ("alias", alias),
            ("instance", instance),
            ("x", LaunchConfiguration("x")),
            ("y", LaunchConfiguration("y")),
            ("z", LaunchConfiguration("z")),
    ])

    # TODO: build the agent cmd depending if sim or physical
    agent_cmd = ["MicroXRCEAgent", "udp4", "-p", "8888"]

    if LaunchConfiguration("verbose").perform(context) == "true":
        agent_cmd.extend(["-v", "4"])
    else:
        agent_cmd.extend(["-v", "1"])

    start_agent = ExecuteProcess(
        cmd=agent_cmd,
        output="both"
    )

    start_rviz = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", PathJoinSubstitution([FindPackageShare("eolab_bringup"), "rviz", "default.rviz"])]
    )

    start_pose = Node(
        name="pose_node",
        namespace=LaunchConfiguration("alias"),
        package="eolab_utils",
        executable="pose",
        output="both"
    )

    return [
        start_gz_world,
        drone_spawn,
        start_agent,
        start_rviz,
        start_pose,
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        name="system",
        default_value=EnvironmentVariable("EOLAB_SYSTEM_TYPE", default_value="gz"),
        choices=["gz", "physical"],
        description="Select the components depending on this argument."
    ))

    ld.add_action(DeclareLaunchArgument(
        name="drone",
        default_value=EnvironmentVariable("EOLAB_DRONE_NAME", default_value="protoflyer"),
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
        description="X position to spawn the drone in sim (ignore in physical system)"
    ))

    ld.add_action(DeclareLaunchArgument(
        name="y",
        default_value="0.0",
        description="Y position to spawn the drone in sim (ignore in physical system)"
    ))

    ld.add_action(DeclareLaunchArgument(
        name="z",
        default_value="0.8",
        description="Z position to spawn the drone in sim (ignore in physical system)"
    ))

    ld.add_action(DeclareLaunchArgument(
        name="lat",
        default_value="51.497741558866004",
        description="GPS Coordinate Latitude in WGS84 (ignore in physical system)"
    ))

    ld.add_action(DeclareLaunchArgument(
        name="lon",
        default_value="6.549182534441797",
        description="GPS Coordinate Longitude in WGS84 (ignore in physical system)"
    ))

    ld.add_action(DeclareLaunchArgument(
        name="alt",
        default_value="26.54",
        description="GPS Coordinate Altitude in WGS84 (ignore in physical system)"
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

    ld.add_action(DeclareLaunchArgument(
        name="rviz",
        default_value="true",
        description="Start RVIZ."
    ))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
