from os import environ
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, GroupAction
from launch_testing.event_handlers import StdoutReadyListener
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import xacro
import eolab_drones

def launch_args(context):

    declared_args = []

    declared_args.append(
        DeclareLaunchArgument(
            name="drone",
            default_value="protoflyer",
            choices=list(eolab_drones.get_catalog().keys()),
            description="Name of the drone to launch"
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            name="x",
            default_value="0.0",
            description="X position to spawn the drone in sim"
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            name="y",
            default_value="0.0",
            description="Y position to spawn the drone in sim"
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            name="z",
            default_value="0.0",
            description="Z position to spawn the drone in sim"
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            name="lat",
            default_value="51.497741558866004",
            description="GPS Coordinate Latitude in WGS84"
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            name="lon",
            default_value="6.549182534441797",
            description="GPS Coordinate Longitude in WGS84"
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            name="alt",
            default_value="26.54",
            description="GPS Coordinate Altitude in WGS84"
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            name="world",
            default_value="empty",
            description="Name of the world to launch (without file extension). Only the ones under the `worlds` folder in the pkg `eolab_description`."
        )
    )

    return declared_args


def launch_setup(context):


    gazebo_world = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("eolab_bringup"), "launch", "start_world.launch.py"]),
        launch_arguments=[
            ("drone", LaunchConfiguration("drone"))
        ]
    )

    drone_name = LaunchConfiguration("drone").perform(context)
    frame_id = str(eolab_drones.get_id(drone_name))

    start_px4 = ExecuteProcess(
        name="px4_sitl",
        cmd=[f"{eolab_drones.get_stil_bin(drone_name)}", "-i", drone_name],
        output="both",
        additional_env={
            "SYSTEM": "gz",
            "PX4_GZ_STANDALONE": "1",
            "PX4_UXRCE_DDS_NS": f"{drone_name}1",
            "PX4_SYS_AUTOSTART": frame_id,
            "PX4_SIMULATOR": "gz",
            "PX4_GZ_WORLD": LaunchConfiguration("world"),
            "PX4_GZ_MODEL_NAME": f"{drone_name}1",
        }
    )


    robot_desc_content = xacro.process_file(
        PathJoinSubstitution(
            [FindPackageShare("eolab_description"), "drones", f"{drone_name}.urdf.xacro"]
        ).perform(context)
    ).toxml().replace("\n", "") # IMPOTANT TO FLAT the string to remove newline characters

    spawn_drone = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", f"{drone_name}1",
            "-string", robot_desc_content,
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            # "-world", LaunchConfiguration("world") # TODO: check if required
        ],
        output='both'
    )

    start_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="both"
    )

    wait_for_sitl_ready = RegisterEventHandler(
        StdoutReadyListener(
            target_action=start_px4,
            ready_txt="INFO  [init] Standalone PX4 launch, waiting for Gazebo",
            actions=[
                LogInfo(msg="Got SITL firmware running..."),
                gazebo_world,
                spawn_drone,
                start_agent
            ]
        )
    )


    return [
        start_px4,
        wait_for_sitl_ready,
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_args))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
