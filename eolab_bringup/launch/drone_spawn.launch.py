import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import eolab_drones


def launch_setup(context):

    """
    TODO:
    - add mechanism to launch sim or physical
    """

    drone_name = LaunchConfiguration("drone").perform(context)
    drone_alias = LaunchConfiguration("alias").perform(context)
    instance = LaunchConfiguration("instance").perform(context)
    frame_id = str(eolab_drones.get_id(drone_name))

    robot_desc_content = xacro.process_file(
        PathJoinSubstitution(
            [FindPackageShare("eolab_description"), "drones", f"{drone_name}.urdf.xacro"]
        ).perform(context)
    ).toxml().replace("\n", "") # IMPOTANT TO FLAT the string to remove newline characters

    start_px4 = ExecuteProcess(
        name=f"{drone_alias}-{instance}",
        cmd=[f"{eolab_drones.get_stil_bin(drone_name)}", "-i", instance],
        output="both",
        additional_env={
            "PX4_GZ_STANDALONE": "1",
            "PX4_UXRCE_DDS_NS": f"{drone_alias}",
            "PX4_SYS_AUTOSTART": frame_id,
            "PX4_SIMULATOR": "gz",
            "PX4_GZ_WORLD": LaunchConfiguration("world"),
            "PX4_GZ_MODEL_NAME": f"{drone_alias}",
        }
    )

    spawn_drone = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", f"{drone_alias}",
            "-string", robot_desc_content,
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            # "-world", LaunchConfiguration("world") # TODO: check if required
        ],
        output='both'
    )

    return [
        start_px4,
        spawn_drone,
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        name="world",
        description="Name of the world in simulation. Required for the bridge to attach to the correct world."
    ))

    ld.add_action(DeclareLaunchArgument(
        name="drone",
        choices=list(eolab_drones.get_drones().keys()),
        description="Name of the drone from EOLab catalog."
    ))

    ld.add_action(DeclareLaunchArgument(
        name="alias",
        description="Drone alias (must be unique). Useful to identify the drone when there are multiple instances of the same drone running."
    ))

    ld.add_action(DeclareLaunchArgument(
        name="instance",
        description="Set the parameter 'SYS_MAV_ID'. Useful when are multiple instances."
    ))

    ld.add_action(DeclareLaunchArgument(
        name="x",
        description="X spawn position."
    ))

    ld.add_action(DeclareLaunchArgument(
        name="y",
        description="Y spawn position."
    ))

    ld.add_action(DeclareLaunchArgument(
        name="z",
        description="Z spawn position."
    ))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld


