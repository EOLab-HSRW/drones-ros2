from os import environ
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch_testing.event_handlers import StdoutReadyListener
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import xacro

"""
Conceptual launch sequence:
1. Export assets (e.g. Models and worlds)
2. Launch PX4 SITL and wait until msg in stdout "INFO  [init] Standalone PX4 launch, waiting for Gazebo" to launch gazebo.
    - this help to minize launch error due to missing sitl firmware binary
2. Launch gazebo simulator (server and client)
3. spawn drone
"""

def launch_args(context):

    declared_args = []

    declared_args.append(
        DeclareLaunchArgument(
            name="drone",
            default_value="phoenix",
            description="Name of the drone to launch"
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            name="frame_id",
            default_value="22103",
            description="Drone Frame ID"
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            name="world",
            default_value="empty",
            description="Name of the world to launch (without file extension). Only the ones in our worlds folder."
        )
    )

    return declared_args


def launch_setup(context):

    # TODO harley: this is just a quick fix to export
    # GZ_SIM_RESOURCE_PATH to point to the `assets` folder of this package.
    # this is due to problems with the export tag `gazebo_ros` inside the package.xml
    # the export function to GZ_SIM_RESOURCE_PATH was introduced in 0.244.14
    # but the default version is 0.244.12 that export to IGN_SIM_RESOURCE_PATH
    path_to_pkg = PathJoinSubstitution([FindPackageShare('eolab_description'), '..']).perform(context)

    export_assets = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        value=(environ.get("GZ_SIM_RESOURCE_PATH", default="") + ":" + path_to_pkg)
    )

    drone_name = LaunchConfiguration("drone").perform(context)

    start_px4 = ExecuteProcess(
        name="px4_sitl",
        cmd=[f"/home/harley/eolab-git/drones-fw/PX4-Autopilot/build/px4_sitl_{drone_name}/bin/px4"],
        output="both",
        additional_env={
            "SYSTEM": "gz",
            "PX4_GZ_STANDALONE": "1",
            "PX4_UXRCE_DDS_NS": LaunchConfiguration("drone"), # namespace topics
            "PX4_SYS_AUTOSTART": LaunchConfiguration("frame_id"),
            "PX4_SIMULATOR": "gz",
            "PX4_GZ_WORLD": LaunchConfiguration("world"),
            "PX4_GZ_MODEL_NAME": LaunchConfiguration("drone")
        }
    )

    world_name = LaunchConfiguration("world").perform(context)

    world_path = PathJoinSubstitution([
        FindPackageShare("eolab_description"),
        "worlds",
        f"{world_name}.sdf"
    ])

    gz = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("ros_gz_sim"),
            "launch",
            "gz_sim.launch.py"
        ]),
        launch_arguments={
            "gz_args": ["-r ", world_path, " -v"],
            "gz_version": "8",
            "on_exit_shutdown": "True",
        }.items()
    )

    robot_desc_content = xacro.process_file(
        PathJoinSubstitution(
            [FindPackageShare("eolab_description"), "drones", f"{drone_name}.urdf.xacro"]
        ).perform(context)
    ).toxml().replace("\n", "") # IMPOTANT TO FLAT the string to remove newline characters

    # gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/path/to/model.urdf", name: "urdf_model"'
    # see: https://gazebosim.org/docs/latest/spawn_urdf/
    spawn_drone = ExecuteProcess(
        name="spawn_drone",
        cmd=["gz", "service", "-s", f"/world/{world_name}/create", "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", f"sdf: '{robot_desc_content}', name: '{drone_name}'"], # working (gazebo 8) with urdf content
        # cmd=["gz", "service", "-s", "/world/empty/create", "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean", "--timeout", "10000", "--req", f"sdf_filename: '{path_to_urdf_file}', name: 'urdf_model'"], # working (gazebo 8) with path
        # cmd=["gz", "service", "-s", "/world/empty/create", "--reqtype", "ignition.msgs.EntityFactory", "--reptype", "ignition.msgs.Boolean", "--timeout", "10000", "--req", f"sdf: '{robot_desc_content}', name: 'phoenix'"], # not working
        output="both"
    )

    # BUG report (harley): ros_gz_sim `create` in humble does not work with 
    # gazebo 8 due to msg type
    # spawn_drone = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     arguments=[
    #         "-name", "phoenix",
    #         "-topic", "/robot_description",
    #         "-x", "0.0",
    #         "-y", "0.0",
    #         "-z", "0.2",
    #         "-world", "empty"
    #     ],
    #     output='both'
    # )

    wait_for_sitl_ready = RegisterEventHandler(
        StdoutReadyListener(
            target_action=start_px4,
            ready_txt="INFO  [init] Standalone PX4 launch, waiting for Gazebo",
            actions=[
                LogInfo(msg="Got SITL firmware running..."),
                gz,
                spawn_drone
            ]
        )
    )

    return [
        export_assets,
        start_px4,
        wait_for_sitl_ready
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_args))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
