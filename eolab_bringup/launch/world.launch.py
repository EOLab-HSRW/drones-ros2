from os import environ, path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
import eolab_drones

def launch_setup(context):


    # TODO harley: this is just a quick fix to export
    # GZ_SIM_RESOURCE_PATH to point to the `assets` folder of this package.
    # this is due to problems with the export tag `gazebo_ros` inside the package.xml in eolab_description
    # the export function to GZ_SIM_RESOURCE_PATH was introduced in 0.244.14
    # but current debian pkg (2025 May) is version 0.244.12 only export to IGN_SIM_RESOURCE_PATH making gazebo 8
    # not able to find the resources
    export_assets = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        value=(environ.get("GZ_SIM_RESOURCE_PATH", default="") + ":" + PathJoinSubstitution([FindPackageShare("eolab_description"), ".."]).perform(context))
    )

    # TODO get the path to the SITL dynamically based on the drones name
    # OR register plugin during build on PX4 side
    export_plugins = SetEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=(environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", default="") + ":" + f"{eolab_drones.get_build_dir(LaunchConfiguration('drone').perform(context))}/src/modules/simulation/gz_plugins")
    )

    export_server_config = SetEnvironmentVariable(
        "GZ_SIM_SERVER_CONFIG_PATH",
        value=(environ.get("GZ_SIM_SERVER_CONFIG_PATH", default=PathJoinSubstitution([FindPackageShare("eolab_description"), "server.config"]).perform(context)))
    )

    gz_args = ["-r ", PathJoinSubstitution([FindPackageShare("eolab_description"), "worlds", f"{LaunchConfiguration('world').perform(context)}.sdf"])]
    if LaunchConfiguration("verbose").perform(context) == "true":
        gz_args.append(" -v")

    is_wsl = "true" if path.exists("/proc/sys/fs/binfmt_misc/WSLInterop") else "false"

    wsl_env = GroupAction(
        actions=[
            LogInfo(msg="It seems that you are in WSL"),
            SetEnvironmentVariable("GALLIUM_DRIVER", "d3d12"),
            SetEnvironmentVariable("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA"),
        ],
        condition=IfCondition(is_wsl)
    )

    set_gps_coord = ExecuteProcess(
        name="set_gps_coord",
        cmd=[
            "gz", "service", "-s", f"/world/{LaunchConfiguration('world').perform(context)}/set_spherical_coordinates",
            "--reqtype", "gz.msgs.SphericalCoordinates",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "5000",
            "--req", f"surface_model: EARTH_WGS84, latitude_deg: {LaunchConfiguration('lat').perform(context)}, longitude_deg: {LaunchConfiguration('lon').perform(context)}, elevation: {LaunchConfiguration('alt').perform(context)}",
            # "--render-engine", "ogre"
        ]
    )

    gz = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("ros_gz_sim"),
            "launch",
            "gz_sim.launch.py"
        ]),
        launch_arguments={
            "gz_args": gz_args,
            "gz_version": "8",
            "on_exit_shutdown": "True",
        }.items(),
    )

    return [
        wsl_env,
        export_assets,
        export_plugins,
        export_server_config,
        set_gps_coord,
        gz
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            name="drone",
            description="Name of the drone. (TEMPORAL USE) to load gz plugins"
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="world",
            default_value="empty",
            description="World file in the `world` folder in eolab_description."
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="lat",
            default_value="51.497741558866004",
            description="GPS Coordinate Latitude in WGS84"
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="lon",
            default_value="6.549182534441797",
            description="GPS Coordinate Longitude in WGS84"
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="alt",
            default_value="26.54",
            description="GPS Coordinate Altitude in WGS84"
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="verbose",
            default_value="false",
            description="Gazebo Verbose ouput"
        )
    )

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
