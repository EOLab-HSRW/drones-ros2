from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

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
            name="world",
            default_value="empty",
            description="Name of the world to launch (without file extension). Only the ones in our worlds folder."
        )
    )

    return declared_args


def launch_setup(context):

    world_path = PathJoinSubstitution([
        FindPackageShare("eolab_description"),
        "worlds",
        f"{LaunchConfiguration('world').perform(context)}.sdf"
    ])

    drone_name = LaunchConfiguration("drone").perform(context)

    robot_desc_content = xacro.process_file(
        PathJoinSubstitution(
            [FindPackageShare("eolab_description"), "xacros", f"{drone_name}.urdf.xacro"]
        ).perform(context)
    ).toxml().replace("\n", "") # IMPOTANT TO FLAT the string

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

    # BUG report (harley): ros_gz_sim `create` in humble does not work with 
    # gazebo 8 due to msg type
    spawn_drone = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "phoenix",
            "-topic", "/robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.2",
            "-world", "empty"
        ],
        output='both'
    )

    # gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/path/to/model.urdf", name: "urdf_model"'
    # see: https://gazebosim.org/docs/latest/spawn_urdf/
    spawn = ExecuteProcess(
        name="spawn",
        cmd=["gz", "service", "-s", "/world/empty/create", "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", f"sdf: '{robot_desc_content}', name: '{drone_name}'"], # working (gazebo 8) with urdf content
        # cmd=["gz", "service", "-s", "/world/empty/create", "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean", "--timeout", "10000", "--req", f"sdf_filename: '{path_to_urdf_file}', name: 'urdf_model'"], # working (gazebo 8) with path
        # cmd=["gz", "service", "-s", "/world/empty/create", "--reqtype", "ignition.msgs.EntityFactory", "--reptype", "ignition.msgs.Boolean", "--timeout", "10000", "--req", f"sdf: '{robot_desc_content}', name: 'phoenix'"], # not working
        output="both"
    )

    return [
        gz,
        spawn
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_args))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
