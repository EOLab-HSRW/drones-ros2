import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    GroupAction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnExecutionComplete

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import eolab_drones


def launch_setup(context):

    system = LaunchConfiguration("system").perform(context)
    world_name = LaunchConfiguration("world").perform(context)
    drone_name = LaunchConfiguration("drone").perform(context)
    drone_alias = LaunchConfiguration("alias").perform(context)
    instance = LaunchConfiguration("instance").perform(context)
    frame_id = str(eolab_drones.get_id(drone_name))

    is_gz = "true" if system == "gz" else "false"

    robot_desc_content = xacro.process_file(
        PathJoinSubstitution(
            [FindPackageShare("eolab_description"), "drones", f"{drone_name}.urdf.xacro"]
        ).perform(context),
        # mappings={
        #     "drone_name": drone_alias,
        # }
    ).toxml().replace("\n", "") # NOTE (IMPOTANT): It needs to be flat string to remove newline characters.


    start_px4 = ExecuteProcess(
        name=f"{drone_alias}-{instance}",
        cmd=[f"eolab-sitl-{drone_name}", "-i", instance],
        output="both",
        additional_env={
            "PX4_GZ_STANDALONE": "1",
            "PX4_UXRCE_DDS_NS": f"{drone_alias}",
            "PX4_SYS_AUTOSTART": frame_id,
            "PX4_SIMULATOR": "gz",
            "PX4_GZ_WORLD": world_name,
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
            "-world", world_name,
        ],
        output='both'
    )

    bridge_gz_topics = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            f'/world/{world_name}/model/{drone_alias}/link/base_link/sensor/rgb_camera_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/world/{world_name}/model/{drone_alias}/link/base_link/sensor/rgb_camera_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # f'/{LaunchConfiguration("robot_name").perform(context)}/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # f'/{LaunchConfiguration("robot_name").perform(context)}/lidar_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        # parameters=[
        #     {"use_sim_time": LaunchConfiguration("use_sim_time")}
        # ],
        remappings=[
            (f'/world/{world_name}/model/{drone_alias}/link/base_link/sensor/rgb_camera_sensor/image', f'/{drone_alias}/rgb_camera_sensor/image_raw'),
            (f'/world/{world_name}/model/{drone_alias}/link/base_link/sensor/rgb_camera_sensor/camera_info', f'/{drone_alias}/rgb_camera_sensor/camera_info')
        ],
        # NOTE (harley): the expansion of gz topic only works for 1 level higher
        # parameters=[
        #     {"expand_gz_topic_names": True}
        # ],
        output='screen'
    )

    set_follow_entity = ExecuteProcess(
        name="follow_entity",
        cmd=[
            "gz", "service", "-s", "/gui/follow",
            "--reqtype", "gz.msgs.StringMsg",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "8000",
            "--req", f'data: "{drone_name}"',
        ]
    )

    wait_gz_entity = Node(
        package="eolab_bringup",
        executable="wait_gz_entity",
        output="both",
        parameters=[
            {"entity_name": drone_name},
            {"world_name": world_name},
            {"timeout_s": 30.0}
        ]
    )

    on_gz_entity_ready = RegisterEventHandler(
        OnExecutionComplete(
            target_action = wait_gz_entity,
            on_completion = [
                set_follow_entity,
            ]
        )
    )

    spawn_in_gz = GroupAction(
        actions=[
            start_px4,
            spawn_drone,
            bridge_gz_topics,
            on_gz_entity_ready,
            wait_gz_entity,
        ],
        condition=IfCondition(is_gz)
    )

    robot_state_publiser_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc_content},
            {"use_sim_time": True} # TODO (harley): check of system type
        ]
    )

    return [
        spawn_in_gz,
        # robot_state_publiser_node
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            name="system",
        )
    )

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


