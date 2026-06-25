from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction
)
from launch.substitutions import LaunchConfiguration

def launch_setup(context):

    verbose: str = LaunchConfiguration("verbose").perform(context)

    # note: the /dev/ttyTHS1 correspont to the UART pins
    # on the Jetson Nano
    #
    # in a different platform, e.g. raspberry pi it may be a different dev node.
    agent_cmd = ["MicroXRCEAgent", "serial", "--dev", "/dev/ttyTHS1", "--baudrate", "115200"]

    if (verbose == "true"):
        agent_cmd.extend(["-v", "4"])
    else:
        agent_cmd.extend(["-v", "1"])

    start_agent = ExecuteProcess(
        cmd=agent_cmd,
        output="both"
    )

    return [
        start_agent
    ]

def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument("verbose")
    )

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
