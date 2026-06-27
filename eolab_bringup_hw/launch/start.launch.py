from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction
)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

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
        output="both",
        additional_env={
            # Note: this overwrite only works IF the MicroXRCEClient,
            # the one running in the flight controller
            # is configure with `UXRCE_DDS_DOM_ID` set to `255`
            # See for official docs for client:
            #   https://micro-xrce-dds.docs.eprosima.com/en/latest/agent.html?highlight=domain#domain-id
            # See for PX4 client available parameters:
            #   https://docs.px4.io/main/en/middleware/uxrce_dds#starting-the-client
            #
            # Extra Note: It looks like the ROS_DOMAIN_ID env var
            # is not applicable to the MicroXRCEAgent.
            "XRCE_DOMAIN_ID_OVERRIDE": EnvironmentVariable("ROS_DOMAIN_ID", default_value="0"),
        },
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
