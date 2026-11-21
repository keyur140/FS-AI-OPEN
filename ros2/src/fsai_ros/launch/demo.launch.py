# fsai-open: launch the VCU emulator, the CAN bridge and the scripted Acceleration demo together
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bus = LaunchConfiguration("bus")
    result = LaunchConfiguration("result")
    driver = Node(package="fsai_ros", executable="demo_driver", output="screen",
                  parameters=[{"result": result}])
    return LaunchDescription([
        DeclareLaunchArgument("bus", default_value="sim"),
        DeclareLaunchArgument("result", default_value="/tmp/fsai_demo_result.json"),
        Node(package="fsai_ros", executable="vcu_emulator", output="screen", parameters=[{"bus": bus}]),
        Node(package="fsai_ros", executable="can_bridge", output="screen", parameters=[{"bus": bus}]),
        driver,
        RegisterEventHandler(OnProcessExit(target_action=driver, on_exit=[EmitEvent(event=Shutdown())])),
    ])
