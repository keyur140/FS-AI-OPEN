# fsai-open: launch a whole driverless event on the desk - emulator, bridge, pretend world and the driving stack
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
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    a = {k: LaunchConfiguration(k) for k in ("bus", "event", "seed", "result", "replay", "clutter", "latency_comp", "debug")}
    lifted = PythonExpression(["'", a["event"], "'.startswith('inspection')"])   # IN2.2: car raised off the ground
    world = Node(package="fsai_ros", executable="sim_world", output="screen",
                 parameters=[{"event": a["event"], "seed": a["seed"], "result": a["result"], "replay": a["replay"],
                              "clutter": a["clutter"], "debug": a["debug"], "imu": False}])
    return LaunchDescription([
        DeclareLaunchArgument("bus", default_value="sim"),
        DeclareLaunchArgument("event", default_value="autocross"),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("result", default_value="/tmp/fsai_sim_result.json"),
        DeclareLaunchArgument("replay", default_value="/tmp/fsai_sim_replay.html"),
        DeclareLaunchArgument("clutter", default_value="0.0"),
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("latency_comp", default_value="true"),
        Node(package="fsai_ros", executable="vcu_emulator", output="screen",
             parameters=[{"bus": a["bus"], "lifted": lifted, "imu": True, "seed": a["seed"]}]),
        Node(package="fsai_ros", executable="can_bridge", output="screen", parameters=[{"bus": a["bus"]}]),
        Node(package="fsai_ros", executable="autonomy", output="screen",
             parameters=[{"latency_comp": PythonExpression(["'", a["latency_comp"], "'.lower() in ('1', 'true')"])}]),
        world,
        RegisterEventHandler(OnProcessExit(target_action=world, on_exit=[EmitEvent(event=Shutdown())])),
    ])
