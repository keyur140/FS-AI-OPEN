# fsai-open: on the real ADS-DV - the CAN bridge plus the driving stack (no emulator, no pretend world)
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bus = LaunchConfiguration("bus")
    return LaunchDescription([
        DeclareLaunchArgument("bus", default_value="can0"),
        Node(package="fsai_ros", executable="can_bridge", output="screen", parameters=[{"bus": bus}]),
        Node(package="fsai_ros", executable="autonomy", output="screen"),
    ])
