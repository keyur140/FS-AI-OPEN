# fsai-open: ROS 2 package setup for fsai_ros
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
from glob import glob

from setuptools import setup

setup(
    name="fsai_ros",
    version="0.2.0",
    packages=["fsai_ros"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/fsai_ros"]),
        ("share/fsai_ros", ["package.xml"]),
        ("share/fsai_ros/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KEYUR SHAILESHKUMAR PATEL",
    description="ROS 2 nodes for fsai-open",
    license="MIT",
    entry_points={"console_scripts": [
        "can_bridge = fsai_ros.can_bridge_node:main",
        "vcu_emulator = fsai_ros.vcu_emulator_node:main",
        "demo_driver = fsai_ros.demo_driver_node:main",
        "autonomy = fsai_ros.autonomy_node:main",
        "sim_world = fsai_ros.sim_world_node:main",
    ]},
)
