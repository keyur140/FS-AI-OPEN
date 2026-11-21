#!/bin/bash
# fsai-open: container entry point - one word chooses what to run
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
set -e
source /opt/ros/humble/setup.bash
source /opt/fsai-open/ros2/install/setup.bash
# fsai_ros/package.xml deliberately carries no maintainer email (author's rule: no email anywhere).
# ROS tooling then installs it as a plain Python package, so add it to the ament prefix path by hand.
export AMENT_PREFIX_PATH="/opt/fsai-open/ros2/install/fsai_ros:${AMENT_PREFIX_PATH}"
cd /opt/fsai-open
case "${1:-demo}" in
  demo)     shift || true; exec ros2 launch fsai_ros demo.launch.py "$@" ;;
  sim)      shift || true; exec ros2 launch fsai_ros autonomy_sim.launch.py "$@" ;;
  car)      shift || true; exec ros2 launch fsai_ros car.launch.py "$@" ;;
  car-autonomy) shift || true; exec ros2 launch fsai_ros car_autonomy.launch.py "$@" ;;
  bridge)   shift || true; exec ros2 run fsai_ros can_bridge --ros-args "$@" ;;
  emulator) shift || true; exec ros2 run fsai_ros vcu_emulator --ros-args "$@" ;;
  test)     shift || true; exec python3 run_all.py --in-container "$@" ;;
  verify)   exec python3 verify.py ;;
  shell)    exec bash ;;
  *)        exec "$@" ;;
esac
