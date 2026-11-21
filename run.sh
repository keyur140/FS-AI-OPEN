#!/usr/bin/env bash
# fsai-open: one command to run anything - demo, tests, verify, or the real car
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
set -euo pipefail
cd "$(dirname "$0")"
IMAGE=fsai-open:0.2.0
docker image inspect "$IMAGE" >/dev/null 2>&1 || { echo "Run ./install.sh first."; exit 1; }
cmd="${1:-help}"; shift || true
case "$cmd" in
  demo)
    # A whole driverless event on the desk, in real time: VCU emulator + CAN bridge + pretend track and
    # sensors + the driving stack, as separate ROS 2 nodes talking over a virtual CAN bus.
    #   ./run.sh demo                 autocross, seed 0 (about 1.5 minutes)
    #   ./run.sh demo trackdrive 3    any event (acceleration | skidpad | autocross | trackdrive) and seed
    #   ./run.sh demo inspection_a    FS-AI 2026 inspection missions: inspection_a | inspection_b | demo
    #   ./run.sh demo autocross 5 1   third number 1 = trackside clutter (barriers, people, grass, shimmer)
    # Your own runs go to local_runs/, so the shipped, fingerprinted record in runs/ stays verifiable.
    EVENT="${1:-autocross}"; SEED="${2:-0}"; CLUTTER="${3:-0}"
    # two desk runs share the same virtual CAN bus (UDP multicast) and would answer each other's frames
    if [ -n "$(docker ps -q --filter "ancestor=$IMAGE")" ]; then
      echo "Another fsai-open run is still active (docker ps). Stop it first: docker rm -f \$(docker ps -q --filter ancestor=$IMAGE)"; exit 1
    fi
    mkdir -p local_runs/runs/ros2_autonomy
    docker run --rm -v "$PWD/local_runs/runs/ros2_autonomy:/out" "$IMAGE" sim event:="$EVENT" seed:="$SEED" \
      clutter:="${CLUTTER}.0" result:=/out/result_${EVENT}_${SEED}.json replay:=/out/replay_${EVENT}_${SEED}.html
    echo; echo "Result: local_runs/runs/ros2_autonomy/result_${EVENT}_${SEED}.json"
    echo "Replay (open in any browser): local_runs/runs/ros2_autonomy/replay_${EVENT}_${SEED}.html" ;;
  bridge-demo)
    # Stage 1 demo: virtual car + bridge + a scripted 75 m Acceleration run (no driving stack).
    mkdir -p local_runs/runs/ros2_demo
    docker run --rm -v "$PWD/local_runs/runs/ros2_demo:/out" "$IMAGE" demo result:=/out/result.json "$@"
    echo; echo "Verdict saved to local_runs/runs/ros2_demo/result.json" ;;
  test)
    # Every test and scenario; results go to local_runs/runs and local_runs/reports on your computer.
    mkdir -p local_runs/runs local_runs/reports
    docker run --rm -v "$PWD/local_runs/runs:/opt/fsai-open/runs" -v "$PWD/local_runs/reports:/opt/fsai-open/reports" \
      "$IMAGE" test --no-hash "$@" ;;
  verify)
    # Check every fingerprint and re-run every scenario byte for byte, using the files on your computer.
    docker run --rm -v "$PWD:/check:ro" -w /check -e PYTHONPATH=/check "$IMAGE" python3 /check/verify.py ;;
  vcan)
    # Linux desk set-up on real SocketCAN: needs sudo once to create vcan0 on the host.
    if ! ip link show vcan0 >/dev/null 2>&1; then
      sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
    fi
    docker run --rm --network host "$IMAGE" demo bus:=vcan0 "$@" ;;
  car)
    # REAL CAR. CAN bridge + driving stack (no emulator, no pretend world). Default interface can0 at
    # 500 kbit/s, set up on the host first:  sudo ip link set can0 type can bitrate 500000 && sudo ip link set up can0
    # Your sensors must publish /lidar/points, /fsai/camera/cones (or /fsai/perception/cones) and /imu/data.
    #   ./run.sh car can0            bridge + driving stack
    #   ./run.sh car can0 bridge     the bridge alone (your own driving software)
    IFACE="${1:-can0}"; MODE="${2:-autonomy}"
    echo "Starting on $IFACE for the REAL ADS-DV ($MODE). Follow docs/TEAM_SETUP_GUIDE.md section 5 first."
    if [ "$MODE" = bridge ]; then docker run --rm -it --network host "$IMAGE" car bus:="$IFACE"
    else docker run --rm -it --network host "$IMAGE" car-autonomy bus:="$IFACE"; fi ;;
  shell)
    docker run --rm -it --network host "$IMAGE" shell ;;
  *)
    echo "Usage: ./run.sh demo [event] [seed] | bridge-demo | test | verify | vcan | car [can0] [bridge] | shell"
    echo "  demo         a whole driverless event on the desk (default: autocross, seed 0) + replay page"
    echo "  bridge-demo  stage 1 demo: scripted 75 m run through the bridge and emulator only"
    echo "  test         every automatic test, scenario and event -> local_runs/reports/RESULTS.md"
    echo "  verify       check SHA-256 fingerprints and re-run scenarios byte for byte"
    echo "  vcan         bridge demo over Linux virtual CAN (vcan0)"
    echo "  car          the real car: bridge + driving stack (add 'bridge' for the bridge alone)" ;;
esac
