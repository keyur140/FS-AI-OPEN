# fsai-open: Team Setup Guide

**Author:** KEYUR SHAILESHKUMAR PATEL · ORCID https://orcid.org/0009-0008-5261-043X · MSc Artificial Intelligence student, Department of Computer Science, Brunel University London, UK; Independent researcher · *Conducted independently; not an official Brunel University London project.* · Drafted with an AI assistant (Claude, Anthropic); decisions by the author.

Written for an FS-AI team's software lead. Sections 1 to 4 need only a laptop. Section 5 is for the car, and section 6 is the list for the first day at a real track.

Version 0.2.0 has two layers you can use together or apart: the **safety layer** (CAN bridge and VCU emulator, stage 1) and the **driving stack** (cone detection, mapping, planning, control and the mission logic for every FS-AI 2026 event and inspection, stage 2).

---

## 1. Install (about 5 minutes, once)

You need Docker (Linux, macOS or Windows with Docker Desktop) and internet access once.

```bash
unzip fsai-open-0.2.0.zip && cd fsai-open
./install.sh                         # Windows PowerShell: .\run.ps1 install
```

The image is pinned: ROS 2 Humble on Ubuntu 22.04 (base image digest in `docker/Dockerfile`), with exact Python package versions (`requirements.txt`). During the build it downloads the official 2026 CAN database at a pinned commit and refuses it if its SHA-256 differs.

## 2. See it work (2 minutes)

```bash
./run.sh demo                       # one autocross lap on a seeded track, in real time
./run.sh demo acceleration 0        # any event and seed: acceleration | skidpad | autocross | trackdrive
./run.sh demo inspection_a          # inspection missions: inspection_a | inspection_b | demo
./run.sh demo autocross 5 1         # third number 1 = trackside clutter (barriers, people, grass, heat shimmer)
./run.sh bridge-demo                # stage 1 only: the bridge drives a scripted 75 m run, no driving stack
```

Each `demo` starts four ROS 2 nodes that talk over a virtual CAN bus: the pretend VCU, the CAN bridge, the pretend track with its sensors (LiDAR, camera, IMU), and the driving stack. A referee watches the car from outside the stack. You should see `AS_OFF → AS_READY → AS_DRIVING → AS_FINISHED` (or `AS_EMERGENCY_BRAKE` for Inspection B and the Autonomous Demo, which end with the EBS by rule). The result is written to `local_runs/runs/ros2_autonomy/result_<event>_<seed>.json`, with `"ok": true`, the lap times, cones hit and time off track. Open `replay_<event>_<seed>.html` next to it in any browser to watch the run from above.

Only one desk run can be active at a time, because runs share the virtual CAN bus; `./run.sh demo` refuses to start a second one.

Your own runs always go to `local_runs/`, so the shipped record in `runs/` and `reports/` stays verifiable with `./run.sh verify`.

## 3. Check it yourself

```bash
./run.sh test      # every test, scenario, event and stress run inside the image -> local_runs/reports/RESULTS.md
./run.sh verify    # every file fingerprint + byte-identical scenario re-runs
```

Read `tests/conformance/test_spec_v4.py` alongside the specification. Each test name starts with the clause it checks, for example `test_3_2_go_before_5s_is_ignored_and_must_be_cycled`. The driving stack tests are in `tests/autonomy/`; the closed-loop ones drive whole events against the emulator and the referee.

## 4. Plug in your own driving software

Run the bridge (plus the emulator for desk work), then talk to it from your stack:

| Direction | Name | Type | Meaning |
|---|---|---|---|
| your stack → bridge | `/fsai/cmd` | `ackermann_msgs/AckermannDriveStamped` | `speed` target [m/s]; `steering_angle` [rad, + = left, bicycle model]; `acceleration` > 0 limits acceleration, < 0 sets braking deceleration |
| your stack → bridge | `/fsai/fsg_logging` | `std_msgs/UInt16MultiArray` | `[lap, cones_actual, cones_all]` for the FSG logging fields |
| service | `/fsai/mission_finished` | `std_srvs/Trigger` | brake to a stop, then report FINISHED (only when every wheel reads ≤ 10 rpm) |
| service | `/fsai/ebs` | `std_srvs/Trigger` | request an emergency stop |
| bridge → you | `/fsai/state` | `std_msgs/String` | `AS_OFF`, `AS_READY`, `AS_DRIVING`, `AS_FINISHED`, `AS_EMERGENCY_BRAKE` |
| bridge → you | `/fsai/status` | `std_msgs/String` (JSON) | everything: faults, shutdown cause, live limits, what was sent, notes on anything changed, loop timing |
| bridge → you | `/fsai/wheel_speeds`, `/fsai/speed` | `Float32MultiArray`, `Float32` | wheel rpm [FL, FR, RL, RR]; speed [m/s] |
| bridge → you | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | OK / WARN / ERROR once per second |

Desk session with your own nodes (two terminals):

```bash
docker run --rm -it --network host fsai-open:0.2.0 emulator      # the pretend car
docker run --rm -it --network host fsai-open:0.2.0 bridge        # the bridge (bus "sim" by default)
# "flip the switches" on the pretend car (third terminal: ./run.sh shell):
ros2 topic pub --once /fsai_emulator/inputs std_msgs/String "{data: '{\"asms\": true, \"tsms\": true, \"ami\": 1}'}"
ros2 topic pub --once /fsai_emulator/inputs std_msgs/String "{data: '{\"res_go\": true}'}"     # after 5 s
```

**What the bridge guarantees, whatever your stack sends:**
- It sends all five AI frames every 10 ms, on its own thread with a monotonic clock, and echoes the handshake.
- It sends only zeros and NEUTRAL in AS_READY.
- It sends FORWARD in AS_DRIVING and never NEUTRAL while moving.
- It never sends torque and brake in the same cycle, and between braking and driving it leaves one 10 ms cycle with neither, because the VCU receives the drive frames before the brake frame.
- It sends FINISHED only at standstill.
- It clamps steering and torque only to the limits the VCU reports live. If those are not yet known, it holds them at zero and says so.
- If your command is older than 0.2 s, it brakes in a controlled way. This is not an e-stop, so a crashed node does not cost you a power cycle.
- It rejects NaN and infinity.
- It refuses any reserved CAN ID.

**Using the driving stack too.** `./run.sh car can0` starts the bridge and the driving stack together. The stack needs your sensor drivers to publish:

| Direction | Name | Type | Meaning |
|---|---|---|---|
| sensors → stack | `/lidar/points` | `sensor_msgs/PointCloud2` (x, y, z in the car frame: x forward, y left, metres, origin at the rear axle on the ground) | raw LiDAR points; the stack finds the cones itself |
| sensors → stack | `/fsai/camera/cones` | `PointCloud2` (x, y, z, colour, probability) | camera cone detections in the car frame; colour 0 blue, 1 yellow, 2 orange, 3 big orange |
| sensors → stack | `/fsai/perception/cones` | same as the camera | optional: your own fused cones, used instead of the built-in LiDAR detector |
| sensors → stack | `/imu/data` | `sensor_msgs/Imu` | yaw rate (z); wheel speeds come from the VCU through the bridge |
| stack → you | `/fsai/autonomy/status` | `String` (JSON) | phase, laps, pose, map size, `grip_ratio`, `steer_ratio`, `alerts`, EBS reason |
| stack → you | `/fsai/autonomy/path`, `/fsai/autonomy/map` | `nav_msgs/Path`, `PointCloud2` | the planned path and the cone map, for RViz |

Stamp every sensor message with the time the data was captured, on the same clock as the stack. The stack uses the stamps to undo sensor delay. A scan older than `perception.max_scan_age_s` (0.4 s) when it is processed is dropped, because an old scan used as new matches the wrong cones (we saw exactly this on the skidpad rings in the real-time runs).

**Is the stack keeping up?** Once a second the driving node prints a `health:` line: scans per second, how old the scans are when they arrive, how long each takes to process, how many cones matched the map, the pose, the learned tyre slip, grip and steering ratios, and the longest wait between IMU and control callbacks. On the car, scan ages should stay well under 0.2 s and the control gap near 10 ms. Perception, IMU and control run on separate threads, so a slow scan does not delay the steering.

**Own workspace?** Set `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1` for the driving node (the Docker image does this). The matrices are small, and without it each process starts a maths thread per core; we measured 280% CPU and 0.3 s stalls. The mission (acceleration, skidpad, autocross, trackdrive, inspections, demo) is read from the VCU's mission selector, so nothing else needs to be set. All tunable numbers are in `config/autonomy.yaml`, each with a comment.

**Not using ROS?** Use the Python API directly (`fsai.bridge.core.BridgeCore` with `fsai.bridge.runner`). `fsai/vcu/lockstep.py` shows the whole loop in about 40 lines.

**Have your own ROS workspace?** Copy `ros2/src/fsai_ros` into it and `pip install -r requirements.txt`. The package deliberately has no maintainer email (the author's rule), so ROS tooling installs it as a plain Python package. Either run `export AMENT_PREFIX_PATH=<install>/fsai_ros:$AMENT_PREFIX_PATH`, as `docker/entrypoint.sh` does, or add your team's own maintainer email in your fork.

## 5. On the real ADS-DV

> Follow the IMechE ADS-DV User Manual and Operational Safety documents. This section adds steps; it does not replace them.

**Hardware.** You need a SocketCAN adapter at 500 kbit/s (IMechE tests with PEAK) on the Harting plug. The compute power must fit within the shared 30 A supply.

```bash
sudo ip link set can0 type can bitrate 500000 && sudo ip link set up can0
./run.sh car can0                    # the bridge only; the emulator is never started in this mode
```

**First powered session: wheels off the ground (on the jack), RES in hand.**
1. Run `candump -l can0` on the host for the whole session. That log validates the emulator against the real VCU.
2. Check that `/fsai/status` shows `angle_max_deg` and `torque_cap_nm` filled in: the live limits were received.
3. Work through [ASSUMPTIONS.md](ASSUMPTIONS.md) items A1 to A9 and tick each one.
4. Rehearse: the 5 s AS_READY wait, the Go rising edge, a low-speed wheel spin, finishing with the service brake, a RES e-stop, then a full power cycle.
5. Correct anything that differs in `config/ads_dv.yaml`, run `./run.sh test` again, and keep the log with the run.

**Inspection missions before any dynamic event (FS-AI 2026 rules IN2.2, IN2.3).** Select the mission on the car and press Go; the software does the rest.
- *Static Inspection A* (car on stands): full steering sweep ending straight, drivetrain to 200 rpm in 10 s, stop within 5 s, AS Finished. Rehearse on the desk with `./run.sh demo inspection_a`.
- *Static Inspection B* (after a power cycle): spin up to 50 rpm, then the software triggers the EBS (AS Emergency). `./run.sh demo inspection_b`.
- *Autonomous Demo* (between cones like the acceleration lane): steering sweep left and right, drive at least 10 m, stop within 10 m, drive a further 10 m, EBS. `./run.sh demo demo`.
- The rules say "drivetrain rpm". We read it as the wheel rpm the VCU reports (`mission.rpm_basis: wheel` in `config/autonomy.yaml`). Ask the scrutineer on the day; if they mean motor rpm, set `rpm_basis: motor`.

**Sensor loss means EBS (rule T4.2.2).** If the LiDAR or perception goes quiet for 1 s, the IMU for 0.5 s, the wheel speeds for 0.5 s, or the car stops matching cones to its map for 2 s, the software asks for the EBS. The reason is written to `/fsai/autonomy/status` (`alerts`) and to the console. The limits are in the `watchdog:` section of `config/autonomy.yaml`.

## 6. Day 1 at a real track: sim vs asphalt

The desk simulator is cleaner than a real track. The software already has a knob for each thing that is different on real asphalt, and the stress runs in `runs/events/stress.json` show how much margin each knob has. Spend the first test day on this list, in this order, with the RES in hand and the car at walking pace first.

**1. Actuator delays (do this first, or the car weaves).**
- From the `candump` log, measure the time from a steering request on CAN to the measured angle starting to move. Do the same for brake request to pressure, and torque request to wheel speed change.
- Put the numbers in `config/ads_dv.yaml` (`steer_delay_s`, `brake_delay_s`, `drive_delay_s`). Set `control.latency_comp_s` in `config/autonomy.yaml` to about the steering delay plus 0.02 s.
- Check: drive a straight line at 5 m/s and a 10 m circle. If the car weaves left and right, raise `latency_comp_s` by 0.02 s, or raise `steer_smoothing`. If it is slow to turn in, lower them.
- In the simulator, three times the estimated delay still finishes clean with compensation on; the same delay without compensation is a separate stress row, so you can see what it costs.

**2. Tyre grip (cold tyres, track temperature, bumps).**
- Start with `speed.grip_factor: 0.8` in `config/autonomy.yaml` on cold tyres, then raise it in steps of 0.05 as the tyres warm up.
- The software measures understeer itself: it compares the yaw rate it gets with the yaw rate the steering should give. `grip_ratio` in `/fsai/autonomy/status` should stay near 1.0. If it falls below 0.85, the software slows the racing line down at the next lap line and writes an alert.
- The software also learns two things by itself: the rear tyre slip (in the mapping filter; without this, cold tyres smeared the simulated map with over 140 phantom cones) and the steering compliance (`steer_ratio` in the status; it asks for more steering if gentle corners come out wide). Write the settled values into `slam.rear_slip_gain` and `vehicle.steer_gain`, so the next run starts from them and the simulator matches the car; then re-run `./run.sh test`.

**3. Messy sensors (grass, barriers, people, heat shimmer).**
- Record a bag (`ros2 bag record /lidar/points /fsai/camera/cones /fsai/perception/cones`) with the car standing at the start, and look at `/fsai/autonomy/map` in RViz. Every cone in the map should be a real cone.
- The filters are in the `perception:` section: tall or wide blobs are rejected (`cone_max_height_m`, `cone_max_width_m`); near and low blobs are grass (`cone_min_height_near_m`); a blob with other raised points around it is grass, a barrier or a person (`isolation_m`); and while the camera works, a LiDAR blob counts only if the camera also sees a cone there (`confirm_with_camera`).
- If grass is mapped as cones, raise `cone_min_height_near_m` by 0.02 m. If real cones go missing at the edge of the map, lower it or reduce `isolation_m`.
- If people in yellow jackets are mapped as yellow cones, check the camera detector first; the LiDAR filters already reject anything taller than a big cone.
- The simulator's version of all this is `sensors.clutter_level: 1` (barrier runs, people, photographers, grass patches, heat shimmer). Run `./run.sh demo autocross 5 1` to watch it (the third number is the clutter level).

- **What a real LiDAR adds beyond 10 m (measured, not guessed).** On real, labelled scans from another Formula Student car (`runs/exploratory_real/`, data card `data_cards/real_lidar_conescenes.md`), the detector with shipped settings finds 98-99 % of cones up to 10 m, as in the simulator. From 15 to 20 m it finds 45 % (the simulator: 87 %), and most of its false cones are there too. So on Day 1: check `/fsai/perception/cones` at 15-20 m first; lowering `min_points` lets far, sparse cones through, and the mapping filter's confirmation (`prune_after_seen`, `prune_ratio`) removes one-off false cones. Change one value at a time and re-run `./run.sh test`.

**4. Then the events.** Run the acceleration lane at a capped `speed.max_mps`, then the skidpad, then one autocross lap at `speed.explore_mps` 5. Raise the speeds only after every log shows 0 cones hit and `grip_ratio` near 1.

**If something goes wrong.**
- **A desk run e-stops with AI_COMMS_LOST in the first second.** Another fsai-open container is still running and shares the virtual CAN bus. Run `docker ps`, stop it, and try again; `./run.sh demo` checks for this itself.
- **E-stop with no obvious reason.** Check `/fsai/status`: `faults` and `shutdown_cause` are decoded, unlike in the 2021 library.
- **The car drifts off its line, or the map turns, only in real time.** Read the `health:` lines. Scan ages near 0.4 s, drops, or control gaps over 50 ms mean the computer is overloaded; check the maths-thread setting above and close other programs.
- **AI_COMMS_LOST.** Look at `loop.max_period_ms` in `/fsai/status`. Anything near 50 ms means the computer is overloaded.
- **The car will not leave AS_READY.** Go must be switched off and on again *after* the 5 s, the wheels must be steered less than 5°, and all requests must be zero. The bridge handles the last one.
