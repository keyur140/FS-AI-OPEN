# fsai-open: results of this build

**Author:** KEYUR SHAILESHKUMAR PATEL · ORCID https://orcid.org/0009-0008-5261-043X  
MSc Artificial Intelligence student, Department of Computer Science, Brunel University London, UK; Independent researcher  
*Conducted independently; not an official Brunel University London project.*  
*AI assistance:* Code and documents were drafted with an AI assistant (Claude, by Anthropic). Every design decision was made and approved by the author; every result comes from deterministic code and tests that anyone can re-run.

Every number on this page is read from the files in `runs/`. Re-create it with `python run_all.py`.

## In plain words

- **Overall: everything passed.**
- 194 automatic checks passed, 0 failed, 0 skipped.
- 8 of 8 pretend test days ended exactly as the rules say they must, and each one gave the identical fingerprint when run a second time.
- On this computer the 10 ms loop never took longer than 20.496 ms; the rules allow 50 ms before the car stops itself.
- The ROS 2 bridge demo in Docker finished cleanly: AS_FINISHED with no faults, 86.7 m travelled.
- **Driving software:** 15 of 15 pretend competition runs (acceleration, skidpad, autocross, 10-lap trackdrive on generated tracks) were clean as the FS-AI 2026 rules judge them: right number of laps, stopped inside the allowed distance, AS Finished, no cone touched, never off track, no rule broken, no phantom cones in the map. Re-runs gave identical fingerprints: yes.
- **Inspection missions:** 4 of 4 runs of Static Inspection A, Static Inspection B and the Autonomous Demo did every step the rules list (IN2.2.2, IN2.3.2).
- **ROS 2 in Docker, real time:** 11 of 11 driverless desk runs finished cleanly with every part as a separate process.
- What this does **not** show: anything about the real car. All of this is software against a software copy of the car's safety rules. The real check is the team's first powered, wheels-off session (see docs/TEAM_SETUP_GUIDE.md).

## Evidence level

**Synthetic-validated** (desk tests against a specification emulator). Not yet run on an ADS-DV.

## Scenarios

| Scenario | Expected | Observed | Pass | Re-run identical | Fingerprint |
|---|---|---|---|---|---|
| acceleration | AS_FINISHED, no faults | AS_FINISHED, no faults | yes | yes | `2cbaa0cb509ec622…` |
| skidpad | AS_FINISHED, no faults | AS_FINISHED, no faults | yes | yes | `fed8fec373b5df37…` |
| trackdrive_slalom | AS_FINISHED, no faults | AS_FINISHED, no faults | yes | yes | `4f8ba21ee5eaeea1…` |
| autonomous_demo | AS_FINISHED, no faults | AS_FINISHED, no faults | yes | yes | `24addbc6f719bc68…` |
| stale_team_software | AS_FINISHED, no faults | AS_FINISHED, no faults | yes | yes | `024f3fbf195b2821…` |
| ai_stall_150ms | AS_EMERGENCY_BRAKE, cause AI_COMMS_FAULT | AS_EMERGENCY_BRAKE, cause AI_COMMS_FAULT | yes | yes | `77eaa77b1dec3fb5…` |
| res_estop | AS_EMERGENCY_BRAKE, stopped | AS_EMERGENCY_BRAKE, stopped | yes | yes | `2b69dbaaf0538d88…` |
| team_estop | AS_EMERGENCY_BRAKE, cause AI_COMPUTER_REQUEST | AS_EMERGENCY_BRAKE, cause AI_COMPUTER_REQUEST | yes | yes | `99c78cf1324d4d1e…` |

## Inspection missions (FS-AI 2026 rules IN2.2.2 and IN2.3.2)

| Mission | Seed | End state | Checks | Pass |
|---|---|---|---|---|
| inspection_a | 0 | AS_FINISHED | ended as finished: True; peak rpm: 196.3; peak within 8pct: True; ramp s: 9.22; reached target rpm in ramp time: True; steering sweep full range then straight: True; stop s: 1.07; stopped within stop time: True | yes |
| inspection_b | 0 | AS_EMERGENCY_BRAKE | ebs requested by ai: True; ended as emergency: True; peak rpm: 48.1; reached target rpm: True | yes |
| demo | 0 | AS_EMERGENCY_BRAKE | braking m: 3.65; ebs requested by ai: True; ended as emergency: True; first leg at least 10m: True; legs m: [15.31, 12.22]; second leg at least 10m: True; stayed within cones: True; steering sweep full range then straight: True; stopped within 10m: True | yes |
| demo | 1 | AS_EMERGENCY_BRAKE | braking m: 3.65; ebs requested by ai: True; ended as emergency: True; first leg at least 10m: True; legs m: [15.31, 12.22]; second leg at least 10m: True; stayed within cones: True; steering sweep full range then straight: True; stopped within 10m: True | yes |

## Driverless events (stage 2, judged by the FS-AI 2026 rules D4-D9)

| Event | Seed | Track [m] | Laps | Lap times [s] | Official time [s] | Stop after line [m] (allowed) | Cones hit | Off track [s] | Max speed [m/s] | Map cones (false) | Colour acc. | Pass | Re-run identical |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| acceleration | 0 | 179.0 | 1 | 8.41 | 8.41 | 19.05 (100.0) | 0 | 0.0 | 14.0 | 62 (0) | 1.0 | yes | yes |
| acceleration | 1 | 179.0 | 1 | 8.41 | 8.41 | 19.21 (100.0) | 0 | 0.0 | 14.03 | 64 (0) | 1.0 | yes | - |
| acceleration | 2 | 179.0 | 1 | 8.41 | 8.41 | 19.21 (100.0) | 0 | 0.0 | 14.03 | 61 (0) | 1.0 | yes | - |
| skidpad | 0 | 273.5 | 4 | 9.04, 9.05, 9.02, 9.06 | 9.055 | 15.57 (25.0) | 0 | 0.0 | 6.43 | 78 (0) | 1.0 | yes | yes |
| skidpad | 1 | 273.5 | 4 | 9.05, 9.05, 9.02, 9.06 | 9.055 | 15.57 (25.0) | 0 | 0.0 | 6.43 | 78 (0) | 1.0 | yes | - |
| skidpad | 2 | 273.5 | 4 | 9.04, 9.06, 9.01, 9.06 | 9.06 | 15.58 (25.0) | 0 | 0.0 | 6.43 | 78 (0) | 1.0 | yes | - |
| autocross | 0 | 483.2 | 1 | 58.84 | 58.84 | 12.25 (30.0) | 0 | 0.0 | 9.78 | 290 (0) | 1.0 | yes | yes |
| autocross | 1 | 406.7 | 1 | 49.90 | 49.9 | 12.5 (30.0) | 0 | 0.0 | 9.92 | 251 (0) | 1.0 | yes | - |
| autocross | 2 | 447.7 | 1 | 55.87 | 55.87 | 11.5 (30.0) | 0 | 0.0 | 9.95 | 282 (0) | 1.0 | yes | - |
| autocross | 3 | 309.5 | 1 | 39.05 | 39.05 | 10.25 (30.0) | 0 | 0.0 | 9.57 | 186 (0) | 1.0 | yes | - |
| autocross | 4 | 442.8 | 1 | 55.10 | 55.1 | 12.0 (30.0) | 0 | 0.0 | 9.97 | 267 (0) | 1.0 | yes | - |
| autocross | 5 | 271.4 | 1 | 35.90 | 35.9 | 11.75 (30.0) | 0 | 0.0 | 9.65 | 171 (0) | 1.0 | yes | - |
| trackdrive | 1 | 406.7 | 10 | 54.01, then 35.36-35.80 | 372.78 | 15.0 (30.0) | 0 | 0.0 | 16.59 | 251 (0) | 1.0 | yes | - |
| trackdrive | 3 | 309.5 | 10 | 41.64, then 29.16-29.36 | 304.34 | 10.25 (30.0) | 0 | 0.0 | 15.3 | 186 (0) | 1.0 | yes | yes |
| trackdrive | 5 | 271.4 | 10 | 37.30, then 28.37-28.72 | 293.07 | 15.25 (30.0) | 0 | 0.0 | 12.62 | 171 (0) | 1.0 | yes | - |

### Stress (autocross, seeds 0-2): bad sensors and sim-vs-asphalt effects. Where does it break?

| Condition | Changed settings | Clean runs | Official times [s] | Cones hit | Phantom map cones | Why not clean |
|---|---|---|---|---|---|---|
| actuator_delay_x3 | brake_delay_s=0.12, drive_delay_s=0.09, steer_delay_s=0.12 | 3/3 | [59.88, 49.32, 55.76] | [0, 0, 0] | [0, 0, 0] | - |
| baseline | - | 3/3 | [58.84, 49.9, 55.87] | [0, 0, 0] | [0, 0, 0] | - |
| bumpy_grip_15pct | tyre_mu_variation=0.15 | 3/3 | [58.44, 49.33, 56.08] | [0, 0, 0] | [0, 0, 0] | - |
| camera_blind | camera_false_per_frame=0.0, camera_range_m=0.0 | 3/3 | [56.84, 47.63, 54.76] | [0, 0, 0] | [0, 0, 0] | - |
| camera_colour_errors_x3 | camera_colour_error_far=0.36, camera_colour_error_near=0.03 | 3/3 | [59.13, 49.51, 55.88] | [0, 0, 0] | [0, 0, 0] | - |
| camera_phantoms_x5 | camera_false_per_frame=1.5 | 3/3 | [58.64, 49.54, 55.72] | [0, 0, 0] | [0, 0, 0] | - |
| cold_tyres_grip_0p8 | tyre_mu=0.96 | 3/3 | [58.83, 49.7, 56.05] | [0, 0, 0] | [0, 0, 0] | - |
| delay_x3_no_compensation | latency_comp_s=0.0, brake_delay_s=0.12, drive_delay_s=0.09, steer_delay_s=0.12 | 3/3 | [58.16, 48.64, 55.32] | [0, 0, 0] | [0, 0, 0] | - |
| gyro_bias_1dps | imu_yaw_rate_bias_dps=1.0 | 3/3 | [59.3, 49.09, 55.96] | [0, 0, 0] | [0, 0, 0] | - |
| lidar_dropout_30pct | lidar_dropout=0.3 | 3/3 | [60.74, 51.2, 57.42] | [0, 0, 0] | [0, 0, 0] | - |
| lidar_noise_x3 | lidar_noise_m=0.06 | 3/3 | [58.73, 49.24, 56.02] | [0, 0, 0] | [0, 0, 0] | - |
| real_world_mix | clutter_level=1.0, brake_delay_s=0.08, drive_delay_s=0.06, steer_delay_s=0.08, steer_gain=0.9, tyre_mu=1.08, tyre_mu_variation=0.1 | 2/3 | [58.79, 49.51, 55.75] | [0, 0, 0] | [0, 0, 1] | 1 phantom cones in the map |
| steer_gain_0p85 | steer_gain=0.85 | 3/3 | [58.93, 49.69, 55.73] | [0, 0, 0] | [0, 0, 0] | - |
| trackside_clutter | clutter_level=1.0 | 1/3 | [58.64, 49.66, 55.95] | [0, 0, 0] | [1, 0, 1] | 1 phantom cones in the map |
| wheel_speed_noise_x5 | wheel_speed_noise_frac=0.05 | 3/3 | [58.63, 49.4, 56.37] | [0, 0, 0] | [0, 0, 0] | - |

Replays: open `runs/events/replay_*.html` in any browser (one per event, seed shown).

A stress run is "not clean" for any one fault, including a single phantom cone left in the map (a piece of trackside clutter that looked like a cone often enough). Cones hit and time off track are the columns that matter for driving.

## ROS 2 in Docker, real time (every part a separate process, sim CAN bus)

| Run | End state | Laps | Lap times [s] | Cones hit | Off track [s] | Max off line [m] | Clean |
|---|---|---|---|---|---|---|---|
| acceleration 0 | AS_FINISHED | 1 | 8.40 | 0 | 0.0 | 0.253 | yes |
| autocross 1 clutter | AS_FINISHED | 1 | 53.02 | 0 | 0.0 | 0.286 | yes |
| autocross 4 | AS_FINISHED | 1 | 57.45 | 0 | 0.0 | 0.286 | yes |
| autocross 5 | AS_FINISHED | 1 | 36.44 | 0 | 0.0 | 0.395 | yes |
| autocross 5 clutter | AS_FINISHED | 1 | 36.56 | 0 | 0.0 | 0.4 | yes |
| demo 0 | AS_EMERGENCY_BRAKE | 0 | - | 0 | 0.0 | 0.251 | yes |
| inspection a 0 | AS_FINISHED | 0 | - | 0 | 0.0 | 0.0 | yes |
| inspection b 0 | AS_EMERGENCY_BRAKE | 0 | - | 0 | 0.0 | 0.0 | yes |
| skidpad 0 | AS_FINISHED | 4 | 9.08, 9.06, 9.02, 9.08 | 0 | 0.0 | 0.334 | yes |
| trackdrive 1 | AS_FINISHED | 10 | 55.82, then 35.41-36.20 | 0 | 0.0 | 0.457 | yes |
| trackdrive 3 | AS_FINISHED | 10 | 42.00, then 29.22-29.84 | 0 | 0.0 | 0.454 | yes |

Inspection B and the Autonomous Demo end in AS Emergency on purpose: the rules ask for an EBS test at the end of each (IN2.3.2).

## Exploratory run on real LiDAR scans (stage 3)

Our LiDAR cone detector, shipped settings, no tuning, on 63 labelled frames of one real Formula Student scene (coneScenes (Chalmers Formula Student), `cfs_vargarda8.zip`, SHA-256 `650ff7ecaa31a692…`), and the same measure on fsai-open's own simulated scans. Front view (+-80 deg). Data card: `data_cards/real_lidar_conescenes.md`.

| Range [m] | Real: labelled | Real: found | Real: false | Real: error [m] | Sim: labelled | Sim: found | Sim: false | Sim: error [m] |
|---|---|---|---|---|---|---|---|---|
| 0-5 | 213 | 209 (0.981) | 2 | 0.067 | 169 | 165 (0.976) | 0 | 0.014 |
| 5-10 | 282 | 280 (0.993) | 1 | 0.066 | 185 | 184 (0.995) | 0 | 0.016 |
| 10-15 | 386 | 330 (0.855) | 8 | 0.086 | 180 | 170 (0.944) | 0 | 0.018 |
| 15-20 | 509 | 228 (0.448) | 46 | 0.106 | 172 | 150 (0.872) | 0 | 0.02 |
| 20-30 | 566 | 48 (0.085) | 52 | 0.125 | 354 | 235 (0.664) | 0 | 0.018 |

Up to 20 m: real recall 0.753, precision 0.948, position error 0.081 m; simulated recall 0.948, precision 1.0, error 0.017 m. Exploratory: one scene, one sensor.

## Tests by group

| Group | Passed | Failed | Skipped |
|---|---|---|---|
| `autonomy.test_closed_loop` | 19 | 0 | 0 |
| `autonomy.test_perception` | 8 | 0 | 0 |
| `autonomy.test_planning_control` | 13 | 0 | 0 |
| `autonomy.test_slam` | 5 | 0 | 0 |
| `autonomy.test_track` | 57 | 0 | 0 |
| `conformance.test_pitfalls` | 30 | 0 | 0 |
| `conformance.test_spec_v4` | 48 | 0 | 0 |
| `realtime.test_realtime` | 2 | 0 | 0 |
| `unit.test_codec_vs_dbc` | 3 | 0 | 0 |
| `unit.test_known_answers` | 9 | 0 | 0 |

## Where each number comes from

| Number | File |
|---|---|
| Test counts | `runs/tests/summary.json`, `runs/tests/junit.xml` |
| Scenarios and fingerprints | `runs/scenarios/results.json` |
| Loop timing | `runs/timing/results.json` |
| ROS 2 Docker bridge demo | `runs/ros2_demo/result.json` |
| Driverless events, stress runs, replays | `runs/events/results.json`, `runs/events/stress.json`, `runs/events/replay_*.html` |
| ROS 2 Docker driverless runs | `runs/ros2_autonomy/result_*.json` |
| Exploratory real-data run | `runs/exploratory_real/results.json` |
| Every file's SHA-256 | `SHA256SUMS.txt`, root in `MERKLE_ROOT.txt` |
