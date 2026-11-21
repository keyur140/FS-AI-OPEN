# fsai-open

**Complete, open, tested driverless software for the IMechE FS-AI ADS-DV car: the safety link to the car, a full driving stack for every FS-AI 2026 event and inspection, and a desk copy of the car and track to test it all without the car.**

**Author:** KEYUR SHAILESHKUMAR PATEL · ORCID [0009-0008-5261-043X](https://orcid.org/0009-0008-5261-043X)
(1) MSc Artificial Intelligence student, Department of Computer Science, Brunel University London, UK · (2) Independent researcher
*Conducted independently; not an official Brunel University London project.*

> **AI assistance, stated openly.** The code and documents were drafted with an AI assistant (Claude, by Anthropic). Every design decision was made and approved by the author. Every result comes from ordinary code and tests that anyone can re-run and check.

Version 0.2.0 (safety layer + driving stack) · Licence: MIT · Evidence level: **Synthetic-validated** (desk tests only; not yet run on an ADS-DV)

---

## In plain words (for everyone)

**What is the problem?** In FS-AI, university teams write the software that drives a shared driverless race car, the IMechE ADS-DV. The car only accepts commands in one strict format over a cable called a CAN bus. It also has a safety computer, the VCU, that slams on the emergency brakes if the team's software is even a tenth of a second late or sends something it doesn't allow. Every team has to build this link themselves. The official helper library dates from 2021, no longer matches the 2026 car, and has known traps. For example, it silently sets steering to zero if you call things in the wrong order.

**What is this?** fsai-open is that link, built carefully and tested hard, plus the driving software on top of it. It finds the cones with the LiDAR and camera, builds a map while it drives, plans a path and a racing line, steers and controls speed, and runs each FS-AI 2026 event (Acceleration, Skidpad, Autocross, Trackdrive) and inspection (Static Inspection A and B, Autonomous Demo) by the written rules. It also contains a **pretend car and track**: a copy of the car's safety computer, simulated sensors and a referee, so a team can test everything at a desk.

**How do I try it?** Install Docker, then run two commands: `./install.sh` and `./run.sh demo`. You will watch the virtual car arm, wait the required 5 seconds and drive an autocross lap on its own, then you can open a replay in your browser. On Windows, use `.\run.ps1 install` then `.\run.ps1 demo`.

**How do we know it works?**
- **Automatic checks** for every rule in the car's official interface document, every known trap in the old library, and every part of the driving stack. The full count is in [reports/RESULTS.md](reports/RESULTS.md).
- **Every event, on several seeded tracks**, judged by a referee that sits outside the driving software and counts cones hit, time off track, lap times and the FS-AI 2026 penalties and stop distances.
- **Stress runs for the real world**: cold tyres, grip patches and bumps, a car that turns less than asked, three times the actuator delay, and a trackside full of barriers, people, grass and heat shimmer.
- **Real-time runs** where the safety computer, the bridge, the sensors and the driving stack are separate programs talking over a virtual CAN bus, just like on the car.
- **Fingerprinted results.** Every run is fingerprinted (SHA-256). Running it again gives the identical fingerprint.

**What does it not show?** It says nothing about the real car yet. Everything here is software tested against a software copy of the car, its sensors and the track. The team's first powered, wheels-off session and first track day are the real check, and [the setup guide](docs/TEAM_SETUP_GUIDE.md) lists exactly what to confirm and tune.

<details><summary><b>Word list</b></summary>

- **ADS-DV**: the shared driverless race car IMechE provides to FS-AI teams.
- **VCU**: the car's own safety and control computer. Teams cannot change it.
- **CAN bus**: the cable network the car's computers talk over. Messages are short, numbered "frames".
- **Bridge**: this project's translator between the team's software and the CAN bus.
- **Emulator**: a software copy of the VCU's behaviour, for desk testing.
- **Handshake**: a bit that the VCU flips and the team's computer must echo back within 50 ms (FS-AI 2026 rules T1.1.8; the older spec v4.0 said 100 ms, we use the stricter one), or the car e-stops.
- **E-stop / EBS**: emergency stop and the emergency braking system.
- **ROS 2**: the robotics software framework most teams use. The bridge plugs into it.
- **Docker**: packages the whole set-up into one box, so it runs the same on any computer.
- **SHA-256 fingerprint**: a code computed from a file. Change one letter and it changes completely.
- **SLAM**: building a map of the cones and working out where the car is on it, at the same time.
- **Racing line**: the fastest path round the track, chosen once the map is complete.
- **Seed**: a number that fixes a randomly generated track, so the same seed always gives the same track.
- **Synthetic-validated**: shown to work against a software copy, not yet against the real thing.
</details>

---

## Quick start

```bash
./install.sh                   # once: builds the pinned Docker image (ROS 2 Humble + everything)
./run.sh demo                  # a whole driverless autocross lap on the desk, in real time, with a browser replay
./run.sh demo skidpad 0        # any event and seed: acceleration | skidpad | autocross | trackdrive
./run.sh demo inspection_a     # inspections: inspection_a | inspection_b | demo
./run.sh demo autocross 5 1    # with trackside clutter (barriers, people, grass, heat shimmer)
./run.sh test                  # every test, event and stress run -> local_runs/reports/RESULTS.md (your own record)
./run.sh verify                # check every SHA-256 fingerprint and re-run every scenario byte for byte
./run.sh car can0              # the REAL car: bridge + driving stack (read the setup guide first)
./run.sh car can0 bridge       # the REAL car: the bridge alone, for your own driving software
```

Without Docker (Python 3.10+): `pip install -r requirements.txt && python run_all.py`.

## What is inside

| Part | What it does | Where |
|---|---|---|
| CAN interface table | Every AI↔VCU signal, written as data. Checked bit by bit against both official CAN databases (2021 and 2026), plus 10,000 random frames cross-decoded by an independent decoder | `fsai/can/` |
| Bridge | Handshake echo, mission status, direction, live steering/torque limits, never torque+brake together, never NEUTRAL/FINISHED while moving, controlled braking if team software goes quiet, NaN/∞ rejection, reserved-ID guard. Every change it makes is reported, never silent | `fsai/bridge/core.py` |
| VCU emulator | AS_OFF → READY → DRIVING → FINISHED / EMERGENCY_BRAKE, exactly per spec v4.0 §2.4, §3, §4, with every rule cited. Simple vehicle physics so wheel speeds respond | `fsai/vcu/emulator.py` |
| Real-time runner | 10 ms loop on a monotonic clock with fixed deadlines and jitter statistics. Runs on its own thread, so a busy ROS executor cannot starve the handshake | `fsai/bridge/runner.py` |
| ROS 2 nodes | `can_bridge` (EUFS-style topics: `ackermann_msgs` in, state/status/wheel speeds/diagnostics out, Trigger services for mission-finished and e-stop), `vcu_emulator`, `demo_driver`, `autonomy` (the driving stack), `sim_world` (pretend track, sensors and referee) | `ros2/src/fsai_ros/` |
| Track generator | Seeded tracks for every event, placed exactly by the FS-AI 2026 rules (D4 to D8: skidpad 18.25 m centres and 16/13 cones, staging distances, straights ≤ 80 m, hairpins ≥ 9 m) and checked by `validate()` | `fsai/autonomy/track.py` |
| Sensors and world | LiDAR (ray-sampled cone shapes), camera detections with colour mistakes, IMU, trackside clutter, heat shimmer, tyre-slip vehicle model with grip patches and actuator dead times, and an outside referee with the D9 penalties | `fsai/autonomy/sensors.py`, `sim.py` |
| Perception | Ground fit, cone clustering, height/width/grass/isolation filters, camera-LiDAR fusion with camera confirmation | `fsai/autonomy/perception.py` |
| SLAM | EKF with gyro bias and learned rear tyre slip, gated data association, map pruning | `fsai/autonomy/slam.py` |
| Planning and control | Local path from cone pairs, closed-loop centreline and racing line, grip-scaled speed profile, pure-pursuit steering with delay compensation and learned steering compliance | `fsai/autonomy/planning.py`, `control.py` |
| Missions and safety | Every event and inspection, rule-based stopping, sensor-loss watchdog that asks for the EBS (T4.2.2), grip monitor | `fsai/autonomy/stack.py` |
| Tests | Spec clauses, known pitfalls, known-answer bytes, determinism, "test the tests", real-time buses, perception, SLAM, planning, control, whole closed-loop events | `tests/` |
| Parameters | Every number about the car and the tracks, each with its source; estimates are marked. Every driving-stack setting, with a comment | `config/ads_dv.yaml`, `config/tracks.yaml`, `config/autonomy.yaml` |
| Provenance | `run_all.py`, `verify.py`, `SHA256SUMS.txt`, `MERKLE_ROOT.txt`, `runs/*/manifest.json` | root |

## Sim vs asphalt: what a real track adds, and what the software does about it

A simulator is always cleaner than a real track. Three things always happen on the real car, so each one has a model in the simulator, a defence in the driving software, a stress run that measures the margin, and a knob for the first test day ([guide section 6](docs/TEAM_SETUP_GUIDE.md)).

| Real-world problem | In the simulator | In the driving software | On-site knob |
|---|---|---|---|
| **Messy sensors**: the LiDAR sees grass, barriers, people and heat shimmer; the camera can call a person in a yellow jacket a cone | `sensors.clutter_level: 1` adds barrier runs, standing people, crouching photographers, grass patches and random shimmer returns | Rejects blobs taller or wider than a big cone, low blobs close by (grass), blobs with raised points around them (grass, barriers, people). While the camera works, a LiDAR blob counts only if the camera also sees a cone there. The map drops cones that are not seen again | `perception.*` |
| **Real tyre grip**: cold tyres, track temperature, bumps; the car turns less than the steering asks | Tyre-slip model with grip, grip patches over the track (`tyre_mu_variation`) and steering compliance (`steer_gain`) | Learns three things while driving: the rear tyre slip (a state in the mapping filter, so cold tyres do not smear the map), the steering compliance (asks for more steering if gentle corners come out wide), and the grip (slows the racing line down when hard corners come out wide). One `grip_factor` knob scales every speed limit | `speed.grip_factor`, `vehicle.tyre_mu` |
| **Hardware latency**: brakes and steering take a moment to move after the command | Dead time on steering, drive torque and brakes, on top of the steering rate limit and brake pressure build-up | Steers for where the car will be when the actuator acts (`latency_comp_s`), so the car does not weave | `vehicle.*_delay_s`, `control.latency_comp_s` |

None of this replaces a test day. It means the first test day is spent setting a few measured numbers, not rewriting code.

## Results of this build

See [reports/RESULTS.md](reports/RESULTS.md) and the six colour PDFs in [reports/pdf/](reports/pdf/) (Results, Project Brief, Introduction, Exploratory run, Impact, Technical Report; re-create them with `pip install reportlab matplotlib && python tools/make_pdfs.py`). Every number there is read from `runs/`. In short, for this build:

- 194 automatic checks passed, 0 failed. 8 of 8 safety scenarios ended as the rules require, with identical fingerprints when re-run.
- 15 of 15 simulated competition runs (acceleration, skidpad, autocross, 10-lap trackdrive) and 4 of 4 inspection runs were clean as the FS-AI 2026 rules judge them. Re-runs were byte-identical.
- 11 of 11 real-time runs in Docker, with every part as a separate ROS 2 process, were clean (for example trackdrive seed 3: 10 laps, 29.2-29.8 s after the first, 0 cones hit).
- Stress runs show where it breaks: trackside clutter sometimes leaves one phantom cone in the map (1 of 3 and 2 of 3 runs clean in the two clutter rows), with no cone hit.
- On real LiDAR scans (exploratory, one scene), the cone detector finds 75 % of cones up to 20 m against 95 % in the simulator; the gap is beyond 10 m. See `runs/exploratory_real/`.

## What we extend, and what it could mean (impact, in easy words)

- **For Brunel's team:** you skip months of building and debugging the car link and a driving stack, and start from a version that already drives every event and inspection on the desk and is tested against every written rule. You can rehearse every emergency stop at a desk, so your limited, booked test days on the real car go to driving rather than debugging.
- **For every FS-AI team:** every team that uses the IMechE car needs this layer, and today each one rebuilds it on their own. An open, tested version with its tests included can become a shared starting point. That could matter most for smaller universities, which the IMechE car exists to help.
- **For the organisers:** the tests turn their interface document into something you can run. They also point out where the documents disagree with each other, for example the steering limit (21°, 24°, 27.2° or 30° depending on the document) and the 2026 file marking on/off flags as signed. These are listed so they can be fixed at the source.
- **For research and teaching:** this is a deterministic, fingerprinted test bench. Anyone can change one part (say, the cone detector) and measure the effect on whole events, with the referee and stress runs unchanged.
- **Honest limits:** nothing here has touched a real ADS-DV. The safety computer's timings are from the 2021 specification, and the tyre, delay and sensor numbers are estimates; all must be confirmed on the car (see [docs/ASSUMPTIONS.md](docs/ASSUMPTIONS.md)). No one outside the project has reviewed or replicated it yet.

## Roadmap

1. **Stage 1 (0.1.0):** safety layer: bridge, emulator, tests, ROS 2, Docker.
2. **Stage 2 (0.2.0, this version):** driving stack, seeded FS-AI 2026 tracks, simulated sensors, referee, every event and inspection, sim-vs-asphalt defences, real-time ROS 2 runs, browser replays.
3. **Stage 3 (in 0.2.0):** a small exploratory run of the perception on real recorded data (coneScenes, Chalmers Formula Student), plus the Results, Project Brief, Introduction, Exploratory run and Impact PDFs.
4. **On the car (the team):** the checks in [setup guide](docs/TEAM_SETUP_GUIDE.md) sections 5 and 6.

## Sources and credits

- ADS-DV Software Interface Specification v4.0 (Hypermotive, 2021) and `ADS-DV_2026_CAN1.dbc` from [FS-AI/FS-AI_ADS-DV_Documentation](https://github.com/FS-AI/FS-AI_ADS-DV_Documentation) @ `bb8f162`. These carry no open licence, so they are **not redistributed**. `tools/fetch_dbc.py` downloads the pinned file and checks its SHA-256.
- `third_party/dbc/ADSDV_2021_VCU_AI_interface_v2.dbc` and the logic ported in `fsai/reference/` come from [FS-AI/FS-AI_API](https://github.com/FS-AI/FS-AI_API) @ `33bcb61`, MIT © 2021 IMechE Formula Student AI.
- `ros2/src/third_party/ackermann_msgs` 2.0.2, BSD, vendored unmodified.
- The provenance code is adapted from the author's own Vidya Open project.

## How to cite

> PATEL, Keyur Shaileshkumar. *fsai-open: open, tested driverless software for the IMechE FS-AI ADS-DV* (version 0.2.0). Independent research, 2026. ORCID: https://orcid.org/0009-0008-5261-043X

Machine-readable: [CITATION.cff](CITATION.cff).

No email addresses are published in this project.
